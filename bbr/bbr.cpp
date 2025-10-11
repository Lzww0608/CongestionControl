/*
@Author: Lzww
@LastEditTime: 2025-10-11 23:26:45
@Description: BBR (Bottleneck Bandwidth and RTT) Congestion Control Algorithm Implementation
@Language: C++17
*/

#include "bbr.h"
#include <algorithm>
#include <cstdint>
#include <cmath>

// Define static constexpr members
constexpr uint32_t BBR::PROBE_BW_GAINS[8];

// Default constructor
BBR::BBR() 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::BBR), "BBR"),
      m_cwnd(0),                    // Will be set based on initial cwnd
      m_maxCwnd(65535),             // Default max window
      m_mode(BBRMode::STARTUP),     // Start in STARTUP mode
      m_maxBandwidth(0),            // No bandwidth observed yet
      m_bandwidthWindow(BANDWIDTH_WINDOW_SIZE),
      m_minRTT(0xFFFFFFFF),         // Maximum initial value
      m_minRTTWindow(MIN_RTT_WINDOW_SEC),
      m_pacingRate(0),              // Will be calculated
      m_pacingGain(HIGH_GAIN),      // High gain in STARTUP
      m_cwndGain(CWND_GAIN),        // Default cwnd gain
      m_prevMaxBandwidth(0),
      m_roundsWithoutGrowth(0),
      m_roundCount(0),
      m_lastRoundStartSeq(0),
      m_probeBWCycleIndex(0),
      m_probeRTTDuration(PROBE_RTT_DURATION_MS),
      m_probeRTTRoundDone(false),
      m_deliveredBytes(0),
      m_deliveredTime(0)
{
    InitializeParameters();
}

// Copy constructor
BBR::BBR(const BBR& other) 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::BBR), "BBR"),
      m_cwnd(other.m_cwnd),
      m_maxCwnd(other.m_maxCwnd),
      m_mode(other.m_mode),
      m_bandwidthSamples(other.m_bandwidthSamples),
      m_maxBandwidth(other.m_maxBandwidth),
      m_bandwidthWindow(other.m_bandwidthWindow),
      m_rttSamples(other.m_rttSamples),
      m_minRTT(other.m_minRTT),
      m_minRTTTimestamp(other.m_minRTTTimestamp),
      m_minRTTWindow(other.m_minRTTWindow),
      m_pacingRate(other.m_pacingRate),
      m_pacingGain(other.m_pacingGain),
      m_cwndGain(other.m_cwndGain),
      m_prevMaxBandwidth(other.m_prevMaxBandwidth),
      m_roundsWithoutGrowth(other.m_roundsWithoutGrowth),
      m_roundCount(other.m_roundCount),
      m_lastRoundStartSeq(other.m_lastRoundStartSeq),
      m_probeBWCycleIndex(other.m_probeBWCycleIndex),
      m_probeBWCycleStart(other.m_probeBWCycleStart),
      m_probeRTTStart(other.m_probeRTTStart),
      m_probeRTTDuration(other.m_probeRTTDuration),
      m_probeRTTRoundDone(other.m_probeRTTRoundDone),
      m_deliveredBytes(other.m_deliveredBytes),
      m_deliveredTime(other.m_deliveredTime)
{
}

// Destructor
BBR::~BBR() {
    // No dynamic memory to clean up
}

// Get type ID
TypeId BBR::GetTypeId() {
    return static_cast<TypeId>(CongestionAlgorithm::BBR);
}

// Get algorithm name
std::string BBR::GetAlgorithmName() {
    return "BBR";
}

// Get slow start threshold (BBR doesn't use ssthresh in traditional way)
uint32_t BBR::GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) {
    // BBR doesn't use ssthresh, return a large value
    return 0x7fffffff;
}

// Increase congestion window based on BBR model
void BBR::IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return;
    }

    // Update local state
    m_cwnd = socket->cwnd_;

    // BBR calculates cwnd based on BDP (Bandwidth-Delay Product)
    uint32_t targetCwnd = CalculateTargetCwnd(m_cwndGain);
    
    // In PROBE_RTT, use minimum cwnd
    if (m_mode == BBRMode::PROBE_RTT) {
        targetCwnd = std::max(4 * socket->mss_bytes_, targetCwnd / 2);
    }
    
    // Gradually move towards target
    if (m_cwnd < targetCwnd) {
        m_cwnd = std::min(m_cwnd + segmentsAcked * socket->mss_bytes_, targetCwnd);
    } else if (m_cwnd > targetCwnd) {
        m_cwnd = targetCwnd;
    }

    // Ensure we don't exceed maximum window
    m_cwnd = std::min(m_cwnd, m_maxCwnd);
    m_cwnd = std::max(m_cwnd, 4 * socket->mss_bytes_);  // Minimum 4 MSS
    
    socket->cwnd_ = m_cwnd;
}

// Handle ACKed packets - core BBR logic
void BBR::PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) {
    if (socket == nullptr || segmentsAcked == 0) {
        return;
    }

    // Update RTT information
    socket->rtt_us_ = static_cast<uint32_t>(rtt);
    
    // Calculate delivered bytes
    uint32_t ackedBytes = segmentsAcked * socket->mss_bytes_;
    m_deliveredBytes += ackedBytes;
    
    // Run BBR main update logic
    BBRUpdate(socket, ackedBytes, rtt);
}

// Set congestion state
void BBR::CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) {
    if (socket == nullptr) {
        return;
    }

    socket->tcp_state_ = congestionState;
    
    // BBR is less sensitive to traditional congestion states
    // but we still track them for compatibility
}

// Handle congestion window events
void BBR::CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) {
    if (socket == nullptr) {
        return;
    }

    socket->congestion_event_ = congestionEvent;

    // BBR doesn't react strongly to packet loss like traditional algorithms
    // It primarily uses bandwidth and RTT measurements
    switch (congestionEvent) {
        case CongestionEvent::PacketLoss:
            // BBR: packet loss is expected when probing bandwidth
            // Don't reduce cwnd aggressively
            break;

        case CongestionEvent::Timeout:
            // Timeout suggests severe congestion, reset to conservative state
            m_cwnd = 4 * socket->mss_bytes_;
            socket->cwnd_ = m_cwnd;
            EnterStartup();  // Restart from STARTUP
            break;

        case CongestionEvent::ECN:
            // ECN signal - could indicate congestion
            // BBR can optionally reduce pacing rate slightly
            break;

        default:
            break;
    }
}

// Check if congestion control is enabled
bool BBR::HasCongControl() const {
    return true;
}

// Main congestion control logic
void BBR::CongControl(std::unique_ptr<SocketState>& socket, 
                      const CongestionEvent& congestionEvent,
                      const RTTSample& rtt) {
    if (socket == nullptr) {
        return;
    }

    // Handle the congestion event
    CwndEvent(socket, congestionEvent);

    // Update with RTT if valid
    if (rtt.rtt.count() > 0) {
        PktsAcked(socket, 1, rtt.rtt.count());
    }
}

// Enter STARTUP mode
void BBR::EnterStartup() {
    m_mode = BBRMode::STARTUP;
    m_pacingGain = HIGH_GAIN;       // 2.89x
    m_cwndGain = CWND_GAIN;         // 2.0x
    m_roundsWithoutGrowth = 0;
    m_prevMaxBandwidth = 0;
}

// Enter DRAIN mode
void BBR::EnterDrain() {
    m_mode = BBRMode::DRAIN;
    m_pacingGain = 100 * 100 / HIGH_GAIN;  // 1/2.89 to drain queue
    m_cwndGain = CWND_GAIN;
}

// Enter PROBE_BW mode
void BBR::EnterProbeBW() {
    m_mode = BBRMode::PROBE_BW;
    m_pacingGain = PROBE_BW_GAIN;
    m_cwndGain = CWND_GAIN;
    m_probeBWCycleIndex = 0;
    m_probeBWCycleStart = std::chrono::steady_clock::now();
    UpdateProbeBWGain();
}

// Enter PROBE_RTT mode
void BBR::EnterProbeRTT() {
    m_mode = BBRMode::PROBE_RTT;
    m_pacingGain = PROBE_BW_GAIN;
    m_cwndGain = PROBE_RTT_CWND_GAIN;  // 0.5x to reduce queue
    m_probeRTTStart = std::chrono::steady_clock::now();
    m_probeRTTRoundDone = false;
}

// BBR main update logic
void BBR::BBRUpdate(std::unique_ptr<SocketState>& socket, uint32_t ackedBytes, uint64_t rtt) {
    // Update bandwidth estimate
    UpdateBandwidth(ackedBytes, rtt);
    
    // Update min RTT
    UpdateMinRTT(rtt);
    
    // Clean up old samples
    CleanupOldSamples();
    
    // Update pacing rate
    m_pacingRate = CalculatePacingRate(m_pacingGain);
    
    // State machine transitions
    switch (m_mode) {
        case BBRMode::STARTUP:
            // Check if we've filled the pipe
            if (IsFullPipe()) {
                EnterDrain();
            }
            break;
            
        case BBRMode::DRAIN:
            // Check if we've drained the queue
            // (inflight <= BDP)
            if (socket->cwnd_ <= CalculateTargetCwnd(100)) {
                EnterProbeBW();
            }
            break;
            
        case BBRMode::PROBE_BW:
            // Cycle through gains to probe for bandwidth
            UpdateProbeBWGain();
            
            // Check if we should probe RTT
            if (ShouldProbeRTT()) {
                EnterProbeRTT();
            }
            break;
            
        case BBRMode::PROBE_RTT:
            // Stay in PROBE_RTT for minimum duration
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - m_probeRTTStart);
            
            if (elapsed.count() >= m_probeRTTDuration) {
                // Exit PROBE_RTT
                m_minRTTTimestamp = now;
                
                if (IsFullPipe()) {
                    EnterProbeBW();
                } else {
                    EnterStartup();
                }
            }
            break;
    }
}

// Update bandwidth estimate
void BBR::UpdateBandwidth(uint32_t ackedBytes, uint64_t rtt) {
    if (rtt == 0) {
        return;
    }
    
    // Calculate bandwidth: bytes / time
    // bandwidth (bytes/sec) = ackedBytes / (rtt / 1000000)
    uint64_t bandwidth = (static_cast<uint64_t>(ackedBytes) * 1000000) / rtt;
    
    // Add new sample
    m_bandwidthSamples.push_back(BandwidthSample(bandwidth));
    
    // Keep only recent samples (windowed max)
    while (m_bandwidthSamples.size() > m_bandwidthWindow) {
        m_bandwidthSamples.pop_front();
    }
    
    // Update max bandwidth
    uint64_t newMaxBandwidth = GetMaxBandwidth();
    
    // Track bandwidth growth for STARTUP
    if (m_mode == BBRMode::STARTUP) {
        if (newMaxBandwidth < m_prevMaxBandwidth * FULL_PIPE_THRESHOLD) {
            m_roundsWithoutGrowth++;
        } else {
            m_roundsWithoutGrowth = 0;
        }
        m_prevMaxBandwidth = newMaxBandwidth;
    }
    
    m_maxBandwidth = newMaxBandwidth;
}

// Get maximum bandwidth from samples
uint64_t BBR::GetMaxBandwidth() const {
    if (m_bandwidthSamples.empty()) {
        return 0;
    }
    
    uint64_t maxBW = 0;
    for (const auto& sample : m_bandwidthSamples) {
        maxBW = std::max(maxBW, sample.bandwidth);
    }
    
    return maxBW;
}

// Update minimum RTT
void BBR::UpdateMinRTT(uint64_t rtt) {
    if (rtt == 0) {
        return;
    }
    
    uint32_t rtt_us = static_cast<uint32_t>(rtt);
    
    // Add new sample
    m_rttSamples.push_back(BBRRTTSample(rtt_us));
    
    // Update min RTT if this is smaller
    if (rtt_us < m_minRTT) {
        m_minRTT = rtt_us;
        m_minRTTTimestamp = std::chrono::steady_clock::now();
    }
}

// Get minimum RTT
uint32_t BBR::GetMinRTT() const {
    return m_minRTT != 0xFFFFFFFF ? m_minRTT : 10000;  // Default 10ms if unknown
}

// Calculate target congestion window
uint32_t BBR::CalculateTargetCwnd(uint32_t gain_percent) {
    if (m_maxBandwidth == 0 || m_minRTT == 0xFFFFFFFF) {
        // No measurements yet, use default
        return 4 * 1460;  // 4 MSS
    }
    
    // BDP = bandwidth * RTT
    // cwnd = BDP * gain
    uint64_t bdp = (m_maxBandwidth * m_minRTT) / 1000000;  // bytes
    uint64_t targetCwnd = (bdp * gain_percent) / 100;
    
    // Ensure minimum window
    targetCwnd = std::max(targetCwnd, static_cast<uint64_t>(4 * 1460));
    
    return static_cast<uint32_t>(std::min(targetCwnd, static_cast<uint64_t>(m_maxCwnd)));
}

// Calculate pacing rate
uint64_t BBR::CalculatePacingRate(uint32_t gain_percent) {
    if (m_maxBandwidth == 0) {
        // No bandwidth estimate yet
        return 1000000;  // Default 1 MB/s
    }
    
    // Pacing rate = bandwidth * gain
    uint64_t rate = (m_maxBandwidth * gain_percent) / 100;
    
    return std::max(rate, static_cast<uint64_t>(1000));  // Minimum pacing rate
}

// Check if we should enter PROBE_RTT
bool BBR::ShouldProbeRTT() {
    if (m_minRTT == 0xFFFFFFFF) {
        return false;
    }
    
    // Check if min RTT measurement is stale
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - m_minRTTTimestamp);
    
    return elapsed.count() >= m_minRTTWindow;
}

// Update gain for PROBE_BW cycling
void BBR::UpdateProbeBWGain() {
    if (m_mode != BBRMode::PROBE_BW) {
        return;
    }
    
    // Check if we should move to next gain in cycle
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - m_probeBWCycleStart);
    
    // Stay in each gain phase for approximately one RTT
    uint32_t minRTT_ms = m_minRTT / 1000;
    if (minRTT_ms == 0) minRTT_ms = 100;  // Default 100ms
    
    if (elapsed.count() >= minRTT_ms) {
        // Move to next gain in cycle
        m_probeBWCycleIndex = (m_probeBWCycleIndex + 1) % 8;
        m_pacingGain = PROBE_BW_GAINS[m_probeBWCycleIndex];
        m_probeBWCycleStart = now;
    }
}

// Check for full pipe (bandwidth plateau)
bool BBR::IsFullPipe() const {
    // Full pipe is detected when bandwidth stops growing
    return m_roundsWithoutGrowth >= FULL_PIPE_ROUNDS;
}

// Clean up old samples
void BBR::CleanupOldSamples() {
    auto now = std::chrono::steady_clock::now();
    
    // Clean up bandwidth samples older than window
    while (!m_bandwidthSamples.empty()) {
        auto age = std::chrono::duration_cast<std::chrono::seconds>(
            now - m_bandwidthSamples.front().timestamp);
        if (age.count() > 60) {  // Keep samples for up to 60 seconds
            m_bandwidthSamples.pop_front();
        } else {
            break;
        }
    }
    
    // Clean up RTT samples
    while (!m_rttSamples.empty()) {
        auto age = std::chrono::duration_cast<std::chrono::seconds>(
            now - m_rttSamples.front().timestamp);
        if (age.count() > 60) {
            m_rttSamples.pop_front();
        } else {
            break;
        }
    }
}

// Initialize parameters
void BBR::InitializeParameters() {
    m_minRTTTimestamp = std::chrono::steady_clock::now();
    m_probeBWCycleStart = std::chrono::steady_clock::now();
    m_probeRTTStart = std::chrono::steady_clock::now();
}

