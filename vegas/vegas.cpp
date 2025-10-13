/*
@Author: Lzww
@LastEditTime: 2025-10-13 20:57:35
@Description: Vegas (Delay-based TCP) Congestion Control Algorithm Implementation
@Language: C++17
*/

#include "vegas.h"
#include <algorithm>
#include <cstdint>
#include <cmath>

// Default constructor
Vegas::Vegas() 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::VEGAS), "Vegas"),
      m_ssthresh(0x7fffffff),      // Initially very large
      m_cwnd(0),                    // Will be set based on MSS
      m_maxCwnd(65535),             // Default max window
      m_phase(VegasPhase::SLOW_START), // Start in slow start
      m_baseRTT(0xFFFFFFFF),        // Maximum initial value
      m_currentRTT(0),              // No current RTT yet
      m_alpha(DEFAULT_ALPHA),       // Alpha = 2 segments
      m_beta(DEFAULT_BETA),         // Beta = 4 segments
      m_gamma(DEFAULT_GAMMA),       // Gamma = 1 segment
      m_cntRtt(0),                  // RTT counter
      m_minRtt(0xFFFFFFFF),         // Maximum initial value
      m_doingVegasNow(false),       // Not using Vegas yet
      m_begSndNxt(0),               // Begin sequence
      m_endSndNxt(0),               // End sequence
      m_segmentsSent(0),            // No segments sent
      m_segmentsAcked(0),           // No segments acked
      m_ssCount(0),                 // Slow start counter
      m_enableSlowStart(true)       // Enable Vegas slow start
{
    InitializeVegas();
}

// Copy constructor
Vegas::Vegas(const Vegas& other) 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::VEGAS), "Vegas"),
      m_ssthresh(other.m_ssthresh),
      m_cwnd(other.m_cwnd),
      m_maxCwnd(other.m_maxCwnd),
      m_phase(other.m_phase),
      m_rttSamples(other.m_rttSamples),
      m_baseRTT(other.m_baseRTT),
      m_currentRTT(other.m_currentRTT),
      m_baseRTTTimestamp(other.m_baseRTTTimestamp),
      m_alpha(other.m_alpha),
      m_beta(other.m_beta),
      m_gamma(other.m_gamma),
      m_cntRtt(other.m_cntRtt),
      m_minRtt(other.m_minRtt),
      m_doingVegasNow(other.m_doingVegasNow),
      m_begSndNxt(other.m_begSndNxt),
      m_endSndNxt(other.m_endSndNxt),
      m_segmentsSent(other.m_segmentsSent),
      m_segmentsAcked(other.m_segmentsAcked),
      m_begTime(other.m_begTime),
      m_ssCount(other.m_ssCount),
      m_enableSlowStart(other.m_enableSlowStart)
{
}

// Destructor
Vegas::~Vegas() {
    // No dynamic memory to clean up
}

// Get type ID
TypeId Vegas::GetTypeId() {
    return static_cast<TypeId>(CongestionAlgorithm::VEGAS);
}

// Get algorithm name
std::string Vegas::GetAlgorithmName() {
    return "Vegas";
}

// Get slow start threshold
uint32_t Vegas::GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) {
    if (socket == nullptr) {
        return m_ssthresh;
    }

    // Vegas: reduce to half (similar to Reno)
    m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
    
    socket->ssthresh_ = m_ssthresh;
    return m_ssthresh;
}

// Increase congestion window based on Vegas algorithm
void Vegas::IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return;
    }

    // Update local state
    m_cwnd = socket->cwnd_;
    m_ssthresh = socket->ssthresh_;
    m_segmentsAcked += segmentsAcked;

    // Determine which phase we're in
    if (socket->tcp_state_ == TCPState::Recovery) {
        // Fast recovery
        m_cwnd = FastRecovery(socket, segmentsAcked);
        m_phase = VegasPhase::RECOVERY;
    } else if (m_cwnd < m_ssthresh) {
        // Slow start phase
        m_cwnd = SlowStart(socket, segmentsAcked);
        m_phase = VegasPhase::SLOW_START;
    } else {
        // Congestion avoidance phase - use Vegas algorithm
        m_cwnd = CongestionAvoidance(socket, segmentsAcked);
        m_phase = VegasPhase::CONGESTION_AVOIDANCE;
    }

    // Ensure we don't exceed maximum window
    m_cwnd = std::min(m_cwnd, m_maxCwnd);
    m_cwnd = std::max(m_cwnd, 2 * socket->mss_bytes_);  // Minimum 2 MSS
    
    socket->cwnd_ = m_cwnd;
}

// Handle ACKed packets - core Vegas logic
void Vegas::PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) {
    if (socket == nullptr) {
        return;
    }

    // Update RTT information
    socket->rtt_us_ = static_cast<uint32_t>(rtt);
    m_currentRTT = static_cast<uint32_t>(rtt);
    
    // Basic RTT variance calculation
    if (socket->rtt_var_ == 0) {
        socket->rtt_var_ = rtt / 2;
    } else {
        socket->rtt_var_ = (3 * socket->rtt_var_ + rtt) / 4;
    }
    socket->rto_us_ = socket->rtt_us_ + 4 * socket->rtt_var_;

    // Update base RTT
    UpdateBaseRTT(static_cast<uint32_t>(rtt));
    
    // Track minimum RTT for this period
    if (rtt < m_minRtt) {
        m_minRtt = static_cast<uint32_t>(rtt);
    }
    
    // Increment RTT counter
    m_cntRtt++;
    
    // Enable Vegas if we have base RTT
    if (!m_doingVegasNow && m_baseRTT != 0xFFFFFFFF) {
        EnableVegas();
    }
}

// Set congestion state
void Vegas::CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) {
    if (socket == nullptr) {
        return;
    }

    socket->tcp_state_ = congestionState;

    // When entering recovery or loss, adjust parameters
    if (congestionState == TCPState::Recovery || congestionState == TCPState::Loss) {
        GetSsThresh(socket, 0);
        DisableVegas();  // Disable Vegas during recovery
    }
}

// Handle congestion window events
void Vegas::CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) {
    if (socket == nullptr) {
        return;
    }

    socket->congestion_event_ = congestionEvent;

    switch (congestionEvent) {
        case CongestionEvent::PacketLoss:
            // Packet loss: fall back to Reno behavior
            m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
            socket->ssthresh_ = m_ssthresh;
            m_cwnd = m_ssthresh;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::Recovery;
            DisableVegas();
            break;

        case CongestionEvent::Timeout:
            // Timeout: reset cwnd to initial window
            m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
            socket->ssthresh_ = m_ssthresh;
            m_cwnd = socket->mss_bytes_;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::Loss;
            ResetVegasState();
            break;

        case CongestionEvent::ECN:
            // ECN: treat similar to packet loss
            m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
            m_cwnd = m_ssthresh;
            socket->ssthresh_ = m_ssthresh;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::CWR;
            break;

        case CongestionEvent::FastRecovery:
            socket->tcp_state_ = TCPState::Recovery;
            DisableVegas();
            break;

        default:
            break;
    }
}

// Check if congestion control is enabled
bool Vegas::HasCongControl() const {
    return true;
}

// Main congestion control logic
void Vegas::CongControl(std::unique_ptr<SocketState>& socket, 
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

// Slow start: exponential growth with Vegas check
uint32_t Vegas::SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return m_cwnd;
    }

    // Check if should exit slow start using Vegas logic
    if (m_enableSlowStart && m_doingVegasNow && ShouldExitSlowStart()) {
        // Exit slow start early
        m_ssthresh = m_cwnd;
        socket->ssthresh_ = m_ssthresh;
        return m_cwnd;
    }

    // Standard exponential growth
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    // Don't exceed ssthresh in slow start
    if (newCwnd > m_ssthresh) {
        newCwnd = m_ssthresh;
    }

    return std::min(newCwnd, m_maxCwnd);
}

// Congestion avoidance: Vegas algorithm
uint32_t Vegas::CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return m_cwnd;
    }

    if (!m_doingVegasNow) {
        // Vegas not ready, fall back to Reno
        uint32_t mss = socket->mss_bytes_;
        uint32_t increment = (segmentsAcked * mss * mss) / m_cwnd;
        
        if (increment == 0 && segmentsAcked > 0) {
            increment = 1;
        }

        uint32_t newCwnd = m_cwnd + increment;
        return std::min(newCwnd, m_maxCwnd);
    }

    // Vegas algorithm: adjust based on queue delay
    VegasUpdate(socket);
    
    return m_cwnd;
}

// Fast recovery: maintain cwnd
uint32_t Vegas::FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr) {
        return m_cwnd;
    }

    // Inflate window for each additional duplicate ACK
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    return std::min(newCwnd, m_maxCwnd);
}

// Vegas core update logic
void Vegas::VegasUpdate(std::unique_ptr<SocketState>& socket) {
    if (socket == nullptr || !m_doingVegasNow) {
        return;
    }

    // Calculate the difference between expected and actual throughput
    int32_t diff = CalculateDiff();
    
    // Vegas decision logic
    uint32_t mss = socket->mss_bytes_;
    
    if (diff < static_cast<int32_t>(m_alpha)) {
        // Increase cwnd (network underutilized)
        m_cwnd += mss;
    } else if (diff > static_cast<int32_t>(m_beta)) {
        // Decrease cwnd (network congested)
        if (m_cwnd > 2 * mss) {
            m_cwnd -= mss;
        }
    }
    // else: diff is between alpha and beta, keep cwnd unchanged
    
    // Reset for next RTT measurement
    m_minRtt = 0xFFFFFFFF;
}

// Update base RTT
void Vegas::UpdateBaseRTT(uint32_t rtt) {
    if (rtt == 0) {
        return;
    }
    
    // Add new sample
    m_rttSamples.push_back(VegasRTTSample(rtt));
    
    // Keep only recent samples
    while (m_rttSamples.size() > RTT_SAMPLE_WINDOW) {
        m_rttSamples.pop_front();
    }
    
    // Update base RTT if this is smaller
    if (rtt < m_baseRTT) {
        m_baseRTT = rtt;
        m_baseRTTTimestamp = std::chrono::steady_clock::now();
    }
    
    // Check if base RTT is stale
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - m_baseRTTTimestamp);
    
    if (elapsed.count() >= BASE_RTT_WINDOW_SEC) {
        // Reset base RTT from current samples
        if (!m_rttSamples.empty()) {
            uint32_t minRTT = 0xFFFFFFFF;
            for (const auto& sample : m_rttSamples) {
                minRTT = std::min(minRTT, sample.rtt_us);
            }
            m_baseRTT = minRTT;
            m_baseRTTTimestamp = now;
        }
    }
}

// Get base RTT
uint32_t Vegas::GetBaseRTT() const {
    return m_baseRTT != 0xFFFFFFFF ? m_baseRTT : 10000;  // Default 10ms if unknown
}

// Calculate difference between expected and actual throughput
int32_t Vegas::CalculateDiff() {
    if (m_baseRTT == 0xFFFFFFFF || m_baseRTT == 0) {
        return 0;  // Can't calculate without base RTT
    }
    
    if (m_currentRTT == 0) {
        return 0;  // Can't calculate without current RTT
    }
    
    // Expected throughput (in segments): cwnd / baseRTT
    // Actual throughput (in segments): cwnd / currentRTT
    // Diff = Expected - Actual = cwnd * (1/baseRTT - 1/currentRTT)
    //      = cwnd * (currentRTT - baseRTT) / (baseRTT * currentRTT)
    
    // Simplified: diff ≈ cwnd * (currentRTT - baseRTT) / baseRTT
    int32_t rttDiff = m_currentRTT - m_baseRTT;
    
    // Convert to segments (cwnd is in bytes, we want segments)
    // Assuming MSS = 1460 bytes
    uint32_t cwndSegments = m_cwnd / 1460;
    
    // diff = cwndSegments * rttDiff / baseRTT
    int32_t diff = (cwndSegments * rttDiff) / m_baseRTT;
    
    return diff;
}

// Get expected rate
double Vegas::GetExpectedRate() {
    if (m_baseRTT == 0) {
        return 0.0;
    }
    
    // Expected rate = cwnd / baseRTT (bytes per microsecond)
    return static_cast<double>(m_cwnd) / static_cast<double>(m_baseRTT);
}

// Get actual rate
double Vegas::GetActualRate() {
    if (m_currentRTT == 0) {
        return 0.0;
    }
    
    // Actual rate = cwnd / currentRTT (bytes per microsecond)
    return static_cast<double>(m_cwnd) / static_cast<double>(m_currentRTT);
}

// Check if should exit slow start
bool Vegas::ShouldExitSlowStart() {
    if (!m_doingVegasNow) {
        return false;
    }
    
    // Exit slow start if diff > gamma
    int32_t diff = CalculateDiff();
    return diff > static_cast<int32_t>(m_gamma);
}

// Reset Vegas state
void Vegas::ResetVegasState() {
    m_doingVegasNow = false;
    m_cntRtt = 0;
    m_minRtt = 0xFFFFFFFF;
    m_segmentsSent = 0;
    m_segmentsAcked = 0;
    m_begSndNxt = 0;
    m_endSndNxt = 0;
    m_ssCount = 0;
}

// Clean up old RTT samples
void Vegas::CleanupOldRTTSamples() {
    auto now = std::chrono::steady_clock::now();
    
    // Remove samples older than 10 seconds
    while (!m_rttSamples.empty()) {
        auto age = std::chrono::duration_cast<std::chrono::seconds>(
            now - m_rttSamples.front().timestamp);
        if (age.count() > 10) {
            m_rttSamples.pop_front();
        } else {
            break;
        }
    }
}

// Initialize Vegas
void Vegas::InitializeVegas() {
    m_baseRTTTimestamp = std::chrono::steady_clock::now();
    m_begTime = std::chrono::steady_clock::now();
    ResetVegasState();
}

// Enable Vegas
void Vegas::EnableVegas() {
    m_doingVegasNow = true;
    m_begTime = std::chrono::steady_clock::now();
    m_begSndNxt = 0;
    m_cntRtt = 0;
}

// Disable Vegas
void Vegas::DisableVegas() {
    m_doingVegasNow = false;
}

