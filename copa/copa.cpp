/*
@Author: Lzww
@LastEditTime: 2025-10-12 17:18:37
@Description: Copa (Delay-based Congestion control) Algorithm Implementation
@Language: C++17
*/

#include "copa.h"
#include <algorithm>
#include <cstdint>
#include <cmath>

// Default constructor
Copa::Copa() 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::RENO), "Copa"),
      m_ssthresh(0x7fffffff),      // Initially very large
      m_cwnd(0),                    // Will be set based on MSS
      m_maxCwnd(65535),             // Default max window
      m_mode(CopaMode::SLOW_START), // Start in slow start
      m_minRTT(0xFFFFFFFF),         // Maximum initial value
      m_standingRTT(0),             // No standing RTT yet
      m_delta(DEFAULT_DELTA),       // Target 0.5 RTT queueing delay
      m_velocity(0.0),              // No velocity yet
      m_targetRate(0),              // Will be calculated
      m_useCompetitiveMode(false),  // Default to non-competitive
      m_competitiveDelta(1),        // 1 packet competitive delta
      m_lastRateUpdate(0),          // No update yet
      m_deliveredBytes(0),          // No bytes delivered
      m_ssExitThreshold(SS_EXIT_THRESHOLD_US),
      m_inSlowStart(true),
      m_prevDirection(0),           // No previous direction
      m_prevQueueingDelay(0.0),     // No previous delay
      m_rttCount(0),
      m_totalAckedBytes(0)
{
    InitializeParameters();
}

// Copy constructor
Copa::Copa(const Copa& other) 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::RENO), "Copa"),
      m_ssthresh(other.m_ssthresh),
      m_cwnd(other.m_cwnd),
      m_maxCwnd(other.m_maxCwnd),
      m_mode(other.m_mode),
      m_rttSamples(other.m_rttSamples),
      m_minRTT(other.m_minRTT),
      m_standingRTT(other.m_standingRTT),
      m_minRTTTimestamp(other.m_minRTTTimestamp),
      m_delta(other.m_delta),
      m_velocity(other.m_velocity),
      m_targetRate(other.m_targetRate),
      m_useCompetitiveMode(other.m_useCompetitiveMode),
      m_competitiveDelta(other.m_competitiveDelta),
      m_lastRateUpdate(other.m_lastRateUpdate),
      m_deliveredBytes(other.m_deliveredBytes),
      m_rttStart(other.m_rttStart),
      m_ssExitThreshold(other.m_ssExitThreshold),
      m_inSlowStart(other.m_inSlowStart),
      m_prevDirection(other.m_prevDirection),
      m_prevQueueingDelay(other.m_prevQueueingDelay),
      m_rttCount(other.m_rttCount),
      m_totalAckedBytes(other.m_totalAckedBytes)
{
}

// Destructor
Copa::~Copa() {
    // No dynamic memory to clean up
}

// Get type ID
TypeId Copa::GetTypeId() {
    return static_cast<TypeId>(CongestionAlgorithm::RENO);  // Using RENO as placeholder
}

// Get algorithm name
std::string Copa::GetAlgorithmName() {
    return "Copa";
}

// Get slow start threshold
uint32_t Copa::GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) {
    if (socket == nullptr) {
        return m_ssthresh;
    }

    // Copa: reduce to current cwnd * (1 - delta/2)
    m_ssthresh = static_cast<uint32_t>(socket->cwnd_ * (1.0 - m_delta / 2.0));
    
    // Ensure minimum of 2 MSS
    m_ssthresh = std::max(m_ssthresh, 2 * socket->mss_bytes_);
    
    socket->ssthresh_ = m_ssthresh;
    return m_ssthresh;
}

// Increase congestion window based on Copa model
void Copa::IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return;
    }

    // Update local state
    m_cwnd = socket->cwnd_;
    m_ssthresh = socket->ssthresh_;

    if (m_mode == CopaMode::SLOW_START) {
        // Slow start: exponential growth
        m_cwnd += segmentsAcked * socket->mss_bytes_;
        
        // Check if should exit slow start
        if (ShouldExitSlowStart()) {
            EnterVelocityMode();
        }
    } else {
        // Velocity mode or competitive mode: adjust based on rate
        UpdateCwndFromRate(socket);
    }

    // Ensure we don't exceed maximum window
    m_cwnd = std::min(m_cwnd, m_maxCwnd);
    m_cwnd = std::max(m_cwnd, 2 * socket->mss_bytes_);  // Minimum 2 MSS
    
    socket->cwnd_ = m_cwnd;
}

// Handle ACKed packets - core Copa logic
void Copa::PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) {
    if (socket == nullptr || segmentsAcked == 0) {
        return;
    }

    // Update RTT information
    socket->rtt_us_ = static_cast<uint32_t>(rtt);
    
    // Update RTT variance
    if (socket->rtt_var_ == 0) {
        socket->rtt_var_ = rtt / 2;
    } else {
        socket->rtt_var_ = (3 * socket->rtt_var_ + rtt) / 4;
    }
    socket->rto_us_ = socket->rtt_us_ + 4 * socket->rtt_var_;
    
    // Calculate delivered bytes
    uint32_t ackedBytes = segmentsAcked * socket->mss_bytes_;
    m_deliveredBytes += ackedBytes;
    m_totalAckedBytes += ackedBytes;
    m_rttCount++;
    
    // Run Copa main update logic
    CopaUpdate(socket, ackedBytes, rtt);
}

// Set congestion state
void Copa::CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) {
    if (socket == nullptr) {
        return;
    }

    socket->tcp_state_ = congestionState;

    // When entering recovery or loss, adjust parameters
    if (congestionState == TCPState::Recovery || congestionState == TCPState::Loss) {
        GetSsThresh(socket, 0);
    }
}

// Handle congestion window events
void Copa::CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) {
    if (socket == nullptr) {
        return;
    }

    socket->congestion_event_ = congestionEvent;

    switch (congestionEvent) {
        case CongestionEvent::PacketLoss:
            // Copa: moderate response to packet loss
            m_cwnd = static_cast<uint32_t>(m_cwnd * (1.0 - m_delta / 2.0));
            m_cwnd = std::max(m_cwnd, 4 * socket->mss_bytes_);
            socket->cwnd_ = m_cwnd;
            
            // Reset velocity
            m_velocity = 0.0;
            m_prevDirection = 0;
            break;

        case CongestionEvent::Timeout:
            // Timeout: reset to conservative state
            m_cwnd = 4 * socket->mss_bytes_;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::Loss;
            EnterSlowStart();
            break;

        case CongestionEvent::ECN:
            // ECN: similar to packet loss
            m_cwnd = static_cast<uint32_t>(m_cwnd * (1.0 - m_delta / 2.0));
            m_cwnd = std::max(m_cwnd, 4 * socket->mss_bytes_);
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::CWR;
            break;

        case CongestionEvent::FastRecovery:
            socket->tcp_state_ = TCPState::Recovery;
            break;

        default:
            break;
    }
}

// Check if congestion control is enabled
bool Copa::HasCongControl() const {
    return true;
}

// Main congestion control logic
void Copa::CongControl(std::unique_ptr<SocketState>& socket, 
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

// Enter slow start mode
void Copa::EnterSlowStart() {
    m_mode = CopaMode::SLOW_START;
    m_inSlowStart = true;
    m_velocity = 0.0;
    m_prevDirection = 0;
}

// Enter competitive mode
void Copa::EnterCompetitiveMode() {
    m_mode = CopaMode::COMPETITIVE;
    m_inSlowStart = false;
    m_velocity = 0.0;
}

// Enter velocity mode
void Copa::EnterVelocityMode() {
    m_mode = CopaMode::VELOCITY;
    m_inSlowStart = false;
    m_velocity = 0.0;
    m_prevQueueingDelay = GetQueueingDelay();
}

// Copa main update logic
void Copa::CopaUpdate(std::unique_ptr<SocketState>& socket, uint32_t ackedBytes, uint64_t rtt) {
    // Update RTT measurements
    UpdateRTT(rtt);
    
    // Clean up old samples
    CleanupOldRTTSamples();
    
    // Check for mode transitions
    CheckModeTransition();
    
    if (m_mode == CopaMode::VELOCITY || m_mode == CopaMode::COMPETITIVE) {
        // Calculate velocity based on queueing delay
        m_velocity = CalculateVelocity();
        
        // Calculate target rate
        m_targetRate = CalculateTargetRate();
    }
}

// Update RTT measurements
void Copa::UpdateRTT(uint64_t rtt) {
    if (rtt == 0) {
        return;
    }
    
    uint32_t rtt_us = static_cast<uint32_t>(rtt);
    
    // Add new sample
    m_rttSamples.push_back(CopaRTTMeasurement(rtt_us));
    
    // Keep only recent samples
    while (m_rttSamples.size() > RTT_SAMPLE_WINDOW) {
        m_rttSamples.pop_front();
    }
    
    // Update min RTT if this is smaller
    if (rtt_us < m_minRTT) {
        m_minRTT = rtt_us;
        m_minRTTTimestamp = std::chrono::steady_clock::now();
    }
    
    // Update standing RTT (recent average)
    if (!m_rttSamples.empty()) {
        uint64_t sum = 0;
        for (const auto& sample : m_rttSamples) {
            sum += sample.rtt_us;
        }
        m_standingRTT = static_cast<uint32_t>(sum / m_rttSamples.size());
    }
}

// Get minimum RTT
uint32_t Copa::GetMinRTT() const {
    return m_minRTT != 0xFFFFFFFF ? m_minRTT : 10000;  // Default 10ms if unknown
}

// Get standing queue delay
uint32_t Copa::GetStandingQueueDelay() const {
    if (m_minRTT == 0xFFFFFFFF || m_standingRTT == 0) {
        return 0;
    }
    
    // Queue delay = standing RTT - min RTT
    return m_standingRTT > m_minRTT ? (m_standingRTT - m_minRTT) : 0;
}

// Get queueing delay in units of RTTs
double Copa::GetQueueingDelay() const {
    uint32_t queueDelay = GetStandingQueueDelay();
    uint32_t minRTT = GetMinRTT();
    
    if (minRTT == 0) {
        return 0.0;
    }
    
    // Return as fraction of RTT
    return static_cast<double>(queueDelay) / static_cast<double>(minRTT);
}

// Calculate velocity for rate adjustment
double Copa::CalculateVelocity() {
    double currentQueueingDelay = GetQueueingDelay();
    
    // Calculate the direction of change
    int32_t direction = 0;
    if (currentQueueingDelay < m_delta) {
        direction = 1;   // Increase rate (less than target delay)
    } else if (currentQueueingDelay > m_delta) {
        direction = -1;  // Decrease rate (more than target delay)
    }
    
    // Velocity update based on Copa algorithm
    // v(t) = v(t-1) + δ * direction_change
    double velocityUpdate = 0.0;
    
    if (direction != m_prevDirection && m_prevDirection != 0) {
        // Direction changed, adjust velocity
        velocityUpdate = m_delta * direction;
    } else if (direction != 0) {
        // Same direction, continue with small adjustment
        velocityUpdate = m_delta * direction * 0.5;
    }
    
    double newVelocity = m_velocity + velocityUpdate * VELOCITY_GAIN;
    
    // Clamp velocity to reasonable bounds
    newVelocity = std::max(-1.0, std::min(1.0, newVelocity));
    
    m_prevDirection = direction;
    m_prevQueueingDelay = currentQueueingDelay;
    
    return newVelocity;
}

// Calculate target sending rate
uint32_t Copa::CalculateTargetRate() {
    if (m_minRTT == 0xFFFFFFFF || m_minRTT == 0) {
        return m_cwnd * 1000;  // Default rate
    }
    
    // Calculate current rate: cwnd / RTT
    // Rate in bytes per second
    double currentRate = static_cast<double>(m_cwnd) * 1000000.0 / static_cast<double>(m_minRTT);
    
    // Apply velocity: rate(t+1) = rate(t) * (1 + v(t) * δ)
    double rateMultiplier = 1.0 + m_velocity * m_delta;
    double targetRate = currentRate * rateMultiplier;
    
    // Ensure minimum rate
    targetRate = std::max(targetRate, 1000.0);  // At least 1 KB/s
    
    return static_cast<uint32_t>(targetRate);
}

// Update cwnd from target rate
void Copa::UpdateCwndFromRate(std::unique_ptr<SocketState>& socket) {
    if (socket == nullptr) {
        return;
    }
    
    if (m_targetRate == 0 || m_minRTT == 0xFFFFFFFF) {
        return;
    }
    
    // cwnd = rate * RTT
    // Convert: rate (bytes/sec) * RTT (microsec) / 1,000,000
    uint64_t newCwnd = (static_cast<uint64_t>(m_targetRate) * m_minRTT) / 1000000;
    
    // Smooth transition
    if (newCwnd > m_cwnd) {
        m_cwnd = std::min(static_cast<uint32_t>(newCwnd), m_cwnd + socket->mss_bytes_);
    } else if (newCwnd < m_cwnd) {
        m_cwnd = std::max(static_cast<uint32_t>(newCwnd), m_cwnd - socket->mss_bytes_);
    }
}

// Check if should exit slow start
bool Copa::ShouldExitSlowStart() {
    if (!m_inSlowStart) {
        return false;
    }
    
    // Exit if queueing delay exceeds threshold
    uint32_t queueDelay = GetStandingQueueDelay();
    return queueDelay > m_ssExitThreshold;
}

// Check for mode transitions
void Copa::CheckModeTransition() {
    // Check if min RTT measurement is stale
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - m_minRTTTimestamp);
    
    if (elapsed.count() >= MIN_RTT_WINDOW_SEC) {
        // Min RTT is stale, need to re-measure
        // Could transition to a probe mode here
    }
    
    // Transition from slow start to velocity mode
    if (m_mode == CopaMode::SLOW_START && ShouldExitSlowStart()) {
        EnterVelocityMode();
    }
}

// Clean up old RTT samples
void Copa::CleanupOldRTTSamples() {
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

// Initialize parameters
void Copa::InitializeParameters() {
    m_minRTTTimestamp = std::chrono::steady_clock::now();
    m_rttStart = std::chrono::steady_clock::now();
}

