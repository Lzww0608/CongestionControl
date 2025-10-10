/*
@Author: Lzww
@LastEditTime: 2025-10-9 22:15:00
@Description: BIC (Binary Increase Congestion control) Algorithm Implementation
@Language: C++17
*/

#include "bic.h"
#include <algorithm>
#include <cstdint>
#include <cmath>

// Default constructor
BIC::BIC() 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::BIC), "BIC"),
      m_ssthresh(0x7fffffff),      // Initially very large
      m_cwnd(0),                    // Will be set based on MSS
      m_maxCwnd(65535),             // Default max window
      m_lastMaxCwnd(0),             // No previous max
      m_lastCwnd(0),                // No previous cwnd
      m_minWin(0),                  // Will be set on first reduction
      m_maxIncr(32),                // Smax = 32 segments (default)
      m_minIncr(1),                 // Smin = 1 segment
      m_beta(0.8),                  // Beta = 0.8 (less aggressive than 0.125)
      m_lowWindow(14),              // Low window threshold
      m_smoothPart(0),              // Smooth increase counter
      m_foundNewMax(false),         // Haven't found new max yet
      m_ackCount(0)                 // No ACKs counted yet
{
    m_epochStart = std::chrono::steady_clock::now();
}

// Copy constructor
BIC::BIC(const BIC& other) 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::BIC), "BIC"),
      m_ssthresh(other.m_ssthresh),
      m_cwnd(other.m_cwnd),
      m_maxCwnd(other.m_maxCwnd),
      m_lastMaxCwnd(other.m_lastMaxCwnd),
      m_lastCwnd(other.m_lastCwnd),
      m_minWin(other.m_minWin),
      m_maxIncr(other.m_maxIncr),
      m_minIncr(other.m_minIncr),
      m_beta(other.m_beta),
      m_lowWindow(other.m_lowWindow),
      m_smoothPart(other.m_smoothPart),
      m_foundNewMax(other.m_foundNewMax),
      m_ackCount(other.m_ackCount),
      m_epochStart(other.m_epochStart)
{
}

// Destructor
BIC::~BIC() {
    // No dynamic memory to clean up
}

// Get type ID
TypeId BIC::GetTypeId() {
    return static_cast<TypeId>(CongestionAlgorithm::BIC);
}

// Get algorithm name
std::string BIC::GetAlgorithmName() {
    return "BIC";
}

// Get slow start threshold
uint32_t BIC::GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) {
    if (socket == nullptr) {
        return m_ssthresh;
    }

    // BIC uses beta * cwnd as the new ssthresh
    m_lastMaxCwnd = socket->cwnd_;
    m_ssthresh = static_cast<uint32_t>(socket->cwnd_ * m_beta);
    
    // Ensure minimum of 2 MSS
    m_ssthresh = std::max(m_ssthresh, 2 * socket->mss_bytes_);
    
    socket->ssthresh_ = m_ssthresh;
    return m_ssthresh;
}

// Increase congestion window based on current state
void BIC::IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return;
    }

    // Update local state
    m_cwnd = socket->cwnd_;
    m_ssthresh = socket->ssthresh_;

    // Determine which phase we're in
    if (socket->tcp_state_ == TCPState::Recovery) {
        // Fast recovery
        m_cwnd = FastRecovery(socket, segmentsAcked);
    } else if (m_cwnd < m_ssthresh) {
        // Slow start phase
        m_cwnd = SlowStart(socket, segmentsAcked);
    } else {
        // Congestion avoidance phase - use BIC algorithm
        m_cwnd = CongestionAvoidance(socket, segmentsAcked);
    }

    // Ensure we don't exceed maximum window
    m_cwnd = std::min(m_cwnd, m_maxCwnd);
    socket->cwnd_ = m_cwnd;
}

// Handle ACKed packets
void BIC::PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) {
    if (socket == nullptr) {
        return;
    }

    // Update RTT information
    socket->rtt_us_ = static_cast<uint32_t>(rtt);
    
    // Basic RTT variance calculation (simplified)
    if (socket->rtt_var_ == 0) {
        socket->rtt_var_ = rtt / 2;
    } else {
        socket->rtt_var_ = (3 * socket->rtt_var_ + rtt) / 4;
    }

    // Update RTO (simplified: RTO = RTT + 4 * RTT_VAR)
    socket->rto_us_ = socket->rtt_us_ + 4 * socket->rtt_var_;

    // Count ACKs for BIC algorithm
    m_ackCount += segmentsAcked;
}

// Set congestion state
void BIC::CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) {
    if (socket == nullptr) {
        return;
    }

    socket->tcp_state_ = congestionState;

    // When entering recovery or loss, adjust parameters
    if (congestionState == TCPState::Recovery || congestionState == TCPState::Loss) {
        GetSsThresh(socket, 0);
        m_minWin = m_ssthresh;
        m_foundNewMax = false;
    }
}

// Handle congestion window events
void BIC::CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) {
    if (socket == nullptr) {
        return;
    }

    socket->congestion_event_ = congestionEvent;

    switch (congestionEvent) {
        case CongestionEvent::PacketLoss:
        case CongestionEvent::Timeout:
            // Save the last max cwnd
            if (socket->cwnd_ > m_lastMaxCwnd) {
                m_lastMaxCwnd = socket->cwnd_;
            }

            // Reduce window using BIC's beta factor
            GetSsThresh(socket, 0);
            m_minWin = m_ssthresh;
            m_foundNewMax = false;
            
            if (congestionEvent == CongestionEvent::Timeout) {
                // Timeout: reset cwnd to initial window
                m_cwnd = socket->mss_bytes_;
                socket->cwnd_ = m_cwnd;
                socket->tcp_state_ = TCPState::Loss;
                BicReset();
            } else {
                // Fast retransmit: enter recovery
                m_cwnd = m_ssthresh;
                socket->cwnd_ = m_cwnd;
                socket->tcp_state_ = TCPState::Recovery;
            }
            
            m_epochStart = std::chrono::steady_clock::now();
            m_ackCount = 0;
            break;

        case CongestionEvent::ECN:
            // ECN: similar to packet loss
            GetSsThresh(socket, 0);
            m_cwnd = m_ssthresh;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::CWR;
            m_minWin = m_ssthresh;
            m_foundNewMax = false;
            break;

        case CongestionEvent::FastRecovery:
            socket->tcp_state_ = TCPState::Recovery;
            break;

        default:
            break;
    }
}

// Check if congestion control is enabled
bool BIC::HasCongControl() const {
    return true;
}

// Main congestion control logic
void BIC::CongControl(std::unique_ptr<SocketState>& socket, 
                      const CongestionEvent& congestionEvent,
                      const RTTSample& rtt) {
    if (socket == nullptr) {
        return;
    }

    // Handle the congestion event
    CwndEvent(socket, congestionEvent);

    // Update RTT if valid
    if (rtt.rtt.count() > 0) {
        PktsAcked(socket, 1, rtt.rtt.count());
    }
}

// Slow start: exponential growth (same as Reno)
uint32_t BIC::SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return m_cwnd;
    }

    // Increase cwnd by segmentsAcked * MSS (exponential growth)
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    // Don't exceed ssthresh in slow start
    if (newCwnd > m_ssthresh) {
        newCwnd = m_ssthresh;
    }

    return std::min(newCwnd, m_maxCwnd);
}

// Congestion avoidance: BIC algorithm
uint32_t BIC::CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return m_cwnd;
    }

    // Update BIC state and get new window size
    BicUpdate(socket);

    return m_cwnd;
}

// Fast recovery: maintain cwnd
uint32_t BIC::FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr) {
        return m_cwnd;
    }

    // In recovery, maintain or slightly inflate window
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    return std::min(newCwnd, m_maxCwnd);
}

// BIC-specific window update algorithm
void BIC::BicUpdate(std::unique_ptr<SocketState>& socket) {
    if (socket == nullptr) {
        return;
    }

    uint32_t mss = socket->mss_bytes_;
    m_ackCount++;

    // Calculate the target window size
    uint32_t targetWin = m_lastMaxCwnd;
    
    // If we haven't found a new max, use a large target
    if (!m_foundNewMax || m_lastMaxCwnd == 0) {
        targetWin = m_cwnd + m_maxIncr * mss;
    }

    // Calculate the distance to the target
    int32_t dist = (targetWin - m_cwnd) / mss;

    if (dist > static_cast<int32_t>(m_maxIncr)) {
        // We're far from target: additive increase with Smax
        m_cwnd += m_maxIncr * mss;
    } else if (dist > 0) {
        // Binary search increase
        // Use binary search to find the optimal window
        uint32_t increment;
        
        if (dist > static_cast<int32_t>(m_minIncr)) {
            // Binary search phase
            increment = (dist / 2) * mss;
            if (increment < m_minIncr * mss) {
                increment = m_minIncr * mss;
            }
        } else {
            // Linear increase near target
            increment = m_minIncr * mss;
        }
        
        m_cwnd += increment;
    } else {
        // We've reached or passed the target
        if (!m_foundNewMax) {
            // First time reaching the target
            m_foundNewMax = true;
            m_lastMaxCwnd = m_cwnd;
        }
        
        // Slow increase beyond previous max
        if (m_cwnd < m_lastMaxCwnd + m_maxIncr * mss) {
            m_cwnd += m_minIncr * mss;
        } else {
            m_cwnd += m_maxIncr * mss;
            m_lastMaxCwnd = m_cwnd;
        }
    }

    // Ensure minimum window size
    if (m_cwnd < m_minWin) {
        m_cwnd = m_minWin;
    }
}

// Reset BIC state
void BIC::BicReset() {
    m_lastMaxCwnd = 0;
    m_lastCwnd = 0;
    m_minWin = 0;
    m_foundNewMax = false;
    m_ackCount = 0;
    m_smoothPart = 0;
    m_epochStart = std::chrono::steady_clock::now();
}

