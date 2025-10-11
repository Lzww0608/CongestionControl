/*
@Author: Lzww
@LastEditTime: 2025-10-11 20:56:57
@Description: CUBIC Congestion Control Algorithm Implementation
@Language: C++17
*/

#include "cubic.h"
#include <algorithm>
#include <cstdint>
#include <cmath>

// Default constructor
Cubic::Cubic() 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::CUBIC), "CUBIC"),
      m_ssthresh(0x7fffffff),      // Initially very large
      m_cwnd(0),                    // Will be set based on MSS
      m_maxCwnd(65535),             // Default max window
      m_lastMaxCwnd(0),             // No previous max (W_max)
      m_lastCwnd(0),                // No previous cwnd
      m_k(0.0),                     // Time to reach W_max
      m_cubicBeta(0.7),             // Beta = 0.7 (CUBIC standard)
      m_cubicC(0.4),                // C = 0.4 (CUBIC standard)
      m_fastConvergence(true),      // Fast convergence enabled
      m_tcpFriendly(true),          // TCP-friendly mode enabled
      m_tcpCwnd(0),                 // TCP Reno estimate
      m_lastTime(0.0),              // No time elapsed yet
      m_ackCount(0),                // No ACKs counted yet
      m_cntRtt(0),                  // RTT counter
      m_delayMin(0xFFFFFFFF),       // Maximum initial value
      m_hystartEnabled(true),       // Hystart enabled by default
      m_hystartAckDelta(2),         // ACK delta threshold
      m_hystartDelayMin(0xFFFFFFFF),// Minimum delay in round
      m_hystartDelayMax(0)          // Maximum delay in round
{
    m_epochStart = std::chrono::steady_clock::now();
}

// Copy constructor
Cubic::Cubic(const Cubic& other) 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::CUBIC), "CUBIC"),
      m_ssthresh(other.m_ssthresh),
      m_cwnd(other.m_cwnd),
      m_maxCwnd(other.m_maxCwnd),
      m_lastMaxCwnd(other.m_lastMaxCwnd),
      m_lastCwnd(other.m_lastCwnd),
      m_k(other.m_k),
      m_cubicBeta(other.m_cubicBeta),
      m_cubicC(other.m_cubicC),
      m_fastConvergence(other.m_fastConvergence),
      m_tcpFriendly(other.m_tcpFriendly),
      m_tcpCwnd(other.m_tcpCwnd),
      m_epochStart(other.m_epochStart),
      m_lastTime(other.m_lastTime),
      m_ackCount(other.m_ackCount),
      m_cntRtt(other.m_cntRtt),
      m_delayMin(other.m_delayMin),
      m_hystartEnabled(other.m_hystartEnabled),
      m_hystartAckDelta(other.m_hystartAckDelta),
      m_hystartDelayMin(other.m_hystartDelayMin),
      m_hystartDelayMax(other.m_hystartDelayMax)
{
}

// Destructor
Cubic::~Cubic() {
    // No dynamic memory to clean up
}

// Get type ID
TypeId Cubic::GetTypeId() {
    return static_cast<TypeId>(CongestionAlgorithm::CUBIC);
}

// Get algorithm name
std::string Cubic::GetAlgorithmName() {
    return "CUBIC";
}

// Get slow start threshold
uint32_t Cubic::GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) {
    if (socket == nullptr) {
        return m_ssthresh;
    }

    // CUBIC uses beta * cwnd as the new ssthresh
    m_lastCwnd = socket->cwnd_;
    
    // Fast convergence: if cwnd < last_max_cwnd, further reduce W_max
    if (m_fastConvergence && socket->cwnd_ < m_lastMaxCwnd) {
        m_lastMaxCwnd = static_cast<uint32_t>(socket->cwnd_ * (2.0 - m_cubicBeta) / 2.0);
    } else {
        m_lastMaxCwnd = socket->cwnd_;
    }
    
    m_ssthresh = static_cast<uint32_t>(socket->cwnd_ * m_cubicBeta);
    
    // Ensure minimum of 2 MSS
    m_ssthresh = std::max(m_ssthresh, 2 * socket->mss_bytes_);
    
    socket->ssthresh_ = m_ssthresh;
    
    // Calculate K (time to reach W_max)
    CalculateK();
    
    return m_ssthresh;
}

// Increase congestion window based on current state
void Cubic::IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
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
        // Congestion avoidance phase - use CUBIC algorithm
        m_cwnd = CongestionAvoidance(socket, segmentsAcked);
    }

    // Ensure we don't exceed maximum window
    m_cwnd = std::min(m_cwnd, m_maxCwnd);
    socket->cwnd_ = m_cwnd;
}

// Handle ACKed packets
void Cubic::PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) {
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

    // Track minimum delay
    if (rtt > 0 && rtt < m_delayMin) {
        m_delayMin = static_cast<uint32_t>(rtt);
    }

    // Hystart delay tracking
    if (m_hystartEnabled && m_cwnd < m_ssthresh) {
        if (rtt < m_hystartDelayMin) {
            m_hystartDelayMin = static_cast<uint32_t>(rtt);
        }
        if (rtt > m_hystartDelayMax) {
            m_hystartDelayMax = static_cast<uint32_t>(rtt);
        }
        
        // Check if we should exit slow start early (Hystart)
        if (m_hystartDelayMax - m_hystartDelayMin > m_hystartAckDelta) {
            // Exit slow start - set ssthresh to current cwnd
            m_ssthresh = m_cwnd;
            socket->ssthresh_ = m_ssthresh;
        }
    }

    // Count ACKs for CUBIC algorithm
    m_ackCount += segmentsAcked;
    m_cntRtt++;
}

// Set congestion state
void Cubic::CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) {
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
void Cubic::CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) {
    if (socket == nullptr) {
        return;
    }

    socket->congestion_event_ = congestionEvent;

    switch (congestionEvent) {
        case CongestionEvent::PacketLoss:
        case CongestionEvent::Timeout:
            // Reduce window using CUBIC's beta factor
            GetSsThresh(socket, 0);
            
            if (congestionEvent == CongestionEvent::Timeout) {
                // Timeout: reset cwnd to initial window
                m_cwnd = socket->mss_bytes_;
                socket->cwnd_ = m_cwnd;
                socket->tcp_state_ = TCPState::Loss;
                CubicReset();
            } else {
                // Fast retransmit: enter recovery
                m_cwnd = m_ssthresh;
                socket->cwnd_ = m_cwnd;
                socket->tcp_state_ = TCPState::Recovery;
            }
            
            // Reset epoch
            m_epochStart = std::chrono::steady_clock::now();
            m_lastTime = 0.0;
            m_ackCount = 0;
            m_tcpCwnd = 0;
            
            // Reset Hystart
            m_hystartDelayMin = 0xFFFFFFFF;
            m_hystartDelayMax = 0;
            break;

        case CongestionEvent::ECN:
            // ECN: similar to packet loss
            GetSsThresh(socket, 0);
            m_cwnd = m_ssthresh;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::CWR;
            m_epochStart = std::chrono::steady_clock::now();
            m_lastTime = 0.0;
            break;

        case CongestionEvent::FastRecovery:
            socket->tcp_state_ = TCPState::Recovery;
            break;

        default:
            break;
    }
}

// Check if congestion control is enabled
bool Cubic::HasCongControl() const {
    return true;
}

// Main congestion control logic
void Cubic::CongControl(std::unique_ptr<SocketState>& socket, 
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

// Slow start: exponential growth (same as Reno, with optional Hystart)
uint32_t Cubic::SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return m_cwnd;
    }

    // Increase cwnd by segmentsAcked * MSS (exponential growth)
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    // Don't exceed ssthresh in slow start
    if (newCwnd > m_ssthresh) {
        newCwnd = m_ssthresh;
        // Reset Hystart when exiting slow start
        m_hystartDelayMin = 0xFFFFFFFF;
        m_hystartDelayMax = 0;
    }

    return std::min(newCwnd, m_maxCwnd);
}

// Congestion avoidance: CUBIC algorithm
uint32_t Cubic::CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return m_cwnd;
    }

    // Update CUBIC state and get new window size
    CubicUpdate(socket);

    return m_cwnd;
}

// Fast recovery: maintain cwnd
uint32_t Cubic::FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr) {
        return m_cwnd;
    }

    // In recovery, maintain or slightly inflate window
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    return std::min(newCwnd, m_maxCwnd);
}

// CUBIC-specific window update algorithm
void Cubic::CubicUpdate(std::unique_ptr<SocketState>& socket) {
    if (socket == nullptr) {
        return;
    }

    m_ackCount++;
    
    // Calculate elapsed time since epoch start
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_epochStart);
    double t = elapsed.count() / 1000.0;  // Convert to seconds
    
    // Calculate target cwnd using CUBIC function
    uint32_t cubicTarget = CubicWindowCalculation(t);
    
    // TCP-friendly region calculation
    if (m_tcpFriendly) {
        // Estimate TCP Reno cwnd
        // W_tcp(t) = W_max * (1 - β) + 3β/(2 - β) * t/RTT
        if (socket->rtt_us_ > 0) {
            double rtt_sec = socket->rtt_us_ / 1000000.0;
            uint32_t mss = socket->mss_bytes_;
            
            if (m_tcpCwnd == 0) {
                m_tcpCwnd = m_cwnd;
            }
            
            // Simplified TCP Reno estimate: increase by 1 MSS per RTT
            double tcp_increment = (3.0 * m_cubicBeta / (2.0 - m_cubicBeta)) * (t / rtt_sec) * mss;
            m_tcpCwnd = static_cast<uint32_t>(m_lastMaxCwnd * (1.0 - m_cubicBeta) + tcp_increment);
            
            // Use the larger of CUBIC or TCP estimate
            if (m_tcpCwnd > cubicTarget) {
                cubicTarget = m_tcpCwnd;
            }
        }
    }
    
    // Calculate the increment
    if (cubicTarget > m_cwnd) {
        // Calculate how many segments to add
        uint32_t delta = cubicTarget - m_cwnd;
        uint32_t mss = socket->mss_bytes_;
        uint32_t cnt = m_cwnd / delta;
        
        if (cnt == 0) {
            cnt = 1;
        }
        
        // Increase cwnd
        if (m_ackCount >= cnt) {
            m_cwnd += mss;
            m_ackCount = 0;
        }
    } else {
        // We're above the target, slow increase
        if (m_ackCount >= m_cwnd / socket->mss_bytes_) {
            m_cwnd += socket->mss_bytes_;
            m_ackCount = 0;
        }
    }
    
    m_lastTime = t;
}

// Calculate CUBIC window based on time
uint32_t Cubic::CubicWindowCalculation(double t) {
    // CUBIC function: W(t) = C * (t - K)^3 + W_max
    double delta_t = t - m_k;
    double cubic_term = m_cubicC * delta_t * delta_t * delta_t;
    
    // Convert to segments (assuming MSS = 1460 bytes for calculation)
    double target = m_lastMaxCwnd + cubic_term * 1460.0;
    
    // Ensure non-negative
    if (target < 0) {
        target = 0;
    }
    
    return static_cast<uint32_t>(target);
}

// Calculate the time K (inflection point)
void Cubic::CalculateK() {
    // K = ∛(W_max * β / C)
    // For W_max in bytes, convert to segments first
    if (m_lastMaxCwnd == 0 || m_cubicC == 0) {
        m_k = 0.0;
        return;
    }
    
    double w_max_segments = m_lastMaxCwnd / 1460.0;  // Assuming MSS = 1460
    double numerator = w_max_segments * (1.0 - m_cubicBeta);
    double k_cubed = numerator / m_cubicC;
    
    if (k_cubed < 0) {
        m_k = 0.0;
    } else {
        m_k = std::cbrt(k_cubed);  // Cube root
    }
}

// Reset CUBIC state
void Cubic::CubicReset() {
    m_lastMaxCwnd = 0;
    m_lastCwnd = 0;
    m_k = 0.0;
    m_ackCount = 0;
    m_cntRtt = 0;
    m_lastTime = 0.0;
    m_tcpCwnd = 0;
    m_delayMin = 0xFFFFFFFF;
    m_hystartDelayMin = 0xFFFFFFFF;
    m_hystartDelayMax = 0;
    m_epochStart = std::chrono::steady_clock::now();
}

