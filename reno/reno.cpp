/*
@Author: Lzww
@LastEditTime: 2025-10-9 22:00:38
@Description: Reno Congestion Control Algorithm Implementation
@Language: C++17
*/

#include "reno.h"
#include <algorithm>
#include <cstdint>

// Default constructor
Reno::Reno() 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::RENO), "Reno"),
      m_ssthresh(0x7fffffff),  // Initially very large (effectively no limit)
      m_cwnd(0),                // Will be set based on MSS
      m_maxCwnd(65535)          // Default max window
{
}

// Copy constructor
Reno::Reno(const Reno& other) 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::RENO), "Reno"),
      m_ssthresh(other.m_ssthresh),
      m_cwnd(other.m_cwnd),
      m_maxCwnd(other.m_maxCwnd)
{
}

// Destructor
Reno::~Reno() {
    // No dynamic memory to clean up
}

// Get type ID
TypeId Reno::GetTypeId() {
    return static_cast<TypeId>(CongestionAlgorithm::RENO);
}

// Get algorithm name
std::string Reno::GetAlgorithmName() {
    return "Reno";
}

// Increase congestion window based on current state
void Reno::IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return;
    }

    // Update local state
    m_cwnd = socket->cwnd_;
    m_ssthresh = socket->ssthresh_;

    // Determine which phase we're in
    if (socket->tcp_state_ == TCPState::Recovery) {
        // Fast recovery - handled separately
        m_cwnd = FastRecovery(socket, segmentsAcked);
    } else if (m_cwnd < m_ssthresh) {
        // Slow start phase
        m_cwnd = SlowStart(socket, segmentsAcked);
    } else {
        // Congestion avoidance phase
        m_cwnd = CongestionAvoidance(socket, segmentsAcked);
    }

    // Ensure we don't exceed maximum window
    m_cwnd = std::min(m_cwnd, m_maxCwnd);
    socket->cwnd_ = m_cwnd;
}

// Handle ACKed packets
void Reno::PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) {
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
}

// Set congestion state
void Reno::CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) {
    if (socket == nullptr) {
        return;
    }

    socket->tcp_state_ = congestionState;

    // When entering recovery, set ssthresh
    if (congestionState == TCPState::Recovery || congestionState == TCPState::Loss) {
        m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
        socket->ssthresh_ = m_ssthresh;
    }
}

// Handle congestion window events
void Reno::CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) {
    if (socket == nullptr) {
        return;
    }

    socket->congestion_event_ = congestionEvent;

    switch (congestionEvent) {
        case CongestionEvent::PacketLoss:
        case CongestionEvent::Timeout:
            // Reduce window and enter slow start
            m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
            socket->ssthresh_ = m_ssthresh;
            
            if (congestionEvent == CongestionEvent::Timeout) {
                // Timeout: reset cwnd to initial window
                m_cwnd = socket->mss_bytes_;
                socket->cwnd_ = m_cwnd;
                socket->tcp_state_ = TCPState::Loss;
            } else {
                // Fast retransmit: enter recovery
                socket->tcp_state_ = TCPState::Recovery;
            }
            break;

        case CongestionEvent::ECN:
            // ECN: similar to packet loss but less aggressive
            m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
            m_cwnd = m_ssthresh;
            socket->ssthresh_ = m_ssthresh;
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
bool Reno::HasCongControl() const {
    return true;
}

// Main congestion control logic
void Reno::CongControl(std::unique_ptr<SocketState>& socket, 
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

// Slow start: exponential growth
uint32_t Reno::SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
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

// Congestion avoidance: linear growth
uint32_t Reno::CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr || segmentsAcked == 0) {
        return m_cwnd;
    }

    // Increase cwnd by approximately MSS per RTT
    // Formula: cwnd += MSS * MSS / cwnd (for each ACK)
    uint32_t mss = socket->mss_bytes_;
    uint32_t increment = (segmentsAcked * mss * mss) / m_cwnd;
    
    if (increment == 0 && segmentsAcked > 0) {
        increment = 1;  // Ensure at least some progress
    }

    uint32_t newCwnd = m_cwnd + increment;
    return std::min(newCwnd, m_maxCwnd);
}

// Fast retransmit (triggered by duplicate ACKs)
uint32_t Reno::FastRetransmit(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr) {
        return m_cwnd;
    }

    // Set ssthresh to half of current cwnd
    m_ssthresh = std::max(m_cwnd / 2, 2 * socket->mss_bytes_);
    socket->ssthresh_ = m_ssthresh;

    // Enter fast recovery
    socket->tcp_state_ = TCPState::Recovery;

    return m_ssthresh + 3 * socket->mss_bytes_;  // Inflate window by 3 MSS
}

// Fast recovery: maintain cwnd until loss is repaired
uint32_t Reno::FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr) {
        return m_cwnd;
    }

    // Inflate window for each additional duplicate ACK
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    return std::min(newCwnd, m_maxCwnd);
}

