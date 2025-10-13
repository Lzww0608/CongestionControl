/*
@Author: Lzww
@LastEditTime: 2025-10-13 16:54:15
@Description: DCTCP (Data Center TCP) Congestion Control Algorithm Implementation
@Language: C++17
*/

#include "dctcp.h"
#include <algorithm>
#include <cstdint>
#include <cmath>

// Default constructor
DCTCP::DCTCP() 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::DCTCP), "DCTCP"),
      m_ssthresh(0x7fffffff),      // Initially very large
      m_cwnd(0),                    // Will be set based on MSS
      m_maxCwnd(65535),             // Default max window
      m_alpha(1.0),                 // Start with max alpha (conservative)
      m_g(DEFAULT_G),               // EWMA weight = 1/16
      m_ackedBytesEcn(0),           // No ECN bytes yet
      m_ackedBytesTotal(0),         // No total bytes yet
      m_ceState(false),             // No congestion experienced
      m_delayedAckReserved(false),  // No delayed ACK
      m_priorRcvNxt(0),             // No prior sequence
      m_priorRcvNxtFlag(0),         // No flag
      m_nextSeq(0),                 // Start sequence
      m_initialized(false),         // Not initialized
      m_ecnEchoSeq(0)               // No ECN echo
{
    InitializeDCTCP();
}

// Copy constructor
DCTCP::DCTCP(const DCTCP& other) 
    : CongestionControl(static_cast<TypeId>(CongestionAlgorithm::DCTCP), "DCTCP"),
      m_ssthresh(other.m_ssthresh),
      m_cwnd(other.m_cwnd),
      m_maxCwnd(other.m_maxCwnd),
      m_alpha(other.m_alpha),
      m_g(other.m_g),
      m_ackedBytesEcn(other.m_ackedBytesEcn),
      m_ackedBytesTotal(other.m_ackedBytesTotal),
      m_ceState(other.m_ceState),
      m_delayedAckReserved(other.m_delayedAckReserved),
      m_priorRcvNxt(other.m_priorRcvNxt),
      m_priorRcvNxtFlag(other.m_priorRcvNxtFlag),
      m_nextSeq(other.m_nextSeq),
      m_initialized(other.m_initialized),
      m_ecnEchoSeq(other.m_ecnEchoSeq)
{
}

// Destructor
DCTCP::~DCTCP() {
    // No dynamic memory to clean up
}

// Get type ID
TypeId DCTCP::GetTypeId() {
    return static_cast<TypeId>(CongestionAlgorithm::DCTCP);
}

// Get algorithm name
std::string DCTCP::GetAlgorithmName() {
    return "DCTCP";
}

// Get slow start threshold
uint32_t DCTCP::GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) {
    if (socket == nullptr) {
        return m_ssthresh;
    }

    // DCTCP: ssthresh = cwnd * (1 - α/2)
    // This is more gentle than standard TCP's cwnd/2
    m_ssthresh = static_cast<uint32_t>(socket->cwnd_ * (1.0 - m_alpha / 2.0));
    
    // Ensure minimum of 2 MSS
    m_ssthresh = std::max(m_ssthresh, 2 * socket->mss_bytes_);
    
    socket->ssthresh_ = m_ssthresh;
    return m_ssthresh;
}

// Increase congestion window based on current state
void DCTCP::IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
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
    } else if (InSlowStart()) {
        // Slow start phase - use standard TCP behavior
        m_cwnd = SlowStart(socket, segmentsAcked);
    } else {
        // Congestion avoidance phase - use DCTCP behavior
        m_cwnd = CongestionAvoidance(socket, segmentsAcked);
    }

    // Ensure we don't exceed maximum window
    m_cwnd = std::min(m_cwnd, m_maxCwnd);
    socket->cwnd_ = m_cwnd;
}

// Handle ACKed packets with ECN feedback
void DCTCP::PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) {
    if (socket == nullptr) {
        return;
    }

    // Update RTT information
    socket->rtt_us_ = static_cast<uint32_t>(rtt);
    
    // Basic RTT variance calculation
    if (socket->rtt_var_ == 0) {
        socket->rtt_var_ = rtt / 2;
    } else {
        socket->rtt_var_ = (3 * socket->rtt_var_ + rtt) / 4;
    }

    // Update RTO
    socket->rto_us_ = socket->rtt_us_ + 4 * socket->rtt_var_;

    // Update ACK count for this window
    uint32_t ackedBytes = segmentsAcked * socket->mss_bytes_;
    m_ackedBytesTotal += ackedBytes;
    
    // Process ECN marking (simulated - in real implementation would come from packet)
    // Here we assume ECN info is available through some mechanism
    bool ecnMarked = false;  // Would be extracted from actual packet in real implementation
    
    if (ecnMarked) {
        m_ackedBytesEcn += ackedBytes;
    }
    
    // Check if we've completed a window and should update alpha
    // This happens approximately once per RTT
    if (m_ackedBytesTotal >= m_cwnd) {
        UpdateAlpha();
        ResetECNCounters();
    }
}

// Set congestion state
void DCTCP::CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) {
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
void DCTCP::CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) {
    if (socket == nullptr) {
        return;
    }

    socket->congestion_event_ = congestionEvent;

    switch (congestionEvent) {
        case CongestionEvent::PacketLoss:
            // Packet loss: use DCTCP reduction
            GetSsThresh(socket, 0);
            m_cwnd = m_ssthresh;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::Recovery;
            break;

        case CongestionEvent::Timeout:
            // Timeout: reset cwnd to initial window
            m_ssthresh = std::max(socket->cwnd_ / 2, 2 * socket->mss_bytes_);
            socket->ssthresh_ = m_ssthresh;
            m_cwnd = socket->mss_bytes_;
            socket->cwnd_ = m_cwnd;
            socket->tcp_state_ = TCPState::Loss;
            
            // Reset alpha to conservative value
            m_alpha = 1.0;
            ResetECNCounters();
            break;

        case CongestionEvent::ECN:
            // ECN: DCTCP's primary congestion signal
            ProcessECN(true);
            
            // Only reduce cwnd if not in slow start
            if (!InSlowStart()) {
                GetSsThresh(socket, 0);
                m_cwnd = m_ssthresh;
                socket->cwnd_ = m_cwnd;
            }
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
bool DCTCP::HasCongControl() const {
    return true;
}

// Main congestion control logic
void DCTCP::CongControl(std::unique_ptr<SocketState>& socket, 
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

// Slow start: exponential growth (standard TCP)
uint32_t DCTCP::SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
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

// Congestion avoidance: additive increase (standard TCP)
uint32_t DCTCP::CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
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

// Fast recovery: maintain cwnd
uint32_t DCTCP::FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) {
    if (socket == nullptr) {
        return m_cwnd;
    }

    // Inflate window for each additional duplicate ACK
    uint32_t newCwnd = m_cwnd + (segmentsAcked * socket->mss_bytes_);
    
    return std::min(newCwnd, m_maxCwnd);
}

// Update alpha (ECN fraction estimate)
void DCTCP::UpdateAlpha() {
    if (m_ackedBytesTotal == 0) {
        return;
    }
    
    // Calculate the fraction of bytes that were ECN-marked in this window
    double F = static_cast<double>(m_ackedBytesEcn) / static_cast<double>(m_ackedBytesTotal);
    
    // Update alpha using EWMA (Exponentially Weighted Moving Average)
    // α = (1 - g) * α + g * F
    m_alpha = (1.0 - m_g) * m_alpha + m_g * F;
    
    // Clamp alpha to valid range [0, 1]
    m_alpha = std::max(0.0, std::min(DCTCP_MAX_ALPHA, m_alpha));
}

// Process ECN feedback
void DCTCP::ProcessECN(bool ecnMarked) {
    if (!m_initialized) {
        InitializeDCTCP();
    }
    
    // Update CE state
    m_ceState = ecnMarked;
    
    // If this ACK has ECN marking, we'll account for it in the next update
}

// Calculate new cwnd based on alpha
uint32_t DCTCP::CalculateNewCwnd() {
    // DCTCP reduction: cwnd_new = cwnd * (1 - α/2)
    // This is more gentle than TCP's cwnd/2 when α is small
    double reduction_factor = 1.0 - m_alpha / 2.0;
    uint32_t newCwnd = static_cast<uint32_t>(m_cwnd * reduction_factor);
    
    // Ensure minimum window size
    newCwnd = std::max(newCwnd, 2 * 1460);  // At least 2 MSS
    
    return newCwnd;
}

// Reset ECN counters for new window
void DCTCP::ResetECNCounters() {
    m_ackedBytesEcn = 0;
    m_ackedBytesTotal = 0;
    m_priorRcvNxt = m_nextSeq;
}

// Initialize DCTCP parameters
void DCTCP::InitializeDCTCP() {
    m_alpha = 1.0;              // Start conservative
    m_ackedBytesEcn = 0;
    m_ackedBytesTotal = 0;
    m_ceState = false;
    m_initialized = true;
    m_priorRcvNxt = 0;
    m_nextSeq = 0;
}

// Check if in slow start
bool DCTCP::InSlowStart() const {
    return m_cwnd < m_ssthresh;
}

