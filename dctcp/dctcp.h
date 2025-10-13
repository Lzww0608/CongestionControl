/*
@Author: Lzww
@LastEditTime: 2025-10-13 16:54:07
@Description: DCTCP (Data Center TCP) Congestion Control Algorithm
@Language: C++17
*/

#ifndef DCTCP_H
#define DCTCP_H

#include "../utils/cong.h"

#include <string>
#include <chrono>

class DCTCP: public CongestionControl {
public:
    DCTCP();
    DCTCP(const DCTCP& other);
    ~DCTCP() override;
    
    TypeId GetTypeId();

    std::string GetAlgorithmName() override;

    uint32_t GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) override;

    void IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) override;

    void PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) override;

    void CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) override;

    void CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) override;

    bool HasCongControl() const override;

    void CongControl(std::unique_ptr<SocketState>& socket, const CongestionEvent& congestionEvent, const RTTSample& rtt) override;

protected:
    // DCTCP specific methods
    virtual uint32_t SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    // DCTCP core methods
    virtual void UpdateAlpha();                     // Update ECN fraction estimate (α)
    virtual void ProcessECN(bool ecnMarked);        // Process ECN feedback
    virtual uint32_t CalculateNewCwnd();            // Calculate cwnd based on α
    virtual void ResetECNCounters();                // Reset ECN counters for new window

private:
    // Standard TCP parameters
    uint32_t m_ssthresh;           // Slow start threshold
    uint32_t m_cwnd;               // Current congestion window
    uint32_t m_maxCwnd;            // Maximum congestion window
    
    // DCTCP-specific parameters
    double m_alpha;                // ECN fraction estimate (α)
    double m_g;                    // Weight parameter for EWMA (typically 1/16)
    
    // ECN tracking
    uint32_t m_ackedBytesEcn;      // Bytes ACKed with ECN marks in current window
    uint32_t m_ackedBytesTotal;    // Total bytes ACKed in current window
    bool m_ceState;                // Current CE (Congestion Experienced) state
    bool m_delayedAckReserved;     // Delayed ACK flag
    
    // Window tracking for ECN measurement
    uint64_t m_priorRcvNxt;        // Sequence number at start of window
    uint64_t m_priorRcvNxtFlag;    // Flag to indicate start of new window
    uint32_t m_nextSeq;            // Next expected sequence number
    bool m_initialized;            // Whether DCTCP is initialized
    
    // Statistics
    uint32_t m_ecnEchoSeq;         // ECN Echo sequence tracking
    
    // Configuration constants
    static constexpr double DEFAULT_G = 0.0625;          // g = 1/16 (EWMA weight)
    static constexpr double DCTCP_MAX_ALPHA = 1.0;       // Maximum alpha value
    static constexpr uint32_t USE_DCTCP_SSTHRESH = 0x7fffffff;  // Use DCTCP in CA only
    
    // Helper methods
    void InitializeDCTCP();
    bool InSlowStart() const;
};

#endif // DCTCP_H

