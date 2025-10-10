/*
@Author: Lzww
@LastEditTime: 2025-10-9 22:15:00
@Description: BIC (Binary Increase Congestion control) Algorithm
@Language: C++17
*/

#ifndef BIC_H
#define BIC_H

#include "../utils/cong.h"

#include <string>
#include <chrono>

class BIC: public CongestionControl {
public:
    BIC();
    BIC(const BIC& other);
    ~BIC() override;
    
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
    // BIC specific methods
    virtual uint32_t SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    // BIC-specific window update
    virtual void BicUpdate(std::unique_ptr<SocketState>& socket);

    // Reset BIC state
    virtual void BicReset();

private:
    // Standard TCP parameters
    uint32_t m_ssthresh;           // Slow start threshold
    uint32_t m_cwnd;               // Current congestion window
    uint32_t m_maxCwnd;            // Maximum congestion window
    
    // BIC-specific parameters
    uint32_t m_lastMaxCwnd;        // Window size before last reduction
    uint32_t m_lastCwnd;           // Last window size
    uint32_t m_minWin;             // Minimum window after reduction
    uint32_t m_maxIncr;            // Maximum increment (Smax)
    uint32_t m_minIncr;            // Minimum increment (Smin)
    
    // BIC control parameters
    double m_beta;                 // Multiplicative decrease factor (typically 0.8 or 0.125)
    uint32_t m_lowWindow;          // Low window threshold for binary search
    uint32_t m_smoothPart;         // Smooth increase parameter
    
    // State tracking
    bool m_foundNewMax;            // Whether we found a new max window
    uint32_t m_ackCount;           // Count of ACKs in current RTT
    std::chrono::steady_clock::time_point m_epochStart;  // Start of current epoch
};

#endif // BIC_H

