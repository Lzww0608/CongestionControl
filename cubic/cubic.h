/*
@Author: Lzww
@LastEditTime: 2025-10-11 20:56:51
@Description: CUBIC Congestion Control Algorithm
@Language: C++17
*/

#ifndef CUBIC_H
#define CUBIC_H

#include "../utils/cong.h"

#include <string>
#include <chrono>

class Cubic: public CongestionControl {
public:
    Cubic();
    Cubic(const Cubic& other);
    ~Cubic() override;
    
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
    // CUBIC specific methods
    virtual uint32_t SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    // CUBIC-specific window update
    virtual void CubicUpdate(std::unique_ptr<SocketState>& socket);

    // Calculate CUBIC window based on time
    virtual uint32_t CubicWindowCalculation(double elapsedTime);

    // Reset CUBIC state
    virtual void CubicReset();

    // Calculate the time K (inflection point)
    virtual void CalculateK();

private:
    // Standard TCP parameters
    uint32_t m_ssthresh;           // Slow start threshold
    uint32_t m_cwnd;               // Current congestion window
    uint32_t m_maxCwnd;            // Maximum congestion window
    
    // CUBIC-specific parameters
    uint32_t m_lastMaxCwnd;        // Window size before last reduction (W_max)
    uint32_t m_lastCwnd;           // Last congestion window before event
    double m_k;                    // Time period for cwnd to grow to W_max (K)
    
    // CUBIC constants
    double m_cubicBeta;            // Multiplicative decrease factor (β = 0.7)
    double m_cubicC;               // CUBIC parameter (C = 0.4)
    bool m_fastConvergence;        // Fast convergence enabled
    
    // TCP-friendly mode parameters
    bool m_tcpFriendly;            // Enable TCP-friendly mode
    uint32_t m_tcpCwnd;            // Estimated TCP Reno cwnd
    
    // Time tracking
    std::chrono::steady_clock::time_point m_epochStart;  // Start of current epoch
    double m_lastTime;             // Last update time since epoch start
    
    // ACK counting
    uint32_t m_ackCount;           // Count of ACKs in current RTT
    uint32_t m_cntRtt;             // RTT counter for updates
    
    // Delay tracking
    uint32_t m_delayMin;           // Minimum delay observed (microseconds)
    
    // Hystart parameters (for slow start optimization)
    bool m_hystartEnabled;         // Hystart enabled flag
    uint32_t m_hystartAckDelta;    // ACK train threshold
    uint32_t m_hystartDelayMin;    // Minimum delay in current round
    uint32_t m_hystartDelayMax;    // Maximum delay in current round
};

#endif // CUBIC_H

