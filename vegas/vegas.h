/*
@Author: Lzww
@LastEditTime: 2025-10-13 20:57:31
@Description: Vegas (Delay-based TCP) Congestion Control Algorithm
@Language: C++17
*/

#ifndef VEGAS_H
#define VEGAS_H

#include "../utils/cong.h"

#include <string>
#include <chrono>
#include <deque>

// Vegas operating phases
enum class VegasPhase {
    SLOW_START,         // Slow start phase
    CONGESTION_AVOIDANCE, // Congestion avoidance phase
    RECOVERY            // Recovery phase
};

// RTT measurement for Vegas
struct VegasRTTSample {
    uint32_t rtt_us;                                      // RTT in microseconds
    std::chrono::steady_clock::time_point timestamp;      // When measured
    
    VegasRTTSample(uint32_t rtt = 0) 
        : rtt_us(rtt), timestamp(std::chrono::steady_clock::now()) {}
};

class Vegas: public CongestionControl {
public:
    Vegas();
    Vegas(const Vegas& other);
    ~Vegas() override;
    
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
    // Vegas specific methods
    virtual uint32_t SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    // Vegas core algorithm
    virtual void VegasUpdate(std::unique_ptr<SocketState>& socket);
    
    // RTT tracking
    virtual void UpdateBaseRTT(uint32_t rtt);
    virtual uint32_t GetBaseRTT() const;
    
    // Vegas calculation
    virtual int32_t CalculateDiff();  // Calculate (Expected - Actual)
    virtual double GetExpectedRate();  // Expected throughput
    virtual double GetActualRate();    // Actual throughput
    
    // State management
    virtual bool ShouldExitSlowStart();
    virtual void ResetVegasState();

private:
    // Standard TCP parameters
    uint32_t m_ssthresh;           // Slow start threshold
    uint32_t m_cwnd;               // Current congestion window
    uint32_t m_maxCwnd;            // Maximum congestion window
    
    // Vegas phase
    VegasPhase m_phase;            // Current Vegas phase
    
    // RTT measurements
    std::deque<VegasRTTSample> m_rttSamples;  // Recent RTT samples
    uint32_t m_baseRTT;            // Minimum RTT observed (base RTT, microseconds)
    uint32_t m_currentRTT;         // Current RTT (microseconds)
    std::chrono::steady_clock::time_point m_baseRTTTimestamp;  // When baseRTT was updated
    
    // Vegas thresholds (in segments)
    uint32_t m_alpha;              // Lower threshold for cwnd increase
    uint32_t m_beta;               // Upper threshold for cwnd decrease
    uint32_t m_gamma;              // Slow start exit threshold
    
    // Throughput tracking
    uint64_t m_cntRtt;             // RTT counter for updates
    uint64_t m_minRtt;             // Minimum RTT in current period
    bool m_doingVegasNow;          // Currently using Vegas logic
    
    // Byte tracking for rate calculation
    uint64_t m_begSndNxt;          // Begin sequence number for rate calculation
    uint64_t m_endSndNxt;          // End sequence number
    uint32_t m_segmentsSent;       // Segments sent in current RTT
    uint32_t m_segmentsAcked;      // Segments acked in current RTT
    
    // Timestamps for rate calculation
    std::chrono::steady_clock::time_point m_begTime;  // Begin time for measurement
    
    // Slow start parameters
    uint32_t m_ssCount;            // Slow start counter
    bool m_enableSlowStart;        // Enable Vegas slow start logic
    
    // Configuration constants
    static constexpr uint32_t DEFAULT_ALPHA = 2;       // 2 segments
    static constexpr uint32_t DEFAULT_BETA = 4;        // 4 segments
    static constexpr uint32_t DEFAULT_GAMMA = 1;       // 1 segment for SS exit
    static constexpr uint32_t BASE_RTT_WINDOW_SEC = 10; // Base RTT validity window
    static constexpr uint32_t RTT_SAMPLE_WINDOW = 100;  // Keep 100 RTT samples
    
    // Helper methods
    void CleanupOldRTTSamples();
    void InitializeVegas();
    void EnableVegas();
    void DisableVegas();
};

#endif // VEGAS_H

