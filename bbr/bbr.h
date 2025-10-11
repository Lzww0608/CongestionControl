/*
@Author: Lzww
@LastEditTime: 2025-10-11 23:26:50
@Description: BBR (Bottleneck Bandwidth and RTT) Congestion Control Algorithm
@Language: C++17
*/

#ifndef BBR_H
#define BBR_H

#include "../utils/cong.h"

#include <string>
#include <chrono>
#include <deque>
#include <algorithm>

// BBR operating modes
enum class BBRMode {
    STARTUP,        // Exponential growth to find bandwidth
    DRAIN,          // Drain the queue created during STARTUP
    PROBE_BW,       // Cyclically probe for more bandwidth
    PROBE_RTT       // Probe for minimum RTT
};

// Bandwidth sample structure
struct BandwidthSample {
    uint64_t bandwidth;     // Bytes per second
    std::chrono::steady_clock::time_point timestamp;
    
    BandwidthSample(uint64_t bw = 0) 
        : bandwidth(bw), timestamp(std::chrono::steady_clock::now()) {}
};

// RTT sample with timestamp
struct BBRRTTSample {
    uint32_t rtt_us;        // RTT in microseconds
    std::chrono::steady_clock::time_point timestamp;
    
    BBRRTTSample(uint32_t rtt = 0) 
        : rtt_us(rtt), timestamp(std::chrono::steady_clock::now()) {}
};

class BBR: public CongestionControl {
public:
    BBR();
    BBR(const BBR& other);
    ~BBR() override;
    
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
    // BBR state machine methods
    virtual void EnterStartup();
    virtual void EnterDrain();
    virtual void EnterProbeBW();
    virtual void EnterProbeRTT();
    
    // BBR main update logic
    virtual void BBRUpdate(std::unique_ptr<SocketState>& socket, uint32_t ackedBytes, uint64_t rtt);
    
    // Bandwidth estimation
    virtual void UpdateBandwidth(uint32_t ackedBytes, uint64_t rtt);
    virtual uint64_t GetMaxBandwidth() const;
    
    // RTT estimation
    virtual void UpdateMinRTT(uint64_t rtt);
    virtual uint32_t GetMinRTT() const;
    
    // Calculate target cwnd
    virtual uint32_t CalculateTargetCwnd(uint32_t gain_percent);
    
    // Calculate pacing rate
    virtual uint64_t CalculatePacingRate(uint32_t gain_percent);
    
    // Check if we should enter PROBE_RTT
    virtual bool ShouldProbeRTT();
    
    // Update gain for PROBE_BW cycling
    virtual void UpdateProbeBWGain();
    
    // Check for full pipe (bandwidth plateau)
    virtual bool IsFullPipe() const;

private:
    // Standard TCP parameters
    uint32_t m_cwnd;               // Current congestion window
    uint32_t m_maxCwnd;            // Maximum congestion window
    
    // BBR mode
    BBRMode m_mode;                // Current BBR mode
    
    // Bandwidth tracking
    std::deque<BandwidthSample> m_bandwidthSamples;  // Bandwidth samples
    uint64_t m_maxBandwidth;       // Maximum bandwidth observed (bytes/sec)
    uint32_t m_bandwidthWindow;    // Window size for bandwidth samples (RTTs)
    
    // RTT tracking
    std::deque<BBRRTTSample> m_rttSamples;    // RTT samples
    uint32_t m_minRTT;             // Minimum RTT observed (microseconds)
    std::chrono::steady_clock::time_point m_minRTTTimestamp;  // When minRTT was last updated
    uint32_t m_minRTTWindow;       // Window size for minRTT validity (seconds)
    
    // Pacing rate
    uint64_t m_pacingRate;         // Current pacing rate (bytes/sec)
    
    // BBR gain factors (in percentage)
    uint32_t m_pacingGain;         // Pacing rate gain
    uint32_t m_cwndGain;           // Congestion window gain
    
    // STARTUP phase tracking
    uint64_t m_prevMaxBandwidth;   // Previous max bandwidth for plateau detection
    uint32_t m_roundsWithoutGrowth;// Rounds without bandwidth growth
    uint32_t m_roundCount;         // Current round number
    uint64_t m_lastRoundStartSeq;  // Sequence at start of current round
    
    // PROBE_BW cycling
    uint32_t m_probeBWCycleIndex;  // Current position in gain cycle
    std::chrono::steady_clock::time_point m_probeBWCycleStart;  // Start of current cycle
    static constexpr uint32_t PROBE_BW_GAINS[8] = {125, 75, 100, 100, 100, 100, 100, 100};
    
    // PROBE_RTT state
    std::chrono::steady_clock::time_point m_probeRTTStart;  // When PROBE_RTT started
    uint32_t m_probeRTTDuration;   // Duration to stay in PROBE_RTT (ms)
    bool m_probeRTTRoundDone;      // Completed one round at min cwnd
    
    // Packet accounting
    uint64_t m_deliveredBytes;     // Total bytes delivered
    uint64_t m_deliveredTime;      // Time of last delivery (microseconds)
    
    // Configuration constants
    static constexpr uint32_t STARTUP_GAIN = 289;      // 2/ln(2) ≈ 2.89
    static constexpr uint32_t DRAIN_GAIN = 100;        // 1.0
    static constexpr uint32_t PROBE_BW_GAIN = 100;     // 1.0 (base)
    static constexpr uint32_t CWND_GAIN = 200;         // 2.0
    static constexpr uint32_t HIGH_GAIN = 289;         // 2.89
    static constexpr uint32_t PROBE_RTT_CWND_GAIN = 50; // 0.5
    
    // Thresholds
    static constexpr uint32_t BANDWIDTH_WINDOW_SIZE = 10;   // 10 RTTs
    static constexpr uint32_t MIN_RTT_WINDOW_SEC = 10;      // 10 seconds
    static constexpr uint32_t PROBE_RTT_DURATION_MS = 200;  // 200ms
    static constexpr uint32_t FULL_PIPE_ROUNDS = 3;         // Rounds to confirm full pipe
    static constexpr double FULL_PIPE_THRESHOLD = 1.25;     // 25% growth threshold
    
    // Helper methods
    void CleanupOldSamples();
    void InitializeParameters();
};

#endif // BBR_H

