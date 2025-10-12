/*
@Author: Lzww
@LastEditTime: 2025-10-12 17:18:33
@Description: Copa (Delay-based Congestion control) Algorithm
@Language: C++17
*/

#ifndef COPA_H
#define COPA_H

#include "../utils/cong.h"

#include <string>
#include <chrono>
#include <deque>
#include <cmath>

// Copa operating modes
enum class CopaMode {
    SLOW_START,         // Slow start phase
    COMPETITIVE,        // Competitive mode (with other flows)
    VELOCITY            // Velocity mode (adjust rate based on delay)
};

// RTT measurement structure
struct CopaRTTMeasurement {
    uint32_t rtt_us;                                      // RTT in microseconds
    std::chrono::steady_clock::time_point timestamp;      // When measured
    
    CopaRTTMeasurement(uint32_t rtt = 0) 
        : rtt_us(rtt), timestamp(std::chrono::steady_clock::now()) {}
};

class Copa: public CongestionControl {
public:
    Copa();
    Copa(const Copa& other);
    ~Copa() override;
    
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
    // Copa state machine
    virtual void EnterSlowStart();
    virtual void EnterCompetitiveMode();
    virtual void EnterVelocityMode();
    
    // Copa core methods
    virtual void CopaUpdate(std::unique_ptr<SocketState>& socket, uint32_t ackedBytes, uint64_t rtt);
    
    // RTT tracking
    virtual void UpdateRTT(uint64_t rtt);
    virtual uint32_t GetMinRTT() const;
    virtual uint32_t GetStandingQueueDelay() const;
    
    // Rate calculation
    virtual double CalculateVelocity();
    virtual uint32_t CalculateTargetRate();
    virtual void UpdateCwndFromRate(std::unique_ptr<SocketState>& socket);
    
    // Mode transitions
    virtual bool ShouldExitSlowStart();
    virtual void CheckModeTransition();

private:
    // Standard TCP parameters
    uint32_t m_ssthresh;           // Slow start threshold
    uint32_t m_cwnd;               // Current congestion window
    uint32_t m_maxCwnd;            // Maximum congestion window
    
    // Copa mode
    CopaMode m_mode;               // Current operating mode
    
    // RTT measurements
    std::deque<CopaRTTMeasurement> m_rttSamples;  // Recent RTT samples
    uint32_t m_minRTT;             // Minimum RTT observed (base RTT, microseconds)
    uint32_t m_standingRTT;        // Standing RTT (with queueing delay)
    std::chrono::steady_clock::time_point m_minRTTTimestamp;  // When minRTT was updated
    
    // Copa parameters
    double m_delta;                // Target queueing delay (in RTTs), typically 0.5
    double m_velocity;             // Current rate adjustment velocity
    uint32_t m_targetRate;         // Target sending rate (bytes/sec)
    
    // Competitive mode parameters
    bool m_useCompetitiveMode;     // Whether to use competitive mode
    uint32_t m_competitiveDelta;   // Competitive delta (packets)
    
    // Rate tracking
    uint64_t m_lastRateUpdate;     // Last time rate was updated (microseconds)
    uint32_t m_deliveredBytes;     // Bytes delivered in current RTT
    std::chrono::steady_clock::time_point m_rttStart;  // Start of current RTT
    
    // Slow start parameters
    uint32_t m_ssExitThreshold;    // Exit slow start if queueing delay exceeds this
    bool m_inSlowStart;            // Currently in slow start
    
    // Direction tracking for velocity adjustment
    int32_t m_prevDirection;       // Previous velocity direction (+1, 0, -1)
    double m_prevQueueingDelay;    // Previous queueing delay
    
    // Statistics
    uint32_t m_rttCount;           // Number of RTT samples collected
    uint32_t m_totalAckedBytes;    // Total bytes acknowledged
    
    // Configuration constants
    static constexpr double DEFAULT_DELTA = 0.5;           // 0.5 RTT target delay
    static constexpr double VELOCITY_GAIN = 1.0;           // Velocity adjustment gain
    static constexpr uint32_t MIN_RTT_WINDOW_SEC = 10;    // Min RTT validity window
    static constexpr uint32_t RTT_SAMPLE_WINDOW = 100;    // Keep 100 RTT samples
    static constexpr uint32_t SS_EXIT_THRESHOLD_US = 1000;// 1ms queueing delay to exit SS
    
    // Helper methods
    void CleanupOldRTTSamples();
    void InitializeParameters();
    double GetQueueingDelay() const;  // Current queueing delay in RTTs
};

#endif // COPA_H

