/*
@Author: Lzww
@LastEditTime: 2025-9-17 20:00:00
@Description: Congestion Control Framework - Support for Reno, Cubic, BIC, BBR, etc.
@Language: C++17
*/

#ifndef CONG_H
#define CONG_H

#include <cstdint>
#include <chrono>
#include <string>
#include <memory>

// TCP connection states
enum class TCPState {
    Open,     // normal state
    Disorder, // disorder state
    CWR,      // congestion window reduced state
    Recovery, // quick recovery state
    Loss,     // loss state(timeout)
};

// Congestion control algorithm types
enum class CongestionAlgorithm {
    Reno,
    NewReno,
    Cubic,
    BIC,
    BBR,
    BBRv2,
    Vegas,
    Westwood
};

// Congestion event types
enum class CongestionEvent {
    SlowStart,              // slow start
    CongestionAvoidance,    // congestion avoidance
    FastRecovery,           // fast recovery
    Timeout,                // timeout
    ECN,                    // explicit congestion notification
    PacketLoss,             // packet loss
    Reordering              // packet reordering
};

// Basic data structures - keep only essential ones, complex algorithms can define their own

// Simple RTT information (algorithms like BBR can extend this)
struct RTTSample {
    std::chrono::microseconds rtt;  // current RTT
    RTTSample(std::chrono::microseconds r = std::chrono::microseconds(0)) : rtt(r) {}
};

// Basic congestion control parameters (algorithms can extend as needed)
struct BasicCongestionParams {
    uint32_t mss;               // maximum segment size
    uint32_t max_cwnd;          // maximum congestion window
    
    BasicCongestionParams() : mss(1460), max_cwnd(65535) {}
};

// Congestion control base class (pure virtual) - contains only core interfaces
class CongestionControl {
public:
    virtual ~CongestionControl() = default;

    // ====== Core pure virtual interfaces - must be implemented by all algorithms ======
    
    // Algorithm identification
    virtual CongestionAlgorithm GetAlgorithm() const = 0;
    
    // Core congestion control events - the three most basic events
    virtual void OnAck(uint32_t acked_bytes) = 0;        // ACK received (basic version)
    virtual void OnLoss() = 0;                           // packet loss detected
    virtual void OnTimeout() = 0;                        // timeout occurred
    
    // Extended version - algorithms that need RTT information can override this method
    virtual void OnAck(uint32_t acked_bytes, const RTTSample& rtt) {
        OnAck(acked_bytes);  // default calls basic version, ignoring RTT
    }

    // Window management - basic window retrieval
    virtual uint32_t GetCongestionWindow() const = 0;    // get current congestion window
    
    // ====== Optional interfaces - different algorithms may need, with default empty implementations ======
    
    // Extended event handling (some algorithms may not need)
    virtual void OnECN() {}                              // ECN signal (optional)
    virtual void OnReordering() {}                       // packet reordering (optional)
    
    // Optional parameter configuration (not all algorithms need)
    virtual void SetMSS(uint32_t mss) {}                // set MSS (optional)
    virtual void SetMaxWindow(uint32_t max_cwnd) {}      // set maximum window (optional)
    
    // Optional state queries
    virtual uint32_t GetSlowStartThreshold() const { return UINT32_MAX; } // slow start threshold (optional)
    
    // Debug interface (optional)
    virtual std::string GetStatusString() const { return ""; }  // status string (optional)

protected:
    // Constructor - can only be called by subclasses
    CongestionControl() = default;

private:
    // Disable copy and assignment
    CongestionControl(const CongestionControl&) = delete;
    CongestionControl& operator=(const CongestionControl&) = delete;
};


#endif