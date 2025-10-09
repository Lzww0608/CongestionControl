/*
@Author: Lzww
@LastEditTime: 2025-10-9 21:50:05
@Description: Congestion Control Framework - Support for Reno, Cubic, BIC, BBR, etc.
@Language: C++17
*/

#ifndef CONG_H
#define CONG_H

#include <cstdint>
#include <chrono>
#include <string>
#include <memory>

using TypeId = uint64_t;

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
    BBR,
    BIC,
    CUBIC,
    DCTCP,
    RENO,
    VEGAS,
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

// Basic data structures

// Simple RTT information (algorithms like BBR can extend this)
struct RTTSample {
    std::chrono::microseconds rtt;  // current RTT
    RTTSample(std::chrono::microseconds r = std::chrono::microseconds(0)) : rtt(r) {}
};

// Basic congestion control parameters
struct BasicCongestionParams {
    uint32_t mss;               // maximum segment size
    uint32_t max_cwnd;          // maximum congestion window
    
    BasicCongestionParams() : mss(1460), max_cwnd(65535) {}
};


class SocketState {
public: 
    SocketState();
    virtual ~SocketState() = default;

    TCPState tcp_state_;
    CongestionEvent congestion_event_;
    uint32_t cwnd_;
    uint32_t ssthresh_;
    uint32_t max_cwnd_;
    uint32_t mss_bytes_;
    uint32_t rtt_us_;
    uint32_t rto_us_;
    uint32_t rtt_var_;
};

// Congestion control base class (pure virtual) - contains only core interfaces
class CongestionControl {
public:
    CongestionControl(TypeId type_id, std::string algorithm_name);
    virtual ~CongestionControl() = default;

    /**
     * @brief Get the type ID.
     * @return the object type ID.
     */
    TypeId GetTypeId();

    void SetTypeId(TypeId type_id) [[maybe_unused]];


    /**
     * @brief Get the name of the congestion control algorithm
     *
     * @return A string identifying the name
     */
    virtual std::string GetAlgorithmName() = 0;

    /**
     * @brief Get the slow start threshold.
     *
     * @param socket internal congestion state
     * @param bytesInFlight total bytes in flight
     * @return Slow start threshold
     */
    virtual uint32_t GetSsThresh(std::unique_ptr<SocketState>& socket, uint32_t bytesInFlight) = 0;

    /**
     * @brief Increase the congestion window.
     *
     * @param socket internal congestion state
     * @param segmentsAcked number of segments acked
     */
    virtual void IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    /**
     * @brief Handle packets acked.
     *
     * @param socket internal congestion state
     * @param segmentsAcked number of segments acked
     * @param rtt round trip time
     */
    virtual void PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt);

    /**
     * @brief Set the congestion state.
     *
     * @param socket internal congestion state
     * @param congestionState congestion state
     */
    virtual void CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState);

    /**
     * @brief Handle the congestion event.
     *
     * @param socket internal congestion state
     * @param congestionEvent congestion event
     */
    virtual void CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent);

    /**
     * @brief Check if the congestion control is enabled.
     *
     * @return true if the congestion control is enabled, false otherwise
     */
    virtual bool HasCongControl() const;

    virtual void CongControl(std::unique_ptr<SocketState>& socket,
                             const CongestionEvent& congestionEvent,
                             const RTTSample& rtt);

protected:
    // Constructor - can only be called by subclasses
    CongestionControl() = default;

private:
    // Disable copy and assignment
    CongestionControl(const CongestionControl&) = delete;
    CongestionControl& operator=(const CongestionControl&) = delete;

    // Type identification
    TypeId m_typeId;                        // Type identifier for this congestion control
    std::string m_algorithmName;            // Name of the congestion control algorithm
    
    // TCP state management
    TCPState m_tcpState;                    // Current TCP congestion state
    CongestionEvent m_congestionEvent;      // Current congestion event type
    
    // Congestion window parameters
    uint32_t m_cwnd;                        // Current congestion window size (in bytes)
    uint32_t m_ssthresh;                    // Slow start threshold (in bytes)
    uint32_t m_maxCwnd;                     // Maximum congestion window size (in bytes)
    uint32_t m_segmentSize;                 // Maximum segment size (MSS in bytes)
    uint32_t m_initialCwnd;                 // Initial congestion window size
    
    // RTT and RTO parameters
    uint32_t m_rttUs;                       // Current round trip time (in microseconds)
    uint32_t m_rtoUs;                       // Retransmission timeout (in microseconds)
    uint32_t m_rttVar;                      // RTT variation for RTO calculation
    uint32_t m_minRtt;                      // Minimum RTT observed (in microseconds)
    
    // Packet tracking
    uint32_t m_bytesInFlight;               // Number of bytes currently in flight
    uint32_t m_segmentsAcked;               // Number of segments acknowledged
    
    // Control flags
    bool m_congControlEnabled;              // Whether congestion control is enabled
};


#endif