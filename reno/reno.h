/*
@Author: Lzww
@LastEditTime: 2025-10-9 22:00:38
@Description: Reno Congestion Control Algorithm
@Language: C++17
*/

#ifndef RENO_H
#define RENO_H

#include "../utils/cong.h"

#include <string>

class Reno: public CongestionControl {
public:
    Reno();
    Reno(const Reno& other);
    ~Reno() override;
    
    TypeId GetTypeId();

    std::string GetAlgorithmName() override;

    void IncreaseWindow(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked) override;

    void PktsAcked(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked, const uint64_t rtt) override;

    void CongestionStateSet(std::unique_ptr<SocketState>& socket, const TCPState congestionState) override;

    void CwndEvent(std::unique_ptr<SocketState>& socket, const CongestionEvent congestionEvent) override;

    bool HasCongControl() const override;

    void CongControl(std::unique_ptr<SocketState>& socket, const CongestionEvent& congestionEvent, const RTTSample& rtt) override;

protected:
    virtual uint32_t SlowStart(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t CongestionAvoidance(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t FastRetransmit(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);

    virtual uint32_t FastRecovery(std::unique_ptr<SocketState>& socket, uint32_t segmentsAcked);
private:
    uint32_t m_ssthresh;
    uint32_t m_cwnd;
    uint32_t m_maxCwnd;
};

#endif // RENO_H