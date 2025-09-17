# Congestion Control Algorithm Framework

## Design Philosophy

This is a lightweight congestion control algorithm framework using pure virtual base class design. It defines only the most essential interfaces, allowing different algorithms to implement according to their own needs.

## Core Design Principles

1. **Minimal Interface**: Define only core methods that all algorithms must implement
2. **Optional Extensions**: Complex functionality provided through optional interfaces with default implementations
3. **Algorithm Autonomy**: Each algorithm can define its own data structures and state management

## Base Class Interface

### Required Core Interfaces
```cpp
class CongestionControl {
public:
    // Algorithm identification
    virtual CongestionAlgorithm GetAlgorithm() const = 0;
    
    // Core event handling
    virtual void OnAck(uint32_t acked_bytes) = 0;     // ACK received
    virtual void OnLoss() = 0;                        // packet loss detected  
    virtual void OnTimeout() = 0;                     // timeout occurred
    
    // Window management
    virtual uint32_t GetCongestionWindow() const = 0; // get congestion window
};
```

### Optional Interfaces (with default implementations)
```cpp
    // RTT version of OnAck (algorithms like BBR may need this)
    virtual void OnAck(uint32_t acked_bytes, const RTTSample& rtt);
    
    // Other optional events
    virtual void OnECN();                            // ECN signal
    virtual void OnReordering();                     // packet reordering
    
    // Parameter configuration
    virtual void SetMSS(uint32_t mss);              // set MSS
    virtual void SetMaxWindow(uint32_t max_cwnd);    // set maximum window
    
    // State queries
    virtual uint32_t GetSlowStartThreshold() const; // slow start threshold
    virtual std::string GetStatusString() const;    // debug string
```

## Usage Examples

### Implementing Simple Algorithm (like Reno)
```cpp
#include "utils/cong.h"

class RenoCC : public CongestionControl {
private:
    uint32_t cwnd_;
    uint32_t ssthresh_;
    
public:
    RenoCC() : cwnd_(1), ssthresh_(65535) {}
    
    CongestionAlgorithm GetAlgorithm() const override {
        return CongestionAlgorithm::Reno;
    }
    
    void OnAck(uint32_t acked_bytes) override {
        // Reno congestion window growth logic
        if (cwnd_ < ssthresh_) {
            cwnd_ += 1;  // slow start
        } else {
            cwnd_ += 1460 / cwnd_;  // congestion avoidance
        }
    }
    
    void OnLoss() override {
        ssthresh_ = cwnd_ / 2;
        cwnd_ = ssthresh_;
    }
    
    void OnTimeout() override {
        ssthresh_ = cwnd_ / 2;
        cwnd_ = 1;
    }
    
    uint32_t GetCongestionWindow() const override {
        return cwnd_;
    }
};

// Usage
auto reno = std::make_unique<RenoCC>();
reno->OnAck(1460);
```

### Implementing Complex Algorithm (like BBR)
```cpp
class BBRCC : public CongestionControl {
private:
    uint32_t cwnd_;
    RTTSample min_rtt_;
    uint64_t max_bandwidth_;
    // BBR-specific states and parameters...
    
public:
    // Override RTT version of OnAck method
    void OnAck(uint32_t acked_bytes, const RTTSample& rtt) override {
        // BBR logic that needs RTT information
        updateMinRtt(rtt);
        updateBandwidth(acked_bytes, rtt);
        // ... BBR algorithm logic
    }
    
    // Other required methods...
};
```

## Supported Algorithm Types
- Reno
- NewReno  
- Cubic
- BIC
- BBR
- BBRv2
- Vegas
- Westwood

## File Structure
```
CC/
├── utils/
│   ├── cong.h          # Base class definition and basic data structures (header-only)
│   └── cong.cpp        # Usage documentation
├── algorithms/         # Specific algorithm implementations (implemented by users)
│   ├── reno.h          # Users can create their own
│   ├── cubic.h
│   └── bbr.h
└── README.md          # This file
```

## Features

- **Header-only library**: Just include `cong.h`
- **Pure virtual base class**: Maximum flexibility for algorithm implementations
- **Minimal core interface**: Only 4 methods must be implemented
- **Optional extensions**: Rich optional functionality with sensible defaults
- **No factory pattern**: Direct instantiation for simplicity
- **Algorithm autonomy**: Each algorithm manages its own state and data structures