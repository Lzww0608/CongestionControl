# Congestion Control Algorithm Framework

**Author**: Lzww  
**Language**: C++17  
**Description**: A comprehensive implementation of modern TCP congestion control algorithms

---

## 项目概述

这是一个完整的拥塞控制算法框架，实现了从经典到现代的多种 TCP 拥塞控制算法。本项目采用面向对象设计，提供统一的基类接口，使得不同算法可以灵活实现和对比。

## 核心设计理念

1. **统一接口**: 所有算法继承自 `CongestionControl` 基类，提供一致的调用接口
2. **算法自治**: 每个算法可以定义自己的状态变量和辅助方法
3. **易于扩展**: 清晰的类层次结构，便于添加新算法
4. **完整实现**: 包含慢启动、拥塞避免、快速恢复等完整的 TCP 状态机

---

## 已实现的算法

### 1. **Reno** (经典基于丢包)
- **文件**: `reno/reno.h`, `reno/reno.cpp`
- **特点**: 
  - TCP 的经典实现
  - AIMD (加性增、乘性减)
  - 快速重传和快速恢复
- **适用场景**: 传统互联网、低带宽网络

### 2. **BIC** (Binary Increase Congestion control)
- **文件**: `bic/bic.h`, `bic/bic.cpp`
- **特点**:
  - 二分搜索算法
  - 快速收敛到最优窗口
  - β = 0.8（比 Reno 温和）
- **适用场景**: 高带宽长延迟网络

### 3. **CUBIC** (Linux 默认算法)
- **文件**: `cubic/cubic.h`, `cubic/cubic.cpp`
- **特点**:
  - 使用三次函数控制窗口增长
  - RTT 公平性好
  - Fast Convergence 和 TCP-Friendly 模式
  - Hystart (混合慢启动)
- **核心公式**: `W(t) = C × (t - K)³ + W_max`
- **适用场景**: 通用场景，Linux 默认

### 4. **BBR** (Bottleneck Bandwidth and RTT)
- **文件**: `bbr/bbr.h`, `bbr/bbr.cpp`
- **文档**: `docs/BBR/`
- **特点**:
  - 不依赖丢包信号
  - 四个状态：STARTUP, DRAIN, PROBE_BW, PROBE_RTT
  - 基于 BDP (带宽时延积)
  - Pacing rate 控制
- **核心思想**: `cwnd = BDP × gain`
- **适用场景**: 高带宽长延迟、无线网络、流媒体

### 5. **Copa** (Delay-based with Velocity control)
- **文件**: `copa/copa.h`, `copa/copa.cpp`
- **文档**: `docs/COPA/`
- **特点**:
  - 基于延迟的精确控制
  - 速度控制机制
  - 目标排队延迟: δ = 0.5 RTT
- **核心公式**: 
  - `排队延迟 = standing_RTT - min_RTT`
  - `rate(t+1) = rate(t) × (1 + v(t) × δ)`
- **适用场景**: 延迟敏感应用、数据中心

### 6. **DCTCP** (Data Center TCP)
- **文件**: `dctcp/dctcp.h`, `dctcp/dctcp.cpp`
- **文档**: `docs/DCTCP/`
- **特点**:
  - 基于 ECN (显式拥塞通知)
  - α 参数表示拥塞程度
  - 按比例减少窗口
- **核心公式**: 
  - `α = (1 - g) × α + g × F`
  - `cwnd = cwnd × (1 - α/2)`
- **适用场景**: 数据中心网络（需要 ECN 支持）

### 7. **Vegas** (Delay-based, 历史性)
- **文件**: `vegas/vegas.h`, `vegas/vegas.cpp`
- **特点**:
  - 最早的基于延迟的算法
  - 主动避免拥塞
  - α, β 阈值控制
- **核心思想**: `Diff = Expected - Actual`
- **适用场景**: 学术研究、低竞争环境

---

## 算法对比

| 算法 | 类型 | 拥塞信号 | 窗口调整 | RTT公平性 | 收敛速度 | 延迟 | 适用场景 |
|------|------|---------|---------|-----------|---------|------|---------|
| **Reno** | 基于丢包 | 丢包 | AIMD (×0.5) | 中 | 慢 | 高 | 传统网络 |
| **BIC** | 基于丢包 | 丢包 | 二分搜索 | 中 | 快 | 中 | 高带宽长延迟 |
| **CUBIC** | 基于丢包 | 丢包 | 三次函数 | 好 | 快 | 中 | 通用（Linux默认）|
| **BBR** | 模型驱动 | BW+RTT | 基于BDP | 一般 | 极快 | 低 | 高BW、无线、流媒体 |
| **Copa** | 基于延迟 | 排队延迟 | 速度控制 | 好 | 中 | 极低 | 延迟敏感应用 |
| **DCTCP** | 基于ECN | ECN标记 | 按比例 (×(1-α/2)) | 好 | 快 | 极低 | 数据中心 |
| **Vegas** | 基于延迟 | RTT变化 | ±1 MSS | 较差* | 慢 | 低 | 学术研究 |

*注: Vegas 在与基于丢包的算法竞争时可能处于劣势

---

## 项目结构

```
CC/
├── utils/
│   ├── cong.h              # 基类定义和数据结构
│   └── cong.cpp            # 基类实现
│
├── reno/
│   ├── reno.h              # Reno 算法头文件
│   └── reno.cpp            # Reno 算法实现
│
├── bic/
│   ├── bic.h               # BIC 算法头文件
│   └── bic.cpp             # BIC 算法实现
│
├── cubic/
│   ├── cubic.h             # CUBIC 算法头文件
│   └── cubic.cpp           # CUBIC 算法实现
│
├── bbr/
│   ├── bbr.h               # BBR 算法头文件
│   └── bbr.cpp             # BBR 算法实现
│
├── copa/
│   ├── copa.h              # Copa 算法头文件
│   └── copa.cpp            # Copa 算法实现
│
├── dctcp/
│   ├── dctcp.h             # DCTCP 算法头文件
│   └── dctcp.cpp           # DCTCP 算法实现
│
├── vegas/
│   ├── vegas.h             # Vegas 算法头文件
│   └── vegas.cpp           # Vegas 算法实现
│
├── docs/                   # 详细文档
│   ├── BBR/
│   │   ├── BBR.md         # BBR 算法详解
│   │   └── RTT测量.md     # RTT 测量机制
│   ├── COPA/
│   │   ├── copa.md        # Copa 算法详解
│   │   └── 排队延迟.md    # 排队延迟计算
│   └── DCTCP/
│       ├── dctcp.md       # DCTCP 算法详解
│       ├── ecn.md         # ECN 机制
│       └── 路由器ecn.md   # 路由器 ECN 配置
│
└── README.md               # 本文件
```

---

## 核心接口

### CongestionControl 基类

```cpp
class CongestionControl {
public:
    // 算法标识
    virtual std::string GetAlgorithmName() = 0;
    virtual TypeId GetTypeId() = 0;
    
    // 核心方法
    virtual uint32_t GetSsThresh(std::unique_ptr<SocketState>& socket, 
                                  uint32_t bytesInFlight) = 0;
    
    virtual void IncreaseWindow(std::unique_ptr<SocketState>& socket, 
                                uint32_t segmentsAcked);
    
    virtual void PktsAcked(std::unique_ptr<SocketState>& socket, 
                           uint32_t segmentsAcked, 
                           const uint64_t rtt);
    
    virtual void CongestionStateSet(std::unique_ptr<SocketState>& socket, 
                                     const TCPState congestionState);
    
    virtual void CwndEvent(std::unique_ptr<SocketState>& socket, 
                          const CongestionEvent congestionEvent);
    
    virtual void CongControl(std::unique_ptr<SocketState>& socket,
                            const CongestionEvent& congestionEvent,
                            const RTTSample& rtt);
    
    virtual bool HasCongControl() const;
};
```

### SocketState 结构

```cpp
class SocketState {
public:
    TCPState tcp_state_;              // TCP 状态
    CongestionEvent congestion_event_; // 拥塞事件
    uint32_t cwnd_;                   // 拥塞窗口
    uint32_t ssthresh_;               // 慢启动阈值
    uint32_t max_cwnd_;               // 最大窗口
    uint32_t mss_bytes_;              // MSS大小
    uint32_t rtt_us_;                 // RTT (微秒)
    uint32_t rto_us_;                 // RTO
    uint32_t rtt_var_;                // RTT 方差
};
```

---

## 使用示例

### 基本使用

```cpp
#include "reno/reno.h"
#include "cubic/cubic.h"
#include "bbr/bbr.h"

// 创建算法实例
auto reno = std::make_unique<Reno>();
auto cubic = std::make_unique<Cubic>();
auto bbr = std::make_unique<BBR>();

// 创建 socket 状态
auto socket = std::make_unique<SocketState>();
socket->mss_bytes_ = 1460;
socket->cwnd_ = 4 * 1460;  // 初始窗口
socket->ssthresh_ = 0x7fffffff;

// 处理 ACK
reno->PktsAcked(socket, 1, 50000);  // 1 segment, RTT=50ms
reno->IncreaseWindow(socket, 1);

// 处理拥塞事件
reno->CwndEvent(socket, CongestionEvent::PacketLoss);

// 获取当前窗口
std::cout << "Current cwnd: " << socket->cwnd_ << std::endl;
```

### 算法对比测试

```cpp
// 创建不同算法实例
std::vector<std::unique_ptr<CongestionControl>> algorithms;
algorithms.push_back(std::make_unique<Reno>());
algorithms.push_back(std::make_unique<Cubic>());
algorithms.push_back(std::make_unique<BBR>());

// 对比测试
for (auto& algo : algorithms) {
    std::cout << "Testing: " << algo->GetAlgorithmName() << std::endl;
    // 运行测试...
}
```

---

## 关键概念

### TCP 状态

```cpp
enum class TCPState {
    Open,       // 正常状态
    Disorder,   // 轻微失序
    CWR,        // 拥塞窗口减小
    Recovery,   // 快速恢复
    Loss        // 丢包状态（超时）
};
```

### 拥塞事件

```cpp
enum class CongestionEvent {
    SlowStart,          // 慢启动
    CongestionAvoidance,// 拥塞避免
    FastRecovery,       // 快速恢复
    Timeout,            // 超时
    ECN,                // ECN 信号
    PacketLoss,         // 丢包
    Reordering          // 乱序
};
```

---

## 编译要求

- **C++17** 或更高版本
- 支持的编译器:
  - GCC 7.0+
  - Clang 5.0+
  - MSVC 2017+

### 编译示例

```bash
# 编译单个算法
g++ -std=c++17 -o reno_test reno/reno.cpp utils/cong.cpp test_main.cpp

# 编译所有算法
g++ -std=c++17 -o cc_test \
    reno/reno.cpp \
    cubic/cubic.cpp \
    bbr/bbr.cpp \
    copa/copa.cpp \
    dctcp/dctcp.cpp \
    vegas/vegas.cpp \
    bic/bic.cpp \
    utils/cong.cpp \
    main.cpp
```

---

## 详细文档

项目包含详细的算法文档，位于 `docs/` 目录：

### BBR 文档
- `docs/BBR/BBR.md`: BBR 四个阶段详解
- `docs/BBR/RTT测量.md`: TCP 时间戳和 RTT 测量机制

### Copa 文档
- `docs/COPA/copa.md`: Copa 速度控制机制
- `docs/COPA/排队延迟.md`: 排队延迟计算方法

### DCTCP 文档
- `docs/DCTCP/dctcp.md`: DCTCP α 参数更新算法
- `docs/DCTCP/ecn.md`: ECN 三方协作机制
- `docs/DCTCP/路由器ecn.md`: 路由器 ECN 配置

---

## 算法特性总结

### 基于丢包的算法 (Loss-based)
- **Reno**: 最经典，AIMD，简单稳定
- **BIC**: 二分搜索，收敛快
- **CUBIC**: 三次函数，Linux 默认，RTT 公平

### 基于延迟的算法 (Delay-based)
- **Vegas**: 历史性，主动避免拥塞
- **Copa**: 现代化，速度控制，延迟精确

### 模型驱动的算法 (Model-based)
- **BBR**: 革命性，基于 BDP，不依赖丢包

### 数据中心专用算法
- **DCTCP**: ECN 驱动，按比例控制，极低延迟

---

## 性能特点

### 吞吐量排序 (高→低)
1. BBR (高带宽场景)
2. CUBIC
3. BIC
4. Reno
5. DCTCP (数据中心优化)
6. Copa
7. Vegas

### 延迟排序 (低→高)
1. DCTCP (需 ECN)
2. Copa
3. BBR
4. Vegas
5. CUBIC
6. BIC
7. Reno

### 公平性排序 (好→差)
1. Reno
2. CUBIC
3. DCTCP
4. BIC
5. Copa
6. BBR
7. Vegas (混合环境)

---

## 参考资料

### 经典论文
- **Reno**: TCP Congestion Control (RFC 5681)
- **Vegas**: TCP Vegas: End to End Congestion Avoidance on a Global Internet (1995)
- **BIC**: Binary Increase Congestion Control (BIC-TCP) (2004)
- **CUBIC**: CUBIC: A New TCP-Friendly High-Speed TCP Variant (2008)
- **DCTCP**: Data Center TCP (DCTCP) (SIGCOMM 2010)
- **BBR**: BBR: Congestion-Based Congestion Control (ACM Queue 2016)
- **Copa**: Copa: Practical Delay-Based Congestion Control for the Internet (NSDI 2018)

### 在线资源
- [RFC 5681 - TCP Congestion Control](https://tools.ietf.org/html/rfc5681)
- [RFC 8312 - CUBIC](https://tools.ietf.org/html/rfc8312)
- [BBR GitHub](https://github.com/google/bbr)
- [Linux TCP Implementation](https://elixir.bootlin.com/linux/latest/source/net/ipv4)

---

## License

本项目仅用于学习和研究目的。

---

## Author

**Lzww**  
Last Update: 2025-10-13

---

## 致谢

感谢所有为 TCP 拥塞控制算法发展做出贡献的研究者和工程师们。
