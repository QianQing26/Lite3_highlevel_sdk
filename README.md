# Lite3 Highlevel SDK

Lite3 四足机器人的**上层控制 SDK**，运行在计算侧（Jetson NX / PC）。通过 UDP 与 [Lite3 Lowlevel SDK](https://github.com/DeepRoboticsLab/Lite3_lowlevel_sdk)（RK3588）通信，负责传感器数据接收、策略推理和关节指令下发。

提供 **Python 和 C++ 两套 API**，接口一致。

---

## 架构

```
┌──────────────────────────────────────────────────┐
│              Lite3 Highlevel SDK (本仓库)           │
│                                                   │
│  ┌──────────┐  ┌────────────┐  ┌───────────────┐ │
│  │  Bridge   │  │ ObsBuilder  │  │  OnnxRunner   │ │
│  │ UDP 收发  │  │ 观测构建     │  │ ONNX 推理     │ │
│  └──────────┘  └────────────┘  └───────────────┘ │
│         │              │               │          │
│  ┌──────┴──────────────┴───────────────┴────────┐ │
│  │              CmdProcessor                     │ │
│  │        action → JointCommand 转换             │ │
│  └──────────────────────────────────────────────┘ │
│                                                   │
│  ┌──────────────────────────────────────────────┐ │
│  │          您的策略代码                           │ │
│  │    def policy(state) -> JointCommand           │ │
│  │    class MyPolicy : public PolicyBase          │ │
│  └──────────────────────────────────────────────┘ │
└──────────────────┬───────────────────────────────┘
                   │  UDP :31001 ← → :31002
┌──────────────────┴───────────────────────────────┐
│           Lite3 Lowlevel SDK (RK3588)              │
│      状态机 · 硬件通信 · PD 闭环 · 安全保护          │
└──────────────────────────────────────────────────┘
```

---

## 快速开始

### Python

```bash
pip install onnxruntime numpy
```

```python
from lite3_highlevel import Bridge, OnnxRunner, ObsBuilder, CmdProcessor

# 加载 ONNX 策略
runner = OnnxRunner("policy.onnx")
builder = ObsBuilder()
processor = CmdProcessor()

# 定义控制回调
last_action = [0.0] * 12
def policy(state):
    global last_action
    obs = builder.build(state, last_action=last_action)
    last_action = runner.infer(obs)
    return processor.process(last_action)

# 启动控制循环（阻塞，Ctrl+C 停止）
bridge = Bridge(robot_ip="192.168.1.120")
bridge.run(policy, frequency_hz=50)
```

约 20 行即可完成一个完整的 ONNX 策略部署。

### C++

```bash
# 下载 ONNX Runtime 放到 third_party/onnxruntime/
# 或手动指定: cmake .. -DONNXRUNTIME_ROOT=/path/to/onnxruntime
mkdir build && cd build
cmake ..
make -j
```

```cpp
#include "lite3_highlevel/lite3_highlevel.hpp"

int main() {
    lite3::OnnxRunner runner("policy.onnx");
    lite3::ObsBuilder builder;
    lite3::CmdProcessor processor;
    lite3::Bridge bridge("192.168.1.120");
    bridge.open();

    float obs[45], action[12], last_action[12] = {0};

    while (running) {
        SensorPacket pkt;
        if (bridge.recv(pkt)) {
            // 构建观测 → 推理 → 处理 → 发送
            builder.build(convert(pkt), obs);
            builder.setLastAction(last_action);
            runner.infer(obs, action);
            bridge.sendCommand(processor.process(action));
            std::memcpy(last_action, action, sizeof(action));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
```

无 ONNX Runtime 时，C++ 库以 header-only 模式构建，`Bridge` + `ObsBuilder` + `CmdProcessor` 仍然可用。

---

## API 对照

| 模块 | Python | C++ |
|------|--------|-----|
| UDP 通信 | `Bridge(ip).run(policy, hz=50)` | `Bridge::open()` + `recv()` / `sendCommand()` |
| 数据类 | `RobotState`, `JointCommand` | `lite3::RobotState`, `lite3::JointCommand` |
| 观测构建 | `ObsBuilder().build(state)` → list | `ObsBuilder().build(state, float*)` |
| 指令处理 | `CmdProcessor().process(action)` → JointCommand | `CmdProcessor().process(float*)` → JointCommand |
| ONNX 推理 | `OnnxRunner("m.onnx").infer(obs)` | `OnnxRunner("m.onnx").infer(float*, float*)` |

### ObsBuilder 默认格式（45 维）

与原始 Lite3 ONNX 策略兼容：

```
[omega*0.25 (3) | projected_gravity (3) | cmd_vel*max_vel (3) |
 joint_pos - default_pos (12) | joint_vel*0.05 (12) | last_action (12)]
```

可通过 `ObsConfig` / 构造函数参数自定义缩放因子和关节默认姿态。

### CmdProcessor 处理流程

```
raw_action[i]  →  action = raw_action[i] * action_scale[i]
joint_pos_des  =  default_joint_pos + action
kp=30, kd=1, tau_ff=0
```

---

## 目录结构

```
Lite3_highlevel_sdk/
├── bridge/                          # 通信协议（语言无关）
│   ├── network_codes.hpp            #   C++ packet struct
│   └── network_codes.py             #   Python struct format
├── python/lite3_highlevel/          # Python API
│   ├── bridge.py                    #   UDP 通信
│   ├── observation.py               #   ObsBuilder + CmdProcessor
│   ├── onnx_runner.py               #   ONNX Runtime 封装
│   └── __init__.py
├── cpp/                             # C++ API
│   ├── include/lite3_highlevel/
│   │   ├── types.hpp                #   数据类型 + PolicyBase
│   │   ├── bridge.hpp               #   UDP 通信
│   │   ├── observation.hpp          #   ObsBuilder + CmdProcessor
│   │   └── onnx_runner.hpp          #   ONNX Runner (PIMPL)
│   └── src/
│       └── onnx_runner.cpp          #   ONNX 实现
├── examples/
│   ├── simple_controller.py
│   └── simple_controller.cpp
├── CMakeLists.txt                   # C++ 构建 (ONNX 可选)
├── CLAUDE.md
└── README.md
```

---

## 与 Lowlevel SDK 的关系

| | Lowlevel SDK | Highlevel SDK（本仓库） |
|---|---|---|
| 运行位置 | RK3588（运动主机） | Jetson NX / PC |
| 职责 | 状态机、硬件、PD、安全 | 策略推理、数据转换 |
| 通信 | 通过 MotionSDK 与电机通信 | 通过 UDP 与 lowlevel SDK 通信 |
| 语言 | C++ | Python + C++ |

通信协议定义在 `bridge/network_codes.*`，两端需保持同步。

---

## 许可

Copyright (c) 2024 DeepRobotics.
