# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Lite3 Highlevel SDK is the upper-layer control library for the Lite3 quadruped robot. It runs on the compute side (e.g. Jetson NX) and communicates with `Lite3_lowlevel_sdk` (RK3588) over UDP. The SDK handles UDP communication, observation building, command processing, and ONNX Runtime inference — so the user only writes a policy function.

Provides **both Python and C++ APIs** with the same interface surface.

## Build Commands

### C++ (with ONNX Runtime)

```bash
# Place ONNX Runtime in third_party/onnxruntime/ (auto-detected)
# Or set manually:
cmake .. -DONNXRUNTIME_ROOT=/path/to/onnxruntime

mkdir build && cd build
cmake ..
make -j
```

### C++ (header-only, no ONNX)

```bash
mkdir build && cd build
cmake ..
make -j   # builds lite3_highlevel as INTERFACE (header-only) library
```

### Python

Python does not require compilation. Install dependencies:

```bash
pip install onnxruntime numpy
```

## Architecture

### Communication Layer (`bridge/`)
- `network_codes.hpp` / `network_codes.py`: Binary packet struct definitions — `SensorPacket` (204 B) and `CommandPacket` (248 B). Port 31001 (recv) / 31002 (send). Shared between C++ and Python.

### Python API (`python/lite3_highlevel/`)
- `bridge.py`: `Bridge` class — non-blocking UDP socket drain + fixed-frequency send. Single-threaded `run(policy, frequency_hz)` blocking loop.
- `observation.py`: `ObsBuilder` (45-dim observation vector matching original ONNX policy format) + `CmdProcessor` (action scale + default pose → `JointCommand`).
- `onnx_runner.py`: `OnnxRunner` — wraps `onnxruntime.InferenceSession`, warm-up inference, `infer(obs) -> action`.
- `__init__.py`: Exports all public symbols.

### C++ API (`cpp/`)
- `include/lite3_highlevel/types.hpp`: `RobotState`, `JointCommand`, `PolicyBase` (abstract interface).
- `include/lite3_highlevel/bridge.hpp`: `Bridge` class — non-blocking `recv()`, `sendCommand()`, `sendShutdown()`. Header-only.
- `include/lite3_highlevel/observation.hpp`: `ObsBuilder` + `CmdProcessor`. Config structs (`ObsConfig`, `CmdConfig`) with Lite3 defaults. Header-only.
- `include/lite3_highlevel/onnx_runner.hpp`: `OnnxRunner` — PIMPL pattern, onnxruntime types hidden from users.
- `src/onnx_runner.cpp`: PIMPL implementation. Only compiled when `ONNXRUNTIME_ROOT` is set.

### Examples (`examples/`)
- `simple_controller.py`: End-to-end ONNX policy control loop in ~20 lines.
- `simple_controller.cpp`: Same in C++. Shows `recv → build obs → infer → process → send` loop at 50 Hz.

## Key Files for Common Tasks

| Task | File |
|------|------|
| Change ONNX model path | Pass to `OnnxRunner(model_path)` constructor |
| Tune observation format | `observation.py` `ObsBuilder.__init__` or `observation.hpp` `ObsConfig` |
| Change PD gains | `observation.py` `CmdProcessor.__init__` or `observation.hpp` `CmdConfig` |
| Change control frequency | Pass `frequency_hz` to `bridge.run()` or modify sleep in C++ loop |
| Change robot IP | Pass `robot_ip` to `Bridge(robot_ip)` |

## Relationship to Lite3_lowlevel_sdk

- **lowlevel** runs on RK3588: state machine, hardware interface, 2 kHz PD control, safety protection.
- **highlevel** runs on NX/PC: receives sensor data, runs policy, sends joint commands.
- Communication: UDP binary protocol, ports 31001 (sensor) / 31002 (command), ~50 Hz.
- Packet definitions must stay in sync between the two repos.
