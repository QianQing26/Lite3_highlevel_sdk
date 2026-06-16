"""
Shared binary packet definitions for Lite3 lowlevel ↔ highlevel SDK communication.

Mirrors the C++ structs in Lite3_lowlevel_sdk/communication/nx_network_codes.hpp.
"""
import struct

# --- Port defaults ---
DEFAULT_SENSOR_PORT = 31001   # lowlevel → highlevel
DEFAULT_COMMAND_PORT = 31002  # highlevel → lowlevel

# --- Packet sizes ---
SENSOR_PACKET_SIZE = 204   # !IIi48f
COMMAND_PACKET_SIZE = 248  # !II60f

# --- Format strings ---
_SENSOR_FMT = "!IIi48f"
_COMMAND_FMT = "!II60f"

# Verify at import time
assert struct.calcsize(_SENSOR_FMT) == SENSOR_PACKET_SIZE, \
    f"SensorPacket size mismatch: {struct.calcsize(_SENSOR_FMT)} != {SENSOR_PACKET_SIZE}"
assert struct.calcsize(_COMMAND_FMT) == COMMAND_PACKET_SIZE, \
    f"CommandPacket size mismatch: {struct.calcsize(_COMMAND_FMT)} != {COMMAND_PACKET_SIZE}"

# --- Sentinel values ---
SHUTDOWN_SEQUENCE = 0xFFFFFFFF

# --- Heartbeat values (mirrors nx_network_codes.hpp) ---
HEARTBEAT_SHUTDOWN = 0          # NX sends this heartbeat to stop communication
HEARTBEAT_READY    = 0x0B0B0B0B # NX sends this to signal readiness for RL control

# --- RobotMotionState enum (mirrors types/custom_types.h) ---
# These are the possible values of SensorPacket.current_state.
class RobotMotionState:
    WaitingForStand   = 0  # Idle — waiting for stand-up command
    StandingUp        = 1  # Executing cubic-spline stand-up sequence
    JointDamping      = 2  # Safety fallback — passive joint damping
    RLHandshakeMode   = 5  # Waiting for NX to send HEARTBEAT_READY
    RLControlMode     = 6  # Receiving commands from NX, normal operation

# --- State name constants (mirrors types/custom_types.h) ---
# These are internal state machine identifiers (not sent over the wire).
class StateName:
    kInvalid      = -1
    kIdle         = 0
    kStandUp      = 1
    kJointDamping = 2
    kRLHandshake  = 5
    kRLControl    = 6
