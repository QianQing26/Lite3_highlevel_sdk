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

# --- Sentinel ---
SHUTDOWN_SEQUENCE = 0xFFFFFFFF
