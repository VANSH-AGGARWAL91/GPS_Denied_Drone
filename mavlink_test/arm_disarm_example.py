#!/usr/bin/env python3
"""
Simple example showing how to arm and disarm drone
"""

from pymavlink import mavutil
import time

# Connect to drone
print("Connecting to drone...")
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}")

# Function to check if armed
def is_armed(master):
    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if heartbeat:
        return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None

# Check current status
print(f"\nCurrent status: {'ARMED' if is_armed(master) else 'DISARMED'}")

# ARM THE DRONE
print("\n=== ARMING DRONE ===")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # confirmation
    1,  # param1: 1 = arm
    0,  # param2: 0 = normal arm (21196 = force arm)
    0, 0, 0, 0, 0
)

# Wait for acknowledgment
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack and ack.result == 0:
    print("✓ Arm command accepted")
    time.sleep(1)
    print(f"Status: {'ARMED' if is_armed(master) else 'DISARMED'}")
else:
    print("✗ Arm command failed")

# Wait 3 seconds
print("\nWaiting 3 seconds...")
time.sleep(3)

# DISARM THE DRONE
print("\n=== DISARMING DRONE ===")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # confirmation
    0,  # param1: 0 = disarm
    0,  # param2: 0 = normal disarm (21196 = force disarm)
    0, 0, 0, 0, 0
)

# Wait for acknowledgment
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack and ack.result == 0:
    print("✓ Disarm command accepted")
    time.sleep(1)
    print(f"Status: {'ARMED' if is_armed(master) else 'DISARMED'}")
else:
    print("✗ Disarm command failed")

print("\nDone!")
