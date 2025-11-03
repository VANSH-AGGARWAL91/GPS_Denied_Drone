#!/usr/bin/env python3
"""
Simple script to test MAVLink connection with drone over USB telemetry
Includes arm/disarm command functionality
"""

from pymavlink import mavutil
import time
import sys

def arm_drone(master, force=False):
    """
    Arm the drone motors

    Args:
        master: MAVLink connection object
        force: Force arming (bypass pre-arm checks) - use with caution
    """
    print("\n" + "="*50)
    print("ARMING DRONE")
    print("="*50)

    # MAV_CMD_COMPONENT_ARM_DISARM command
    # param1: 1 to arm, 0 to disarm
    # param2: 21196 to force arm (bypass pre-arm checks)
    force_param = 21196 if force else 0

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # param1: 1 to arm
        force_param,  # param2: force arm
        0, 0, 0, 0, 0  # params 3-7 (unused)
    )

    print("Arm command sent...")

    # Wait for command acknowledgment
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✓ Arm command ACCEPTED")

            # Verify armed state
            time.sleep(1)
            heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if heartbeat and (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("✓ Drone is ARMED")
                print("\n⚠️  WARNING: Motors can spin! Use caution!")
                return True
            else:
                print("✗ Drone did not arm successfully")
                return False
        else:
            result_codes = {
                0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"
            }
            print(f"✗ Arm command {result_codes.get(ack.result, 'UNKNOWN')}")
            return False
    else:
        print("✗ No acknowledgment received")
        return False

def disarm_drone(master, force=False):
    """
    Disarm the drone motors

    Args:
        master: MAVLink connection object
        force: Force disarm (emergency) - use with caution
    """
    print("\n" + "="*50)
    print("DISARMING DRONE")
    print("="*50)

    # MAV_CMD_COMPONENT_ARM_DISARM command
    # param1: 0 to disarm
    # param2: 21196 to force disarm
    force_param = 21196 if force else 0

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # param1: 0 to disarm
        force_param,  # param2: force disarm
        0, 0, 0, 0, 0  # params 3-7 (unused)
    )

    print("Disarm command sent...")

    # Wait for command acknowledgment
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✓ Disarm command ACCEPTED")

            # Verify disarmed state
            time.sleep(1)
            heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if heartbeat and not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("✓ Drone is DISARMED")
                print("✓ Motors are safe")
                return True
            else:
                print("✗ Drone did not disarm successfully")
                return False
        else:
            result_codes = {
                0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"
            }
            print(f"✗ Disarm command {result_codes.get(ack.result, 'UNKNOWN')}")
            return False
    else:
        print("✗ No acknowledgment received")
        return False

def check_arm_status(master):
    """
    Check if drone is currently armed or disarmed

    Args:
        master: MAVLink connection object
    """
    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if heartbeat:
        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        status = "ARMED ⚠️" if armed else "DISARMED ✓"
        print(f"\nCurrent Status: {status}")
        return armed
    else:
        print("\n✗ Could not determine arm status")
        return None

def test_connection(device='/dev/ttyUSB0', baud=57600, timeout=10):
    """
    Test MAVLink connection to drone

    Args:
        device: Serial device path (e.g., /dev/ttyUSB0, /dev/ttyACM0)
        baud: Baud rate (common: 57600, 115200)
        timeout: Connection timeout in seconds
    """
    print(f"Attempting to connect to {device} at {baud} baud...")

    try:
        # Create MAVLink connection
        master = mavutil.mavlink_connection(device, baud=baud)

        print("Waiting for heartbeat...")
        start_time = time.time()

        # Wait for heartbeat
        master.wait_heartbeat(timeout=timeout)

        elapsed = time.time() - start_time
        print(f"✓ Heartbeat received! (took {elapsed:.2f}s)")
        print(f"  System ID: {master.target_system}")
        print(f"  Component ID: {master.target_component}")
        print(f"  MAVLink version: {master.WIRE_PROTOCOL_VERSION}")

        # Request data streams
        print("\nRequesting data streams...")
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,  # 4 Hz
            1   # Start streaming
        )

        # Wait a moment for data stream to start
        time.sleep(1)

        # Read and display drone status
        print("\n" + "="*50)
        print("DRONE STATUS")
        print("="*50)

        # Initialize data storage
        battery_data = {}
        gps_data = {}
        mode_data = {}

        # Collect messages for 10 seconds
        print("\nCollecting data...")
        end_time = time.time() + 10
        msg_count = 0

        while time.time() < end_time:
            msg = master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_count += 1
                msg_type = msg.get_type()

                # Battery status
                if msg_type == 'SYS_STATUS':
                    battery_data = {
                        'voltage': msg.voltage_battery / 1000.0,  # mV to V
                        'current': msg.current_battery / 100.0 if msg.current_battery != -1 else None,  # cA to A
                        'remaining': msg.battery_remaining,  # percentage
                    }

                # GPS data
                elif msg_type == 'GPS_RAW_INT':
                    gps_data = {
                        'lat': msg.lat / 1e7,
                        'lon': msg.lon / 1e7,
                        'alt': msg.alt / 1000.0,  # mm to m
                        'fix_type': msg.fix_type,
                        'satellites': msg.satellites_visible,
                        'eph': msg.eph / 100.0 if msg.eph != 65535 else None,  # cm to m
                        'epv': msg.epv / 100.0 if msg.epv != 65535 else None,  # cm to m
                    }

                # Flight mode from HEARTBEAT
                elif msg_type == 'HEARTBEAT':
                    mode_data = {
                        'base_mode': msg.base_mode,
                        'custom_mode': msg.custom_mode,
                        'system_status': msg.system_status,
                        'autopilot': msg.autopilot,
                        'type': msg.type,
                    }

        # Display collected data
        print("\n" + "-"*50)
        print("FLIGHT MODE")
        print("-"*50)
        if mode_data:
            # Try to get mode name, handle different autopilot types
            try:
                # For ArduPilot (most common)
                if mode_data.get('autopilot') == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                    mode_mapping = master.mode_mapping()
                    if mode_mapping:
                        mode_name = [k for k, v in mode_mapping.items() if v == mode_data.get('custom_mode')]
                        mode_name = mode_name[0] if mode_name else f"UNKNOWN ({mode_data.get('custom_mode')})"
                    else:
                        mode_name = f"Mode {mode_data.get('custom_mode')}"
                else:
                    mode_name = f"Mode {mode_data.get('custom_mode')}"
            except:
                mode_name = f"Mode {mode_data.get('custom_mode')}"

            autopilot_types = {
                0: "Generic", 1: "Reserved", 2: "SLUGS", 3: "ArduPilot",
                4: "OpenPilot", 5: "Generic Waypoint Only", 6: "Generic Waypoint & Simple Nav",
                7: "Generic Mission Full", 8: "Invalid", 12: "PX4", 13: "SmartAP"
            }
            vehicle_types = {
                0: "Generic", 1: "Fixed Wing", 2: "Quadrotor", 3: "Coaxial",
                4: "Helicopter", 5: "Antenna Tracker", 6: "GCS", 7: "Airship",
                8: "Free Balloon", 9: "Rocket", 10: "Ground Rover", 11: "Surface Boat",
                12: "Submarine", 13: "Hexarotor", 14: "Octorotor", 15: "Tricopter"
            }

            print(f"  Mode: {mode_name}")
            print(f"  Custom mode: {mode_data.get('custom_mode', 'N/A')}")
            print(f"  Base mode: {mode_data.get('base_mode', 'N/A')}")
            print(f"  Autopilot: {autopilot_types.get(mode_data.get('autopilot'), 'Unknown')}")
            print(f"  Vehicle type: {vehicle_types.get(mode_data.get('type'), 'Unknown')}")
            print(f"  System status: {mode_data.get('system_status', 'N/A')}")
        else:
            print("  No mode data received")

        print("\n" + "-"*50)
        print("BATTERY STATUS")
        print("-"*50)
        if battery_data:
            print(f"  Voltage: {battery_data.get('voltage', 'N/A'):.2f} V")
            if battery_data.get('current') is not None:
                print(f"  Current: {battery_data.get('current'):.2f} A")
            else:
                print(f"  Current: N/A")
            print(f"  Remaining: {battery_data.get('remaining', 'N/A')} %")
        else:
            print("  No battery data received")

        print("\n" + "-"*50)
        print("GPS STATUS")
        print("-"*50)
        if gps_data:
            fix_types = {0: "No fix", 1: "No fix", 2: "2D fix", 3: "3D fix",
                        4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
            fix_type = gps_data.get('fix_type', 0)
            print(f"  Fix Type: {fix_types.get(fix_type, 'Unknown')} ({fix_type})")
            print(f"  Satellites: {gps_data.get('satellites', 'N/A')}")
            print(f"  Latitude: {gps_data.get('lat', 'N/A'):.7f}°")
            print(f"  Longitude: {gps_data.get('lon', 'N/A'):.7f}°")
            print(f"  Altitude: {gps_data.get('alt', 'N/A'):.2f} m")
            if gps_data.get('eph') is not None:
                print(f"  Horizontal accuracy: {gps_data.get('eph'):.2f} m")
            if gps_data.get('epv') is not None:
                print(f"  Vertical accuracy: {gps_data.get('epv'):.2f} m")
        else:
            print("  No GPS data received")

        print("\n" + "="*50)
        print(f"✓ Connection stable! Received {msg_count} messages total")
        print("="*50)
        print("\n--- Connection Test PASSED ---")
        return True

    except Exception as e:
        print(f"\n✗ Connection failed: {e}")
        print("\n--- Connection Test FAILED ---")
        print("\nTroubleshooting tips:")
        print("  1. Check device path (try: ls /dev/ttyUSB* or /dev/ttyACM*)")
        print("  2. Verify baud rate matches your telemetry settings")
        print("  3. Ensure user has permission (sudo usermod -a -G dialout $USER)")
        print("  4. Check physical USB connection")
        return False

def interactive_mode(device='/dev/ttyUSB0', baud=57600):
    """
    Interactive mode for testing arm/disarm commands
    """
    print("=" * 50)
    print("MAVLink Interactive Command Mode")
    print("=" * 50)
    print(f"Connecting to {device} at {baud} baud...")

    try:
        # Create MAVLink connection
        master = mavutil.mavlink_connection(device, baud=baud)

        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)

        print(f"✓ Connected!")
        print(f"  System ID: {master.target_system}")
        print(f"  Component ID: {master.target_component}")

        while True:
            print("\n" + "=" * 50)
            print("COMMANDS")
            print("=" * 50)
            print("  1. Check arm status")
            print("  2. Arm drone")
            print("  3. Disarm drone")
            print("  4. Force arm (bypass pre-arm checks)")
            print("  5. Force disarm (emergency)")
            print("  6. Read drone data")
            print("  0. Exit")
            print("=" * 50)

            try:
                choice = input("\nEnter command: ").strip()
            except (KeyboardInterrupt, EOFError):
                print("\n\nExiting...")
                break

            if choice == '1':
                check_arm_status(master)

            elif choice == '2':
                print("\n⚠️  WARNING: This will arm the drone! Motors can spin!")
                confirm = input("Type 'YES' to confirm: ").strip()
                if confirm == 'YES':
                    arm_drone(master, force=False)
                else:
                    print("Cancelled.")

            elif choice == '3':
                disarm_drone(master, force=False)

            elif choice == '4':
                print("\n⚠️  DANGER: Force arming bypasses safety checks!")
                print("Only use if you understand the risks!")
                confirm = input("Type 'FORCE' to confirm: ").strip()
                if confirm == 'FORCE':
                    arm_drone(master, force=True)
                else:
                    print("Cancelled.")

            elif choice == '5':
                print("\n⚠️  Emergency force disarm!")
                confirm = input("Type 'YES' to confirm: ").strip()
                if confirm == 'YES':
                    disarm_drone(master, force=True)
                else:
                    print("Cancelled.")

            elif choice == '6':
                # Quick status read
                print("\nReading drone data...")
                master.mav.request_data_stream_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    4, 1
                )

                battery_data = {}
                gps_data = {}
                mode_data = {}

                end_time = time.time() + 3
                while time.time() < end_time:
                    msg = master.recv_match(blocking=True, timeout=1)
                    if msg:
                        msg_type = msg.get_type()
                        if msg_type == 'SYS_STATUS':
                            battery_data = {
                                'voltage': msg.voltage_battery / 1000.0,
                                'remaining': msg.battery_remaining,
                            }
                        elif msg_type == 'GPS_RAW_INT':
                            gps_data = {
                                'fix_type': msg.fix_type,
                                'satellites': msg.satellites_visible,
                            }
                        elif msg_type == 'HEARTBEAT':
                            mode_data = {
                                'custom_mode': msg.custom_mode,
                                'base_mode': msg.base_mode,
                            }

                print("\nQuick Status:")
                if mode_data:
                    armed = bool(mode_data.get('base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    print(f"  Armed: {'YES ⚠️' if armed else 'NO ✓'}")
                if battery_data:
                    print(f"  Battery: {battery_data.get('voltage', 'N/A'):.1f}V ({battery_data.get('remaining', 'N/A')}%)")
                if gps_data:
                    fix_types = {0: "No fix", 1: "No fix", 2: "2D", 3: "3D"}
                    print(f"  GPS: {fix_types.get(gps_data.get('fix_type', 0), 'Unknown')} - {gps_data.get('satellites', 0)} sats")

            elif choice == '0':
                print("\nExiting...")
                break

            else:
                print("\n✗ Invalid choice. Please try again.")

        print("\n✓ Disconnected")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False

    return True

if __name__ == "__main__":
    # You can modify these defaults or pass as command line arguments
    device = '/dev/ttyUSB0'  # Change to /dev/ttyACM0 or other if needed
    baud = 57600             # Change to 115200 if needed

    # Parse command line arguments
    mode = 'test'  # default mode
    if len(sys.argv) > 1:
        if sys.argv[1] in ['interactive', 'i', 'commands', 'cmd']:
            mode = 'interactive'
        elif sys.argv[1].startswith('/dev/'):
            device = sys.argv[1]
        else:
            print("Usage:")
            print("  python3 test_mavlink_connection.py                    # Run connection test")
            print("  python3 test_mavlink_connection.py interactive        # Interactive command mode")
            print("  python3 test_mavlink_connection.py /dev/ttyUSB0 57600 # Specify device/baud")
            sys.exit(1)

    if len(sys.argv) > 2 and sys.argv[2].isdigit():
        baud = int(sys.argv[2])

    if mode == 'interactive':
        success = interactive_mode(device, baud)
    else:
        print("=" * 50)
        print("MAVLink Connection Test")
        print("=" * 50)
        success = test_connection(device, baud)

    sys.exit(0 if success else 1)
