import time
import sys
import math
from pymavlink import mavutil
import serial.tools.list_ports

class MavlinkVehicle:
    """
    A wrapper class for MAVLink communication to simplify drone interaction.
    """
    def __init__(self, connection_string='/dev/ttyUSB0', baud_rate=57600):
        """
        Initializes the vehicle and connects to it.
        Raises Exception on connection failure.
        """
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.master = None
        self.state = {
            'gps_fix_type': None,
            'satellites_visible': None,
            'lat': None,
            'lon': None,
            'alt': None,
            'relative_alt': None,
            'voltage': None,
            'current': None,
            'level': None,
            'last_heartbeat': time.time(),
            'armed': False,
            'mode': 'UNKNOWN',
            'system_status': 'UNKNOWN',
            'yaw': None,
            'is_armable': False,
        }
        
        # Establish connection
        self._connect()

    @staticmethod
    def list_available_ports():
        """List all available COM ports"""
        print("Available COM ports:")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  {port.device} - {port.description}")
        return ports

    def _connect(self):
        """Establish connection to the vehicle"""
        try:
            print(f"[INFO] Connecting to vehicle on: {self.connection_string} at {self.baud_rate} baud...")
            
            # Create the connection
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baud_rate)
            
            # Wait for the first heartbeat
            print("[INFO] Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)  # 10 second timeout
            print(f"[INFO] Heartbeat received from system {self.master.target_system} component {self.master.target_component}")
            
            # Initial state update
            self.update_state()
            
        except Exception as e:
            print(f"[ERROR] Failed to connect to vehicle: {e}")
            print("\nTroubleshooting steps:")
            print("1. Ensure the vehicle is powered on and connected to your computer")
            print(f"2. Check if {self.connection_string} is the correct port")
            print("3. Try running this script as Administrator")
            print("4. Close any other applications that might be using the COM port")
            print("5. Try different baud rates: 57600, 115200, 9600")
            print("6. Check device manager to confirm the COM port number")
            raise

    def update_state(self):
        """
        Updates the internal state dictionary with the latest telemetry.
        Processes any pending MAVLink messages.
        """
        if not self.master:
            return
            
        try:
            # Process multiple messages to get latest data
            messages_processed = 0
            while messages_processed < 20:  # Process up to 20 messages per update
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break
                    
                messages_processed += 1
                msg_type = msg.get_type()
                
                if msg_type == 'HEARTBEAT':
                    self.state['last_heartbeat'] = time.time()
                    self.state['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    self.state['mode'] = mavutil.mode_string_v10(msg)
                    if hasattr(mavutil.mavlink.enums, 'MAV_STATE') and msg.system_status in mavutil.mavlink.enums['MAV_STATE']:
                        self.state['system_status'] = mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name
                    else:
                        self.state['system_status'] = f"STATUS_{msg.system_status}"
                        
                elif msg_type == 'SYS_STATUS':
                    self.state['voltage'] = msg.voltage_battery / 1000.0 if msg.voltage_battery > 0 else None
                    self.state['current'] = msg.current_battery / 100.0 if msg.current_battery != -1 else None
                    self.state['level'] = msg.battery_remaining if msg.battery_remaining != -1 else None
                    
                elif msg_type == 'GPS_RAW_INT':
                    self.state['gps_fix_type'] = msg.fix_type
                    self.state['satellites_visible'] = msg.satellites_visible
                    if msg.lat != 0 and msg.lon != 0:  # Valid coordinates
                        self.state['lat'] = msg.lat / 1e7
                        self.state['lon'] = msg.lon / 1e7
                    if msg.alt != 0:
                        self.state['alt'] = msg.alt / 1e3  # Altitude in meters (MSL)
                    
                elif msg_type == 'GLOBAL_POSITION_INT':
                    if msg.relative_alt != 0:
                        self.state['relative_alt'] = msg.relative_alt / 1e3  # Altitude above home in meters
                    
                elif msg_type == 'ATTITUDE':
                    self.state['yaw'] = msg.yaw  # in radians
                    
                elif msg_type == 'EKF_STATUS_REPORT':
                    # A simplified check for armability based on EKF flags
                    try:
                        flags = msg.flags
                        pos_horiz_abs_ok = (flags & mavutil.mavlink.EKF_POS_HORIZ_ABS) > 0
                        pred_pos_horiz_abs_ok = (flags & mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS) > 0
                        gps_ok = self.state['gps_fix_type'] is not None and self.state['gps_fix_type'] > 1
                        self.state['is_armable'] = (pos_horiz_abs_ok or pred_pos_horiz_abs_ok) and gps_ok
                    except:
                        pass  # If EKF flags processing fails, skip it
                        
        except Exception as e:
            # Don't print errors during normal operation, just skip the update
            pass

    def send_command(self, command_str):
        """
        Parses and sends a command to the vehicle.
        """
        if not self.master:
            print("[ERROR] No connection to vehicle")
            return
            
        cmd = command_str.lower().strip()
        print(f"\n[CMD] Received: '{cmd}'")

        try:
            if cmd == 'arm':
                if not self.state.get('is_armable', False):
                    print("[WARN] Vehicle may not be armable. Check GPS and EKF status.")
                print("[CMD] Arming vehicle...")
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1, 0, 0, 0, 0, 0, 0  # ARM command
                )
                print("[INFO] Arm command sent")

            elif cmd == 'disarm':
                print("[CMD] Disarming vehicle...")
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0, 0, 0, 0, 0, 0, 0  # DISARM command
                )
                print("[INFO] Disarm command sent")

            elif cmd.startswith('takeoff'):
                parts = cmd.split()
                target_altitude = 10  # default altitude
                if len(parts) >= 2:
                    try:
                        target_altitude = float(parts[1])
                    except ValueError:
                        print("[ERROR] Invalid altitude for takeoff, using default 10m")
                        
                if not self.state.get('armed', False):
                    print("[WARN] Vehicle must be armed before takeoff.")
                    return
                # Set mode to GUIDED (required for takeoff in ArduPilot)
                print("[CMD] Setting mode to GUIDED...")
                self.master.set_mode_apm('GUIDED')
                time.sleep(2)  # give some time to switch mode
                    
                print(f"[CMD] Taking off to {target_altitude} meters...")
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0, 0, 0, 0, 0, 0, target_altitude
                )
                print(f"[INFO] Takeoff command sent to {target_altitude} meters")

            elif cmd == 'land':
                print("[CMD] Landing...")
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )
                print("[INFO] Land command sent")

            elif cmd == 'rtl':
                print("[CMD] Returning to launch...")
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )
                print("[INFO] RTL command sent")

            elif cmd == 'status':
                self.print_detailed_state()
                
            elif cmd == 'test':
                self.test_connection()

            else:
                print(f"[WARN] Unknown command: '{cmd}'")
                print("[INFO] Available commands: arm, disarm, takeoff [altitude], land, rtl, status, test")
                
        except Exception as e:
            print(f"[ERROR] Failed to execute command '{cmd}': {e}")

    def print_detailed_state(self):
        """Prints a detailed state of the vehicle."""
        print("\n--- Vehicle State ---")
        
        gps_fix = self.state['gps_fix_type']
        sats = self.state['satellites_visible']
        print(f" GPS Fix: {gps_fix}, Sats: {sats}")

        lat, lon = self.state['lat'], self.state['lon']
        alt, rel_alt = self.state['alt'], self.state['relative_alt']
        lat_str = f"{lat:.7f}" if lat is not None else "N/A"
        lon_str = f"{lon:.7f}" if lon is not None else "N/A"
        alt_str = f"{alt:.2f}m" if alt is not None else "N/A"
        rel_alt_str = f"{rel_alt:.2f}m" if rel_alt is not None else "N/A"
        print(f" Location: Lat={lat_str}, Lon={lon_str}, Alt(MSL)={alt_str}, RelAlt={rel_alt_str}")

        voltage, current, level = self.state['voltage'], self.state['current'], self.state['level']
        batt_str = f"{voltage:.2f}V" if voltage is not None else "N/A"
        batt_str += f", {current:.2f}A" if current is not None else ", N/A"
        batt_str += f", {level}%" if level is not None else ", N/A"
        print(f" Battery: {batt_str}")
        
        print(f" Armed: {self.state['armed']}, Mode: {self.state['mode']}")
        print(f" System Status: {self.state['system_status']}")

        yaw_rad = self.state['yaw']
        yaw_deg = math.degrees(yaw_rad) if yaw_rad is not None else None
        yaw_str = f"{yaw_deg:.2f}°" if yaw_deg is not None else "N/A"
        print(f" Yaw: {yaw_str}")
        
        print(f" Is Armable: {self.state['is_armable']}")
        
        last_hb = time.time() - self.state['last_heartbeat']
        print(f" Last Heartbeat: {last_hb:.2f}s ago")
        print("---------------------\n")

    def test_connection(self):
        """Test the connection by requesting a few messages"""
        if not self.master:
            print("[ERROR] No connection to test")
            return
            
        print("\n[TEST] Testing connection...")
        
        # Try to get a fresh heartbeat
        try:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                print(f"[TEST] ✓ Heartbeat received from system {msg.get_srcSystem()}")
                self.state['last_heartbeat'] = time.time()
            else:
                print("[TEST] ✗ No heartbeat received within 5 seconds")
        except Exception as e:
            print(f"[TEST] ✗ Heartbeat test failed: {e}")
            
        # Test message flow
        print("[TEST] Checking message flow for 3 seconds...")
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < 3:
            msg = self.master.recv_match(blocking=False)
            if msg:
                message_count += 1
            time.sleep(0.01)
                
        print(f"[TEST] Received {message_count} messages in 3 seconds")
        if message_count > 0:
            print("[TEST] ✓ Message flow is working")
        else:
            print("[TEST] ✗ No messages received - connection may be stale")
            
        print("[TEST] Connection test complete\n")

    def close(self):
        """
        Closes the connection to the vehicle.
        """
        if self.master:
            self.master.close()
            self.master = None
            print("[INFO] MAVLink connection closed")