import time
from mav import MavlinkVehicle

def wait_until_armable(vehicle, timeout=60):
    """
    Wait until the vehicle is armable (EKF + GPS OK).
    """
    print("[INFO] Waiting for vehicle to become armable...")
    start = time.time()
    while time.time() - start < timeout:
        vehicle.update_state()
        gps_fix = vehicle.state.get("gps_fix_type", 0)
        sats = vehicle.state.get("satellites_visible", 0)
        is_armable = vehicle.state.get("is_armable", False)
        
        print(f"[INFO] GPS Fix: {gps_fix}, Satellites: {sats}, Armable: {is_armable}")
        
        if is_armable and gps_fix >= 3 and sats >= 6:
            print("[INFO] Vehicle is armable with good GPS.")
            return True
            
        print("[WARN] Waiting for GPS lock and EKF... (need GPS fix ≥3 and ≥6 sats)")
        time.sleep(2)
    
    print("[ERROR] Timeout: Vehicle not armable within timeout period.")
    return False

def wait_until_armed(vehicle, timeout=20):
    """
    Wait until the vehicle reports 'armed'.
    """
    print("[INFO] Waiting for vehicle to arm...")
    start = time.time()
    while time.time() - start < timeout:
        vehicle.update_state()
        if vehicle.state.get("armed", False):
            print("[INFO] Vehicle is armed!")
            return True
        print("[INFO] Still arming...")
        time.sleep(1)
    
    print("[ERROR] Timeout: Vehicle did not arm within timeout period.")
    return False

def wait_for_takeoff(vehicle, target_altitude, timeout=60):
    """
    Wait for the vehicle to reach the target altitude.
    """
    print(f"[INFO] Waiting for takeoff to {target_altitude}m...")
    start = time.time()
    
    while time.time() - start < timeout:
        vehicle.update_state()
        current_alt = vehicle.state.get("relative_alt", 0) or 0
        mode = vehicle.state.get("mode", "UNKNOWN")
        
        print(f"[INFO] Current altitude: {current_alt:.2f}m, Mode: {mode}")
        
        # Consider takeoff complete when we're within 1m of target
        if current_alt >= (target_altitude - 1.0):
            print(f"[INFO] Takeoff complete! Reached {current_alt:.2f}m")
            return True
            
        time.sleep(2)
    
    print("[ERROR] Timeout: Takeoff did not complete within timeout period.")
    return False

def wait_for_landing(vehicle, timeout=120):
    """
    Wait for the vehicle to land and disarm.
    """
    print("[INFO] Waiting for landing...")
    start = time.time()
    
    while time.time() - start < timeout:
        vehicle.update_state()
        current_alt = vehicle.state.get("relative_alt", 0) or 0
        armed = vehicle.state.get("armed", True)
        mode = vehicle.state.get("mode", "UNKNOWN")
        
        print(f"[INFO] Altitude: {current_alt:.2f}m, Armed: {armed}, Mode: {mode}")
        
        # Landing is complete when altitude is very low and vehicle disarms
        if current_alt < 0.5 and not armed:
            print("[INFO] Landing complete and vehicle disarmed!")
            return True
        elif current_alt < 0.5:
            print("[INFO] Landed but still armed, waiting for disarm...")
        
        time.sleep(2)
    
    print("[WARN] Landing timeout reached.")
    return False

def main():
    print("[INFO] Starting autonomous flight script...")
    print("[INFO] This script will: ARM → TAKEOFF to 10m → LAND")
    
    # Connection string - adjust as needed for your setup
    # Common options: '/dev/ttyUSB0', '/dev/ttyTHS0', '/dev/ttyACM0'
    connection_string = '/dev/ttyUSB0'
    baud_rate = 57600
    
    print(f"[INFO] Connecting to vehicle on {connection_string} at {baud_rate} baud...")

    try:
        # Create vehicle connection
        vehicle = MavlinkVehicle(connection_string, baud_rate)
        
        # Initial state update
        vehicle.update_state()
        print("[INFO] Connection established, checking initial status...")
        vehicle.print_detailed_state()
        
        # Step 1: Wait until vehicle is armable
        if not wait_until_armable(vehicle, timeout=120):
            print("[ERROR] Vehicle not armable. Check GPS and EKF status.")
            vehicle.close()
            return False

        # Step 2: Arm the vehicle
        print("[INFO] Attempting to arm the vehicle...")
        vehicle.send_command("arm")
        
        if not wait_until_armed(vehicle, timeout=30):
            print("[ERROR] Failed to arm vehicle.")
            vehicle.close()
            return False

        # Step 3: Set mode to GUIDED and takeoff
        print("[INFO] Setting mode to GUIDED for takeoff...")
        # The takeoff command in mav.py already sets GUIDED mode
        target_altitude = 10
        
        print(f"[INFO] Initiating takeoff to {target_altitude} meters...")
        vehicle.send_command(f"takeoff {target_altitude}")
        
        # Wait for takeoff to complete
        if not wait_for_takeoff(vehicle, target_altitude, timeout=90):
            print("[ERROR] Takeoff failed or timed out.")
            print("[INFO] Attempting emergency land...")
            vehicle.send_command("land")
            vehicle.close()
            return False

        # Step 4: Hold altitude for a few seconds
        print("[INFO] Holding altitude for 10 seconds...")
        for i in range(10):
            vehicle.update_state()
            alt = vehicle.state.get("relative_alt", 0) or 0
            print(f"[INFO] Holding at {alt:.2f}m... ({10-i}s remaining)")
            time.sleep(1)

        # Step 5: Land
        print("[INFO] Initiating landing sequence...")
        vehicle.send_command("land")
        
        if not wait_for_landing(vehicle, timeout=120):
            print("[WARN] Landing sequence timed out, but continuing...")

        # Final status check
        print("[INFO] Final status check...")
        vehicle.update_state()
        vehicle.print_detailed_state()

        print("[INFO] ✅ Autonomous flight mission complete!")
        return True

    except Exception as e:
        print(f"[ERROR] Flight script failed: {e}")
        return False
    
    finally:
        # Always close the connection
        if 'vehicle' in locals():
            vehicle.close()
            print("[INFO] Connection closed.")

if __name__ == "__main__":
    success = main()
    if success:
        print("[INFO] 🚁 First autonomous flight completed successfully!")
    else:
        print("[ERROR] ❌ Flight mission failed. Check logs above for details.")