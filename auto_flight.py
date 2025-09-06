import time
from mav import MavlinkVehicle

def wait_until_armable(vehicle, timeout=30):
    """
    Wait until the vehicle is armable (EKF + GPS OK).
    """
    print("[INFO] Waiting for vehicle to become armable...")
    start = time.time()
    while time.time() - start < timeout:
        vehicle.update_state()
        if vehicle.state.get("is_armable", False):
            print("[INFO] Vehicle is armable.")
            return True
        print("[WARN] Not armable yet... (GPS/EKF)")
        time.sleep(1)
    print("[ERROR] Timeout: Vehicle not armable.")
    return False

def wait_until_armed(vehicle, timeout=15):
    """
    Wait until the vehicle reports 'armed'.
    """
    print("[INFO] Waiting for vehicle to arm...")
    start = time.time()
    while time.time() - start < timeout:
        vehicle.update_state()
        if vehicle.state.get("armed", False):
            print("[INFO] Vehicle is armed.")
            return True
        time.sleep(1)
    print("[ERROR] Timeout: Vehicle did not arm.")
    return False

def main():
    print("[INFO] Starting autonomous script...")

    # Adjust port: /dev/ttyTHS1 (Jetson UART) or /dev/ttyUSB0 (USB)
    vehicle = MavlinkVehicle('/dev/ttyTHS0', 57600)

    # Wait until vehicle is armable
    if not wait_until_armable(vehicle):
        vehicle.close()
        return

    # Arm the vehicle
    vehicle.send_command("arm")
    if not wait_until_armed(vehicle):
        vehicle.close()
        return

    # Take off to 10 meters
    vehicle.send_command("takeoff 10")
    time.sleep(30)  # Give it time to climb

    # Land
    vehicle.send_command("land")
    time.sleep(15)

    # Close connection
    vehicle.close()
    print("[INFO] Mission complete.")

if __name__ == "__main__":
    main()
