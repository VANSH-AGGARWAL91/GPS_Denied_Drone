# interactive_control.py
import sys
import time
import threading
from mav import MavlinkVehicle

# Use a threading Event to signal when the telemetry loop should stop
stop_thread_event = threading.Event()

def telemetry_loop(vehicle):
    """
    This function runs in a separate thread and continuously updates and prints
    the vehicle's telemetry data.
    """
    while not stop_thread_event.is_set():
        try:
            vehicle.update_state()
            
            state = vehicle.state
            # Use carriage return '\r' to print on the same line
            # Handle None values safely
            armed = str(state.get('armed', 'Unknown'))
            mode = str(state.get('mode', 'UNKNOWN'))
            gps_fix = state.get('gps_fix_type', 'N/A')
            sats = state.get('satellites_visible', 'N/A')
            alt = state.get('relative_alt', 0.0) or 0.0
            voltage = state.get('voltage', 0.0) or 0.0
            level = state.get('level', 0) or 0
            status = str(state.get('system_status', 'UNKNOWN'))
            
            status_line = (
                f"\rArmed: {armed:<5} | "
                f"Mode: {mode:<12} | "
                f"GPS: {gps_fix}/{sats} sats | "
                f"Alt: {alt:.2f}m | "
                f"Batt: {voltage:.2f}V ({level}%) | "
                f"Status: {status}"
            )
            print(status_line, end="", flush=True)
            
        except Exception as e:
            print(f"\rTelemetry error: {e}", end="", flush=True)
        
        time.sleep(0.2) # Update 5 times a second

def main():
    """
    Main function to run the interactive console.
    """
    print("[INFO] Starting interactive MAVLink console...")
    
    # Show available ports first
    print("\n" + "="*50)
    MavlinkVehicle.list_available_ports()
    print("="*50)
    
    # Get connection parameters from user or use defaults
    connection_string = input("\nEnter COM port (press Enter for /dev/ttyUSB0'): ").strip()
    if not connection_string:
        connection_string = '/dev/ttyUSB0'
    
    baud_input = input("Enter baud rate (press Enter for 57600): ").strip()
    if not baud_input:
        baud_rate = 57600
    else:
        try:
            baud_rate = int(baud_input)
        except ValueError:
            print("[WARN] Invalid baud rate, using 57600")
            baud_rate = 57600

    # 1. Create a vehicle instance, handling potential connection errors.
    vehicle = None
    try:
        vehicle = MavlinkVehicle(connection_string, baud_rate)
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        print("[INFO] Exiting due to connection failure.")
        sys.exit(1)

    if not vehicle or not vehicle.master:
        print("[ERROR] Could not establish a valid connection. Exiting.")
        sys.exit(1)

    # 2. Start the telemetry thread
    telemetry_thread = threading.Thread(target=telemetry_loop, args=(vehicle,))
    telemetry_thread.daemon = True  # Dies when main thread dies
    telemetry_thread.start()
    
    print("\n[INFO] Connection established. Telemetry running.")
    print("[INFO] Available commands:")
    print("  - arm: Arm the vehicle")
    print("  - disarm: Disarm the vehicle") 
    print("  - takeoff [altitude]: Take off to specified altitude (default: 10m)")
    print("  - land: Land the vehicle")
    print("  - rtl: Return to launch")
    print("  - status: Show detailed vehicle status")
    print("  - test: Test connection and message flow")
    print("  - exit: Exit the program")
    print("\nType commands below:")

    try:
        while True:
            # 3. Wait for user input
            # Add a newline and prompt to avoid being overwritten by telemetry
            command = input("\n> ")
            
            if command.lower() == 'exit':
                break
            elif command.strip():
                vehicle.send_command(command)
            else:
                # Empty command, just continue
                continue
                
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt detected.")
    
    finally:
        # 4. Cleanly shut down
        print("\n[INFO] Shutting down...")
        stop_thread_event.set() # Signal the telemetry thread to exit
        if telemetry_thread.is_alive():
            telemetry_thread.join(timeout=2) # Wait up to 2 seconds for the thread to finish
        vehicle.close()
        print("[INFO] Connection closed. Goodbye.")

if __name__ == "__main__":
    main()