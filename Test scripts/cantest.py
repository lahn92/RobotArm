import serial
import time
import sys

# --- CONFIGURATION ---
# Change this to match your Teensy's COM port (e.g., '/dev/ttyACM0' for Linux/Pi)
COM_PORT = 'COM7'  
BAUDRATE = 2000000 
MOTOR_ID = 1

def run_speed_mode():
    try:
        # Connect to the Teensy via Serial
        conn = serial.Serial(COM_PORT, BAUDRATE, timeout=0.1)
        time.sleep(2)  # Wait for Teensy to initialize
        print(f"Connected to Teensy on {COM_PORT}")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    def send_command(cmd_str, description=""):
        # Send a formatted string to the Teensy
        conn.write((cmd_str + '\n').encode('utf-8'))
        print(f"Sent [{description}]: {cmd_str}")

    # ==========================================
    # 1. Initialization
    # ==========================================
    send_command(f"E,{MOTOR_ID},1", "Enable Motor")
    time.sleep(0.5)

    # ==========================================
    # 2. Spin Up
    # ==========================================
    target_speed = 5000  # Positive for forward, negative for reverse (e.g., -500)
    acc = 240           # FAST acceleration (Max is 255)

    # Send the new Velocity command format: V,ID,SPEED,ACCEL
    send_command(f"V,{MOTOR_ID},{target_speed},{acc}", f"Commanding {target_speed} RPM")

    print("\n" + "="*50)
    print(" MOTOR RUNNING IN SPEED MODE")
    print(" >>> PRESS CTRL+C TO STOP <<< ")
    print("="*50 + "\n")

    try:
        # Keep the script alive while the motor runs
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStop command received! Initiating smooth deceleration...")
        
    finally:
        # ==========================================
        # 3. Safe Shutdown
        # ==========================================
        # Decelerate to 0 smoothly using the same high acceleration value
        send_command(f"V,{MOTOR_ID},0,{acc}", "Smooth Stop (Speed 0)")
        
        print("Waiting for motor to spin down...")
        time.sleep(2)  # Give it 2 seconds to reach 0 RPM
        
        send_command(f"E,{MOTOR_ID},0", "Disable Motor (Free Spin)")
        
        conn.close()
        print("Serial connection closed.")

if __name__ == "__main__":
    run_speed_mode()