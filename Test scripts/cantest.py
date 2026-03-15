import can
import time
import sys

# --- CONFIGURATION ---
COM_PORT = 'COM4'      
BITRATE = 500000       
MOTOR_ID = 0x01        

def calculate_checksum(data_list):
    """Calculates the MKS CAN checksum: (ID + sum(Data)) & 0xFF"""
    return (MOTOR_ID + sum(data_list)) & 0xFF

def run_and_monitor_speed():
    try:
        bus = can.interface.Bus(interface='slcan', channel=COM_PORT, bitrate=BITRATE)
        print(f"Connected to {COM_PORT} at {BITRATE}bps")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    def send_command(data_list, description="", silent=False):
        checksum = calculate_checksum(data_list)
        data_list_with_crc = data_list + [checksum]
        
        data_bytes = bytes(data_list_with_crc)
        msg = can.Message(
            arbitration_id=MOTOR_ID,
            data=data_bytes,
            is_extended_id=False
        )
        try:
            bus.send(msg)
            if not silent:
                print(f"Sent [{description}]: {data_bytes.hex().upper()}")
        except can.CanError:
            if not silent:
                print(f"Failed to send: {description}")

    # ==========================================
    # 1. Initialization
    # ==========================================
    send_command([0x82, 0x05], "Set Bus FOC Mode")
    time.sleep(0.1)

    send_command([0xF3, 0x01], "Enable Motor")
    time.sleep(0.5)

    # ==========================================
    # 2. Spin Up to 3000 RPM
    # ==========================================
    target_speed = 250
    direction = 1       # 0 = Forward, 1 = Reverse
    acc = 240           # FAST acceleration (Max is 255)

    byte2 = (direction << 7) | ((target_speed >> 8) & 0x0F)
    byte3 = target_speed & 0xFF
    byte4 = acc
    
    send_command([0xF6, byte2, byte3, byte4], f"Commanding {target_speed} RPM")
    
    print("\n" + "="*50)
    print(" MOTOR RUNNING - LIVE DASHBOARD")
    print(" >>> PRESS CTRL+C TO STOP <<< ")
    print("="*50 + "\n")
    
    # Pre-calculate the Speed Request Message (Command 0x32)
    # Checksum = ID(0x01) + Cmd(0x32) = 0x33
    speed_req_msg = can.Message(
        arbitration_id=MOTOR_ID, 
        data=bytes([0x32, 0x33]), 
        is_extended_id=False
    )
    
    try:
        # Keep the script alive and poll the speed
        while True:
            # Ask motor for its current speed
            bus.send(speed_req_msg)
            
            # Wait up to 0.1s for the reply
            reply = bus.recv(timeout=0.1)
            
            # Check if it's the valid 0x32 reply (Length should be 4 bytes)
            if reply and len(reply.data) == 4 and reply.data[0] == 0x32:
                # Speed is stored in Byte 1 and Byte 2 as a 16-bit signed integer
                current_speed = int.from_bytes(reply.data[1:3], byteorder='big', signed=True)
                
                # Print dynamically on the same line
                sys.stdout.write(f"\rTarget: {target_speed} RPM  |  Actual: {abs(current_speed):4d} RPM   ")
                sys.stdout.flush()
                
            # Small delay to prevent flooding the CAN bus
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\nStop command received! Initiating smooth deceleration...")
        
    finally:
        # ==========================================
        # 3. Safe Shutdown
        # ==========================================
        # Decelerate to 0 smoothly using the same high acceleration value
        send_command([0xF6, 0x00, 0x00, acc], "Smooth Stop (Speed 0)")
        
        print("Waiting for motor to spin down...")
        time.sleep(2) # Give it 2 seconds to reach 0 RPM
        
        send_command([0xF3, 0x00], "Disable Motor (Free Spin)")
        
        bus.shutdown()
        print("Bus shut down successfully.")

if __name__ == "__main__":
    run_and_monitor_speed()