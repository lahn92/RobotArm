import serial
import time

class SerialRobotController:
    def __init__(self, port='/dev/ttyACM0', baudrate=2000000):
        self.port = port
        self.baudrate = baudrate
        self.conn = None
        
    def connect(self):
        try:
            # Connect to Teensy
            self.conn = serial.Serial(self.port, self.baudrate, timeout=0)
            time.sleep(1) # Wait for microcontroller to initialize
            print(f"Connected to Teensy on {self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to Teensy: {e}")
            return False

    def send_joint_angle(self, motor_id, angle_deg, enabled=True, speed=1500):
        if not self.conn or not enabled:
            return
        cmd = f"J,{motor_id},{angle_deg:.2f},{int(speed)}\n"
        self.conn.write(cmd.encode('utf-8'))


    def set_gpio(self, pin, state):
        """Easily expandable for future grippers/LEDs"""
        if not self.conn: return
        # Format: G,<pin>,<state>
        # Example: G,13,1\n
        cmd = f"G,{pin},{int(state)}\n"
        self.conn.write(cmd.encode('utf-8'))
        
    def shutdown(self):
        if self.conn:
            self.conn.close()

# Keep a wrapper to match your GUI's current expectations seamlessly
class MKSJoint:
    def __init__(self, controller, motor_id, can_enabled=True):
        self.controller = controller
        self.motor_id = motor_id
        self.can_enabled = can_enabled
        
    def setup(self):
        if not self.can_enabled or not self.controller.conn:
            return
            
        # Send E,<id>,1 to Enable the motor
        cmd_enable = f"E,{self.motor_id},1\n"
        self.controller.conn.write(cmd_enable.encode('utf-8'))
        time.sleep(0.05)
        
        # Send Z,<id> to set the current physical position as 0.0
        cmd_zero = f"Z,{self.motor_id}\n"
        self.controller.conn.write(cmd_zero.encode('utf-8'))
        time.sleep(0.05)

    def move_to_angle(self, angle_deg, speed=1500):
        self.controller.send_joint_angle(self.motor_id, angle_deg, self.can_enabled, speed)

    def disable(self):
        if not self.can_enabled or not self.controller.conn:
            return
        # Send E,<id>,0 to Disable the motor
        cmd_disable = f"E,{self.motor_id},0\n"
        self.controller.conn.write(cmd_disable.encode('utf-8'))