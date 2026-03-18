import serial
import time

class SerialRobotController:
    def __init__(self, port='/dev/ttyACM0', baudrate=2000000):
        self.port = port
        self.baudrate = baudrate
        self.conn = None
        self.connect()

    def connect(self):
        try:
            self.conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)
            print(f"Connected to Teensy on {self.port}")
        except serial.SerialException as e:
            print(f"Failed to connect on {self.port}: {e}")

    def send_joint_angle(self, motor_id, angle_deg, enabled=True, speed=1500):
        if not self.conn or not enabled:
            return
        cmd = f"J,{motor_id},{angle_deg:.2f},{int(speed)}\n"
        self.conn.write(cmd.encode('utf-8'))

    def set_gpio(self, pin, state):
        if not self.conn:
            return
        cmd = f"G,{pin},{state}\n"
        self.conn.write(cmd.encode('utf-8'))

    def read_messages(self):
        messages = []
        if self.conn and self.conn.in_waiting > 0:
            try:
                lines = self.conn.readlines()
                for line in lines:
                    msg = line.decode('utf-8').strip()
                    if msg:
                        messages.append(msg)
            except Exception:
                pass
        return messages

    def shutdown(self):
        if self.conn and self.conn.is_open:
            self.conn.close()

class MKSJoint:
    def __init__(self, controller, motor_id, can_enabled=True):
        self.controller = controller
        self.motor_id = motor_id
        self.can_enabled = can_enabled

    def setup(self):
        if not self.can_enabled or not self.controller.conn:
            return
        # Enable the motor
        cmd_enable = f"E,{self.motor_id},1\n"
        self.controller.conn.write(cmd_enable.encode('utf-8'))
        time.sleep(0.05)
        
        # Zero the motor
        cmd_zero = f"Z,{self.motor_id}\n"
        self.controller.conn.write(cmd_zero.encode('utf-8'))
        time.sleep(0.05)

    def move_to_angle(self, angle_deg, speed=1500):
        self.controller.send_joint_angle(self.motor_id, angle_deg, self.can_enabled, speed)

    def disable(self):
        if not self.can_enabled or not self.controller.conn:
            return
        cmd_disable = f"E,{self.motor_id},0\n"
        self.controller.conn.write(cmd_disable.encode('utf-8'))
        
    def zero_motor(self):
        if not self.can_enabled or not self.controller.conn:
            return
        cmd_zero = f"Z,{self.motor_id}\n"
        self.controller.conn.write(cmd_zero.encode('utf-8'))