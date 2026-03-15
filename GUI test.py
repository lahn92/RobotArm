import sys
import can
import time
import os
import numpy as np
import pybullet as p
import pybullet_data
from pathlib import Path
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QPushButton, 
                             QLineEdit, QScrollArea, QGridLayout, QFrame)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QDoubleValidator

# --- CONFIGURATION ---
COM_PORT = 'COM4'
BITRATE = 500000
JOG_STEP = 1.0      # Degrees per tick
JOG_INTERVAL = 50   # ms between ticks
JOG_DELAY = 250     # ms before auto-repeat starts

SCRIPT_DIR = Path(__file__).parent
URDF_PATH = str(SCRIPT_DIR / "my-robot" / "robot.urdf")

# --- CONFIGURATION ---
COM_PORT = 'COM4'
BITRATE = 500000
JOG_STEP = 1.0      # Degrees per tick
JOG_INTERVAL = 50   # ms between ticks
JOG_DELAY = 250     # ms before auto-repeat starts

SCRIPT_DIR = Path(__file__).parent
URDF_PATH = str(SCRIPT_DIR / "my-robot" / "robot.urdf")

JOINT_CONFIGS = [
    {"id": 0, "can_id": 0x01, "name": "Joint 0 (Base)", "urdf_link": "base", "invert": True,  "can_enabled": True},
    {"id": 1, "can_id": 0x02, "name": "Joint 1 (Shoulder)", "urdf_link": "shoulder", "invert": False, "can_enabled": False},
    {"id": 2, "can_id": 0x03, "name": "Joint 2 (Elbow)", "urdf_link": "elbow", "invert": False, "can_enabled": False},
    {"id": 3, "can_id": 0x04, "name": "Joint 3 (Rotation)", "urdf_link": "rotation", "invert": False, "can_enabled": False},
    {"id": 4, "can_id": 0x05, "name": "Joint 4 (Wrist)", "urdf_link": "wrist", "invert": False, "can_enabled": False},
    {"id": 5, "can_id": 0x06, "name": "Joint 5 (Swivel)", "urdf_link": "swivel", "invert": False, "can_enabled": False},
]

class MKSJoint:
    def __init__(self, bus, motor_id, can_enabled=True):
        self.bus = bus
        self.motor_id = motor_id
        self.can_enabled = can_enabled
        self.speed = 1500 
        self.accel = 255
        
    def _send(self, data):
        if not self.can_enabled or not self.bus: return
        checksum = (self.motor_id + sum(data)) & 0xFF
        payload = bytes(data + [checksum])
        msg = can.Message(arbitration_id=self.motor_id, data=payload, is_extended_id=False)
        try: self.bus.send(msg)
        except can.CanError: print(f"Failed to send to CAN ID {self.motor_id}")

    def setup(self):
        if not self.can_enabled: return
        self._send([0x82, 0x05])
        time.sleep(0.02)
        self._send([0xF3, 0x01])
        time.sleep(0.02)
        self._send([0x92])

    def move_to_angle(self, angle_deg):
        if not self.can_enabled: return
        ticks = int((angle_deg / 360.0) * 16384)
        try: pos_bytes = ticks.to_bytes(3, byteorder='big', signed=True)
        except OverflowError: return
        speed_h = (self.speed >> 8) & 0xFF
        speed_l = self.speed & 0xFF
        self._send([0xF5, speed_h, speed_l, self.accel, pos_bytes[0], pos_bytes[1], pos_bytes[2]])

    def disable(self):
        if not self.can_enabled: return
        self._send([0xF3, 0x00])

class RobotArmWindow(QMainWindow):
    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.setWindowTitle("Robot Control Panel")
        self.setMinimumWidth(950)
        self.setMinimumHeight(650)

        self.hardware_joints = {}
        self.ui_elements = {}
        self.joint_values = {cfg["id"]: 0.0 for cfg in JOINT_CONFIGS} # Master state
        self.robot_id = None
        self.pb_joint_map = {}

        # Jog Repeat Timer
        self.jog_timer = QTimer()
        self.jog_timer.timeout.connect(self.process_jog)
        self.active_jog = None 

        for cfg in JOINT_CONFIGS:
            joint = MKSJoint(self.bus, motor_id=cfg["can_id"], can_enabled=cfg["can_enabled"])
            if self.bus and cfg["can_enabled"]:
                joint.setup()
            self.hardware_joints[cfg["id"]] = joint

        self.init_ui()

    def is_pb_connected(self):
        try:
            info = p.getConnectionInfo()
            return info['isConnected'] == 1
        except: return False

    def toggle_live_view(self):
        if not self.is_pb_connected():
            try:
                p.connect(p.GUI)
                p.setAdditionalSearchPath(pybullet_data.getDataPath())
                p.setGravity(0, 0, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
                p.loadURDF("plane.urdf")
                if os.path.exists(URDF_PATH):
                    self.robot_id = p.loadURDF(URDF_PATH, [0, 0, 0], useFixedBase=True)
                    p.resetDebugVisualizerCamera(0.8, 45, -30, [0, 0, 0.3])
                    self.pb_joint_map = {p.getJointInfo(self.robot_id, i)[1].decode('utf-8'): i 
                                         for i in range(p.getNumJoints(self.robot_id))}
                    self.sync_vis_to_ui()
            except Exception as e: print(f"Vis Error: {e}")
        else:
            p.disconnect()
            self.robot_id = None

    def sync_vis_to_ui(self):
        for j_id in self.joint_values:
            self.update_vis_only(j_id, self.joint_values[j_id])

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.layout_main = QHBoxLayout(central_widget)

        # Side buttons
        self.sidebar = QVBoxLayout()
        self.btn_live_view = QPushButton("Live View")
        self.btn_live_view.setFixedSize(140, 50)
        self.btn_live_view.setStyleSheet("background-color: #5da9f6; color: white; font-weight: bold; border-radius: 4px;")
        self.btn_live_view.clicked.connect(self.toggle_live_view)

        self.btn_home_all = QPushButton("HOME ALL")
        self.btn_home_all.setFixedSize(140, 50)
        self.btn_home_all.setStyleSheet("background-color: #f66d6d; color: white; font-weight: bold; border-radius: 4px;")
        self.btn_home_all.clicked.connect(self.home_all)

        self.sidebar.addWidget(self.btn_live_view)
        self.sidebar.addWidget(self.btn_home_all)
        self.sidebar.addStretch()
        self.layout_main.addLayout(self.sidebar)

        # Grid
        self.grid_layout = QGridLayout()
        for idx, cfg in enumerate(JOINT_CONFIGS):
            container = self.create_joint_control(cfg)
            self.grid_layout.addWidget(container, idx // 2, idx % 2)
        self.layout_main.addLayout(self.grid_layout)

    def create_joint_control(self, cfg):
        j_id = cfg["id"]
        status = " [REAL]" if cfg["can_enabled"] else " [VIRTUAL]"
        
        frame = QFrame()
        # 1. FIXED WIDTH: This prevents the boxes from stretching too wide
        frame.setFixedWidth(380) 
        # 2. FIXED HEIGHT: This prevents the boxes from stretching too tall
        frame.setFixedHeight(180) 
        
        frame.setStyleSheet("""
            QFrame { 
                background-color: #2b2b2b; 
                border: 1px solid #3d3d3d; 
                border-radius: 6px; 
            }
        """)
        
        lyt = QVBoxLayout(frame)
        # 3. SPACING: Reduce the gap between elements to keep it compact
        lyt.setSpacing(10)
        lyt.setContentsMargins(15, 15, 15, 15)
        
        title = QLabel(f"<b>{cfg['name']}</b><span style='color: #888;'>{status}</span>")
        title.setStyleSheet("border:none; color: white;")
        lyt.addWidget(title)
        
        input_row = QHBoxLayout()
        angle_input = QLineEdit("0.0")
        angle_input.setValidator(QDoubleValidator(-360.0, 360.0, 1))
        angle_input.setFixedWidth(70)
        angle_input.setStyleSheet("background-color: #1e1e1e; color: white;")
        input_row.addWidget(QLabel("Angle:"))
        input_row.addStretch()
        input_row.addWidget(angle_input)
        lyt.addLayout(input_row)

        slider_row = QHBoxLayout()
        btn_minus = QPushButton("-")
        btn_minus.setFixedSize(32, 32)
        btn_minus.setStyleSheet("background-color: #444; color: white;")
        
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(-360, 360)
        
        btn_plus = QPushButton("+")
        btn_plus.setFixedSize(32, 32)
        btn_plus.setStyleSheet("background-color: #444; color: white;")
        
        slider_row.addWidget(btn_minus)
        slider_row.addWidget(slider)
        slider_row.addWidget(btn_plus)
        lyt.addLayout(slider_row)

        self.ui_elements[j_id] = {"slider": slider, "input": angle_input}
        
        # UI Events
        slider.valueChanged.connect(lambda val, cid=j_id: self.update_joint(cid, float(val)))
        angle_input.returnPressed.connect(lambda cid=j_id: self.on_text_confirm(cid))
        
        # Button Events (Improved)
        btn_minus.pressed.connect(lambda cid=j_id: self.initiate_jog(cid, -JOG_STEP))
        btn_plus.pressed.connect(lambda cid=j_id: self.initiate_jog(cid, JOG_STEP))
        btn_minus.released.connect(self.terminate_jog)
        btn_plus.released.connect(self.terminate_jog)
        
        return frame

    def initiate_jog(self, j_id, delta):
        self.active_jog = (j_id, delta)
        self.process_jog() # Run first step
        QTimer.singleShot(JOG_DELAY, self.start_repeating)

    def start_repeating(self):
        if self.active_jog:
            self.jog_timer.start(JOG_INTERVAL)

    def terminate_jog(self):
        self.active_jog = None
        self.jog_timer.stop()
        self.setFocus() # Clear focus from the button

    def process_jog(self):
        if self.active_jog:
            jid, delta = self.active_jog
            new_val = self.joint_values[jid] + delta
            self.update_joint(jid, new_val)

    def update_joint(self, j_id, angle):
        angle = max(-360.0, min(360.0, angle))
        self.joint_values[j_id] = angle # Update master state
        
        cfg = JOINT_CONFIGS[j_id]
        elements = self.ui_elements[j_id]
        
        elements["slider"].blockSignals(True)
        elements["input"].blockSignals(True)
        elements["slider"].setValue(int(angle))
        elements["input"].setText(f"{angle:.1f}")
        elements["slider"].blockSignals(False)
        elements["input"].blockSignals(False)
        
        if j_id in self.hardware_joints:
            cmd = -angle if cfg["invert"] else angle
            self.hardware_joints[j_id].move_to_angle(cmd)
        
        self.update_vis_only(j_id, angle)

    def update_vis_only(self, j_id, angle):
        if self.is_pb_connected() and self.robot_id is not None:
            joint_name = JOINT_CONFIGS[j_id]["urdf_link"]
            if joint_name in self.pb_joint_map:
                p.resetJointState(self.robot_id, self.pb_joint_map[joint_name], np.radians(angle))

    def on_text_confirm(self, j_id):
        try:
            val = float(self.ui_elements[j_id]["input"].text())
            self.update_joint(j_id, val)
        except: pass
        self.setFocus()

    def home_all(self):
        for cfg in JOINT_CONFIGS:
            idx = cfg["id"]
            if idx in self.hardware_joints: self.hardware_joints[idx]._send([0x92])
            self.update_joint(idx, 0.0)
        self.setFocus()

    def closeEvent(self, event):
        for j in self.hardware_joints.values(): j.disable()
        if self.bus: self.bus.shutdown()
        if self.is_pb_connected(): p.disconnect()
        event.accept()

def main():
    try: 
        bus = can.interface.Bus(interface='slcan', channel=COM_PORT, bitrate=BITRATE)
        print(f"Successfully connected to CAN adapter on {COM_PORT}")
    except Exception as e: 
        print(f"\n[ERROR] Failed to connect to CAN bus!")
        print(f"Reason: {e}\n")
        bus = None
        
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = RobotArmWindow(bus)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()