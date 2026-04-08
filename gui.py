from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTabWidget, QGridLayout
from PyQt6.QtCore import QTimer, QUrl, QSettings
from PyQt6.QtGui import QColor
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEngineSettings

import numpy as np
from spatialmath import SE3

import config
from hardware import MKSJoint
from kinematics import RobotKinematics

from trajectory import TrajectoryPlanner
# --- NEW: Import the DemoPanel ---
from ui_panels import JoggingPanel, JointsPanel, GPIOPanel, SettingsPanel, DemoPanel


class RobotArmWindow(QMainWindow):
    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.setWindowTitle("Robot Control Panel")
        self.setMinimumWidth(1280) 
        self.setMinimumHeight(720)

        self.settings = QSettings("RobotArmProject", "ControlPanel")
        self.kinematics_engine = RobotKinematics()
        self.hardware_joints = {}
        self.joint_values = {cfg["id"]: 0.0 for cfg in config.JOINT_CONFIGS}
        
        self.tcp_enabled = self.settings.value("tcp_enabled", False, type=bool)
        self.tcp_offset = [
            float(self.settings.value("tcp_dx", 0.0)), float(self.settings.value("tcp_dy", 0.0)),
            float(self.settings.value("tcp_dz", 0.0)), float(self.settings.value("tcp_drx", 0.0)),
            float(self.settings.value("tcp_dry", 0.0)), float(self.settings.value("tcp_drz", 0.0))
        ]

        for cfg in config.JOINT_CONFIGS:
            joint = MKSJoint(self.bus, motor_id=cfg["can_id"], can_enabled=cfg["can_enabled"])
            if self.bus and cfg["can_enabled"]:
                joint.setup()
            self.hardware_joints[cfg["id"]] = joint

        self.active_jog = None 
        self.jog_timer = QTimer()
        self.jog_timer.timeout.connect(self.process_jog)

        self.poll_timer = QTimer()
        self.poll_timer.timeout.connect(self.poll_serial)
        self.poll_timer.start(50)

        self.planner = TrajectoryPlanner()
        self.jog_panel = JoggingPanel()
        self.joints_panel = JointsPanel()
        self.gpio_panel = GPIOPanel()
        self.settings_panel = SettingsPanel(self.tcp_enabled, self.tcp_offset)
        self.demo_panel = DemoPanel() # Instantiate new panel

        # Connections
        self.jog_panel.jog_started.connect(self.start_ik_jog)
        self.jog_panel.jog_stopped.connect(self.stop_jog)
        self.jog_panel.snap_requested.connect(self.trigger_ik_snap)
        self.jog_panel.move_requested.connect(self.trigger_ik_move)
        
        self.demo_panel.demo_requested.connect(self.run_demo) # Demo Connection

        self.joints_panel.jog_started.connect(self.start_joint_jog)
        self.joints_panel.jog_stopped.connect(self.stop_jog)
        self.joints_panel.set_angle.connect(self.update_joint)

        self.gpio_panel.output_toggled.connect(self.set_gpio)
        self.settings_panel.settings_changed.connect(self.on_tcp_changed)

        self.planner.step_ready.connect(self.execute_motor_step)
        self.planner.finished.connect(self.update_kinematics)

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        central_widget.setStyleSheet("QWidget { background-color: #121212; }")
        self.setCentralWidget(central_widget)
        self.layout_main = QHBoxLayout(central_widget)

        self.web_view = QWebEngineView()
        self.web_view.setMinimumWidth(500)
        self.web_view.page().setBackgroundColor(QColor("#1e1e1e"))
        self.layout_main.addWidget(self.web_view, stretch=3)
        self._load_3d_viewer()

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #3d3d3d; border-radius: 4px; background-color: #1e1e1e; }
            QTabBar::tab { background: #2b2b2b; color: #aaa; padding: 10px 25px; border: 1px solid #3d3d3d; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px; font-weight: bold; font-size: 13px;}
            QTabBar::tab:selected { background: #1e1e1e; color: #5da9f6; }
            QTabBar::tab:hover:!selected { background: #333; }
        """)
        self.layout_main.addWidget(self.tabs, stretch=2)

        # Tab 1: Main Controls
        tab_main = QWidget()
        lyt_main = QVBoxLayout(tab_main)
        lyt_main.setContentsMargins(15, 15, 15, 15)
        lyt_main.setSpacing(10)

        self.btn_mode_toggle = QPushButton("MODE: FORWARD KINEMATICS")
        self.btn_mode_toggle.setCheckable(True)
        self.btn_mode_toggle.setFixedHeight(50) 
        self.btn_mode_toggle.setStyleSheet("background-color: #2b2b2b; color: #aaa; font-weight: bold; border: 1px solid #444; border-radius: 4px; font-size: 14px;")
        self.btn_mode_toggle.toggled.connect(self.toggle_mode)

        self.btn_home = QPushButton("HOME ALL JOINTS")
        self.btn_home.setFixedHeight(50)
        self.btn_home.setStyleSheet("background-color: #f66d6d; color: white; font-weight: bold; border-radius: 4px; font-size: 14px;")
        self.btn_home.clicked.connect(self.home_all)

        lyt_main.addWidget(self.btn_mode_toggle)
        lyt_main.addWidget(self.jog_panel)
        lyt_main.addWidget(self.btn_home)
        lyt_main.addWidget(self.joints_panel)
        lyt_main.addStretch()
        self.tabs.addTab(tab_main, "Main Controls")

        # Tab 2, 3, & 4
        self.tabs.addTab(self.demo_panel, "Show Off") # <--- NEW TAB!
        self.tabs.addTab(self.gpio_panel, "GPIO / Tools")
        
        tab_set = QWidget()
        lyt_set = QVBoxLayout(tab_set)
        lyt_set.setContentsMargins(15, 15, 15, 15)
        grid_set = QGridLayout()
        grid_set.addWidget(self.settings_panel, 0, 0)
        grid_set.setColumnStretch(1, 1)
        grid_set.setRowStretch(1, 1)
        lyt_set.addLayout(grid_set)
        self.tabs.addTab(tab_set, "Settings")

        self.update_kinematics()

    def _load_3d_viewer(self):
        settings = self.web_view.settings()
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        self.web_view.setUrl(QUrl.fromLocalFile(str(config.HTML_PATH)))
        self.web_view.loadFinished.connect(lambda success: self.web_view.page().runJavaScript("window.dispatchEvent(new Event('resize'));") if success else None)

    # --- DEMO LOGIC ---
    # --- DEMO LOGIC ---
    def run_demo(self, demo_id):
        if not self.btn_mode_toggle.isChecked():
            print("Please switch to INVERSE KINEMATICS mode to run shape demos.")
            return

        pos, rpy = self.kinematics_engine.calculate_fk(
            self.joint_values, self.tcp_enabled, self.tcp_offset
        )
        x, y, z = pos
        rx, ry, rz = rpy
        
        waypoints_cartesian = []
        duration = 1.2 # Default speed
        
        if demo_id == "square":
            waypoints_cartesian = [
                (x, y, z, rx, ry, rz),
                (x, y + 30, z, rx, ry, rz),
                (x, y + 30, z + 60, rx, ry, rz),
                (x, y - 30, z + 60, rx, ry, rz),
                (x, y - 30, z , rx, ry, rz),
                (x, y, z, rx, ry, rz) 
            ]
        elif demo_id == "triangle":
            waypoints_cartesian = [
                (x, y, z, rx, ry, rz),
                (x, y + 80, z, rx, ry, rz),
                (x, y + 40, z + 69.2, rx, ry, rz),
                (x, y, z, rx, ry, rz)
            ]
        elif demo_id == "drill":
            waypoints_cartesian = [
                (x, y, z, rx, ry, rz),
                (x + 50, y, z, rx, ry, rz), 
                (x, y, z, rx, ry, rz),      
                (x + 50, y, z, rx, ry, rz),
                (x, y, z, rx, ry, rz)
            ]
        elif demo_id == "cone":
            T_current = SE3(*pos) * SE3.RPY(np.radians(rpy))
            tilt_deg = 20.0  # How wide the cone sweeps
            steps = 8        # 8 waypoints to draw a perfect circle
            
            # 1. Start at center
            waypoints_cartesian.append((x, y, z, rx, ry, rz))
            
            # 2. Use Sine/Cosine to sweep a perfect circle around the TCP
            for i in range(steps + 1): # +1 to perfectly close the loop
                angle = np.radians(i * (360.0 / steps))
                d_rx = tilt_deg * np.cos(angle)
                d_ry = tilt_deg * np.sin(angle)
                
                # Right-multiply safely rotates around the locked tool tip!
                T_new = T_current * SE3.Rx(np.radians(d_rx)) * SE3.Ry(np.radians(d_ry))
                waypoints_cartesian.append((*T_new.t, *np.degrees(T_new.rpy())))
                
            # 3. Return to center
            waypoints_cartesian.append((x, y, z, rx, ry, rz))
            
            # Speed up the segments so the 8 circular points blend into a seamless sweep
            duration = 0.5 

        # Math Engine: Convert Cartesian shapes into Joint Angles
        joint_sequence = []
        current_q = [self.joint_values.get(i, 0.0) for i in range(6)]
        joint_sequence.append(current_q) 
        
        for wp in waypoints_cartesian[1:]:
            # We use the previous joint angle as the "seed" so the elbow doesn't suddenly flip!
            success, q = self.kinematics_engine.calculate_ik(
                wp[0], wp[1], wp[2], wp[3], wp[4], wp[5], 
                joint_sequence[-1], 
                self.tcp_enabled, self.tcp_offset
            )
            if success:
                joint_sequence.append(q)
            else:
                print(f"Demo aborted: The shape hit the edge of the physical workspace!")
                return

        # Execute the perfect sequence
        self.planner.execute_sequence(joint_sequence, duration_per_segment=duration)
    # --- CONTROLLER ROUTING LOGIC ---
    def toggle_mode(self, checked):
        if checked:
            self.btn_mode_toggle.setText("MODE: INVERSE KINEMATICS")
            self.btn_mode_toggle.setStyleSheet("background-color: #5da9f6; color: white; font-weight: bold; border: 1px solid #5da9f6; border-radius: 4px; font-size: 14px;")
            self.joints_panel.set_sliders_enabled(False)
            self.jog_panel.set_ik_enabled(True)
        else:
            self.btn_mode_toggle.setText("MODE: FORWARD KINEMATICS")
            self.btn_mode_toggle.setStyleSheet("background-color: #2b2b2b; color: #aaa; font-weight: bold; border: 1px solid #444; border-radius: 4px; font-size: 14px;")
            self.joints_panel.set_sliders_enabled(True)
            self.jog_panel.set_ik_enabled(False)
            self.update_kinematics()

    def on_tcp_changed(self, enabled, offsets):
        self.tcp_enabled = enabled
        self.tcp_offset = offsets
        self.settings.setValue("tcp_enabled", self.tcp_enabled)
        keys = ["tcp_dx", "tcp_dy", "tcp_dz", "tcp_drx", "tcp_dry", "tcp_drz"]
        for key, val in zip(keys, self.tcp_offset): self.settings.setValue(key, val)
        if self.btn_mode_toggle.isChecked(): pass 
        self.update_kinematics()

    def update_joint(self, j_id, angle, override_speed=None):
        angle = max(-360.0, min(360.0, angle))
        self.joint_values[j_id] = angle 
        self.joints_panel.update_display(j_id, angle)
        
        if j_id in self.hardware_joints:
            cmd = -angle if config.JOINT_CONFIGS[j_id]["invert"] else angle
            current_speed = override_speed if override_speed is not None else (85 if self.active_jog else 1500)
            self.hardware_joints[j_id].move_to_angle(cmd, speed=current_speed)
        
        vis_cfg = config.JOINT_CONFIGS[j_id]
        axis = vis_cfg.get("axis", "z")
        name = vis_cfg["urdf_link"]
        vis_angle = -angle if vis_cfg.get("vis_invert", False) else angle
        self.web_view.page().runJavaScript(f"if(window.setJointAngle) window.setJointAngle('{name}', {vis_angle}, '{axis}');")
        
        if not self.btn_mode_toggle.isChecked() and override_speed is None:
            self.update_kinematics()

    def execute_motor_step(self, j_id, angle, motor_rpm):
        self.update_joint(j_id, angle, override_speed=motor_rpm)

    def update_kinematics(self):
        pos, rpy = self.kinematics_engine.calculate_fk(
            self.joint_values, self.tcp_enabled, self.tcp_offset
        )
        self.jog_panel.update_displays(pos, rpy)

    # --- IK ROUTING ---
    def trigger_ik_snap(self, x, y, z, rx, ry, rz):
        if not self.btn_mode_toggle.isChecked(): return
        success, new_angles = self.kinematics_engine.calculate_ik(
            x, y, z, rx, ry, rz, self.joint_values, self.tcp_enabled, self.tcp_offset
        )
        if success:
            for i, angle in enumerate(new_angles): self.update_joint(i, angle)
        else: print("Target pose out of reach!")

    def trigger_ik_move(self, x, y, z, rx, ry, rz):
        if not self.btn_mode_toggle.isChecked(): return
        success, new_angles = self.kinematics_engine.calculate_ik(
            x, y, z, rx, ry, rz, self.joint_values, self.tcp_enabled, self.tcp_offset
        )
        if success:
            current_q = [self.joint_values.get(i, 0.0) for i in range(len(new_angles))]
            self.planner.execute(current_q, new_angles, duration_sec=2.0)
        else: print("Target pose out of reach!")

    # --- JOG TIMERS & MATH ---
    def start_joint_jog(self, j_id, delta):
        self.active_jog = ("joint", j_id, delta)
        self.process_jog()
        QTimer.singleShot(config.JOG_DELAY, lambda: self.jog_timer.start(config.JOG_INTERVAL) if self.active_jog else None)

    def start_ik_jog(self, frame_type, coord_name, delta):
        if not self.btn_mode_toggle.isChecked(): return
        self.active_jog = ("ik", frame_type, coord_name, delta)
        self.process_jog()
        QTimer.singleShot(config.JOG_DELAY, lambda: self.jog_timer.start(config.JOG_INTERVAL) if self.active_jog else None)

    def stop_jog(self):
        self.active_jog = None
        self.jog_timer.stop()
        self.setFocus()

    def process_jog(self):
        if not self.active_jog: return
        
        if self.active_jog[0] == "joint":
            _, jid, delta = self.active_jog
            self.update_joint(jid, self.joint_values[jid] + delta)
            return

        _, frame_type, coord, delta = self.active_jog 
        try:
            pos, rpy = self.kinematics_engine.calculate_fk(
                self.joint_values, self.tcp_enabled, self.tcp_offset
            )
            T_current = SE3(*pos) * SE3.RPY(np.radians(rpy))

            if frame_type == "world":
                if coord in ["X", "Y", "Z"]:
                    dx = delta if coord == "X" else 0
                    dy = delta if coord == "Y" else 0
                    dz = delta if coord == "Z" else 0
                    T_new = SE3(dx, dy, dz) * T_current
                else:
                    rad = np.radians(delta)
                    if coord == "Rx": R_delta = SE3.Rx(rad).R
                    elif coord == "Ry": R_delta = SE3.Ry(rad).R
                    elif coord == "Rz": R_delta = SE3.Rz(rad).R
                    T_new = SE3.Rt(R_delta @ T_current.R, T_current.t)
            else:
                if coord in ["X", "Y", "Z"]:
                    dx = delta if coord == "X" else 0
                    dy = delta if coord == "Y" else 0
                    dz = delta if coord == "Z" else 0
                    T_new = T_current * SE3(dx, dy, dz)
                else:
                    rad = np.radians(delta)
                    if coord == "Rx": T_new = T_current * SE3.Rx(rad)
                    elif coord == "Ry": T_new = T_current * SE3.Ry(rad)
                    elif coord == "Rz": T_new = T_current * SE3.Rz(rad)

            new_pos = T_new.t
            new_rpy = np.degrees(T_new.rpy())
            
            self.jog_panel.update_displays(new_pos, new_rpy)
            self.trigger_ik_snap(new_pos[0], new_pos[1], new_pos[2], new_rpy[0], new_rpy[1], new_rpy[2])
            
        except ValueError: pass

    # --- HARDWARE I/O ---
    def poll_serial(self):
        if not hasattr(self, 'bus') or not self.bus: return
        msgs = self.bus.read_messages()
        for msg in msgs:
            if msg.startswith("I,"):
                parts = msg.split(',')
                if len(parts) == 3:
                    self.gpio_panel.update_input(int(parts[1]), int(parts[2]))

    def set_gpio(self, pin, state):
        if hasattr(self, 'bus') and self.bus:
            self.bus.set_gpio(pin, 1 if state else 0)

    def home_all(self):
        for idx in self.joint_values:
            if idx in self.hardware_joints: self.hardware_joints[idx].zero_motor()
            self.update_joint(idx, 0.0)
        self.update_kinematics()

    def closeEvent(self, event):
        for j in self.hardware_joints.values(): j.disable()
        if self.bus: self.bus.shutdown()
        event.accept()