from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QSlider, QLabel, QPushButton, QLineEdit, QGridLayout, 
                             QFrame, QTabWidget)
from PyQt6.QtCore import Qt, QTimer, QUrl, QSettings
from PyQt6.QtGui import QDoubleValidator, QColor
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEngineSettings

# Import from our other files
import config
from hardware import MKSJoint
from kinematics import RobotKinematics

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
        self.ui_elements = {}
        self.pose_inputs = {}
        self.tcp_inputs = {}
        self.ik_buttons = []
        self.joint_values = {cfg["id"]: 0.0 for cfg in config.JOINT_CONFIGS}
        
        self.tcp_enabled = self.settings.value("tcp_enabled", False, type=bool)
        self.tcp_offset = [
            float(self.settings.value("tcp_dx", 0.0)),
            float(self.settings.value("tcp_dy", 0.0)),
            float(self.settings.value("tcp_dz", 0.0)),
            float(self.settings.value("tcp_drx", 0.0)),
            float(self.settings.value("tcp_dry", 0.0)),
            float(self.settings.value("tcp_drz", 0.0))
        ]

        self.jog_timer = QTimer()
        self.jog_timer.timeout.connect(self.process_jog)
        self.active_jog = None 

        for cfg in config.JOINT_CONFIGS:
            joint = MKSJoint(self.bus, motor_id=cfg["can_id"], can_enabled=cfg["can_enabled"])
            if self.bus and cfg["can_enabled"]:
                joint.setup()
            self.hardware_joints[cfg["id"]] = joint

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        central_widget.setObjectName("CentralWidget")
        central_widget.setStyleSheet("QWidget#CentralWidget { background-color: #121212; }")
        
        self.setCentralWidget(central_widget)
        self.layout_main = QHBoxLayout(central_widget)

        # 3D Viewer
        self.web_view = QWebEngineView()
        self.web_view.setMinimumWidth(500)
        self.web_view.page().setBackgroundColor(QColor("#1e1e1e"))
        self.layout_main.addWidget(self.web_view, stretch=3)
        self.load_3d_viewer()

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #3d3d3d; border-radius: 4px; background-color: #1e1e1e; }
            QTabBar::tab { background: #2b2b2b; color: #aaa; padding: 10px 25px; border: 1px solid #3d3d3d; border-bottom: none; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px; font-weight: bold; font-size: 13px;}
            QTabBar::tab:selected { background: #1e1e1e; color: #5da9f6; }
            QTabBar::tab:hover:!selected { background: #333; }
        """)
        self.layout_main.addWidget(self.tabs, stretch=2)

        # Tab 1: Main Controls
        self.tab_main = QWidget()
        self.tab_main.setObjectName("TabMain")
        self.tab_main.setStyleSheet("QWidget#TabMain { background-color: #1e1e1e; }")
        self.layout_main_tab = QVBoxLayout(self.tab_main)
        self.layout_main_tab.setContentsMargins(15, 15, 15, 15)
        self.layout_main_tab.setSpacing(10)

        self.btn_mode_toggle = QPushButton("MODE: FORWARD KINEMATICS")
        self.btn_mode_toggle.setCheckable(True)
        self.btn_mode_toggle.setFixedHeight(50) 
        self.btn_mode_toggle.setStyleSheet("background-color: #2b2b2b; color: #aaa; font-weight: bold; border: 1px solid #444; border-radius: 4px; font-size: 14px;")
        self.btn_mode_toggle.toggled.connect(self.toggle_mode)
        self.layout_main_tab.addWidget(self.btn_mode_toggle)

        self.layout_main_tab.addWidget(self.create_pose_display())

        self.btn_home_all = QPushButton("HOME ALL JOINTS")
        self.btn_home_all.setFixedHeight(50)
        self.btn_home_all.setStyleSheet("background-color: #f66d6d; color: white; font-weight: bold; border-radius: 4px; font-size: 14px;")
        self.btn_home_all.clicked.connect(self.home_all)
        self.layout_main_tab.addWidget(self.btn_home_all)

        self.grid_layout = QGridLayout()
        for idx, cfg in enumerate(config.JOINT_CONFIGS):
            container = self.create_joint_control(cfg)
            self.grid_layout.addWidget(container, idx // 2, idx % 2)
        
        self.layout_main_tab.addLayout(self.grid_layout)
        self.layout_main_tab.addStretch()
        self.tabs.addTab(self.tab_main, "Main Controls")

        # Tab 2: Settings
        self.tab_settings = QWidget()
        self.tab_settings.setObjectName("TabSettings")
        self.tab_settings.setStyleSheet("QWidget#TabSettings { background-color: #1e1e1e; }")
        self.layout_settings_tab = QVBoxLayout(self.tab_settings)
        self.layout_settings_tab.setContentsMargins(15, 15, 15, 15)
        
        self.settings_grid = QGridLayout()
        tcp_card = self.create_tcp_display()
        self.settings_grid.addWidget(tcp_card, 0, 0)
        self.settings_grid.setColumnStretch(1, 1)
        self.settings_grid.setRowStretch(1, 1)
        
        self.layout_settings_tab.addLayout(self.settings_grid)
        self.tabs.addTab(self.tab_settings, "Settings")

        self.update_kinematics()

    def create_pose_display(self):
        frame = QFrame()
        frame.setFixedHeight(90)
        frame.setStyleSheet("background-color: #1e1e1e; border-radius: 6px; border: 1px solid #3d3d3d;")
        
        lyt = QGridLayout(frame)
        lyt.setContentsMargins(15, 10, 15, 10)
        lyt.setHorizontalSpacing(8)
        lyt.setVerticalSpacing(8)

        lyt.setColumnStretch(1, 1)
        lyt.setColumnStretch(3, 1)
        lyt.setColumnStretch(5, 1)

        labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        btn_style = "QPushButton { background-color: #444; color: white; border-radius: 4px; font-weight: bold; } QPushButton:disabled { background-color: #2b2b2b; color: #555; border: 1px solid #333; }"

        for i, name in enumerate(labels):
            lbl = QLabel(f"<b>{name}:</b>")
            lbl.setStyleSheet("border:none; color: #aaa; font-size: 13px;")
            lyt.addWidget(lbl, i // 3, (i % 3) * 2)
            
            ctrl_layout = QHBoxLayout()
            ctrl_layout.setContentsMargins(0, 0, 0, 0)
            ctrl_layout.setSpacing(4)
            
            btn_minus = QPushButton("-")
            btn_minus.setFixedSize(22, 28)
            btn_minus.setStyleSheet(btn_style)
            btn_minus.setEnabled(False) 
            
            edit = QLineEdit("0.0")
            edit.setMinimumWidth(60) 
            edit.setFixedHeight(28) 
            edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            edit.setStyleSheet("QLineEdit { color: #5da9f6; font-weight: bold; background: #2b2b2b; border: 1px solid #333; border-radius: 4px; font-size: 13px; }")
            edit.returnPressed.connect(self.trigger_ik_solve)
            self.pose_inputs[name] = edit
            
            btn_plus = QPushButton("+")
            btn_plus.setFixedSize(22, 28)
            btn_plus.setStyleSheet(btn_style)
            btn_plus.setEnabled(False) 
            
            btn_minus.pressed.connect(lambda n=name: self.initiate_ik_jog(n, -config.JOG_STEP))
            btn_plus.pressed.connect(lambda n=name: self.initiate_ik_jog(n, config.JOG_STEP))
            btn_minus.released.connect(self.terminate_jog)
            btn_plus.released.connect(self.terminate_jog)
            
            self.ik_buttons.extend([btn_minus, btn_plus])
            ctrl_layout.addWidget(btn_minus)
            ctrl_layout.addWidget(edit)
            ctrl_layout.addWidget(btn_plus)
            
            container = QWidget()
            container.setLayout(ctrl_layout)
            lyt.addWidget(container, i // 3, (i % 3) * 2 + 1)
        return frame

    def create_tcp_display(self):
        frame = QFrame()
        frame.setFixedWidth(360) 
        frame.setFixedHeight(80) 
        frame.setStyleSheet("QFrame { background-color: #2b2b2b; border: 1px solid #3d3d3d; border-radius: 6px; }")
        
        lyt = QVBoxLayout(frame)
        lyt.setSpacing(4)
        lyt.setContentsMargins(12, 8, 12, 8)
        
        header_lyt = QHBoxLayout()
        title = QLabel("<b>TOOL OFFSET (TCP)</b>")
        title.setStyleSheet("border:none; color: #e2b714; font-size: 12px; font-weight: bold;")
        
        self.btn_tcp_enable = QPushButton("Tool offset: ON" if self.tcp_enabled else "Tool offset: OFF")
        self.btn_tcp_enable.setCheckable(True)
        self.btn_tcp_enable.setChecked(self.tcp_enabled)
        self.btn_tcp_enable.setFixedSize(90, 22)
        self.btn_tcp_enable.setStyleSheet("""
            QPushButton { background-color: #444; color: #aaa; border-radius: 4px; font-size: 11px; font-weight: bold; border: 1px solid #333; }
            QPushButton:hover { border: 1px solid #e2b714; }
            QPushButton:checked { background-color: #5da9f6; color: white; border: 1px solid #5da9f6; }
        """)
        self.btn_tcp_enable.toggled.connect(self.on_tcp_toggled)
        
        header_lyt.addWidget(title)
        header_lyt.addStretch()
        header_lyt.addWidget(self.btn_tcp_enable)
        lyt.addLayout(header_lyt)
        
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(4)
        
        grid.setColumnStretch(2, 1) 
        grid.setColumnStretch(5, 1) 
        
        labels = ["dX:", "dY:", "dZ:", "dRx:", "dRy:", "dRz:"]
        dict_keys = ["dX", "dY", "dZ", "dRx", "dRy", "dRz"]
        
        for i, (display_text, dict_key) in enumerate(zip(labels, dict_keys)):
            lbl = QLabel(display_text)
            lbl.setFixedWidth(25)
            lbl.setStyleSheet("color: #aaa; border: none; font-size: 11px;")
            
            edit = QLineEdit(f"{self.tcp_offset[i]:.1f}")
            edit.setValidator(QDoubleValidator(-1000.0, 1000.0, 1))
            edit.setFixedWidth(38)
            edit.setFixedHeight(20)
            edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            edit.setStyleSheet("QLineEdit { background: #1e1e1e; color: #e2b714; border: 1px solid #444; border-radius: 3px; font-weight: bold; font-size: 11px; }")
            edit.editingFinished.connect(self.update_tcp_from_ui)
            
            row = i // 3
            col = (i % 3) * 3 
            
            grid.addWidget(lbl, row, col)
            grid.addWidget(edit, row, col + 1)
            self.tcp_inputs[dict_key] = edit
            
        lyt.addLayout(grid)
        return frame

    def on_tcp_toggled(self, checked):
        self.btn_tcp_enable.setText("Tool offset: ON" if checked else "Tool offset: OFF")
        self.tcp_enabled = checked
        self.settings.setValue("tcp_enabled", self.tcp_enabled)
        
        if self.btn_mode_toggle.isChecked():
            self.trigger_ik_solve()
        else:
            self.update_kinematics()

    def update_tcp_from_ui(self):
        try:
            self.tcp_offset = [
                float(self.tcp_inputs["dX"].text()), float(self.tcp_inputs["dY"].text()), float(self.tcp_inputs["dZ"].text()),
                float(self.tcp_inputs["dRx"].text()), float(self.tcp_inputs["dRy"].text()), float(self.tcp_inputs["dRz"].text())
            ]
            
            memory_keys = ["tcp_dx", "tcp_dy", "tcp_dz", "tcp_drx", "tcp_dry", "tcp_drz"]
            for key, val in zip(memory_keys, self.tcp_offset):
                self.settings.setValue(key, val)
            
            if self.tcp_enabled:
                if self.btn_mode_toggle.isChecked():
                    self.trigger_ik_solve()
                else:
                    self.update_kinematics()
        except ValueError:
            pass

    def create_joint_control(self, cfg):
        j_id = cfg["id"]
        status = " [R]" if cfg["can_enabled"] else " [V]"
        
        frame = QFrame()
        frame.setFixedWidth(360) 
        frame.setFixedHeight(80) 
        frame.setStyleSheet("QFrame { background-color: #2b2b2b; border: 1px solid #3d3d3d; border-radius: 6px; }")
        
        lyt = QVBoxLayout(frame)
        lyt.setSpacing(8)
        lyt.setContentsMargins(12, 10, 12, 10)
        
        header = QHBoxLayout()
        title = QLabel(f"<b>{cfg['name']}</b><span style='color: #666;'>{status}</span>")
        title.setStyleSheet("border:none; color: white; font-size: 13px;")
        header.addWidget(title)
        header.addStretch()
        header.addWidget(QLabel("Angle:"))
        
        angle_input = QLineEdit("0.0")
        angle_input.setValidator(QDoubleValidator(-360.0, 360.0, 1))
        angle_input.setFixedWidth(55)
        angle_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        angle_input.setStyleSheet("background-color: #1e1e1e; color: #f69d5d; font-weight: bold; border-radius: 3px;")
        header.addWidget(angle_input)
        lyt.addLayout(header)

        ctrl = QHBoxLayout()
        btn_m = QPushButton("-")
        btn_m.setFixedSize(25, 25)
        btn_m.setStyleSheet("background-color: #444; color: white; border-radius: 4px;")
        
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(-360, 360)
        slider.setStyleSheet("QSlider::groove:horizontal { height: 4px; background: #1e1e1e; border-radius: 2px; } QSlider::handle:horizontal { background: #f69d5d; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }")
        
        btn_p = QPushButton("+")
        btn_p.setFixedSize(25, 25)
        btn_p.setStyleSheet("background-color: #444; color: white; border-radius: 4px;")
        
        ctrl.addWidget(btn_m)
        ctrl.addWidget(slider)
        ctrl.addWidget(btn_p)
        lyt.addLayout(ctrl)

        self.ui_elements[j_id] = {"slider": slider, "input": angle_input}
        
        slider.valueChanged.connect(lambda val, cid=j_id: self.update_joint(cid, float(val)))
        angle_input.returnPressed.connect(lambda cid=j_id: self.on_text_confirm(cid))
        btn_m.pressed.connect(lambda cid=j_id: self.initiate_jog(cid, -config.JOG_STEP))
        btn_p.pressed.connect(lambda cid=j_id: self.initiate_jog(cid, config.JOG_STEP))
        btn_m.released.connect(self.terminate_jog)
        btn_p.released.connect(self.terminate_jog)
        
        return frame

    def load_3d_viewer(self):
        settings = self.web_view.settings()
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
        self.web_view.setUrl(QUrl.fromLocalFile(str(config.HTML_PATH)))
        self.web_view.loadFinished.connect(lambda success: self.web_view.page().runJavaScript("window.dispatchEvent(new Event('resize'));") if success else None)

    def toggle_mode(self, checked):
        if checked:
            self.btn_mode_toggle.setText("MODE: INVERSE KINEMATICS")
            self.btn_mode_toggle.setStyleSheet("background-color: #5da9f6; color: white; font-weight: bold; border: 1px solid #5da9f6; border-radius: 4px; font-size: 14px;")
            for ui in self.ui_elements.values(): ui["slider"].setEnabled(False)
            for btn in self.ik_buttons: btn.setEnabled(True)
        else:
            self.btn_mode_toggle.setText("MODE: FORWARD KINEMATICS")
            self.btn_mode_toggle.setStyleSheet("background-color: #2b2b2b; color: #aaa; font-weight: bold; border: 1px solid #444; border-radius: 4px; font-size: 14px;")
            for ui in self.ui_elements.values(): ui["slider"].setEnabled(True)
            for btn in self.ik_buttons: btn.setEnabled(False)
            self.update_kinematics()

    def update_joint(self, j_id, angle):
        angle = max(-360.0, min(360.0, angle))
        self.joint_values[j_id] = angle 
        
        ui = self.ui_elements[j_id]
        ui["slider"].blockSignals(True)
        ui["input"].blockSignals(True)
        ui["slider"].setValue(int(angle))
        ui["input"].setText(f"{angle:.1f}")
        ui["slider"].blockSignals(False)
        ui["input"].blockSignals(False)
        
        # Look for this block inside update_joint():
        if j_id in self.hardware_joints:
            cmd = -angle if config.JOINT_CONFIGS[j_id]["invert"] else angle
            
            # --- NEW DYNAMIC SPEED LOGIC ---
            # If the user is currently holding a jog button, use 85 RPM. Otherwise, 1500.
            current_speed = 85 if self.active_jog else 1500
            self.hardware_joints[j_id].move_to_angle(cmd, speed=current_speed)
            # -------------------------------
        
        vis_cfg = config.JOINT_CONFIGS[j_id]
        axis = vis_cfg.get("axis", "z")
        name = vis_cfg["urdf_link"]
        
        vis_angle = -angle if vis_cfg.get("vis_invert", False) else angle
        self.web_view.page().runJavaScript(f"if(window.setJointAngle) window.setJointAngle('{name}', {vis_angle}, '{axis}');")
        
        if not self.btn_mode_toggle.isChecked():
            self.update_kinematics()

    def update_kinematics(self):
        # Call the math engine instead of doing math here
        pos, rpy = self.kinematics_engine.calculate_fk(
            self.joint_values, self.tcp_enabled, self.tcp_offset
        )

        for edit in self.pose_inputs.values(): edit.blockSignals(True)
        self.pose_inputs["X"].setText(f"{pos[0]:.1f}")
        self.pose_inputs["Y"].setText(f"{pos[1]:.1f}")
        self.pose_inputs["Z"].setText(f"{pos[2]:.1f}")
        self.pose_inputs["Rx"].setText(f"{rpy[0]:.1f}")
        self.pose_inputs["Ry"].setText(f"{rpy[1]:.1f}")
        self.pose_inputs["Rz"].setText(f"{rpy[2]:.1f}")
        for edit in self.pose_inputs.values(): edit.blockSignals(False)

    def trigger_ik_solve(self):
        if not self.btn_mode_toggle.isChecked(): return
        try:
            x = float(self.pose_inputs["X"].text())
            y = float(self.pose_inputs["Y"].text())
            z = float(self.pose_inputs["Z"].text())
            rx = float(self.pose_inputs["Rx"].text())
            ry = float(self.pose_inputs["Ry"].text())
            rz = float(self.pose_inputs["Rz"].text())
            
            # Use the math engine to find the new angles
            success, new_angles = self.kinematics_engine.calculate_ik(
                x, y, z, rx, ry, rz, self.joint_values, self.tcp_enabled, self.tcp_offset
            )
            
            if success:
                for i, angle in enumerate(new_angles):
                    self.update_joint(i, angle)
            else:
                print("Target pose is out of physical reach!")
                
        except ValueError:
            print("Invalid pose input")

    def initiate_jog(self, j_id, delta):
        self.active_jog = (j_id, delta)
        self.process_jog()
        QTimer.singleShot(config.JOG_DELAY, lambda: self.jog_timer.start(config.JOG_INTERVAL) if self.active_jog else None)

    def initiate_ik_jog(self, coord_name, delta):
        if not self.btn_mode_toggle.isChecked(): return
        self.active_jog = ("ik", coord_name, delta)
        self.process_jog()
        QTimer.singleShot(config.JOG_DELAY, lambda: self.jog_timer.start(config.JOG_INTERVAL) if self.active_jog else None)

    def terminate_jog(self):
        self.active_jog = None
        self.jog_timer.stop()
        self.setFocus()

    def process_jog(self):
        if self.active_jog:
            if len(self.active_jog) == 3 and self.active_jog[0] == "ik":
                _, coord, delta = self.active_jog
                try:
                    val = float(self.pose_inputs[coord].text())
                    self.pose_inputs[coord].setText(f"{val + delta:.1f}")
                    self.trigger_ik_solve()
                except ValueError:
                    pass
            else:
                jid, delta = self.active_jog
                self.update_joint(jid, self.joint_values[jid] + delta)

    def on_text_confirm(self, j_id):
        try: self.update_joint(j_id, float(self.ui_elements[j_id]["input"].text()))
        except: pass
        self.setFocus()

    def home_all(self):
        for idx in self.joint_values:
            if idx in self.hardware_joints: self.hardware_joints[idx]._send([0x92])
            self.update_joint(idx, 0.0)
        self.update_kinematics()

    def closeEvent(self, event):
        for j in self.hardware_joints.values(): j.disable()
        if self.bus: self.bus.shutdown()
        event.accept()