from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                             QLabel, QPushButton, QLineEdit, QSlider, QFrame)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QDoubleValidator
import config

class JoggingPanel(QWidget):
    # Custom signals to alert gui.py
    jog_started = pyqtSignal(str, str, float) # frame_type, coord, delta
    jog_stopped = pyqtSignal()
    snap_requested = pyqtSignal(float, float, float, float, float, float)
    move_requested = pyqtSignal(float, float, float, float, float, float)

    def __init__(self):
        super().__init__()
        self.pose_inputs = {}
        self.tool_inputs = {}
        self.traj_inputs = {}
        self.ik_buttons = []
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        layout.addWidget(self._build_world_block())
        layout.addWidget(self._build_tool_block())
        layout.addWidget(self._build_trajectory_block())

    def update_displays(self, pos, rpy):
        # Update both World and Tool text boxes at the same time
        for inputs in [self.pose_inputs, self.tool_inputs]:
            for edit in inputs.values(): edit.blockSignals(True)
            inputs["X"].setText(f"{pos[0]:.1f}")
            inputs["Y"].setText(f"{pos[1]:.1f}")
            inputs["Z"].setText(f"{pos[2]:.1f}")
            inputs["Rx"].setText(f"{rpy[0]:.1f}")
            inputs["Ry"].setText(f"{rpy[1]:.1f}")
            inputs["Rz"].setText(f"{rpy[2]:.1f}")
            for edit in inputs.values(): edit.blockSignals(False)

    def set_ik_enabled(self, enabled):
        for btn in self.ik_buttons:
            btn.setEnabled(enabled)

    # --- UI Builders ---
    def _build_world_block(self):
        frame = QFrame()
        frame.setStyleSheet("background-color: #1e1e1e; border-radius: 6px; border: 1px solid #3d3d3d;")
        lyt = QVBoxLayout(frame)
        lyt.setContentsMargins(15, 10, 15, 10)
        
        lbl = QLabel("<b>CARTESIAN JOGGING (WORLD FRAME)</b>")
        lbl.setStyleSheet("color: #e2b714; border: none; font-size: 13px;")
        lyt.addWidget(lbl)
        
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        
        btn_style = "QPushButton { background-color: #444; color: white; border-radius: 4px; font-weight: bold; } QPushButton:disabled { background-color: #2b2b2b; color: #555; border: 1px solid #333; }"
        labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        
        for i, name in enumerate(labels):
            lbl_w = QLabel(f"<b>{name}:</b>")
            lbl_w.setStyleSheet("border:none; color: #aaa; font-size: 13px;")
            
            ctrl = QHBoxLayout()
            ctrl.setContentsMargins(0, 0, 0, 0)
            ctrl.setSpacing(4)
            
            btn_minus = QPushButton("-")
            btn_minus.setFixedSize(22, 28)
            btn_minus.setStyleSheet(btn_style)
            btn_minus.setEnabled(False) 
            
            edit = QLineEdit("0.0")
            edit.setMinimumWidth(60)
            edit.setFixedHeight(28)
            edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            edit.setStyleSheet("QLineEdit { color: #e2b714; font-weight: bold; background: #2b2b2b; border: 1px solid #333; border-radius: 4px; font-size: 13px; }")
            edit.returnPressed.connect(self._on_world_snap)
            self.pose_inputs[name] = edit 
            
            btn_plus = QPushButton("+")
            btn_plus.setFixedSize(22, 28)
            btn_plus.setStyleSheet(btn_style)
            btn_plus.setEnabled(False) 
            
            btn_minus.pressed.connect(lambda n=name: self.jog_started.emit("world", n, -config.JOG_STEP))
            btn_plus.pressed.connect(lambda n=name: self.jog_started.emit("world", n, config.JOG_STEP))
            btn_minus.released.connect(self.jog_stopped)
            btn_plus.released.connect(self.jog_stopped)
            
            self.ik_buttons.extend([btn_minus, btn_plus])
            ctrl.addWidget(btn_minus)
            ctrl.addWidget(edit)
            ctrl.addWidget(btn_plus)
            
            container = QWidget()
            container.setLayout(ctrl)
            grid.addWidget(lbl_w, i // 3, (i % 3) * 2)
            grid.addWidget(container, i // 3, (i % 3) * 2 + 1)
            
        lyt.addLayout(grid)
        return frame

    def _build_tool_block(self):
        frame = QFrame()
        frame.setStyleSheet("background-color: #1e1e1e; border-radius: 6px; border: 1px solid #3d3d3d;")
        lyt = QVBoxLayout(frame)
        lyt.setContentsMargins(15, 10, 15, 10)
        
        lbl = QLabel("<b>TOOL JOGGING (LOCAL FRAME)</b>")
        lbl.setStyleSheet("color: #5da9f6; border: none; font-size: 13px;")
        lyt.addWidget(lbl)
        
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        
        btn_style = "QPushButton { background-color: #444; color: white; border-radius: 4px; font-weight: bold; } QPushButton:disabled { background-color: #2b2b2b; color: #555; border: 1px solid #333; }"
        labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        
        for i, name in enumerate(labels):
            lbl_w = QLabel(f"<b>{name}:</b>")
            lbl_w.setStyleSheet("border:none; color: #aaa; font-size: 13px;")
            
            ctrl = QHBoxLayout()
            ctrl.setContentsMargins(0, 0, 0, 0)
            ctrl.setSpacing(4)
            
            btn_minus = QPushButton("-")
            btn_minus.setFixedSize(22, 28)
            btn_minus.setStyleSheet(btn_style)
            btn_minus.setEnabled(False) 
            
            edit = QLineEdit("0.0")
            edit.setMinimumWidth(60)
            edit.setFixedHeight(28)
            edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            edit.setStyleSheet("QLineEdit { color: #5da9f6; font-weight: bold; background: #2b2b2b; border: 1px solid #333; border-radius: 4px; font-size: 13px; }")
            edit.returnPressed.connect(self._on_tool_snap)
            self.tool_inputs[name] = edit 
            
            btn_plus = QPushButton("+")
            btn_plus.setFixedSize(22, 28)
            btn_plus.setStyleSheet(btn_style)
            btn_plus.setEnabled(False) 
            
            btn_minus.pressed.connect(lambda n=name: self.jog_started.emit("tool", n, -config.JOG_STEP))
            btn_plus.pressed.connect(lambda n=name: self.jog_started.emit("tool", n, config.JOG_STEP))
            btn_minus.released.connect(self.jog_stopped)
            btn_plus.released.connect(self.jog_stopped)
            
            self.ik_buttons.extend([btn_minus, btn_plus])
            ctrl.addWidget(btn_minus)
            ctrl.addWidget(edit)
            ctrl.addWidget(btn_plus)
            
            container = QWidget()
            container.setLayout(ctrl)
            grid.addWidget(lbl_w, i // 3, (i % 3) * 2)
            grid.addWidget(container, i // 3, (i % 3) * 2 + 1)
            
        lyt.addLayout(grid)
        return frame

    def _build_trajectory_block(self):
        frame = QFrame()
        frame.setStyleSheet("background-color: #1e1e1e; border-radius: 6px; border: 1px solid #3d3d3d;")
        lyt = QVBoxLayout(frame)
        lyt.setContentsMargins(15, 10, 15, 10)
        
        lbl = QLabel("<b>COORDINATE MOVE (SMART TRAJECTORY)</b>")
        lbl.setStyleSheet("color: #e2b714; border: none; font-size: 13px;")
        lyt.addWidget(lbl)

        h_layout = QHBoxLayout()
        h_layout.setSpacing(15)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        
        labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        for i, name in enumerate(labels):
            lbl_axis = QLabel(f"{name}:")
            lbl_axis.setStyleSheet("color: #aaa; border: none; font-weight: bold;")
            
            edit = QLineEdit("0.0")
            edit.setAlignment(Qt.AlignmentFlag.AlignCenter)
            edit.setStyleSheet("QLineEdit { color: #5da9f6; font-weight: bold; background: #2b2b2b; border: 1px solid #333; border-radius: 4px; padding: 4px; }")
            self.traj_inputs[name] = edit
            
            grid.addWidget(lbl_axis, i // 3, (i % 3) * 2)
            grid.addWidget(edit, i // 3, (i % 3) * 2 + 1)

        h_layout.addLayout(grid)
        
        btn_move = QPushButton("MOVE")
        btn_move.setFixedSize(65, 65) 
        btn_move.setStyleSheet("QPushButton { background-color: #5da9f6; color: white; font-weight: bold; border-radius: 6px; font-size: 14px; } QPushButton:hover { background-color: #4b8cd6; }")
        btn_move.clicked.connect(self._on_move)
        h_layout.addWidget(btn_move)
        
        lyt.addLayout(h_layout)
        return frame

    # --- Internal Event Handlers ---
    def _on_world_snap(self):
        self._emit_snap(self.pose_inputs)

    def _on_tool_snap(self):
        self._emit_snap(self.tool_inputs)
        
    def _emit_snap(self, source_dict):
        try:
            vals = [float(source_dict[k].text()) for k in ["X", "Y", "Z", "Rx", "Ry", "Rz"]]
            self.snap_requested.emit(*vals)
        except ValueError: pass

    def _on_move(self):
        try:
            vals = [float(self.traj_inputs[k].text()) for k in ["X", "Y", "Z", "Rx", "Ry", "Rz"]]
            self.move_requested.emit(*vals)
        except ValueError: pass


class JointsPanel(QWidget):
    jog_started = pyqtSignal(int, float)
    jog_stopped = pyqtSignal()
    set_angle = pyqtSignal(int, float)

    def __init__(self):
        super().__init__()
        self.sliders = {}
        self.inputs = {}
        
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        
        for idx, cfg in enumerate(config.JOINT_CONFIGS):
            container = self._build_joint(cfg)
            grid.addWidget(container, idx // 2, idx % 2)

    def update_display(self, j_id, angle):
        self.sliders[j_id].blockSignals(True)
        self.inputs[j_id].blockSignals(True)
        self.sliders[j_id].setValue(int(angle))
        self.inputs[j_id].setText(f"{angle:.1f}")
        self.sliders[j_id].blockSignals(False)
        self.inputs[j_id].blockSignals(False)

    def set_sliders_enabled(self, enabled):
        for slider in self.sliders.values():
            slider.setEnabled(enabled)

    def _build_joint(self, cfg):
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

        self.sliders[j_id] = slider
        self.inputs[j_id] = angle_input
        
        slider.valueChanged.connect(lambda val, cid=j_id: self.set_angle.emit(cid, float(val)))
        angle_input.returnPressed.connect(lambda cid=j_id: self._on_text(cid))
        btn_m.pressed.connect(lambda cid=j_id: self.jog_started.emit(cid, -config.JOG_STEP))
        btn_p.pressed.connect(lambda cid=j_id: self.jog_started.emit(cid, config.JOG_STEP))
        btn_m.released.connect(self.jog_stopped)
        btn_p.released.connect(self.jog_stopped)
        return frame
        
    def _on_text(self, j_id):
        try: self.set_angle.emit(j_id, float(self.inputs[j_id].text()))
        except ValueError: pass


class GPIOPanel(QWidget):
    output_toggled = pyqtSignal(int, bool)

    def __init__(self):
        super().__init__()
        self.input_indicators = {}
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(40) 
        
        # Outputs Left
        lyt_out = QVBoxLayout()
        lyt_out.setAlignment(Qt.AlignmentFlag.AlignTop)
        lbl_out = QLabel("<b>OUTPUTS:</b>")
        lbl_out.setStyleSheet("color: #e2b714; font-size: 14px; border: none; padding-bottom: 10px;")
        lyt_out.addWidget(lbl_out)
        grid_out = QGridLayout()
        grid_out.setVerticalSpacing(10)
        grid_out.setHorizontalSpacing(15)
        
        for idx, pin in enumerate(config.GPIO_OUTPUT_PINS):
            btn = QPushButton(f"Pin {pin}: OFF")
            btn.setCheckable(True)
            btn.setFixedSize(140, 45) 
            btn.setStyleSheet("QPushButton { background-color: #444; color: white; border-radius: 4px; font-weight: bold; font-size: 13px; }")
            btn.toggled.connect(lambda checked, p=pin, b=btn: self._on_toggle(p, checked, b))
            grid_out.addWidget(btn, idx // 2, idx % 2)
            
        lyt_out.addLayout(grid_out)
        lyt_out.addStretch()
        
        # Inputs Right
        lyt_in = QVBoxLayout()
        lyt_in.setAlignment(Qt.AlignmentFlag.AlignTop)
        lbl_in = QLabel("<b>INPUTS:</b>")
        lbl_in.setStyleSheet("color: #5da9f6; font-size: 14px; border: none; padding-bottom: 10px;")
        lyt_in.addWidget(lbl_in)
        grid_in = QGridLayout()
        grid_in.setVerticalSpacing(10)
        grid_in.setHorizontalSpacing(15)
        
        for idx, pin in enumerate(config.GPIO_INPUT_PINS):
            ind = QLabel(f"Pin {pin}: LOW")
            ind.setFixedSize(140, 45) 
            ind.setAlignment(Qt.AlignmentFlag.AlignCenter)
            ind.setStyleSheet("QLabel { background-color: #444; color: white; border-radius: 4px; font-weight: bold; font-size: 13px; }")
            self.input_indicators[pin] = ind
            grid_in.addWidget(ind, idx // 2, idx % 2)
            
        lyt_in.addLayout(grid_in)
        lyt_in.addStretch() 
        
        layout.addLayout(lyt_out)
        layout.addLayout(lyt_in)

    def _on_toggle(self, pin, checked, button):
        if checked:
            button.setText(f"Pin {pin}: ON")
            button.setStyleSheet("QPushButton { background-color: #2e7d32; color: white; border-radius: 4px; font-weight: bold; font-size: 13px; }") 
        else:
            button.setText(f"Pin {pin}: OFF")
            button.setStyleSheet("QPushButton { background-color: #444; color: white; border-radius: 4px; font-weight: bold; font-size: 13px; }") 
        self.output_toggled.emit(pin, checked)

    def update_input(self, pin, state):
        if pin in self.input_indicators:
            ind = self.input_indicators[pin]
            if state == 1:
                ind.setText(f"Pin {pin}: HIGH")
                ind.setStyleSheet("QLabel { background-color: #5da9f6; color: white; border-radius: 4px; font-weight: bold; font-size: 13px; }")
            else:
                ind.setText(f"Pin {pin}: LOW")
                ind.setStyleSheet("QLabel { background-color: #444; color: white; border-radius: 4px; font-weight: bold; font-size: 13px; }")


class SettingsPanel(QWidget):
    settings_changed = pyqtSignal(bool, list)

    def __init__(self, init_enabled, init_offsets):
        super().__init__()
        self.tcp_enabled = init_enabled
        self.tcp_offset = init_offsets
        self.inputs = {}
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
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
        
        self.btn_tcp = QPushButton("Tool offset: ON" if self.tcp_enabled else "Tool offset: OFF")
        self.btn_tcp.setCheckable(True)
        self.btn_tcp.setChecked(self.tcp_enabled)
        self.btn_tcp.setFixedSize(90, 22)
        self.btn_tcp.setStyleSheet("""
            QPushButton { background-color: #444; color: #aaa; border-radius: 4px; font-size: 11px; font-weight: bold; border: 1px solid #333; }
            QPushButton:hover { border: 1px solid #e2b714; }
            QPushButton:checked { background-color: #5da9f6; color: white; border: 1px solid #5da9f6; }
        """)
        self.btn_tcp.toggled.connect(self._on_toggle)
        
        header_lyt.addWidget(title)
        header_lyt.addStretch()
        header_lyt.addWidget(self.btn_tcp)
        lyt.addLayout(header_lyt)
        
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(4)
        
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
            edit.editingFinished.connect(self._on_edit)
            grid.addWidget(lbl, i // 3, (i % 3) * 3)
            grid.addWidget(edit, i // 3, (i % 3) * 3 + 1)
            self.inputs[dict_key] = edit
            
        lyt.addLayout(grid)
        layout.addWidget(frame)
        
    def _on_toggle(self, checked):
        self.tcp_enabled = checked
        self.btn_tcp.setText("Tool offset: ON" if checked else "Tool offset: OFF")
        self.settings_changed.emit(self.tcp_enabled, self.tcp_offset)
        
    def _on_edit(self):
        try:
            self.tcp_offset = [float(self.inputs[k].text()) for k in ["dX", "dY", "dZ", "dRx", "dRy", "dRz"]]
            self.settings_changed.emit(self.tcp_enabled, self.tcp_offset)
        except ValueError: pass

class DemoPanel(QWidget):
    demo_requested = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(15)

        lbl = QLabel("<b>PRE-PROGRAMMED DEMONSTRATIONS</b>")
        lbl.setStyleSheet("color: #e2b714; border: none; font-size: 14px;")
        layout.addWidget(lbl)

        grid = QGridLayout()
        grid.setHorizontalSpacing(15)
        grid.setVerticalSpacing(15)

        demos = [
            ("Draw Square (YZ Plane)", "square"),
            ("Draw Triangle (YZ Plane)", "triangle"),
            ("Peck / Drill Cycle", "drill"),
            ("TCP Cone Dance", "cone"),
        ]

        btn_style = """
            QPushButton { background-color: #2b2b2b; color: white; border-radius: 6px; font-weight: bold; font-size: 13px; border: 1px solid #444; height: 60px; }
            QPushButton:hover { background-color: #5da9f6; border: 1px solid #5da9f6; }
        """

        for i, (text, demo_id) in enumerate(demos):
            btn = QPushButton(text)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda checked, d=demo_id: self.demo_requested.emit(d))
            grid.addWidget(btn, i // 2, i % 2)

        layout.addLayout(grid)
        layout.addStretch()