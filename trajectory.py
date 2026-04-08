import numpy as np
import roboticstoolbox as rtb
from PyQt6.QtCore import QObject, QTimer, pyqtSignal

class TrajectoryPlanner(QObject):
    step_ready = pyqtSignal(int, float, float)
    finished = pyqtSignal()

    def __init__(self, hz=20):
        super().__init__()
        self.hz = hz
        self.queue = []
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_step)

    def execute(self, current_q, target_q, duration_sec=2.0):
        # Wraps a single point-to-point move into a sequence
        self.execute_sequence([current_q, target_q], duration_sec)

    def execute_sequence(self, joint_waypoints, duration_per_segment=1.5):
        if len(joint_waypoints) < 2: return
        
        steps = int(duration_per_segment * self.hz)
        time_vector = np.linspace(0, duration_per_segment, steps)
        self.queue = []
        
        # Loop through the list of waypoints and generate connecting curves
        for i in range(len(joint_waypoints) - 1):
            start_q = joint_waypoints[i]
            end_q = joint_waypoints[i+1]
            traj = rtb.jtraj(start_q, end_q, time_vector)
            
            for j in range(steps):
                self.queue.append((traj.q[j], traj.qd[j]))
                
        self.timer.start(1000 // self.hz)

    def process_step(self):
        if not self.queue:
            self.timer.stop()
            self.finished.emit()
            return
            
        positions, velocities = self.queue.pop(0)
        for j_id in range(len(positions)):
            angle = positions[j_id]
            vel_deg_per_sec = abs(velocities[j_id])
            motor_rpm = max((vel_deg_per_sec / 6.0) * 25.0, 15.0)
            self.step_ready.emit(j_id, angle, motor_rpm)