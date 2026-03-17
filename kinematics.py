import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

class RobotKinematics:
    def __init__(self):
        # Initialize Robot Math (DH Parameters)
        self.robot = rtb.DHRobot([
            rtb.RevoluteDH(d=93.0,  a=0.0,   alpha=-np.pi/2, offset=0.0),
            rtb.RevoluteDH(d=0.0,   a=125.0, alpha=0.0,      offset=-np.pi/2),
            rtb.RevoluteDH(d=0.0,   a=48.0,  alpha=-np.pi/2, offset=0.0),
            rtb.RevoluteDH(d=64.0,  a=0.0,   alpha=np.pi/2,  offset=0.0),
            rtb.RevoluteDH(d=0.0,   a=0.0,   alpha=np.pi/2,  offset=np.pi),
            rtb.RevoluteDH(d=116.0, a=0.0,   alpha=0.0,      offset=0.0),
        ], name="6DOF_Arm")

    def calculate_fk(self, joint_values, tcp_enabled, tcp_offset):
        q = np.radians([joint_values[i] for i in range(6)])
        T_flange = self.robot.fkine(q)
        
        if tcp_enabled:
            T_tool = SE3(*tcp_offset[0:3]) * SE3.RPY(np.radians(tcp_offset[3:6]))
            T_tcp = T_flange * T_tool
        else:
            T_tcp = T_flange
            
        pos = T_tcp.t
        rpy = np.degrees(T_tcp.rpy())
        return pos, rpy

    def calculate_ik(self, x, y, z, rx, ry, rz, current_joints, tcp_enabled, tcp_offset):
        T_target_tcp = SE3(x, y, z) * SE3.RPY(np.radians([rx, ry, rz]))
        
        if tcp_enabled:
            T_tool = SE3(*tcp_offset[0:3]) * SE3.RPY(np.radians(tcp_offset[3:6]))
            T_target = T_target_tcp * T_tool.inv()
        else:
            T_target = T_target_tcp
            
        d6 = 116.0
        p_wc = T_target.t - (d6 * T_target.a)

        theta0 = np.arctan2(p_wc[1], p_wc[0])
        r = np.sqrt(p_wc[0]**2 + p_wc[1]**2)
        s = p_wc[2] - 93.0 
        
        D = np.sqrt(r**2 + s**2) 
        L1 = 125.0 
        L2 = np.sqrt(48.0**2 + 64.0**2) 

        if D > (L1 + L2) or D < abs(L1 - L2):
            return False, None # Target is out of reach

        cos_elbow = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_elbow = np.clip(cos_elbow, -1.0, 1.0) 
        sin_elbow = np.sqrt(1 - cos_elbow**2) 
        
        phi = np.arccos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
        alpha = np.arctan2(s, r)
        
        geom_theta1 = alpha + phi 
        geom_theta2 = np.arctan2(sin_elbow, cos_elbow) 
        
        theta1 = (np.pi / 2) - geom_theta1       
        gamma = np.arctan2(64.0, 48.0) 
        theta2 = geom_theta2 - gamma             

        q0, q1, q2 = theta0, theta1, theta2
        
        T_01 = self.robot.links[0].A(q0)
        T_12 = self.robot.links[1].A(q1)
        T_23 = self.robot.links[2].A(q2)
        T_03 = T_01 * T_12 * T_23
        
        R = T_03.R.T @ T_target.R 
        c4 = np.clip(R[2, 2], -1.0, 1.0) 
        
        curr_j3, curr_j4, curr_j5 = current_joints[3], current_joints[4], current_joints[5]
        
        if abs(c4) >= 0.9999: 
            q4_1 = 0.0 if c4 > 0 else np.pi
            q3_1 = np.radians(curr_j3) 
            sum_angle = np.arctan2(-R[1, 0], -R[0, 0])
            q5_1 = sum_angle - q3_1
            q4_2, q3_2, q5_2 = q4_1, q3_1, q5_1
        else:
            q4_1 = np.arccos(c4)
            q3_1 = np.arctan2(-R[1, 2], -R[0, 2])
            q5_1 = np.arctan2(R[2, 1], -R[2, 0])
            q4_2 = -q4_1
            q3_2 = np.arctan2(R[1, 2], R[0, 2])
            q5_2 = np.arctan2(-R[2, 1], R[2, 0])

        deg3_1, deg4_1, deg5_1 = np.degrees([q3_1, q4_1, q5_1])
        deg3_2, deg4_2, deg5_2 = np.degrees([q3_2, q4_2, q5_2])

        def closest_angle(target_deg, current_deg):
            diff = (target_deg - current_deg + 180) % 360 - 180
            return current_deg + diff

        sol1 = [closest_angle(deg3_1, curr_j3), closest_angle(deg4_1, curr_j4), closest_angle(deg5_1, curr_j5)]
        sol2 = [closest_angle(deg3_2, curr_j3), closest_angle(deg4_2, curr_j4), closest_angle(deg5_2, curr_j5)]

        cost1 = abs(sol1[0] - curr_j3) + abs(sol1[1] - curr_j4) + abs(sol1[2] - curr_j5)
        cost2 = abs(sol2[0] - curr_j3) + abs(sol2[1] - curr_j4) + abs(sol2[2] - curr_j5)

        if cost1 <= cost2:
            final_deg3, final_deg4, final_deg5 = sol1
        else:
            final_deg3, final_deg4, final_deg5 = sol2

        new_angles = [np.degrees(theta0), np.degrees(theta1), np.degrees(theta2), final_deg3, final_deg4, final_deg5]
        return True, new_angles