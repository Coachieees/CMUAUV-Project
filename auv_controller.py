import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import numpy as np
import scipy.linalg
import math
import time

class SDREControllerNode(Node):
    def __init__(self):
        super().__init__('auv_sdre_controller')

        # --- 1. PHYSICAL PARAMETERS ---
        self.m = 11.71
        self.W = self.m * 9.81
        self.Vol = 0.012138
        self.B = 1000 * self.Vol * 9.81
        self.rg = np.array([0, 0, 0.0620])
        self.rb = np.array([0, 0, 0.0220])
        
        self.I0 = np.array([
            [0.201587, 0, 0.003534],
            [0, 0.268106, 0],
            [0.002566, 0, 0.255593]
        ])

        # Rigid Body Mass Matrix (Mrb)
        self.Mrb = np.vstack((
            np.hstack((self.m * np.eye(3), -self.m * self.smtrx(self.rg))),
            np.hstack((self.m * self.smtrx(self.rg), self.I0))
        ))

        # --- 2. HYDRODYNAMIC PARAMETERS ---
        self.Ma = np.diag([6.36, 7.12, 18.68, 0.189, 0.135, 0.222])
        self.M = self.Mrb + self.Ma
        self.M_inv = np.linalg.inv(self.M)

        self.Dl = np.diag([13.7, 0, 33, 0, 0.8, 0])
        self.Dq = np.diag([141.0, 217.0, 190.0, 1.19, 0.47, 1.5])

        # --- 3. THRUSTER CONFIGURATION MATRIX (TCM) ---
        l_com = np.array([
            [150,  110, 44],  [150, -110, 44],
            [-150, 110, 44],  [-150, -110, 44],
            [50,  120,  0],   [50, -120,  0],
            [-50, 120,  0],   [-50, -120,  0]
        ]) / 1000.0

        v = np.zeros((8, 3))
        alpha = np.deg2rad([45, 135, -45, -135])
        for i in range(4):
            v[i, :] = [np.sin(alpha[i]), np.cos(alpha[i]), 0]
        for i in range(4, 8):
            v[i, :] = [0, 0, 1]

        self.T = np.zeros((6, 8))
        for i in range(8):
            force_dir = v[i, :]
            pos = l_com[i, :]
            moment = np.cross(pos, force_dir)
            self.T[:, i] = np.concatenate((force_dir, moment))
            
        self.T_pinv = np.linalg.pinv(self.T)

        # Buoyancy Feedforward (Trim)
        self.tau_trim = np.array([0, 0, self.B - self.W, 0, 0, 0])

        # --- 4. TUNING WEIGHTS (LQR & LQE) ---
        Q_vel = np.diag([50, 10, 200, 10, 50, 50])
        Q_pos = np.diag([100, 100, 100, 50, 50, 200])
        self.Q_x = scipy.linalg.block_diag(Q_vel, Q_pos)
        self.Qi = np.array([[1.0]])
        self.Q_aug = scipy.linalg.block_diag(self.Q_x, self.Qi)
        
        self.R = np.diag([0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05])
        
        self.C_meas = np.hstack((np.zeros((9, 3)), np.eye(9)))
        self.Qn = np.eye(12) * 0.01
        self.Rn = np.eye(9) * 0.02

        # --- 5. RUNTIME STATE VARIABLES ---
        self.X_hat = np.zeros(12)
        self.X_ref = np.zeros(12)
        self.z_integral_error = 0.0
        
        self.is_auto_mode = False  # 🟢 สถานะโหมดควบคุม

        # --- 6. ROS 2 COMMUNICATIONS ---
        self.sensor_sub = self.create_subscription(String, '/auv/sensors', self.sensor_cb, 10)
        self.setpoint_sub = self.create_subscription(String, '/auv/setpoint', self.setpoint_cb, 10) # 🟢 ดักฟังเป้าหมายจาก GUI
        self.pwm_pub = self.create_publisher(Int32MultiArray, '/auv/cmd_pwm', 10)
        
        # 20Hz Control Loop
        self.dt = 0.05
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('SDRE Controller Node Initialized. Waiting for Setpoints...')

    def smtrx(self, a):
        return np.array([
            [0, -a[2], a[1]],
            [a[2], 0, -a[0]],
            [-a[1], a[0], 0]
        ])

    def calc_body_error(self, X_hat, X_ref):
        nu_hat, eta_hat = X_hat[0:6], X_hat[6:12]
        nu_ref, eta_ref = X_ref[0:6], X_ref[6:12]
        
        nu_error = nu_hat - nu_ref
        eta_error = eta_hat - eta_ref
        
        psi = eta_hat[5]
        R_yaw = np.array([
            [np.cos(psi), np.sin(psi), 0],
            [-np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])
        
        pos_error_body = R_yaw @ eta_error[0:3]
        ang_error = eta_error[3:6]
        
        ang_error = np.arctan2(np.sin(ang_error), np.cos(ang_error))
        return np.concatenate((nu_error, pos_error_body, ang_error))

    def dynamic_sdre(self, X_hat):
        nu = X_hat[0:6]
        eta = X_hat[6:12]
        phi, theta, psi = eta[3], eta[4], eta[5]
        
        D_nu = self.Dl + self.Dq @ np.diag(np.abs(nu))
        
        v1 = self.M_inv[0:3, 0:3] @ nu[0:3] + self.M_inv[0:3, 3:6] @ nu[3:6]
        v2 = self.M_inv[3:6, 0:3] @ nu[0:3] + self.M_inv[3:6, 3:6] @ nu[3:6]
        
        C_nu = np.zeros((6, 6))
        C_nu[0:3, 3:6] = -self.smtrx(v1)
        C_nu[3:6, 0:3] = -self.smtrx(v1)
        C_nu[3:6, 3:6] = -self.smtrx(v2)
        
        J1 = np.array([
            [np.cos(psi)*np.cos(theta), -np.sin(psi)*np.cos(phi)+np.cos(psi)*np.sin(theta)*np.sin(phi), np.sin(psi)*np.sin(phi)+np.cos(psi)*np.cos(phi)*np.sin(theta)],
            [np.sin(psi)*np.cos(theta),  np.cos(psi)*np.cos(phi)+np.sin(phi)*np.sin(theta)*np.sin(psi), -np.cos(psi)*np.sin(phi)+np.sin(theta)*np.cos(phi)*np.sin(psi)],
            [-np.sin(theta),             np.cos(theta)*np.sin(phi),                                      np.cos(theta)*np.cos(phi)]
        ])
        J2 = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi),              -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])
        J_eta = np.zeros((6, 6))
        J_eta[0:3, 0:3] = J1
        J_eta[3:6, 3:6] = J2
        
        G_eta = np.zeros((6, 6))
        G_eta[0, 4] = (self.W - self.B) * np.cos(theta)
        G_eta[1, 3] = -(self.W - self.B) * np.cos(theta) * np.cos(phi)
        G_eta[3, 3] = self.rg[2] * self.W * np.cos(theta) * np.cos(phi) + self.rg[1] * self.W * np.cos(theta) * np.sin(phi)
        G_eta[4, 4] = self.rg[2] * self.W * np.cos(theta) * np.cos(phi) - self.rg[0] * self.W * np.sin(theta) * np.cos(phi)
        
        A11 = -self.M_inv @ (C_nu + D_nu)
        A12 = -self.M_inv @ G_eta
        A21 = J_eta
        A22 = np.zeros((6, 6))
        
        A_sys_live = np.vstack((np.hstack((A11, A12)), np.hstack((A21, A22))))
        B_sys_live = np.vstack((self.M_inv @ self.T, np.zeros((6, 8))))
        
        Cz = np.zeros((1, 12))
        Cz[0, 8] = 1.0 
        A_aug = np.vstack((
            np.hstack((A_sys_live, np.zeros((12, 1)))),
            np.hstack((Cz, [[0.0]]))
        ))
        B_aug = np.vstack((B_sys_live, np.zeros((1, 8))))
        
        P_lqr = scipy.linalg.solve_continuous_are(A_aug, B_aug, self.Q_aug, self.R)
        K_aug = np.linalg.inv(self.R) @ B_aug.T @ P_lqr
        K_x = K_aug[:, 0:12]
        K_i = K_aug[:, 12]
        
        P_lqe = scipy.linalg.solve_continuous_are(A_sys_live.T, self.C_meas.T, self.Qn, self.Rn)
        L_gain = P_lqe @ self.C_meas.T @ np.linalg.inv(self.Rn)
        
        return K_x, K_i, L_gain, A_sys_live, B_sys_live

    # 🟢 รับค่าเซ็นเซอร์ (เอามาอัปเดต Y_meas)
    def sensor_cb(self, msg):
        parts = msg.data.split(',')
        if len(parts) >= 18 and parts[0] == "DATA":
            try:
                yaw, pitch, roll = float(parts[1]), float(parts[2]), float(parts[3])
                depth = float(parts[4])
                gx, gy, gz = float(parts[14]), float(parts[15]), float(parts[16])
                
                phi, theta, psi = math.radians(roll), math.radians(pitch), math.radians(yaw)
                self.Y_meas = np.array([gx, gy, gz, 0.0, 0.0, depth, phi, theta, psi])
            except ValueError:
                pass

    # 🟢 รับเป้าหมายจาก GUI
    def setpoint_cb(self, msg):
        parts = msg.data.split(',')
        if parts[0] == "AUTO" and len(parts) == 5:
            self.is_auto_mode = True
            # X_ref = [u,v,w, p,q,r, x,y,z, phi,theta,psi] -> Indicies: z=8, phi=9, theta=10, psi=11
            self.X_ref[8] = float(parts[1])
            self.X_ref[9] = math.radians(float(parts[2])) # Roll
            self.X_ref[10] = math.radians(float(parts[3])) # Pitch
            self.X_ref[11] = math.radians(float(parts[4])) # Yaw
        elif parts[0] == "MANUAL":
            self.is_auto_mode = False
            self.z_integral_error = 0.0 # ล้างค่า Integral สะสมเมื่อสลับโหมด

    def control_loop(self):
        # 🟢 ถ้าระบบไม่ได้อยู่ในโหมด Auto หรือเซ็นเซอร์ยังไม่ส่งค่ามา ให้ข้ามการคำนวณไปเลย
        if not self.is_auto_mode or not hasattr(self, 'Y_meas'): 
            return

        K_x, K_i, L_gain, A_sys, B_sys = self.dynamic_sdre(self.X_hat)
        
        Y_hat = self.C_meas @ self.X_hat
        U_cmd = getattr(self, 'last_U', np.zeros(8)) 
        
        X_hat_dot = A_sys @ self.X_hat + B_sys @ U_cmd + L_gain @ (self.Y_meas - Y_hat)
        self.X_hat = self.X_hat + X_hat_dot * self.dt 
        
        X_error = self.calc_body_error(self.X_hat, self.X_ref)
        
        z_error = X_error[8] 
        self.z_integral_error += z_error * self.dt
        
        U_optimal = -K_x @ X_error - (K_i * self.z_integral_error)
        self.last_U = U_optimal 
        
        thrust_cmd = U_optimal + (self.T_pinv @ self.tau_trim)
        
        pwms = np.zeros(8, dtype=int)
        for i in range(8):
            pwm_val = 1500 - int(thrust_cmd[i] * 400) 
            pwms[i] = max(1000, min(2000, pwm_val))
            
        msg = Int32MultiArray()
        msg.data = pwms.tolist()
        self.pwm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SDREControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
