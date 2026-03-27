import numpy as np

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ==========================================
# ONBOARD EKF (Kopi av din C-kode: EstimatorTask.c)
# ==========================================
class FirmwareEKF:
    def __init__(self, start_pose=[0.5, 0.5, 0.0]):
        # State: [x, y, theta, v, omega]
        self.x = np.array([start_pose[0], start_pose[1], start_pose[2], 0.0, 0.0])
        self.P = np.eye(5) * 0.01
        
        # Prosess-støy (Q)
        self.Q = np.diag([0.0001, 0.0001, 0.01, 0.01, 0.01])
        
        # Målestøy (R) fra C-koden: [v_enc, v_imu, w_enc, w_gyro]
        self.R = np.diag([0.004102, 0.1531, 0.1235, 0.004214])
        
        # Observasjonsmatrise (H)
        self.H = np.zeros((4, 5))
        self.H[0, 3] = 1.0  # Z[0] måler v
        self.H[1, 3] = 1.0  # Z[1] måler v
        self.H[2, 4] = 1.0  # Z[2] måler omega
        self.H[3, 4] = 1.0  # Z[3] måler omega
        
        # Kalibreringsvariabler for ZUPT
        self.gyroOffset = 0.0
        self.accelXoffset = 0.0

    def step(self, Z, dt):
        """ Kjører ett Kalman-steg (tilsvarer ekf_step i C) """
        x_prev, y_prev, th_prev, v_prev, w_prev = self.x
        
        # --- PREDICTION ---
        x_pred = np.zeros(5)
        x_pred[0] = x_prev + v_prev * np.cos(th_prev) * dt
        x_pred[1] = y_prev + v_prev * np.sin(th_prev) * dt
        x_pred[2] = normalize_angle(th_prev + w_prev * dt)
        x_pred[3] = v_prev
        x_pred[4] = w_prev
        
        F = np.eye(5)
        F[0, 2] = -v_prev * np.sin(th_prev) * dt
        F[0, 3] = np.cos(th_prev) * dt
        F[1, 2] = v_prev * np.cos(th_prev) * dt
        F[1, 3] = np.sin(th_prev) * dt
        F[2, 4] = dt
        
        P_pred = F @ self.P @ F.T + self.Q
        
        # --- UPDATE ---
        Z_pred = self.H @ x_pred
        y_res = Z - Z_pred
        
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        
        self.x = x_pred + K @ y_res
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ self.H) @ P_pred

# ==========================================
# DIGITAL TWIN (Fysikkmotor for simulering)
# ==========================================
class RobotDigitalTwin:
    def __init__(self, start_pose=[0.5, 0.5, 0.0], true_walls=None):
        # Spesifikasjoner
        self.WHEEL_RADIUS = 0.07    # 7 cm radius
        self.WHEEL_BASE = 0.175     # 17.5 cm akselavstand
        self.TICKS_PER_REV = 894    # Fra masteroppgaven
        self.WHEEL_FACTOR_MM = (2 * np.pi * self.WHEEL_RADIUS * 1000) / self.TICKS_PER_REV
        
        self.true_pose = np.array(start_pose, dtype=float)
        self.true_walls = true_walls if true_walls is not None else []
        
        self.omega_l_true = 0.0
        self.omega_r_true = 0.0
        
        self.ekf = FirmwareEKF(start_pose)
        
        self.timer_40ms = 0.0
        self.accumulated_ticks_l = 0.0
        self.accumulated_ticks_r = 0.0
        self.prev_v_true = 0.0
        
        # Simulerer treghet
        self.MOTOR_ACCEL = 7.0 

    def step_physics(self, target_omega_l, target_omega_r, dt=0.01):
        """ Kjører 100Hz fysikksimulering og mater 25Hz C-koden """
        
        # 1. Simuler motor-treghet og slip
        accel_step = self.MOTOR_ACCEL * dt
        self.omega_l_true += np.clip(target_omega_l - self.omega_l_true, -accel_step, accel_step)
        self.omega_r_true += np.clip(target_omega_r - self.omega_r_true, -accel_step, accel_step)
        
        w_l_noisy = self.omega_l_true + np.random.normal(0, 0.02 * abs(self.omega_l_true))
        w_r_noisy = self.omega_r_true + np.random.normal(0, 0.05 * abs(self.omega_r_true))
        
        # 2. Sann kinematikk
        v_true = (self.WHEEL_RADIUS / 2.0) * (w_r_noisy + w_l_noisy)
        w_true = (self.WHEEL_RADIUS / self.WHEEL_BASE) * (w_r_noisy - w_l_noisy)
        
        self.true_pose[0] += v_true * np.cos(self.true_pose[2]) * dt
        self.true_pose[1] += v_true * np.sin(self.true_pose[2]) * dt
        self.true_pose[2] = normalize_angle(self.true_pose[2] + w_true * dt)
        
        # 3. Maskinvare: Samle enkoder-ticks 
        self.accumulated_ticks_l += (w_l_noisy * dt / (2 * np.pi)) * self.TICKS_PER_REV
        self.accumulated_ticks_r += (w_r_noisy * dt / (2 * np.pi)) * self.TICKS_PER_REV
        
        # 4. Firmware-syklus (40ms / 25Hz)
        self.timer_40ms += dt
        if self.timer_40ms >= 0.040:
            delta_t = 0.040
            
            # Konverterer ticks til distanse nøyaktig som i C-koden din
            dLeft = self.accumulated_ticks_l * self.WHEEL_FACTOR_MM
            dRight = self.accumulated_ticks_r * self.WHEEL_FACTOR_MM
            
            dRobot = (dLeft + dRight) / 2.0
            dTheta = (dRight - dLeft) / (self.WHEEL_BASE * 1000.0)
            
            # IMU-simulering (Endring i fart, ikke absolutt fart!)
            accel_x_ms2 = (v_true - self.prev_v_true) / delta_t
            self.prev_v_true = v_true 
            gyro_z_raw = w_true + np.random.normal(0, np.sqrt(0.004214))

            corrected_gyro = gyro_z_raw - self.ekf.gyroOffset
            
            # Bygg målevektor (Z)
            Z = np.zeros(4)
            Z[0] = dRobot / (delta_t * 1000.0)
            # Legger til akselerasjonen på forrige EKF-fart for å simulere IMU-hastighet
            Z[1] = self.ekf.x[3] + accel_x_ms2 * delta_t + np.random.normal(0, 0.01)
            Z[2] = dTheta / delta_t
            Z[3] = corrected_gyro
            
           # --- ZUPT OG DYNAMISK VARIANS FRA C-KODEN ---
            
            # 🚀 FIKS: Ekstremt strenge krav for å tro at vi står stille!
            # dTheta må være under 0.0001 (0.14 grader per sekund)
            if abs(dRobot) < 0.001 and abs(dTheta) < 0.0001:
                Z[1] = 0.0  
                
                # Low-pass filter for bias. Oppdateres forsiktig KUN når vi er 100% i ro.
                # Vi bruker den rå gyro-verdien for å unngå at feilen eskalerer.
                self.ekf.gyroOffset += (gyro_z_raw - self.ekf.gyroOffset) * 0.01 
            else:
                if abs(Z[3] - Z[2]) > 0.2 * abs(Z[3]):
                    self.ekf.R[2, 2] = 1.0    
                else:
                    self.ekf.R[2, 2] = 0.1235
            
            # Kjør Kalman-filteret
            self.ekf.step(Z, delta_t)
            
            # Nullstill maskinvare-registre
            self.accumulated_ticks_l = 0.0
            self.accumulated_ticks_r = 0.0
            self.timer_40ms -= 0.040
            
            scan_data = self.raycast_tof(self.true_pose)
            
            return {
                'ekf_pose': self.ekf.x[:3].copy(),
                'scan': scan_data
            }
            
        return None

    def raycast_tof(self, pose, fov=np.deg2rad(27), rays=15, max_range=3.0):
        """ Simulerer ToF-sensoren """
        scan = []
        for ang in np.linspace(-fov/2, fov/2, rays):
            global_ang = pose[2] + ang
            ray_dir = np.array([np.cos(global_ang), np.sin(global_ang)])
            min_dist = max_range + 1.0
            
            for wall in self.true_walls:
                A, B = wall[0], wall[1]
                v1 = pose[:2] - A
                v2 = B - A
                v3 = np.array([-ray_dir[1], ray_dir[0]])
                
                dot = np.dot(v2, v3)
                if abs(dot) < 1e-6: continue
                
                t = (v2[0] * v1[1] - v2[1] * v1[0]) / dot             
                u = np.dot(v1, v3) / dot    
                
                if 0.01 < t < min_dist and 0.0 <= u <= 1.0:
                    min_dist = t

            if min_dist < max_range:
                scan.append((ang, min_dist + np.random.normal(0, 0.05)))
            else:
                # Send max_range to indicate free space with no hit
                scan.append((ang, max_range))
        return scan