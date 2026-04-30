import numpy as np
import time

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class PIDController:
    """ PID-kontroller som eksakt speiler C-koden """
    def __init__(self, kp, ki, kd, max_out, min_out=0.0, integral_boundary=1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.min_out = min_out
        self.integral_boundary = integral_boundary
        
        # Tilstandsvariabler fra C-structen (PID_parameters_t)
        self.error_integral = 0.0
        self.error_previous = 0.0
        self.output_previous = 0.0
        self.measurement_previous = 0.0
        self.reference_previous = 0.0

    def reset(self):
        """ Tilsvarer PID_reset i C-koden """
        self.error_integral = 0.0
        self.error_previous = 0.0
        self.output_previous = 0.0
        self.measurement_previous = 0.0
        self.reference_previous = 0.0

    def compute(self, error, dt, reference=0.0, measurement=0.0):
        """ 
        Tilsvarer PID_controller_with_error_as_input i C-koden.
        Legg merke til at dt (period) brukes identisk med C-koden.
        """
        if dt <= 0.0:
            return self.output_previous

        # 1. Akkumuler feil (NB: Ikke ganget med dt ennå, akkurat som i C)
        self.error_integral += error

        # 2. Integral boundary (Klipping av selve integralet)
        if self.error_integral > self.integral_boundary:
            self.error_integral = self.integral_boundary
        elif self.error_integral < -self.integral_boundary:
            self.error_integral = -self.integral_boundary

        # 3. Beregn umettet pådrag (Output unsaturated)
        p_term = self.kp * error
        i_term = self.ki * self.error_integral * dt
        d_term = self.kd * (error - self.error_previous) / dt
        
        output_unsaturated = p_term + i_term + d_term
        output = output_unsaturated

        # 4. Anti-windup via back-calculation
        if output_unsaturated > self.max_out:
            output = self.max_out
            if self.ki > 0:
                self.error_integral -= error
        elif output_unsaturated < -self.max_out:
            output = -self.max_out
            if self.ki > 0:
                self.error_integral -= error

        # 5. Dødsone (Minimum output)
        if abs(output) <= self.min_out:
            output = 0.0

        # 6. Spike filter (Ett-syklus filter for plutselige hopp)
        if abs(error - self.error_previous) > 70.0:
            output = self.output_previous

        # 7. Oppdater historikk
        self.measurement_previous = measurement
        self.error_previous = error
        self.output_previous = output
        self.reference_previous = reference

        return output

    def is_steady_state(self, error, margin, change_in_reference):
        """ Tilsvarer PID_steady_state i C-koden """
        if abs(error - self.error_previous) > margin or change_in_reference:
            return False
        return True

class HybridController:
    """
    Hybridkontroller med Stop-and-Scan funksjonalitet.
    Tilstandsmaskin: TURN -> DRIVE -> PIROUETTE
    """
    def __init__(self, wheel_radius=0.07, wheel_base=0.175, wp_radius=0.05):
        self.R = wheel_radius
        self.L = wheel_base
        
        # --- PID FOR FREMDRIFT ---
        self.pid_dist = PIDController(kp=1.5, ki=0.1, kd=0.05, max_out=0.4, min_out=0.05, integral_boundary=50.0) 
        self.pid_head = PIDController(kp=2.0, ki=0.5, kd=0.0, max_out=0.8, min_out=0.1, integral_boundary=20.0) 
        
        # --- LYAPUNOV FOR SVINGING ---
        self.lyap_kd = 1.5
        self.lyap_kp = 0.8  
        
        self.drive_threshold = np.deg2rad(2.0) 
        self.goal_epsilon = wp_radius 
        
        # Tilstandsvariabler for piruett
        self.mode = "TURN"
        self.pirouette_direction = 1.0
        self.turn_settle_count = 0
        self.accumulated_yaw = 0.0
        self.last_theta = 0.0

        self.max_theta_correction = 0.3
        self.decay_factor = 20.0

        self.idle_timer = 0.0
        self.calibration_wait_time = 0.5 
        self.is_idle = False

        self.last_scan_center = None
        self.scan_min_separation = 0.60

    def get_wheel_speeds(self, current_pose, target_pose, dt):
        x, y, theta = current_pose
        tx, ty = target_pose
        
        dist_error = np.hypot(tx - x, ty - y)
        target_heading = np.arctan2(ty - y, tx - x)
        heading_error = normalize_angle(target_heading - theta)
            
        omega_l = 0.0
        omega_r = 0.0

        v_robot = 0.0
        w_robot = 0.0
        
        # ==========================================
        # TILSTANDSMASKIN
        # ==========================================
        if self.mode == "TURN":
            if abs(heading_error) < self.drive_threshold:
                self.turn_settle_count += 1
                if self.turn_settle_count > 10: 
                    self.mode = "IDLE"
                    self.real_time_start = time.time()
                    self.pid_dist.reset() 
                    self.pid_head.reset()
                    self.turn_settle_count = 0
            else:
                self.turn_settle_count = 0 # Nullstilles hvis støyen hopper ut igjen
                
            # Lyapunov Turn-Only matematikk
            w_robot = self.lyap_kd * np.sin(heading_error) * np.cos(heading_error) + self.lyap_kp * heading_error
            w_robot = np.clip(w_robot, -1.5, 1.5)

        elif self.mode == "IDLE":
            self.is_idle = True
            
            # Hold hjulene stille
            v_robot = 0.0
            w_robot = 0.0
            
            if (time.time() - self.real_time_start) >= self.calibration_wait_time:
                self.mode = "DRIVE"
                self.is_idle = False
                return 0.0, 0.0, False 

            return 0.0, 0.0, False
                
        elif self.mode == "DRIVE":
            # Hvis vi sporer helt av, stopp og sving på nytt
            if abs(heading_error) > np.deg2rad(15.0):
                self.mode = "TURN"
                return 0.0, 0.0, False
                
            # Er vi fremme ved stjernen?
            if dist_error < self.goal_epsilon:
                if self.last_scan_center is not None:
                    # Vi måler distansen fra hvor roboten *FYSISK ER* i Odom-koordinater
                    # mot hvor den tok det forrige spinnet sitt! (Ikke target - det kan ha hoppet!)
                    dx = x - self.last_scan_center[0]
                    dy = y - self.last_scan_center[1]
                    if np.hypot(dx, dy) < self.scan_min_separation:
                        self.mode = "TURN"
                        return 0.0, 0.0, True

                # Gå inn i Piruett-modus og gjør klar rotasjons-telleren!
                self.mode = "PIROUETTE"
                self.last_theta = theta
                self.accumulated_yaw = 0.0
                return 0.0, 0.0, False
                
            # Kjør mot målet
            v_robot = self.pid_dist.compute(dist_error, dt)
            w_robot = self.pid_head.compute(heading_error, dt)

            attenuation = 1.0 - np.exp(-self.decay_factor * dist_error)

            u_heading_attenuated = w_robot * attenuation

            theta_correction = np.clip(u_heading_attenuated, -self.max_theta_correction, self.max_theta_correction)

            u_left = v_robot - theta_correction
            u_right = v_robot + theta_correction
            
            # Sørg for at hjulene ikke overstiger makshastighet (samme som climping i C-koden)
            u_left = np.clip(u_left, -self.pid_dist.max_out, self.pid_dist.max_out)
            u_right = np.clip(u_right, -self.pid_dist.max_out, self.pid_dist.max_out)

            # Konverter motorpådrag (m/s) til hjulrotasjon (rad/s) for simulatoren
            omega_l = u_left / self.R
            omega_r = u_right / self.R

            return omega_l, omega_r, False

        elif self.mode == "PIROUETTE":
            # 🚀 Regn ut hvor mye vi har rotert siden forrige millisekund
            delta_yaw = abs(normalize_angle(theta - self.last_theta))
            self.accumulated_yaw += delta_yaw
            self.last_theta = theta
            
            # Stå helt stille og snurr rundt (ca 1.5 rad/s = 85 grader/sekund)
            v_robot = 0.0
            w_robot = 1.5 * self.pirouette_direction
            
            # Har vi tatt en full 360? (2 * Pi radianer)
            if self.accumulated_yaw >= (2 * np.pi):
                self.last_scan_center = (x, y)
                self.mode = "TURN" # Gjør klar for neste waypoint
                self.pirouette_direction *= -1.0 # Bytt retning for neste piruett
                return 0.0, 0.0, True # NÅ sier vi ifra til main.py at vi er ferdige!

        # Konverter til hjulhastighet
        omega_r = (v_robot / self.R) + (w_robot * self.L) / (2 * self.R)
        omega_l = (v_robot / self.R) - (w_robot * self.L) / (2 * self.R)
        
        return omega_l, omega_r, False