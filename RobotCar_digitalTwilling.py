import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import struct
import threading
import queue
import time

# --- CONFIG ---
ROBOT_ID = 99
TCP_HOST = 'localhost'
TCP_PORT = 9000

MAP_SIZE_M = 4.0       
MAP_OFFSET = MAP_SIZE_M / 2.0  # 2.0 meter

# --- NOISE SETTINGS ---
# Simulated sensor depth noise standard deviation (meters). 
# Example: 0.05 = 68% of rays are within +/- 5cm of reality
SENSOR_NOISE_STD_DEV = 0.05  

# Noise injected into the calculated angle of the fitted line (radians)
LINE_ANGLE_NOISE_STD_DEV = 0.02

# Optional noise injected into the *reported* position (odometry drift over time / slip).
# Set to 0.0 to disable.
# Example: 0.02 = 2cm position noise per frame sent to the server.
LOCATION_NOISE_STD_DEV = 0.0
LOCATION_ANGLE_NOISE_STD_DEV = 0.0  # Radians

# --- TRACKING MODE ---
# If True, only the single closest fitted line to the robot is sent to the server.
# If False, all visible lines are sent.
TRACK_CLOSEST_ONLY = False

# --- 1. ASYNKRON TCP KLIENT ---
class AsyncTcpClient:
    def __init__(self, host, port):
        self.address = (host, port)
        self.connected = False
        self.queue = queue.Queue(maxsize=100)
        self.running = True
        
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def send_packet(self, robot_id, x_cm, y_cm, theta_deg, local_start, local_end):
        # --- 1. Forbered Linjedata (mm) ---
        p1x = int(local_start[0] * 1000)
        p1y = int(local_start[1] * 1000)
        p2x = int(local_end[0] * 1000)
        p2y = int(local_end[1] * 1000)

        # --- 2. Beregn Hesses Normalform (Lokal) ---
        # Vi regner på meter først for enkelhets skyld
        lx1, ly1 = local_start
        lx2, ly2 = local_end
        
        # Finn vinkel til linjen, og legg til 90 grader for normalen
        line_angle = np.arctan2(ly2 - ly1, lx2 - lx1)
        alpha_rad = line_angle + (np.pi / 2.0)
        
        # Finn avstand rho = x*cos(alpha) + y*sin(alpha)
        # Vi kan bruke et av punktene (f.eks p1)
        rho_val = lx1 * np.cos(alpha_rad) + ly1 * np.sin(alpha_rad)
        
        # Standardiser Hesse: rho skal være >= 0
        if rho_val < 0:
            rho_val = -rho_val
            alpha_rad -= np.pi # Snu vinkelen 180 grader
            
        # Normaliser alpha til -pi til +pi
        alpha_rad = (alpha_rad + np.pi) % (2 * np.pi) - np.pi
        
        # Konverter til sendbart format
        rho_mm = int(rho_val * 1000)
        alpha_mrad = int(alpha_rad * 1000) # Milliradianer

        # --- 3. Forbered Posisjon ---
        x_val = int(x_cm - (MAP_OFFSET * 100))
        y_val = int(y_cm - (MAP_OFFSET * 100))
        theta_val = int(theta_deg)

        # Begrens verdier
        rho_mm = max(min(rho_mm, 32000), 0)
        alpha_mrad = max(min(alpha_mrad, 32000), -32000)

        # --- 4. Pakking (19 bytes) ---
        # B (ID) + hhh (Pos) + hhhh (Line) + hh (Hesse)
        try:
            payload = struct.pack('<BBBhhhhhhhhh', 
                                  0xFF, 0xFE,          # Header
                                  int(robot_id) % 255, # ID
                                  x_val, y_val, theta_val,
                                  p1x, p1y, p2x, p2y,
                                  rho_mm, alpha_mrad)
            self.queue.put(payload, block=False)
        except:
            pass

    def _worker_loop(self):
        sock = None
        while self.running:
            if not self.connected:
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(2.0) 
                    sock.connect(self.address)
                    self.connected = True
                    print(f"--- TCP TILKOBLET (Standalone Mode) ---")
                except:
                    self.connected = False
                    time.sleep(1.0)
                    continue 
            
            try:
                payload = self.queue.get(timeout=0.1)
                sock.sendall(payload)
                self.queue.task_done()
            except queue.Empty:
                pass
            except:
                print("--- MISTET KOBLING ---")
                self.connected = False
                if sock: sock.close()

# --- 2. DIGITAL TVILLING (Logikk) ---
class DigitalTwin:
    def __init__(self):
        # Start i midten av 4x4 rommet (2.0, 2.0)
        # Dette vil bli sendt som (0,0) til Go pga offset.
        self.pose = np.array([2.0, 2.0, 0.0]) 
        
        self.fov = np.radians(120)
        self.num_rays = 60
        self.max_range = 3.5
        self.safe_distance = 0.5
        self.speed_step = 0.05
        self.turn_step = 0.1
        self.manual_mode = False 
        self.keys = {'up': False, 'down': False, 'left': False, 'right': False}
        
        # Kart tilpasset 4x4 meter (Go MapSize = 400)
        # Koordinater her er 0 til 4.
        self.obstacles = [
            [(0, 0), (4, 0)],   # Bunn
            [(4, 0), (4, 4)],   # Høyre
            [(4, 4), (0, 4)],   # Topp
            [(0, 4), (0, 0)],   # Venstre
            
            [(1, 1), (1, 3)],   # Vegg A
            [(2.5, 1), (3.5, 1)], # Vegg B
            [(0.5, 3.5), (1.5, 3.5)] # Vegg C
        ]

    def get_lidar_hits(self):
        hits = []
        angles = np.linspace(self.pose[2] - self.fov/2, self.pose[2] + self.fov/2, self.num_rays)
        for angle in angles:
            ray_dir = np.array([np.cos(angle), np.sin(angle)])
            closest_dist = self.max_range
            closest_point = None
            p1 = self.pose[:2]
            for wall in self.obstacles:
                w1, w2 = np.array(wall[0]), np.array(wall[1])
                v1 = p1 - w1; v2 = w2 - w1
                v3 = np.array([-ray_dir[1], ray_dir[0]])
                dot = np.dot(v2, v3)
                if abs(dot) < 1e-6: continue
                t1 = np.cross(v2, v1) / dot
                t2 = np.dot(v1, v3) / dot
                if t1 >= 0 and t1 < closest_dist and 0 <= t2 <= 1:
                    noisy_t1 = t1
                    if SENSOR_NOISE_STD_DEV > 0:
                        noisy_t1 += np.random.normal(0, SENSOR_NOISE_STD_DEV)
                    
                    if noisy_t1 < 0.1: 
                        noisy_t1 = 0.1

                    closest_dist = noisy_t1
                    closest_point = p1 + ray_dir * noisy_t1
            if closest_point is not None: hits.append(closest_point)
        return np.array(hits)

    def update_physics(self, lidar_points):
        target_speed = 0.0; target_turn = 0.0
        if self.manual_mode:
            if self.keys['up']: target_speed = self.speed_step
            if self.keys['down']: target_speed = -self.speed_step
            if self.keys['left']: target_turn = self.turn_step
            if self.keys['right']: target_turn = -self.turn_step
        else:
            min_dist = self.max_range
            if len(lidar_points) > 0:
                min_dist = np.min(np.linalg.norm(lidar_points - self.pose[:2], axis=1))
            if min_dist < self.safe_distance: target_turn = self.turn_step
            else: target_speed = self.speed_step; target_turn = np.random.normal(0, 0.05)
        self.pose[2] += target_turn
        self.pose[0] += target_speed * np.cos(self.pose[2])
        self.pose[1] += target_speed * np.sin(self.pose[2])
        
        # Enkel kollisjonssjekk mot veggene (0-4m)
        self.pose[0] = max(0.1, min(3.9, self.pose[0]))
        self.pose[1] = max(0.1, min(3.9, self.pose[1]))

# --- Helper functions ---
def fit_line_to_cluster(points):
    """
    Fits a straight line to a cluster of 2D points using linear regression (PCA).
    Returns the two extreme points projected onto this best-fit line.
    """
    if len(points) < 2:
        return [points[0], points[0]]
        
    points = np.array(points)
    
    # Check if points are mostly vertical to avoid infinite slope
    x_spread = np.max(points[:, 0]) - np.min(points[:, 0])
    y_spread = np.max(points[:, 1]) - np.min(points[:, 1])
    
    if y_spread > x_spread:
        # Fit x as a function of y
        slope, intercept = np.polyfit(points[:, 1], points[:, 0], 1)
        # Find endpoints by projecting min and max Y
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        p1 = np.array([slope * y_min + intercept, y_min])
        p2 = np.array([slope * y_max + intercept, y_max])
    else:
        # Fit y as a function of x
        slope, intercept = np.polyfit(points[:, 0], points[:, 1], 1)
        # Find endpoints by projecting min and max X
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        p1 = np.array([x_min, slope * x_min + intercept])
        p2 = np.array([x_max, slope * x_max + intercept])

    # --- Apply Angle Noise to the Fitted Line ---
    if LINE_ANGLE_NOISE_STD_DEV > 0:
        # 1. Find the center of the line
        center = (p1 + p2) / 2.0
        
        # 2. Add noise to the angle
        alpha = np.random.normal(0, LINE_ANGLE_NOISE_STD_DEV)
        
        # 3. Rotate endpoints around the center
        c, s = np.cos(alpha), np.sin(alpha)
        # Shift to origin, rotate, shift back
        v1 = p1 - center
        p1 = center + np.array([c*v1[0] - s*v1[1], s*v1[0] + c*v1[1]])
        
        v2 = p2 - center
        p2 = center + np.array([c*v2[0] - s*v2[1], s*v2[0] + c*v2[1]])
        
    return [p1, p2]

def process_lidar_data(points):
    if len(points) == 0: return []
    clusters = []; curr = [points[0]]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i]-points[i-1]) > 0.5: # 0.5m threshold (gap between walls)
            clusters.append(np.array(curr)); curr = []
        curr.append(points[i])
    clusters.append(np.array(curr))
    
    lines = []
    for c in clusters: 
        if len(c) > 2: # Need a few points to make a reliable fitted line
            lines.append(fit_line_to_cluster(c))
    return lines

# --- MAIN ---
sim = DigitalTwin()
tcp_client = AsyncTcpClient(TCP_HOST, TCP_PORT)

fig, ax = plt.subplots(figsize=(6, 6)) # Mindre vindu, passer 4x4
ax.set_aspect('equal'); ax.grid(True)
ax.set_xlim(-0.5, 4.5); ax.set_ylim(-0.5, 4.5) # Vis litt utenfor veggene

robot_marker, = ax.plot([], [], 'bo', markersize=10, zorder=10)
arrow = ax.arrow(0,0,0,0, head_width=0.2, color='k', zorder=11)
line_objects = []

def on_key(event, pressed):
    if event.key == 'm' and pressed: sim.manual_mode = not sim.manual_mode
    if event.key in sim.keys: sim.keys[event.key] = pressed
fig.canvas.mpl_connect('key_press_event', lambda e: on_key(e, True))
fig.canvas.mpl_connect('key_release_event', lambda e: on_key(e, False))
fig.canvas.mpl_connect('close_event', lambda e: setattr(tcp_client, 'running', False))

def update(frame):
    global line_objects, arrow
    raw_points = sim.get_lidar_hits()
    sim.update_physics(raw_points)
    extracted_lines = process_lidar_data(raw_points)
    
    # Konverter posisjon til CM og legg til eventuell "Location Noise" (Odometry drift)
    noisy_x = sim.pose[0]
    noisy_y = sim.pose[1]
    noisy_theta = sim.pose[2]
    
    if LOCATION_NOISE_STD_DEV > 0:
        noisy_x += np.random.normal(0, LOCATION_NOISE_STD_DEV)
        noisy_y += np.random.normal(0, LOCATION_NOISE_STD_DEV)
    if LOCATION_ANGLE_NOISE_STD_DEV > 0:
        noisy_theta += np.random.normal(0, LOCATION_ANGLE_NOISE_STD_DEV)

    x_cm = noisy_x * 100
    y_cm = noisy_y * 100
    theta_deg = np.degrees(noisy_theta)

    if len(extracted_lines) > 0 and TRACK_CLOSEST_ONLY:
        best_line = None
        min_dist = float('inf')
        
        for line in extracted_lines:
            # Orthogonal distance from robot to this fitted line segment
            p1 = np.array(line[0])
            p2 = np.array(line[1])
            robot_pt = sim.pose[:2]
            
            # Simple midpoint distance approximation for selecting "closest wall"
            midpoint = (p1 + p2) / 2.0
            dist = np.linalg.norm(midpoint - robot_pt)
            
            if dist < min_dist:
                min_dist = dist
                best_line = line
                
        extracted_lines = [best_line]

    # Send data (Posisjonene blir sentrert inni send_packet)
    if len(extracted_lines) == 0:
        tcp_client.send_packet(ROBOT_ID, x_cm, y_cm, theta_deg, np.array([0,0]), np.array([0,0]))
    else:
        for line in extracted_lines:
            dx = line[0][0] - noisy_x
            dy = line[0][1] - noisy_y
            c, s = np.cos(-noisy_theta), np.sin(-noisy_theta)
            local_start = np.array([c*dx - s*dy, s*dx + c*dy])
            
            dx2 = line[1][0] - noisy_x
            dy2 = line[1][1] - noisy_y
            local_end = np.array([c*dx2 - s*dy2, s*dx2 + c*dy2])
            
            tcp_client.send_packet(ROBOT_ID, x_cm, y_cm, theta_deg, local_start, local_end)
    
    # Visuelt i Python
    robot_marker.set_data([sim.pose[0]], [sim.pose[1]])
    arrow.remove()
    arrow = ax.arrow(sim.pose[0], sim.pose[1], 0.4*np.cos(sim.pose[2]), 0.4*np.sin(sim.pose[2]), head_width=0.2, color='black', zorder=11)
    for l in line_objects: l.remove()
    line_objects = []
    for seg in extracted_lines:
        l, = ax.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'g-', lw=2.5)
        line_objects.append(l)
    return robot_marker,

anim = FuncAnimation(fig, update, frames=None, interval=30, blit=False)
plt.title("Digital Twin")
plt.show()