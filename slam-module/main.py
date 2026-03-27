import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Importerer de fire modulene våre
from robot import RobotDigitalTwin
from slam import GlobalSLAMBackend
from controller import HybridController
from map import Submap, get_global_occupancy_grid

# ==========================================
# 1. MILJØOPPSETT
# ==========================================
ROOM_SIZE = 4.0
# Et enkelt kvadratisk rom med en skillevegg i midten
TRUE_WALLS = np.array([
    [[0.0, 0.0], [4.0, 0.0]],
    [[4.0, 0.0], [4.0, 4.0]],
    [[4.0, 4.0], [0.0, 4.0]],
    [[0.0, 4.0], [0.0, 0.0]],
    [[2.0, 0.0], [2.0, 1.5]]  # Skillevegg
])

# Bilen skal kjøre i en firkant
WAYPOINTS = [
    [1.0, 1.0],  # 0. Startposisjon (nede til venstre)
    [1.0, 3.0],  # 1. Kjører rett opp på venstre side (trygt over veggen på y=1.5)
    [3.0, 3.0],  # 2. Krysser over rommet til høyre side
    [3.0, 1.0],  # 3. Kjører ned på høyre side av veggen
    [3.0, 3.0],  # 4. Snur og kjører opp igjen på høyre side
    [1.0, 3.0],  # 5. Krysser tilbake til venstre side
    #[1.0, 1.0]   # 6. Kjører ned til startposisjonen igjen
]

# ==========================================
# 2. INITIALISERING
# ==========================================
start_pose = [0.5, 0.5, 0.0]

# Den digitale tvillingen (Maskinvaren)
robot = RobotDigitalTwin(start_pose=start_pose, true_walls=TRUE_WALLS)

# SLAM Backend (Geometri og Posisjonering)
slam = GlobalSLAMBackend(start_pose=start_pose)

# Autonom Kontroller (Sjåføren)
controller = HybridController()

# Historikk for plotting
true_trajectory = []
ekf_trajectory = []

# Matplotlib oppsett for live animasjon
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))

# ==========================================
# 3. HOVEDLØKKE (Simulering)
# ==========================================
dt = 0.01  # 100 Hz fysikksimulering
wp_index = 0
step_counter = 0

print("🚀 Starter Autonom Digital Tvilling!")
print("Mål: Navigere firkant-mønster mens SLAM kartlegger rommet.")

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

while plt.fignum_exists(fig.number):
    
    # --- A. KONTROLLER (Sjåføren tar en beslutning) ---
    target_wp = WAYPOINTS[wp_index]
    
    # Kontrolleren bruker robotens egen EKF (opplevd posisjon) for å navigere
    omega_l, omega_r, reached_goal = controller.get_wheel_speeds(
        current_pose=robot.ekf.x[:3], 
        target_pose=target_wp, 
        dt=dt
    )
    
    if reached_goal:
        print(f"✅ Nådde Waypoint {wp_index+1}! Svinger mot neste...")
        wp_index = (wp_index + 1) % len(WAYPOINTS)
    
    # --- B. FYSIKK & MASKINVARE ---
    # Motorene får strøm, hjulene spinner, og mikrokontrolleren jobber
    payload = robot.step_physics(target_omega_l=omega_l, target_omega_r=omega_r, dt=dt)
    
    true_trajectory.append(robot.true_pose.copy())
    ekf_trajectory.append(robot.ekf.x[:3].copy())
    
    # --- C. SLAM (Oppdateres kun når roboten sender en 40ms pakke) ---
    if payload is not None:
        # 1. Få korrigert posisjon fra SLAM
        slam_pose = slam.update(payload['ekf_pose'], payload['scan'])
        
        # 2. Hvor mye stoler vi på SLAM akkurat nå? 
        # Hvis SLAM-hoppet er gigantisk, er det ofte en feilmåling (Outlier rejection)
        current_ekf_pose = robot.ekf.x[:3]
        pos_diff = np.hypot(slam_pose[0] - current_ekf_pose[0], slam_pose[1] - current_ekf_pose[1])
        angle_diff = abs(normalize_angle(slam_pose[2] - current_ekf_pose[2]))

        # Outlier rejection: Hvis hoppet er over 30cm eller 20 grader, ignorer det (støy)
        if pos_diff < 1.3 and angle_diff < np.deg2rad(45):
            
            # 3. "Soft Blending" i stedet for overskriving
            # Dette gjør at EKF-en dras mot SLAM-posisjonen uten å hoppe.
            alpha_pos = 1.0   # Hvor mye vi stoler på SLAM (0.0 - 1.0)
            alpha_theta = 1.0 # Stols mindre på vinkeloppdateringer fra SLAM (bruk heller gyro)

            new_x = (1 - alpha_pos) * current_ekf_pose[0] + alpha_pos * slam_pose[0]
            new_y = (1 - alpha_pos) * current_ekf_pose[1] + alpha_pos * slam_pose[1]
            
            # Vinkel-blending må gjøres forsiktig pga. wrapping!
            diff_theta = normalize_angle(slam_pose[2] - current_ekf_pose[2])
            new_theta = normalize_angle(current_ekf_pose[2] + alpha_theta * diff_theta)

            # Oppdater EKF-tilstanden mykt
            robot.ekf.x[0] = new_x
            robot.ekf.x[1] = new_y
            robot.ekf.x[2] = new_theta
            
        else:
            print(f"⚠️ SLAM Outlier detektert! Hopp på {pos_diff:.2f}m ignorert.")
    
    # --- D. PLOTTING (Tegner opp skjermen ca. hver 0.1 sekund) ---
    step_counter += 1
    if step_counter % 20 == 0:
        ax.clear()
        
        # 1. Tegn rommet (Fasiten)
        for wall in TRUE_WALLS:
            ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], 'k-', linewidth=2)
            
        # 2. Tegn alle Waypoints
        for i, wp in enumerate(WAYPOINTS):
            color = 'gold' if i == wp_index else 'lightgray'
            ax.plot(wp[0], wp[1], marker='*', color=color, markersize=15)
            
        # 3. Tegn baner (Historikk)
        true_arr = np.array(true_trajectory)
        ekf_arr = np.array(ekf_trajectory)
        slam_arr = np.array(slam.poses)
        
        ax.plot(true_arr[:,0], true_arr[:,1], 'b-', alpha=0.3, label="Sann rute (Fysikk)")
        ax.plot(ekf_arr[:,0], ekf_arr[:,1], 'g--', alpha=0.5, label="EKF Odometri")
        ax.plot(slam_arr[:,0], slam_arr[:,1], 'r-', linewidth=2, label="SLAM Estimering")
        
        # 4. TEGN SLAM-KARTET (Hentes nå ryddig fra map.py)
        global_grid, extent = get_global_occupancy_grid(slam.submaps, room_size=ROOM_SIZE)
        
        ax.imshow(global_grid, origin='lower', cmap='gray_r', 
                  extent=extent, vmin=0.0, vmax=1.0)

        # 5. Tegn selve robotbilen
        curr = true_arr[-1]
        car_rect = Rectangle((curr[0]-0.1, curr[1]-0.075), 0.2, 0.15, angle=np.rad2deg(curr[2]), color='blue', alpha=0.7)
        ax.add_patch(car_rect)
        ax.plot([curr[0], curr[0] + 0.2*np.cos(curr[2])], [curr[1], curr[1] + 0.2*np.sin(curr[2])], 'y-', linewidth=2)

        ax.set_xlim(-1, ROOM_SIZE + 1)
        ax.set_ylim(-1, ROOM_SIZE + 1)
        ax.set_title(f"Autonom Kjøring | Waypoint {wp_index+1}/{len(WAYPOINTS)} | Modus: {controller.mode}")
        ax.legend(loc="upper left", fontsize=8)
        
        plt.pause(0.001)

plt.ioff()