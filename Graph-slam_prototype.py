import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from matplotlib.patches import Circle, Arrow

# -----------------------------
# WORLD
# -----------------------------
# [[x1, y1], [x2, y2]]
true_wall_segments = [
    np.array([[2.0, 1.0], [2.0, 4.0]]),   # Venstre vertikal vegg
    np.array([[2.0, 4.0], [6.0, 4.0]]),   # Øvre horisontal vegg
    np.array([[6.0, 4.0], [6.0, 1.0]]),   # Høyre vertikal vegg
]

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def segment_to_infinite_line(segment):
    p1 = segment[0]
    p2 = segment[1]
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    # Normalvektor
    nx = -dy
    ny = dx
    alpha = np.arctan2(ny, nx)
    rho = p1[0] * np.cos(alpha) + p1[1] * np.sin(alpha)
    if rho < 0:
        rho *= -1
        alpha = normalize_angle(alpha + np.pi)
    return np.array([rho, alpha])

true_lines = []
for seg in true_wall_segments:
    true_lines.append(segment_to_infinite_line(seg))
true_lines = np.array(true_lines)

# -----------------------------
# SIMULATED ROBOT TRAJECTORY
# -----------------------------

def simulate_robot_path():
    """
    Simulerer en mer kompleks bane for roboten.
    1. Kjører rett frem inn i rommet.
    2. Svinger 90 grader til venstre.
    3. Kjører et stykke oppover.
    4. Svinger 90 grader til venstre igjen.
    5. Svinger 180 grader til høyre for å komme tilbake mot utgangen.
    """
    poses = []
    x, y, th = 0.0, 0.0, 0.0
    poses.append([x, y, th])

    # 1. Kjører rett frem til x=4
    for _ in range(10):
        x += 0.4
        poses.append([x, y, th])

    # 2. Svinger 90 grader (pi/2) mot venstre (mot positiv y)
    for _ in range(5):
        th += np.pi / 2 / 5
        poses.append([x, y, th])
    th = np.pi/2 # Korriger til nøyaktig 90 grader
    poses[-1] = [x,y,th]

    # 3. Kjører rett frem til y=2.5
    for _ in range(6):
        y += 0.4
        poses.append([x, y, th])

    # 4. Svinger 90 grader mot venstre (mot negativ x)
    for _ in range(5):
        th += np.pi / 2 / 5
        poses.append([x, y, th])
    th = np.pi # Korriger til nøyaktig 180 grader
    poses[-1] = [x,y,th]

    # 4. Svinger 180 grader mot høyre (mot positiv x)
    for _ in range(10):
        th -= np.pi / 10
        poses.append([x, y, th])
    th = 0 # Korriger til nøyaktig 0 grader (mot positiv x)
    poses[-1] = [x,y,th]

    return np.array(poses)

true_poses = simulate_robot_path()

# -----------------------------
# ODOM WITH DRIFT
# -----------------------------

def simulate_odometry(true_poses):
    noisy = []
    prev_true_pose = true_poses[0]
    noisy_pose = prev_true_pose.copy()
    noisy.append(noisy_pose)

    for curr_true_pose in true_poses[1:]:
        # 1. Sann lokal bevegelse
        true_global_motion = curr_true_pose - prev_true_pose
        th_prev_true = prev_true_pose[2]
        c, s = np.cos(th_prev_true), np.sin(th_prev_true)
        true_local_dx = true_global_motion[0] * c + true_global_motion[1] * s
        true_local_dy = -true_global_motion[0] * s + true_global_motion[1] * c
        true_local_dth = normalize_angle(true_global_motion[2])

        # 2. Legg til støy på lokal bevegelse
        noisy_local_motion = np.array([true_local_dx, true_local_dy, true_local_dth]) + np.random.normal(0, [0.03, 0.03, 0.01])

        # 3. Oppdater støyete globale pose ved å bruke støyete lokal bevegelse (KORREKT MÅTE)
        prev_noisy_pose = noisy[-1]
        th_prev_noisy = prev_noisy_pose[2]
        noisy_pose = prev_noisy_pose + np.array([noisy_local_motion[0]*np.cos(th_prev_noisy) - noisy_local_motion[1]*np.sin(th_prev_noisy),
                                                 noisy_local_motion[0]*np.sin(th_prev_noisy) + noisy_local_motion[1]*np.cos(th_prev_noisy),
                                                 noisy_local_motion[2]])
        noisy.append(noisy_pose)
        prev_true_pose = curr_true_pose
        
    return np.array(noisy)

odom_poses = simulate_odometry(true_poses)

# -----------------------------
# LINE OBSERVATION MODEL
# -----------------------------

def observe_line(pose, line):
    x, y, th = pose
    rho, alpha = line
    
    alpha_rel = normalize_angle(alpha - th)
    
    rho_rel = rho - (x * np.cos(alpha) + y * np.sin(alpha))
    
    # Add measurement noise
    rho_rel += np.random.normal(0, 0.05)
    alpha_rel += np.random.normal(0, 0.02)
    
    return np.array([rho_rel, alpha_rel])

# generate measurements
measurements = []

for i, pose in enumerate(true_poses):
    for j, line in enumerate(true_lines):
        z = observe_line(pose, line)
        measurements.append((i, j, z))

# -----------------------------
# GRAPH-SLAM OPTIMIZATION
# -----------------------------

num_poses = len(odom_poses)
num_lines = len(true_lines)

def pack_state(poses, lines):
    return np.hstack([poses.flatten(), np.array(lines).flatten()])

def unpack_state(state):
    poses = state[:num_poses*3].reshape((-1,3))
    lines = state[num_poses*3:].reshape((-1,2))
    return poses, lines

def residuals(state):
    poses, lines = unpack_state(state)
    res = []

    res.extend(poses[0] - [0, 0, 0])
    
    # Odometry constraints
    for i in range(1, num_poses):
        pose_prev, pose_curr = poses[i-1], poses[i]
        odom_prev, odom_curr = odom_poses[i-1], odom_poses[i]

        # Målt lokal bevegelse fra odometri
        meas_global_motion = odom_curr - odom_prev
        th_odom_prev = odom_prev[2]
        c, s = np.cos(th_odom_prev), np.sin(th_odom_prev)
        meas_local_dx = meas_global_motion[0] * c + meas_global_motion[1] * s
        meas_local_dy = -meas_global_motion[0] * s + meas_global_motion[1] * c
        meas_local_dth = normalize_angle(meas_global_motion[2])

        # Predikert lokal bevegelse fra våre estimerte poser
        pred_global_motion = pose_curr - pose_prev
        th_pred_prev = pose_prev[2]
        c, s = np.cos(th_pred_prev), np.sin(th_pred_prev)
        pred_local_dx = pred_global_motion[0] * c + pred_global_motion[1] * s
        pred_local_dy = -pred_global_motion[0] * s + pred_global_motion[1] * c
        pred_local_dth = normalize_angle(pred_global_motion[2])
        
        res.extend([pred_local_dx - meas_local_dx, pred_local_dy - meas_local_dy, normalize_angle(pred_local_dth - meas_local_dth)])
    
    # Line observation constraints
    for (i, j, z) in measurements:
        pose = poses[i]
        line = lines[j]
        z_rho, z_alpha = z
        
        x, y, th = pose
        est_rho, est_alpha = line
        
        pred_alpha = normalize_angle(z_alpha + th)
        pred_rho = x * np.cos(pred_alpha) + y * np.sin(pred_alpha) + z_rho
        
        res.extend([pred_rho - est_rho, normalize_angle(pred_alpha - est_alpha)])
        
    return np.array(res)

# initial guess
init_lines = [[1.0, 0.1], [5.0, 0.1], [3.0, np.pi/3]]
#init_lines = true_lines + np.random.normal(0, 0.5, true_lines.shape)

state0 = pack_state(odom_poses, init_lines)

#state0 = np.hstack([odom_poses.flatten(), np.array([[1,0],[5,0],[3,1]]).flatten()])
result = least_squares(residuals, state0, loss='huber', f_scale=0.1)

opt_poses, opt_lines = unpack_state(result.x)

# -----------------------------
# MAPPING (MED RAYCASTING / LiDAR-SIMULERING)
# -----------------------------

def ray_segment_intersection(ray_origin, ray_dir, segment):
    """Finner skjæringspunktet mellom en stråle og et linjestykke."""
    v1 = ray_origin - segment[0]
    v2 = segment[1] - segment[0]
    v3 = np.array([-ray_dir[1], ray_dir[0]])
    
    dot = np.dot(v2, v3)
    if abs(dot) < 1e-6:
        return None
    
    t1 = np.cross(v2, v1) / dot
    t2 = np.dot(v1, v3) / dot  
    
    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return ray_origin + t1 * ray_dir
    return None

def simulate_lidar_scan(pose, true_segments, num_rays=60, fov=np.pi/2):
    """Skyter ut stråler fra roboten og returnerer treffpunkter (point cloud)."""
    x, y, th = pose
    ray_origin = np.array([x, y])
    hit_points = []
    
    angles = np.linspace(th - fov/2, th + fov/2, num_rays)
    for angle in angles:
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        closest_hit = None
        min_dist = float('inf')
        
        for segment in true_segments:
            hit = ray_segment_intersection(ray_origin, ray_dir, segment)
            if hit is not None:
                dist = np.linalg.norm(hit - ray_origin)
                if dist < min_dist:
                    min_dist = dist
                    closest_hit = hit
                    
        if closest_hit is not None:
            hit_points.append(closest_hit)
            
    return hit_points

all_lidar_hits = []
for pose in true_poses: 
    hits = simulate_lidar_scan(pose, true_wall_segments)
    all_lidar_hits.extend(hits)
all_lidar_hits = np.array(all_lidar_hits)

def extract_segment_from_hits(line, lidar_hits, distance_threshold=0.5):
    if len(lidar_hits) == 0:
        return np.array([[0,0], [0,0]])
        
    rho, alpha = line
    c, s = np.cos(alpha), np.sin(alpha)
    p0 = np.array([c*rho, s*rho]) # Punkt på linjen nærmest origo
    u = np.array([-s, c])         # Retningsvektor langs linjen
    normal = np.array([c, s])     # Normalvektor
    
    distances = np.abs(np.dot(lidar_hits - p0, normal))
    relevant_hits = lidar_hits[distances < distance_threshold] 
    
    if len(relevant_hits) == 0:
        return np.array([[0,0], [0,0]])
        
    projections = np.dot(relevant_hits - p0, u)
    
    min_proj = np.min(projections)
    max_proj = np.max(projections)
    
    p_min = p0 + min_proj * u
    p_max = p0 + max_proj * u
    
    return np.array([p_min, p_max])

opt_wall_segments = []
for j, line in enumerate(opt_lines):
    opt_wall_segments.append(extract_segment_from_hits(line, all_lidar_hits))

# -----------------------------
# PLOT
# -----------------------------

# def plot_line(rho, alpha, style, label=None):
#     angle = alpha
#     c = np.cos(angle)
#     s = np.sin(angle)
#     x0 = c * rho
#     y0 = s * rho
#     x1 = x0 + 1000 * (-s)
#     y1 = y0 + 1000 * (c)
#     x2 = x0 - 1000 * (-s)
#     y2 = y0 - 1000 * (c)
#     plt.plot([x1, x2], [y1, y2], style, alpha=0.5, label=label)

# plt.figure(figsize=(10,6))

# for i, line in enumerate(true_lines):
#     plot_line(line[0], line[1], 'b--', label="Sann vegg" if i==0 else "")

# for i, line in enumerate(opt_lines):
#     plot_line(line[0], line[1], 'r-', label="Estimert vegg" if i==0 else "")

# plt.plot(true_poses[:,0], true_poses[:,1], 'b--', label="True")
# plt.plot(odom_poses[:,0], odom_poses[:,1], 'g--',  label="Odom")
# plt.plot(opt_poses[:,0], opt_poses[:,1], 'r--', label="Graph-SLAM")

# plt.xlim(-2, 10); plt.ylim(-5, 5)
# plt.legend()
# plt.title("Graph-SLAM Prototype")
# plt.show()



# -----------------------------
# PLOT (Mapping med linjestykker)
# -----------------------------
fig, ax = plt.subplots(figsize=(8,8))

# 1. Tegn sanne vegger (Blå, stiplede)
for seg in true_wall_segments:
    ax.plot(seg[:,0], seg[:,1], 'b--', alpha=0.5, label="Sann Vegg" if np.all(seg==true_wall_segments[0]) else "")

# 2. Tegn estimerte vegger (Røde, solide linjestykker)
for seg in opt_wall_segments:
    ax.plot(seg[:,0], seg[:,1], 'r-', label="Estimert Vegg (Mapping)" if np.all(seg==opt_wall_segments[0]) else "")

# 3. Tegn roboten på siste posisjon (blå sirkel med pil)
last_pose = opt_poses[-1]
ax.add_patch(Circle((last_pose[0], last_pose[1]), 0.1, color='blue', label="Robot"))
# Legg til pil for retning
arrow_len = 0.4
dx = arrow_len * np.cos(last_pose[2])
dy = arrow_len * np.sin(last_pose[2])
ax.add_patch(Arrow(last_pose[0], last_pose[1], dx, dy, width=0.1, color='black'))

# 4. Tegn robotens korrigerte bane (rød stiplede)
ax.plot(opt_poses[:,0], opt_poses[:,1], 'r--', label="Graph-SLAM Bane")
ax.plot(odom_poses[:,0], odom_poses[:,1], 'g--', alpha=0.3, label="Odometri")
ax.plot(true_poses[:,0], true_poses[:,1], 'b-', alpha=0.5, label="Sann Bane")

# Zoom og rutenett
ax.set_xlim(-1, 8); ax.set_ylim(-1, 8)
ax.set_aspect('equal')
ax.grid(True)
ax.legend(loc='upper right', fontsize='small')
ax.set_title("SLAM med Uendelige Linjer & Mapping med Linjestykker")
plt.show()