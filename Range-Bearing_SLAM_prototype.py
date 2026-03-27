import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from matplotlib.patches import Circle, Rectangle

# -----------------------------
# 1. VERDEN OG MILJØ
# -----------------------------
ROOM_SIZE = 10.0 
TRUE_LANDMARKS = {0: np.array([3.0, 3.0]), 1: np.array([7.0, 7.0]), 2: np.array([3.0, 7.0])}
OBJ_RADIUS = 0.5 

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Nye sensorbegrensninger
CAM_FOV = np.deg2rad(60)
TOF_FOV = np.deg2rad(27)
MAX_RANGE = 4.0   # Ser ikke lenger enn 4 meter
MIN_RANGE = 0.5   # Bilen kan ikke kjøre nærmere enn 0.5 meter (fysisk begrensning)

def raycast_to_walls(pose, angles):
    x, y, th = pose
    min_dist = MAX_RANGE + 1.0 # Standardverdi utenfor rekkevidde
    for angle in angles:
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        t_vals = []
        if ray_dir[0] > 0: t_vals.append((ROOM_SIZE - x) / ray_dir[0])
        if ray_dir[0] < 0: t_vals.append((0 - x) / ray_dir[0])
        if ray_dir[1] > 0: t_vals.append((ROOM_SIZE - y) / ray_dir[1])
        if ray_dir[1] < 0: t_vals.append((0 - y) / ray_dir[1])
        
        valid_t = [t for t in t_vals if t > 0]
        if valid_t: min_dist = min(min_dist, min(valid_t))
    return min_dist

# -----------------------------
# 2. ROBOTENS KINEMATIKK (Med Kollisjonsvern)
# -----------------------------
def simulate_car_movement(dt=0.5, steps=300):
    poses = [[1.0, 1.0, 0.0]] 
    odom_poses = [[1.0, 1.0, 0.0]]
    
    for i in range(steps):
        prev_x, prev_y, prev_th = poses[-1]
        
        # Sjekk avstand rett frem for å unngå kollisjon!
        dist_ahead = raycast_to_walls(poses[-1], [prev_th])
        
        if dist_ahead < MIN_RANGE:
            # For nærme veggen! Rygge og sving kraftig
            v, w = -0.1, 0.4
        else:
            if i < 40: v, w = 0.2, 0.0          # Rett frem
            elif i < 60: v, w = 0.15, 0.15      # Sving mot midten
            elif i < 90: v, w = 0.15, 0.0        # Rett frem
            elif i < 110: v, w = 0.15, -0.1     # Sving høyre
            elif i < 150: v, w = 0.05, 0.0       # Rett mot veggen
            elif i < 180: v, w = 0.1, 0.1       # Krapp sving unna veggen
            elif i < 260: v, w = 0.2, 0.1       # Kjør i en bue
            elif i < 280: v, w = 0.2, 0.0       # Rett frem
            else: v, w = 0.1, -0.15           
            
        new_th = normalize_angle(prev_th + w * dt)
        new_x = prev_x + v * np.cos(prev_th) * dt
        new_y = prev_y + v * np.sin(prev_th) * dt
        poses.append([new_x, new_y, new_th])
        
        v_noisy = v + np.random.normal(0, 0.02)
        w_noisy = w + np.random.normal(0, 0.01)
        prev_ox, prev_oy, prev_oth = odom_poses[-1]
        new_oth = normalize_angle(prev_oth + w_noisy * dt)
        new_ox = prev_ox + v_noisy * np.cos(prev_oth) * dt
        new_oy = prev_oy + v_noisy * np.sin(prev_oth) * dt
        odom_poses.append([new_ox, new_oy, new_oth])
        
    return np.array(poses), np.array(odom_poses)

true_poses, odom_poses = simulate_car_movement()

# -----------------------------
# 3. SENSOR MODELL (OBJEKTER + VEGGER)
# -----------------------------
CAM_FOV = np.deg2rad(60)
TOF_FOV = np.deg2rad(27)
MAX_RANGE = 5.0

def raycast_to_walls(pose, angles):
    x, y, th = pose
    min_dist = MAX_RANGE
    for angle in angles:
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        t_vals = []
        if ray_dir[0] > 0: t_vals.append((ROOM_SIZE - x) / ray_dir[0])
        if ray_dir[0] < 0: t_vals.append((0 - x) / ray_dir[0])
        if ray_dir[1] > 0: t_vals.append((ROOM_SIZE - y) / ray_dir[1])
        if ray_dir[1] < 0: t_vals.append((0 - y) / ray_dir[1])
        
        valid_t = [t for t in t_vals if t > 0]
        if valid_t: min_dist = min(min_dist, min(valid_t))
    return min_dist

def get_sensor_readings(pose):
    x, y, th = pose
    obj_readings = []
    wall_readings = []
    
    # 1. Avstand til nærmeste vegg med ToF
    tof_angles = np.linspace(th - TOF_FOV/2, th + TOF_FOV/2, 10)
    nearest_wall_dist = raycast_to_walls(pose, tof_angles)
    
    # 2. Avstand til nærmeste objekt med ToF
    nearest_obj_dist = MAX_RANGE
    nearest_obj_id = None
    for obj_id, lm_pos in TRUE_LANDMARKS.items():
        dx = lm_pos[0] - x; dy = lm_pos[1] - y
        dist_to_center = np.hypot(dx, dy)
        bearing = normalize_angle(np.arctan2(dy, dx) - th)
        
        if abs(bearing) < (TOF_FOV/2 + np.arctan2(OBJ_RADIUS, dist_to_center)):
            dist_to_surf = dist_to_center - OBJ_RADIUS
            if 0 < dist_to_surf < nearest_obj_dist:
                nearest_obj_dist = dist_to_surf
                nearest_obj_id = obj_id

    # 3. Logikk: Hva traff sensoren først?
    if nearest_wall_dist < nearest_obj_dist and nearest_wall_dist < MAX_RANGE:
        # Vegg er nærmest! Generer en vegg-patch
        dist_m = nearest_wall_dist + np.random.normal(0, 0.03)
        patch_width = 2 * dist_m * np.tan(TOF_FOV / 2) # Bredden på kjeglen
        wall_readings.append({
            'Type': 'Wall',
            'DistanceMM': int(dist_m * 1000),
            'WidthMM': int(patch_width * 1000),
            'StartMM': int(-patch_width/2 * 1000) # Senter av ToF er midt foran bilen
        })
        
    elif nearest_obj_dist < MAX_RANGE:
        # Objekt er nærmest! Sjekk om kameraet også ser det for ID
        dist_m = nearest_obj_dist + np.random.normal(0, 0.03)
        lm_pos = TRUE_LANDMARKS[nearest_obj_id]
        bearing = normalize_angle(np.arctan2(lm_pos[1] - y, lm_pos[0] - x) - th)
        
        if abs(bearing) < CAM_FOV/2: # Kameraet ser det
            width_mm = int((OBJ_RADIUS * 2) * 1000)
            y_offset = np.hypot(lm_pos[0]-x, lm_pos[1]-y) * np.sin(bearing + np.random.normal(0, 0.02))
            start_mm = int((y_offset * 1000) - (width_mm / 2))
            
            obj_readings.append({
                'Type': 'Object', 'Id': nearest_obj_id, 
                'DistanceMM': int(dist_m * 1000), 'WidthMM': width_mm, 'StartMM': start_mm
            })
            
    return obj_readings, wall_readings

all_obj_measurements = []
all_wall_measurements = []
for i, pose in enumerate(true_poses):
    objs, walls = get_sensor_readings(pose)
    for o in objs: all_obj_measurements.append((i, o))
    for w in walls: all_wall_measurements.append((i, w))

# -----------------------------
# 4. GRAPH-SLAM OPTIMALISERING (Kun for objekter)
# -----------------------------
def nicla_to_range_bearing(nicla_data, assumed_radius=OBJ_RADIUS):
    dist_m = nicla_data['DistanceMM'] / 1000.0
    start_m = nicla_data['StartMM'] / 1000.0
    width_m = nicla_data['WidthMM'] / 1000.0
    y_center_local = start_m + (width_m / 2.0)
    x_center_local = dist_m + assumed_radius 
    return np.hypot(x_center_local, y_center_local), np.arctan2(y_center_local, x_center_local)

def residuals(state, num_poses, num_landmarks):
    poses = state[:num_poses*3].reshape((-1, 3))
    landmarks = state[num_poses*3:].reshape((num_landmarks, 2))
    res = []
    
    res.extend((poses[0] - [odom_poses[0][0], odom_poses[0][1], odom_poses[0][2]]) * 100)
    
    for i in range(1, num_poses):
        p_prev, p_curr = poses[i-1], poses[i]
        o_prev, o_curr = odom_poses[i-1], odom_poses[i]
        
        c, s = np.cos(o_prev[2]), np.sin(o_prev[2])
        m_dx = (o_curr[0] - o_prev[0])*c + (o_curr[1] - o_prev[1])*s
        m_dy = -(o_curr[0] - o_prev[0])*s + (o_curr[1] - o_prev[1])*c
        m_dth = normalize_angle(o_curr[2] - o_prev[2])
        
        c, s = np.cos(p_prev[2]), np.sin(p_prev[2])
        p_dx = (p_curr[0] - p_prev[0])*c + (p_curr[1] - p_prev[1])*s
        p_dy = -(p_curr[0] - p_prev[0])*s + (p_curr[1] - p_prev[1])*c
        p_dth = normalize_angle(p_curr[2] - p_prev[2])
        
        res.extend([p_dx - m_dx, p_dy - m_dy, normalize_angle(p_dth - m_dth)])

    for pose_idx, meas in all_obj_measurements:
        pose = poses[pose_idx]
        lm = landmarks[meas['Id']]
        r_z, b_z = nicla_to_range_bearing(meas)
        
        pred_dx, pred_dy = lm[0] - pose[0], lm[1] - pose[1]
        pred_r = np.hypot(pred_dx, pred_dy)
        pred_b = normalize_angle(np.arctan2(pred_dy, pred_dx) - pose[2])
        
        res.extend([(pred_r - r_z), normalize_angle(pred_b - b_z)])
        
    return np.array(res)

init_landmarks = np.array([[5.0, 5.0], [5.0, 5.0], [5.0, 5.0]]) 
state0 = np.hstack([odom_poses.flatten(), init_landmarks.flatten()])
result = least_squares(residuals, state0, args=(len(odom_poses), len(TRUE_LANDMARKS)), loss='huber')

opt_poses = result.x[:len(odom_poses)*3].reshape((-1, 3))
opt_landmarks = result.x[len(odom_poses)*3:].reshape((-1, 2))

# -----------------------------
# 5. MAPPING / SKULPTURERING
# -----------------------------
def get_nicla_segment_global(pose, data):
    """ Tegner den rette Nicla-streken for objekter """
    x, y, th = pose
    dist = data['DistanceMM'] / 1000.0
    start = data['StartMM'] / 1000.0
    width = data['WidthMM'] / 1000.0
    
    c, s = np.cos(th), np.sin(th)
    gx1 = x + (dist * c - start * s)
    gy1 = y + (dist * s + start * c)
    gx2 = x + (dist * c - (start + width) * s)
    gy2 = y + (dist * s + (start + width) * c)
    return np.array([[gx1, gy1], [gx2, gy2]])

def get_tof_arc_global(pose, data):
    """ Tegner usikkerhets-buen (27 grader) for ToF-målinger mot vegger """
    x, y, th = pose
    dist = data['DistanceMM'] / 1000.0
    
    # Generer punkter langs en bue fra -13.5 til +13.5 grader
    arc_angles = np.linspace(-TOF_FOV/2, TOF_FOV/2, 10)
    arc_points = []
    for angle in arc_angles:
        ray_angle = th + angle
        px = x + dist * np.cos(ray_angle)
        py = y + dist * np.sin(ray_angle)
        arc_points.append([px, py])
        
    return np.array(arc_points)

# # -----------------------------
# # 6. VEGG-EKSTRAHERING (RANSAC)
# # -----------------------------
# def get_wall_point_cloud(poses, measurements, min_range, max_range):
#     """ Gjør ToF-målingene om til en sky av (x,y) punkter """
#     points = []
#     for pose_idx, meas in measurements:
#         dist = meas['DistanceMM'] / 1000.0
#         if min_range <= dist <= max_range:
#             x, y, th = poses[pose_idx]
#             # Vi forenkler ved å anta at det korteste treffet er rett foran bilen
#             px = x + dist * np.cos(th)
#             py = y + dist * np.sin(th)
#             points.append([px, py])
#     return np.array(points)

# def extract_walls_ransac(points, distance_threshold=0.25, iterations=200, min_inliers=15):
#     """ Finner rette linjer (vegger) i punktskyen """
#     extracted_segments = []
#     remaining_points = points.copy()
    
#     while len(remaining_points) > min_inliers:
#         best_inliers = []
#         best_line = None
        
#         for _ in range(iterations):
#             # 1. Velg to tilfeldige punkter
#             idx = np.random.choice(len(remaining_points), 2, replace=False)
#             p1, p2 = remaining_points[idx]
            
#             # 2. Beregn linjens ligning: ax + by + c = 0
#             dx = p2[0] - p1[0]
#             dy = p2[1] - p1[1]
#             length = np.hypot(dx, dy)
#             if length < 1e-6: continue
            
#             # Normaliser
#             a = -dy / length
#             b = dx / length
#             c = -(a * p1[0] + b * p1[1])
            
#             # 3. Mål avstanden fra alle gjenværende punkter til denne linjen
#             distances = np.abs(a * remaining_points[:, 0] + b * remaining_points[:, 1] + c)
            
#             # 4. Finn punktene som ligger nær nok linjen (inliers)
#             inliers = remaining_points[distances < distance_threshold]
            
#             # Lagre hvis dette er den beste linjen vi har sett hittil
#             if len(inliers) > len(best_inliers):
#                 best_inliers = inliers
#                 best_line = (a, b, c)
                
#         # Hvis den beste linjen vi fant har færre punkter enn kravet, er vi ferdige
#         if len(best_inliers) < min_inliers:
#             break
            
#         # 5. Lag et visuelt linjestykke av de inlierne vi fant
#         a, b, c = best_line
#         u = np.array([-b, a]) # Retningsvektor for linjen
#         p0 = np.array([-a*c, -b*c]) # Et punkt på linjen
        
#         # Prosjekter inliers ned på linjen for å finne start- og sluttpunkt
#         projections = np.dot(best_inliers - p0, u)
#         p_min = p0 + np.min(projections) * u
#         p_max = p0 + np.max(projections) * u
#         extracted_segments.append(np.array([p_min, p_max]))
        
#         # 6. Fjern inliers fra punktskyen slik at vi kan finne neste vegg
#         distances = np.abs(a * remaining_points[:, 0] + b * remaining_points[:, 1] + c)
#         remaining_points = remaining_points[distances >= distance_threshold]
        
#     return extracted_segments

# def merge_extracted_walls(segments, rho_thresh=0.8, alpha_thresh=0.3):
#     """
#     Slår sammen linjestykker som har omtrent samme vinkel (alpha) 
#     og avstand fra origo (rho).
#     """
#     merged = []
#     used = [False] * len(segments)

#     def get_polar(p_min, p_max):
#         dx = p_max[0] - p_min[0]; dy = p_max[1] - p_min[1]
#         alpha = normalize_angle(np.arctan2(dy, dx) + np.pi/2)
#         rho = p_min[0]*np.cos(alpha) + p_min[1]*np.sin(alpha)
#         if rho < 0:
#             rho = -rho; alpha = normalize_angle(alpha + np.pi)
#         return rho, alpha

#     for i in range(len(segments)):
#         if used[i]: continue
        
#         group = [segments[i]]
#         rho_i, alpha_i = get_polar(segments[i][0], segments[i][1])
#         used[i] = True
        
#         for j in range(i + 1, len(segments)):
#             if used[j]: continue
#             rho_j, alpha_j = get_polar(segments[j][0], segments[j][1])
            
#             # Er linjene parallelle og nærme hverandre?
#             d_rho = abs(rho_i - rho_j)
#             d_alpha = abs(normalize_angle(alpha_i - alpha_j))
#             d_alpha_anti = abs(normalize_angle(alpha_i - alpha_j + np.pi))
            
#             if d_rho < rho_thresh and (d_alpha < alpha_thresh or d_alpha_anti < alpha_thresh):
#                 group.append(segments[j])
#                 used[j] = True
                
#         # Regn ut gjennomsnittet av gruppen
#         all_pts = np.vstack(group)
#         sin_sum = sum([np.sin(get_polar(s[0], s[1])[1]) for s in group])
#         cos_sum = sum([np.cos(get_polar(s[0], s[1])[1]) for s in group])
#         avg_alpha = np.arctan2(sin_sum, cos_sum)
#         avg_rho = np.mean([get_polar(s[0], s[1])[0] for s in group])
        
#         # Finn ytterpunktene for den sammenslåtte veggen
#         n = np.array([np.cos(avg_alpha), np.sin(avg_alpha)])
#         v = np.array([-np.sin(avg_alpha), np.cos(avg_alpha)])
#         p0 = avg_rho * n
        
#         projs = np.dot(all_pts - p0, v)
#         merged.append(np.array([p0 + np.min(projs)*v, p0 + np.max(projs)*v]))
        
#     return merged

# # Kjør funksjonene på dataene våre!
# wall_point_cloud = get_wall_point_cloud(opt_poses, all_wall_measurements, MIN_RANGE, MAX_RANGE)
# extracted_walls = extract_walls_ransac(wall_point_cloud)
# final_walls = merge_extracted_walls(extracted_walls)

# -----------------------------
# 6. OCCUPANCY GRID MAPPING
# -----------------------------
GRID_RES = 0.1 # Størrelse på hver celle i meter (10 cm)
GRID_W = int(12.0 / GRID_RES) # Kartet er 12x12 meter
GRID_H = int(12.0 / GRID_RES)
grid = np.zeros((GRID_H, GRID_W))

# Log-Odds sannsynligheter
L_OCC = 0.9    # Pluss-poeng når sensoren treffer noe
L_FREE = -0.4  # Minus-poeng for luften mellom roboten og treffpunktet
MAX_LOG = 5.0  # Tak for å unngå uendelig sikkerhet
MIN_LOG = -5.0 # Bunn for å unngå uendelig sikkerhet

def world_to_grid(x, y):
    """ Gjør om meter til grid-indekser (med 1 meter offset for å sentrere) """
    gx = int((x + 1.0) / GRID_RES)
    gy = int((y + 1.0) / GRID_RES)
    return gx, gy

def get_ray_cells(x0, y0, x1, y1):
    """ Finner alle celler som en stråle passerer gjennom (DDA algoritme) """
    cells = []
    dist = np.hypot(x1-x0, y1-y0)
    steps = int(dist / (GRID_RES / 2.0)) # Oppløsning på strålesveipet
    if steps == 0: return [(x0, y0)]
    
    for i in range(steps + 1):
        px = x0 + (x1-x0) * (i/steps)
        py = y0 + (y1-y0) * (i/steps)
        cells.append((int(px), int(py)))
    
    # Returner unike celler i rekkefølge
    unique_cells = []
    for c in cells:
        if not unique_cells or unique_cells[-1] != c:
            unique_cells.append(c)
    return unique_cells

# Oppdater kartet for hver måling i ToF-sensoren (veggene)
for pose_idx, meas in all_wall_measurements:
    x, y, th = opt_poses[pose_idx]
    dist = meas['DistanceMM'] / 1000.0
    
    if dist < MIN_RANGE or dist > MAX_RANGE:
        continue
        
    rx, ry = world_to_grid(x, y)
    
    # Skyt ut flere stråler innenfor 27-graders kjeglen til ToF-sensoren
    ray_angles = np.linspace(-TOF_FOV/2, TOF_FOV/2, 5)
    for angle in ray_angles:
        hit_x = x + dist * np.cos(th + angle)
        hit_y = y + dist * np.sin(th + angle)
        hx, hy = world_to_grid(hit_x, hit_y)
        
        # Finn cellene strålen kutter gjennom
        ray_cells = get_ray_cells(rx, ry, hx, hy)
        
        # 1. Visker ut plassen før veggen (Free space)
        for cx, cy in ray_cells[:-1]:
            if 0 <= cx < GRID_W and 0 <= cy < GRID_H:
                grid[cy, cx] += L_FREE
                
        # 2. Forsterker selve treffpunktet (Occupied space)
        if 0 <= hx < GRID_W and 0 <= hy < GRID_H:
            grid[hy, hx] += L_OCC

# Klipp log-odds verdiene slik at de holder seg innenfor grensene
grid = np.clip(grid, MIN_LOG, MAX_LOG)

# Konverter Log-Odds til sannsynlighet (0.0 til 1.0) for plottet
prob_grid = 1.0 - (1.0 / (1.0 + np.exp(grid))) 


# -----------------------------
# PLOTTING AV OCCUPANCY GRID
# -----------------------------
fig, ax = plt.subplots(figsize=(10, 10))

# Vi viser rutenettet som et bilde. Hvit = ledig, Svart = hinder, Grå = ukjent
extent = [-1.0, -1.0 + GRID_W*GRID_RES, -1.0, -1.0 + GRID_H*GRID_RES]
# Reverserer Y-aksen for at bildet skal matche koordinatsystemet
ax.imshow(prob_grid, cmap='Greys', origin='lower', extent=extent, vmin=0.0, vmax=1.0)

# Tegn inn de sanne veggene for å sammenligne hvor bra kartet ble
ax.add_patch(Rectangle((0, 0), ROOM_SIZE, ROOM_SIZE, fill=False, edgecolor='blue', linewidth=1.5, linestyle='--', label="Sanne Vegger"))

# Plott banene oppå kartet
ax.plot(true_poses[:,0], true_poses[:,1], 'b-', alpha=0.2, label="Sann Bane")
ax.plot(opt_poses[:,0], opt_poses[:,1], 'r--', linewidth=2, label="SLAM Korrigert Bane")

# Tegn inn Nicla-objektene oppå kartet slik vi gjorde før (svart strek)
for pose_idx, meas in all_obj_measurements:
    if MIN_RANGE * 1000 <= meas['DistanceMM'] <= MAX_RANGE * 1000:
        segment = get_nicla_segment_global(opt_poses[pose_idx], meas)
        ax.plot(segment[:,0], segment[:,1], 'k-', linewidth=1.5, alpha=0.8)

ax.plot(opt_landmarks[:,0], opt_landmarks[:,1], 'rx', markersize=10, label="Estimerte Objekter")

ax.set_aspect('equal')
ax.set_xlim(-1, ROOM_SIZE + 1)
ax.set_ylim(-1, ROOM_SIZE + 1)
ax.legend(loc="upper right")
ax.grid(False) # Skrur av det vanlige rutenettet for å se Occupancy Gridet bedre
ax.set_title("Occupancy Grid Mapping med korrigert SLAM-bane")
plt.show()

# -----------------------------
# PLOTTING
# -----------------------------
fig, ax = plt.subplots(figsize=(12, 12))

# Sanne omgivelser
ax.add_patch(Rectangle((0, 0), ROOM_SIZE, ROOM_SIZE, fill=False, edgecolor='blue', linewidth=1.5, linestyle='--', label="Sanne Vegger"))
for id, pos in TRUE_LANDMARKS.items():
    ax.add_patch(Circle((pos[0], pos[1]), OBJ_RADIUS, color='lightblue', alpha=0.5))

# Mapping: Vegger (Grå farge)
for pose_idx, meas in all_wall_measurements:
    # Ikke tegn målinger som er for nærme (støy) eller for langt unna
    if MIN_RANGE * 1000 <= meas['DistanceMM'] <= MAX_RANGE * 1000:
        arc = get_tof_arc_global(opt_poses[pose_idx], meas)
        ax.plot(arc[:,0], arc[:,1], color='gray', linewidth=1.5, alpha=0.4)

# Mapping: Objekter (Svart farge)
for pose_idx, meas in all_obj_measurements:
    if MIN_RANGE * 1000 <= meas['DistanceMM'] <= MAX_RANGE * 1000:
        segment = get_nicla_segment_global(opt_poses[pose_idx], meas)
        ax.plot(segment[:,0], segment[:,1], 'k-', linewidth=1.5, alpha=0.8)

#ax.scatter(wall_point_cloud[:,0], wall_point_cloud[:,1], s=5, c='lightgray', label="Rå ToF Punktsky")

# Tegn de faktiske veggene
#for i, segment in enumerate(extracted_walls):
#    ax.plot(segment[:,0], segment[:,1], color='orange', linewidth=4, label="Ekstrahert Vegg (RANSAC)" if i==0 else "")

#for i, segment in enumerate(final_walls):
#    ax.plot(segment[:,0], segment[:,1], color='orange', linewidth=2, label="Sammenslått Vegg (Line Merging)" if i==0 else "")

# Lokalisering: Baner
ax.plot(true_poses[:,0], true_poses[:,1], 'b-', alpha=0.2, label="Sann Bane")
ax.plot(odom_poses[:,0], odom_poses[:,1], 'g--', label="Odometri")
ax.plot(opt_poses[:,0], opt_poses[:,1], 'r--', linewidth=2, label="SLAM Korrigert Bane")

ax.plot(opt_landmarks[:,0], opt_landmarks[:,1], 'rx', markersize=10, label="Estimerte Objekter")

ax.set_aspect('equal')
ax.set_xlim(-1, ROOM_SIZE + 1); ax.set_ylim(-1, ROOM_SIZE + 1)
ax.legend(loc="upper right")
ax.grid(True)
ax.set_title("SLAM og Total Mapping (Objekter + Vegger)")
plt.show()