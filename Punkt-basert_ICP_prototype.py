import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.optimize import least_squares
from matplotlib.patches import Rectangle, Circle

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ==========================================
# 1. MILJØ, VEGGER & SIMULERING (Skalert til 4x4m)
# ==========================================
ROOM_SIZE = 4.0
# Skalerte ned posisjonene til landemerkene
TRUE_LANDMARKS = {0: np.array([1.0, 1.0]), 1: np.array([3.0, 3.0]), 2: np.array([1.0, 3.0])}
OBJ_RADIUS = 0.15 # Halvert størrelse på objektene
MAX_RANGE = 3.0 # Beholdt 3m rekkevidde (Ser nå mye lenger relativt til rommet!)
TOF_FOV = np.deg2rad(27)
CAM_FOV = np.deg2rad(60)

# Skalert ned alle vegger med 50%
TRUE_WALLS = np.array([
    [[0.0, 0.0], [4.0, 0.0]],
    [[4.0, 0.0], [4.0, 4.0]],
    [[4.0, 4.0], [0.0, 4.0]],
    [[0.0, 4.0], [0.0, 0.0]],
    #[[2.0, 0.0], [2.0, 1.75]],  
    #[[2.0, 2.5], [2.0, 4.0]],  
    #[[0.0, 2.0], [1.0, 2.0]]   
])

def raycast_to_walls(pose, angles):
    x, y, th = pose
    ray_origin = np.array([x, y])
    min_dist = MAX_RANGE + 1.0
    
    for angle in angles:
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        for wall in TRUE_WALLS:
            A, B = wall[0], wall[1]
            v1 = ray_origin - A
            v2 = B - A
            v3 = np.array([-ray_dir[1], ray_dir[0]])
            
            dot = np.dot(v2, v3)
            if abs(dot) < 1e-6: continue
            
            cross = v2[0] * v1[1] - v2[1] * v1[0]
            t = cross / dot             
            u = np.dot(v1, v3) / dot    
            
            if t > 0.01 and 0.0 <= u <= 1.0:
                if t < min_dist: min_dist = t
    return min_dist

# ==========================================
# 2. ICP SCAN MATCHER & OCCUPANCY GRID MOTOR
# ==========================================
class Submap:
    def __init__(self, origin_pose):
        self.origin = origin_pose.copy() # Hvor i verden startet dette kartet?
        self.local_points = np.empty((0, 2)) # Punkter RELATIVT til origin
        self.scans = [] # Lagrer rådata for å kunne tegne Occupancy Grid på nytt

class GlobalSLAM:
    # Satt grid_size til 6.0 for å passe det nye 4x4 rommet med litt margin
    def __init__(self, grid_size=6.0, grid_res=0.05): 
        # Start i hjørnet av det nye rommet
        start_pose = np.array([0.5, 0.5, 0.5]) 
        self.submaps = [Submap(start_pose)]
        self.poses = [start_pose]
        self.odom_poses = [start_pose]
        
        self.edges_odom = [] 
        self.edges_loop = [] 
        
        self.grid_res = grid_res 
        self.grid_size = grid_size
        self.grid_w = int(grid_size / grid_res)
        self.grid_h = int(grid_size / grid_res)
        self.grid = np.zeros((self.grid_h, self.grid_w))
        
        self.last_icp_pose = start_pose.copy()
        self.DIST_THRESH = 0.05
        self.ANGLE_THRESH = 0.10
        
        self.SUBMAP_RADIUS = 1.45

    def get_local_scan(self, tof_dist, nicla_data=None):
        local_pts = []
        if 0.2 < tof_dist < 3.0:
            local_pts.append([tof_dist, 0.0]) 
        if nicla_data:
            for dy in np.linspace(nicla_data['start_y'], nicla_data['start_y'] + nicla_data['width'], 5):
                local_pts.append([nicla_data['dist'], dy]) 
        return np.array(local_pts)

    def icp_match(self, local_points, global_guess, target_global_points, max_iter=15, is_loop_closure=False):
        if len(target_global_points) < 15 or len(local_points) < 4:
            return global_guess, False 
            
        pose = global_guess.copy()
        kdtree = KDTree(target_global_points) 
        
        for i in range(max_iter):
            c, s = np.cos(pose[2]), np.sin(pose[2])
            R = np.array([[c, -s], [s, c]])
            global_pts = np.dot(local_points, R.T) + pose[:2]
            
            distances, indices = kdtree.query(global_pts)
            valid = distances < 0.3 
            if not np.any(valid): break
            
            P = local_points[valid]
            Q = target_global_points[indices[valid]]
            
            # KVALITETSKONTROLL: Hvor mange punkter MÅ overlappe?
            min_match_points = 200 if is_loop_closure else 4
            if len(P) < min_match_points: 
                break # Avbryt hvis vi ikke har nok sikre treff!
            
            cP, cQ = np.mean(P, axis=0), np.mean(Q, axis=0)
            W = np.dot((P - cP).T, (Q - cQ))
            U, _, Vt = np.linalg.svd(W)
            R_opt = np.dot(Vt.T, U.T)
            
            if np.linalg.det(R_opt) < 0:
                Vt[1, :] *= -1
                R_opt = np.dot(Vt.T, U.T)
                
            t_opt = cQ - np.dot(cP, R_opt.T)
            pose[0], pose[1], pose[2] = t_opt[0], t_opt[1], np.arctan2(R_opt[1, 0], R_opt[0, 0])
            
       # ==========================================
        # FYSIKK-FILTER (Non-holonomic & Loop Checks)
        # ==========================================
        dx = pose[0] - global_guess[0]
        dy = pose[1] - global_guess[1]
        jump_angle = abs(normalize_angle(pose[2] - global_guess[2]))
        
        if is_loop_closure:
            jump_dist = np.hypot(dx, dy)
            # Vi øker grensen! Map-to-Map er robust, så vi tillater inntil 2m og 30 grader korreksjon
            if jump_dist > 2.0 or jump_angle > 0.5: 
                return global_guess, False
        else:
            c_th = np.cos(global_guess[2])
            s_th = np.sin(global_guess[2])
            
            jump_forward = abs(dx * c_th + dy * s_th)
            jump_sideways = abs(-dx * s_th + dy * c_th)
            
            if jump_sideways > 0.10 or jump_forward > 0.15 or jump_angle > 0.02:
                return global_guess, False 
                
        return pose, True

    def get_relative_pose(self, p1, p2):
        c, s = np.cos(p1[2]), np.sin(p1[2])
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        return np.array([dx*c + dy*s, -dx*s + dy*c, normalize_angle(p2[2] - p1[2])])

    def add_to_occupancy_grid(self, pose, dist_m):
        if dist_m < 0.2 or dist_m > 3.0: return
        rx, ry = int((pose[0]+1.0)/self.grid_res), int((pose[1]+1.0)/self.grid_res)
        angles = np.linspace(-np.deg2rad(27)/2, np.deg2rad(27)/2, 5)
        for angle in angles:
            hx, hy = int((pose[0] + dist_m*np.cos(pose[2]+angle)+1.0)/self.grid_res), int((pose[1] + dist_m*np.sin(pose[2]+angle)+1.0)/self.grid_res)
            steps = int(np.hypot(hx-rx, hy-ry))
            if steps == 0: continue
            for i in range(steps):
                cx, cy = int(rx + (hx-rx)*(i/steps)), int(ry + (hy-ry)*(i/steps))
                if 0 <= cx < self.grid_w and 0 <= cy < self.grid_h: self.grid[cy, cx] += -0.4
            if 0 <= hx < self.grid_w and 0 <= hy < self.grid_h: self.grid[hy, hx] += 0.9
        self.grid = np.clip(self.grid, -5.0, 5.0)

    def optimize_graph(self):
        print("\n🚀 LOOP CLOSURE! Kjører Graph-SLAM...")
        def residuals(state):
            res = []
            origins = state.reshape((-1, 3))
            # Lås startpunktet fast i betong (ganger med 1000)
            res.extend((origins[0] - self.submaps[0].origin) * 1000) 
            
            # 1. ODOMETRI-STRIKKER (Veldig stive på rotasjon!)
            for (i, j, rel) in self.edges_odom:
                pi, pj = origins[i], origins[j]
                c, s = np.cos(pi[2]), np.sin(pi[2])
                pred_dx = (pj[0] - pi[0])*c + (pj[1] - pi[1])*s
                pred_dy = -(pj[0] - pi[0])*s + (pj[1] - pi[1])*c
                pred_dth = normalize_angle(pj[2] - pi[2])
                
                # Ganger vinkelfeilen med 2. Graph-SLAM vil "hate" å vri på kartet.
                res.extend([ (pred_dx - rel[0]) * 2.0, 
                             (pred_dy - rel[1]) * 2.0, 
                             normalize_angle(pred_dth - rel[2]) * 4.0 ])
                
            # 2. LOOP CLOSURE-STRIKKER (Litt mykere, de skal bare dra ting på plass)
            for (i, j, rel) in self.edges_loop:
                pi, pj = origins[i], origins[j]
                c, s = np.cos(pi[2]), np.sin(pi[2])
                pred_dx = (pj[0] - pi[0])*c + (pj[1] - pi[1])*s
                pred_dy = -(pj[0] - pi[0])*s + (pj[1] - pi[1])*c
                pred_dth = normalize_angle(pj[2] - pi[2])
                
                res.extend([ pred_dx - rel[0], 
                             pred_dy - rel[1], 
                             normalize_angle(pred_dth - rel[2]) * 2.0 ])
                             
            return np.array(res)

        init_state = np.array([sm.origin for sm in self.submaps]).flatten()
        result = least_squares(residuals, init_state, loss='huber')
        opt_origins = result.x.reshape((-1, 3))
        
        for i, sm in enumerate(self.submaps):
            sm.origin = opt_origins[i]
            
        print("🧹 Sletter gammelt rutenett og tegner korrigert kart...")
        self.grid = np.zeros((self.grid_h, self.grid_w))
        for sm in self.submaps:
            c, s = np.cos(sm.origin[2]), np.sin(sm.origin[2])
            for (rel_x, rel_y, rel_th, dist) in sm.scans:
                global_pose = np.array([sm.origin[0] + rel_x*c - rel_y*s, sm.origin[1] + rel_x*s + rel_y*c, normalize_angle(sm.origin[2] + rel_th)])
                self.add_to_occupancy_grid(global_pose, dist)
        print("✅ Ferdig!")

    def update(self, dx, dy, dth, tof_dist, nicla_data=None):
        prev_p = self.poses[-1]
        guess_pose = np.array([
            prev_p[0] + dx*np.cos(prev_p[2]) - dy*np.sin(prev_p[2]),
            prev_p[1] + dx*np.sin(prev_p[2]) + dy*np.cos(prev_p[2]),
            normalize_angle(prev_p[2] + dth)
        ])
        
        prev_o = self.odom_poses[-1]
        self.odom_poses.append(np.array([prev_o[0] + dx*np.cos(prev_o[2]), prev_o[1] + dx*np.sin(prev_o[2]), normalize_angle(prev_o[2] + dth)]))

        local_pts = self.get_local_scan(tof_dist, nicla_data)
        active_sm = self.submaps[-1]
        
        dist_moved = np.hypot(guess_pose[0] - self.last_icp_pose[0], guess_pose[1] - self.last_icp_pose[1])
        angle_moved = abs(normalize_angle(guess_pose[2] - self.last_icp_pose[2]))
        
        if len(local_pts) > 0 and (dist_moved >= self.DIST_THRESH or angle_moved >= self.ANGLE_THRESH):
            # Lokal ICP Matching (Scan-to-Map)
            c, s = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
            R = np.array([[c, -s], [s, c]])
            global_map_pts = np.dot(active_sm.local_points, R.T) + active_sm.origin[:2] if len(active_sm.local_points) > 0 else np.empty((0,2))
            
            opt_pose, _ = self.icp_match(local_pts, guess_pose, global_map_pts)
            
            rel_pose = self.get_relative_pose(active_sm.origin, opt_pose)
            rel_pts = np.dot(local_pts, np.array([[np.cos(rel_pose[2]), -np.sin(rel_pose[2])], [np.sin(rel_pose[2]), np.cos(rel_pose[2])]]).T) + rel_pose[:2]
            
            active_sm.local_points = np.vstack((active_sm.local_points, rel_pts))
            active_sm.scans.append((rel_pose[0], rel_pose[1], rel_pose[2], tof_dist))
            self.add_to_occupancy_grid(opt_pose, tof_dist)
            
            guess_pose = opt_pose
            self.last_icp_pose = guess_pose 

            # =======================================================
            # 4. PAKKING AV SUBMAP OG MAP-TO-MAP LOOP CLOSURE
            # =======================================================
            dist_from_origin = np.hypot(guess_pose[0] - active_sm.origin[0], guess_pose[1] - active_sm.origin[1])
            
            # KREVER at vi både har kjørt langt nok, OG har samlet en god form (> 50 punkter)
            if dist_from_origin > self.SUBMAP_RADIUS and len(active_sm.local_points) > 50:
                current_idx = len(self.submaps) - 1
                print(f"📦 Pakker inn Submap {current_idx} (med {len(active_sm.local_points)} punkter)...")
                
                # Regn ut odometri-strikken fra dette kartet til det neste (Lokalt)
                rel_odom = self.get_relative_pose(active_sm.origin, guess_pose)
                
                # --- SJEKK LOOP CLOSURE MED DET FERDIGE KARTET FØRST! ---
                for old_idx, old_sm in enumerate(self.submaps[:-1]): 
                    if old_idx == current_idx: continue
                    
                    dist_to_old = np.hypot(active_sm.origin[0] - old_sm.origin[0], active_sm.origin[1] - old_sm.origin[1])
                    
                    # Hvis vi er i nærheten av et gammelt kart, prøv å matche hele punktskyen
                    search_radius = 3.0 if old_idx == 0 else self.SUBMAP_RADIUS
                    if dist_to_old < search_radius and not any(e[0]==old_idx and e[1]==current_idx for e in self.edges_loop):
                        c_old, s_old = np.cos(old_sm.origin[2]), np.sin(old_sm.origin[2])
                        R_old = np.array([[c_old, -s_old], [s_old, c_old]])
                        old_global_pts = np.dot(old_sm.local_points, R_old.T) + old_sm.origin[:2]
                        
                        # Matcher det aktive kartet mot det gamle kartet
                        loop_origin, success = self.icp_match(active_sm.local_points, active_sm.origin, old_global_pts, is_loop_closure=True)
                        
                        if success:
                            print(f"🔗 MAP-TO-MAP Loop Closure! Matcher Submap {current_idx} med {old_idx}")
                            rel_loop = self.get_relative_pose(old_sm.origin, loop_origin)
                            self.edges_loop.append((old_idx, current_idx, rel_loop))
                            
                            # Kjører optimalisering på de EKSISTERENDE kartene
                            self.optimize_graph() 
                            
                            # Etter Graph-SLAM har active_sm.origin flyttet seg.
                            # Oppdater bilens gjeldende posisjon slik at vi starter det nye kartet på rett plass!
                            c_new, s_new = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
                            guess_pose = np.array([
                                active_sm.origin[0] + rel_odom[0]*c_new - rel_odom[1]*s_new,
                                active_sm.origin[1] + rel_odom[0]*s_new + rel_odom[1]*c_new,
                                normalize_angle(active_sm.origin[2] + rel_odom[2])
                            ])
                            break # Tar én massiv korreksjon per pakking

                # NÅ legger vi til odometri-strikken til det neste kartet...
                self.edges_odom.append((current_idx, current_idx + 1, rel_odom))
                
                # ...og oppretter selve kartet! (Dette fikser feilen)
                self.submaps.append(Submap(guess_pose))

        self.poses.append(guess_pose)

# ==========================================
# 3. TASTATUR-KONTROLL & HOVED-LOOP
# ==========================================
slam = GlobalSLAM()
v_cmd, w_cmd = 0.0, 0.0

def on_press(event):
    global v_cmd, w_cmd
    if event.key == 'up': v_cmd = 0.15 # Tilpasset lavere fart for liten bil
    elif event.key == 'down': v_cmd = -0.15
    elif event.key == 'left': w_cmd = 0.5
    elif event.key == 'right': w_cmd = -0.5

def on_release(event):
    global v_cmd, w_cmd
    if event.key in ['up', 'down']: v_cmd = 0.0
    elif event.key in ['left', 'right']: w_cmd = 0.0

fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.mpl_connect('key_press_event', on_press)
fig.canvas.mpl_connect('key_release_event', on_release)
plt.ion()

true_pose = np.array([0.5, 0.5, 0.5])
true_poses_hist = [true_pose.copy()]
dt = 0.1 

print("\n🚗 Liten bil (20x15cm) - Max Range: 3m")
print("🕹️ Klikk på plottet og styr med piltastene!")

while plt.fignum_exists(fig.number):
    
    # A. OPPDATER "SANN" POSISJON
    new_th = normalize_angle(true_pose[2] + w_cmd * dt)
    new_x = true_pose[0] + v_cmd * np.cos(true_pose[2]) * dt
    new_y = true_pose[1] + v_cmd * np.sin(true_pose[2]) * dt
    
    if 0.2 < new_x < ROOM_SIZE-0.2 and 0.2 < new_y < ROOM_SIZE-0.2:
        true_pose = np.array([new_x, new_y, new_th])
    else:
        true_pose[2] = new_th 
    true_poses_hist.append(true_pose.copy())
    
    # B. SENSOR-STØY
    v_noisy = v_cmd + np.random.normal(0, 0.02) if v_cmd != 0 else 0
    w_noisy = w_cmd + np.random.normal(0, 0.05) if w_cmd != 0 else 0
    
    tof_angles = np.linspace(true_pose[2] - TOF_FOV/2, true_pose[2] + TOF_FOV/2, 5)
    wall_dist = raycast_to_walls(true_pose, tof_angles)
    nearest_obj_dist = MAX_RANGE + 1.0
    nearest_obj_id = None
    
    for obj_id, lm_pos in TRUE_LANDMARKS.items():
        dx, dy = lm_pos[0] - true_pose[0], lm_pos[1] - true_pose[1]
        dist_to_center = np.hypot(dx, dy)
        bearing = normalize_angle(np.arctan2(dy, dx) - true_pose[2])
        
        if abs(bearing) < (TOF_FOV/2 + np.arctan2(OBJ_RADIUS, dist_to_center)):
            dist_to_surf = dist_to_center - OBJ_RADIUS
            if 0 < dist_to_surf < nearest_obj_dist:
                nearest_obj_dist = dist_to_surf
                nearest_obj_id = obj_id

    final_tof_dist = min(wall_dist, nearest_obj_dist)
    if final_tof_dist < MAX_RANGE: final_tof_dist += np.random.normal(0, 0.02)
    
    # Hent Nicla Data (ANONYMT - Ingen ID overføres til SLAM)
    nicla_data = None
    if nearest_obj_dist < MAX_RANGE and nearest_obj_dist < wall_dist:
        lm_pos = TRUE_LANDMARKS[nearest_obj_id]
        bearing = normalize_angle(np.arctan2(lm_pos[1] - true_pose[1], lm_pos[0] - true_pose[0]) - true_pose[2])
        if abs(bearing) < CAM_FOV/2:
            bearing_noisy = bearing + np.random.normal(0, 0.02)
            
            # Regn om til Nicla's "vinkelrette linje"-format
            width_m = OBJ_RADIUS * 2
            start_y_m = final_tof_dist * np.sin(bearing_noisy) - (width_m / 2)
            nicla_data = {'dist': final_tof_dist, 'width': width_m, 'start_y': start_y_m}
            
    # C. MAT DATA INN I SLAM
    slam.update(v_noisy * dt, 0.0, w_noisy * dt, final_tof_dist, nicla_data)
            
   # D. TEGN LIVE PLOTT
    if len(slam.poses) % 2 == 0:
        ax.clear()
        
        # 1. Tegn Occupancy Grid! (Her forsvinner de skrå strekene)
        prob_grid = 1.0 - (1.0 / (1.0 + np.exp(slam.grid)))
        extent = [-1.0, -1.0 + slam.grid_w*slam.grid_res, -1.0, -1.0 + slam.grid_h*slam.grid_res]
        ax.imshow(prob_grid, cmap='Greys', origin='lower', extent=extent, vmin=0.0, vmax=1.0)
        
        # 2. Sanne Omgivelser (Blå linjer)
        for wall in TRUE_WALLS:
            ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], 'b--', linewidth=1.5)
        for id, pos in TRUE_LANDMARKS.items():
            ax.add_patch(Circle((pos[0], pos[1]), OBJ_RADIUS, color='lightblue', alpha=0.3))
            
        true_arr = np.array(true_poses_hist)
        slam_arr = np.array(slam.poses)
        odom_arr = np.array(slam.odom_poses)
        
        ax.plot(true_arr[:,0], true_arr[:,1], 'b-', alpha=0.3, label="Sann rute")
        ax.plot(odom_arr[:,0], odom_arr[:,1], 'g--', alpha=0.5, label="Ren Odometri (Drift)")
        ax.plot(slam_arr[:,0], slam_arr[:,1], 'r-', linewidth=2, label="ICP + Grid Map")
        
        # Tegn bilen tydelig og nedskalert for 4x4m rom
        curr = slam_arr[-1]
        
        # 1. Den røde glorien krympes fra 0.25 til 0.15
        ax.add_patch(Circle((curr[0], curr[1]), 0.15, color='red', alpha=0.3))
        
        # 2. Den faktiske bilen (20 x 15 cm forblir lik for realisme)
        car_rect = Rectangle((curr[0]-0.1, curr[1]-0.075), 0.2, 0.15, angle=np.rad2deg(curr[2]), color='darkred')
        ax.add_patch(car_rect)
        
        # 3. Retningspilen gjøres kortere (fra 0.4 til 0.2 meter)
        ax.plot([curr[0], curr[0] + 0.2*np.cos(curr[2])], [curr[1], curr[1] + 0.2*np.sin(curr[2])], 'y-', linewidth=2.5)
            
        ax.set_xlim(-1, ROOM_SIZE + 1); ax.set_ylim(-1, ROOM_SIZE + 1)
        ax.set_title("Sanntids Submap | ICP Lokalisering + Occupancy Grid")
        ax.legend(loc="upper left", fontsize=8)
        
        plt.pause(0.01)

plt.ioff()

