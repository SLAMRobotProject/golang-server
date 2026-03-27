import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from matplotlib.patches import Rectangle, Circle

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ==========================================
# KONFIGURASJON AV SENSOR & SYSTEM
# ==========================================
ACTIVE_SENSOR = 'tof'  # Vector SLAM krever skarpheten fra 'lidar'
LIDAR_FOV = np.deg2rad(60)
LIDAR_RAYS = 31 
MAX_BUILD_DIST = 3.0
TOF_FOV = np.deg2rad(27)
CAM_FOV = np.deg2rad(60)

if ACTIVE_SENSOR == 'lidar':
    MAX_BUILD_DIST = 3.0
elif ACTIVE_SENSOR == 'tof':
    MAX_BUILD_DIST = 1.5

# ==========================================
# 1. MILJØ & SIMULERING (4x4m Rom)
# ==========================================
ROOM_SIZE = 4.0
MAX_RANGE = 3.0 
TRUE_LANDMARKS = {0: np.array([1.0, 1.0]), 1: np.array([3.0, 3.0]), 2: np.array([1.0, 3.0])}
OBJ_RADIUS = 0.15 

TRUE_WALLS = np.array([
    [[0.0, 0.0], [4.0, 0.0]],
    [[4.0, 0.0], [4.0, 4.0]],
    [[4.0, 4.0], [0.0, 4.0]],
    [[0.0, 4.0], [0.0, 0.0]],
    [[2.0, 0.0], [2.0, 1.5]],  
    [[2.0, 3.0], [2.0, 4.0]],  
    [[0.0, 2.0], [1.0, 2.0]]   
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

def merge_line_segments(lines, angle_thresh=np.deg2rad(20), dist_thresh=0.15):
    """ Går gjennom alle linjer og smelter sammen de som ligger oppå hverandre """
    merged = True
    current_lines = list(lines)

    while merged:
        merged = False
        new_lines = []
        used = set()

        for i in range(len(current_lines)):
            if i in used: continue
            A1, A2 = current_lines[i]
            v1 = A2 - A1
            l1 = np.linalg.norm(v1)
            if l1 < 1e-6:
                used.add(i)
                continue
            dir1 = v1 / l1

            best_merge_j = -1

            for j in range(i + 1, len(current_lines)):
                if j in used: continue
                B1, B2 = current_lines[j]
                v2 = B2 - B1
                l2 = np.linalg.norm(v2)
                if l2 < 1e-6: continue
                dir2 = v2 / l2

                # 1. Er de nesten parallelle?
                if abs(np.dot(dir1, dir2)) < np.cos(angle_thresh):
                    continue

                # 2. Er de nærme hverandre sideveis? (Perpendicular distance)
                n1 = np.array([-dir1[1], dir1[0]])
                dist_B1 = abs(np.dot(B1 - A1, n1))
                dist_B2 = abs(np.dot(B2 - A1, n1))
                if dist_B1 > dist_thresh or dist_B2 > dist_thresh:
                    continue

                # 3. Overlapper de på langs? (Projeksjon)
                proj_B1 = np.dot(B1 - A1, dir1)
                proj_B2 = np.dot(B2 - A1, dir1)
                min_B, max_B = min(proj_B1, proj_B2), max(proj_B1, proj_B2)

                if max_B > -0.2 and min_B < l1 + 0.2: # Tillat et lite gap på 20cm
                    best_merge_j = j
                    break

            if best_merge_j != -1:
                # VI FANT EN MATCH! Smelt dem sammen.
                B1, B2 = current_lines[best_merge_j]
                pts = np.array([A1, A2, B1, B2])
                
                # Finn sentrum og en gjennomsnittlig retning
                center = np.mean(pts, axis=0)
                if np.dot(dir1, dir2) < 0: dir2 = -dir2
                avg_dir = (dir1 + dir2) / np.linalg.norm(dir1 + dir2)

                # Finn de to ytterpunktene
                projs = np.dot(pts - center, avg_dir)
                new_A1 = center + avg_dir * projs[np.argmin(projs)]
                new_A2 = center + avg_dir * projs[np.argmax(projs)]

                new_lines.append((new_A1, new_A2))
                used.add(i)
                used.add(best_merge_j)
                merged = True
                
                # Legg tilbake resten av linjene
                for k in range(len(current_lines)):
                    if k not in used: new_lines.append(current_lines[k])
                break # Start while-løkken på nytt!

        if merged:
            current_lines = new_lines

    return current_lines

# ==========================================
# 2. SUBMAP (Midlertidig støpeform for linjer)
# ==========================================
class Submap:
    def __init__(self, origin_pose):
        self.origin = origin_pose.copy() 
        self.line_segments = [] # Vektorene som blir igjen for alltid!
        
        # MIDLERTIDIGE variabler (slettes for å spare minne!)
        self.raw_points = np.empty((0, 2))
        self.grid_res = 0.05
        self.grid_size = 6.0 
        self.grid_w = int(self.grid_size / self.grid_res)
        self.grid_h = int(self.grid_size / self.grid_res)
        self.local_grid = np.zeros((self.grid_h, self.grid_w))
        self.offset = self.grid_size / 2.0 

    def add_scan_to_local_grid(self, rel_pose, sensor_scan):
        """ Viskelær og penn for å rydde opp støyen før linjebygging """
        if self.local_grid is None: return
        rx = int((rel_pose[0] + self.offset) / self.grid_res)
        ry = int((rel_pose[1] + self.offset) / self.grid_res)
        
        for angle, dist_m in sensor_scan:
            if dist_m < 0.2 or dist_m > 3.0: continue
            
            hit_x = rel_pose[0] + dist_m * np.cos(rel_pose[2] + angle)
            hit_y = rel_pose[1] + dist_m * np.sin(rel_pose[2] + angle)
            hx = int((hit_x + self.offset) / self.grid_res)
            hy = int((hit_y + self.offset) / self.grid_res)
            
            steps = int(np.hypot(hx - rx, hy - ry))
            if steps == 0: continue
            
            for i in range(steps):
                cx = int(rx + (hx - rx) * (i / steps))
                cy = int(ry + (hy - ry) * (i / steps))
                if 0 <= cx < self.grid_w and 0 <= cy < self.grid_h: 
                    self.local_grid[cy, cx] -= 0.15 
                    
            if dist_m <= MAX_BUILD_DIST:
                if 0 <= hx < self.grid_w and 0 <= hy < self.grid_h: 
                    self.local_grid[hy, hx] += 0.5 * np.exp(-dist_m / 2.0)
                
        self.local_grid = np.clip(self.local_grid, -5.0, 5.0)

    def pack_and_convert_to_lines(self):
        """ STØPING: Konverter grid til linjer, og SLETT rutenettet! """
        # 1. Hent ut solide piksler
        y_idx, x_idx = np.where(self.local_grid > 0.5)
        clean_pts = []
        for y, x in zip(y_idx, x_idx):
            clean_pts.append([(x * self.grid_res) - self.offset, (y * self.grid_res) - self.offset])
        pts = np.array(clean_pts)

        # NYTT: Vi sparer på disse slik at vi kan plotte hva RANSAC faktisk ser!
        self.clean_points = pts.copy()

        # 2. RANSAC med Gap-detection
        lines = []
        min_points = 10
        distance_threshold = 0.08
        
        while len(pts) > min_points:
            best_line = None; best_inliers_idx = []
            
            for _ in range(50):
                idx = np.random.choice(len(pts), 2, replace=False)
                p1, p2 = pts[idx[0]], pts[idx[1]]
                if np.linalg.norm(p2 - p1) < 1e-6: continue
                
                v = p2 - p1
                v_norm = np.linalg.norm(v)
                n = np.array([-v[1], v[0]]) / v_norm 
                
                distances = np.abs(np.dot(pts - p1, n))
                inliers_idx = np.where(distances < distance_threshold)[0]
                
                if len(inliers_idx) > len(best_inliers_idx):
                    best_inliers_idx = inliers_idx
                    best_line = (p1, v / v_norm)
                    
            if len(best_inliers_idx) < min_points: break

            inliers = pts[best_inliers_idx]
            p1, line_dir = best_line
            projections = np.dot(inliers - p1, line_dir)
            sort_idx = np.argsort(projections)
            sorted_proj = projections[sort_idx]
            
            MIN_LINE_LENGTH = 0.30 # Nøyaktig bredden på bilen/Nicla-hinderet!
            
            segment_start_idx = 0
            for i in range(len(sorted_proj) - 1):
                if sorted_proj[i+1] - sorted_proj[i] > 0.25: # Døråpning!
                    min_t, max_t = sorted_proj[segment_start_idx], sorted_proj[i]
                    start_pt, end_pt = p1 + min_t * line_dir, p1 + max_t * line_dir
                    
                    # STRENG GRENSE HER:
                    if np.linalg.norm(end_pt - start_pt) >= MIN_LINE_LENGTH: 
                        lines.append((start_pt, end_pt))
                        
                    segment_start_idx = i + 1 
                    
            min_t, max_t = sorted_proj[segment_start_idx], sorted_proj[-1]
            start_pt, end_pt = p1 + min_t * line_dir, p1 + max_t * line_dir
            
            # STRENG GRENSE HER OGSÅ:
            if np.linalg.norm(end_pt - start_pt) >= MIN_LINE_LENGTH: 
                lines.append((start_pt, end_pt))
            
            pts = np.delete(pts, best_inliers_idx, axis=0)

        self.line_segments = lines
        
        # 3. FRIGJØR MINNE! Vi trenger aldri rådata eller piksler igjen.
        self.local_grid = None 
        self.raw_points = None

# ==========================================
# 3. GLOBAL SLAM (TRUE VECTOR MATCHING)
# ==========================================
class GlobalSLAM_V4:
    def __init__(self):
        start_pose = np.array([0.5, 0.5, 0.5]) 
        self.submaps = [Submap(start_pose.copy())]
        self.poses = [start_pose.copy()]
        self.odom_poses = [start_pose.copy()]
        
        self.edges_odom = [] 
        self.edges_loop = [] 
        
        self.last_icp_pose = start_pose.copy() 
        self.DIST_THRESH = 0.03
        self.ANGLE_THRESH = 0.05
        self.SUBMAP_RADIUS = 1.8
    
    def voxel_downsample(self, points, voxel_size=0.10):
        """ 
        Samler punkter i "bokser" (voxels) av en gitt størrelse 
        og returnerer gjennomsnittet av punktene i hver boks.
        """
        if len(points) == 0:
            return points
            
        # 1. Rund av punktene til nærmeste voxel-boks
        voxels = np.round(points / voxel_size)
        
        # 2. Finn de unike boksene, og tell hvor mange punkter som er i hver
        unique_voxels, inverse_idx, counts = np.unique(
            voxels, axis=0, return_inverse=True, return_counts=True
        )
        
        # 3. Regn ut gjennomsnittet (tyngdepunktet) for punktene i hver boks
        centroids = np.zeros((len(unique_voxels), 2))
        np.add.at(centroids, inverse_idx, points) # Lynrask NumPy-magi
        centroids = centroids / counts[:, np.newaxis]
        
        return centroids


    def point_to_line_score_fast(self, trans_pts, A_arr, dir_v_arr, n_arr, L_arr):
        """ LYNKJARP Vektor-matematikk (NumPy Broadcasting) """
        if len(A_arr) == 0 or len(trans_pts) == 0: return 0.0
        
        # Vi har nå råd til å sjekke annethvert punkt! (Mer nøyaktig)
        fast_pts = trans_pts[::2] 
        
        # Regner ut avstand fra ALLE punkter til ALLE linjer samtidig
        diff = fast_pts[:, np.newaxis, :] - A_arr[np.newaxis, :, :] # Form: (N punkter, M linjer, 2)
        
        proj = np.sum(diff * dir_v_arr[np.newaxis, :, :], axis=2) 
        dist_perp = np.abs(np.sum(diff * n_arr[np.newaxis, :, :], axis=2)) 
        
        # Hvilke punkter treffer en vegg innenfor 30cm?
        valid = (proj >= -0.2) & (proj <= L_arr[np.newaxis, :] + 0.2) & (dist_perp < 0.3)
        
        # Gaussisk poengsum
        match_vals = np.zeros_like(dist_perp)
        match_vals[valid] = np.exp(-(dist_perp[valid]**2) / 0.02) # 2 * 0.1^2 = 0.02
        
        # Finn den linjen som ga høyest score for hvert punkt
        best_matches = np.max(match_vals, axis=1) 
        return np.mean(best_matches)

    def fast_vector_correlative_match(self, active_pts, old_sm, global_guess, job_type='global'):
        old_lines = old_sm.line_segments
        if len(active_pts) == 0 or len(old_lines) == 0:
            return global_guess, False

        if job_type == 'local':
            search_xy, search_th = 0.30, 0.30
            pen_dist, pen_rot = 0.5, 1.5
            step_xy, step_th = 0.05, 0.02
            min_overlap = 0.25
        else:
            search_xy, search_th = 0.80, 0.95
            pen_dist, pen_rot = 0.05, 0.2
            step_xy, step_th = 0.15, 0.05
            min_overlap = 0.50

        best_pose = global_guess.copy()
        best_score = -np.inf
        c_old, s_old = np.cos(old_sm.origin[2]), np.sin(old_sm.origin[2])

        # =========================================================
        # PRE-COMPUTE: Regn ut linjenes egenskaper ÉN gang!
        # =========================================================
        A_list, dir_list, n_list, L_list = [], [], [], []
        for (B1, B2) in old_lines:
            g_B1 = np.array([B1[0]*c_old - B1[1]*s_old, B1[0]*s_old + B1[1]*c_old]) + old_sm.origin[:2]
            g_B2 = np.array([B2[0]*c_old - B2[1]*s_old, B2[0]*s_old + B2[1]*c_old]) + old_sm.origin[:2]
            
            v = g_B2 - g_B1
            L = np.linalg.norm(v)
            if L < 1e-6: continue
            dir_v = v / L
            
            A_list.append(g_B1)
            dir_list.append(dir_v)
            n_list.append(np.array([-dir_v[1], dir_v[0]]))
            L_list.append(L)
            
        if len(A_list) == 0: return global_guess, False
            
        A_arr = np.array(A_list)
        dir_v_arr = np.array(dir_list)
        n_arr = np.array(n_list)
        L_arr = np.array(L_list)

        # =========================================================
        # GROVSØK OG FINSØK
        # =========================================================
        for dth in np.arange(-search_th, search_th + step_th, step_th):
            test_th = global_guess[2] + dth
            rot_penalty = abs(dth) * pen_rot
            c_new, s_new = np.cos(test_th), np.sin(test_th)
            
            rot_x = active_pts[:, 0] * c_new - active_pts[:, 1] * s_new
            rot_y = active_pts[:, 0] * s_new + active_pts[:, 1] * c_new
            
            for dx in np.arange(-search_xy, search_xy + step_xy, step_xy):
                for dy in np.arange(-search_xy, search_xy + step_xy, step_xy):
                    test_x, test_y = global_guess[0] + dx, global_guess[1] + dy
                    
                    trans_pts = np.column_stack((rot_x + test_x, rot_y + test_y))
                    
                    # MAGIEN SKJER HER: Sender inn pre-computed arrays!
                    score = self.point_to_line_score_fast(trans_pts, A_arr, dir_v_arr, n_arr, L_arr)
                    total_penalty = (np.hypot(dx, dy) * pen_dist) + rot_penalty
                    
                    if (score - total_penalty) > best_score:
                        best_score = score - total_penalty
                        best_pose = np.array([test_x, test_y, test_th])

        if best_score > min_overlap:
            return best_pose, True
        return global_guess, False

    def get_relative_pose(self, p1, p2):
        c, s = np.cos(p1[2]), np.sin(p1[2])
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        return np.array([dx*c + dy*s, -dx*s + dy*c, normalize_angle(p2[2] - p1[2])])

    def optimize_graph(self):
        print("\n🚀 LOOP CLOSURE! Kjører Graph-SLAM...")
        def residuals(state):
            res = []
            origins = state.reshape((-1, 3))
            res.extend((origins[0] - self.submaps[0].origin) * 1000) 
            
            for (i, j, rel) in self.edges_odom:
                pi, pj = origins[i], origins[j]
                c, s = np.cos(pi[2]), np.sin(pi[2])
                pred_dx = (pj[0] - pi[0])*c + (pj[1] - pi[1])*s
                pred_dy = -(pj[0] - pi[0])*s + (pj[1] - pi[1])*c
                pred_dth = normalize_angle(pj[2] - pi[2])
                res.extend([(pred_dx - rel[0])*2.0, (pred_dy - rel[1])*2.0, normalize_angle(pred_dth - rel[2])*2.0])
                
            for (i, j, rel) in self.edges_loop:
                pi, pj = origins[i], origins[j]
                c, s = np.cos(pi[2]), np.sin(pi[2])
                pred_dx = (pj[0] - pi[0])*c + (pj[1] - pi[1])*s
                pred_dy = -(pj[0] - pi[0])*s + (pj[1] - pi[1])*c
                pred_dth = normalize_angle(pj[2] - pi[2])
                res.extend([pred_dx - rel[0], pred_dy - rel[1], normalize_angle(pred_dth - rel[2])*4.0])
            return np.array(res)

        init_state = np.array([sm.origin for sm in self.submaps]).flatten()
        result = least_squares(residuals, init_state, loss='huber')
        opt_origins = result.x.reshape((-1, 3))
        
        for i, sm in enumerate(self.submaps):
            sm.origin = opt_origins[i]

    def update(self, dx, dy, dth, sensor_scan, nicla_data=None):
        prev_p = self.poses[-1]
        guess_pose = np.array([
            prev_p[0] + dx*np.cos(prev_p[2]) - dy*np.sin(prev_p[2]),
            prev_p[1] + dx*np.sin(prev_p[2]) + dy*np.cos(prev_p[2]),
            normalize_angle(prev_p[2] + dth)
        ])
        
        prev_o = self.odom_poses[-1]
        self.odom_poses.append(np.array([
            prev_o[0] + dx*np.cos(prev_o[2]), prev_o[1] + dx*np.sin(prev_o[2]), normalize_angle(prev_o[2] + dth)
        ]))

        active_sm = self.submaps[-1]
        dist_moved = np.hypot(guess_pose[0] - self.last_icp_pose[0], guess_pose[1] - self.last_icp_pose[1])
        angle_moved = abs(normalize_angle(guess_pose[2] - self.last_icp_pose[2]))
        
        if dist_moved >= self.DIST_THRESH or angle_moved >= self.ANGLE_THRESH:
            # =========================================================
            # HENT UT RÅPUNKTER FRA BÅDE TOF OG NICLA
            # =========================================================
            raw_pts = []
            for angle, dist in sensor_scan:
                if 0.2 < dist <= MAX_BUILD_DIST:
                    raw_pts.append([dist * np.cos(angle), dist * np.sin(angle)])
                    
            if nicla_data:
                dist = nicla_data['dist']
                width = nicla_data['width']
                start_y = nicla_data['start_y']
                for dy in np.linspace(start_y, start_y + width, 5):
                    raw_pts.append([dist, dy]) 
                    
            raw_pts = np.array(raw_pts)

            if len(raw_pts) > 0:
                rel_pose_raw = self.get_relative_pose(active_sm.origin, guess_pose)
                c, s = np.cos(rel_pose_raw[2]), np.sin(rel_pose_raw[2])
                
                # Transformer til lokalt rom
                trans_pts = np.dot(raw_pts, np.array([[c, -s], [s, c]]).T) + rel_pose_raw[:2]
                
                # ========================================================
                # VOXEL DOWNSAMPLING 
                # ========================================================
                clean_voxel_pts = self.voxel_downsample(trans_pts, voxel_size=0.10)
                
                if active_sm.raw_points is not None:
                    # Legg kun til de vaskede punktene i minnet
                    active_sm.raw_points = np.vstack((active_sm.raw_points, clean_voxel_pts))
                    
                    # (Men send originale, tette scan til Occupancy Gridet for å tegne gode vegger)
                    active_sm.add_scan_to_local_grid(rel_pose_raw, sensor_scan)
                
            self.last_icp_pose = guess_pose

            dist_from_origin = np.hypot(guess_pose[0] - active_sm.origin[0], guess_pose[1] - active_sm.origin[1])
            num_pts = len(active_sm.raw_points) if active_sm.raw_points is not None else 0
            
            # Sjekk om vi har nok spredning (geometri) for å pakke
            has_good_geometry = False
            if num_pts > 150:
                spread_x = np.max(active_sm.raw_points[:, 0]) - np.min(active_sm.raw_points[:, 0])
                spread_y = np.max(active_sm.raw_points[:, 1]) - np.min(active_sm.raw_points[:, 1])
                if spread_x > 0.8 and spread_y > 0.8:
                    has_good_geometry = True

            # Kriterier for å pakke
            is_ready_to_pack = (dist_from_origin > self.SUBMAP_RADIUS and has_good_geometry) or (num_pts > 600) or (dist_from_origin > 4.0)

            if is_ready_to_pack:
                current_idx = len(self.submaps) - 1
                rel_odom = self.get_relative_pose(active_sm.origin, guess_pose) 
                
                if num_pts < 200 and not has_good_geometry:
                    print(f"🗑️ Kaster data fra Submap {current_idx}")
                    active_sm.raw_points = np.empty((0, 2)) 
                else:
                    # ==========================================================
                    # 1. LOKAL MATCH (Rådata mot nabokart)
                    # ==========================================================
                    if current_idx > 0:
                        prev_sm = self.submaps[-2]
                        opt_origin, success = self.fast_vector_correlative_match(
                            active_sm.raw_points, prev_sm, active_sm.origin, job_type='local'
                        )
                        if success:
                            print(f"🤝 LOKAL KORREKSJON: Sydde Submap {current_idx} fast i {current_idx-1}")
                            active_sm.origin = opt_origin
                            self.edges_odom[-1] = (current_idx - 1, current_idx, self.get_relative_pose(prev_sm.origin, active_sm.origin))
                            
                            c_new, s_new = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
                            guess_pose = np.array([
                                active_sm.origin[0] + rel_odom[0]*c_new - rel_odom[1]*s_new,
                                active_sm.origin[1] + rel_odom[0]*s_new + rel_odom[1]*c_new,
                                normalize_angle(active_sm.origin[2] + rel_odom[2])
                            ])
                            rel_odom = self.get_relative_pose(active_sm.origin, guess_pose)

                    # ==========================================================
                    # 2. GLOBAL LOOP CLOSURE (Før vi sletter rådataene!)
                    # ==========================================================
                    last_lc = getattr(self, 'last_lc_idx', -100)
                    GROUP_SIZE = 3 # Vi krever 3 subkart!

                    # 1. Vi prøver KUN Loop Closure hvis vi akkurat har pakket et kart, 
                    # og vi har minst 3 nye kart siden forrige Loop Closure.
                    if len(self.submaps) >= 5 and (current_idx - last_lc) >= GROUP_SIZE:
                        
                        # 2. Samle råpunktene (eller voxels/linjer) fra de SISTE 3 subkartene
                        super_points = []
                        
                        # Vi bruker det nyeste subkartet som "senter"
                        center_sm = self.submaps[-1] 
                        
                        for i in range(current_idx - GROUP_SIZE + 1, current_idx + 1):
                            sm = self.submaps[i]
                            
                            # Finn ut nøyaktig hvordan de gamle kartene ligger i forhold til det nyeste
                            rel_pose = self.get_relative_pose(center_sm.origin, sm.origin)
                            c, s = np.cos(rel_pose[2]), np.sin(rel_pose[2])
                            
                            # Transformer punktene deres inn i senter-kartet
                            if sm.raw_points is not None and len(sm.raw_points) > 0:
                                trans_pts = np.dot(sm.raw_points, np.array([[c, -s], [s, c]]).T) + rel_pose[:2]
                                super_points.append(trans_pts)
                                
                        if len(super_points) > 0:
                            super_points = np.vstack(super_points) # Slå sammen til én gigantisk punktsky!
                            
                            # 3. NÅ kan vi lete i historien med vårt massive, unike fingeravtrykk
                            candidates = []
                            for old_idx, old_sm in enumerate(self.submaps[:-GROUP_SIZE - 5]): # Pass på at vi ikke matcher med oss selv
                                dist_to_old = np.hypot(center_sm.origin[0] - old_sm.origin[0], center_sm.origin[1] - old_sm.origin[1])
                                if dist_to_old < 2.0: # Leter innenfor 2 meter
                                    candidates.append((dist_to_old, old_idx, old_sm))
                            
                            candidates.sort(key=lambda x: x[0])
                            
                            for dist, old_idx, old_sm in candidates:
                                # Vi matcher det gigantiske Super-kartet mot historien!
                                loop_origin, success = self.fast_vector_correlative_match(
                                    super_points, old_sm, center_sm.origin, job_type='global'
                                )
                                
                                if success:
                                    print(f"🔗 UNIKT FINGERAVTRYKK FUNNET! Kobler gruppen til Submap {old_idx}")
                                    rel_loop = self.get_relative_pose(old_sm.origin, loop_origin)
                                    self.edges_loop.append((old_idx, current_idx, rel_loop))
                                    self.last_lc_idx = current_idx
                                    self.optimize_graph()
                                    break # Én god match er alt vi trenger!
                            
                    # ==========================================================
                    # 3. STØP TIL LINJER OG SLETT RÅDATA (Nå er vi ferdige med dem!)
                    # ==========================================================
                    print(f"📦 STØPER VEKTORER for Submap {current_idx}")
                    active_sm.pack_and_convert_to_lines()

                # Klargjør neste subkart
                self.edges_odom.append((current_idx, current_idx + 1, self.get_relative_pose(active_sm.origin, guess_pose)))
                self.submaps.append(Submap(guess_pose))

        self.poses.append(guess_pose)

# ==========================================
# 4. TASTATUR-KONTROLL & HOVED-LOOP
# ==========================================
slam = GlobalSLAM_V4()
v_cmd, w_cmd = 0.0, 0.0

def on_press(event):
    global v_cmd, w_cmd
    if event.key == 'up': v_cmd = 0.15 
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

print("\n🚀 V4: TRUE VECTOR SLAM (Linje-til-linje matching)")
print("🚨 MERK: Kartet tegnes nå utelukkende med vektorer, INGEN PIKSLER!")
print("🕹️ Klikk på plottet og styr med piltastene!")

while plt.fignum_exists(fig.number):
    new_th = normalize_angle(true_pose[2] + w_cmd * dt)
    new_x = true_pose[0] + v_cmd * np.cos(true_pose[2]) * dt
    new_y = true_pose[1] + v_cmd * np.sin(true_pose[2]) * dt
    
    if 0.2 < new_x < ROOM_SIZE-0.2 and 0.2 < new_y < ROOM_SIZE-0.2:
        true_pose = np.array([new_x, new_y, new_th])
    else: true_pose[2] = new_th 
    true_poses_hist.append(true_pose.copy())
    
    if v_cmd != 0 or w_cmd != 0:
        std_v = 0.05 * abs(v_cmd) + 0.05 * abs(w_cmd) 
        std_w = 0.05 * abs(v_cmd) + 0.05 * abs(w_cmd) 
        v_noisy = v_cmd + np.random.normal(0, std_v)
        w_noisy = w_cmd + np.random.normal(0, std_w)
        v_noisy = v_cmd 
        w_noisy = w_cmd
    else:
        v_noisy, w_noisy = 0.0, 0.0
    
    # =========================================================
    # SENSOR-SIMULERING (Sender en "LaserScan" til SLAM)
    # =========================================================
    sensor_scan = [] 
    nicla_data = None 
    
    if ACTIVE_SENSOR == 'tof':
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

        final_dist = min(wall_dist, nearest_obj_dist)
        if final_dist < MAX_RANGE: 
            final_dist += np.random.normal(0, 0.00) 
        
        for ang in np.linspace(-TOF_FOV/2, TOF_FOV/2, 15):
            sensor_scan.append((ang, final_dist))

        if nearest_obj_dist < MAX_RANGE and nearest_obj_dist < wall_dist:
            lm_pos = TRUE_LANDMARKS[nearest_obj_id]
            bearing = normalize_angle(np.arctan2(lm_pos[1] - true_pose[1], lm_pos[0] - true_pose[0]) - true_pose[2])
            
            if abs(bearing) < CAM_FOV/2:
                bearing_noisy = bearing + np.random.normal(0, 0.00)
                width_m = OBJ_RADIUS * 2
                start_y_m = final_dist * np.sin(bearing_noisy) - (width_m / 2)
                nicla_data = {'dist': final_dist, 'width': width_m, 'start_y': start_y_m}

    elif ACTIVE_SENSOR == 'lidar':
        for ang in np.linspace(-LIDAR_FOV/2, LIDAR_FOV/2, LIDAR_RAYS):
            global_ang = true_pose[2] + ang
            dist = raycast_to_walls(true_pose, [global_ang])
            
            for obj_id, lm_pos in TRUE_LANDMARKS.items():
                dx, dy = lm_pos[0] - true_pose[0], lm_pos[1] - true_pose[1]
                obj_dist = np.hypot(dx, dy)
                obj_bearing = normalize_angle(np.arctan2(dy, dx) - true_pose[2])
                
                if abs(normalize_angle(ang - obj_bearing)) < np.arctan2(OBJ_RADIUS, obj_dist):
                    dist_to_surf = obj_dist - OBJ_RADIUS
                    if 0 < dist_to_surf < dist:
                        dist = dist_to_surf
            
            if dist < MAX_RANGE:
                dist += np.random.normal(0, 0.01) 
                sensor_scan.append((ang, dist))

    # NYTT: Send både sensor_scan og nicla_data til SLAM
    slam.update(v_noisy * dt, 0.0, w_noisy * dt, sensor_scan, nicla_data)
            
    if len(slam.poses) % 2 == 0:
        ax.clear()
        
        for wall in TRUE_WALLS: ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], 'b--', linewidth=1.5, alpha=0.3)
            
        true_arr, slam_arr, odom_arr = np.array(true_poses_hist), np.array(slam.poses), np.array(slam.odom_poses)
        ax.plot(true_arr[:,0], true_arr[:,1], 'b-', alpha=0.3, label="Sann rute")
        ax.plot(odom_arr[:,0], odom_arr[:,1], 'g--', alpha=0.5, label="Ren Odometri")
        ax.plot(slam_arr[:,0], slam_arr[:,1], 'r-', linewidth=2, label="V4: Vector SLAM")

        # --- TEGN VEKTORKARTET (MED LINE MERGING) ---
        all_global_lines = []
        
        for sm in slam.submaps: 
            c, s = np.cos(sm.origin[2]), np.sin(sm.origin[2])
            R = np.array([[c, -s], [s, c]])
            
            # NYTT: Tegn cyan prikker for de ryddede RANSAC-punktene!
            if hasattr(sm, 'clean_points') and len(sm.clean_points) > 0:
                g_clean = np.dot(sm.clean_points, R.T) + sm.origin[:2]
                ax.scatter(g_clean[:, 0], g_clean[:, 1], s=4, c='cyan', alpha=0.5, zorder=4)

            for (start_pt, end_pt) in sm.line_segments:
                g_start = np.dot(start_pt, R.T) + sm.origin[:2]
                g_end = np.dot(end_pt, R.T) + sm.origin[:2]
                all_global_lines.append((g_start, g_end))
                
        # Smelt sammen de oransje linjene for å fjerne ghosting
        clean_merged_lines = merge_line_segments(all_global_lines)
        
        for (start_pt, end_pt) in clean_merged_lines:
            ax.plot([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], color='darkorange', linewidth=3.0, zorder=6)

        curr = slam_arr[-1]
        ax.add_patch(Circle((curr[0], curr[1]), 0.15, color='red', alpha=0.3))
        car_rect = Rectangle((curr[0]-0.1, curr[1]-0.075), 0.2, 0.15, angle=np.rad2deg(curr[2]), color='darkred')
        ax.add_patch(car_rect)
        ax.plot([curr[0], curr[0] + 0.2*np.cos(curr[2])], [curr[1], curr[1] + 0.2*np.sin(curr[2])], 'y-', linewidth=2.5)
            
        ax.set_xlim(-1, ROOM_SIZE + 1); ax.set_ylim(-1, ROOM_SIZE + 1)
        ax.set_title("V4: TRUE VECTOR SLAM | Memory: ULTRALOW")
        ax.legend(loc="upper left", fontsize=8)
        
        # Svart bakgrunn siden vi ikke har noe Grid lenger!
        ax.set_facecolor('#202020')
        plt.pause(0.01)

plt.ioff()