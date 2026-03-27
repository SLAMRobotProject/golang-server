import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.optimize import least_squares
from scipy.ndimage import distance_transform_edt
from matplotlib.patches import Rectangle, Circle

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# ==========================================
# KONFIGURASJON AV SENSOR
# ==========================================
ACTIVE_SENSOR = 'tof'  # Velg mellom 'tof' eller 'lidar'
LIDAR_FOV = np.deg2rad(60)
LIDAR_RAYS = 31 # Antall distinkte laserstråler i 60-graders feltet

if ACTIVE_SENSOR == 'tof':
    MAX_BUILD_DIST = 1.5  
else:
    MAX_BUILD_DIST = 3.0

# ==========================================
# 1. MILJØ & SIMULERING (4x4m Rom)
# ==========================================
ROOM_SIZE = 4.0
TRUE_LANDMARKS = {0: np.array([1.0, 1.0]), 1: np.array([3.0, 3.0]), 2: np.array([1.0, 3.0])}
OBJ_RADIUS = 0.15 
MAX_RANGE = 3.0 
TOF_FOV = np.deg2rad(27)
CAM_FOV = np.deg2rad(60)

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

# ==========================================
# 2. INVERSE SENSOR MODEL & GRAPH-SLAM
# ==========================================
class Submap:
    def __init__(self, origin_pose):
        self.origin = origin_pose.copy() 
        self.scans = [] 

        self.raw_points = np.empty((0, 2))
        self.clean_points = np.empty((0, 2))
        
        # =========================================================
        # LOKALT OCCUPANCY GRID (Fjerner støy, ingen drift-bias!)
        # =========================================================
        self.grid_res = 0.05
        # Lager et lokalt kart på f.eks. 6x6 meter rundt startpunktet
        self.grid_size = 6.0 
        self.grid_w = int(self.grid_size / self.grid_res)
        self.grid_h = int(self.grid_size / self.grid_res)
        self.local_grid = np.zeros((self.grid_h, self.grid_w))
        self.offset = self.grid_size / 2.0 # Plasserer (0,0) midt i gridet
        
        # Dataene vi sitter igjen med til slutt
        self.local_points = np.empty((0, 2)) 
        self.match_grid = None
        self.min_x = 0.0
        self.min_y = 0.0
        self.w = 0
        self.h = 0

    def add_scan_to_local_grid(self, rel_pose, sensor_scan):
        """ Kalles hver gang bilen beveger seg. Tar imot en liste med (vinkel, avstand) """
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
            
            # Viskelær (Freespace) - Alltid ut til 3m for å rydde rommet!
            for i in range(steps):
                cx = int(rx + (hx - rx) * (i / steps))
                cy = int(ry + (hy - ry) * (i / steps))
                if 0 <= cx < self.grid_w and 0 <= cy < self.grid_h: 
                    self.local_grid[cy, cx] -= 0.15 
                    
            # Vegg (Occupied) - KUN innenfor pålitelig avstand!
            if dist_m <= MAX_BUILD_DIST:
                if 0 <= hx < self.grid_w and 0 <= hy < self.grid_h: 
                    weight = np.exp(-dist_m / 2.0)
                    self.local_grid[hy, hx] += 0.5 * weight
                
        self.local_grid = np.clip(self.local_grid, -5.0, 5.0)

    def add_line_segment_to_grid(self, rel_pose, dist, y_start, y_end):
        """
        Tegner ToF-målingen som en solid vegg (linje) for maksimal scan-matching presisjon.
        """
        # Hvor mange "piksler" trenger vi for å fylle linjen uten hull?
        # Vi deler lengden på linjen på grid-oppløsningen (f.eks. 5 cm)
        n_steps = max(int(abs(y_end - y_start) / self.grid_res), 2)
        
        # Vi stoler mindre på målingen jo lenger unna veggen er (eksponentielt fall)
        weight = np.exp(-dist / 2.0)
        
        # Robotens vinkel
        theta = rel_pose[2]
        cos_th = np.cos(theta)
        sin_th = np.sin(theta)
        
        # Generer tett sampling langs hele segmentet (i sensorens LOKALE ramme)
        for y_local in np.linspace(y_start, y_end, n_steps):
            
            # 🚀 FIKS: Korrekt 2D-rotasjon av punktet (dist, y_local)
            hit_x = rel_pose[0] + (dist * cos_th) - (y_local * sin_th)
            hit_y = rel_pose[1] + (dist * sin_th) + (y_local * cos_th)
            
            # Konverter til grid-koordinater
            hx = int((hit_x + self.offset) / self.grid_res)
            hy = int((hit_y + self.offset) / self.grid_res)
            
            # Sjekk at vi er innenfor kartets grenser
            if 0 <= hx < self.grid_size and 0 <= hy < self.grid_size:
                # Pluss på log-odds for okkupert vegg
                self.local_grid[hy, hx] += 0.5 * weight

    def extract_clean_points_and_build_lf(self, sigma=0.20, padding=2.0):
        """ Kalles NÅR SUBKARTET PAKKES! Gjør grid om til punkter og varme-kart """
        
        # 1. Hent ut kun de cellene som vi er SIKRE på at er en vegg (log-odds > 0.5)
        y_idx, x_idx = np.where(self.local_grid > 0.5)
        clean_pts = []
        
        for y, x in zip(y_idx, x_idx):
            # Gjør piksel-koordinater tilbake til fysiske meter relativt til subkartet
            real_x = (x * self.grid_res) - self.offset
            real_y = (y * self.grid_res) - self.offset
            clean_pts.append([real_x, real_y])
            
        self.clean_points = np.array(clean_pts)
        
        # Hvis vi ikke fant noen vegger, avbryt
        if len(self.clean_points) == 0: return
        
        # 2. Bygg Likelihood Field basert på disse sylskarpe punktene
        self.min_x = np.min(self.clean_points[:, 0]) - padding
        max_x = np.max(self.clean_points[:, 0]) + padding
        self.min_y = np.min(self.clean_points[:, 1]) - padding
        max_y = np.max(self.clean_points[:, 1]) + padding
        
        inv_res = 1.0 / self.grid_res
        self.w = int((max_x - self.min_x) * inv_res)
        self.h = int((max_y - self.min_y) * inv_res)
        
        occ = np.ones((self.h, self.w))
        
        # 🚀 LYN-RASK NUMPY VEKTORISERING (Eliminerer alle trege for-løkker!)
        # Vi regner ut alle piksel-koordinatene på ett millisekund
        ix = ((self.clean_points[:, 0] - self.min_x) * inv_res).astype(int)
        iy = ((self.clean_points[:, 1] - self.min_y) * inv_res).astype(int)
        
        # Sorter ut punkter som havner utenfor kartet
        valid = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        
        # Sett alle gyldige vegg-piksler til 0.0 samtidig!
        occ[iy[valid], ix[valid]] = 0.0 
        
        # Vi dropper den manuelle 3x3 utvidelsen (for-løkkene vi hadde her før), 
        # og øker heller sigma litt, slik at distance_transform gjør jobben gratis!
        dist = distance_transform_edt(occ) * self.grid_res
        self.match_grid = np.exp(-(dist**2) / (2 * sigma**2)) # Økt sigma fra 0.15 til 0.20
        

class GlobalSLAM_V2:
    def __init__(self, grid_size=6.0, grid_res=0.05):
        start_pose = np.array([0.5, 0.5, 0.5]) 
        self.submaps = [Submap(start_pose.copy())]
        self.poses = [start_pose.copy()]
        self.odom_poses = [start_pose.copy()]
        
        self.edges_odom = [] 
        self.edges_loop = [] 
        
        self.grid_res = grid_res 
        self.grid_size = grid_size
        self.grid_w = int(grid_size / grid_res)
        self.grid_h = int(grid_size / grid_res)
        self.grid = np.zeros((self.grid_h, self.grid_w))
        
        self.last_icp_pose = start_pose.copy() 
        self.DIST_THRESH = 0.03
        self.ANGLE_THRESH = 0.05
        self.SUBMAP_RADIUS = 1.5

    def get_raw_sensor_points(self, sensor_scan, nicla_data=None):
        raw_pts = []
        for angle, dist in sensor_scan:
            # Trekker kun ut punkter vi stoler på!
            if 0.2 < dist <= MAX_BUILD_DIST:
                raw_pts.append([dist * np.cos(angle), dist * np.sin(angle)])
                
        if nicla_data:
            dist = nicla_data['dist']
            width = nicla_data['width']
            start_y = nicla_data['start_y']
            # Tettere sampling (5cm mellomrom istedenfor fast 5 punkter)
            n_steps = max(int(abs(width) / 0.05), 5)
            for dy in np.linspace(start_y, start_y + width, n_steps):
                raw_pts.append([dist, dy])
        
        return np.array(raw_pts) if len(raw_pts) > 0 else np.empty((0, 2))

    def get_raycasted_clean_points(self, pose, nicla_data=None, max_range=1.5):
        local_pts = []
        
        # 1. RAYCASTING: Simuler en ren LIDAR basert på det ferdige rutenettet!
        # Vi skyter 36 stråler (hver 10. grad) ut fra bilen
        for angle in np.linspace(0, 2*np.pi, 36, endpoint=False):
            # Marsjerer utover langs strålen
            for r in np.arange(0.2, max_range, self.grid_res):
                global_angle = pose[2] + angle
                hit_x = pose[0] + r * np.cos(global_angle)
                hit_y = pose[1] + r * np.sin(global_angle)
                hx, hy = int((hit_x+1.0)/self.grid_res), int((hit_y+1.0)/self.grid_res)
                
                if 0 <= hx < self.grid_w and 0 <= hy < self.grid_h:
                    # Hvis vi treffer en "solid" piksel (log-odds over 0.5)
                    if self.grid[hy, hx] > 0.5: 
                        # Vi lagrer punktet relativt til bilen, akkurat som en ekte sensor!
                        local_pts.append([r * np.cos(angle), r * np.sin(angle)])
                        break # Stopp strålen, veggen er truffet
                else:
                    break # Utenfor kartet
        
        # 2. NICLA-DATA: Kameraet er alltid presist, så de dataene tar vi direkte
        if nicla_data:
            dist = nicla_data['dist']
            width = nicla_data['width']
            start_y = nicla_data['start_y']
            for dy in np.linspace(start_y, start_y + width, 5):
                local_pts.append([dist, dy]) 
                
        return np.array(local_pts)

    def fast_submap_correlative_match(self, active_pts, old_sm, global_guess, job_type='global'):
        if len(active_pts) == 0 or len(old_sm.clean_points) == 0 or old_sm.match_grid is None:
            return global_guess, False
        
        # =========================================================
        # 1. PROFILER (Job Descriptors)
        # =========================================================
        if job_type == 'local':
            # LOKAL JOBB: Submap til Submap. Lite område, streng straff, fine steg.
            cfg = {
                'min_overlap': 0.40,
                'search_xy': 0.20,  'search_th': 0.16,
                'pen_dist': 0.5,   'pen_rot': 0.8,
                'c_step_xy': 0.05,  'c_step_th': 0.02,  # Finere grovsøk
                'f_step_xy': 0.005, 'f_step_th': 0.005   # Veldig presist finsøk
            }
        else: # 'global'
            # GLOBAL JOBB: Loop Closure. Stort område, lav straff, grove steg.
            cfg = {
                'min_overlap': 0.50,
                'search_xy': 0.80,  'search_th': 0.65,
                'pen_dist': 0.05,    'pen_rot': 0.2,
                'c_step_xy': 0.10,  'c_step_th': 0.10,  # Raske, grove byks
                'f_step_xy': 0.020, 'f_step_th': 0.01   # Standard finsøk
            }

        local_res = old_sm.grid_res
        inv_res = 1.0 / local_res
        num_pts = len(active_pts)
        c_old, s_old = np.cos(old_sm.origin[2]), np.sin(old_sm.origin[2])

        best_pose = global_guess.copy()
        best_coarse_pose = best_pose.copy()
        best_score = -np.inf

        # =========================================================
        # 2. GROVSØK (Precomputed Rotation + Likelihood Field)
        # =========================================================
        for dth in np.arange(-cfg['search_th'], cfg['search_th'] + cfg['c_step_th'], cfg['c_step_th']):
            test_th = global_guess[2] + dth
            
            # Precompute rotasjon
            rel_th = normalize_angle(test_th - old_sm.origin[2])
            c, s = np.cos(rel_th), np.sin(rel_th)
            rot_x = active_pts[:, 0] * c - active_pts[:, 1] * s
            rot_y = active_pts[:, 0] * s + active_pts[:, 1] * c
            
            rot_penalty = abs(dth) * cfg['pen_rot']
            
            for dx in np.arange(-cfg['search_xy'], cfg['search_xy'] + cfg['c_step_xy'], cfg['c_step_xy']):
                for dy in np.arange(-cfg['search_xy'], cfg['search_xy'] + cfg['c_step_xy'], cfg['c_step_xy']):
                    
                    test_x = global_guess[0] + dx
                    test_y = global_guess[1] + dy
                    
                    # Flytt roterte punkter
                    glob_dx = test_x - old_sm.origin[0]
                    glob_dy = test_y - old_sm.origin[1]
                    rel_x = glob_dx * c_old + glob_dy * s_old
                    rel_y = -glob_dx * s_old + glob_dy * c_old
                    
                    final_pts_x = rot_x + rel_x
                    final_pts_y = rot_y + rel_y
                    
                    # --- LIKELIHOOD FIELD SCORING (Direkte i løkken for maks fart) ---
                    ix = ((final_pts_x - old_sm.min_x) * inv_res).astype(int)
                    iy = ((final_pts_y - old_sm.min_y) * inv_res).astype(int)
                    valid = (ix >= 0) & (ix < old_sm.w) & (iy >= 0) & (iy < old_sm.h)
                    
                    score = np.sum(old_sm.match_grid[iy[valid], ix[valid]]) / num_pts if np.any(valid) else 0.0
                    
                    dist_penalty = np.hypot(dx, dy) * cfg['pen_dist']
                    total_penalty = dist_penalty + rot_penalty
                    
                    if (score - total_penalty) > best_score:
                        best_score = score - total_penalty
                        best_coarse_pose = np.array([test_x, test_y, test_th])

        # =========================================================
        # 3. FINSØK (Precomputed Rotation + Likelihood Field)
        # =========================================================
        fine_xy, fine_th = cfg['c_step_xy'], cfg['c_step_th']
        
        for dth in np.arange(-fine_th, fine_th + cfg['f_step_th'], cfg['f_step_th']):
            test_th = best_coarse_pose[2] + dth
            
            rel_th = normalize_angle(test_th - old_sm.origin[2])
            c, s = np.cos(rel_th), np.sin(rel_th)
            rot_x = active_pts[:, 0] * c - active_pts[:, 1] * s
            rot_y = active_pts[:, 0] * s + active_pts[:, 1] * c
            
            tot_dth = normalize_angle(test_th - global_guess[2])
            rot_penalty = abs(tot_dth) * cfg['pen_rot']
            
            for dx in np.arange(-fine_xy, fine_xy + cfg['f_step_xy'], cfg['f_step_xy']):
                for dy in np.arange(-fine_xy, fine_xy + cfg['f_step_xy'], cfg['f_step_xy']):
                    
                    test_x = best_coarse_pose[0] + dx
                    test_y = best_coarse_pose[1] + dy
                    
                    glob_dx = test_x - old_sm.origin[0]
                    glob_dy = test_y - old_sm.origin[1]
                    rel_x = glob_dx * c_old + glob_dy * s_old
                    rel_y = -glob_dx * s_old + glob_dy * c_old
                    
                    final_pts_x = rot_x + rel_x
                    final_pts_y = rot_y + rel_y
                    
                    # --- LIKELIHOOD FIELD SCORING ---
                    ix = ((final_pts_x - old_sm.min_x) * inv_res).astype(int)
                    iy = ((final_pts_y - old_sm.min_y) * inv_res).astype(int)
                    valid = (ix >= 0) & (ix < old_sm.w) & (iy >= 0) & (iy < old_sm.h)
                    
                    score = np.sum(old_sm.match_grid[iy[valid], ix[valid]]) / num_pts if np.any(valid) else 0.0
                    
                    tot_dx = test_x - global_guess[0]
                    tot_dy = test_y - global_guess[1]
                    dist_penalty = np.hypot(tot_dx, tot_dy) * cfg['pen_dist']
                    
                    total_penalty = dist_penalty + rot_penalty
                    
                    if (score - total_penalty) > best_score:
                        best_score = score - total_penalty
                        best_pose = np.array([test_x, test_y, test_th])

        # KVALITETSKONTROLL (Direkte sammenligning pga normalisering)
        if best_score > cfg['min_overlap']:
            return best_pose, True
        return global_guess, False

    def get_relative_pose(self, p1, p2):
        c, s = np.cos(p1[2]), np.sin(p1[2])
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        return np.array([dx*c + dy*s, -dx*s + dy*c, normalize_angle(p2[2] - p1[2])])

    def add_to_occupancy_grid(self, pose, sensor_scan):
        rx, ry = int((pose[0]+1.0)/self.grid_res), int((pose[1]+1.0)/self.grid_res)
        
        for angle, dist_m in sensor_scan:
            if dist_m < 0.2 or dist_m > 3.0: continue
            
            hit_x = pose[0] + dist_m*np.cos(pose[2]+angle)
            hit_y = pose[1] + dist_m*np.sin(pose[2]+angle)
            hx, hy = int((hit_x+1.0)/self.grid_res), int((hit_y+1.0)/self.grid_res)
            
            steps = int(np.hypot(hx-rx, hy-ry))
            if steps == 0: continue
            
            # Viskelær
            for i in range(steps):
                cx, cy = int(rx + (hx-rx)*(i/steps)), int(ry + (hy-ry)*(i/steps))
                if 0 <= cx < self.grid_w and 0 <= cy < self.grid_h: 
                    self.grid[cy, cx] += -0.15 
                    
            # Vegg (Occupied)
            if dist_m <= MAX_BUILD_DIST:
                if 0 <= hx < self.grid_w and 0 <= hy < self.grid_h: 
                    sigma = 0.7 
                    weight = np.exp(-(dist_m**2)/(2*sigma**2)) 
                    self.grid[hy, hx] += 0.5 * weight
                
        self.grid = np.clip(self.grid, -5.0, 5.0)

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
                res.extend([ 
                    (pred_dx - rel[0]) * 2.0, 
                    (pred_dy - rel[1]) * 2.0, 
                    normalize_angle(pred_dth - rel[2]) * 0.5 ])
                
            for (i, j, rel) in self.edges_loop:
                pi, pj = origins[i], origins[j]
                c, s = np.cos(pi[2]), np.sin(pi[2])
                pred_dx = (pj[0] - pi[0])*c + (pj[1] - pi[1])*s
                pred_dy = -(pj[0] - pi[0])*s + (pj[1] - pi[1])*c
                pred_dth = normalize_angle(pj[2] - pi[2])
                res.extend([ 
                    (pred_dx - rel[0]) * 5.0, 
                    (pred_dy - rel[1]) * 5.0,
                    normalize_angle(pred_dth - rel[2]) * 10.0 ])
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

    def voxel_downsample(self, points, voxel_size=0.10):
        """ 
        Gjør punktskyen lettere og renere ved å samle punkter i 10cm-bokser, 
        og returnere gjennomsnittet av punktene i hver boks.
        """
        if len(points) == 0:
            return points
            
        # 1. Tildel hvert punkt til en "voxel-boks"
        voxels = np.round(points / voxel_size)
        
        # 2. Finn de unike boksene, og tell hvor mange punkter som er i hver
        unique_voxels, inverse_idx, counts = np.unique(
            voxels, axis=0, return_inverse=True, return_counts=True
        )
        
        # 3. Regn ut det nøyaktige tyngdepunktet for punktene inni hver boks
        centroids = np.zeros((len(unique_voxels), 2))
        np.add.at(centroids, inverse_idx, points) 
        centroids = centroids / counts[:, np.newaxis]
        
        return centroids

    def update(self, dx, dy, dth, sensor_scan, nicla_data=None):
        # 1. ODOMETRI (Vår beste venn lokalt!)
        prev_p = self.poses[-1]
        guess_pose = np.array([
            prev_p[0] + dx*np.cos(prev_p[2]) - dy*np.sin(prev_p[2]),
            prev_p[1] + dx*np.sin(prev_p[2]) + dy*np.cos(prev_p[2]),
            normalize_angle(prev_p[2] + dth)
        ])
        
        prev_o = self.odom_poses[-1]
        self.odom_poses.append(np.array([
            prev_o[0] + dx*np.cos(prev_o[2]), 
            prev_o[1] + dx*np.sin(prev_o[2]), 
            normalize_angle(prev_o[2] + dth)
        ]))

        active_sm = self.submaps[-1]

        # Oppdater globalt kart kun for visualisering/historikk
        self.add_to_occupancy_grid(guess_pose, sensor_scan)
        
        dist_moved = np.hypot(guess_pose[0] - self.last_icp_pose[0], guess_pose[1] - self.last_icp_pose[1])
        angle_moved = abs(normalize_angle(guess_pose[2] - self.last_icp_pose[2]))
        
        # Siste rådata (for visualisering)
        self.last_raw_pts_global = np.empty((0, 2))
        
        if dist_moved >= self.DIST_THRESH or angle_moved >= self.ANGLE_THRESH:
            
            # =========================================================
            # DATAINNSAMLING TIL SUBKARTET
            # =========================================================
            raw_pts = self.get_raw_sensor_points(sensor_scan, nicla_data)

            if raw_pts is not None and len(raw_pts) > 0:
                rel_pose_raw = self.get_relative_pose(active_sm.origin, guess_pose)
                
                # Transformer punktene til Submapets lokale senter
                c, s = np.cos(rel_pose_raw[2]), np.sin(rel_pose_raw[2])
                trans_pts = np.dot(raw_pts, np.array([[c, -s], [s, c]]).T) + rel_pose_raw[:2]
                
                # =====================================================
                # NYTT: VOXEL-FILTER (Krymper datamengden drastisk!)
                # =====================================================
                clean_voxel_pts = self.voxel_downsample(trans_pts, voxel_size=0.10)
                
                # Samle KUN de filtrerte punktene opp i matchings-minnet
                active_sm.raw_points = np.vstack((active_sm.raw_points, clean_voxel_pts))
                active_sm.scans.append((rel_pose_raw[0], rel_pose_raw[1], rel_pose_raw[2], sensor_scan))

                # Legg inn hele det rå scanet i det lokale rutenettet for pene vegger
                active_sm.add_scan_to_local_grid(rel_pose_raw, sensor_scan)

                if nicla_data:
                    active_sm.add_line_segment_to_grid(
                        rel_pose_raw,
                        nicla_data['dist'],
                        nicla_data['start_y'],
                        nicla_data['start_y'] + nicla_data['width']
    )
                
                # Lagre global versjon for plotting
                c_g, s_g = np.cos(guess_pose[2]), np.sin(guess_pose[2])
                self.last_raw_pts_global = np.dot(raw_pts, np.array([[c_g, -s_g], [s_g, c_g]]).T) + guess_pose[:2]

            self.last_icp_pose = guess_pose 

            # =========================================================
            # PAKKING AV DYNAMISK MINI-SUBMAP (Areal-basert!)
            # =========================================================
            dist_from_origin = np.hypot(guess_pose[0] - active_sm.origin[0], guess_pose[1] - active_sm.origin[1])

            has_good_geometry = False
            spatial_coverage = 0 
            is_ready_to_pack = False
            num_pts = len(active_sm.raw_points)
            
            if num_pts >= 30:
                pts = active_sm.raw_points
                
                spread_x = np.max(pts[:, 0]) - np.min(pts[:, 0])
                spread_y = np.max(pts[:, 1]) - np.min(pts[:, 1])
                
                # FARTSTRIKS 2: Vi kjører KUN den tunge 'np.unique' sjekken hvis vi er 
                # i sonen for å kanskje pakke (god spredning, lang avstand, eller mange punkter)
                if (spread_x > 0.5 and spread_y > 0.5) or dist_from_origin > self.SUBMAP_RADIUS or num_pts > 100:
                    
                    # NÅ gjør vi den tunge areal-utregningen
                    voxels = np.round(pts / 0.10)
                    unique_voxels = np.unique(voxels, axis=0)
                    spatial_coverage = len(unique_voxels) 
                    
                    if spread_x > 0.5 and spread_y > 0.5 and spatial_coverage >= 30:
                        has_good_geometry = True

                    # NY PAKKE-LOGIKK:
                    # 1. Kjørt langt nok OG god geometri.
                    # 2. ELLER: Massivt areal utforsket (f.eks 80 unike blokker) uansett avstand.
                    # 3. ELLER: Kjørt for langt (Tvinger en pakking for å unngå for mye odometri-drift).
                    is_ready_to_pack = (dist_from_origin > self.SUBMAP_RADIUS and has_good_geometry) or (spatial_coverage >= 100) or (dist_from_origin > 3.0)

            if is_ready_to_pack:
                current_idx = len(self.submaps) - 1
                rel_odom = self.get_relative_pose(active_sm.origin, guess_pose) 
                
                # Hvis vi tvinges til å pakke (pga avstand), men har elendig dekning, kaster vi det.
                if spatial_coverage < 20 and not has_good_geometry:
                    print(f"🗑️ Kaster data fra Submap {current_idx} (For lite areal: {spatial_coverage} voxels)")
                    active_sm.raw_points = np.empty((0, 2)) 
                    
                else:
                    print(f"📦 Pakker dynamisk Submap {current_idx} (Areal-dekning: {spatial_coverage} voxels)")

                    # 1. Bygg det magnetiske feltet (Likelihood Field)
                    active_sm.extract_clean_points_and_build_lf()
                    
                    if current_idx > 0:
                        prev_sm = self.submaps[-2]
                        
                        clean_match_pts = self.voxel_downsample(active_sm.raw_points, voxel_size=0.10)
                        
                        opt_origin, success = self.fast_submap_correlative_match(
                            clean_match_pts, prev_sm, active_sm.origin, job_type='local'
                        )
                        
                        if success:
                            print(f"🤝 LOKAL KORREKSJON: Sydde Submap {current_idx} fast i Submap {current_idx-1}")
                            active_sm.origin = opt_origin
                            new_rel = self.get_relative_pose(prev_sm.origin, active_sm.origin)
                            self.edges_odom[-1] = (current_idx - 1, current_idx, new_rel)
                            
                            c_new, s_new = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
                            guess_pose = np.array([
                                active_sm.origin[0] + rel_odom[0]*c_new - rel_odom[1]*s_new,
                                active_sm.origin[1] + rel_odom[0]*s_new + rel_odom[1]*c_new,
                                normalize_angle(active_sm.origin[2] + rel_odom[2])
                            ])
                            rel_odom = self.get_relative_pose(active_sm.origin, guess_pose)

                    # =========================================================
                    # 3. SUPER-SUBMAP LOOP CLOSURE (Gruppe-matching)
                    # =========================================================
                    MIN_SUBMAPS_BEFORE_LC = 6
                    COOLDOWN_SUBMAPS = 4
                    last_lc = getattr(self, 'last_lc_idx', -100)
                    GROUP_SIZE = 3 
                    
                    if len(self.submaps) >= MIN_SUBMAPS_BEFORE_LC and (current_idx - last_lc) >= COOLDOWN_SUBMAPS:
                        
                        # 🚀 FARTSTRIKS 1: Finn kandidater FØRST! (Dette koster nesten null CPU-tid)
                        candidates = []
                        for old_idx, old_sm in enumerate(self.submaps[:-GROUP_SIZE - 4]): 
                            dist_to_old = np.hypot(active_sm.origin[0] - old_sm.origin[0], active_sm.origin[1] - old_sm.origin[1])
                            if dist_to_old < 2.0 and not any(e[0]==old_idx and e[1]==current_idx for e in self.edges_loop):
                                candidates.append((dist_to_old, old_idx, old_sm))
                        
                        # 🚀 FARTSTRIKS 2: Vi bygger KUN det tunge Super-kartet hvis vi faktisk har noe å matche det mot!
                        if len(candidates) > 0:
                            candidates.sort(key=lambda x: x[0])
                            
                            super_points = []
                            center_sm = self.submaps[-1] 
                            
                            for i in range(current_idx - GROUP_SIZE + 1, current_idx + 1):
                                sm = self.submaps[i]
                                if len(sm.raw_points) == 0: continue
                                    
                                rel_pose = self.get_relative_pose(center_sm.origin, sm.origin)
                                c, s = np.cos(rel_pose[2]), np.sin(rel_pose[2])
                                trans_pts = np.dot(sm.raw_points, np.array([[c, -s], [s, c]]).T) + rel_pose[:2]
                                super_points.append(trans_pts)
                                    
                            if len(super_points) > 0:
                                super_points = np.vstack(super_points) 
                                super_points = self.voxel_downsample(super_points, voxel_size=0.10)
                                
                                # Sjekk kun de 3 beste kandidatene
                                for dist, old_idx, old_sm in candidates[:3]: 
                                    loop_origin, success = self.fast_submap_correlative_match(
                                        super_points, old_sm, center_sm.origin, job_type='global'
                                    )
                                    
                                    if success:
                                        print(f"🔗 UNIKT FINGERAVTRYKK FUNNET! Kobler gruppe (Submap {current_idx}) til Submap {old_idx}")
                                        rel_loop = self.get_relative_pose(old_sm.origin, loop_origin)
                                        self.edges_loop.append((old_idx, current_idx, rel_loop))
                                        self.last_lc_idx = current_idx 
                                        
                                        self.optimize_graph() 
                                        
                                        c_new, s_new = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
                                        guess_pose = np.array([
                                            active_sm.origin[0] + rel_odom[0]*c_new - rel_odom[1]*s_new,
                                            active_sm.origin[1] + rel_odom[0]*s_new + rel_odom[1]*c_new,
                                            normalize_angle(active_sm.origin[2] + rel_odom[2])
                                        ])
                                        break

                # Klargjør neste subkart
                self.edges_odom.append((current_idx, current_idx + 1, self.get_relative_pose(active_sm.origin, guess_pose)))
                self.submaps.append(Submap(guess_pose))

        self.poses.append(guess_pose)

# ==========================================
# 3. TASTATUR-KONTROLL & HOVED-LOOP
# ==========================================
slam = GlobalSLAM_V2()
v_cmd, w_cmd = 0.0, 0.0

def on_press(event):
    global v_cmd, w_cmd
    if event.key == 'up': v_cmd = 0.2 
    elif event.key == 'down': v_cmd = -0.2
    elif event.key == 'left': w_cmd = 0.75
    elif event.key == 'right': w_cmd = -0.75

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

# --- REALISTISKE TIDS-BEGRENSNINGER (Sim2Real) ---
dt = 0.01              # Fysikk og odometri kjører på 100Hz (10ms)
SENSOR_RATE = 0.10     # Sensorer og SLAM kjører på 10Hz (100ms)
sensor_timer = 0.0

# Akkumulator for odometri mellom hvert sensorsveip
accum_dx = 0.0
accum_dth = 0.0

print("\n🚀 V2: Probabilistisk SLAM (Med realistisk 50ms Sensor-delay)")
print("🕹️ Klikk på plottet og styr med piltastene!")

while plt.fignum_exists(fig.number):
    
    # =========================================================
    # 1. RASK FYSIKK & ODOMETRI (Kjører 100 ganger i sekundet)
    # =========================================================
    new_th = normalize_angle(true_pose[2] + w_cmd * dt)
    new_x = true_pose[0] + v_cmd * np.cos(true_pose[2]) * dt
    new_y = true_pose[1] + v_cmd * np.sin(true_pose[2]) * dt
    
    if 0.2 < new_x < ROOM_SIZE-0.2 and 0.2 < new_y < ROOM_SIZE-0.2:
        true_pose = np.array([new_x, new_y, new_th])
    else:
        true_pose[2] = new_th 
    true_poses_hist.append(true_pose.copy())
    
    if v_cmd != 0 or w_cmd != 0:
        std_v = 0.05 * abs(v_cmd) + 0.05 * abs(w_cmd) 
        std_w = 0.05 * abs(v_cmd) + 0.05 * abs(w_cmd) 
        v_noisy = v_cmd + np.random.normal(0, std_v)
        w_noisy = w_cmd + np.random.normal(0, std_w)
    else:
        v_noisy, w_noisy = 0.0, 0.0
        
    # Akkumuler odometri (Vi bygger opp distansen til neste gang sensoren er klar)
    accum_dx += v_noisy * dt
    accum_dth += w_noisy * dt
    
    # =========================================================
    # 2. TREG SENSOR & SLAM OPPDATERING (Kjører kun hver 50ms)
    # =========================================================
    sensor_timer += dt
    if sensor_timer >= SENSOR_RATE:
        
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
                final_dist += np.random.normal(0, 0.05) 
            
            for ang in np.linspace(-TOF_FOV/2, TOF_FOV/2, 15):
                sensor_scan.append((ang, final_dist))

            if nearest_obj_dist < MAX_RANGE and nearest_obj_dist < wall_dist:
                lm_pos = TRUE_LANDMARKS[nearest_obj_id]
                bearing = normalize_angle(np.arctan2(lm_pos[1] - true_pose[1], lm_pos[0] - true_pose[0]) - true_pose[2])
                
                if abs(bearing) < CAM_FOV/2:
                    bearing_noisy = bearing + np.random.normal(0, 0.02)
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
                        if 0 < dist_to_surf < dist: dist = dist_to_surf
                if dist < MAX_RANGE:
                    dist += np.random.normal(0, 0.01) 
                    sensor_scan.append((ang, dist))

        # --- SEND DATA TIL SLAM ---
        # SLAM får all oppsamlet bevegelse siden forrige gang, pluss ferske sensordata!
        slam.update(accum_dx, 0.0, accum_dth, sensor_scan, nicla_data)
        
        # Nullstill tellerne til neste pakke overføres
        accum_dx = 0.0
        accum_dth = 0.0
        sensor_timer -= SENSOR_RATE 
                
        # =========================================================
        # 3. PLOTTING (Skjer kun når vi har nye SLAM-data)
        # =========================================================
        if len(slam.poses) % 2 == 0:
            ax.clear()
            
            prob_grid = 1.0 - (1.0 / (1.0 + np.exp(slam.grid)))
            extent = [-1.0, -1.0 + slam.grid_w*slam.grid_res, -1.0, -1.0 + slam.grid_h*slam.grid_res]
            ax.imshow(prob_grid, cmap='bone_r', origin='lower', extent=extent, vmin=0.0, vmax=1.0)
            
            for wall in TRUE_WALLS:
                ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], 'b--', linewidth=1.5)
            for id, pos in TRUE_LANDMARKS.items():
                ax.add_patch(Circle((pos[0], pos[1]), OBJ_RADIUS, color='lightblue', alpha=0.3))
                
            true_arr = np.array(true_poses_hist)
            slam_arr = np.array(slam.poses)
            odom_arr = np.array(slam.odom_poses)
            
            ax.plot(true_arr[:,0], true_arr[:,1], 'b-', alpha=0.3, label="Sann rute")
            ax.plot(odom_arr[:,0], odom_arr[:,1], 'g--', alpha=0.5, label="Ren Odometri")
            ax.plot(slam_arr[:,0], slam_arr[:,1], 'r-', linewidth=2, label="V2: SLAM (50ms delay)")

            active_sm = slam.submaps[-1]
            if len(active_sm.raw_points) > 0:
                c, s = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
                R = np.array([[c, -s], [s, c]])
                global_pts = np.dot(active_sm.raw_points, R.T) + active_sm.origin[:2]
                ax.scatter(global_pts[:, 0], global_pts[:, 1], s=5, c='magenta', alpha=0.5, label="Submap Rådata", zorder=5)
            
            curr = slam_arr[-1]
            ax.add_patch(Circle((curr[0], curr[1]), 0.15, color='red', alpha=0.3))
            car_rect = Rectangle((curr[0]-0.1, curr[1]-0.075), 0.2, 0.15, angle=np.rad2deg(curr[2]), color='darkred')
            ax.add_patch(car_rect)
            ax.plot([curr[0], curr[0] + 0.2*np.cos(curr[2])], [curr[1], curr[1] + 0.2*np.sin(curr[2])], 'y-', linewidth=2.5)
                
            ax.set_xlim(-1, ROOM_SIZE + 1); ax.set_ylim(-1, ROOM_SIZE + 1)
            ax.set_title("V2: SLAM | Realistisk Kommunikasjons-delay")
            ax.legend(loc="upper left", fontsize=8)
            
            # Oppdatert pause slik at animasjonen forblir "sanntid"
            plt.pause(0.001) 

plt.ioff()