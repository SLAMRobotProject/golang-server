import numpy as np
from scipy.optimize import least_squares
from map import Submap, voxel_downsample

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class GlobalSLAMBackend:
    def __init__(self, start_pose=[0.5, 0.5, 0.0]):
        start_array = np.array(start_pose)
        self.submaps = [Submap(start_array)]
        self.poses = [start_array.copy()]
        
        self.edges_odom = [] 
        self.edges_loop = [] 
        
        self.last_icp_pose = start_array.copy()
        self.DIST_THRESH = 0.015  # Nedskalert fra 0.03
        self.ANGLE_THRESH = 0.05

        # SLAM Konfigurasjon
        self.SUBMAP_RADIUS = 0.5 # 0.5 meter per kart (Redusert fra 1.0 siden alt er mindre)
        self.last_lc_idx = -100

    def get_relative_pose(self, p1, p2):
        """ Finner ut hvor p2 er relativt til p1 (Lokalt koordinatsystem) """
        c, s = np.cos(p1[2]), np.sin(p1[2])
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        return np.array([dx*c + dy*s, -dx*s + dy*c, normalize_angle(p2[2] - p1[2])])

    def fast_submap_correlative_match(self, active_pts, old_sm, global_guess, job_type='global'):
        """ Kjerne-matcheren vår. Skyver kart oppå hverandre for å finne beste passform. """
        if len(active_pts) == 0 or len(old_sm.clean_points) == 0 or old_sm.match_grid is None:
            return global_guess, False

        # Profiler for søket
        if job_type == 'local':
            cfg = {
                'min_overlap': 0.60,
                'search_xy': 0.10,  'search_th': 0.20,
                'pen_dist': 0.5,    'pen_rot': 0.8,
                'c_step_xy': 0.025, 'c_step_th': 0.02,
                'f_step_xy': 0.010, 'f_step_th': 0.005
            }
        else: # 'global'
            cfg = {
                'min_overlap': 0.60,
                'search_xy': 0.75,  'search_th': 0.50, # Halvert fra 1.50
                'pen_dist': 0.05,   'pen_rot': 0.2,
                'c_step_xy': 0.10,  'c_step_th': 0.05,
                'f_step_xy': 0.02,  'f_step_th': 0.01
            }

        inv_res = 1.0 / old_sm.grid_res
        c_old, s_old = np.cos(old_sm.origin[2]), np.sin(old_sm.origin[2])

        best_pose = global_guess.copy()
        best_coarse_pose = best_pose.copy()
        best_score = -np.inf

        # --- 1. GROVSØK ---
        for dth in np.arange(-cfg['search_th'], cfg['search_th'] + cfg['c_step_th'], cfg['c_step_th']):
            test_th = global_guess[2] + dth
            rel_th = normalize_angle(test_th - old_sm.origin[2])
            c, s = np.cos(rel_th), np.sin(rel_th)
            
            rot_x = active_pts[:, 0] * c - active_pts[:, 1] * s
            rot_y = active_pts[:, 0] * s + active_pts[:, 1] * c
            rot_penalty = abs(dth) * cfg['pen_rot']
            
            for dx in np.arange(-cfg['search_xy'], cfg['search_xy'] + cfg['c_step_xy'], cfg['c_step_xy']):
                for dy in np.arange(-cfg['search_xy'], cfg['search_xy'] + cfg['c_step_xy'], cfg['c_step_xy']):
                    test_x, test_y = global_guess[0] + dx, global_guess[1] + dy
                    
                    glob_dx = test_x - old_sm.origin[0]
                    glob_dy = test_y - old_sm.origin[1]
                    rel_x = glob_dx * c_old + glob_dy * s_old
                    rel_y = -glob_dx * s_old + glob_dy * c_old
                    
                    final_pts_x = rot_x + rel_x
                    final_pts_y = rot_y + rel_y
                    
                    ix = ((final_pts_x - old_sm.min_x) * inv_res).astype(int)
                    iy = ((final_pts_y - old_sm.min_y) * inv_res).astype(int)
                    valid = (ix >= 0) & (ix < old_sm.w) & (iy >= 0) & (iy < old_sm.h)
                    
                    num_in_bounds = np.sum(valid)
                    # SIKKERHETSLÅS: Må ha minst 15 punkter felles (The Overlap Trap)
                    if num_in_bounds >= 15:
                        score = np.sum(old_sm.match_grid[iy[valid], ix[valid]]) / num_in_bounds
                    else:
                        score = 0.0
                        
                    total_penalty = (np.hypot(dx, dy) * cfg['pen_dist']) + rot_penalty
                    
                    if (score - total_penalty) > best_score:
                        best_score = score - total_penalty
                        best_coarse_pose = np.array([test_x, test_y, test_th])

        # --- 2. FINSØK ---
        fine_xy, fine_th = cfg['c_step_xy'], cfg['c_step_th']
        for dth in np.arange(-fine_th, fine_th + cfg['f_step_th'], cfg['f_step_th']):
            test_th = best_coarse_pose[2] + dth
            rel_th = normalize_angle(test_th - old_sm.origin[2])
            c, s = np.cos(rel_th), np.sin(rel_th)
            
            rot_x = active_pts[:, 0] * c - active_pts[:, 1] * s
            rot_y = active_pts[:, 0] * s + active_pts[:, 1] * c
            rot_penalty = abs(normalize_angle(test_th - global_guess[2])) * cfg['pen_rot']
            
            for dx in np.arange(-fine_xy, fine_xy + cfg['f_step_xy'], cfg['f_step_xy']):
                for dy in np.arange(-fine_xy, fine_xy + cfg['f_step_xy'], cfg['f_step_xy']):
                    test_x, test_y = best_coarse_pose[0] + dx, best_coarse_pose[1] + dy
                    
                    glob_dx = test_x - old_sm.origin[0]
                    glob_dy = test_y - old_sm.origin[1]
                    rel_x = glob_dx * c_old + glob_dy * s_old
                    rel_y = -glob_dx * s_old + glob_dy * c_old
                    
                    final_pts_x = rot_x + rel_x
                    final_pts_y = rot_y + rel_y
                    
                    ix = ((final_pts_x - old_sm.min_x) * inv_res).astype(int)
                    iy = ((final_pts_y - old_sm.min_y) * inv_res).astype(int)
                    valid = (ix >= 0) & (ix < old_sm.w) & (iy >= 0) & (iy < old_sm.h)
                    
                    num_in_bounds = np.sum(valid)
                    if num_in_bounds >= 15:
                        score = np.sum(old_sm.match_grid[iy[valid], ix[valid]]) / num_in_bounds
                    else:
                        score = 0.0
                        
                    total_penalty = (np.hypot(test_x - global_guess[0], test_y - global_guess[1]) * cfg['pen_dist']) + rot_penalty
                    
                    if (score - total_penalty) > best_score:
                        best_score = score - total_penalty
                        best_pose = np.array([test_x, test_y, test_th])

        if best_score > cfg['min_overlap']:
            return best_pose, True
        return global_guess, False

    def optimize_graph(self):
        """ Pose Graph Optimization: Fordeler feil bakover i historikken """
        print("\n🚀 LOOP CLOSURE! Kjører Graph-SLAM...")
        def residuals(state):
            res = []
            origins = state.reshape((-1, 3))
            
            # Anker: Submap 0 står bom fast
            res.extend((origins[0] - self.submaps[0].origin) * 1000)
            
            # Odometri-kanter (Lite tillit til rotasjon)
            for (i, j, rel) in self.edges_odom:
                pi, pj = origins[i], origins[j]
                c, s = np.cos(pi[2]), np.sin(pi[2])
                pred_dx = (pj[0] - pi[0])*c + (pj[1] - pi[1])*s
                pred_dy = -(pj[0] - pi[0])*s + (pj[1] - pi[1])*c
                pred_dth = normalize_angle(pj[2] - pi[2])
                res.extend([ 
                    (pred_dx - rel[0]) * 2.0, 
                    (pred_dy - rel[1]) * 2.0, 
                    normalize_angle(pred_dth - rel[2]) * 0.5 
                ])
                
            # Loop Closure-kanter (Høy tillit!)
            for (i, j, rel) in self.edges_loop:
                pi, pj = origins[i], origins[j]
                c, s = np.cos(pi[2]), np.sin(pi[2])
                pred_dx = (pj[0] - pi[0])*c + (pj[1] - pi[1])*s
                pred_dy = -(pj[0] - pi[0])*s + (pj[1] - pi[1])*c
                pred_dth = normalize_angle(pj[2] - pi[2])
                res.extend([ 
                    (pred_dx - rel[0]) * 5.0, 
                    (pred_dy - rel[1]) * 5.0, 
                    normalize_angle(pred_dth - rel[2]) * 10.0 
                ])
            return np.array(res)

        init_state = np.array([sm.origin for sm in self.submaps]).flatten()
        result = least_squares(residuals, init_state, loss='huber')
        opt_origins = result.x.reshape((-1, 3))
        
        for i, sm in enumerate(self.submaps):
            sm.origin = opt_origins[i]
        print("✅ PGO Ferdig!")

    def update(self, ekf_pose, sensor_scan):
        """ Hovedfunksjon. Kalles hver gang vi mottar en payload fra roboten. """
        active_sm = self.submaps[-1]
        
        dist_moved = np.hypot(ekf_pose[0] - self.last_icp_pose[0], ekf_pose[1] - self.last_icp_pose[1])
        angle_moved = abs(normalize_angle(ekf_pose[2] - self.last_icp_pose[2]))
        
        if dist_moved >= self.DIST_THRESH or angle_moved >= self.ANGLE_THRESH:
            
            # 1. Behandle rådata
            raw_pts = []
            for angle, dist in sensor_scan:
                if 0.2 < dist <= active_sm.MAX_BUILD_DIST:
                    raw_pts.append([dist * np.cos(angle), dist * np.sin(angle)])
            raw_pts = np.array(raw_pts)

            if len(raw_pts) > 0:
                rel_pose_raw = self.get_relative_pose(active_sm.origin, ekf_pose)
                c, s = np.cos(rel_pose_raw[2]), np.sin(rel_pose_raw[2])
                trans_pts = np.dot(raw_pts, np.array([[c, -s], [s, c]]).T) + rel_pose_raw[:2]
                
                # Vask dataen! (voxel nedskalert til 5cm)
                clean_voxel_pts = voxel_downsample(trans_pts, voxel_size=0.08)
                active_sm.raw_points = np.vstack((active_sm.raw_points, clean_voxel_pts))
                active_sm.scans.append((rel_pose_raw[0], rel_pose_raw[1], rel_pose_raw[2], sensor_scan))
                active_sm.add_scan_to_local_grid(rel_pose_raw, sensor_scan)
            
            self.last_icp_pose = ekf_pose.copy() 

            # 2. Pakking av Submap (Areal-basert Logikk)
            dist_from_origin = np.hypot(ekf_pose[0] - active_sm.origin[0], ekf_pose[1] - active_sm.origin[1])
            has_good_geometry = False
            spatial_coverage = 0 
            is_ready_to_pack = False
            num_pts = len(active_sm.raw_points)
            
            if num_pts >= 30:
                pts = active_sm.raw_points
                spread_x = np.max(pts[:, 0]) - np.min(pts[:, 0])
                spread_y = np.max(pts[:, 1]) - np.min(pts[:, 1])
                
                if (spread_x > 0.5 and spread_y > 0.5) or dist_from_origin > self.SUBMAP_RADIUS or num_pts > 80:
                    voxels = np.round(pts / 0.08)
                    spatial_coverage = len(np.unique(voxels, axis=0))
                    
                    if spread_x > 0.5 and spread_y > 0.5 and spatial_coverage >= 20:
                        has_good_geometry = True

                    is_ready_to_pack = (dist_from_origin > self.SUBMAP_RADIUS and has_good_geometry) or (spatial_coverage >= 80) or (dist_from_origin > 3.0)

            if is_ready_to_pack:
                current_idx = len(self.submaps) - 1
                rel_odom = self.get_relative_pose(active_sm.origin, ekf_pose) 
                
                if spatial_coverage < 15 and not has_good_geometry:
                    print(f"🗑️ Kaster data fra Submap {current_idx}")
                    active_sm.raw_points = np.empty((0, 2)) 
                else:
                    print(f"📦 Pakker dynamisk Submap {current_idx} ({spatial_coverage} voxels)")
                    active_sm.extract_clean_points_and_build_lf(sigma=0.12)
                    
                    # A. LOKAL KORREKSJON
                    if current_idx > 0:
                        prev_sm = self.submaps[-2]
                        clean_match_pts = voxel_downsample(active_sm.raw_points, voxel_size=0.08)
                        opt_origin, success = self.fast_submap_correlative_match(
                            clean_match_pts, prev_sm, active_sm.origin, job_type='local'
                        )
                        
                        if success:
                            print(f"🤝 LOKAL KORREKSJON: Sydde fast i Submap {current_idx-1}")
                            active_sm.origin = opt_origin
                            new_rel = self.get_relative_pose(prev_sm.origin, active_sm.origin)
                            self.edges_odom[-1] = (current_idx - 1, current_idx, new_rel)
                            
                            # Oppdater bilens posisjon
                            c_new, s_new = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
                            ekf_pose = np.array([
                                active_sm.origin[0] + rel_odom[0]*c_new - rel_odom[1]*s_new,
                                active_sm.origin[1] + rel_odom[0]*s_new + rel_odom[1]*c_new,
                                normalize_angle(active_sm.origin[2] + rel_odom[2])
                            ])
                            rel_odom = self.get_relative_pose(active_sm.origin, ekf_pose)

                    # B. SUPER-SUBMAP LOOP CLOSURE
                    MIN_SUBMAPS_BEFORE_LC = 4
                    COOLDOWN_SUBMAPS = 3 
                    GROUP_SIZE = 3 
                    
                    if len(self.submaps) >= MIN_SUBMAPS_BEFORE_LC and (current_idx - self.last_lc_idx) >= COOLDOWN_SUBMAPS:
                        candidates = []
                        for old_idx, old_sm in enumerate(self.submaps[:-GROUP_SIZE - 4]): 
                            dist_to_old = np.hypot(active_sm.origin[0] - old_sm.origin[0], active_sm.origin[1] - old_sm.origin[1])
                            if dist_to_old < 2.0 and not any(e[0]==old_idx and e[1]==current_idx for e in self.edges_loop):
                                candidates.append((dist_to_old, old_idx, old_sm))
                        
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
                                super_points = voxel_downsample(np.vstack(super_points), voxel_size=0.08)
                                
                                for dist, old_idx, old_sm in candidates[:3]: 
                                    loop_origin, success = self.fast_submap_correlative_match(
                                        super_points, old_sm, center_sm.origin, job_type='global'
                                    )
                                    
                                    if success:
                                        print(f"🔗 UNIKT FINGERAVTRYKK FUNNET! (Kobler til Submap {old_idx})")
                                        rel_loop = self.get_relative_pose(old_sm.origin, loop_origin)
                                        self.edges_loop.append((old_idx, current_idx, rel_loop))
                                        self.last_lc_idx = current_idx 
                                        
                                        self.optimize_graph() 
                                        
                                        # Hent inn den oppdaterte posisjonen etter gummistrikk-korreksjon
                                        c_new, s_new = np.cos(active_sm.origin[2]), np.sin(active_sm.origin[2])
                                        ekf_pose = np.array([
                                            active_sm.origin[0] + rel_odom[0]*c_new - rel_odom[1]*s_new,
                                            active_sm.origin[1] + rel_odom[0]*s_new + rel_odom[1]*c_new,
                                            normalize_angle(active_sm.origin[2] + rel_odom[2])
                                        ])
                                        break 

                # Klargjør neste subkart
                self.edges_odom.append((current_idx, current_idx + 1, self.get_relative_pose(active_sm.origin, ekf_pose)))
                self.submaps.append(Submap(ekf_pose))

        self.poses.append(ekf_pose)

        return ekf_pose



