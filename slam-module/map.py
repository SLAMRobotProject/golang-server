import numpy as np
from scipy.ndimage import distance_transform_edt

def voxel_downsample(points, voxel_size=0.10):
    """ 
    Gjør punktskyen lettere og renere ved å samle punkter i 10cm-bokser, 
    og returnere gjennomsnittet av punktene i hver boks.
    """
    if len(points) == 0:
        return points
        
    voxels = np.round(points / voxel_size)
    unique_voxels, inverse_idx, counts = np.unique(
        voxels, axis=0, return_inverse=True, return_counts=True
    )
    
    centroids = np.zeros((len(unique_voxels), 2))
    np.add.at(centroids, inverse_idx, points) 
    centroids = centroids / counts[:, np.newaxis]
    
    return centroids

class Submap:
    def __init__(self, origin_pose, grid_res=0.04, grid_size=4.0):
        self.origin = origin_pose.copy() 
        self.scans = [] 

        self.raw_points = np.empty((0, 2))
        self.clean_points = np.empty((0, 2))
        
        # =========================================================
        # LOKALT OCCUPANCY GRID (Fjerner støy, "Viskelær-effekt")
        # =========================================================
        self.grid_res = grid_res
        self.grid_size = grid_size 
        self.grid_w = int(self.grid_size / self.grid_res)
        self.grid_h = int(self.grid_size / self.grid_res)
        self.local_grid = np.zeros((self.grid_h, self.grid_w))
        self.offset = self.grid_size / 2.0 # (0,0) i midten av gridet
        
        # Likelihood Field variabler
        self.match_grid = None
        self.min_x = 0.0
        self.min_y = 0.0
        self.w = 0
        self.h = 0
        
        self.MAX_BUILD_DIST = 1.0 # Bygger kun kart av det vi er sikre på (1.5m for ToF)

    def add_scan_to_local_grid(self, rel_pose, sensor_scan):
        """ Tegner inn ferske ToF-stråler i det lokale rutenettet """
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
            
            # 1. Viskelær (Freespace) - Straffer celler mellom roboten og veggen
            for i in range(steps):
                cx = int(rx + (hx - rx) * (i / steps))
                cy = int(ry + (hy - ry) * (i / steps))
                if 0 <= cx < self.grid_w and 0 <= cy < self.grid_h: 
                    self.local_grid[cy, cx] -= 0.15 
                    
            # 2. Vegg (Occupied) - Belønner cellen der strålen traff
            if dist_m <= self.MAX_BUILD_DIST:
                if 0 <= hx < self.grid_w and 0 <= hy < self.grid_h: 
                    weight = np.exp(-dist_m / 2.0)
                    self.local_grid[hy, hx] += 0.5 * weight
                
        # Klipp log-odds for å unngå at vi blir "for" sikre
        self.local_grid = np.clip(self.local_grid, -5.0, 5.0)

    def extract_clean_points_and_build_lf(self, sigma=0.10, padding=2.0):
        """ 
        Kalles NÅR SUBKARTET PAKKES. 
        Gjør rutenettet om til punkter og bygger "Magnetfeltet" (Likelihood Field).
        """
        # 1. Hent ut kun de cellene som vi er SIKRE på er en vegg (log-odds > 0.2)
        y_idx, x_idx = np.where(self.local_grid > 0.2)
        clean_pts = []
        
        for y, x in zip(y_idx, x_idx):
            real_x = (x * self.grid_res) - self.offset
            real_y = (y * self.grid_res) - self.offset
            clean_pts.append([real_x, real_y])
            
        self.clean_points = np.array(clean_pts)
        if len(self.clean_points) == 0: return
        
        # 2. Finn Bounding Box for kartet
        self.min_x = np.min(self.clean_points[:, 0]) - padding
        max_x = np.max(self.clean_points[:, 0]) + padding
        self.min_y = np.min(self.clean_points[:, 1]) - padding
        max_y = np.max(self.clean_points[:, 1]) + padding
        
        inv_res = 1.0 / self.grid_res
        self.w = int((max_x - self.min_x) * inv_res)
        self.h = int((max_y - self.min_y) * inv_res)
        
        occ = np.ones((self.h, self.w))
        
        # 🚀 LYN-RASK NUMPY VEKTORISERING (Eliminerer for-løkker)
        ix = ((self.clean_points[:, 0] - self.min_x) * inv_res).astype(int)
        iy = ((self.clean_points[:, 1] - self.min_y) * inv_res).astype(int)
        
        valid = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        occ[iy[valid], ix[valid]] = 0.0 
        
        # 3. Euclidean Distance Transform (Lager Gaussian-feltet)
        dist = distance_transform_edt(occ) * self.grid_res
        self.match_grid = np.exp(-(dist**2) / (2 * sigma**2))

def get_global_occupancy_grid(submaps, grid_res=0.04, room_size=4.0):
    """
    Bygger et samlet 2D bilde av alle subkartene basert på EKTE log-odds.
    """
    min_x, max_x = -1.0, room_size + 1.0
    min_y, max_y = -1.0, room_size + 1.0
    
    w = int((max_x - min_x) / grid_res)
    h = int((max_y - min_y) / grid_res)
    
    # Setter hele verden til 0.5 (Ukjent / Grått)
    global_grid = np.full((h, w), 0.5) 
    
    for sm in submaps:
        # Rotasjonsmatrise for subkartet
        c, s = np.cos(sm.origin[2]), np.sin(sm.origin[2])
        R = np.array([[c, -s], [s, c]])
        
        # --- 1. TEGN FREESPACE (VISKELÆR) ---
        # Hent ut alle celler i subkartet som vi VET er ledige (log-odds < -0.1)
        y_free, x_free = np.where(sm.local_grid < -0.1)
        if len(y_free) > 0:
            # Konverter grid-indeks til lokale meter
            local_free_x = (x_free * sm.grid_res) - sm.offset
            local_free_y = (y_free * sm.grid_res) - sm.offset
            local_free_pts = np.column_stack((local_free_x, local_free_y))
            
            # Transformer til global posisjon
            global_free_pts = np.dot(local_free_pts, R.T) + sm.origin[:2]
            
            # Gå fra globale meter til det store bildets piksler
            ix_f = np.clip(((global_free_pts[:, 0] - min_x) / grid_res).astype(int), 0, w - 1)
            iy_f = np.clip(((global_free_pts[:, 1] - min_y) / grid_res).astype(int), 0, h - 1)
            
            # Fargelegg som hvitt (0.0)
            global_grid[iy_f, ix_f] = 0.0  
            
        # --- 2. TEGN VEGGER (OCCUPIED) ---
        # Hent ut celler som vi er SIKRE på er vegg (log-odds > 0.2)
        y_occ, x_occ = np.where(sm.local_grid > 0.2)
        if len(y_occ) > 0:
            local_occ_x = (x_occ * sm.grid_res) - sm.offset
            local_occ_y = (y_occ * sm.grid_res) - sm.offset
            local_occ_pts = np.column_stack((local_occ_x, local_occ_y))
            
            global_occ_pts = np.dot(local_occ_pts, R.T) + sm.origin[:2]
            
            ix_o = np.clip(((global_occ_pts[:, 0] - min_x) / grid_res).astype(int), 0, w - 1)
            iy_o = np.clip(((global_occ_pts[:, 1] - min_y) / grid_res).astype(int), 0, h - 1)
            
            # Fargelegg som svart (1.0)
            global_grid[iy_o, ix_o] = 1.0  

    extent = (min_x, max_x, min_y, max_y)
    return global_grid, extent
