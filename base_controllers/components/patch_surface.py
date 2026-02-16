from .terrain_manager import TerrainManager
from .point_cloud_filter import PointCloudFilter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from scipy.interpolate import griddata

class PatchSurface:
    
    def __init__(self, points_t, number_of_patches_width=10, number_of_patches_height=10):
        # quante patches in y e z 
        self.number_of_patches_width = number_of_patches_width
        self.number_of_patches_height = number_of_patches_height
        
        # self.patch_width
        # self.get_all_cost
        N = len(points_t)
        self.points_t = points_t
        self.all_pc = np.vstack([p['position'] for p in points_t])
        
        self.y_min, self.y_max = float(self.all_pc[:, 1].min()), float(self.all_pc[:, 1].max())
        self.z_min, self.z_max = float(self.all_pc[:, 2].min()), float(self.all_pc[:, 2].max())
        
        #paramters for patches
        # self.id = np.arange(N)
        # self.some_points = []
        # self.center_point = np.zeros(N)
        # self.patch_cost = np.zeros(N)
        # self.patch_color = np.full((N, 3), [0,0,0])
        
        self.patches = [ ]
                        # {
                        # 'id': self.id[i],
                        # 'points': self.some_points[i],
                        # 'centroid': self.center_point[i],
                        # 'cost': self.patch_cost[i],
                        # 'color': self.patch_color[i],
                        # }
    
        # check status points:
        if not self.points_t:
            print ("[patch_surface] Error: No points type points_t provided for patch surface creation.")
            return
        self.lx, self.ly, self.patch_height, self.patch_width = self.dimension_of_map()
        print(f"[patch_surface] dimension of map and patch: Ly = {self.lx:.2f} m, Lz = {self.ly:.2f} m, patch_height = {self.patch_height:.2f} m, patch_width = {self.patch_width:.2f} m")
        
        self.create_patches()
    
    #dimension of the map and patches 
    def dimension_of_map(self):
        '''
        Returns the total dimensions of the map and individual patches
        '''
        Ly = self.y_max - self.y_min
        Lz = self.z_max - self.z_min
        
        # Dimensions of single patch
        patch_width = Ly / self.number_of_patches_width if self.number_of_patches_width else 0.0
        patch_height = Lz / self.number_of_patches_height if self.number_of_patches_height else 0.0
        return Ly, Lz, patch_height, patch_width
    
    def create_patches(self):
        '''
        create patches from point cloud
        '''
        Ly, Lz, patch_height, patch_width = self.dimension_of_map()

        y_edges = self.y_min + np.arange(self.number_of_patches_width + 1)  * patch_width
        z_edges = self.z_min + np.arange(self.number_of_patches_height + 1) * patch_height
        
        patch_id = 0
        
        for j in reversed(range(self.number_of_patches_height)): # ora parte da alto a sx e va ogni riga
            for i in range(self.number_of_patches_width):
                #border of the patch
                y_min = y_edges[i]
                y_max = y_edges[i + 1]
                z_min = z_edges[j]
                z_max = z_edges[j + 1]
                #mask for points in the patch
                mask = (self.all_pc[:, 1] >= y_min) & (self.all_pc[:, 1] < y_max) & \
                       (self.all_pc[:, 2] >= z_min) & (self.all_pc[:, 2] < z_max)
                
                idx = np.where(mask)[0]
                points_in_patch = [self.points_t[k] for k in idx]
                
                mean_cost = self.cost_patch(points_in_patch)
                centroid = self.centroid_patch(points_in_patch, y_min, y_max, z_min, z_max)
                #add point cloud and id inside the self.patches
                self.patches.append({
                    'id': patch_id,
                    'centroid': centroid,
                    'points_in_patch': points_in_patch,
                    'cost_patch': mean_cost,
                })
                
                patch_id += 1
        
    # === CHECKER METHODS ===
    def is_point_in_patch(self, patch_id, point): 
        '''
        Check if a given point is within the boundaries of a specified patch.
        '''
        if not self.patches[patch_id]['points_in_patch']:
            print(f"[patch_surface] Patch {patch_id} has no points, cannot contain any point")
            return False 
               
        if patch_id < 0 or patch_id >= len(self.patches):
            return False
        
        # Calculate patch boundaries
        row_idx = patch_id // self.number_of_patches_width
        i = patch_id % self.number_of_patches_width
        j = (self.number_of_patches_height - 1) - row_idx
        # Calcola i confini usando gli stessi edges di create_patches()
        y_edges = self.y_min + np.arange(self.number_of_patches_width + 1) * self.patch_width
        z_edges = self.z_min + np.arange(self.number_of_patches_height + 1) * self.patch_height
        
        y_min = y_edges[i]
        y_max = y_edges[i + 1] 
        z_min = z_edges[j]
        z_max = z_edges[j + 1]
        
        # Get patch centroid for x reference
        patch_centroid = self.patches[patch_id]['centroid']
        x_centroid = patch_centroid[0]
        x_threshold = 0.5
        # Check if point is within patch boundaries
        y_coord = point['position'][1]
        z_coord = point['position'][2]
        x_coord = point['position'][0]
            
        # Check y and z boundaries
        within_yz = (y_min <= y_coord < y_max) and (z_min <= z_coord < z_max)
        
        # Check x threshold
        within_x = abs(x_coord - x_centroid) <= x_threshold
        
        if within_yz and within_x:
            print(f"[patch_surface] Point {point['position']} is within patch {patch_id}")
            print(f"[patch_surface] Patch {patch_id} boundaries: y[{y_min}, {y_max}], z[{z_min}, {z_max}]")
            print(f"[patch_surface] X centroid: {x_centroid:.3f}, point X: {x_coord:.3f}, threshold: ±{x_threshold}")
            return True
        else:
            print(f"[patch_surface] Point {point['position']} is NOT within patch {patch_id}")
            print(f"[patch_surface] Patch {patch_id} boundaries: y[{y_min}, {y_max}], z[{z_min}, {z_max}]")
            print(f"[patch_surface] X centroid: {x_centroid:.3f}, point X: {x_coord:.3f}, threshold: ±{x_threshold}")
            if not within_yz:
                print("[patch_surface]   - Point outside Y/Z boundaries")
            if not within_x:
                print(f"[patch_surface]   - Point outside X threshold (distance: {abs(x_coord - x_centroid):.3f})")
            return False
        
    def is_point_2D_in_patch(self, patch_id, y_point, z_point):
        '''
        Check if a given (y,z) point is within the boundaries of a specified patch.
        '''
        if patch_id < 0 or patch_id >= len(self.patches):
            return False
        
        # Calculate patch boundaries
        row_idx = patch_id // self.number_of_patches_width
        i = patch_id % self.number_of_patches_width
        j = (self.number_of_patches_height - 1) - row_idx
        # Calcola i confini usando gli stessi edges di create_patches()
        y_edges = self.y_min + np.arange(self.number_of_patches_width + 1) * self.patch_width
        z_edges = self.z_min + np.arange(self.number_of_patches_height + 1) * self.patch_height
        
        y_min = y_edges[i]
        y_max = y_edges[i + 1] 
        z_min = z_edges[j]
        z_max = z_edges[j + 1]
    
        y_coord = y_point
        z_coord = z_point
            
        # Check y and z boundaries
        within_yz = (y_min <= y_coord < y_max) and (z_min <= z_coord < z_max)
        
        if within_yz:
            # print(f"[patch_surface] Point is within patch {patch_id} in YZ plane")
            # print(f"[patch_surface] Patch {patch_id} boundaries: y[{y_min}, {y_max}], z[{z_min}, {z_max}]")
            return True
        else:
            return False
                 
    def cost_patch(self, some_points): 
        '''
        compute the mean cost of the patch
        '''              
        costs = [p['cost'] for p in some_points if 'cost' in p]
        if not costs:
           return None # nessun costo disponibile per questa patch
        return float(np.mean(costs))
    
    def centroid_patch(self, points_in_patch, y_min, y_max, z_min, z_max):
        '''
        return the centroid of the patch
        '''
        y_centroid = (y_min + y_max) / 2.0
        z_centroid = (z_min + z_max) / 2.0
        if points_in_patch:
            x_positions = [p['position'][0] for p in points_in_patch]
            x_centroid = np.mean(x_positions)
        else:
            x_centroid = np.mean(self.all_pc[:, 0])
        
        return np.array([x_centroid, y_centroid, z_centroid])
        
    def normal_vector_of_point_in_patch(self, patch_id, point, print_info = False, plot_normal_patch=False):
        '''
        Calculate the normal vector of the surface at a specific point within a specified patch.
        '''
        if not self.is_point_in_patch(patch_id, point):
            print(f"[patch_surface] Point is not in patch {patch_id}, cannot calculate normal vector")
            return None
        
        points_in_patch = self.patches[patch_id]['points_in_patch']
        
        if len(points_in_patch) < 4:
            print(f"[patch_surface] Patch {patch_id} has insufficient points ({len(points_in_patch)}) to create mesh grid")
            return None
        X_grid, Y_grid, Z_grid = self.get_mesh_grid_patch(patch_id, plot_patch=False)
        
        dX_dZ, dX_dY = np.gradient(X_grid)

        target_point = point['position']
        
        distances = np.sqrt((X_grid - target_point[0])**2 + 
                        (Y_grid - target_point[1])**2 + 
                        (Z_grid - target_point[2])**2)
        #find the index of the closest point
        min_idx = np.unravel_index(np.argmin(distances), distances.shape)
        i, j = min_idx
        dx_dy_local = dX_dY[i, j]
        dx_dz_local = dX_dZ[i, j]
        
        # Il vettore normale alla superficie X = f(Y,Z) è (1, -dX/dY, -dX/dZ)
        # Questo deriva dal fatto che se x = f(y,z), il piano tangente ha equazione:
        # x - x0 = (df/dy)(y - y0) + (df/dz)(z - z0)
        # che può essere riscritta come: x - (df/dy)y - (df/dz)z + costante = 0
        # quindi il vettore normale è (1, -df/dy, -df/dz)
        normal = np.array([1.0, -dx_dy_local, -dx_dz_local])
        
        normal_magnitude = np.linalg.norm(normal)
        if normal_magnitude == 0:
            print("[patch_surface] Cannot calculate normal vector: surface is flat")
            return None
        
        normal_unit = normal / normal_magnitude
        
        if normal_unit[0] < 0:
            normal_unit = -normal_unit
            
        if print_info:
            print ("[patch_surface] data: info")
            print(f"[patch_surface] Normal vector calculated for point {target_point} in patch {patch_id}")
            print(f"[patch_surface] Normal vector: {normal_unit}")
            print(f"[patch_surface] Surface gradients at closest point: dX/dY = {dx_dy_local:.4f}, dX/dZ = {dx_dz_local:.4f}")
            print(f"[patch_surface] Closest grid point index: ({i}, {j})")
        
        if plot_normal_patch:
            
            normal_unit_scale = normal_unit * 0.5  # normal vectro scale for better visualization
            
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot_surface(X_grid, Y_grid, Z_grid, alpha=0.6, cmap='viridis')
            target_pos = point['position']
            ax.scatter(target_pos[0], target_pos[1], target_pos[2], 
                        c='red', s=100, marker='o', label='Target Point')
            closest_grid_point = np.array([X_grid[i, j], Y_grid[i, j], Z_grid[i, j]])
            ax.scatter(closest_grid_point[0], closest_grid_point[1], closest_grid_point[2], 
                        c='orange', s=80, marker='s', label='Closest Grid Point')
            
            ax.quiver(closest_grid_point[0], closest_grid_point[1], closest_grid_point[2],
                        normal_unit_scale[0], normal_unit_scale[1], normal_unit_scale[2],
                        color='purple', arrow_length_ratio=0.15, linewidth=2,
                        label='Normal Vector')
            
            ax.set_xlabel('X (depth from wall)')
            ax.set_ylabel('Y (horizontal)')
            ax.set_zlabel('Z (height)')
            ax.set_title(f'Wall Surface Normal Vector - Patch {patch_id}')
            ax.legend(fontsize=8)
            plt.tight_layout()
            plt.show()
        return normal_unit
    
    def gaussian_cost_all_patch(self, sigma=0.3, weight_gauss_cost=1.0):
        '''
        apply the gaussian function to the cost of each point in each patch
        '''
        sigma_y = sigma
        sigma_z = sigma
        all_final_costs = []
        for patch in self.patches:
            centroid = patch['centroid']
            points_in_patch = patch['points_in_patch']
            
            for point in points_in_patch:
                pos = point['position']
                
                
                diff_y_sq = (pos[1] - centroid[1])**2
                diff_z_sq = (pos[2] - centroid[2])**2
                
                
                exponent = - ( (diff_y_sq / (2 * sigma_y**2)) + (diff_z_sq / (2 * sigma_z**2)) )
                gauss_val = weight_gauss_cost * (1-np.exp(exponent)) #OCCHIO valore basso al centro!!!!
                
                point['cost'] += gauss_val
                
                all_final_costs.append(point['cost'])
                
            costs = [p['cost'] for p in points_in_patch]
            patch['cost_patch'] = float(np.mean(costs))
        self.cost_color()
        
        # Update individual point colors based on their costs
        self.update_point_colors()
        
    def update_point_colors(self):
        '''
        Update the color of each point based on its cost value
        '''
        # Collect all costs from all points
        all_costs = []
        for patch in self.patches:
            for point in patch['points_in_patch']:
                all_costs.append(point['cost'])
        
        if not all_costs:
            return
        
        all_costs = np.array(all_costs)
        min_c, max_c = np.min(all_costs), np.max(all_costs)
        
        # Normalize costs
        if max_c - min_c > 0:
            norm_costs = (all_costs - min_c) / (max_c - min_c)
        else:
            norm_costs = np.zeros_like(all_costs)
        
        # Apply colormap (red-yellow-green reversed: high cost = red, low cost = green)
        cmap = cm.get_cmap('RdYlGn_r')
        
        # Update each point's color
        idx = 0
        for patch in self.patches:
            for point in patch['points_in_patch']:
                point['color'] = cmap(norm_costs[idx])[:3]
                idx += 1
        
        print(f"[patch_surface] Updated colors for {len(all_costs)} points based on costs")
        print(f"[patch_surface] Cost range: [{min_c:.3f}, {max_c:.3f}]")             
            
    #  ==== GET METHODS ====
    def get_patches(self):
        return self.patches
    
    def get_number_of_patches(self):
        return len(self.patches)
    
    def get_patch_centroid(self, patch_id):   
        if patch_id < 0 or patch_id >= len(self.patches):
            print(f"[patch_surface] Invalid patch_id {patch_id}. Must be between 0 and {len(self.patches)-1}.")
            return None
        return self.patches[patch_id].get('centroid', None)
    
    def get_avarege_cost(self,patch_id=None):
        
        cost = self.patches[patch_id]['cost_patch']
        
        if not cost:
            print("[patch_surface] cost not found")
            return None
        return cost
    
    def get_patch_cost(self, patch_id):
        if patch_id < 0 or patch_id >= len(self.patches):
            print(f"[patch_surface] Invalid patch_id {patch_id}. Must be between 0 and {len(self.patches)-1}.")
            return None
        return self.patches[patch_id].get('cost_patch', None)

    def get_point_in_patch(self, patch_id, point):
        if patch_id < 0 or patch_id >= len(self.patches):
            print(f"[patch_surface] Invalid patch_id {patch_id}. Must be between 0 and {len(self.patches)-1}.")
            return False
        return self.is_point_in_patch(patch_id, point)

    def get_cost_in_point(self, patch_id, abs_pointyz):
        y_point = abs_pointyz[0]
        z_point = abs_pointyz[1]
        k_neighbors = 4

        # get near points in patch
        X_grid, Y_grid, Z_grid = self.get_mesh_grid_patch(patch_id, plot_patch=False)
        distances_yz = np.sqrt((Y_grid - y_point) ** 2 + (Z_grid - z_point) ** 2)
        min_idx = np.unravel_index(np.argmin(distances_yz), distances_yz.shape)
        i, j = min_idx
        x_estimated = X_grid[i, j]

        points_in_patch = self.patches[patch_id]['points_in_patch']

        new_position = np.array([x_estimated, y_point, z_point])
        distances_pc = [
            (idx, np.sqrt((p['position'][1] - y_point) ** 2 + (p['position'][2] - z_point) ** 2))
            for idx, p in enumerate(points_in_patch)
        ]
        distances_pc.sort(key=lambda x: x[1])
        neighbors_idx = [idx for idx, _ in distances_pc[:k_neighbors]]

        neighbors = [points_in_patch[idx] for idx in neighbors_idx]
        cost_avg = float(np.mean([n['cost'] for n in neighbors]))

        return cost_avg
    
    def get_mesh_grid_patch(self, patch_id, plot_patch=True):
        # Recupera i punti della patch
        points_in_patch = self.patches[patch_id]['points_in_patch']
    
        x_coords = [point['position'][0] for point in points_in_patch]
        y_coords = [point['position'][1] for point in points_in_patch]
        z_coords = [point['position'][2] for point in points_in_patch]
        # conversion to numpy array            
        x_coords = np.array(x_coords)
        y_coords = np.array(y_coords)
        z_coords = np.array(z_coords)
        
        num_points = len(points_in_patch)
        grid_size = int(np.sqrt(num_points))
        
        if grid_size * grid_size == num_points:
            # re_shape for the right format
            X_grid = x_coords.reshape((grid_size, grid_size))
            Y_grid = y_coords.reshape((grid_size, grid_size))
            Z_grid = z_coords.reshape((grid_size, grid_size))
        else:
            # Not a perfect square - create interpolated grid
            #print(f"Patch {patch_id} has {num_points} points, creating interpolated {grid_size}x{grid_size} grid")
            
            # Find boundaries of the patch
            patch_row = patch_id // self.number_of_patches_height
            patch_col = patch_id % self.number_of_patches_height
            
            y_min = self.y_min + patch_row * self.patch_width
            y_max = y_min + self.patch_width
            z_min = self.z_min + patch_col * self.patch_height
            z_max = z_min + self.patch_height
            
            # Create regular grid
            y_grid_1d = np.linspace(y_min, y_max, grid_size)
            z_grid_1d = np.linspace(z_min, z_max, grid_size)
            Y_grid, Z_grid = np.meshgrid(y_grid_1d, z_grid_1d)
            
            # Interpolate X values using simple nearest neighbor
            X_grid = np.zeros_like(Y_grid)
            for i in range(grid_size):
                for j in range(grid_size):
                    # Find nearest point for each grid position
                    distances = np.sqrt((y_coords - Y_grid[i, j])**2 + (z_coords - Z_grid[i, j])**2)
                    nearest_idx = np.argmin(distances)
                    X_grid[i, j] = x_coords[nearest_idx]
        
        
        if plot_patch:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(X_grid, Y_grid, Z_grid, 
                    c='skyblue', marker='o', alpha=0.6)

            ax.set_xlabel('Asse X')
            ax.set_ylabel('Asse Y')
            ax.set_zlabel('Asse Z')
            ax.set_title(f'Punti of PatchID: {patch_id}')
            ax.set_aspect('auto')     
            plt.show()

        return X_grid, Y_grid, Z_grid
    
    def get_point_t_in_surface(self, patch_id, y_point, z_point, print_info=False, plot_patch=False):
        # Controllo se il punto (y,z) cade dentro la patch
        if not self.is_point_2D_in_patch(patch_id, y_point, z_point):
            print(f"[patch_surface] (y={y_point}, z={z_point}) is not in patch {patch_id}, cannot find surface point")
            return None

        points_in_patch = self.patches[patch_id]['points_in_patch']
        if len(points_in_patch) < 4:
            print(f"[patch_surface] Patch {patch_id} has insufficient points ({len(points_in_patch)}) to create mesh grid")
            return None

        # Ottieni la mesh della superficie
        X_grid, Y_grid, Z_grid = self.get_mesh_grid_patch(patch_id)

        # Punto target nel piano YZ (X è determinato dalla superficie)
        target_point = np.array([0.0, y_point, z_point])

        distances = np.sqrt(
            (Y_grid - y_point) ** 2 +
            (Z_grid - z_point) ** 2
        )

        # Trova l’indice del punto più vicino sulla superficie
        min_idx = np.unravel_index(np.argmin(distances), distances.shape)
        i, j = min_idx

        # Punto 3D sulla superficie
        closest_point = np.array([X_grid[i, j], Y_grid[i, j], Z_grid[i, j]])

        # Recupera il punto "vero" dai points_in_patch più vicino a (Y,Z)
        distances_pc = [
            (idx, np.sqrt((p['position'][1] - y_point) ** 2 + (p['position'][2] - z_point) ** 2))
            for idx, p in enumerate(points_in_patch)
        ]
        nearest_idx = min(distances_pc, key=lambda x: x[1])[0]
        nearest_point = points_in_patch[nearest_idx]

        point_t = {
            'position':     closest_point,
            'color':        nearest_point['color'],   # prendo colore del punto vicino
            'light':        nearest_point['light'],   # idem per luce
            'size_point':   nearest_point['size_point'],
            'cost':         nearest_point['cost'],    # costo del vicino
        }

        if print_info:
            print(f"[patch_surface] Requested (y={y_point}, z={z_point})")
            print(f"[patch_surface] Closest surface point: {closest_point}")
            print(f"[patch_surface] Used neighbor index: {nearest_idx}")
            print(f"[patch_surface] Final point_t: {point_t}")

        if plot_patch:
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot_surface(X_grid, Y_grid, Z_grid, alpha=0.6, cmap='viridis')

            ax.scatter(closest_point[0], closest_point[1], closest_point[2],
                    c='orange', s=100, marker='s', label='Surface Point')
            ax.scatter(nearest_point['position'][0], nearest_point['position'][1], nearest_point['position'][2],
                    c='blue', s=80, marker='o', label='Nearest Neighbor (for color/cost)')

            ax.set_xlabel('X (depth from wall)')
            ax.set_ylabel('Y (horizontal)')
            ax.set_zlabel('Z (height)')
            ax.set_title(f'Point_t in Surface - Patch {patch_id}')
            ax.legend(fontsize=8)
            plt.tight_layout()
            plt.show()

        return point_t

    def get_patch_id_from_point(self, point):
        for patch in self.patches:
            patch_id = patch['id']
            if self.is_point_in_patch(patch_id, point):
                return patch_id
        print("[patch_surface] Point does not belong to any patch")
        return None
    
    def get_patch_id_from_point_2D(self, y_point, z_point):
        for patch in self.patches:
            patch_id = patch['id']
            if self.is_point_2D_in_patch(patch_id, y_point, z_point):
                return patch_id
        print("[patch_surface] Point (y,z) does not belong to any patch")   
        return None

    def getAbsolutePoseOfPointInsidePatch (self, patch_id, point_local_y, point_local_z, scale=1.0):
        if patch_id < 0 or patch_id >= len(self.patches):
            print(f"[patch_surface] Invalid patch_id {patch_id}. Must be between 0 and {len(self.patches)-1}.")
            return None
    
        # Step 1: Validate input coordinates
        max_coord = scale
        if point_local_y < 0 or point_local_y > max_coord or point_local_z < 0 or point_local_z > max_coord:
            print(f"[patch_surface] Invalid local coordinates ({point_local_y}, {point_local_z}). Must be in range [0, {max_coord}].")
            return None
        
        # Step 2: Calculate patch boundaries using the same logic as create_patches()
        row_idx = patch_id // self.number_of_patches_width
        i = patch_id % self.number_of_patches_width
        j = (self.number_of_patches_height - 1) - row_idx
        
        # Calculate edges using the same method as create_patches()
        y_edges = self.y_min + np.arange(self.number_of_patches_width + 1) * self.patch_width
        z_edges = self.z_min + np.arange(self.number_of_patches_height + 1) * self.patch_height
        
        y_min_patch = y_edges[i]
        y_max_patch = y_edges[i + 1]
        z_min_patch = z_edges[j]
        z_max_patch = z_edges[j + 1]
        
        # Step 3: Convert relative coordinates to absolute coordinates
        actual_patch_width = y_max_patch - y_min_patch
        actual_patch_height = z_max_patch - z_min_patch
        
        y_absolute = y_min_patch + (point_local_y / scale) * actual_patch_width
        z_absolute = z_min_patch + (point_local_z / scale) * actual_patch_height
        
        # Step 4: Validate that the absolute coordinates are within patch boundaries
        if not (y_min_patch <= y_absolute <= y_max_patch and z_min_patch <= z_absolute <= z_max_patch):
            print(f"[patch_surface] Warning: Calculated absolute coordinates ({y_absolute:.3f}, {z_absolute:.3f}) are outside patch {patch_id} boundaries.")
            print(f"[patch_surface] Patch boundaries: Y[{y_min_patch:.3f}, {y_max_patch:.3f}], Z[{z_min_patch:.3f}, {z_max_patch:.3f}]")
        
        # we have already the function
        # # Step 5: Use get_point_t_in_surface to find the corresponding x value on the surface.
        # point_t = self.get_point_t_in_surface(patch_id, y_absolute, z_absolute)
        # if point_t is None:
        #     # If interpolation fails, use the patch's centroid X as a fallback.
        #     x_absolute = self.patches[patch_id]['centroid'][0]
        #     print(f"Warning: Could not interpolate X coordinate for absolute point ({y_absolute:.3f}, {z_absolute:.3f}). Using patch centroid X = {x_absolute:.3f}")
        # else:
        #     x_absolute = point_t['position'][0]
        #

        absolute_position = np.array([0.,  y_absolute, z_absolute])
        
        # Debugging information
        #print(f"Patch {patch_id}: relative point ({point_local_y}, {point_local_z}) with scale {scale} -> absolute {absolute_position}")
        #print(f"  Patch boundaries: Y[{y_min_patch:.3f}, {y_max_patch:.3f}], Z[{z_min_patch:.3f}, {z_max_patch:.3f}]")
    
        return absolute_position

    def get_points_in_patch(self, patch_id):
        if patch_id < 0 or patch_id >= len(self.patches):
            print(f"[patch_surface] Invalid patch_id {patch_id}. Must be between 0 and {len(self.patches)-1}.")
            return None
        return self.patches[patch_id].get('points_in_patch', [])
    
    
    def get_cost_meshgrid(self, grid_size):
        # Create regular grid in Y-Z plane
        z = np.linspace(self.z_min, self.z_max, grid_size)
        y = np.linspace(self.y_min, self.y_max, grid_size)
        Z_grid, Y_grid = np.meshgrid(z, y)
        
        y_points = []
        z_points = []
        cost_points = []
        
        for patch in self.patches:
            points_in_patch = patch.get('points_in_patch', [])
            for point in points_in_patch:
                pos = point['position']
                y_points.append(pos[1])
                z_points.append(pos[2])
                cost_points.append(point['cost'])
        y_points = np.array(y_points)
        z_points = np.array(z_points)
        cost_points = np.array(cost_points)
        
        points = np.column_stack((y_points, z_points))
        grid_points = np.column_stack((Y_grid.ravel(), Z_grid.ravel()))
        
        X_grid = griddata(points, cost_points, grid_points, method='linear', fill_value=np.nan)
        
        if np.any(np.isnan(X_grid)):
            X_grid_nearest = griddata(points, cost_points, grid_points, method='nearest')
            X_grid = np.where(np.isnan(X_grid), X_grid_nearest, X_grid)
        
        X_grid = X_grid.reshape(Z_grid.shape)
        
        return X_grid, Y_grid, Z_grid
    
    #  === SET METHODS ====
    def set_new_point_in_patch(self, patch_id, y_point, z_point, update_centroid=True, update_cost=True, plot=True, k_neighbors=5):
        if patch_id < 0 or patch_id >= len(self.patches):
            print(f"[patch_surface] Invalid patch_id {patch_id}. Must be between 0 and {len(self.patches)-1}.")
            print("[patch_surface] A")
            breakpoint()
            return None
        #check if the point (y,z) is in the patch
        if not self.is_point_2D_in_patch(patch_id, y_point, z_point):
            print(f"[patch_surface] (y={y_point}, z={z_point}) is not in patch {patch_id}, cannot add new point")
            print("[patch_surface] B")
            breakpoint()
            return None    
        patch = self.patches[patch_id]
        points_in_patch = patch.get('points_in_patch', [])
        if len(points_in_patch) < 4:
            print(f"[patch_surface] Patch {patch_id} has too few points ({len(points_in_patch)}) to create new point reliably")
            print("[patch_surface] C")
            breakpoint()
            return None

        X_grid, Y_grid, Z_grid = self.get_mesh_grid_patch(patch_id, plot_patch=False)

        # find point nearest to estimate X
        distances_yz = np.sqrt((Y_grid - y_point)**2 + (Z_grid - z_point)**2)
        min_idx = np.unravel_index(np.argmin(distances_yz), distances_yz.shape)
        i, j = min_idx
        x_estimated = X_grid[i, j]

        new_position = np.array([x_estimated, y_point, z_point])
        distances_pc = [
            (idx, np.sqrt((p['position'][1] - y_point) ** 2 + (p['position'][2] - z_point) ** 2))
            for idx, p in enumerate(points_in_patch)
        ]
        distances_pc.sort(key=lambda x: x[1])
        neighbors_idx = [idx for idx, _ in distances_pc[:k_neighbors]]
        neighbors = [points_in_patch[idx] for idx in neighbors_idx]

        # Media dei valori dai vicini
        color_avg = np.mean([n['color'] for n in neighbors], axis=0)
        light_avg = np.mean([n['light'] for n in neighbors], axis=0)
        cost_avg = float(np.mean([n['cost'] for n in neighbors]))
        size_avg = float(np.mean([n['size_point'] for n in neighbors]))

        new_point_t = {
            'position':   new_position,
            'color':      color_avg,
            'light':      light_avg,
            'size_point': size_avg,
            'cost':       cost_avg,
        }

        patch['points_in_patch'].append(new_point_t)

        if update_centroid:
            P = np.vstack([p['position'] for p in patch['points_in_patch']])
            patch['centroid'] = np.mean(P, axis=0)

        if update_cost:
            costs = [p['cost'] for p in patch['points_in_patch']]
            patch['cost_patch'] = float(np.mean(costs))

        print(f"[patch_surface] Added NEW interpolated point (y={y_point}, z={z_point}) to patch {patch_id}")
        print(f"[patch_surface] New centroid: {patch['centroid']}")
        print(f"[patch_surface] New average cost: {patch['cost_patch']:.4f}")

        if plot:
            self.plot_patch(patch_id)
    
    # ===PLOTTING AND COLORING METHODS===
    # ==== COLORING METHODS ====
    def random_color(self):
        num_patches = len(self.patches)
        colors = np.random.rand(num_patches, 3) 
        
        for i in range(num_patches):
            self.patches[i]['color_patch'] = colors[i]
            
    def cost_color(self):
        
        costs = np.array([
            patch.get('cost_patch', None) if patch.get('cost_patch') is not None else 0
            for patch in self.patches
        ], dtype=float)
        min_c, max_c = np.min(costs), np.max(costs)
        if max_c - min_c > 0:
            norm_costs = (costs - min_c) / (max_c - min_c)
        else:
            norm_costs = np.zeros_like(costs)  # tutti uguali → verde
        cmap = cm.get_cmap('RdYlGn_r')

        for i, patch in enumerate(self.patches):
            patch['color_patch'] = cmap(norm_costs[i])[:3]
              
    def color_targhet_points_jump(self, points_t = None):
        if points_t is None:
            print("[patch_surface] No points provided for plotting target points.")
            return
        # Create a set of target point positions for efficient lookup
        target_positions = set()
        for point in points_t:
            pos = tuple(point['position'])
            target_positions.add(pos)
        
        # Iterate through all patches and their points
        for patch in self.patches:
            points_in_patch = patch.get('points_in_patch', [])
            for point in points_in_patch:
                point_pos = tuple(point['position'])
                if point_pos in target_positions:
                    point['color'] = [0.0, 0.0, 1.0]  # Blue color
                    point['size_point'] = 20
        
        print(f"[patch_surface] Updated {len(points_t)} target points to blue color with size 2.")
    
    def color_targhet_patches(self, patches_t=None):
        if patches_t is None:
            print("[patch_surface] No patches provided for coloring target patches.")
            return
        
        # Create a set of target patch IDs for efficient lookup
        target_patch_ids = set(patch['id'] for patch in patches_t)
        
        # Iterate through all patches and color the target patches
        for patch in self.patches:
            if patch['id'] in target_patch_ids:
                patch['color_patch'] = [0.0, 0.0, 1.0]

                patch['size_patch'] = 20  # Increase size for target patches
    
    # ==== PLOTTING METHODS ====                 
    def plot_patches(self):
        alpha = 0.5
        print("[patch_surface] Plotting patches...")
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title('Patches 3D')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        total_points = sum(len(patch.get('points_in_patch', [])) for patch in self.patches)
        num_patches = len(self.patches)
        
        # Automatically switch to fast mode if too many points or patches
        use_fast_mode = total_points > 1000001
        
        # Collect all points for equal axis scaling
        all_x, all_y, all_z = [], [], []
        
        if use_fast_mode:
            print(f"[patch_surface] Using fast mode: {total_points} total points, {num_patches} patches")
            # Fast mode: show only centroids
            centroids = []
            colors = []
            for patch in self.patches:
                centroid = patch.get('centroid')
                if centroid is not None:
                    centroids.append(centroid)
                    colors.append(patch.get('color_patch', [0.5, 0.5, 0.5]))
            
            if centroids:
                centroids = np.array(centroids)
                ax.scatter(centroids[:, 0], centroids[:, 1], centroids[:, 2], 
                          s=24, c=colors, alpha=0.8, marker='s')
                all_x = centroids[:, 0]
                all_y = centroids[:, 1]
                all_z = centroids[:, 2]
        else: 
            for patch in self.patches:
                pts = patch.get('points_in_patch') or patch.get('points') or []
                if not pts:
                    continue

                P = np.vstack([p['position'] for p in pts])  
                s = np.array([p['size_point'] for p in pts])
                all_x.extend(P[:, 0])
                all_y.extend(P[:, 1])
                all_z.extend(P[:, 2])
            
                color = patch.get('color_patch')
                ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=s, alpha=alpha, color=color)
        
        # Equal axis scaling
        if all_x:
            all_x, all_y, all_z = np.array(all_x), np.array(all_y), np.array(all_z)
            max_range = np.array([
                all_x.max() - all_x.min(),
                all_y.max() - all_y.min(),
                all_z.max() - all_z.min()
            ]).max() / 2.0
            
            mid_x = (all_x.max() + all_x.min()) * 0.5
            mid_y = (all_y.max() + all_y.min()) * 0.5
            mid_z = (all_z.max() + all_z.min()) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

        plt.tight_layout()
        plt.show()
        return fig, ax
    
    def plot_patches_by_id(self, target_ids):
        if isinstance(target_ids, int):
            target_ids = [target_ids]
            
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(f'Map Context - Highlighted Patches: {target_ids}')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Colors
        colore_evidenziato = 'purple'
        colore_sfondo = 'lightgrey' # Light gray for the rest of the map

        # Collect all points for equal axis scaling
        all_x, all_y, all_z = [], [], []

        for patch in self.patches:
            pts = patch.get('points_in_patch', [])
            if not pts:
                continue

            # Extract coordinates
            P = np.vstack([p['position'] for p in pts])
            s = np.array([p['size_point'] for p in pts]) if (len(pts) > 0 and 'size_point' in pts[0]) else 10
            
            all_x.extend(P[:, 0])
            all_y.extend(P[:, 1])
            all_z.extend(P[:, 2])
            
            # Decide color and transparency based on ID
            if patch['id'] in target_ids:
                # Selected patch: PURPLE and OPAQUE
                current_color = colore_evidenziato
                current_alpha = 0.8
                zorder = 5 # Bring to foreground
                
                # Add centroid and text only for these
                centroid = patch.get('centroid')
            else:
                # Rest of the map: GRAY and SEMI-TRANSPARENT
                current_color = colore_sfondo
                current_alpha = 0.3 # Very light to not disturb the view
                zorder = 1

            # Draw the patch
            ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=s, alpha=current_alpha, color=current_color, zorder=zorder)

        # Equal axis scaling
        if all_x:
            all_x, all_y, all_z = np.array(all_x), np.array(all_y), np.array(all_z)
            max_range = np.array([
                all_x.max() - all_x.min(),
                all_y.max() - all_y.min(),
                all_z.max() - all_z.min()
            ]).max() / 2.0
            
            mid_x = (all_x.max() + all_x.min()) * 0.5
            mid_y = (all_y.max() + all_y.min()) * 0.5
            mid_z = (all_z.max() + all_z.min()) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Optimization visualization
        plt.tight_layout()
        plt.show()
        return fig, ax
    
    def plot_patches_points_target(self):
        
        alpha = 0.5
        print("[patch_surface] Plotting patches...")
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title('Patches 3D')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        total_points = sum(len(patch.get('points_in_patch', [])) for patch in self.patches)
        num_patches = len(self.patches)
        
        # Automatically switch to fast mode if too many points or patches
        use_fast_mode = total_points > 1000001
        
        # Collect all points for equal axis scaling
        all_x, all_y, all_z = [], [], []
        
        if use_fast_mode:
            print(f"[patch_surface] Using fast mode: {total_points} total points, {num_patches} patches")
            # Fast mode: show only centroids
            centroids = []
            colors = []
            for patch in self.patches:
                centroid = patch.get('centroid')
                if centroid is not None:
                    centroids.append(centroid)
                    colors.append(patch.get('color_patch', [0.5, 0.5, 0.5]))
            
            if centroids:
                centroids = np.array(centroids)
                ax.scatter(centroids[:, 0], centroids[:, 1], centroids[:, 2], 
                          s=24, c=colors, alpha=0.8, marker='s')
                all_x = centroids[:, 0]
                all_y = centroids[:, 1]
                all_z = centroids[:, 2]
        else: 
            for patch in self.patches:
                pts = patch.get('points_in_patch') or patch.get('points') or []
                if not pts:
                    continue

                P = np.vstack([p['position'] for p in pts])  
                s = np.array([p['size_point'] for p in pts])
                
                all_x.extend(P[:, 0])
                all_y.extend(P[:, 1])
                all_z.extend(P[:, 2])
                
                # Check for blue points and preserve their color
                colors = []
                for p in pts:
                    if 'color' in p and np.allclose(p['color'], [0.0, 0.0, 1.0]):
                        colors.append(p['color'])  # Keep blue color
                    else:
                        colors.append(patch.get('color_patch', [0.5, 0.5, 0.5]))  # Use patch color
                
                ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=s, alpha=alpha, c=colors)
                
                # centroid = patch.get('centroid')
                # if centroid is not None:
                #     ax.scatter(centroid[0], centroid[1], centroid[2], 
                #             s=20, c='blue', marker='o', alpha=1.0, edgecolors='white', linewidth=1)

        # Equal axis scaling
        if all_x:
            all_x, all_y, all_z = np.array(all_x), np.array(all_y), np.array(all_z)
            max_range = np.array([
                all_x.max() - all_x.min(),
                all_y.max() - all_y.min(),
                all_z.max() - all_z.min()
            ]).max() / 2.0
            
            mid_x = (all_x.max() + all_x.min()) * 0.5
            mid_y = (all_y.max() + all_y.min()) * 0.5
            mid_z = (all_z.max() + all_z.min()) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

        plt.tight_layout()
        plt.show()
        return fig, ax
        
    def plot_patches_target(self):
        fig, ax = self.plot_patches()
        
        # Plot target points in blue
        for patch in self.patches:
            points_in_patch = patch.get('points_in_patch', [])
            for point in points_in_patch:
                if 'color' in point and np.allclose(point['color'], [0.0, 0.0, 1.0]):
                    ax.scatter(point['position'][0], point['position'][1], point['position'][2], 
                               s=20, c='blue', marker='o', alpha=1.0, edgecolors='white', linewidth=1)
        
        plt.show()
        return fig, ax
    
    def plot_patch(self, patch_id):
        if patch_id < 0 or patch_id >= len(self.patches):
            print(f"[patch_surface] Invalid patch_id {patch_id}. Must be between 0 and {len(self.patches)-1}.")
            return None
        
        patch = self.patches[patch_id]
        points_in_patch = patch.get('points_in_patch', [])
        
        if not points_in_patch:
            print(f"[patch_surface] Patch {patch_id} has no points to plot.")
            return None
        
        P = np.vstack([p['position'] for p in points_in_patch])  
        s = np.array([p['size_point'] for p in points_in_patch])
        color = patch.get('color_patch')
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(f'Patch select: {patch_id} 3D')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=s, alpha=0.8, color=color)
        
        centroid = patch.get('centroid')
        if centroid is not None:
            ax.scatter(centroid[0], centroid[1], centroid[2], 
                       s=50, c='red', marker='o', alpha=1.0, edgecolors='white', linewidth=1)
        
        # Equal axis scaling
        max_range = np.ptp(P, axis=0).max() / 2.0
        mid = np.mean(P, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
        
        plt.tight_layout()
        plt.show()
    
    def visualize_full_cost_map(self):
        # 1. Collection of all points from all patches
        all_points_list = []
        for patch in self.patches:
            all_points_list.extend(patch.get('points_in_patch', []))

        if not all_points_list:
            print("[patch_surface] No points found in patches for visualization.")
            return

        # 2. Extract data for plotting
        x_coords = np.array([p['position'][0] for p in all_points_list])
        y_coords = np.array([p['position'][1] for p in all_points_list])
        z_coords = np.array([p['position'][2] for p in all_points_list])
        colors = np.array([p['color'] for p in all_points_list])
        costs = np.array([p['cost'] for p in all_points_list])
        sizes = np.array([p['size_point'] for p in all_points_list])

        # 3. Create plot (replica of PointCloudFilter subplot)
        fig = plt.figure(figsize=(16, 8))
        
        # Subplot 1: 3D point cloud colored by cost
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        ax1.scatter(x_coords, y_coords, z_coords, c=colors, s=sizes, alpha=0.8)
        
        # Equal axis scaling for 3D plot
        max_range = np.array([
            x_coords.max() - x_coords.min(),
            y_coords.max() - y_coords.min(),
            z_coords.max() - z_coords.min()
        ]).max() / 2.0
        
        mid_x = (x_coords.max() + x_coords.min()) * 0.5
        mid_y = (y_coords.max() + y_coords.min()) * 0.5
        mid_z = (z_coords.max() + z_coords.min()) * 0.5
        
        ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        ax1.set_zlim(mid_z - max_range, mid_z + max_range)
        
        ax1.set_xlabel('X (m) - Height')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Map - Color based on Cost\n(Green=Low, Red=High)')
        
        # Subplot 2: 2D top view (YZ Plane)
        ax2 = fig.add_subplot(1, 2, 2)
        scatter2 = ax2.scatter(y_coords, z_coords, c=colors, s=sizes*3, alpha=1.0) 
        ax2.set_xlabel('Y (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('2D Cost Map View\n(YZ Projection)')
        ax2.set_aspect('equal', adjustable='box')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        
        # 4. Cost statistics (replica of PointCloudFilter logic)
        print(f"\n[patch_surface] Global Cost Statistics (on {len(all_points_list)} points):")
        print(f"  - Minimum Cost: {np.min(costs):.3f}")
        print(f"  - Maximum Cost: {np.max(costs):.3f}")
        print(f"  - Mean Cost: {np.mean(costs):.3f}")
        print(f"  - Standard Deviation: {np.std(costs):.3f}")
        
        plt.show()
        
        
      
    def plot_population_density(self, all_sampled_ids):
        """
        Visualizza quali patch sono state scelte, colorandole in base alla FREQUENZA di selezione.
        Più una patch è stata scelta, più il colore sarà intenso/caldo.
        
        Args:
            all_sampled_ids: array o lista contenente TUTTI gli ID campionati (NON usare np.unique prima!)
        """
        from collections import Counter
        import matplotlib.cm as cm
        import matplotlib.colors as mcolors

        # 1. Conta quante volte ogni patch è stata scelta
        counts = Counter(all_sampled_ids)
        if not counts:
            print("[patch_surface] Nessun ID passato per il plot density.")
            return

        max_count = max(counts.values())
        
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(f'Population Discrete Distribution')
        ax.set_xlabel('X (Depth)')
        ax.set_ylabel('Y (Width)')
        ax.set_zlabel('Z (Height)')

        # Use a colormap that goes from transparent/cold to hot/opaque
        # Example: 'plasma' or 'hot_r' or 'Reds'
        cmap = cm.get_cmap('jet') 

        print(f"[PLOT] Generating Selection Heatmap...")

        # Collect all points for equal axis scaling
        all_x, all_y, all_z = [], [], []

        for patch in self.patches:
            pid = patch['id']
            pts = patch.get('points_in_patch', [])
            if not pts:
                continue

            P = np.vstack([p['position'] for p in pts])
            all_x.extend(P[:, 0])
            all_y.extend(P[:, 1])
            all_z.extend(P[:, 2])
            
            # Color logic:
            if pid in counts:
                frequency = counts[pid]
                # Normalize between 0 and 1 relative to the maximum found in this iteration
                intensity = frequency / max_count 
                
                # Get color from colormap
                color_val = cmap(intensity)
                
                # VISUAL TRICK: 
                # Highly selected patches should be opaque (alpha=1).
                # Less selected patches should be transparent.
                # Unselected patches: very light gray almost invisible.
                alpha_val = 0.3 + (0.7 * intensity) # Minimum 0.3 opacity if selected
                s_val = 15 + (20 * intensity)       # Point size also increases if heavily trafficked
                
            else:
                # NEVER selected patch
                color_val = 'lightgrey'
                alpha_val = 0.02 # Practically invisible, just context
                s_val = 5

            ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=s_val, c=[color_val], alpha=alpha_val)

        # Equal axis scaling
        if all_x:
            all_x, all_y, all_z = np.array(all_x), np.array(all_y), np.array(all_z)
            max_range = np.array([
                all_x.max() - all_x.min(),
                all_y.max() - all_y.min(),
                all_z.max() - all_z.min()
            ]).max() / 2.0
            
            mid_x = (all_x.max() + all_x.min()) * 0.5
            mid_y = (all_y.max() + all_y.min()) * 0.5
            mid_z = (all_z.max() + all_z.min()) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Add a "fake" colorbar to understand the scale (optional but useful)
        m = cm.ScalarMappable(cmap=cmap)
        m.set_array([0, max_count])
        plt.colorbar(m, ax=ax, label='Number of Selections (Frequency)')

        plt.tight_layout()
        plt.show()
        
    def plot_patches_2D_with_ids(self, figsize=(14, 10), show_ids=True, fontsize=8):
        """
        Visualize patches in 2D (YZ plane) with their IDs and colors based on average cost.
        
        Args:
            figsize: tuple, figure size (width, height)
            show_ids: bool, whether to display patch IDs
            fontsize: int, font size for patch ID labels
        """
        print("[patch_surface] Plotting 2D patches with IDs and cost coloring...")
        
        # Collect all patch costs for normalization
        costs = np.array([
            patch.get('cost_patch', 0.0) if patch.get('cost_patch') is not None else 0.0
            for patch in self.patches
        ], dtype=float)
        
        min_cost, max_cost = np.min(costs), np.max(costs)
        
        # Normalize costs
        if max_cost - min_cost > 0:
            norm_costs = (costs - min_cost) / (max_cost - min_cost)
        else:
            norm_costs = np.zeros_like(costs)
        
        # Use colormap (red-yellow-green reversed: high cost = red, low cost = green)
        cmap = cm.get_cmap('RdYlGn_r')
        
        fig, ax = plt.subplots(figsize=figsize)
        ax.set_title('2D Patch Map - Colored by Average Cost', fontsize=14, fontweight='bold')
        ax.set_xlabel('Y (m) - Width', fontsize=12)
        ax.set_ylabel('Z (m) - Height', fontsize=12)
        ax.grid(True, alpha=0.3, linestyle='--')
        
        # Calculate patch edges for rectangles
        y_edges = self.y_min + np.arange(self.number_of_patches_width + 1) * self.patch_width
        z_edges = self.z_min + np.arange(self.number_of_patches_height + 1) * self.patch_height
        
        # Plot each patch as a rectangle
        for patch in self.patches:
            patch_id = patch['id']
            
            # Calculate patch boundaries
            i = patch_id // self.number_of_patches_height
            j = patch_id % self.number_of_patches_height
            
            y_min = y_edges[i]
            y_max = y_edges[i + 1]
            z_min = z_edges[j]
            z_max = z_edges[j + 1]
            
            # Get color based on normalized cost
            color = cmap(norm_costs[patch_id])
            
            # Draw rectangle for patch
            from matplotlib.patches import Rectangle
            rect = Rectangle((y_min, z_min), 
                            y_max - y_min, 
                            z_max - z_min,
                            facecolor=color[:3], 
                            edgecolor='black', 
                            linewidth=0.5,
                            alpha=0.7)
            ax.add_patch(rect)
            
            # Add patch ID text at centroid
            if show_ids:
                centroid = patch.get('centroid')
                if centroid is not None:
                    y_center = centroid[1]
                    z_center = centroid[2]
                    
                    # Choose text color based on background brightness
                    brightness = 0.299 * color[0] + 0.587 * color[1] + 0.114 * color[2]
                    text_color = 'white' if brightness < 0.5 else 'black'
                    
                    ax.text(y_center, z_center, str(patch_id),
                           ha='center', va='center',
                           fontsize=fontsize, fontweight='bold',
                           color=text_color)
        
        # Set axis limits
        ax.set_xlim(self.y_min - 0.1, self.y_max + 0.1)
        ax.set_ylim(self.z_min - 0.1, self.z_max + 0.1)
        ax.set_aspect('equal', adjustable='box')
        
        # Add colorbar
        sm = cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=min_cost, vmax=max_cost))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Average Patch Cost', fontsize=11, fontweight='bold')
        
        # Add statistics text
        stats_text = f'Total Patches: {len(self.patches)}\n'
        stats_text += f'Cost Range: [{min_cost:.3f}, {max_cost:.3f}]\n'
        stats_text += f'Mean Cost: {np.mean(costs):.3f}'
        
        ax.text(0.02, 0.98, stats_text,
               transform=ax.transAxes,
               fontsize=9,
               verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        plt.show()
        
        print(f"[patch_surface] 2D plot generated with {len(self.patches)} patches")
        print(f"[patch_surface] Cost range: [{min_cost:.3f}, {max_cost:.3f}]")
        
        return fig, ax
    
    def print_patch_cost_matrix(self, patch_id, grid_size=None, visualize=True):
            if patch_id < 0 or patch_id >= len(self.patches):
                print(f"[patch_surface] Errore: patch_id {patch_id} non valido.")
                return

            points_in_patch = self.patches[patch_id]['points_in_patch']
            if not points_in_patch:
                print(f"[patch_surface] La patch {patch_id} è vuota.")
                return

            y_coords = np.array([p['position'][1] for p in points_in_patch])
            z_coords = np.array([p['position'][2] for p in points_in_patch])
            costs = np.array([p['cost'] for p in points_in_patch])

            if grid_size is None:
                # Stima una dimensione quadrata basata sul numero di punti (es. 100 punti -> 10x10)
                grid_size = int(np.sqrt(len(points_in_patch)))
                if grid_size < 2: grid_size = 5 # Fallback minimo

            y_grid = np.linspace(y_coords.min(), y_coords.max(), grid_size)
            z_grid = np.linspace(z_coords.min(), z_coords.max(), grid_size)
            Y, Z = np.meshgrid(y_grid, z_grid)

            points_yz = np.column_stack((y_coords, z_coords))
            try:
                cost_matrix = griddata(points_yz, costs, (Y, Z), method='nearest')
            except Exception as e:
                print(f"[patch_surface] Errore durante l'interpolazione: {e}")
                return

            matrix_visual = np.flipud(cost_matrix)
            
            print(f"\n[patch_surface] Cost Matrix for Patch {patch_id} (Resolution: {grid_size}x{grid_size})")
            print("-" * 60)
            with np.printoptions(precision=2, suppress=True, linewidth=120, floatmode='fixed'):
                print(matrix_visual)
            print("-" * 60)
            print(f"Min Cost: {np.min(costs):.4f} | Max Cost: {np.max(costs):.4f} | Avg: {np.mean(costs):.4f}")

            if visualize:
                fig, ax = plt.subplots(figsize=(8, 6))
                
                extent = [y_coords.min(), y_coords.max(), z_coords.min(), z_coords.max()]
                
                im = ax.imshow(matrix_visual, extent=extent, cmap='RdYlGn_r', aspect='auto', interpolation='nearest')
                plt.colorbar(im, label='Cost')
                
                ax.set_title(f'Cost Heatmap - Patch {patch_id}')
                ax.set_xlabel('Y (Width)')
                ax.set_ylabel('Z (Height)')
                ax.scatter(y_coords, z_coords, c='black', s=10, alpha=0.3, label='Real Points')
                ax.legend()
                
                plt.tight_layout()
                plt.show()

            return matrix_visual

    def plot_cost_meshgrid(self, Cost_grid, Y_grid, Z_grid, plot_type='surface', show_colorbar=True):
        
        if plot_type in ['surface', 'both']:
            fig = plt.figure(figsize=(12, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # Surface plot with cost as height
            surf = ax.plot_surface(Y_grid, Z_grid, Cost_grid, 
                                cmap='RdYlGn_r', 
                                alpha=0.8,
                                edgecolor='none')
            
            ax.set_xlabel('Y (horizontal)')
            ax.set_ylabel('Z (height)')
            ax.set_zlabel('Cost')
            ax.set_title('Cost Morphology - 3D Surface')
            
            if show_colorbar:
                fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5, label='Cost Value')
            
            plt.tight_layout()
            plt.show()
        
        if plot_type in ['contour', 'both']:
            fig, ax = plt.subplots(figsize=(10, 8))
            
            # Contour plot (top view)
            contour = ax.contourf(Y_grid, Z_grid, Cost_grid, 
                                levels=20, 
                                cmap='RdYlGn_r')
            
            # Add contour lines
            contour_lines = ax.contour(Y_grid, Z_grid, Cost_grid, 
                                    levels=10, 
                                    colors='black', 
                                    linewidths=0.5,
                                    alpha=0.3)
            
            ax.set_xlabel('Y (horizontal)')
            ax.set_ylabel('Z (height)')
            ax.set_title('Cost Morphology - Contour Map')
            ax.set_aspect('equal')
            
            if show_colorbar:
                cbar = fig.colorbar(contour, ax=ax, label='Cost Value')
                ax.clabel(contour_lines, inline=True, fontsize=8)
            
            plt.tight_layout()
            plt.show()
        
        # Print statistics
        print("\n" + "="*50)
        print(" Cost Meshgrid Statistics:")
        print(f" Min Cost: {np.nanmin(Cost_grid):.4f}")
        print(f" Max Cost: {np.nanmax(Cost_grid):.4f}")
        print(f" Mean Cost: {np.nanmean(Cost_grid):.4f}")
        print(f" Std Dev: {np.nanstd(Cost_grid):.4f}")
        print("="*50 + "\n")
    
    
    def plot_map_with_cost_meshgrid_overlay(self, Cost_grid, Y_grid, Z_grid, x_offset=0.5, alpha_mesh=0.6, alpha_points=0.3):
        """
        Plot the real terrain map with the cost meshgrid overlaid with an X offset.
        
        Args:
            grid_size (int): Resolution of the cost meshgrid
            x_offset (float): Offset in X direction for the meshgrid layer
            alpha_mesh (float): Transparency of the meshgrid surface (0-1)
            alpha_points (float): Transparency of the point cloud (0-1)
        """
        print(f"[patch_surface] Generating map with cost meshgrid overlay (X offset = {x_offset} m)...")
        
        
        all_x, all_y, all_z = [], [], []
        all_colors = []
        all_sizes = []
        
        for patch in self.patches:
            points_in_patch = patch.get('points_in_patch', [])
            for point in points_in_patch:
                pos = point['position']
                all_x.append(pos[0])
                all_y.append(pos[1])
                all_z.append(pos[2])
                all_colors.append(point.get('color', [0.5, 0.5, 0.5]))
                all_sizes.append(point.get('size_point', 1.0))
        
        all_x = np.array(all_x)
        all_y = np.array(all_y)
        all_z = np.array(all_z)
        all_colors = np.array(all_colors)
        all_sizes = np.array(all_sizes)
        
        from scipy.interpolate import griddata
        points_yz = np.column_stack((all_y, all_z))
        grid_points_yz = np.column_stack((Y_grid.ravel(), Z_grid.ravel()))
        
        X_terrain_grid = griddata(points_yz, all_x, grid_points_yz, method='linear', fill_value=np.nan)
        
        if np.any(np.isnan(X_terrain_grid)):
            X_terrain_nearest = griddata(points_yz, all_x, grid_points_yz, method='nearest')
            X_terrain_grid = np.where(np.isnan(X_terrain_grid), X_terrain_nearest, X_terrain_grid)
        
        X_terrain_grid = X_terrain_grid.reshape(Y_grid.shape)
        
        # Create the figure
        fig = plt.figure(figsize=(16, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot 1: Real terrain surface (semi-transparent)
        # Uses viridis colormap for depth visualization
        surf_terrain = ax.plot_surface(X_terrain_grid, Y_grid, Z_grid,
                                    cmap='viridis',
                                    alpha=0.4,
                                    edgecolor='none',
                                    label='Real Terrain')
        
        # Plot 2: Point cloud (very transparent)
        scatter_points = ax.scatter(all_x, all_y, all_z,
                c=all_colors,
                s=all_sizes * 0.5,
                alpha=alpha_points,
                label='Point Cloud')
        
        # Plot 3: Cost meshgrid with X offset
        X_cost_offset = X_terrain_grid + x_offset
        
        # Normalize costs for colormap
        cost_normalized = (Cost_grid - np.nanmin(Cost_grid)) / (np.nanmax(Cost_grid) - np.nanmin(Cost_grid))
        
        surf_cost = ax.plot_surface(X_cost_offset, Y_grid, Z_grid,
                                    facecolors=cm.RdYlGn_r(cost_normalized),
                                    alpha=alpha_mesh,
                                    edgecolor='black',
                                    linewidth=0.1,
                                    label='Cost Meshgrid')
        
        # Formatting
        ax.set_xlabel('X (m) - Depth', fontsize=12)
        ax.set_ylabel('Y (m) - Width', fontsize=12)
        ax.set_zlabel('Z (m) - Height', fontsize=12)
        ax.set_title(f'Terrain Map with Cost Meshgrid Overlay\n(Cost mesh offset: +{x_offset}m in X)', 
                    fontsize=14, fontweight='bold')
        
        # Equal axis scaling
        max_range = np.array([
            all_x.max() - all_x.min() + x_offset,
            all_y.max() - all_y.min(),
            all_z.max() - all_z.min()
        ]).max() / 2.0
        
        mid_x = (all_x.max() + all_x.min()) * 0.5 + x_offset/2
        mid_y = (all_y.max() + all_y.min()) * 0.5
        mid_z = (all_z.max() + all_z.min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        # Add colorbar for cost
        m = cm.ScalarMappable(cmap=cm.RdYlGn_r)
        m.set_array(Cost_grid)
        cbar = plt.colorbar(m, ax=ax, shrink=0.5, aspect=10, pad=0.1)
        cbar.set_label('Cost Value\n(Green=Low, Red=High)', fontsize=10)
        
        # --- INTERACTIVE LEGEND SECTION ---
        from matplotlib.patches import Patch
        
        # Dizionario per mappare elementi della leggenda ai plot
        legend_map = {}
        
        # Crea elementi della leggenda più descrittivi
        legend_elements = [
            Patch(facecolor='teal', alpha=0.4, edgecolor='black', linewidth=1, 
                  label='[TERRAIN] Superficie Terreno Reale'),
            Patch(facecolor='red', alpha=alpha_mesh, edgecolor='black', linewidth=1, 
                  label=f'[COST] Griglia Costi (offset +{x_offset}m in X)'),
            Patch(facecolor='gray', alpha=alpha_points, 
                  label='[POINTS] Nuvola di Punti')
        ]
        
        # Mappa gli elementi ai plot effettivi
        legend_map[legend_elements[0]] = surf_terrain
        legend_map[legend_elements[1]] = surf_cost
        legend_map[legend_elements[2]] = scatter_points
        
        # Crea la leggenda con picker abilitato
        leg = ax.legend(handles=legend_elements, loc='upper left', fontsize=11, 
                      framealpha=0.9, edgecolor='black', 
                      title='Click per mostrare/nascondere',
                      title_fontsize=10, fancybox=True, shadow=True)
        
        # Abilita il picking per tutti gli elementi della leggenda
        for legend_line, legend_text in zip(leg.get_patches(), leg.get_texts()):
            legend_line.set_picker(True)
            legend_text.set_picker(True)
        
        # Funzione di callback per gestire i click sulla leggenda
        def on_pick(event):
            # Trova quale elemento della leggenda è stato cliccato
            for i, (legend_patch, legend_text) in enumerate(zip(leg.get_patches(), leg.get_texts())):
                if event.artist in [legend_patch, legend_text]:
                    # Toggle della visibilità
                    plot_obj = legend_map[legend_elements[i]]
                    visible = not plot_obj.get_visible()
                    plot_obj.set_visible(visible)
                    
                    # Aggiorna trasparenza della leggenda per dare feedback visivo
                    if visible:
                        legend_patch.set_alpha(legend_elements[i].get_alpha())
                        legend_text.set_alpha(1.0)
                    else:
                        legend_patch.set_alpha(0.2)
                        legend_text.set_alpha(0.3)
                    
                    fig.canvas.draw()
                    break
        
        # Connetti l'evento di picking
        fig.canvas.mpl_connect('pick_event', on_pick)
        # ----------------------------
        
        # Add grid
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        print(f"[patch_surface] Plot complete!")
        print(f"  - Real terrain: X range [{all_x.min():.2f}, {all_x.max():.2f}] m")
        print(f"  - Cost mesh: X range [{(all_x.min()+x_offset):.2f}, {(all_x.max()+x_offset):.2f}] m")
        print(f"  - Cost range: [{np.nanmin(Cost_grid):.3f}, {np.nanmax(Cost_grid):.3f}]")
        
        
def main():
    print("[TEST] STARTING PATCH SURFACE TEST SUITE")
    
    print("[TEST] === SETUP: Loading Terrain and Filtering Point Cloud ===")
    terrain  = TerrainManager(grid_size=100,wall_depth =10,max_ridge_depth=0.5, seed="default", Lz=-10, Ly=10, generate_terrain=True, terrain_type="custom_gaussians")
    pc = terrain.point_cloud
    pcs = PointCloudFilter(pc, h_min=1.0, h_max=4.0)    
    pcs.print_map_pc()
    # filter
    # print("\n[TEST] Applying Exponential Height Cost Filter...")
    # pcs.filter_height_profile(x0=1.5, scale=0.5, profile="exponential")
    # print("\n[TEST] Applying Smoothing Filter...")
    # kernel = [pcs.smoothing_kernel] 
    # pcs.filter_process_points_pipeline(kernel, weight=0.5, plot=False)
    print("\n[TEST] Filtered Map Statistics:")
    pcs.print_map_pc()

    # Create PatchSurface object
    print("\n[TEST] === Creating Patch Surface ===")
    patch_surface = PatchSurface(pcs.points_t, number_of_patches_width=10, number_of_patches_height=10)
    
    patch_surface.random_color()
    patch_surface.plot_patches()
    # Apply cost-based coloring
    patch_surface.cost_color()
    
    # TEST 1: Get basic information
    print("\n[TEST] === TEST 1: Getting Basic Patch Information ===")
    num_patches = patch_surface.get_number_of_patches()
    print(f"[TEST] Total number of patches: {num_patches}")
    
    test_patch_id = 25
    centroid = patch_surface.get_patch_centroid(test_patch_id)
    print(f"[TEST] Centroid of patch {test_patch_id}: {centroid}")
    
    cost = patch_surface.get_patch_cost(test_patch_id)
    print(f"[TEST] Cost of patch {test_patch_id}: {cost:.4f}")
    
    points_in_patch = patch_surface.get_points_in_patch(test_patch_id)
    print(f"[TEST] Number of points in patch {test_patch_id}: {len(points_in_patch)}")
    
    # TEST 2: Test point membership in patch
    print("\n[TEST] === TEST 2: Testing Point Membership ===")
    test_point = {
        'position': np.array([5.0, 2.0, 0.0]),
        'color': np.array([0, 0, 0]),
        'light': 0.8,
        'size_point': 4.0,
        'cost': 0.5
    }
    
    patch_id_from_point = patch_surface.get_patch_id_from_point(test_point)
    print(f"[TEST] Test point belongs to patch: {patch_id_from_point}")
    
    # TEST 3: Test 2D point location
    print("\n[TEST] === TEST 3: Testing 2D Point Location ===")
    y_test = 2.5
    z_test = -1.5
    patch_id_2d = patch_surface.get_patch_id_from_point_2D(y_test, z_test)
    print(f"[TEST] Point (y={y_test}, z={z_test}) belongs to patch: {patch_id_2d}")
    
    # TEST 4: Get point on surface
    print("\n[TEST] === TEST 4: Getting Point on Surface ===")
    if patch_id_2d is not None:
        point_on_surface = patch_surface.get_point_t_in_surface(patch_id_2d, y_test, z_test, print_info=True, plot_patch=False)
        if point_on_surface:
            print(f"[TEST] Surface point found at: {point_on_surface['position']}")
            print(f"[TEST] Point cost: {point_on_surface['cost']:.4f}")
    
    # TEST 5: Test absolute position conversion
    print("\n[TEST] === TEST 5: Converting Local to Absolute Coordinates ===")
    test_patch_id = 1
    local_y = 0.5
    local_z = 0.5
    scale = 1.0
    absolute_position = patch_surface.getAbsolutePoseOfPointInsidePatch(test_patch_id, local_y, local_z, scale=scale)
    print(f"[TEST] Local coords ({local_y}, {local_z}) in patch {test_patch_id} -> Absolute: {absolute_position}")
    
    # TEST 6: Get cost at specific point
    print("\n[TEST] === TEST 6: Getting Cost at Specific Point ===")
    if absolute_position is not None:
        abs_yz = absolute_position[1:]  # Extract Y and Z
        cost_at_point = patch_surface.get_cost_in_point(test_patch_id, abs_yz)
        print(f"[TEST] Cost at point {abs_yz}: {cost_at_point:.4f}")
    
    # TEST 7: Test normal vector calculation
    print("\n[TEST] === TEST 7: Calculating Surface Normal Vector ===")
    if patch_id_from_point is not None:
        normal_vector = patch_surface.normal_vector_of_point_in_patch(
            patch_id_from_point, test_point, print_info=True, plot_normal_patch=False)
        if normal_vector is not None:
            print(f"[TEST] Normal vector: {normal_vector}")
    
    # TEST 8: Add new interpolated point
    print("\n[TEST] === TEST 8: Adding New Interpolated Point ===")
    new_y = 3.0
    new_z = -2.0
    patch_for_new_point = patch_surface.get_patch_id_from_point_2D(new_y, new_z)
    if patch_for_new_point is not None:
        print(f"[TEST] Adding new point at (y={new_y}, z={new_z}) to patch {patch_for_new_point}")
        patch_surface.set_new_point_in_patch(patch_for_new_point, new_y, new_z, 
                                            update_centroid=True, update_cost=True, 
                                            plot=False, k_neighbors=5)
    
    # TEST 9: Test color marking for target points
    print("\n[TEST] === TEST 9: Marking Target Points ===")
    random_indices = np.random.choice(len(pcs.points_t), size=5, replace=False)
    target_points = [pcs.points_t[i] for i in random_indices]
    print(f"[TEST] Selected {len(target_points)} random target points")
    patch_surface.color_targhet_points_jump(target_points)
    
    # TEST 10: Test color marking for target patches
    print("\n[TEST] === TEST 10: Marking Target Patches ===")
    random_patch_indices = np.random.choice(len(patch_surface.patches), size=3, replace=False)
    target_patches = [patch_surface.patches[i] for i in random_patch_indices]
    print(f"[TEST] Selected {len(target_patches)} random target patches: {[p['id'] for p in target_patches]}")
    patch_surface.color_targhet_patches(target_patches)
    
    
    # TEST 11: add gaussian cost to a patch
    print("\n[TEST] === TEST 10b: Adding Gaussian Cost to a Patch ===")
    patch_surface.gaussian_cost_all_patch()
    patch_surface.visualize_full_cost_map()    
    patch_surface.print_patch_cost_matrix(0, grid_size=10, visualize=True)
    # # TEST 11: Visualize results
    # print("\n[TEST] === TEST 11: Visualization ===")
    # print("[TEST] Plotting all patches with cost coloring...")
    # patch_surface.plot_patches()
    
    # print("[TEST] Plotting patches with target points highlighted...")
    # patch_surface.plot_patches_points_target()
    
    # # Plot a specific patch
    # print(f"[TEST] Plotting patch {test_patch_id} in detail...")
    # patch_surface.plot_patch(test_patch_id)
    
    # # Plot mesh grid for a patch
    # print(f"[TEST] Getting mesh grid for patch {test_patch_id}...")
    # X_grid, Y_grid, Z_grid = patch_surface.get_mesh_grid_patch(test_patch_id, plot_patch=True)
    
    # # TEST 12: Advanced test - trajectory points
    # print("\n[TEST] === TEST 12: Testing Trajectory Points ===")
    # P0_INIT = np.array([0.0, 2.5, -1.5])
    # PF_INIT = np.array([0.0, 4.0, -0.5])
    
    # print(f"[TEST] Start point P0: {P0_INIT}")
    # print(f"[TEST] End point PF: {PF_INIT}")
    
    # # Find patches for trajectory endpoints
    # patch_p0 = patch_surface.get_patch_id_from_point_2D(P0_INIT[1], P0_INIT[2])
    # patch_pf = patch_surface.get_patch_id_from_point_2D(PF_INIT[1], PF_INIT[2])
    
    # print(f"[TEST] P0 is in patch: {patch_p0}")
    # print(f"[TEST] PF is in patch: {patch_pf}")
    
    # if patch_p0 is not None:
    #     p0_on_surface = patch_surface.get_point_t_in_surface(
    #         patch_p0, P0_INIT[1], P0_INIT[2], print_info=True, plot_patch=False)
    #     if p0_on_surface:
    #         print(f"[TEST] P0 projected on surface: {p0_on_surface['position']}")
    
    # if patch_pf is not None:
    #     pf_on_surface = patch_surface.get_point_t_in_surface(
    #         patch_pf, PF_INIT[1], PF_INIT[2], print_info=True, plot_patch=False)
    #     if pf_on_surface:
    #         print(f"[patch_surface] PF projected on surface: {pf_on_surface['position']}")
    
    # # # TEST 13: Generate cost mesh grid
    # # print("\n[TEST] === TEST 13: Generating Cost Mesh Grid ===")
    # # cost_grid = patch_surface.get_cost_meshgrid(grid_size_y=100, grid_size_z=100, plot_mesh=False)
    
    # # TEST 14: print list of patches
    # print("\n[TEST] === TEST 14: Plotting Specific Patches by ID ===")
    # patches_list = [1 , 10 , 50 , 60 , 4 , 8 , 28]
    # patch_surface.plot_patches_by_id(patches_list)
    
    # # TEST 15: Plot 2D patch map with IDs
    # print("\n[TEST] === TEST 15: Plotting 2D Patch Map with IDs ===")
    # patch_surface.plot_patches_2D_with_ids(figsize=(14, 10), show_ids=True, fontsize=10)
    
if __name__ == "__main__":
    main()