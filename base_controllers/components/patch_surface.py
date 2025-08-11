from terrain_manager import TerrainManager
from point_cloud_filter import PointCloudFilter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm


class PatchSurface:
    def __init__(self, points_t ,number_of_patches_width=10, number_of_patches_height=10):
        self.number_of_patches_width = number_of_patches_width
        self.number_of_patches_height = number_of_patches_height
        
        # self.patch_width
        # self.patch_height
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
            print ("Error: No points tyep points_t provided for patch surface creation.")
            return
        self.lx, self.ly, self.patch_height, self.patch_width = self.dimension_of_map()
    
    #dimension of the map and patches
    def dimension_of_map(self):
        Ly = self.y_max - self.y_min
        Lz = self.z_max - self.z_min
        
        # Dimensioni della singola patch
        patch_width = Ly / self.number_of_patches_width if self.number_of_patches_width else 0.0
        patch_height = Lz / self.number_of_patches_height if self.number_of_patches_height else 0.0
        return Ly, Lz, patch_height, patch_width
    
    
    def create_patches(self):
        Ly, Lz, patch_height, patch_width = self.dimension_of_map()

        y_edges = self.y_min + np.arange(self.number_of_patches_width + 1)  * patch_width
        z_edges = self.z_min + np.arange(self.number_of_patches_height + 1) * patch_height
        
        patch_id = 0
        
        for i in range(self.number_of_patches_width):
            for j in range(self.number_of_patches_height):
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
                    'points_t': points_in_patch,
                    'centroid': centroid,
                    'cost_patch': mean_cost,
                })
                patch_id += 1
                
    def cost_patch(self, some_points):               
        costs = [p['cost'] for p in some_points if 'cost' in p]
        return float(np.mean(costs))
    
    def centroid_patch(self, points_in_patch, y_min, y_max, z_min, z_max):
        
        y_centroid = (y_min + y_max) / 2.0
        z_centroid = (z_min + z_max) / 2.0
        
        if points_in_patch:
            x_positions = [p['position'][0] for p in points_in_patch]
            x_centroid = np.mean(x_positions)
        else:
            x_centroid = np.mean(self.all_pc[:, 0])
        
        return np.array([x_centroid, y_centroid, z_centroid])
    
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
            
            
    def plot_patches(self,s=4, alpha=0.5):

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title('Patches 3D')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        for patch in self.patches:
            pts = patch.get('points_t') or patch.get('points') or []
            if not pts:
                continue

            P = np.vstack([p['position'] for p in pts])  
            color = patch.get('color_patch')
            ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=s, alpha=alpha, color=color)
            
            centroid = patch.get('centroid')
            if centroid is not None:
                ax.scatter(centroid[0], centroid[1], centroid[2], 
                          s=20, c='blue', marker='o', alpha=1.0, edgecolors='white', linewidth=1)

        plt.tight_layout()
        plt.show()
        return fig, ax

def main():
    # Terrain configuration values
    wall_depth = 1
    grid_size = 100
    max_ridge_depth = 0.5
    seed = 47
    Lz = 10
    Ly = 50
    terrain = TerrainManager(grid_size, wall_depth=wall_depth, max_ridge_depth=max_ridge_depth, seed=seed, Lz=Lz, Ly=Ly)
    # terrain.plot_debug(debug=True)
    pc = terrain.point_cloud
    # Point cloud filter test
    pc_filter = PointCloudFilter(pc, h_min=1.0, h_max=4.0)
    print("\n=== Original Map ===")
    pc_filter.print_map_pc()
    #filtro con cancellazione punti
    print("\n=== Height Filter ===")
    new_points = pc_filter.filter_height()
    # pc_filter.print_map_pc(new_points)
    print("\n=== Logarithmic Height Cost Filter ===")
    #filtro con cambio di costo e colore in base all'altezza
    new_points=pc_filter.filter_height_profile(x0=1.5, scale=0.5, profile="exponential")
    # pc_filter.visualize_cost_map(new_points)
    #TEST DIMENSION OF MAP
    patch_surface = PatchSurface(new_points)
    Ly_map, Lz_map, patch_height, patch_width = patch_surface.dimension_of_map()
    patch_surface.create_patches()
    # patch_surface.random_color()
    patch_surface.cost_color()     
    patch_surface.plot_patches()
    print(f"\ndimension of mappa and patch: Ly = {Ly_map:.2f} m, Lz = {Lz_map:.2f} m, patch_height = {patch_height:.2f} m, patch_width = {patch_width:.2f} m")

if __name__ == "__main__":
    main()