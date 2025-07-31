from terrain_manager import TerrainManager  # Assicurati che TerrainManager sia in terrain_manager.py
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage
from scipy.interpolate import griddata

KERNEL_SMOOTHING = 10
KERNEL_LAPLACIAN = 20
KERNEL_BLUR = 30
KERNEL_SOBEL_Y_V1 = 40
KERNEL_SOBEL_Z_V1 = 50
KERNEL_SOBEL_Y_V2 = 60
KERNEL_SOBEL_Z_V2 = 70

class PointCloudFilter:
    def __init__(self, pc, h_min=0, h_max=5):
        N = len(pc)
        self.pc = pc        
        self.color = np.full((N, 3), [0, 1, 0])
        self.size_point = np.ones(N) *1
        
        self.cost = np.ones(N) * 0.5  # Valore iniziale costo --> cosi evito falsi positivi
        
        self.light = np.ones(N) * 0.8  # Valore iniziale di luce (media)
        
        self.points_t = [
            {
                'position':     self.pc[i],
                'color':        self.color[i],
                'light':        self.light[i],  
                'size_point':   self.size_point[i],
                'cost':         self.cost[i],
                
            }
            for i in range(N)
        ]  
        self.x_points = np.array([point['position'][0] for point in self.points_t])
        self.y_points = np.array([point['position'][1] for point in self.points_t])
        self.z_points = np.array([point['position'][2] for point in self.points_t])
        
        self.h_min = h_min
        self.h_max = h_max    
        
        self.print_information()
        
        # hyper parameters
        self.grid_resolution = 0.1      # Risoluzione della griglia per l'interpolazione
        self.threshold_extreme = 0.7    # Soglia per evidenziare pendenze estreme
        self.init_kernel()              
        
    def print_information(self):
    
        print(f"Point cloud statistics:")
        print(f"  -Total points: {len(self.pc)}")
        print(f"  -Height range define: {np.min(self.x_points):.2f} to {np.max(self.x_points):.2f} m")
        print(f"  -Y range: {np.min(self.y_points):.2f} to {np.max(self.y_points):.2f} m")
        print(f"  -Z range: {np.min(self.z_points):.2f} to {np.max(self.z_points):.2f} m")
       
    def print_map_pc(self, points_t=None):
        
        if points_t is None:
            points_t = self.points_t
            x_points = np.array([point['position'][0] for point in self.points_t])
            y_points = np.array([point['position'][1] for point in self.points_t])
            z_points = np.array([point['position'][2] for point in self.points_t])
        else:
            points_t = points_t
            x_points = np.array([point['position'][0] for point in points_t])
            y_points = np.array([point['position'][1] for point in points_t])
            z_points = np.array([point['position'][2] for point in points_t])
        color = np.array([point['color'] for point in points_t])
        size_point = np.array([point['size_point'] for point in points_t])
        
        # Apply lighting effect to colors - fix the multiplication
        color_with_light = np.array([
            [c * point['light'] for c in point['color']] for point in points_t
        ])
        
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d') 
        
        # Fix the scatter plot by using proper color data
        scatter = ax.scatter(x_points, y_points, z_points, 
                           c=x_points,  # Use x coordinates for color mapping
                           s=size_point,
                           alpha=0.7,
                           cmap='viridis')
        
        # Add colorbar
        plt.colorbar(scatter, ax=ax, shrink=0.5, aspect=20)
        
        ax.set_xlabel('X (m) - Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Point Cloud ({len(x_points)} points)')
        # ax.view_init(elev=20, azim=45)
        plt.tight_layout()
        plt.show()
           
    def filter_height(self):
        
        self.x_points = np.array([point['position'][0] for point in self.points_t])
        mask = (self.x_points >= self.h_min) & (self.x_points <= self.h_max)
        
        filtered_points_t = [point for i, point in enumerate(self.points_t) if mask[i]]
    
        return filtered_points_t
      
    def init_kernel(self):
        # Smoothing kernel  --> like laplacian !???
        self.smoothing_kernel = np.array([[1, 1, 1],
                                          [1, 2, 1],
                                          [1, 1, 1]]) 
        self.smoothing_kernel = self.smoothing_kernel / self.smoothing_kernel.sum()
        
        # Laplacian kernel
        self.laplacian_kernel = np.array([[-1, -1, -1],
                                        [-1, 8., -1],
                                        [-1, -1, -1]])
        # Blur kernel
        self.blur_kernel = np.ones((3, 3)) / 9
        
        # Sobel kernels
        self.sobel_y_v2 = np.array([[2, 0, -2],
                                            [4, 0, -4],
                                            [2, 0, -2]])
            
        self.sobel_z_v2 = np.array([[2, 4, 2],
                                         [0, 0, 0],
                                         [-2, -4, -2]])
        
        self.sobel_y_v1 = np.array([[1, 0, -1],
                                [2, 0, -2],
                                [1, 0, -1]])
        
        self.sobel_z_v1 = np.array([[1, 2, 1],
                                    [0, 0, 0],
                                    [-1, -2, -1]])
        
        # blur kernel for smoothing
        self.blur_kernel = np.ones((3, 3)) / 9

    
    def convolution(self,source_points ,kernel):
        if source_points is None:
            points_conv = self.points_t
            x_points = np.array([point['position'][0] for point in self.points_t])
            y_points = np.array([point['position'][1] for point in self.points_t])
            z_points = np.array([point['position'][2] for point in self.points_t])
        else:
            points_conv = source_points
            x_points = np.array([point['position'][0] for point in points_conv])
            y_points = np.array([point['position'][1] for point in points_conv])
            z_points = np.array([point['position'][2] for point in points_conv])
            
        grid_y = np.arange(y_points.min(), y_points.max(), self.grid_resolution)
        grid_z = np.arange(z_points.min(), z_points.max(), self.grid_resolution)
        grid_Y, grid_Z = np.meshgrid(grid_y, grid_z) 
        
        interpolated_x = griddata((y_points, z_points), x_points, 
                                 (grid_Y, grid_Z), method='linear', fill_value=0)
        
        # Initialize grad_result with the correct shape (interpolated grid shape)
        grad_result = np.zeros((2, *interpolated_x.shape))
        
        if len(kernel) ==1:
            print("kernel single")
            grad_result[0] = ndimage.convolve(interpolated_x, kernel[0], mode='constant') 
            magnitude = np.abs(grad_result[0])
        elif len(kernel) == 2:
            print("kernel double")
            grad_result[0] = ndimage.convolve(interpolated_x, kernel[0], mode='constant')
            grad_result[1] = ndimage.convolve(interpolated_x, kernel[1], mode='constant')
            magnitude = np.sqrt(grad_result[0]**2 + grad_result[1]**2)
            
        gradient_norm = magnitude / (np.max(magnitude) + 1e-8)
        gradient_at_points = griddata((grid_Y.flatten(), grid_Z.flatten()),
                                    gradient_norm.flatten(),
                                    (y_points, z_points), method='linear', fill_value=0)
        gradient_colors = plt.cm.hot(gradient_at_points)
        gradient_norm = magnitude / (np.max(magnitude) + 1e-8)
        
        for i in range(len(points_conv)):
            points_conv[i]['color'] = gradient_colors[i][:3] 
        return points_conv,gradient_norm, grid_y, grid_z, interpolated_x,gradient_at_points
    
    def apply_kernel_cost(self, kernel, gradient_at_points=None):
        cost = np.zeros(len(self.points_t))
        
        if len(kernel) == 1:
            if np.array_equal(kernel[0], self.smoothing_kernel):
                base_cost_value = KERNEL_SMOOTHING
            elif np.array_equal(kernel[0], self.laplacian_kernel):
                base_cost_value = KERNEL_LAPLACIAN
            elif np.array_equal(kernel[0], self.blur_kernel):
                base_cost_value = KERNEL_BLUR
            elif np.array_equal(kernel[0], self.sobel_y_v1):
                base_cost_value = KERNEL_SOBEL_Y_V1
            elif np.array_equal(kernel[0], self.sobel_z_v1):
                base_cost_value = KERNEL_SOBEL_Z_V1
            elif np.array_equal(kernel[0], self.sobel_y_v2):
                base_cost_value = KERNEL_SOBEL_Y_V2
            elif np.array_equal(kernel[0], self.sobel_z_v2):
                base_cost_value = KERNEL_SOBEL_Z_V2
            else:
                base_cost_value = 0 
                
        elif len(kernel) == 2:
            if (np.array_equal(kernel[0], self.sobel_y_v1) and 
                np.array_equal(kernel[1], self.sobel_z_v1)):
                base_cost_value = KERNEL_SOBEL_Y_V1 + KERNEL_SOBEL_Z_V1
            elif (np.array_equal(kernel[0], self.sobel_y_v2) and 
                  np.array_equal(kernel[1], self.sobel_z_v2)):
                base_cost_value = KERNEL_SOBEL_Y_V2 + KERNEL_SOBEL_Z_V2
            else:
                base_cost_value = 0 
        else:
            base_cost_value = 0 
        
        if gradient_at_points is not None:
            for i in range(len(self.points_t)):
                gradient_multiplier = gradient_at_points[i] if i < len(gradient_at_points) else 0
                # Modifica: penalità positiva su pendenze alte, negativa su zone piatte
                cost_increment = base_cost_value * (2 * gradient_multiplier - 1)
                self.points_t[i]['cost'] += cost_increment
                cost[i] = self.points_t[i]['cost']
        else:
            for i in range(len(self.points_t)):
                # Se non ho info sul gradiente, lascio il costo invariato oppure applico un valore costante
                self.points_t[i]['cost'] += 0  # oppure += base_cost_value se vuoi comunque aggiornare
                cost[i] = self.points_t[i]['cost']

        # Update colors based on cost values
        self.update_colors_by_cost()
        
        return cost
    
    def update_colors_by_cost(self):
        """Update point colors based on their cost values"""
        # Get all cost values
        all_costs = np.array([point['cost'] for point in self.points_t])
        
        # Normalize costs to [0, 1] range
        min_cost = np.min(all_costs)
        max_cost = np.max(all_costs)
        
        # Avoid division by zero
        if max_cost - min_cost > 1e-8:
            normalized_costs = (all_costs - min_cost) / (max_cost - min_cost)
        else:
            normalized_costs = np.zeros_like(all_costs)
        
        # Update colors: green for low cost, red for high cost
        for i, normalized_cost in enumerate(normalized_costs):
            # Interpolate between green [0, 1, 0] and dark red [0.8, 0, 0]
            red = normalized_cost * 0.8  # Scale to dark red
            green = (1 - normalized_cost)  # Full green for low cost, no green for high cost
            blue = 0  # No blue component
            
            self.points_t[i]['color'] = np.array([red, green, blue])
    
    def compute_conv_cost(self, source_points, kernel):
        
        points_conv, gradient_norm, grid_y, grid_z, interpolated_x, gradient_at_points = self.convolution(source_points, kernel)
        cost = self.apply_kernel_cost(kernel, gradient_at_points)
        return points_conv, gradient_norm, grid_y, grid_z, interpolated_x, cost
    

    
    def generate_map_filtered(self,kernel, source_points =None):
        
        points_conv, gradient_norm, grid_y, grid_z, interpolated_x, cost = self.compute_conv_cost(source_points, kernel)
        fig = plt.figure(figsize=(16, 12))
        
        # Subplot 1: Gradient map (top-left)
        ax1 = fig.add_subplot(221)
        im1 = ax1.imshow(gradient_norm, extent=[np.min(grid_y), np.max(grid_y), 
                                              np.min(grid_z), np.max(grid_z)], 
                        origin='lower', cmap='hot', aspect='auto')
        ax1.set_xlabel('Y (m)')
        ax1.set_ylabel('Z (m)') 
        ax1.set_title('Mappa delle Pendenze (Gradiente)')
        plt.colorbar(im1, ax=ax1, label='Intensità Pendenza')
        
        # Subplot 2: Point cloud colored by gradient (top-right)
        ax2 = fig.add_subplot(222, projection='3d')
        
        point_colors = np.array([point['color'] for point in points_conv])
        point_sizes = np.array([point['size_point'] for point in points_conv])
        
        x_points = np.array([point['position'][0] for point in points_conv])
        y_points = np.array([point['position'][1] for point in points_conv])
        z_points = np.array([point['position'][2] for point in points_conv])
        
        scatter2 = ax2.scatter(x_points, y_points, z_points, 
                              c=point_colors, 
                              s=point_sizes, 
                              alpha=0.8)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.set_title('Point Cloud - Pendenze Colorate')
        
        # Subplot 3: Interpolated surface (bottom-left)
        ax3 = fig.add_subplot(223, projection='3d')
        Y_grid, Z_grid = np.meshgrid(grid_y, grid_z)
        ax3.plot_surface(interpolated_x, Y_grid, Z_grid, 
                        facecolors=plt.cm.hot(gradient_norm),
                        alpha=0.8, shade=True)
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Y (m)')
        ax3.set_zlabel('Z (m)')
        ax3.set_title('Superficie con Mappa Pendenze')
        
        # Subplot 4: Cost map (bottom-right)
        ax4 = fig.add_subplot(224)
        cost_values = np.array([point['cost'] for point in points_conv])
        
        scatter4 = ax4.scatter(y_points, z_points, 
                              c=cost_values, 
                              s=50,  # Point size
                              cmap='plasma', 
                              alpha=0.8,
                              edgecolors='black',
                              linewidth=0.5)
        
        ax4.set_xlabel('Y (m)')
        ax4.set_ylabel('Z (m)')
        ax4.set_title('Mappa dei Costi per Punto')
        plt.colorbar(scatter4, ax=ax4, label='Valore Costo')
        plt.tight_layout()
        plt.show()
        self.print_map_pc()
        return
        

    def get_x_coordinates(self):
        return np.array([point['position'][0] for point in self.points_t])
    
    def get_y_coordinates(self):
        return np.array([point['position'][1] for point in self.points_t])
    
    def get_z_coordinates(self):
        return np.array([point['position'][2] for point in self.points_t])
    
    #to update the cost of a point
    def update_point_cost(self, index, cost):
        if 0 <= index < len(self.points_t):
            self.points_t[index]['cost'] = cost
            self.cost[index] = cost 
    
    def get_serializable_points(self):
        return [
            {
                'position': point['position'].tolist() if hasattr(point['position'], 'tolist') else list(point['position']),
                'color': point['color'].tolist() if hasattr(point['color'], 'tolist') else list(point['color']),
                'light': float(point['light']),
                'size_point': float(point['size_point']),
                'cost': float(point['cost']),
            }
            for point in self.points_t
        ]
    
def main():
    
    #terrain stuff values
    wall_depth = 1            
    grid_size = 100
    max_ridge_depth = 0.5     
    seed = 47                 
    Lz = -60                  
    Ly = 10                   
    
    terrain = TerrainManager(grid_size, wall_depth=wall_depth, max_ridge_depth=max_ridge_depth, seed=seed, Lz=Lz, Ly=Ly)
    # terrain.plot_debug(debug=True)

    pc = terrain.point_cloud

    # point cloud filter test
    pc_filter = PointCloudFilter(pc, h_min=1.0, h_max=4.0)
    
    print("\n=== original map ===")
    pc_filter.print_map_pc()
    
    # print("\n=== height limit ===")
    # new_points =pc_filter.filter_height()
    # pc_filter.print_map_pc(new_points)
    
    # print("\n=== Blur ===")
    # kernel = [pc_filter.laplacian_kernel] 
    # pc_filter.generate_map_filtered(new_points,kernel)

    # print("\n=== Smoothing ===")
    # kernel = [pc_filter.smoothing_kernel] 
    # pc_filter.generate_map_filtered(new_points,kernel)
    
    # print("\n=== I derivative --> slope ===")
    # kernel = [pc_filter.sobel_y_v1, pc_filter.sobel_z_v1] 
    # pc_filter.generate_map_filtered(new_points, kernel)
    
    print("\n=== I derivative --> slope ===")
    kernel = [pc_filter.sobel_y_v2, pc_filter.sobel_z_v2] 
    pc_filter.generate_map_filtered( kernel=kernel)
    
    
    
    

if __name__ == "__main__":
    main()
