from terrain_manager import TerrainManager  
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage
from scipy.interpolate import griddata
from matplotlib.colors import LinearSegmentedColormap

KERNEL_SMOOTHING = 10
KERNEL_LAPLACIAN = 20
KERNEL_BLUR = 30
KERNEL_SOBEL_Y_V1 = 40
KERNEL_SOBEL_Z_V1 = 50
KERNEL_SOBEL_Y_V2 = 60
KERNEL_SOBEL_Z_V2 = 70

class PointCloudFilter:
    
    def __init__(self, pc, h_min=0, h_max=5,plane_x=0.0):
        N = len(pc)
        self.pc = pc        
        self.color = np.full((N, 3), [0,0,0])
        self.size_point = np.ones(N) *1
        
        self.cost = np.ones(N) * 0.5  # Valore iniziale costo --> cosi evito falsi positivi
        
        self.light = np.ones(N) * 0.8 
        
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
        self.threshold_extreme = 0.7    # Soglia per evidenziare pendenze estreme --> ancora da usare
        
        # Grid/surface cache
        self.grid_y = None
        self.grid_z = None
        self.grid_Y = None
        self.grid_Z = None
        self.surface = None     
        
        
        self.print_information()
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
                
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d') 
        
        ax.scatter(x_points, y_points, z_points, c=color, s=size_point) 
        ax.set_xlabel('X (m) - Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Point Cloud ({len(x_points)} points)')
        plt.tight_layout()
        plt.show()
           
    def filter_height(self):
        
        self.x_points = np.array([point['position'][0] for point in self.points_t])
        mask = (self.x_points >= self.h_min) & (self.x_points <= self.h_max)
        filtered_points_t = [point for i, point in enumerate(self.points_t) if mask[i]]

        
        return filtered_points_t   
        

    def filter_height_peak(self, profile="logln",x0 = 0.0, scale=0.5):
        print("equation used: {}".format(profile))
        x_points = np.array([p['position'][0] for p in self.points_t])

        # epsilon per evitare divisione per zero
        epsilon = 1e-8
        x = np.abs(x_points - x0)

        #nota: clip limita i valori quindi non va oltre
        if profile == "linear_positive":
            cost_values = np.clip(x / (scale + epsilon), 0.0, 1.0)
        elif profile == "linear_negative":
            cost_values = np.clip(1.0 - x / (scale + epsilon), 0.0, 1.0)
        elif profile == "logln":
            cost_values = np.log(1.0 + x / (scale + epsilon)) / (x + epsilon)
        elif profile == "exponential":
            cost_values = 1.0 - np.exp(- x / (scale + epsilon))

        else:
            raise ValueError("profile deve essere 'linear_positive', 'linear_negative', 'logln' o 'exponential'")

        # Normalizza su [0, 100]
        cmin, cmax = cost_values.min(), cost_values.max()
        if cmax > cmin:
            normalized_costs = (cost_values - cmin) / (cmax - cmin) * 100.0
        else:
            normalized_costs = np.zeros_like(cost_values)

        # Colori
        cmap = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        gradient_colors = cmap(normalized_costs / 100.0)

        # Aggiorna punti
        for i, p in enumerate(self.points_t):
            p['cost']  = float(normalized_costs[i])
            p['color'] = gradient_colors[i][:3]

        return self.points_t

    def init_kernel(self):
        # Blur kernel
        self.blur_kernel = np.ones((3, 3)) / 9
        
        # Smoothing kernel (like laplacian)
        self.smoothing_kernel = np.array([[1, 1, 1],
                                           [1, 2, 1],
                                           [1, 1, 1]])
        
        self.smoothing_kernel = self.smoothing_kernel / self.smoothing_kernel.sum()
        
        # Sobel kernels
        scale_factor = 1
        self.sobel_y =scale_factor * np.array([[1, 0, -1],
                                                [2, 0, -2],
                                                [1, 0, -1]])
        self.sobel_z =scale_factor * np.array([[1, 2, 1],
                                                [0, 0, 0],
                                                [-1, -2, -1]])
        
        # Laplacian kernel
        self.laplacian_kernel = np.array([[0,  1, 0],
                                            [1, -4, 1],
                                            [0,  1, 0]])
        
        # Laplacian of Gaussian (LoG) kernel
        self.log_kernel = np.array([
                                    [0,  0, -1,  0,  0],
                                    [0, -1, -2, -1,  0],
                                    [-1, -2, 16, -2, -1],
                                    [0, -1, -2, -1,  0],
                                    [0,  0, -1,  0,  0]
                                ])

    def interpolation_to_surface(self, source_points=None):
        
        points = self.points_t
        x_points = np.array([point['position'][0] for point in self.points_t])
        y_points = np.array([point['position'][1] for point in self.points_t])
        z_points = np.array([point['position'][2] for point in self.points_t])
        if source_points is not None:
            points = source_points
            x_points = np.array([point['position'][0] for point in points])
            y_points = np.array([point['position'][1] for point in points])
            z_points = np.array([point['position'][2] for point in points])
        self.grid_y = np.arange(y_points.min(), y_points.max(), self.grid_resolution)
        self.grid_z = np.arange(z_points.min(), z_points.max(), self.grid_resolution)
        # Create a grid for interpolation
        self.grid_Y, self.grid_Z = np.meshgrid(self.grid_y, self.grid_z)
        # Interpolate the surface using griddata
        self.surface = griddata((y_points,z_points),
                                x_points,
                                (self.grid_Y, self.grid_Z),
                                method="linear",
                                fill_value=0.0, 
                                )
               
    def convolution_process(self,surface,kernel):
        mode = 'nearest'
        # To try: surface = ndimage.gaussian_filter(surface, sigma=smooth_sigma, mode="reflect") and other ndimage filters
        if len(kernel) == 1:
            print("kernel single")
            surface_fitered = ndimage.convolve(surface, kernel[0], mode=mode) 
            magnitude = np.abs(surface_fitered)
            return surface_fitered

        elif len(kernel) == 2:
            print("kernel double")
            surface_fitered_0 =  ndimage.convolve(surface, kernel[0], mode=mode)
            surface_fitered_1 =  ndimage.convolve(surface, kernel[1], mode=mode)
            surface_fitered = np.sqrt(surface_fitered_0**2 + surface_fitered_1**2)
            return surface_fitered
        else:
            print("kernel not supported")
            return None
    
    def convolution_into_points(self, source_points, surface):
        
        if self.grid_Y is None or self.grid_Z is None:
            raise RuntimeError("Grid not initialized. Call interpolation() first.")
        
        points = self.points_t
        x_points = np.array([point['position'][0] for point in self.points_t])
        y_points = np.array([point['position'][1] for point in self.points_t])
        z_points = np.array([point['position'][2] for point in self.points_t])
        if source_points is not None:
            points = source_points
            x_points = np.array([point['position'][0] for point in points])
            y_points = np.array([point['position'][1] for point in points])
            z_points = np.array([point['position'][2] for point in points])
        # Interpolate the surface at the point coordinates
        gradient_at_points = griddata(
            (self.grid_Y.flatten(), self.grid_Z.flatten()),
            surface.flatten(),
            (y_points, z_points),
            method="linear",
            fill_value=0.0,
        )
        # Red color scale
        gradient_colors = plt.cm.hot(gradient_at_points)
        for i, point in enumerate(points):
            point['color'] = gradient_colors[i][:3]

        return gradient_at_points
        
    def compute_conv_step(self, kernel, source_points=None,plot=False):
        if source_points is None:
            source_points = self.points_t
        if self.surface is None:
            # 1. Interpolation
            self.interpolation_to_surface(source_points)
        
        # 2. Convolution
        self.surface = self.convolution_process(self.surface, kernel)
        
        # 3. Convolution into points
        gradient_at_points = self.convolution_into_points(source_points, self.surface)
        
        # 4. Plot
        if plot:
            self.visualize_filter_operation(self.surface, source_points, self.grid_y, self.grid_z)
        return gradient_at_points
    
    def compute_cost(self,gradient_at_points,source_points=None,plot=False):
        if source_points is None:
            source_points = self.points_t
        if gradient_at_points is None:
            raise ValueError("gradient_at_points cannot be None, do the compute_conv_step first")
        points = source_points
        grad_min = np.min(gradient_at_points)
        grad_max = np.max(gradient_at_points)
        if grad_max > grad_min:  # Avoid division by zero
            cost_values = (gradient_at_points - grad_min) / (grad_max - grad_min) * 100
        else:
            cost_values = np.zeros_like(gradient_at_points)

        cmap = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        gradient_colors = cmap(cost_values / 100.0)  # Normalize to [0, 1]
        
        for i, point in enumerate(points):
            point['color'] = gradient_colors[i][:3]  
            point['cost'] = float(cost_values[i])    
        if plot:
            self.visualize_cost_map(source_points)
    
    def process_points(self, kernel,source_points=None, plot=False):
        if source_points is None:
            source_points = self.points_t
        gradient_at_points = self.compute_conv_step(kernel, source_points,plot=plot)
        self.compute_cost(gradient_at_points,source_points,plot=plot)
        
    def visualize_filter_operation(self,surface, source_points, grid_y, grid_z):
        
        fig = plt.figure(figsize=(16, 10))
        x_points = np.array([point['position'][0] for point in source_points])
        y_points = np.array([point['position'][1] for point in source_points])
        z_points = np.array([point['position'][2] for point in source_points])
        Y_grid, Z_grid = np.meshgrid(grid_y, grid_z)
        original_surface = griddata(
            (y_points, z_points),
            x_points,
            (Y_grid, Z_grid),
            method="linear",
            fill_value=0.0,
        )
        
        # Subplot 1: Filter response map (top-left)
        ax1 = fig.add_subplot(221)
        im1 = ax1.imshow(surface, extent=[np.min(grid_y), np.max(grid_y), 
                                         np.min(grid_z), np.max(grid_z)], 
                        origin='lower', cmap='hot', aspect='auto')
        ax1.set_xlabel('Y (m)')
        ax1.set_ylabel('Z (m)') 
        ax1.set_title('Filter Response Map')
        plt.colorbar(im1, ax=ax1, label='Response Intensity')
        
        # Subplot 2: Point cloud colored by filter response (top-right)
        ax2 = fig.add_subplot(222, projection='3d')
        
        point_colors = np.array([point['color'] for point in source_points])
        point_sizes = np.array([point['size_point'] for point in source_points])
        
        ax2.scatter(x_points, y_points, z_points, 
                   c=point_colors, 
                   s=point_sizes, 
                   alpha=0.8)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.set_title('Point Cloud - Colored by Filter Response')
        
        # Subplot 3: Filter response as 3D surface (bottom-left)
        ax3 = fig.add_subplot(223, projection='3d')
        ax3.plot_surface(Y_grid, Z_grid, surface,
                        cmap='hot',
                        alpha=0.8, shade=True)
        ax3.set_xlabel('Y (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_zlabel('Filter Response')
        ax3.set_title('3D Filter Response Surface')
        
        # Subplot 4: Original surface colored by filter response (bottom-right)
        ax4 = fig.add_subplot(224, projection='3d')
        # Normalize surface values for coloring
        norm_surface = (surface - np.min(surface)) / (np.max(surface) - np.min(surface))
        ax4.plot_surface(Y_grid, Z_grid, original_surface,
                        facecolors=plt.cm.hot(norm_surface),
                        alpha=0.8, shade=True)
        ax4.set_xlabel('Y (m)')
        ax4.set_ylabel('Z (m)')
        ax4.set_zlabel('X (m) - Height')
        ax4.set_title('Original Terrain Colored by Filter Response')
        
        plt.tight_layout()
        plt.show()
    
    def visualize_cost_map(self, source_points=None):
        x_points = np.array([point['position'][0] for point in source_points])
        y_points = np.array([point['position'][1] for point in source_points])
        z_points = np.array([point['position'][2] for point in source_points])
        cost_values = np.array([point['cost'] for point in source_points])
        point_colors = np.array([point['color'] for point in source_points])
        
        fig = plt.figure(figsize=(16, 8))
        
        # Subplot 1: Point cloud colored by cost (3D)
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        point_sizes = np.array([point['size_point'] for point in source_points])
        scatter1 = ax1.scatter(
            x_points, y_points, z_points, 
            c=point_colors, 
            s=point_sizes,  
            alpha=0.8
        )
        ax1.set_xlabel('X (m) - Height')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('Point Cloud - Colored by Cost\n(Green=Low Cost, Red=High Cost)')
        
        # Subplot 2: Top-down view of cost map
        ax2 = fig.add_subplot(1, 2, 2)
        scatter2 = ax2.scatter(
            y_points, z_points, 
            c=point_colors, 
            s=point_sizes*2, 
            alpha=0.8
        )
        ax2.set_xlabel('Y (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('Top-Down Cost Map\n(Green=Low Cost, Red=High Cost)')
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        
        # Print cost statistics
        print(f"Cost Statistics:")
        print(f" - Min cost: {np.min(cost_values):.3f}")
        print(f" - Max cost: {np.max(cost_values):.3f}")
        print(f" - Mean cost: {np.mean(cost_values):.3f}")
        print(f" - Std cost: {np.std(cost_values):.3f}")
        
        return cost_values
    
    def get_x_coordinates(self):
        return np.array([point['position'][0] for point in self.points_t])
    
    def get_y_coordinates(self):
        return np.array([point['position'][1] for point in self.points_t])
    
    def get_z_coordinates(self):
        return np.array([point['position'][2] for point in self.points_t])
    
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
    
    # Terrain configuration values
    wall_depth = 1            
    grid_size = 100
    max_ridge_depth = 0.5     
    seed = 47                 
    Lz = 10                  
    Ly = 10                   
    
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
    pc_filter.print_map_pc(new_points)
    
    print("\n=== Logarithmic Height Cost Filter ===")    
    #filtro con cambio di costo e colore in base all'altezza
    new_points=pc_filter.filter_height_peak(x0=1.5, scale=0.5, profile="linear_positive")
    pc_filter.visualize_cost_map(new_points)
    new_points=pc_filter.filter_height_peak(x0=1.5, scale=0.5, profile="linear_negative")
    pc_filter.visualize_cost_map(new_points)
    new_points=pc_filter.filter_height_peak(x0=1.5, scale=0.5, profile="logln")
    pc_filter.visualize_cost_map(new_points)
    new_points=pc_filter.filter_height_peak(x0=1.5, scale=0.5, profile="exponential")
    pc_filter.visualize_cost_map(new_points)
    
    print("\n=== Smoothing Filter ===")
    kernel = [pc_filter.smoothing_kernel] 
    pc_filter.process_points(kernel, new_points, plot=False)
    
    print("\n=== First Derivative (Gradient) ===")
    kernel = [pc_filter.sobel_y, pc_filter.sobel_z] 
    pc_filter.process_points(kernel, new_points, plot=True)
    
    print("\n=== Second Derivative (Laplacian) ===")
    kernel = [pc_filter.laplacian_kernel] 
    pc_filter.process_points(kernel,new_points, plot=True)
    
    print("\n=== Laplacian of Gaussian (LoG) ===")
    kernel = [pc_filter.log_kernel] 
    pc_filter.process_points(kernel,new_points, plot=True)
    
    
    

if __name__ == "__main__":
    main()
