from .terrain_manager import TerrainManager  
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage
from scipy.interpolate import griddata
from matplotlib.colors import LinearSegmentedColormap
import time
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
        self.color = np.full((N, 3), [0,0,0])
        self.size_point = np.ones(N) *4
        self.cost = np.ones(N) * 0.0  # Valore iniziale costo --> cosi evito falsi positivi
        
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
        
        self.h_min = h_min #fuori dagli estremi si eliminano i punti --> filter_height()
        self.h_max = h_max    
        
        # hyper parameters
        self.grid_resolution = 0.1      # Risoluzione della griglia per l'interpolazione
        self.threshold_extreme = 0.7    # Soglia per evidenziare pendenze estreme --> ancora da usare
        
        # Grid/surface cache
        self.grid_x = None
        self.grid_y = None
        self.grid_z = None
        self.grid_Y = None
        self.grid_Z = None
        self.surface = None     
        
        self.init_kernel()      
        self.print_information()        
        
    def init_kernel(self):
        ''' 
        Possible kernels to use for filtering: blur, smoothing, sobel, laplacian, LoG
        '''
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
                                                [1, 0, -1]]) *2
        self.sobel_z =scale_factor * np.array([[1, 2, 1],
                                                [0, 0, 0],
                                                [-1, -2, -1]])*2
        
        # Laplacian kernel
        self.laplacian_kernel = np.array([[0,  1, 0],
                                            [1, -8, 1],
                                            [0,  1, 0]])
        
        # Laplacian of Gaussian (LoG) kernel
        self.log_kernel = np.array([
                                    [0,  0, -1,  0,  0],
                                    [0, -1, -2, -1,  0],
                                    [-1, -2, 16, -2, -1],
                                    [0, -1, -2, -1,  0],
                                    [0,  0, -1,  0,  0]
                                ])
        size = 5
        sigma = 1.0
        ax = np.linspace(-(size - 1) / 2., (size - 1) / 2., size)
        gauss = np.exp(-0.5 * np.square(ax) / np.square(sigma))
        kernel_2d = np.outer(gauss, gauss)
        self.gaussian_bump_kernel = kernel_2d / kernel_2d.sum()

    def compute_bump_detection(self, source_points=None, weight=1.0, plot=False):
        '''
        Rileva rigonfiamenti eliminando il bias dell'altezza media (DoG).
        '''
        if source_points is None:
            source_points = self.points_t
        if self.surface is None:
            self.interpolation_to_surface(source_points)
        s1 = ndimage.gaussian_filter(self.surface, sigma=0.0) # sigma piccolo per mantenere i dettagli, ma potrebbe essere un parametro da regolare
        s2 = ndimage.gaussian_filter(self.surface, sigma=100.0) # sigma più grande per catturare la tendenza generale del terreno (piano di riferimento)        
        # La differenza isola il rigonfiamento rispetto al piano
        bump_response = s1 - s2
        # Filtriamo i valori negativi (che indicherebbero buche invece di cupole)
        bump_response = np.maximum(bump_response, 0)
        # Mappatura sui punti
        gradient_at_points = self.convolution_into_points(source_points, bump_response)
        # Calcolo costo con normalizzazione per portare il giallo verso il verde
        self.compute_cost(gradient_at_points, source_points, weight=weight, plot=plot, normalize=False)
    
    
    # ==== filter methods           
    def filter_height(self):
        '''
        filter the point cloud removing points outside h_min and h_max
        '''
        self.x_points = np.array([point['position'][0] for point in self.points_t])
        mask = (self.x_points >= self.h_min) & (self.x_points <= self.h_max)
        self.points_t  = [point for i, point in enumerate(self.points_t) if mask[i]]
                  
    def filter_height_profile(self, source_points=None,profile="logln",x0 = 0.0, scale=0.5, side_application="depth", weight = 1.0):
        '''
        Apply a cost to the point cloud based on height profile.
        profile: "linear_positive", "linear_negative", "logln", "exponential"
        side_application: "both", "up", "depth"
        weight: weight of the cost to add to the point's cost
        x0: reference height --> arganelli
        '''
        if source_points is None:
            source_points = self.points_t
            
        print("[point_cloud_filter] equation used: {}".format(profile))
        x_points = np.array([p['position'][0] for p in source_points])
        # epsilon per evitare divisione per zero 
        epsilon = 1e-8
        if (side_application == "both"):
            x = np.abs(x_points - x0)
        elif (side_application == "up"):
            x = np.where(x_points >= x0, x_points - x0, 0.0)
        elif (side_application == "depth"):
            x = np.where(x_points <= x0, x0 - x_points, 0.0)
        else:
            raise ValueError("side_application should be 'both', 'right' or 'left'")
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
            raise ValueError("profile should be 'linear_positive', 'linear_negative', 'logln' o 'exponential'")

        # Normalizza solo per i colori [0, 1] per la colormap
        cmin, cmax = cost_values.min(), cost_values.max()
        if cmax > cmin:
            normalized_for_colors_plot = (cost_values - cmin) / (cmax - cmin)
        else:
            normalized_for_colors_plot = np.zeros_like(cost_values)
        # Colors
        cmap = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        gradient_colors = cmap(normalized_for_colors_plot)
        
        # update points
        for i, point in enumerate(source_points):
            old_cost = point['cost']
            new_cost = float(cost_values[i])
            point['color'] = gradient_colors[i][:3]
            point['cost'] = old_cost + (new_cost * weight)
            #print del costo vecchio e nuovo
            # print(f"Point {i}: Old Cost = {old_cost:.2f}, New Cost = {new_cost:.2f}, Total Cost = {point['cost']:.2f}")     
        
        self.plot_color_cost_given_cost(source_points)
            
    def interpolation_to_surface(self, source_points=None):
        '''
        Interpolates the point cloud to create a surface grid. (3D --> 2D)
        '''
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
        self.grid_x = np.arange(x_points.min(), x_points.max(), self.grid_resolution)
        # Create a grid for interpolation
        self.grid_Y, self.grid_Z = np.meshgrid(self.grid_y, self.grid_z)
        # Interpolate the surface using griddata
        
        self.surface = griddata((y_points,z_points),
                                x_points,
                                (self.grid_Y, self.grid_Z),
                                method="linear", # cubic
                                fill_value=0.0, 
                                )
               
    def convolution_process(self,surface,kernel):
        '''
        Convolution process on the surface with given kernel(s).
        output: filtered surface --> same size as input surface (pendenza di ogni punto)
        
        '''
        mode = 'nearest'
        
        # To try: surface = ndimage.gaussian_filter(surface, sigma=smooth_sigma, mode="reflect") and other ndimage filters
        if len(kernel) == 1:
            print("[point_cloud_filter] kernel single")
            surface_fitered = ndimage.convolve(surface, kernel[0], mode=mode) 
            magnitude = np.abs(surface_fitered)
            return surface_fitered

        elif len(kernel) == 2:
            print("[point_cloud_filter] kernel double")
            surface_fitered_0 =  ndimage.convolve(surface, kernel[0], mode=mode)
            surface_fitered_1 =  ndimage.convolve(surface, kernel[1], mode=mode)
            surface_fitered = np.sqrt(surface_fitered_0**2 + surface_fitered_1**2)
            return surface_fitered
        else:
            print("[point_cloud_filter] kernel not supported")
            return None
    
    def convolution_into_points(self, source_points, surface):
        '''
        rimappa la matrice surface sui punti della point cloud (2D --> 3D)
        '''
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
        '''
        Interpolation + Convolution + Convolution into points
        '''
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
            self.plot_filter_response_pointcloud_3d(source_points)
            self.plot_filter_response_2d()
        # 5. if you want an incremental convolution commit this: 
        self.surface = None
        return gradient_at_points
    
    def compute_cost(self,gradient_at_points,source_points=None,weight = 1.0,plot=False,normalize=False):
        '''
        Compute cost based on gradient at points. con costo incrementale
        '''
        if source_points is None:
            source_points = self.points_t
        if gradient_at_points is None:
            raise ValueError("gradient_at_points cannot be None, do the compute_conv_step first")
        
        # 1. Prendi i valori assoluti
        cost_values = np.abs(gradient_at_points.copy()) 
        
        # 2. Calcola Min e Max del filtro corrente
        grad_min, grad_max = np.min(cost_values), np.max(cost_values)
        
        # 3. Calcola i valori normalizzati [0, 1]
        if grad_max > grad_min: 
            normalized_values = (cost_values - grad_min) / (grad_max - grad_min)
        else:
            normalized_values = np.zeros_like(cost_values)

        # Gestione colori (usa sempre i normalizzati)
        cmap = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        gradient_colors = cmap(normalized_values)
        
        for i, point in enumerate(source_points):
            old_cost = point['cost']
            
            if normalize:
                val_to_add = float(normalized_values[i])
            else:
                val_to_add = float(cost_values[i])
            
            point['cost'] = old_cost + (val_to_add * weight)
            
        self.plot_color_cost_given_cost(source_points)
        if plot:
            self.visualize_cost_map(source_points)

    def filter_process_points_pipeline(self, kernel, source_points=None, weight=1.0, plot=False):
        if source_points is None:
            source_points = self.points_t
        gradient_at_points = self.compute_conv_step(kernel, source_points,plot=plot)
        self.compute_cost(gradient_at_points,source_points,weight = weight,plot=plot)
    
    # === AUSILIAR METHODS ===
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
    
    def get_all_cost(self):
        return np.array([point['cost'] for point in self.points_t])

    # === PRINT AND PLOT METHODS ===
    def print_information(self):
    
        print(f"[point_cloud_filter] Point cloud statistics:")
        print(f"[point_cloud_filter]   -Total points: {len(self.pc)}")
        print(f"[point_cloud_filter]   -Height range define: {np.min(self.x_points):.2f} to {np.max(self.x_points):.2f} m")
        print(f"[point_cloud_filter]   -Y range: {np.min(self.y_points):.2f} to {np.max(self.y_points):.2f} m")
        print(f"[point_cloud_filter]   -Z range: {np.min(self.z_points):.2f} to {np.max(self.z_points):.2f} m")
       
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
        
        # Equal axis scaling to prevent stretching
        all_pts = np.column_stack([x_points, y_points, z_points])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
        
        ax.set_xlabel('X (m) - Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Point Cloud ({len(x_points)} points)')
        plt.tight_layout()
        plt.show()
    
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
        
        # Equal axis scaling for subplot 2
        all_pts = np.column_stack([x_points, y_points, z_points])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax2.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax2.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax2.set_zlim(mid[2] - max_range, mid[2] + max_range)
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.set_title('Point Cloud')
        sm2 = plt.cm.ScalarMappable(cmap='hot', norm=plt.Normalize(vmin=np.min(surface), vmax=np.max(surface)))
        sm2.set_array([])
        fig.colorbar(sm2, ax=ax2, shrink=0.5, pad=0.1, label='Filter Response')
        
        # Subplot 3: Filter response as 3D surface (bottom-left)
        ax3 = fig.add_subplot(223, projection='3d')
        surf3 = ax3.plot_surface(Y_grid, Z_grid, surface,
                        cmap='hot',
                        alpha=0.8, shade=True)
        ax3.set_xlabel('Y (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_zlabel('Filter Response')
        ax3.set_title('3D Filter Response Surface')
        fig.colorbar(surf3, ax=ax3, shrink=0.5, pad=0.1, label='Filter Response')
        
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
        sm4 = plt.cm.ScalarMappable(cmap='hot', norm=plt.Normalize(vmin=np.min(surface), vmax=np.max(surface)))
        sm4.set_array([])
        fig.colorbar(sm4, ax=ax4, shrink=0.5, pad=0.1, label='Filter Response')
        
        plt.tight_layout()
        plt.show()
    
    def visualize_cost_map(self, source_points=None):
        if source_points is None:
            source_points = self.points_t
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
        
        # Equal axis scaling for 3D plot
        all_pts = np.column_stack([x_points, y_points, z_points])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax1.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax1.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax1.set_zlim(mid[2] - max_range, mid[2] + max_range)
        
        ax1.set_xlabel('X (m) - Height')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('Point Cloud - Colored by Cost\n(Green=Low Cost, Red=High Cost)')
        cmap_cost = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        sm1 = plt.cm.ScalarMappable(cmap=cmap_cost, norm=plt.Normalize(vmin=np.min(cost_values), vmax=np.max(cost_values)))
        sm1.set_array([])
        fig.colorbar(sm1, ax=ax1, shrink=0.5, pad=0.1, label='Cost')
        
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
        ax2.set_aspect('equal', adjustable='box')  # Equal aspect for 2D plot
        ax2.grid(True, alpha=0.3)
        sm2 = plt.cm.ScalarMappable(cmap=cmap_cost, norm=plt.Normalize(vmin=np.min(cost_values), vmax=np.max(cost_values)))
        sm2.set_array([])
        fig.colorbar(sm2, ax=ax2, fraction=0.046, pad=0.04, label='Cost')
        plt.tight_layout()
        plt.show()
        
        # Print cost statistics
        print(f"[point_cloud_filter] Cost Statistics:")
        print(f"[point_cloud_filter]  - Min cost: {np.min(cost_values):.3f}")
        print(f"[point_cloud_filter]  - Max cost: {np.max(cost_values):.3f}")
        print(f"[point_cloud_filter]  - Mean cost: {np.mean(cost_values):.3f}")
        print(f"[point_cloud_filter]  - Std cost: {np.std(cost_values):.3f}")     

    def plot_map_with_target(self, point_xyz):
        x_points = np.array([point['position'][0] for point in self.points_t])
        y_points = np.array([point['position'][1] for point in self.points_t])
        z_points = np.array([point['position'][2] for point in self.points_t])
        color = np.array([point['color'] for point in self.points_t])
        size_point = np.array([point['size_point'] for point in self.points_t])
                
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d') 
        
        ax.scatter(x_points, y_points, z_points, c=color, s=size_point) 
        ax.scatter(point_xyz[0][0], point_xyz[0][1], point_xyz[0][2], c='black', s=100, marker='X')
        for point in point_xyz[1:-1]:
            ax.scatter(point[0], point[1], point[2], c='blue', s=100, marker='X')
        ax.scatter(point_xyz[-1][0], point_xyz[-1][1], point_xyz[-1][2], c='purple', s=100, marker='X')
        
        # Equal axis scaling
        all_x = np.concatenate([x_points, [p[0] for p in point_xyz]])
        all_y = np.concatenate([y_points, [p[1] for p in point_xyz]])
        all_z = np.concatenate([z_points, [p[2] for p in point_xyz]])
        all_pts = np.column_stack([all_x, all_y, all_z])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
        
        ax.set_xlabel('X (m) - Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Point Cloud with Target Point ')
        plt.tight_layout()
        plt.show()
        
    def animate_plot_map_with_target(self, point_xyz):
        x_points = np.array([point['position'][0] for point in self.points_t])
        y_points = np.array([point['position'][1] for point in self.points_t])
        z_points = np.array([point['position'][2] for point in self.points_t])
        color = np.array([point['color'] for point in self.points_t])
        size_point = np.array([point['size_point'] for point in self.points_t])
        
        plt.ion()
        
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.set_xlabel('X (m) - Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Point Cloud with Target Points - Animated')
        ax.scatter(x_points, y_points, z_points, c=color, s=size_point, alpha=0.6)
        
        # Calculate initial equal axis scaling
        all_x = np.concatenate([x_points, [p[0] for p in point_xyz]])
        all_y = np.concatenate([y_points, [p[1] for p in point_xyz]])
        all_z = np.concatenate([z_points, [p[2] for p in point_xyz]])
        
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
        
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(1.0) 
        for i, point in enumerate(point_xyz):
            print(f"[point_cloud_filter] Adding target point {i+1}/{len(point_xyz)}: {point}")
            if i == 0:  
                marker_color = 'green'
                marker = 'X'
                point_name = "Start Point"
            elif i == len(point_xyz) - 1:  
                marker_color = 'red'
                marker = 'X'
                point_name = "End Point"
            else:
                marker_color = 'blue'
                marker = 'X'
                point_name = f"Waypoint {i}"
            
            ax.scatter(point[0], point[1], point[2], 
                    c=marker_color, s=100, marker=marker, 
                    label=point_name if i < 3 else None) 
            
            fig.canvas.draw()
            fig.canvas.flush_events()
            # delay tra ogni punto
            time.sleep(0.3)
        
        ax.set_title(f'Point Cloud with Target Points - Complete Path\n'
                    f'Total points: {len(point_xyz)}')
        
        ax.legend()
        plt.tight_layout()
        # Disabilita modalità interattiva e mostra il plot finale
        plt.ioff()
        plt.show()

    def animate_plot_map_with_target_and_trajectory(self, point_xyz, trajectory_xyz):
        x_points = np.array([point['position'][0] for point in self.points_t])
        y_points = np.array([point['position'][1] for point in self.points_t])
        z_points = np.array([point['position'][2] for point in self.points_t])
        color = np.array([point['color'] for point in self.points_t])
        size_point = np.array([point['size_point'] for point in self.points_t])
        
        self.ax.clear()
        
        # Calculate equal axis scaling including trajectory points
        all_x = [x_points]
        all_y = [y_points]
        all_z = [z_points]
        
        for traj in trajectory_xyz:
            if traj is not None and traj.size > 0:
                all_x.append(traj[0, :])
                all_y.append(traj[1, :])
                all_z.append(traj[2, :])
        
        all_x = np.concatenate(all_x + [[p[0] for p in point_xyz]])
        all_y = np.concatenate(all_y + [[p[1] for p in point_xyz]])
        all_z = np.concatenate(all_z + [[p[2] for p in point_xyz]])
        
        max_range = np.array([
            all_x.max() - all_x.min(),
            all_y.max() - all_y.min(),
            all_z.max() - all_z.min()
        ]).max() / 2.0
        
        mid_x = (all_x.max() + all_x.min()) * 0.5
        mid_y = (all_y.max() + all_y.min()) * 0.5
        mid_z = (all_z.max() + all_z.min()) * 0.5
        
        self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
        self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
        self.ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        self.ax.set_xlabel('X (m) - Height')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Point Cloud with Target Points - Animated')
        self.ax.scatter(x_points, y_points, z_points, c=color, s=size_point, alpha=0.6)

        for i, point in enumerate(point_xyz):
            #print(f"Adding target point {i+1}/{len(point_xyz)}: {point}")
            if i == 0:  
                marker_color = 'green'
                marker = 'X'
                point_name = "Start Point"
            elif i == len(point_xyz) - 1:  
                marker_color = 'red'
                marker = 'X'
                point_name = "End Point"
            else:
                marker_color = 'blue'
                marker = 'X'
                point_name = f"Waypoint {i}"
            
            self.ax.scatter(point[0], point[1], point[2],
                    c=marker_color, s=100, marker=marker, 
                    label=point_name if i < 3 else None) 
            
            ref_com = np.array(trajectory_xyz[i])
            self.ax.plot3D(ref_com[0,:], ref_com[1,:],ref_com[2,:],color=marker_color, linewidth=2.5)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        self.ax.set_title(f'Point Cloud with Target Points - Complete Path\n'
                    f'Total points: {len(point_xyz)}')
        
        self.ax.legend()
        plt.tight_layout()
        # Disabilita modalità interattiva e mostra il plot finale

        plt.show()
        # delay tra ogni punto
        time.sleep(3)
        
    def plot_color_cost_given_cost(self, source_points=None):
        if source_points is None:
            source_points = self.points_t
        
        cost_values = np.array([point['cost'] for point in source_points])
        cost_min, cost_max = np.min(cost_values), np.max(cost_values)
        if cost_max == cost_min:
            for point in source_points:
                point['color'] = (0.0, 0.0, 1.0)  # blu
            return

        normalized_costs = (cost_values - cost_min) / (cost_max - cost_min)

        cmap = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        gradient_colors = cmap(normalized_costs)

        for i, point in enumerate(source_points):
            point['color'] = gradient_colors[i][:3]
            
        # print(f"Colored {len(source_points)} points based on cost values")
        # print(f"Cost range: {cost_min:.3f} to {cost_max:.3f}")
    
    def plot_cost_map_3d(self, source_points=None, point_size=20, alpha=0.85, elev=15, azim=-30):
        '''
        Plot only the 3D cost map with equal axis scaling.
        point_size: size of each point in the scatter plot (increase for thicker cloud)
        alpha: transparency of the points
        '''
        if source_points is None:
            source_points = self.points_t
        x_points = np.array([point['position'][0] for point in source_points])
        y_points = np.array([point['position'][1] for point in source_points])
        z_points = np.array([point['position'][2] for point in source_points])
        cost_values = np.array([point['cost'] for point in source_points])
        point_colors = np.array([point['color'] for point in source_points])

        fig = plt.figure(figsize=(10, 8), dpi=150)
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(
            x_points, y_points, z_points,
            c=point_colors,
            s=point_size,
            alpha=alpha,
            depthshade=True
        )

        # Equal axis scaling
        all_pts = np.column_stack([x_points, y_points, z_points])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

        ax.set_xlabel('X (m) - Height', fontsize=20, labelpad=10)
        ax.set_ylabel('Y (m)', fontsize=20, labelpad=10)
        ax.set_zlabel('Z (m)', fontsize=20, labelpad=10)
        
        # --- CODICE AGGIORNATO QUI ---
        ax.tick_params(axis='both', which='major', labelsize=15)  # x and y
        ax.tick_params(axis='z', which='major', labelsize=15)     # z axis
        # -----------------------------

        # ax.set_title('3D Cost Map', fontsize=13)
        ax.view_init(elev=elev, azim=azim)
        cmap_cost = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        sm = plt.cm.ScalarMappable(cmap=cmap_cost, norm=plt.Normalize(vmin=np.min(cost_values), vmax=np.max(cost_values)))
        sm.set_array([])
        cbar = fig.colorbar(sm, ax=ax, shrink=0.5, pad=0.1)
        cbar.set_label('Cost', fontsize=15)
        cbar.ax.tick_params(labelsize=12)
        plt.tight_layout()
        plt.show()

    def plot_cost_map_2d(self, source_points=None, point_size=40, alpha=0.85, figsize=(10, 8)):
        '''
        Plot only the 2D top-down cost map with large fonts, no title.
        '''
        if source_points is None:
            source_points = self.points_t

        y_points = np.array([point['position'][1] for point in source_points])
        z_points = np.array([point['position'][2] for point in source_points])
        point_colors = np.array([point['color'] for point in source_points])

        fig, ax = plt.subplots(figsize=figsize, dpi=150)

        ax.scatter(
            y_points, z_points,
            c=point_colors,
            s=point_size,
            alpha=alpha
        )

        ax.set_xlabel('Y (m)', fontsize=20, labelpad=10)
        ax.set_ylabel('Z (m)', fontsize=20, labelpad=10)
        ax.tick_params(axis='both', which='major', labelsize=15)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.3)
        cost_values = np.array([point['cost'] for point in source_points])
        cmap_cost = LinearSegmentedColormap.from_list("green_yellow_red", ["green", "yellow", "red"])
        sm = plt.cm.ScalarMappable(cmap=cmap_cost, norm=plt.Normalize(vmin=np.min(cost_values), vmax=np.max(cost_values)))
        sm.set_array([])
        cbar = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Cost', fontsize=15)
        cbar.ax.tick_params(labelsize=12)
        plt.tight_layout()
        plt.show()


    def plot_filter_response_heatmap(self, surface, grid_y, grid_z, point_size=20, alpha=0.85, figsize=(10, 8)):
        '''
        Plot only the 2D heatmap of the filter response (from ax1 of visualize_filter_operation).
        The colormap is remapped to stop at yellow instead of white.
        point_size: not used here but kept for consistency
        alpha: transparency of the heatmap
        figsize: tuple (width, height) of the figure
        '''
        # Custom colormap: black -> red -> yellow (stops before white)
        cmap_hot_yellow = LinearSegmentedColormap.from_list(
            "hot_yellow",
            ["black", "red", "yellow"],
            N=256
        )

        fig, ax = plt.subplots(figsize=figsize)

        im = ax.imshow(
            surface,
            extent=[np.min(grid_y), np.max(grid_y), np.min(grid_z), np.max(grid_z)],
            origin='lower',
            cmap=cmap_hot_yellow,
            aspect='auto',
            alpha=alpha,
            vmin=np.min(surface),
            vmax=np.max(surface)
        )

        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Filter Response Intensity', fontsize=12)
        cbar.ax.tick_params(labelsize=10)

        ax.set_xlabel('Y (m)', fontsize=12, labelpad=8)
        ax.set_ylabel('Z (m)', fontsize=12, labelpad=8)
        ax.set_title('Filter Response Heatmap', fontsize=13)
        ax.tick_params(axis='both', labelsize=10)
        ax.grid(True, alpha=0.2, linestyle='--')

        plt.tight_layout()
        plt.show()
    
    
    def plot_filter_response_2d(self, source_points=None, point_size=40, alpha=0.85, figsize=(10, 8)):
        '''
        Plot the 2D Filter Response Map (top-down heatmap from the interpolated surface)
        in a single figure with large fonts and no title.
        Call this after visualize_filter_operation or after compute_conv_step.
        '''
        if self.surface is None:
            # Try to recompute: need at least the last convolution result
            raise RuntimeError("Surface not computed. Call compute_conv_step() or visualize_filter_operation() first (without resetting self.surface).")

        if self.grid_y is None or self.grid_z is None:
            raise RuntimeError("Grid not initialized. Call interpolation_to_surface() first.")

        # Custom colormap: black -> red -> yellow
        cmap_hot_yellow = LinearSegmentedColormap.from_list(
            "hot_yellow",
            ["black", "red", "yellow"],
            N=256
        )

        fig, ax = plt.subplots(figsize=figsize, dpi=150)

        im = ax.imshow(
            self.surface,
            extent=[np.min(self.grid_y), np.max(self.grid_y),
                    np.min(self.grid_z), np.max(self.grid_z)],
            origin='lower',
            cmap=cmap_hot_yellow,
            aspect='equal',
            alpha=alpha,
            vmin=np.min(self.surface),
            vmax=np.max(self.surface)
        )

        ax.set_xlabel('Y (m)', fontsize=20, labelpad=10)
        ax.set_ylabel('Z (m)', fontsize=20, labelpad=10)
        ax.tick_params(axis='both', which='major', labelsize=15)
        ax.grid(True, alpha=0.2, linestyle='--')
        cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Filter Response Intensity', fontsize=15)
        cbar.ax.tick_params(labelsize=12)
        plt.tight_layout()
        plt.show()

    
    def plot_filter_response_pointcloud_3d(self, source_points=None, point_size=20, alpha=0.85, figsize=(10, 8), elev=15, azim=-30):
        '''
        Plot the 3D point cloud colored by filter response (ax2 of visualize_filter_operation).
        Colormap stops at yellow instead of white.
        point_size: size of each point
        alpha: transparency
        figsize: tuple (width, height)
        '''
        if source_points is None:
            source_points = self.points_t

        x_points = np.array([point['position'][0] for point in source_points])
        y_points = np.array([point['position'][1] for point in source_points])
        z_points = np.array([point['position'][2] for point in source_points])

        # Remap filter response colors using hot_yellow colormap
        # Re-interpolate surface values at point positions to get scalar response
        if self.grid_Y is None or self.grid_Z is None or self.surface is None:
            raise RuntimeError("Surface not computed. Call interpolation_to_surface() and convolution_process() first.")

        # Custom colormap: black -> red -> yellow
        cmap_hot_yellow = LinearSegmentedColormap.from_list(
            "hot_yellow",
            ["black", "red", "yellow"],
            N=256
        )

        # Get filter response value at each point
        response_at_points = griddata(
            (self.grid_Y.flatten(), self.grid_Z.flatten()),
            self.surface.flatten(),
            (y_points, z_points),
            method="linear",
            fill_value=0.0,
        )

        # Normalize [0, 1] for colormap
        r_min, r_max = np.min(response_at_points), np.max(response_at_points)
        if r_max > r_min:
            normalized = (response_at_points - r_min) / (r_max - r_min)
        else:
            normalized = np.zeros_like(response_at_points)

        point_colors = cmap_hot_yellow(normalized)[:, :3]

        fig = plt.figure(figsize=figsize, dpi=150)
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(
            x_points, y_points, z_points,
            c=point_colors,
            s=point_size,
            alpha=alpha,
            depthshade=True
        )

        # Equal axis scaling
        all_pts = np.column_stack([x_points, y_points, z_points])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

        ax.set_xlabel('X (m)', fontsize=20, labelpad=10)
        ax.set_ylabel('Y (m)', fontsize=20, labelpad=10)
        ax.set_zlabel('Z (m)', fontsize=20, labelpad=10)
        ax.tick_params(axis='both', labelsize=15)
        # ax.set_title('Height Map',fontsize=20, pad=-100)
        ax.view_init(elev=elev, azim=azim)
        # Add colorbar via ScalarMappable
        sm = plt.cm.ScalarMappable(cmap=cmap_hot_yellow, norm=plt.Normalize(vmin=r_min, vmax=r_max))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax, fraction=0.03, pad=0.1)
        cbar.set_label('Filter Response Intensity', fontsize=15)
        cbar.ax.tick_params(labelsize=12)

        plt.tight_layout()
        plt.show()


        
def main():
    
    # Terrain configuration values
    # wall_depth = 1            
    # grid_size = 100
    # max_ridge_depth = 0.5     
    # seed = 30                 
    # Lz = -50                  
    # Ly = 10                   
    # terrain  = TerrainManager(grid_size=100,wall_depth =10,max_ridge_depth=0.5, seed="default", Lz=-10, Ly=10, generate_terrain=True, terrain_type="custom_gaussians")
    # terrain = TerrainManager()
    terrain  = TerrainManager(grid_size=100,wall_depth =3,max_ridge_depth=0.5, seed="default", Lz=-10, Ly=10, generate_terrain=True, terrain_type="rock")

    pc = terrain.point_cloud
    # Point cloud filter test
    pc_filter = PointCloudFilter(pc, h_min=0.0, h_max=4.0)
    
    print("\n[TEST] === Original Map ===")
    # pc_filter.print_map_pc()
    
    #filtro con cancellazione punti
    print("\n[TEST] === Height Filter ===")
    # pc_filter.filter_height()
    # pc_filter.print_map_pc()
    
    print("\n[TEST] === Logarithmic Height Cost Filter ===")    
    # filtro con cambio di costo e colore in base all'altezza
    # pc_filter.filter_height_profile(x0=0.0, scale=1.0,side_application="depth", profile="logln")
    # pc_filter.visualize_cost_map()
    
    # pc_filter.plot_cost_map_2d(point_size=80, alpha=0.6)
    # print("\n[TEST] === Smoothing Filter ===")
    # kernel = [pc_filter.smoothing_kernel] 
    # pc_filter.filter_process_points_pipeline(kernel, weight=1.0, plot=True)
    
    print("\n[TEST] === First Derivative (Gradient) ==")
    kernel = [pc_filter.sobel_y, pc_filter.sobel_z] 
    pc_filter.filter_process_points_pipeline(kernel,weight=1.0, plot=True)
    
    # print("\n[TEST] === Second Derivative (Laplacian) ===")
    # kernel = [pc_filter.laplacian_kernel] 
    # pc_filter.filter_process_points_pipeline(kernel, plot=True)
    
    # print("\n[TEST] === Difference of Gaussians (DoG) ===")
    
    # pc_filter.compute_bump_detection(weight=2.0)
    
    print("\n[TEST] === Laplacian of Gaussian (LoG) ===")
    # kernel = [pc_filter.log_kernel] 
    # pc_filter.filter_process_points_pipeline(kernel, plot=True)
    # # Default point size
    pc_filter.plot_cost_map_3d()

    # # Con punti più grandi
    # pc_filter.plot_cost_map_3d(point_size=50)

    # # Con punti molto grandi e più trasparenti
    # pc_filter.plot_cost_map_3d(point_size=80, alpha=0.6)

if __name__ == "__main__":
    main()