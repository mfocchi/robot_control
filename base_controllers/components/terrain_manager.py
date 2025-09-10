import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
import cv2
from scipy.signal import convolve2d
from scipy.interpolate import RegularGridInterpolator

class TerrainManager:
    
    def __init__(self, grid_size=100,wall_depth =1,max_ridge_depth=0.5, seed=47, Lz=-60, Ly=10):
        
        # INPUT VARIABLES
        self.wall_depth = wall_depth
        self.grid_size = grid_size
        self.max_ridge_depth = max_ridge_depth
        self.seed = seed
        self.Lz = Lz  # Length in Z direction (vertical extent) of wall in meters
        self.Ly = Ly  # Length in Y direction (horizontal extent) of wall in meters
        
        # other parameters
        self.debug = True
        self.number_of_patches_width = 10
        self.number_of_patches_height = 10

        self.number_of_patches = self.number_of_patches_width*self.number_of_patches_height
        self.patch_origins = [ np.zeros((2))] * self.number_of_patches #top left corner in absolute coordinates assuming

        self.patch_discretization_width = 20
        self.patch_discretization_height = 20
        self.number_of_points_in_patch = self.patch_discretization_width *self.patch_discretization_height
        
        # Generate the terrain automatically
        self.mesh_x, self.mesh_y, self.mesh_z = self.generate_rock_wall_map(
            self.Lz, self.Ly, self.grid_size, self.wall_depth, 
            self.max_ridge_depth, self.seed
        )
        
        # Convert to point cloud format and store
        self.point_cloud = self.convert_meshgrid_to_pc(self.mesh_x, self.mesh_y, self.mesh_z)

    def generate_rock_wall_map(self, Lz, Ly, grid_size=100, wall_depth=2, max_ridge_depth=0.5, seed=None, x_offset = 0):
        """
        Generate a 3D rock wall height map with fractal noise, ridges, and pillars.

        Parameters:
        -----------
        Lz : float
            Length in Z direction (meters)
        Ly : float
            Length in Y direction (meters)
        grid_size : int, optional
            Size of height map (default: 100)
        wall_depth : float, optional
            Maximum wall depth/height (default: 2)
        max_ridge_depth : float, optional
            Maximum ridge depth (default: 0.5)
        seed : int, str, optional
            Random seed for reproducibility. Can be int, "default", or None (default: None)
        debug : bool, optional
            Whether to display debug plot (default: True)

        Returns:
        --------
        X : ndarray
            Height map array
        Y : ndarray
            Y coordinate meshgrid
        Z : ndarray
            Z coordinate meshgrid
        """
        # Set random seed - handle "default" string like MATLAB
        if seed == "default" or seed is None:
            np.random.seed(47)
        else:
            np.random.seed(int(seed))

        # 1) Fractal noise / multi-scale Perlin-like structures for roughness
        frequencies = [1, 2, 4]
        weights = [0.5, 0.25, 0.15, 0.1]

        # Initialize height map
        X = np.zeros((grid_size, grid_size))

        # Create smooth kernel (equivalent to MATLAB's fspecial('gaussian', [15, 15], 3))
        smooth_kernel_size = 15
        sigma = 3

        for i, scale in enumerate(frequencies):
            # Generate noise at different scales
            noise_size = max(1, grid_size // scale)
            noise = np.random.rand(noise_size, noise_size).T #MATLAB fills out a matrix down columns, while python goes down rows. So in order to get the same matrices in both, you have to transpose:

            # Resize noise to grid size (equivalent to MATLAB's imresize with bilinear)
            noise_resized = cv2.resize(noise, (grid_size, grid_size), interpolation=cv2.INTER_LINEAR)

            # Apply Gaussian smoothing (equivalent to MATLAB's imfilter with gaussian kernel)
            smoothed_noise = gaussian_filter(noise_resized, sigma=sigma, mode='nearest')

            # Add weighted contribution
            if i < len(weights):
                X += weights[i] * smoothed_noise

        # Normalize and scale to realistic wall height
        X = X - np.min(X)
        if np.max(X) > 0:
            X = X / np.max(X) * wall_depth

        # 2) Ridges / dihedrals with directional high gradients
        # Create vertical ridge
        ridge_start = int(grid_size * 0.3)
        ridge_end = min(ridge_start + 15, grid_size)
        X[ridge_start:ridge_end, :] = X[ridge_start:ridge_end, :] + max_ridge_depth

        # 3) Pillars
        num_pillars = 10
        for i in range(num_pillars):
            cz = np.random.randint(0, grid_size)
            cy = np.random.randint(0, grid_size)
            # Create meshgrid for pillar calculation
            Z_mesh, Y_mesh = np.meshgrid(np.arange(grid_size), np.arange(grid_size))

            # Random radius for pillar
            radius = np.random.randint(5, 11)  # randint is exclusive of upper bound

            # Create Gaussian bulge for pillar
            bulge = np.exp(-((Z_mesh - cz) ** 2 + (Y_mesh - cy) ** 2) / (2 * radius ** 2))

            # Add pillar to height map
            X = X + bulge * wall_depth

        X = X + x_offset


        # Create physical grid in meters
        z = np.linspace(Lz, 0, grid_size)
        y = np.linspace(0, Ly, grid_size)
        Z, Y = np.meshgrid(z, y)

        #patches
        self.patch_width = Ly / self.number_of_patches_width
        self.patch_height = Lz / self.number_of_patches_height
        patch_id = 0
        for i in range(self.number_of_patches_width):
            for j in range(self.number_of_patches_height):
                self.patch_origins[patch_id] = np.array([self.patch_width*i,self.patch_height*j]) 
                patch_id +=1
        
        return X, Y, Z
    
    def convert_meshgrid_to_pc(self, X, Y, Z):
        x_position = X.flatten()
        y_position = Y.flatten()
        z_position = Z.flatten()
        points = np.vstack((x_position, y_position, z_position)).T
        return points
            
    def getAbsolutePositionOfPointInsidePatch(self, patch_id,  normalized_y, normalized_z):
        pos_yz = self.patch_origins[patch_id] + np.array([normalized_y * self.patch_width, normalized_z* self.patch_height])
        pos = np.concatenate(([0.], pos_yz))
        return pos

    def retrievePatches(self, patch_id): #points are columns
        y_vec = np.linspace(0, self.patch_width, self.patch_discretization_width)
        z_vec = np.linspace(0, self.patch_height, self.patch_discretization_height)
        points_in_patch = self.patch_origins[patch_id].reshape(2,1) + np.vstack((y_vec,z_vec))
        return points_in_patch

    def wall_surface_eval(self, z_query, y_query, mesh_x, mesh_y, mesh_z):

        """
         Evaluate the wall surface height at given query points using interpolation.

         Parameters:
         -----------
         z_query : float or array-like
             Z coordinate(s) where surface height is to be evaluated
         y_query : float or array-like
             Y coordinate(s) where surface height is to be evaluated
        'mesh_x': Height map array (surface heights)
        'mesh_y': Y coordinate meshgrid
        'mesh_z': Z coordinate meshgrid

         Returns:
         --------
         val : float or ndarray
             Surface height value(s) at the query point(s)
         """

        # Extract coordinate arrays from meshgrids
        z_coords = mesh_z[0, :]  # Z coordinates (1D array)
        y_coords = mesh_y[:, 0]  # Y coordinates (1D array)

        # Create interpolation function using RegularGridInterpolator
        # Note: RegularGridInterpolator expects (y, z) order for 2D data
        wall_surface_fcn = RegularGridInterpolator(
            (y_coords, z_coords),
            mesh_x,
            method='linear',
            bounds_error=False,
            fill_value=0
        )

        # Handle both scalar and array inputs
        if np.isscalar(z_query) and np.isscalar(y_query):
            # Single point query
            query_points = np.array([[y_query, z_query]])
            val = wall_surface_fcn(query_points)[0]
        else:
            # Multiple points query
            z_query = np.asarray(z_query)
            y_query = np.asarray(y_query)

            # Ensure same shape for broadcasting
            if z_query.shape != y_query.shape:
                z_query, y_query = np.meshgrid(z_query, y_query)

            # Flatten for interpolation
            z_flat = z_query.flatten()
            y_flat = y_query.flatten()
            query_points = np.column_stack([y_flat, z_flat])

            val = wall_surface_fcn(query_points)

            # Reshape back to original query shape if needed
            if z_query.ndim > 1:
                val = val.reshape(z_query.shape)

        return val

    def wall_normal_eval(self, z_query, y_query, mesh_x, mesh_y, mesh_z):

        """
         Compute the normal vector to a wall surface at given query points.

         Parameters:
         -----------
         z_query : float or array-like
             Z coordinate(s) where normal is to be evaluated
         y_query : float or array-like  
             Y coordinate(s) where normal is to be evaluated
         params : dict
             Dictionary containing mesh data with keys:
             - 'mesh_x': Height map array
             - 'mesh_y': Y coordinate meshgrid
             - 'mesh_z': Z coordinate meshgrid

         Returns:
         --------
         normal : ndarray
             Unit normal vector [nx, ny, nz] at the query point(s)
         """

        # 1) Compute gradients
        # Note: meshgrid varies x as the SECOND index
        # We need to be careful as the horizontal coordinate of an array is the second one

        # Calculate grid spacing
        dz = mesh_z[0, 1] - mesh_z[0, 0]
        dy = mesh_y[1, 0] - mesh_y[0, 0]

        # Define convolution kernels for gradients
        # df/dz = (F(y, z + dz) - F(y, z - dz))/(2*dz)
        # df/dy = (F(y + dy, z) - F(y - dy, z))/(2*dy)
        kernelz = np.array([[-1, 0, 1]]) / (2 * dz)
        kernely = np.array([[1], [0], [-1]]) / (2 * dy)

        # Compute gradients using convolution with 'valid' mode
        dx_dz = convolve2d(mesh_x, kernelz, mode='valid')
        dx_dy = convolve2d(mesh_x, kernely, mode='valid')

        # Pad edges properly to maintain original dimensions
        # Left and right edges for dz gradient
        left_edge = convolve2d(mesh_x[:, 0:2], np.array([[-1, 1]]) / dz, mode='valid')
        right_edge = convolve2d(mesh_x[:, -2:], np.array([[-1, 1]]) / dz, mode='valid')
        dx_dz = np.concatenate([left_edge, dx_dz, right_edge], axis=1)

        # Top and bottom edges for dy gradient  
        top_edge = convolve2d(mesh_x[0:2, :], np.array([[-1], [1]]) / dy, mode='valid')
        bottom_edge = convolve2d(mesh_x[-2:, :], np.array([[-1], [1]]) / dy, mode='valid')
        dx_dy = np.concatenate([top_edge, dx_dy, bottom_edge], axis=0)

        # Create interpolation functions for gradients
        # Using RegularGridInterpolator for better performance and consistency
        z_coords = mesh_z[0, :]  # Z coordinates (1D)
        y_coords = mesh_y[:, 0]  # Y coordinates (1D)

        # Note: RegularGridInterpolator expects (y, z) order for 2D data
        Fz_interp = RegularGridInterpolator((y_coords, z_coords), dx_dz,
                                            method='linear', bounds_error=False, fill_value=0)
        Fy_interp = RegularGridInterpolator((y_coords, z_coords), dx_dy,
                                            method='linear', bounds_error=False, fill_value=0)

        # 2) Compute normal
        # Ensure query points are in the correct format for interpolation
        if np.isscalar(z_query) and np.isscalar(y_query):
            # Single point query
            query_points = np.array([[y_query, z_query]])
            dz_val = Fz_interp(query_points)[0]
            dy_val = Fy_interp(query_points)[0]

            # Compute normal vector
            normal = np.array([1, -dy_val, -dz_val])
            normal = normal / np.linalg.norm(normal)

        else:
            # Multiple points query
            z_query = np.asarray(z_query)
            y_query = np.asarray(y_query)

            # Ensure same shape
            if z_query.shape != y_query.shape:
                z_query, y_query = np.meshgrid(z_query, y_query)

            # Flatten for interpolation
            z_flat = z_query.flatten()
            y_flat = y_query.flatten()
            query_points = np.column_stack([y_flat, z_flat])

            dz_val = Fz_interp(query_points)
            dy_val = Fy_interp(query_points)

            # Compute normal vectors
            normal = np.column_stack([np.ones_like(dz_val), -dy_val, -dz_val])

            # Normalize each normal vector
            norms = np.linalg.norm(normal, axis=1, keepdims=True)
            normal = normal / norms

            # Reshape back to original query shape if needed
            if z_query.ndim > 1:
                normal = normal.reshape((*z_query.shape, 3))

        return normal

    def custom_random_generator(self, seed, n):
        x = seed
        # Parameters
        a = 1664525;
        c = 1013904223;
        m = 2 ** 32;
        seed = 1234

        result = []
        for _ in range(n):
            x = (a * x + c) % m
            result.append((x % 100) + 1)
        return result

    def set_mesh(self, mesh):
        import open3d as o3d
        self.mesh = mesh
        # Cleanup the mesh
        self.mesh.remove_degenerate_triangles()  # Remove zero-area triangles
        self.mesh.remove_duplicated_triangles()  # Remove duplicate faces
        self.mesh.remove_duplicated_vertices()  # Remove duplicate vertices
        self.mesh.remove_non_manifold_edges()  # Fix non-manifold edges
        self.mesh.orient_triangles()  # If the mesh is orientable this function orients all triangles such that all normals point towards the same direction.
        self.mesh.compute_vertex_normals()  # Recompute normals
        self.mesh.compute_triangle_normals()
        # self.mesh.normalize_normals()
        self.triangle_mesh = o3d.t.geometry.TriangleMesh.from_legacy(self.mesh)

        # define scene
        self.scene = o3d.t.geometry.RaycastingScene()
        # returns the ID for the added geometry
        self.scene.add_triangles(self.triangle_mesh)

    def visualize_normals(self, triangles, vertices, normals, length= 1.):
        import open3d as o3d
        lines = []
        colors = []
        new_vertices = []
        for i, triangle in enumerate(triangles):
            v0, v1, v2 = triangle

            # Compute the center of the triangle (face)
            triangle_center = (vertices[v0] + vertices[v1] + vertices[v2]) / 3.0
            normal_start = triangle_center
            normal_end = triangle_center + normals[i] * length

            new_vertices.append(normal_start)
            new_vertices.append(normal_end)

            lines.append([2 * i, 2 * i + 1])
            colors.extend([[1, 0, 0]])  # Red color for normals
        new_vertices = np.array(new_vertices)
        lines = np.array(lines)

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(new_vertices)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)

        return line_set

    def create_ramp_mesh(self, length, width, inclination=0., origin=np.array([0,0,0])):
        """
        Create a pitch ramp mesh with the specified length, width, and height.

        Parameters:
            length (float): The horizontal length of the ramp.
            width (float): The width of the ramp.
            height (float): The height of the ramp's inclined plane.

        Returns:
            open3d.geometry.TriangleMesh: The ramp mesh.
        """
        # Define the vertices of the pitch ramp
        vertices = [
            [ -length/2,width/2,  length/2*math.tan(inclination)],  # Bottom-left corner
            [ -length/2, -width/2, length/2*math.tan(inclination)],  # Bottom-right corner
            [length/2, -width/2, -length/2*math.tan(inclination)],  # Top-right corner
            [length/2, width/2,  -length/2*math.tan(inclination)]  # Top-left triangle
        ]


        # Define the triangles (faces) of the ramp
        triangles = [
            [0, 1, 2],  # Bottom-right triangle
            [0, 2, 3]  # Top-left triangle
        ]

        # Create the mesh
        ramp_mesh = o3d.geometry.TriangleMesh()
        ramp_mesh.vertices = o3d.utility.Vector3dVector(vertices)
        ramp_mesh.triangles = o3d.utility.Vector3iVector(triangles)

        # Compute normals for better visualization
        ramp_mesh.compute_vertex_normals()

        return ramp_mesh

    def plot_terrain_map(self, X, Y, Z):
        title='Generated Rock Wall Height Map'
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Surface plot
        ax.plot_surface(X, Y, Z, linewidth=2, alpha=1)
        ax.set_title(title)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.view_init(elev=20, azim=9)
        plt.show()

    def plot_patch_and_center(self, X, Y, Z, normalized_y=0.5, normalized_z=0.5):
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X, Y, Z, linewidth=1, alpha=0.7, color='yellow')
        # ax.plot_surface(X, Y, Z, linewidth=1, alpha=0.7, cmap='terrain')
        
        for patch_id in range(self.number_of_patches):
            # Get the position inside the patch
            position = self.getPositionInsidePatch(patch_id, normalized_y, normalized_z)
            pos_x, pos_y, pos_z = position[0], position[1], position[2]
            
            # Get the actual surface height at this position
            surface_height = self.wall_surface_eval(pos_z, pos_y, X, Y, Z)
            
            # Highlight the specific patch boundaries
            patch_y_min = self.patch_origins[patch_id][0]
            patch_y_max = patch_y_min + self.patch_width
            patch_z_min = self.patch_origins[patch_id][1]
            patch_z_max = patch_z_min + self.patch_height
            
            # Create patch boundary points
            patch_y_coords = [patch_y_min, patch_y_max, patch_y_max, patch_y_min, patch_y_min]
            patch_z_coords = [patch_z_min, patch_z_min, patch_z_max, patch_z_max, patch_z_min]
            patch_x_coords =  [self.wall_surface_eval(pz, py, X, Y, Z) for py, pz in zip(patch_y_coords, patch_z_coords)]
            
            # Plot patch outline
            ax.plot(patch_x_coords, patch_y_coords, patch_z_coords, 'b-', linewidth=2)

            # Plot the center point
            ax.scatter([surface_height], [pos_y], [pos_z], color='red', s=50)

        # Add labels and formatting
        ax.set_title(f'Patch {patch_id} - Position Visualization')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.view_init(elev=20, azim=9)
        
        plt.show()
    
    def plot_point_cloud(self, X, Y, Z, plot_patches=True, plot_patch_boundaries=False):
        subsample_factor=1
        title='Terrain Point Cloud'
        # Create figure
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        patch_x = []
        patch_y = []
        patch_z = []
        # Subsample the data if needed (for better performance with large grids)
        if subsample_factor > 1:
            X_sub = X[::subsample_factor, ::subsample_factor]
            Y_sub = Y[::subsample_factor, ::subsample_factor]
            Z_sub = Z[::subsample_factor, ::subsample_factor]
        else:
            X_sub = X
            Y_sub = Y
            Z_sub = Z
        
        # Flatten the arrays to get individual points
        x_points = X_sub.flatten()
        y_points = Y_sub.flatten()
        z_points = Z_sub.flatten()
        
        # Create point cloud plot
        scatter = ax.scatter(x_points, y_points, z_points, 
                           s=20,  # Point size
                           alpha=0.7,
                           color='blue')  # Single color for all points
        
        if plot_patches or plot_patch_boundaries:
            for patch_id in range(self.number_of_patches):
                # Get patch center position
                position = self.getPositionInsidePatch(patch_id, 0.5, 0.5)
                pos_y, pos_z = position[1], position[2]
                
                # Get surface height at center
                surface_height = self.wall_surface_eval(pos_z, pos_y, X, Y, Z)
                
                if plot_patches:
                    patch_x.append(surface_height)
                    patch_y.append(pos_y)
                    patch_z.append(pos_z)
                
                # Plot patch boundaries (same as plot_patch_and_center)
                if plot_patch_boundaries:
                    # Get patch boundaries
                    patch_y_min = self.patch_origins[patch_id][0]
                    patch_y_max = patch_y_min + self.patch_width
                    patch_z_min = self.patch_origins[patch_id][1]
                    patch_z_max = patch_z_min + self.patch_height
                    
                    # Create patch boundary points
                    patch_y_coords = [patch_y_min, patch_y_max, patch_y_max, patch_y_min, patch_y_min]
                    patch_z_coords = [patch_z_min, patch_z_min, patch_z_max, patch_z_max, patch_z_min]
                    patch_x_coords = [self.wall_surface_eval(pz, py, X, Y, Z) for py, pz in zip(patch_y_coords, patch_z_coords)]
                    
                    # Plot patch outline (same as plot_patch_and_center)
                    ax.plot(patch_x_coords, patch_y_coords, patch_z_coords, 'b-', linewidth=2, alpha=0.8)
            
            # Plot patch centers
            if plot_patches and patch_x:
                ax.scatter(patch_x, patch_y, patch_z, 
                          c='red', s=100, alpha=0.9, 
                          marker='o', label='Patch centers')
        
        # Set labels and title
        ax.set_xlabel('X (m) - Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(title)
        
        # Set view angle
        ax.view_init(elev=20, azim=45)
        
        # Print statistics
        print(f"Point cloud statistics:")
        print(f"  Total points: {len(x_points)}")
        print(f"  Height range: {np.min(x_points):.2f} to {np.max(x_points):.2f} m")
        print(f"  Y range: {np.min(y_points):.2f} to {np.max(y_points):.2f} m")
        print(f"  Z range: {np.min(z_points):.2f} to {np.max(z_points):.2f} m")
        
        plt.tight_layout()
        plt.show()
           
    def plot_debug (self, debug=True):
        if debug:
            # self.plot_terrain_map( X,Y,Z)
            self.plot_patch_and_center(self.mesh_x,  self.mesh_y, self.mesh_z )
            self.plot_point_cloud(self.mesh_x,  self.mesh_y, self.mesh_z, plot_patches=True, plot_patch_boundaries=True)
        else:
            print("Debug mode is off. No visualization will be shown.")

            
if __name__ == '__main__':
    
    wall_depth = 1  # how
    grid_size = 50
    max_ridge_depth = 0.5
    seed = 47
    Lz = -60  # Height of wall in meters
    Ly = 10  # Width (horizontal extent) of wall in meters
  
    terrainManager = TerrainManager(grid_size, wall_depth=wall_depth, max_ridge_depth=max_ridge_depth, seed=seed, Lz=Lz, Ly=Ly)
    
    p0 = np.array([0.0, 2.5, -6])
    p0[0] = terrainManager.wall_surface_eval(p0[2], p0[1], terrainManager.mesh_x, terrainManager.mesh_y, terrainManager.mesh_z)
    print(p0)
    normal = terrainManager.wall_normal_eval(p0[2], p0[1],  terrainManager.mesh_x, terrainManager.mesh_y, terrainManager.mesh_z)
    print(normal)
    
    terrainManager.plot_debug(debug=True)
    
    
    

