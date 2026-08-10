from terrain_manager import TerrainManager
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage
from matplotlib.colors import LinearSegmentedColormap
import time


class PointCloudFilter3D:
    """
    3D convolution-based point cloud filter for terrain morphology analysis.
    Instead of projecting to a 2D surface, the point cloud is voxelized into
    a 3D occupancy/height grid, and 3D kernels are applied to extract
    morphological features (gradient, curvature, edges).
    """

    def __init__(self, pc, voxel_resolution=0.1, h_min=None, h_max=None):
        N = len(pc)
        self.pc = pc
        self.voxel_resolution = voxel_resolution

        # Point attributes
        self.color = np.full((N, 3), [0.0, 0.0, 0.0])
        self.size_point = np.ones(N) * 4
        self.cost = np.ones(N) * -1.0
        self.light = np.ones(N) * 0.8

        self.points_t = [
            {
                'position': self.pc[i],
                'color': self.color[i],
                'light': self.light[i],
                'size_point': self.size_point[i],
                'cost': self.cost[i],
            }
            for i in range(N)
        ]

        self.x_points = np.array([p['position'][0] for p in self.points_t])
        self.y_points = np.array([p['position'][1] for p in self.points_t])
        self.z_points = np.array([p['position'][2] for p in self.points_t])

        self.h_min = h_min
        self.h_max = h_max

        # Voxel grid cache
        self.voxel_grid = None
        self.origin = None  # (x_min, y_min, z_min)
        self.grid_shape = None
        self.point_voxel_indices = None  # (N, 3) voxel index per point

        self._init_3d_kernels()
        self._print_information()

    # ================================================================
    # Kernel definitions
    # ================================================================
    def _init_3d_kernels(self):
        """Define 3D convolution kernels for morphology extraction."""

        # --- 3D Blur (box) ---
        self.blur_kernel_3d = np.ones((3, 3, 3)) / 27.0

        # --- 3D Gaussian-like smoothing ---
        # A simple 3x3x3 approximation
        g = np.array([[[1, 2, 1],
                       [2, 4, 2],
                       [1, 2, 1]],
                      [[2, 4, 2],
                       [4, 8, 4],
                       [2, 4, 2]],
                      [[1, 2, 1],
                       [2, 4, 2],
                       [1, 2, 1]]], dtype=float)
        self.gaussian_kernel_3d = g / g.sum()

        # --- 3D Sobel kernels (gradient along each axis) ---
        # Sobel X (axis-0 = height)
        self.sobel_x_3d = np.array([
            [[-1, -2, -1],
             [-2, -4, -2],
             [-1, -2, -1]],
            [[ 0,  0,  0],
             [ 0,  0,  0],
             [ 0,  0,  0]],
            [[ 1,  2,  1],
             [ 2,  4,  2],
             [ 1,  2,  1]]
        ], dtype=float)

        # Sobel Y (axis-1)
        self.sobel_y_3d = np.array([
            [[-1, -2, -1],
             [ 0,  0,  0],
             [ 1,  2,  1]],
            [[-2, -4, -2],
             [ 0,  0,  0],
             [ 2,  4,  2]],
            [[-1, -2, -1],
             [ 0,  0,  0],
             [ 1,  2,  1]]
        ], dtype=float)

        # Sobel Z (axis-2)
        self.sobel_z_3d = np.array([
            [[-1,  0,  1],
             [-2,  0,  2],
             [-1,  0,  1]],
            [[-2,  0,  2],
             [-4,  0,  4],
             [-2,  0,  2]],
            [[-1,  0,  1],
             [-2,  0,  2],
             [-1,  0,  1]]
        ], dtype=float)

        # --- 3D Laplacian ---
        # 6-connectivity laplacian
        self.laplacian_3d_6 = np.array([
            [[ 0,  0,  0],
             [ 0,  1,  0],
             [ 0,  0,  0]],
            [[ 0,  1,  0],
             [ 1, -6,  1],
             [ 0,  1,  0]],
            [[ 0,  0,  0],
             [ 0,  1,  0],
             [ 0,  0,  0]]
        ], dtype=float)

        # 26-connectivity laplacian
        lap26 = np.ones((3, 3, 3), dtype=float)
        lap26[1, 1, 1] = -26.0
        self.laplacian_3d_26 = lap26

        # --- 3D Laplacian of Gaussian (LoG) approximation 5x5x5 ---
        # Built via: LoG = Laplacian(Gaussian)
        # We generate it programmatically
        self.log_kernel_3d = self._build_log_kernel_3d(size=5, sigma=0.8)

    @staticmethod
    def _build_log_kernel_3d(size=5, sigma=0.8):
        """Build a discrete 3D Laplacian-of-Gaussian kernel."""
        half = size // 2
        ax = np.arange(-half, half + 1, dtype=float)
        xx, yy, zz = np.meshgrid(ax, ax, ax, indexing='ij')
        r2 = xx**2 + yy**2 + zz**2
        s2 = sigma**2
        # LoG formula
        kernel = -(1.0 / (np.pi * s2**2)) * (1.0 - r2 / (2.0 * s2)) * np.exp(-r2 / (2.0 * s2))
        kernel -= kernel.mean()  # zero-sum
        return kernel

    # ================================================================
    # Height filter
    # ================================================================
    def filter_height(self):
        """Remove points outside [h_min, h_max]."""
        if self.h_min is None and self.h_max is None:
            return
        h_min = self.h_min if self.h_min is not None else -np.inf
        h_max = self.h_max if self.h_max is not None else np.inf
        x_pts = np.array([p['position'][0] for p in self.points_t])
        mask = (x_pts >= h_min) & (x_pts <= h_max)
        self.points_t = [p for i, p in enumerate(self.points_t) if mask[i]]
        self._refresh_coords()

    # ================================================================
    # Voxelization
    # ================================================================
    def voxelize(self, source_points=None, value_mode='occupancy'):
        """
        Convert point cloud into a 3D voxel grid.
        value_mode:
            'occupancy' – binary (1 where points exist)
            'density'   – count of points per voxel
            'height'    – mean x (height) of points in each voxel
        """
        pts = source_points if source_points is not None else self.points_t
        x = np.array([p['position'][0] for p in pts])
        y = np.array([p['position'][1] for p in pts])
        z = np.array([p['position'][2] for p in pts])

        res = self.voxel_resolution
        x_min, y_min, z_min = x.min(), y.min(), z.min()
        x_max, y_max, z_max = x.max(), y.max(), z.max()

        self.origin = np.array([x_min, y_min, z_min])

        # Voxel indices for each point
        ix = ((x - x_min) / res).astype(int)
        iy = ((y - y_min) / res).astype(int)
        iz = ((z - z_min) / res).astype(int)

        nx = int((x_max - x_min) / res) + 1
        ny = int((y_max - y_min) / res) + 1
        nz = int((z_max - z_min) / res) + 1
        self.grid_shape = (nx, ny, nz)

        # Clamp
        ix = np.clip(ix, 0, nx - 1)
        iy = np.clip(iy, 0, ny - 1)
        iz = np.clip(iz, 0, nz - 1)

        self.point_voxel_indices = np.stack([ix, iy, iz], axis=1)

        grid = np.zeros(self.grid_shape, dtype=float)

        if value_mode == 'occupancy':
            grid[ix, iy, iz] = 1.0
        elif value_mode == 'density':
            np.add.at(grid, (ix, iy, iz), 1.0)
        elif value_mode == 'height':
            count = np.zeros_like(grid)
            np.add.at(grid, (ix, iy, iz), x)
            np.add.at(count, (ix, iy, iz), 1.0)
            valid = count > 0
            grid[valid] /= count[valid]
        else:
            raise ValueError(f"Unknown value_mode: {value_mode}")

        self.voxel_grid = grid
        print(f"[point_cloud_filter_3d] Voxelized: grid shape = {self.grid_shape}, "
              f"resolution = {res:.3f} m, "
              f"occupied voxels = {int(np.sum(grid > 0))}")
        return grid

    # ================================================================
    # 3D Convolution
    # ================================================================
    def convolve_3d(self, grid, kernels, mode='nearest'):
        """
        Apply one or more 3D kernels and return the response magnitude.
        kernels: list of 3D numpy arrays.
          - 1 kernel  → |conv|
          - 2+ kernels → sqrt(sum of squares)
        """
        if len(kernels) == 1:
            response = ndimage.convolve(grid, kernels[0], mode=mode)
            return response
        else:
            responses = [ndimage.convolve(grid, k, mode=mode) for k in kernels]
            magnitude = np.sqrt(sum(r**2 for r in responses))
            return magnitude

    # ================================================================
    # Map voxel response back to points
    # ================================================================
    def voxel_response_to_points(self, response, source_points=None):
        """
        For each point, read the convolution response from the corresponding voxel.
        Returns an array of response values (one per point).
        """
        pts = source_points if source_points is not None else self.points_t
        if self.point_voxel_indices is None:
            raise RuntimeError("Call voxelize() first.")

        N = len(pts)
        values = np.zeros(N)
        for i in range(N):
            ix, iy, iz = self.point_voxel_indices[i]
            values[i] = response[ix, iy, iz]

        return values

    # ================================================================
    # Compute cost from response
    # ================================================================
    def compute_cost(self, response_values, source_points=None, weight=1.0):
        """Incremental cost accumulation from convolution response values."""
        pts = source_points if source_points is not None else self.points_t
        cost_values = np.abs(response_values)

        for i, p in enumerate(pts):
            old_cost = p['cost']
            p['cost'] = old_cost + float(cost_values[i]) * weight

        self._color_by_cost(pts)

    # ================================================================
    # Full pipeline
    # ================================================================
    def pipeline(self, kernels, source_points=None, weight=1.0,
                 value_mode='height', plot=False):
        """
        Full 3D morphology pipeline:
          1. Voxelize
          2. 3D convolution
          3. Map response to points
          4. Compute cost
          5. (Optional) plot
        """
        pts = source_points if source_points is not None else self.points_t

        # 1. Voxelize
        grid = self.voxelize(pts, value_mode=value_mode)

        # 2. Convolve
        response = self.convolve_3d(grid, kernels)
        print(f"[point_cloud_filter_3d] Convolution response range: "
              f"[{response.min():.4f}, {response.max():.4f}]")

        # 3. Map to points
        values = self.voxel_response_to_points(response, pts)

        # 4. Cost
        self.compute_cost(values, pts, weight=weight)

        if plot:
            self.visualize_voxel_response(response)
            self.visualize_cost_map(pts)

        return values

    def pipeline_gradient_3d(self, source_points=None, weight=1.0,
                             value_mode='height', plot=False):
        """Convenience: 3D Sobel gradient magnitude."""
        kernels = [self.sobel_x_3d, self.sobel_y_3d, self.sobel_z_3d]
        return self.pipeline(kernels, source_points, weight, value_mode, plot)

    def pipeline_laplacian_3d(self, source_points=None, weight=1.0,
                              connectivity=6, value_mode='height', plot=False):
        """Convenience: 3D Laplacian (curvature)."""
        kernel = self.laplacian_3d_6 if connectivity == 6 else self.laplacian_3d_26
        return self.pipeline([kernel], source_points, weight, value_mode, plot)

    def pipeline_log_3d(self, source_points=None, weight=1.0,
                        value_mode='height', plot=False):
        """Convenience: 3D Laplacian of Gaussian."""
        return self.pipeline([self.log_kernel_3d], source_points, weight,
                             value_mode, plot)

    def pipeline_smooth_3d(self, source_points=None, weight=0.0,
                           value_mode='height', plot=False):
        """Convenience: 3D Gaussian smoothing (useful as preprocessing)."""
        return self.pipeline([self.gaussian_kernel_3d], source_points, weight,
                             value_mode, plot)

    # ================================================================
    # Height profile cost (same idea as 2D version)
    # ================================================================
    def filter_height_profile(self, source_points=None, profile="exponential",
                              x0=0.0, scale=0.5, side_application="depth",
                              weight=1.0):
        """Apply a height-based cost profile to the points."""
        pts = source_points if source_points is not None else self.points_t
        x_pts = np.array([p['position'][0] for p in pts])
        eps = 1e-8

        if side_application == "both":
            x = np.abs(x_pts - x0)
        elif side_application == "up":
            x = np.where(x_pts >= x0, x_pts - x0, 0.0)
        elif side_application == "depth":
            x = np.where(x_pts <= x0, x0 - x_pts, 0.0)
        else:
            raise ValueError("side_application must be 'both', 'up', or 'depth'")

        if profile == "linear_positive":
            cv = np.clip(x / (scale + eps), 0.0, 1.0)
        elif profile == "linear_negative":
            cv = np.clip(1.0 - x / (scale + eps), 0.0, 1.0)
        elif profile == "logln":
            cv = -(np.log(1.0 + x / (scale + eps)) / (x + eps))
        elif profile == "exponential":
            cv = 1.0 - np.exp(-x / (scale + eps))
        else:
            raise ValueError(f"Unknown profile: {profile}")

        cv = cv - 1.0  # shift like the 2D version

        for i, p in enumerate(pts):
            p['cost'] = p['cost'] + float(cv[i]) * weight

        self._color_by_cost(pts)

    # ================================================================
    # Auxiliary / getters
    # ================================================================
    def _refresh_coords(self):
        self.x_points = np.array([p['position'][0] for p in self.points_t])
        self.y_points = np.array([p['position'][1] for p in self.points_t])
        self.z_points = np.array([p['position'][2] for p in self.points_t])

    def get_all_cost(self):
        return np.array([p['cost'] for p in self.points_t])

    def get_serializable_points(self):
        return [
            {
                'position': p['position'].tolist() if hasattr(p['position'], 'tolist') else list(p['position']),
                'color': p['color'].tolist() if hasattr(p['color'], 'tolist') else list(p['color']),
                'light': float(p['light']),
                'size_point': float(p['size_point']),
                'cost': float(p['cost']),
            }
            for p in self.points_t
        ]

    # ================================================================
    # Coloring
    # ================================================================
    def _color_by_cost(self, pts=None):
        if pts is None:
            pts = self.points_t
        costs = np.array([p['cost'] for p in pts])
        cmin, cmax = costs.min(), costs.max()
        if cmax == cmin:
            for p in pts:
                p['color'] = np.array([0.0, 0.0, 1.0])
            return
        norm = (costs - cmin) / (cmax - cmin)
        cmap = LinearSegmentedColormap.from_list("g_y_r", ["green", "yellow", "red"])
        colors = cmap(norm)
        for i, p in enumerate(pts):
            p['color'] = colors[i][:3]

    # ================================================================
    # Visualization
    # ================================================================
    def visualize_voxel_response(self, response, threshold_pct=0.3):
        """
        Visualize the 3D convolution response as a scatter of voxels
        above a threshold percentage of the max value.
        """
        abs_resp = np.abs(response)
        threshold = abs_resp.max() * threshold_pct
        idx = np.argwhere(abs_resp > threshold)
        if len(idx) == 0:
            print("[point_cloud_filter_3d] No voxels above threshold for visualization.")
            return

        vals = abs_resp[idx[:, 0], idx[:, 1], idx[:, 2]]
        # Map voxel indices back to world coordinates
        world = idx.astype(float) * self.voxel_resolution + self.origin

        norm_vals = (vals - vals.min()) / (vals.max() - vals.min() + 1e-12)
        colors = plt.cm.hot(norm_vals)

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(world[:, 0], world[:, 1], world[:, 2],
                   c=vals, cmap='hot', s=8, alpha=0.6)

        ax.set_xlabel('X (m) – Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'3D Convolution Response (voxels > {threshold_pct*100:.0f}% of max)')
        self._set_equal_axes(ax, world)
        fig.colorbar(sc, ax=ax, shrink=0.5, label='Response magnitude')
        plt.tight_layout()
        plt.show()

    def visualize_cost_map(self, source_points=None):
        pts = source_points if source_points is not None else self.points_t
        x = np.array([p['position'][0] for p in pts])
        y = np.array([p['position'][1] for p in pts])
        z = np.array([p['position'][2] for p in pts])
        colors = np.array([p['color'] for p in pts])
        sizes = np.array([p['size_point'] for p in pts])
        costs = np.array([p['cost'] for p in pts])

        fig = plt.figure(figsize=(16, 8))

        # 3D view
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        sc1 = ax1.scatter(x, y, z, c=costs, cmap=LinearSegmentedColormap.from_list("g_y_r", ["green", "yellow", "red"]),
                          s=sizes, alpha=0.8)
        ax1.set_xlabel('X (m) – Height')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Cost Map\n(Green=Low, Red=High)')
        self._set_equal_axes(ax1, np.column_stack([x, y, z]))
        fig.colorbar(sc1, ax=ax1, shrink=0.5, label='Cost')

        # Top-down
        ax2 = fig.add_subplot(1, 2, 2)
        sc2 = ax2.scatter(y, z, c=costs, cmap=LinearSegmentedColormap.from_list("g_y_r", ["green", "yellow", "red"]),
                          s=sizes * 2, alpha=0.8)
        ax2.set_xlabel('Y (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('Top-Down Cost Map')
        ax2.set_aspect('equal', adjustable='datalim')
        ax2.grid(True, alpha=0.3)
        fig.colorbar(sc2, ax=ax2, label='Cost')

        plt.tight_layout()
        plt.show()

        print(f"[point_cloud_filter_3d] Cost stats: "
              f"min={costs.min():.3f}, max={costs.max():.3f}, "
              f"mean={costs.mean():.3f}, std={costs.std():.3f}")

    def print_map_pc(self, pts=None):
        if pts is None:
            pts = self.points_t
        x = np.array([p['position'][0] for p in pts])
        y = np.array([p['position'][1] for p in pts])
        z = np.array([p['position'][2] for p in pts])
        colors = np.array([p['color'] for p in pts])
        sizes = np.array([p['size_point'] for p in pts])

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(x, y, z, c=colors, s=sizes)
        self._set_equal_axes(ax, np.column_stack([x, y, z]))
        ax.set_xlabel('X (m) – Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Point Cloud ({len(pts)} points)')
        plt.tight_layout()
        plt.show()

    @staticmethod
    def _set_equal_axes(ax, pts):
        """Set equal axis scaling for 3D plot with a minimum range guard."""
        ranges = np.ptp(pts, axis=0)
        max_range = max(ranges.max() / 2.0, 0.5)  # minimum half-range of 0.5 m
        mid = pts.mean(axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
        # Force equal aspect ratio (matplotlib >= 3.3)
        try:
            ax.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass

    # ================================================================
    # Coloring
    # ================================================================
    def _color_by_cost(self, pts=None):
        if pts is None:
            pts = self.points_t
        costs = np.array([p['cost'] for p in pts])
        cmin, cmax = costs.min(), costs.max()
        if cmax == cmin:
            for p in pts:
                p['color'] = np.array([0.0, 0.0, 1.0])
            return
        norm = (costs - cmin) / (cmax - cmin)
        cmap = LinearSegmentedColormap.from_list("g_y_r", ["green", "yellow", "red"])
        colors = cmap(norm)
        for i, p in enumerate(pts):
            p['color'] = colors[i][:3]

    # ================================================================
    # Visualization
    # ================================================================
    def visualize_voxel_response(self, response, threshold_pct=0.3):
        """
        Visualize the 3D convolution response as a scatter of voxels
        above a threshold percentage of the max value.
        """
        abs_resp = np.abs(response)
        threshold = abs_resp.max() * threshold_pct
        idx = np.argwhere(abs_resp > threshold)
        if len(idx) == 0:
            print("[point_cloud_filter_3d] No voxels above threshold for visualization.")
            return

        vals = abs_resp[idx[:, 0], idx[:, 1], idx[:, 2]]
        # Map voxel indices back to world coordinates
        world = idx.astype(float) * self.voxel_resolution + self.origin

        norm_vals = (vals - vals.min()) / (vals.max() - vals.min() + 1e-12)
        colors = plt.cm.hot(norm_vals)

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(world[:, 0], world[:, 1], world[:, 2],
                   c=vals, cmap='hot', s=8, alpha=0.6)

        ax.set_xlabel('X (m) – Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'3D Convolution Response (voxels > {threshold_pct*100:.0f}% of max)')
        self._set_equal_axes(ax, world)
        fig.colorbar(sc, ax=ax, shrink=0.5, label='Response magnitude')
        plt.tight_layout()
        plt.show()

    def visualize_cost_map(self, source_points=None):
        pts = source_points if source_points is not None else self.points_t
        x = np.array([p['position'][0] for p in pts])
        y = np.array([p['position'][1] for p in pts])
        z = np.array([p['position'][2] for p in pts])
        colors = np.array([p['color'] for p in pts])
        sizes = np.array([p['size_point'] for p in pts])
        costs = np.array([p['cost'] for p in pts])

        fig = plt.figure(figsize=(16, 8))

        # 3D view
        ax1 = fig.add_subplot(1, 2, 1, projection='3d')
        sc1 = ax1.scatter(x, y, z, c=costs, cmap=LinearSegmentedColormap.from_list("g_y_r", ["green", "yellow", "red"]),
                          s=sizes, alpha=0.8)
        ax1.set_xlabel('X (m) – Height')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Cost Map\n(Green=Low, Red=High)')
        self._set_equal_axes(ax1, np.column_stack([x, y, z]))
        fig.colorbar(sc1, ax=ax1, shrink=0.5, label='Cost')

        # Top-down
        ax2 = fig.add_subplot(1, 2, 2)
        sc2 = ax2.scatter(y, z, c=costs, cmap=LinearSegmentedColormap.from_list("g_y_r", ["green", "yellow", "red"]),
                          s=sizes * 2, alpha=0.8)
        ax2.set_xlabel('Y (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('Top-Down Cost Map')
        ax2.set_aspect('equal', adjustable='datalim')
        ax2.grid(True, alpha=0.3)
        fig.colorbar(sc2, ax=ax2, label='Cost')

        plt.tight_layout()
        plt.show()

        print(f"[point_cloud_filter_3d] Cost stats: "
              f"min={costs.min():.3f}, max={costs.max():.3f}, "
              f"mean={costs.mean():.3f}, std={costs.std():.3f}")

    def print_map_pc(self, pts=None):
        if pts is None:
            pts = self.points_t
        x = np.array([p['position'][0] for p in pts])
        y = np.array([p['position'][1] for p in pts])
        z = np.array([p['position'][2] for p in pts])
        colors = np.array([p['color'] for p in pts])
        sizes = np.array([p['size_point'] for p in pts])

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(x, y, z, c=colors, s=sizes)
        self._set_equal_axes(ax, np.column_stack([x, y, z]))
        ax.set_xlabel('X (m) – Height')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Point Cloud ({len(pts)} points)')
        plt.tight_layout()
        plt.show()

    @staticmethod
    def _set_equal_axes(ax, pts):
        """Set equal axis scaling for 3D plot with a minimum range guard."""
        ranges = np.ptp(pts, axis=0)
        max_range = max(ranges.max() / 2.0, 0.5)  # minimum half-range of 0.5 m
        mid = pts.mean(axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
        # Force equal aspect ratio (matplotlib >= 3.3)
        try:
            ax.set_box_aspect([1, 1, 1])
        except AttributeError:
            pass

    def _print_information(self):
        print(f"[point_cloud_filter_3d] Point cloud statistics:")
        print(f"  Total points: {len(self.pc)}")
        print(f"  X (height) range: [{self.x_points.min():.2f}, {self.x_points.max():.2f}] m")
        print(f"  Y range: [{self.y_points.min():.2f}, {self.y_points.max():.2f}] m")
        print(f"  Z range: [{self.z_points.min():.2f}, {self.z_points.max():.2f}] m")
        print(f"  Voxel resolution: {self.voxel_resolution} m")


# ====================================================================
# MAIN
# ====================================================================
def main():

    terrain = TerrainManager(
        grid_size=100,
        wall_depth=10,
        max_ridge_depth=0.5,
        seed="default",
        Lz=-20,
        Ly=10,
        generate_terrain=True,
        terrain_type="rock",
    )

    pc = terrain.point_cloud

    # Create the 3D filter
    pcf3d = PointCloudFilter3D(pc, voxel_resolution=0.15, h_min=0.5, h_max=5.0)

    # 0. Show original point cloud
    print("\n=== [STEP 0] Original Point Cloud ===")
    pcf3d.print_map_pc()

    # # 1. (Optional) Height filter – remove points outside range
    # print("\n=== [STEP 1] Height Filter ===")
    # pcf3d.filter_height()
    # pcf3d.print_map_pc()

    # 2. Height-profile cost
    print("\n=== [STEP 2] Height-Profile Cost (exponential) ===")
    pcf3d.filter_height_profile(x0=1.0, scale=1.0,
                                 side_application="depth",
                                 profile="exponential",
                                 weight=1.0)
    pcf3d.visualize_cost_map()

    # 3. 3D Gaussian smoothing (preprocessing, weight=0 → no cost added)
    print("\n=== [STEP 3] 3D Gaussian Smoothing (preprocessing) ===")
    pcf3d.pipeline_smooth_3d(value_mode='height', weight=1.0, plot=True)

    # 4. 3D Gradient (Sobel) – first derivative → slope
    print("\n=== [STEP 4] 3D Gradient (Sobel) – Slope Detection ===")
    pcf3d.pipeline_gradient_3d(value_mode='height', weight=10.0, plot=True)

    # 5. 3D Laplacian – second derivative → curvature
    print("\n=== [STEP 5] 3D Laplacian – Curvature Detection ===")
    pcf3d.pipeline_laplacian_3d(connectivity=6, value_mode='height',
                                 weight=10.0, plot=True)

    # 6. 3D LoG – edge / ridge detection
    print("\n=== [STEP 6] 3D Laplacian of Gaussian (LoG) – Edge Detection ===")
    pcf3d.pipeline_log_3d(value_mode='height', weight=0.5, plot=True)

    # Final cost map
    print("\n=== [FINAL] Accumulated Cost Map ===")
    pcf3d.visualize_cost_map()

    costs = pcf3d.get_all_cost()
    print(f"\nFinal cost array shape: {costs.shape}")
    print(f"Cost range: [{costs.min():.4f}, {costs.max():.4f}]")


if __name__ == "__main__":
    main()
