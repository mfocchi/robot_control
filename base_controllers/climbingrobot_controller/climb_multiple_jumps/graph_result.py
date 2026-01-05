import json
from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from scipy.interpolate import griddata

# === CONFIGURATION ===
FILE_TERRAIN = "result/actual_terrain.json"
FILE_PROGRESS = "result/top_100_progress.json"
FILENAME = "result/top_100_progress.json"


class PlotResultCemMjumps:
    def __init__(self, dir_path_terrain, dir_path_progress, dir_path_final):
        
        self.data_terrain = self.load_json(dir_path_terrain)
        self.data_progress = self.load_json(dir_path_progress)
        self.data_final = self.load_json(dir_path_final)
        
        # Terrain data
        self.px = None
        self.py = None
        self.pz = None
        self.points_np = None
        self.colors_np = None
        self._extract_terrain_data()
        
        # Solution data (from progress)
        self.top_solutions = []
        self.iterations = []
        self.fitnesses = []
        self.n_jumps = []
        self.energies = []
        self.landing_costs = []
        self.all_points = []  # List of point lists per solution
        self._extract_solution_data()
        
        # Derived data for 3D plotting
        self.starts_np = None
        self.ends_np = None
        self.intermediates_np = None
        self._extract_trajectory_points()
    
    def _extract_terrain_data(self):
        """Extract terrain point cloud data."""
        if self.data_terrain is None or 'point_cloud' not in self.data_terrain:
            print("WARNING: No valid terrain data found.")
            return
        
        point_list = self.data_terrain['point_cloud']
        points = []
        colors = []
        for p in point_list:
            points.append(p["position"])
            colors.append(p["color"])
        
        self.points_np = np.array(points)
        self.colors_np = np.array(colors)
        self.px = self.points_np[:, 0]
        self.py = self.points_np[:, 1]
        self.pz = self.points_np[:, 2]
    
    def _extract_solution_data(self):
        """Extract solution data from progress JSON."""
        if self.data_progress is None or 'top_100_solutions' not in self.data_progress:
            print("WARNING: No valid progress data found.")
            return
        
        self.top_solutions = self.data_progress['top_100_solutions']
        
        for s in self.top_solutions:
            self.iterations.append(s['iteration'])
            self.fitnesses.append(s['fitness'])
            self.n_jumps.append(s['n_jumps'])
            self.energies.append(s['consumed_energy'])
            self.landing_costs.append(s['landing_cost'])
            self.all_points.append(s['points'])
    
    def _extract_trajectory_points(self):
        """Extract start, end, and intermediate points from solutions."""
        starts = []
        ends = []
        intermediates = []
        
        for pts in self.all_points:
            if not pts:
                continue
            starts.append(pts[0])
            ends.append(pts[-1])
            if len(pts) > 2:
                intermediates.extend(pts[1:-1])
        
        self.starts_np = np.array([starts[0]]) if starts else np.array([])
        self.ends_np = np.array([ends[0]]) if ends else np.array([])
        self.intermediates_np = np.array(intermediates) if intermediates else np.array([])
        
    def load_json(self, filename):
        """Load JSON file and handle errors."""
        if not os.path.exists(filename):
            print(f"ERROR: File '{filename}' does not exist.")
            return None
        with open(filename, 'r') as f:
            return json.load(f)

    def plot_actual_terrain(self, ax=None):
        if self.points_np is None:
            print("ERROR: No terrain data available.")
            return
        
        print(f"plot {len(self.points_np)} terrain points")
        
        created_fig = False
        if ax is None:
            created_fig = True
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        ax.scatter(
            self.px,
            self.py,
            self.pz,
            c=self.colors_np,
            s=10
        )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        
        if created_fig:
            plt.show()

    def plot_fitness_by_iteration(self, ax=None):
        """
        graph 1: Fitness of top 100 solutions vs Iteration.
        """
        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111)
            created_fig = True
        else:
            fig = ax.figure
        
        scatter = ax.scatter(self.iterations, self.fitnesses, c=self.fitnesses, cmap='viridis', alpha=0.8, edgecolors='k', s=60)
        fig.colorbar(scatter, ax=ax, label='Fitness Value')
    
        ax.set_title('Fitness Distribution vs Iteration', fontsize=14)
        ax.set_xlabel('Iteration where solution was found', fontsize=12)
        ax.set_ylabel('Fitness', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.5)
        
        # Highlight the best solution
        best_idx = np.argmax(self.fitnesses)
        ax.annotate(f'Best: {self.fitnesses[best_idx]:.2f}', 
                    xy=(self.iterations[best_idx], self.fitnesses[best_idx]), 
                    xytext=(self.iterations[best_idx], self.fitnesses[best_idx] + (max(self.fitnesses)-min(self.fitnesses))*0.05),
                    arrowprops=dict(facecolor='red', shrink=0.05))

        fig.tight_layout()
        
        if created_fig:            
            fig.savefig('result/plot_1_fitness_iterazione.png')
            print("Plot 1 saved: result/plot_1_fitness_iterazione.png")
            plt.show()
        
    def plot_jumps_histogram(self, ax=None):
        """
        Graph 2: Histogram that represent which type of jumps is most used.
        """
        bins = np.arange(min(self.n_jumps) - 0.5, max(self.n_jumps) + 1.5, 1)
        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111)
            created_fig = True
        else:
            fig = ax.figure

        ax.hist(self.n_jumps, bins=bins, color='skyblue', edgecolor='black', alpha=0.7, rwidth=0.8)
        
        ax.set_title('Frequency of Number of Jumps (Top 100 Solutions)', fontsize=14)
        ax.set_xlabel('Total Number of Jumps', fontsize=12)
        ax.set_ylabel('Solution Count', fontsize=12)
        ax.set_xticks(range(min(self.n_jumps), max(self.n_jumps) + 1))
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        
        fig.tight_layout()
        
        if created_fig:    
            fig.savefig('result/plot_2_istogramma_salti.png')
            print("Plot 2 saved: result/plot_2_istogramma_salti.png")
            plt.show()
        
    def plot_energy_vs_cost(self, ax=None):
        """
        Graph 3: Consumed Energy vs Landing Cost
        """
        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111)
            created_fig = True
        else:
            fig = ax.figure
        
        scatter = ax.scatter(self.energies, self.landing_costs, c=self.fitnesses, cmap='plasma', s=50, alpha=0.8, edgecolors='gray')
        cbar = fig.colorbar(scatter)
        cbar.set_label('Total Fitness')
        
        ax.set_title('Trade-off: Consumed Energy vs. Landing Cost', fontsize=14)
        ax.set_xlabel('Consumed Energy [J]', fontsize=12)
        ax.set_ylabel('Landing Cost (Terrain)', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.5)
        
        fig.tight_layout()
        
        if created_fig:
            fig.savefig('result/plot_3_energy_vs_cost.png')
            print("Plot 3 saved: result/plot_3_energy_vs_cost.png")
            plt.show()
    
    def plot_3d_scenario(self, ax=None):    
        """
        Plot 4: Complete 3D visualization.
        """
        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')
            created_fig = True
        else:
            fig = ax.figure
        
        ax.scatter(
            self.px,
            self.py,
            self.pz,
            c='gray',
            s=2,
            alpha=0.30,
            label='Terrain Cloud'
        )
        
        if self.starts_np.size > 0:
            ax.scatter(self.starts_np[0,0], self.starts_np[0,1], self.starts_np[0,2], c='lime', s=100, marker='^', label='Start P0', depthshade=False)
        
        if self.ends_np.size > 0:
            ax.scatter(self.ends_np[0,0], self.ends_np[0,1], self.ends_np[0,2], c='red', s=100, marker='X', label='Goal PF', depthshade=False)
            
        if self.intermediates_np.size > 0:
            ax.scatter(self.intermediates_np[:,0], self.intermediates_np[:,1], self.intermediates_np[:,2], c='blue', s=20, marker='o', alpha=0.6, label='Jump Points')
            
        ax.set_title(f"Jump Visualization on Terrain", fontsize=14)
        ax.set_xlabel('X (Height/Horizontal)')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        
        ax.view_init(elev=25, azim=-45)
        
        plt.tight_layout()
        if created_fig:
            plt.savefig('result/plot_3d_jumps_map.png', dpi=150)
            print("Saved: result/plot_3d_jumps_map.png")
            plt.show()
    
    def plot_density_map(self, ax=None):
        """
        Plot 3D:
        1. Original colored point cloud.
        2. Density heatmap computed on YZ plane.
        """        
        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')
            created_fig = True
        else:
            fig = ax.figure

        ax.scatter(self.px, self.py, self.pz, c=self.colors_np, s=2, alpha=0.3, label='Terrain RGB')
        
        # Extract jump points 
        jump_y = []
        jump_z = []
        jump_x_vals = [] 
        
        for pts in self.all_points:
            
            if len(pts) > 2:
                mid_pts = pts[1:-1]
                for p in mid_pts:
                    jump_x_vals.append(p[0])
                    jump_y.append(p[1])
                    jump_z.append(p[2])
        
        if not jump_y:
            print("Warning: No intermediate jumps found for heatmap.")
            return
        
        y_min, y_max = min(self.py), max(self.py)
        z_min, z_max = min(self.pz), max(self.pz)
        
        grid_res = 100
        yi = np.linspace(y_min, y_max, grid_res)
        zi = np.linspace(z_min, z_max, grid_res)
        Yi, Zi = np.meshgrid(yi, zi)
        H, yedges, zedges = np.histogram2d(jump_y, jump_z, bins=50,
                                             range=[[y_min, y_max], [z_min, z_max]])
        ycenters = (yedges[:-1] + yedges[1:]) / 2
        zcenters = (zedges[:-1] + zedges[1:]) / 2
        Yc, Zc = np.meshgrid(ycenters, zcenters)
        density = griddata((Yc.flatten(), Zc.flatten()), H.T.flatten(), 
                           (Yi, Zi), method='cubic', fill_value=0)
        x_base = min(self.px) - (max(self.px) - min(self.px)) * 0.05
        Xi = np.full_like(Yi, x_base)
        
            
        # Plot projected surface
        surf = ax.plot_surface(Xi, Yi, Zi, 
                            facecolors=plt.cm.inferno(density / density.max()),
                            alpha=0.6, shade=False, rstride=5, cstride=5)
        
        # 6. Plot jump points (bright yellow)
        ax.scatter(jump_x_vals, jump_y, jump_z, c='gold', s=20, 
                marker='o', alpha=1.0, label='Jump Points', edgecolors='white', linewidths=0.5)
        
        # Colorbar
        m = plt.cm.ScalarMappable(cmap='inferno')
        m.set_array(density)
        cbar = plt.colorbar(m, ax=ax, pad=0.1, shrink=0.7)
        cbar.set_label('Jump Density (Freq on YZ)', fontsize=10)
        
        # Labels and view
        ax.set_title("Jump Density Map (YZ Plane) + Colored Terrain", fontsize=14)
        ax.set_xlabel('X (Depth/Height)')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        # Set a view that allows seeing the YZ plane
        # Elevation 20, Azimuth 0 or 180 often work well to see YZ as "front"
        ax.view_init(elev=20, azim=10) 
        
        plt.tight_layout()
        if created_fig:
            outfile = 'result/plot_yz_density_colored.png'
            plt.savefig(outfile, dpi=150)
            print(f"Saved: {outfile}")
            plt.show()
                                    
       
    
    def plot_all_in_one(self):
        fig = plt.figure(figsize=(16,12))
        
        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224, projection='3d')
        
        
        sc1=self.plot_fitness_by_iteration(ax1)
        sc2=self.plot_jumps_histogram(ax2)
        sc3=self.plot_energy_vs_cost(ax3)
        sc4=self.plot_3d_scenario(ax4)
        
        fig.colorbar(sc1, ax=ax2, label='Fitness Value')
        
        
        plt.tight_layout()
        plt.show()
        
        



# === MAIN ===
def main():
    plot_result_cem_mjumps = PlotResultCemMjumps(FILE_TERRAIN, FILE_PROGRESS, FILENAME)
    print("ok all is ready")
    plot_result_cem_mjumps.plot_actual_terrain()
    plot_result_cem_mjumps.plot_all_in_one()
    plot_result_cem_mjumps.plot_density_map()
    
if __name__ == "__main__":
    main()