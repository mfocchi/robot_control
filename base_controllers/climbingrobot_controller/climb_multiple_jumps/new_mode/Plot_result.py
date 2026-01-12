import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import colors
import numpy as np
import os
from scipy.interpolate import griddata
from scipy.spatial import ConvexHull 
from params import *
import matplotlib
matplotlib.use('Qt5Agg')


class PlotResultCemMjumps:
    
    def __init__(self):
        self.iterations_folder = ITERATIONS_FOLDER
        
        # Load all JSON files
        self.data_terrain_points = self._load_json(FILE_TERRAIN_POINTS)
        self.data_terrain_patches = self._load_json(FILE_TERRAIN_PATCHES)
        self.data_progress = self._load_json(FILE_PROGRESS)
        self.data_best_log = self._load_json(FILE_BEST_LOG)
        
        # Initialize data containers
        self.px = self.py = self.pz = None
        self.points_np = self.colors_np = None
        self.points_t = []
        self.patches_list = []
        self.edge_patches = []
        
        # Solution data
        self.iterations = self.fitnesses = self.n_jumps = []
        self.energies = self.landing_costs = []
        self.all_points = []
        
        # Trajectory data
        self.best_points_np = None
        self.best_traj_segments = []
        self.starts_np = self.ends_np = self.intermediates_np = None
        
        # Extract all data
        self._extract_terrain_data()
        self._extract_solution_data()
        self._extract_best_trajectory_data()
        self._extract_trajectory_points()
    
    def _load_json(self, filename):
        """Load JSON file with error handling."""
        if not os.path.exists(filename):
            print(f"ERROR: File '{filename}' not found.")
            return None
        with open(filename, 'r') as f:
            return json.load(f)
    
    def _extract_terrain_data(self):
        """Extract terrain points and patches."""
        # Extract points
        if self.data_terrain_points and 'points' in self.data_terrain_points:
            point_list = self.data_terrain_points['points']
            positions, colors = [], []
            
            for p in point_list:
                p_dict = {
                    'position': p.get('position'),
                    'color': p.get('color'),
                    'light': p.get('light', 1.0),
                    'size_point': p.get('size_point', 1.0),
                    'cost': p.get('cost', 0.0)
                }
                self.points_t.append(p_dict)
                positions.append(p_dict['position'])
                colors.append(p_dict['color'])
            
            self.points_np = np.array(positions)
            self.colors_np = np.array(colors)
            if self.points_np.size > 0:
                self.px, self.py, self.pz = self.points_np[:, 0], self.points_np[:, 1], self.points_np[:, 2]
            
            self.start_pos_terrain = self.data_terrain_points.get('start_position')
            self.goal_pos_terrain = self.data_terrain_points.get('goal_position')
        
        # Extract patches
        if self.data_terrain_patches and 'patches' in self.data_terrain_patches:
            meta = self.data_terrain_patches.get('metadata', {})
            self.patch_width = meta.get('patch_width')
            self.patch_height = meta.get('patch_height')
            self.num_patches = meta.get('num_patches')
            
            for patch in self.data_terrain_patches.get('patches', []):
                points_in_patch = patch.get('points_in_patch', [])
                mean_cost = np.mean([pt.get('cost', 0.0) for pt in points_in_patch]) if points_in_patch else 0.0
                
                self.patches_list.append({
                    'id': patch.get('id'),
                    'centroid': patch.get('centroid'),
                    'points_in_patch': points_in_patch,
                    'cost_patch': mean_cost
                })

    def _extract_solution_data(self):
        """Extract solution data from iteration files."""
        if not os.path.exists(self.iterations_folder):
            print(f"WARNING: Iterations folder '{self.iterations_folder}' not found.")
            return
        
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        
        data_lists = {'iterations': [], 'fitnesses': [], 'n_jumps': [], 
                      'energies': [], 'landing_costs': [], 'all_points': []}
        
        for filename in iter_files:
            data = self._load_json(os.path.join(self.iterations_folder, filename))
            if data is None:
                continue
            
            iteration_num = data.get('iteration', 0)
            for elite in data.get('elites', []):
                data_lists['iterations'].append(iteration_num)
                data_lists['fitnesses'].append(elite.get('fitness', 0.0))
                data_lists['n_jumps'].append(elite.get('n_jumps', 0))
                data_lists['energies'].append(elite.get('consumed_energy', 0.0))
                data_lists['landing_costs'].append(elite.get('landing_cost', 0.0))
                data_lists['all_points'].append(elite.get('points', []))
        
        # Convert to numpy arrays
        for key in ['iterations', 'fitnesses', 'n_jumps', 'energies', 'landing_costs']:
            setattr(self, key, np.array(data_lists[key]) if data_lists[key] else np.array([]))
        self.all_points = data_lists['all_points']
        
        print(f"[INFO] Extracted {len(self.fitnesses)} solutions from {len(iter_files)} files.")
        
    def _extract_trajectory_points(self):
        """Extract start, end, and intermediate points from solutions."""
        starts, ends, intermediates = [], [], []
        
        for pts in self.all_points:
            if not pts:
                continue
            pts_np = [np.array(p) if not isinstance(p, np.ndarray) else p for p in pts]
            
            if len(pts_np) > 0:
                starts.append(pts_np[0])
            if len(pts_np) > 1:
                ends.append(pts_np[-1])
            if len(pts_np) > 2:
                intermediates.extend(pts_np[1:-1])
        
        self.starts_np = np.array(starts) if starts else np.array([]).reshape(0, 3)
        self.ends_np = np.array(ends) if ends else np.array([]).reshape(0, 3)
        self.intermediates_np = np.array(intermediates) if intermediates else np.array([]).reshape(0, 3)
        
    def _extract_best_trajectory_data(self):
        """Extract best trajectory data."""
        if self.data_best_log:
            pts = self.data_best_log.get('best_jump_log_points', [])
            if pts:
                self.best_points_np = np.array(pts)
            
            traj = self.data_best_log.get('best_trajectory', [])
            self.best_traj_segments = [np.array(t) for t in traj if t is not None]
            print(f"[INFO] Extracted best trajectory with {len(self.best_traj_segments)} segments.")
    
    def _setup_axis(self, ax=None, projection=None):
        """Helper to create or use existing axis."""
        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection=projection)
            created_fig = True
        else:
            fig = ax.figure
        return fig, ax, created_fig

    def plot_actual_terrain(self, ax=None):
        """Plot terrain point cloud."""
        if self.points_np is None:
            print("ERROR: No terrain data available.")
            return
        
        fig, ax, created = self._setup_axis(ax, '3d')
        ax.scatter(self.px, self.py, self.pz, c=self.colors_np, s=10)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        
        if created:
            plt.show()
            
    def plot_terrain_patches(self, ax=None):
        """Visualize terrain patches with centroids."""
        if not self.patches_list:
            print("WARNING: No patch data available.")
            return

        fig, ax, created = self._setup_axis(ax, '3d')
        cmap = plt.get_cmap('tab20')
        
        for i, patch in enumerate(self.patches_list):
            centroid = patch.get('centroid')
            if centroid is None:
                continue
            
            centroid_np = np.array(centroid)
            color = cmap(i % 20)
            
            ax.scatter(centroid_np[0], centroid_np[1], centroid_np[2],
                      c=[color], s=50, alpha=0.9, edgecolors='black',
                      linewidths=0.5, marker='s')
            ax.text(centroid_np[0], centroid_np[1], centroid_np[2], 
                   f"{patch.get('id', i)}", fontsize=6, ha='center')

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"Terrain Patches ({len(self.patches_list)} patches)")

        if created:
            plt.savefig(f'{MAIN_DIRECTORY}/plot_terrain_patches.png', dpi=150)
            plt.show()
    
    def plot_fitness_by_iteration(self, ax=None):
        """Plot fitness distribution across iterations."""
        if len(self.fitnesses) == 0:
            print("WARNING: No fitness data available.")
            return None
            
        fig, ax, created = self._setup_axis(ax)
        
        scatter = ax.scatter(self.iterations, self.fitnesses, c=self.fitnesses, 
                           cmap='viridis', alpha=0.8, edgecolors='k', s=60)
        fig.colorbar(scatter, ax=ax, label='Fitness Value')
        
        # Highlight best solution
        if len(self.fitnesses) > 0:
            best_idx = np.argmax(self.fitnesses)
            ax.annotate(f'Best: {self.fitnesses[best_idx]:.2f}', 
                       xy=(self.iterations[best_idx], self.fitnesses[best_idx]), 
                       xytext=(self.iterations[best_idx], self.fitnesses[best_idx] + 
                              (max(self.fitnesses)-min(self.fitnesses))*0.05),
                       arrowprops=dict(facecolor='red', shrink=0.05))
        
        ax.set_title('Fitness Distribution vs Iteration', fontsize=14)
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Fitness', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.5)
        fig.tight_layout()
        
        if created:
            fig.savefig(f'{MAIN_DIRECTORY}/plot_fitness_iteration.png')
            plt.show()
        
        return scatter
        
    def plot_jumps_histogram(self, ax=None):
        """Plot histogram of jump counts."""
        if len(self.n_jumps) == 0:
            print("WARNING: No jump data available.")
            return
            
        fig, ax, created = self._setup_axis(ax)
        bins = np.arange(min(self.n_jumps) - 0.5, max(self.n_jumps) + 1.5, 1)
        
        ax.hist(self.n_jumps, bins=bins, color='skyblue', edgecolor='black', 
               alpha=0.7, rwidth=0.8)
        ax.set_title('Jump Count Distribution', fontsize=14)
        ax.set_xlabel('Number of Jumps', fontsize=12)
        ax.set_ylabel('Solution Count', fontsize=12)
        ax.set_xticks(range(min(self.n_jumps), max(self.n_jumps) + 1))
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        fig.tight_layout()
        
        if created:
            fig.savefig(f'{MAIN_DIRECTORY}/plot_jumps_histogram.png')
            plt.show()
        
    def plot_energy_vs_cost(self, ax=None):
        """Plot energy vs landing cost trade-off."""
        if len(self.energies) == 0 or len(self.landing_costs) == 0:
            print("WARNING: No energy/cost data available.")
            return
            
        fig, ax, created = self._setup_axis(ax)
        
        scatter = ax.scatter(self.energies, self.landing_costs, c=self.fitnesses, 
                           cmap='plasma', s=50, alpha=0.8, edgecolors='gray')
        cbar = fig.colorbar(scatter, ax=ax)
        cbar.set_label('Total Fitness')
        
        ax.set_title('Energy vs Landing Cost Trade-off', fontsize=14)
        ax.set_xlabel('Consumed Energy [J]', fontsize=12)
        ax.set_ylabel('Landing Cost', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.5)
        fig.tight_layout()
        
        if created:
            fig.savefig(f'{MAIN_DIRECTORY}/plot_energy_vs_cost.png')
            plt.show()
    
    def plot_3d_scenario(self, ax=None):
        """Complete 3D visualization with terrain and jump points."""
        if self.px is None or len(self.px) == 0:
            print("WARNING: No terrain data available.")
            return
            
        fig, ax, created = self._setup_axis(ax, '3d')
        
        # Terrain
        ax.scatter(self.px, self.py, self.pz, c='gray', s=2, alpha=0.20, label='Terrain')
        
        # Start and goal
        if self.starts_np.size > 0 and len(self.starts_np.shape) > 1:
            ax.scatter(self.starts_np[0,0], self.starts_np[0,1], self.starts_np[0,2], 
                      c='lime', s=100, marker='^', label='Start', depthshade=False)
        
        if self.ends_np.size > 0 and len(self.ends_np.shape) > 1:
            ax.scatter(self.ends_np[0,0], self.ends_np[0,1], self.ends_np[0,2], 
                      c='red', s=100, marker='X', label='Goal', depthshade=False)
        
        # Jump points
        if self.intermediates_np.size > 0 and len(self.intermediates_np.shape) > 1:
            ax.scatter(self.intermediates_np[:,0], self.intermediates_np[:,1], 
                      self.intermediates_np[:,2], c='blue', s=20, marker='o', 
                      alpha=0.6, label='Jump Points')
        
        ax.set_title("Jump Visualization on Terrain", fontsize=14)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        ax.view_init(elev=25, azim=-45)
        
        if created:
            plt.savefig(f'{MAIN_DIRECTORY}/plot_3d_scenario.png', dpi=150)
            plt.show()
    
    def compute_edge_patches(self):
        """Compute patch contours using ConvexHull."""
        self.edge_patches = []
        
        for patch in self.patches_list:
            points_data = patch.get('points_in_patch', [])
            if len(points_data) < 3:
                continue
            
            coords = np.array([p['position'] for p in points_data])
            
            try:
                points_2d = coords[:, [1, 2]]
                hull = ConvexHull(points_2d)
                sorted_pts = coords[hull.vertices]
                sorted_pts = np.vstack([sorted_pts, sorted_pts[0]])
                
                self.edge_patches.append({
                    'position': sorted_pts.tolist(),
                    'color': [0, 0, 0]
                })
            except:
                continue
    
    def plot_density_map(self, ax=None):
        """Plot 3D density heatmap of jump points on YZ plane."""
        fig, ax, created = self._setup_axis(ax, '3d')
        
        # Terrain background
        ax.scatter(self.px, self.py, self.pz, c=self.colors_np, s=2, alpha=0.3, label='Terrain')
        
        # Extract jump points
        jump_y, jump_z, jump_x = [], [], []
        for pts in self.all_points:
            if len(pts) > 2:
                for p in pts[1:-1]:
                    jump_x.append(p[0])
                    jump_y.append(p[1])
                    jump_z.append(p[2])
        
        if not jump_y:
            print("Warning: No jump points for heatmap.")
            return
        
        # Create density grid
        y_min, y_max = min(self.py), max(self.py)
        z_min, z_max = min(self.pz), max(self.pz)
        
        H, yedges, zedges = np.histogram2d(jump_y, jump_z, bins=50,
                                          range=[[y_min, y_max], [z_min, z_max]])
        
        ycenters = (yedges[:-1] + yedges[1:]) / 2
        zcenters = (zedges[:-1] + zedges[1:]) / 2
        Yc, Zc = np.meshgrid(ycenters, zcenters)
        
        # Interpolate density
        yi = np.linspace(y_min, y_max, 100)
        zi = np.linspace(z_min, z_max, 100)
        Yi, Zi = np.meshgrid(yi, zi)
        density = griddata((Yc.flatten(), Zc.flatten()), H.T.flatten(), 
                          (Yi, Zi), method='cubic', fill_value=0)
        
        x_base = min(self.px) - (max(self.px) - min(self.px)) * 0.05
        Xi = np.full_like(Yi, x_base)
        
        # Plot surface and points
        ax.plot_surface(Xi, Yi, Zi, facecolors=plt.cm.inferno(density / density.max()),
                       alpha=0.6, shade=False, rstride=5, cstride=5)
        ax.scatter(jump_x, jump_y, jump_z, c='gold', s=20, marker='o', 
                  alpha=1.0, label='Jump Points', edgecolors='white', linewidths=0.5)
        
        # Colorbar
        m = plt.cm.ScalarMappable(cmap='inferno')
        m.set_array(density)
        cbar = plt.colorbar(m, ax=ax, pad=0.1, shrink=0.7)
        cbar.set_label('Jump Density', fontsize=10)
        
        ax.set_title("Jump Density Map (YZ Plane)", fontsize=14)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.view_init(elev=20, azim=10)
        
        if created:
            plt.savefig(f'{MAIN_DIRECTORY}/plot_density_map.png', dpi=150)
            plt.show()
        
    def plot_evolution_fitness(self):
        """Plot fitness convergence over iterations."""
        if not os.path.exists(self.iterations_folder):
            print("[WARNING] Iterations folder not found")
            return
        
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        if not iter_files:
            return
        
        iterations, best_values, best_ever = [], [], []
        current_best = float('-inf')
        
        for filename in iter_files:
            data = self._load_json(os.path.join(self.iterations_folder, filename))
            if data:
                iter_num = data.get('iteration', 0)
                best_fitness = data.get('best_fitness_this_iter', 0.0)
                current_best = max(current_best, best_fitness)
                
                iterations.append(iter_num)
                best_values.append(best_fitness)
                best_ever.append(current_best)
        
        if not iterations:
            return
        
        plt.figure(figsize=(12, 7))
        plt.plot(iterations, best_values, linewidth=2, marker='o', markersize=4, 
                color='#2E86AB', alpha=0.6, label='Best Fitness (Iter)')
        plt.plot(iterations, best_ever, linewidth=3, color='#27AE60', 
                label='Best Ever', zorder=5)
        
        plt.xlabel('Iteration', fontsize=12, fontweight='bold')
        plt.ylabel('Fitness', fontsize=12, fontweight='bold')
        plt.title('CEM Fitness Convergence', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, linestyle='--')
        plt.legend(fontsize=10)
        plt.tight_layout()
        
        plt.savefig(f'{MAIN_DIRECTORY}/plot_evolution_fitness.png', dpi=150)
        plt.show()

    def plot_evolution_std_dev_cem(self):
        """Plot standard deviation convergence for CEM variables."""
        history_file = f"{MAIN_DIRECTORY}/cem_iteration_history.json"
        
        if not os.path.exists(history_file):
            print("[WARNING] CEM history file not found")
            return
        
        with open(history_file, 'r') as f:
            history_data = json.load(f)
        
        iteration_history = history_data.get('iteration_history', [])
        if not iteration_history:
            return
        
        iterations = [h['iteration'] for h in iteration_history]
        std_history = np.array([h['std_devs'] for h in iteration_history])
        
        plt.figure(figsize=(12, 6))
        colors = plt.cm.tab20(np.linspace(0, 1, std_history.shape[1]))
        
        for i in range(std_history.shape[1]):
            patch_num = i // 2 + 1
            coord = 'Y' if i % 2 == 0 else 'Z'
            plt.plot(iterations, std_history[:, i], 
                    label=f'Patch {patch_num}-{coord}', linewidth=2, 
                    color=colors[i], marker='.')
        
        plt.xlabel('Iteration', fontsize=12, fontweight='bold')
        plt.ylabel('Standard Deviation', fontsize=12, fontweight='bold')
        plt.title('CEM Standard Deviation Convergence', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, linestyle='--')
        plt.legend(fontsize=9, ncol=2)
        plt.tight_layout()
        
        plt.savefig(f'{MAIN_DIRECTORY}/plot_evolution_std_dev.png', dpi=150)
        plt.show()

    def plot_3d_scenario_iterations(self, animated=False, save_animation=False):
        """3D visualization of iteration evolution with correct 1:1:1 scaling."""
        plt.close('all')
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # --- RACCOLTA DATI PER SCALATURA ---
        all_x, all_y, all_z = [], [], []
        if self.px is not None:
            all_x.extend(self.px.flatten()); all_y.extend(self.py.flatten()); all_z.extend(self.pz.flatten())
        
        # Terrain background
        if self.px is not None:
            ax.scatter(self.px, self.py, self.pz, c='gray', s=1, alpha=0.15, label='Terrain', zorder=1)
        
        self.compute_edge_patches()
        for i, edge in enumerate(self.edge_patches):
            pts = np.array(edge['position'])
            lbl = 'Patch Contour' if i == 0 else ""
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], c='black', 
                   linewidth=1.0, alpha=0.5, zorder=2, label=lbl)
        
        # Start and goal markers
        if self.start_pos_terrain:
            sp = np.array(self.start_pos_terrain)
            ax.scatter(sp[0], sp[1], sp[2], c='lime', s=150, marker='^', 
                      label='Start', zorder=15, edgecolors='darkgreen', linewidths=2)
        if self.goal_pos_terrain:
            gp = np.array(self.goal_pos_terrain)
            ax.scatter(gp[0], gp[1], gp[2], c='red', s=150, marker='X', 
                      label='Goal', zorder=15, edgecolors='darkred', linewidths=2)
        
        # Load iteration data
        if not os.path.exists(self.iterations_folder): return
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        cmap = plt.cm.get_cmap('rainbow', len(iter_files))
        
        iteration_data = []
        for idx, filename in enumerate(iter_files):
            data = self._load_json(os.path.join(self.iterations_folder, filename))
            if not data: continue
            
            all_pts = []
            for elite in data.get('elites', []):
                pts = elite.get('points', [])
                if pts: all_pts.extend(pts)
            
            if all_pts:
                pts_np = np.array(all_pts)
                iteration_data.append({'points': pts_np, 'color': cmap(idx), 'num': data.get('iteration', idx)})
                all_x.extend(pts_np[:, 0]); all_y.extend(pts_np[:, 1]); all_z.extend(pts_np[:, 2])

        # --- APPLICAZIONE SCALATURA ---
        if all_x:
            all_x, all_y, all_z = np.array(all_x), np.array(all_y), np.array(all_z)
            max_range = np.array([all_x.max()-all_x.min(), all_y.max()-all_y.min(), all_z.max()-all_z.min()]).max() / 2.0
            mid_x, mid_y, mid_z = (all_x.max()+all_x.min())*0.5, (all_y.max()+all_y.min())*0.5, (all_z.max()+all_z.min())*0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

        if not animated:
            for idx, d in enumerate(iteration_data):
                lbl = f"Iter {d['num']}" if idx in [0, len(iteration_data)-1] else ""
                ax.scatter(d['points'][:, 0], d['points'][:, 1], d['points'][:, 2], 
                          c=[d['color']], s=80, alpha=0.8, label=lbl, zorder=4, edgecolors='white', linewidths=0.5)
            ax.set_title("Evolution of Jumps (Static Scaled)", fontsize=14, weight='bold')
        else:
            scatter_objects = []
            title_text = ax.text2D(0.5, 0.95, '', transform=ax.transAxes, ha='center', fontsize=12, weight='bold', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

            def update(frame):
                for s in scatter_objects: s.remove()
                scatter_objects.clear()
                if frame < len(iteration_data):
                    curr = iteration_data[frame]
                    scat = ax.scatter(curr['points'][:, 0], curr['points'][:, 1], curr['points'][:, 2], 
                                     c=[curr['color']], s=80, edgecolors='white', linewidths=0.7, zorder=4, alpha=0.95)
                    scatter_objects.append(scat)
                    title_text.set_text(f"Iteration: {curr['num']}")
                return scatter_objects + [title_text]

            anim = FuncAnimation(fig, update, frames=len(iteration_data), interval=300, blit=False)
            self._anim_3d = anim
            if save_animation: anim.save(f'{MAIN_DIRECTORY}/animation_3d_iterations.gif', writer='pillow', fps=3)
        
        ax.set_xlabel('X (Depth)'); ax.set_ylabel('Y (Width)'); ax.set_zlabel('Z (Height)')
        ax.view_init(elev=25, azim=-45)
        plt.tight_layout()
        plt.show()

    def plot_2d_iterations_layout(self, animated=False, save_animation=False):
        """2D YZ plane visualization with equal aspect ratio (real proportions)."""
        plt.close('all')
        if not self.patches_list: return

        fig, ax = plt.subplots(figsize=(12, 10))
        
        # --- FORZA ASPETTO EQUALE ---
        ax.set_aspect('equal', adjustable='box')

        # Draw patches
        costs = [p.get('cost_patch', 0) for p in self.patches_list]
        norm_cost = colors.Normalize(vmin=min(costs), vmax=max(costs))
        cmap_terrain = plt.get_cmap('Greys')

        for patch in self.patches_list:
            pts_in = patch.get('points_in_patch', [])
            if len(pts_in) >= 3:
                coords_2d = np.array([[p['position'][1], p['position'][2]] for p in pts_in])
                try:
                    hull = ConvexHull(coords_2d)
                    poly = plt.Polygon(coords_2d[hull.vertices], color=cmap_terrain(norm_cost(patch.get('cost_patch', 0))), 
                                     alpha=0.3, ec='black', lw=0.5)
                    ax.add_patch(poly)
                except: pass

        # Start/Goal
        if self.start_pos_terrain:
            ax.scatter(self.start_pos_terrain[1], self.start_pos_terrain[2], c='lime', s=150, marker='^', zorder=15, label='Start', edgecolors='black')
        if self.goal_pos_terrain:
            ax.scatter(self.goal_pos_terrain[1], self.goal_pos_terrain[2], c='red', s=150, marker='X', zorder=15, label='Goal', edgecolors='black')

        # Load iterations
        if not os.path.exists(self.iterations_folder): return
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        cmap_iters = plt.get_cmap('jet', len(iter_files))

        iteration_data = []
        for idx, filename in enumerate(iter_files):
            data = self._load_json(os.path.join(self.iterations_folder, filename))
            if not data: continue
            pts_2d = []
            for elite in data.get('elites', []):
                p_list = elite.get('points', [])
                if p_list: pts_2d.extend([[p[1], p[2]] for p in p_list])
            if pts_2d:
                iteration_data.append({'points': np.array(pts_2d), 'color': cmap_iters(idx), 'num': data.get('iteration', idx)})

        if not animated:
            for i, d in enumerate(iteration_data):
                lbl = f"Iter {d['num']}" if i % max(1, len(iteration_data)//5) == 0 else ""
                ax.scatter(d['points'][:, 0], d['points'][:, 1], color=[d['color']], s=30, alpha=0.6, label=lbl, edgecolors='white', lw=0.3)
            ax.set_title("2D Evolution (YZ Plane - Scaled)")
        else:
            scatter_objs = []
            title_text = ax.text(0.5, 1.05, '', transform=ax.transAxes, ha='center', fontweight='bold')

            def update(frame):
                for s in scatter_objs: s.remove()
                scatter_objs.clear()
                # Mostra le ultime 2 iterazioni per dare senso di movimento (ghosting)
                for i in range(max(0, frame-1), frame + 1):
                    d = iteration_data[i]
                    alpha = 1.0 if i == frame else 0.3
                    s = ax.scatter(d['points'][:, 0], d['points'][:, 1], color=[d['color']], s=50, alpha=alpha, edgecolors='black', lw=0.5)
                    scatter_objs.append(s)
                title_text.set_text(f"Iteration: {iteration_data[frame]['num']}")
                return scatter_objs + [title_text]

            anim = FuncAnimation(fig, update, frames=len(iteration_data), interval=200, blit=False)
            self._anim_2d = anim
            if save_animation: anim.save(f'{MAIN_DIRECTORY}/animation_2d_iterations.gif', writer='pillow', fps=5)

        ax.set_xlabel("Y (Width)"); ax.set_ylabel("Z (Height)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
        plt.tight_layout()
        plt.show()
    
    def plot_mesh_traj(self, ax=None):
        """Visualize best trajectory with parabolic segments and correct scaling."""
        if self.best_points_np is None or not self.best_traj_segments:
            print("WARNING: No best trajectory data found.")
            return

        fig, ax, created = self._setup_axis(ax, '3d')

        # 1. Terrain background
        if self.px is not None:
            ax.scatter(self.px, self.py, self.pz, c='gray', s=1, alpha=0.80, label='Terrain', zorder=1)

        # 2. Trajectory segments - FIXED: Using row indexing for (3, N) data
        for i, segment in enumerate(self.best_traj_segments):
            # Verifichiamo la forma: BilevelOpt produce (3, N)
            if segment.shape[0] == 3:
                x_vals, y_vals, z_vals = segment[0, :], segment[1, :], segment[2, :]
            else:
                x_vals, y_vals, z_vals = segment[:, 0], segment[:, 1], segment[:, 2]
                
            label = "Best Trajectory" if i == 0 else ""
            ax.plot(x_vals, y_vals, z_vals, color='blue', linewidth=2.5, zorder=10, label=label)

        # 3. Landing points - self.best_points_np è (M, 3)
        ax.scatter(self.best_points_np[:, 0], self.best_points_np[:, 1], self.best_points_np[:, 2], 
                  c='yellow', s=60, edgecolors='black', marker='o', zorder=11, label='Landing Points')

        # 4. Start and goal
        ax.scatter(self.best_points_np[0, 0], self.best_points_np[0, 1], self.best_points_np[0, 2], 
                  c='lime', s=150, marker='^', edgecolors='black', zorder=12, label='Start')
        ax.scatter(self.best_points_np[-1, 0], self.best_points_np[-1, 1], self.best_points_np[-1, 2], 
                  c='red', s=150, marker='X', edgecolors='black', zorder=12, label='Goal')

        # --- LOGICA DI SCALATURA CORRETTA (Sincronizzata con lo script funzionante) ---
        # Estraiamo correttamente le coordinate da tutti i segmenti per i limiti
        all_traj_x = [s[0, :] if s.shape[0] == 3 else s[:, 0] for s in self.best_traj_segments]
        all_traj_y = [s[1, :] if s.shape[0] == 3 else s[:, 1] for s in self.best_traj_segments]
        all_traj_z = [s[2, :] if s.shape[0] == 3 else s[:, 2] for s in self.best_traj_segments]

        # Uniamo con i punti del terreno per decidere il bound globale
        total_x = np.concatenate(all_traj_x + ([self.px] if self.px is not None else []))
        total_y = np.concatenate(all_traj_y + ([self.py] if self.py is not None else []))
        total_z = np.concatenate(all_traj_z + ([self.pz] if self.pz is not None else []))

        max_range = np.array([
            total_x.max() - total_x.min(), 
            total_y.max() - total_y.min(), 
            total_z.max() - total_z.min()
        ]).max() / 2.0

        mid_x = (total_x.max() + total_x.min()) * 0.5
        mid_y = (total_y.max() + total_y.min()) * 0.5
        mid_z = (total_z.max() + total_z.min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        # ------------------------------------------------------------------------------

        ax.set_title("Best Trajectory Visualization", fontsize=14, fontweight='bold')
        ax.set_xlabel("X (Depth/Height)")
        ax.set_ylabel("Y (Horizontal)")
        ax.set_zlabel("Z (Vertical)")
        ax.legend()
        ax.view_init(elev=30, azim=-60)

        if created:
            plt.tight_layout()
            # plt.savefig(f'{MAIN_DIRECTORY}/best_trajectory_plot.png', dpi=150)
            plt.show()
    
    def plot_all_in_one(self):
        """Combined plot with multiple subplots."""
        fig = plt.figure(figsize=(16, 12))
        
        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224, projection='3d')
        
        sc1 = self.plot_fitness_by_iteration(ax1)
        self.plot_jumps_histogram(ax2)
        self.plot_energy_vs_cost(ax3)
        self.plot_3d_scenario(ax4)
        
        if sc1:
            fig.colorbar(sc1, ax=ax2, label='Fitness Value')
        
        plt.tight_layout()
        plt.savefig(f'{MAIN_DIRECTORY}/plot_all_in_one.png', dpi=150)
        plt.show()


def main():
    """Main execution function."""
    plotter = PlotResultCemMjumps()
    print("[INFO] Plotter initialized successfully")
    
    # Generate all plots
    plotter.plot_terrain_patches()
    plotter.plot_actual_terrain()
    plotter.plot_all_in_one()
    # plotter.plot_density_map()
    plotter.plot_3d_scenario_iterations(animated=True)
    plotter.plot_2d_iterations_layout(animated=True)
    # plotter.plot_evolution_fitness()
    # plotter.plot_evolution_std_dev_cem()
    plotter.plot_mesh_traj()
    
    print("[INFO] All plots generated successfully")


if __name__ == "__main__":
    main()