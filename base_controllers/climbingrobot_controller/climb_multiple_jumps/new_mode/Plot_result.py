import json
from typing import Any, List
from attr import dataclass
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import colors
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import os
from scipy.interpolate import griddata
from scipy.spatial import ConvexHull 
from params import *
import matplotlib
matplotlib.use('Qt5Agg')

FILE_TERRAIN_POINTS = f"{MAIN_DIRECTORY}/actual_point_terrain.json"
FILE_TERRAIN_PATCHES = f"{MAIN_DIRECTORY}/actual_patch_terrain.json"
ITERATIONS_FOLDER = f"{MAIN_DIRECTORY}/iteration_reports"
FILE_SAVE_PARAMS = f"{MAIN_DIRECTORY}/simulation_params.json"

@dataclass
class InnerParams:
    Fleg_max: float
    Fr_max: float
    Fr_min: float
    mass: float
    anchor_distance: float
    fitness_weights: List[float]
    filter_weights: List[float]
    inner_opt_params: Any

@dataclass
class CemParams:
    seed: int
    n_threads: int
    cem_iters: int
    pop_size: int
    n_elites: int
    decrease_pop_factor: float
    fraction_elites_reused: float
    dim_discrete: int
    n_values: int
    init_probs: List[float]
    min_prob: float
    dim_continuous: int
    max_value_continuous: List[float]
    min_value_continuous: List[float]
    init_mu_continuous: List[float]
    init_std_continuous: List[float]
    min_std_continuous: List[float]

@dataclass
class eliteData:
    fitness: float
    n_jumps: int
    consumed_energy: float
    landing_cost: float
    points: List[List[float]]  
    traj: List[List[List[float]]]
    patch_ids: List[int]
    


class PlotResultCemMjumps:
    
    def __init__(self):
        
        # LOAD PARAMS
        self.data_params = self.load_simulation_params()
        
        self.p0 = self.data_params['START']
        self.pf = self.data_params['GOAL']
        self.n_jumps = self.data_params['MAX_JUMP']
        self.n_threads = self.data_params['THREADS']
        self.inner_params = InnerParams(**self.data_params['inner_opt_params_order'])
        self.cem_params = CemParams(**self.data_params['cem_params'])
        
        # LOAD POINTS
        self.data_terrain_points = self.load_terrain_points()
        self.num_points = self.data_terrain_points['metadata']['num_points']
        self.mesh_bounds = self.data_terrain_points['mesh_bounds'] # [xmin, xmax, ymin, ymax, zmin, zmax]
        self.points_t_data = self.data_terrain_points['points'] 
                                                # list of dicts:
                                                # - position: [x, y, z]
                                                # - color: [r, g, b]
                                                # - light: float
                                                # - size_point: float
                                                # - cost: float
        self.pc_plotter = PointCloudFilter(pc=[p['position'] for p in self.points_t_data])
        
        # LOAD PATCHES
        self.data_terrain_patches = self.load_terrain_patches()
        self.num_patches = self.data_terrain_patches['metadata']['num_patches']
        self.patch_width = self.data_terrain_patches['metadata']['patch_width']
        self.patch_height = self.data_terrain_patches['metadata']['patch_height']
        self.patches = self.data_terrain_patches['patches'] 
        self.patch_plotter = PatchSurface(points_t=self.points_t_data,
                                          number_of_patches_height=int(self.patch_width),
                                          number_of_patches_width=int(self.patch_height),)
        
        
        # self.data_terrain_patches = self.load_terrain_patches(FILE_TERRAIN_PATCHES)
        # self.data_iteration_history = self.load_iteration_history(ITERATIONS_FOLDER)
    
        # LOAD ITERATION HISTORY
        
        self.best_fit_ever = []         # take from last iteration
        self.best_energy_ever = []      # take from last iteration
        self.best_land_cost_ever = []   # take from last iteration
        self.best_traj_ever = []        # take from last iteration
        self.best_fit_each_iter = []
        self.all_elites = []
        self.iteration_files = sorted([f for f in os.listdir(ITERATIONS_FOLDER) if f.startswith('iteration_') and f.endswith('.json')],
                                      key=lambda x: int(x.split('_')[1].split('.')[0]))
        
        self.load_iteration_history()
        
    def load_iteration_history(self):
        
        for file_name in self.iteration_files:
            with open(os.path.join(ITERATIONS_FOLDER, file_name), 'r') as file:
                iteration_data = json.load(file)
                
                self.best_fit_each_iter.append(iteration_data['best_fitness_this_iter'])
                
                iteration_elites = []
                for e  in iteration_data['elites']:
                    elite_instance = eliteData(
                        fitness=e['fitness'],
                        n_jumps=e['n_jumps'],
                        consumed_energy=e['consumed_energy'],
                        landing_cost=e['landing_cost'],
                        points=e['points'],
                        traj=e['traj'],
                        patch_ids=e['patch_ids']
                    )
                    iteration_elites.append(elite_instance)
                    
                self.all_elites.append(iteration_elites)
                
                self.best_fit_ever = iteration_data['best_fitness_ever']
                self.best_energy_ever = iteration_data['best_consumed_energy_ever']
                self.best_land_cost_ever = iteration_data['best_landing_cost_ever']
                self.best_traj_ever = iteration_data['best_trajectory_ever']
          
    def load_simulation_params(self):
        
        with open(FILE_SAVE_PARAMS, 'r') as file:
            data_params = json.load(file)
        return data_params
    
    def load_terrain_points(self):    
        with open(FILE_TERRAIN_POINTS, 'r') as file:
            data_terrain_points = json.load(file)
        return data_terrain_points
    
    def load_terrain_patches(self):
        with open(FILE_TERRAIN_PATCHES, 'r') as file:
            data_terrain_patches = json.load(file)
        return data_terrain_patches
    
    # ================= 
    # Plot methods
    # =================
    
    def base_plot(self):
        # plot point cloud with cost
        self.pc_plotter.visualize_cost_map(self.points_t_data)
        # plot patches with different colors
        # self.patch_plotter.random_color()
        # self.patch_plotter.cost_color()
        # self.patch_plotter.plot_patches()
        # TODO da capire come mai non va
    
    
    def count_jump_histogram(self, ax=None, use_last_iter_only=False):
        """Plot histogram of jump counts."""
        if use_last_iter_only:
            jump_data = [e.n_jumps for e in self.all_elites[-1]]
            title_suffix = "(Last Iteration)"
        else:
            jump_data = [e.n_jumps for iter_list in self.all_elites for e in iter_list]
            title_suffix = "(All Iterations)"

        if not jump_data:
            print("WARNING: Jump data list is empty.")
            return

        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(8, 6))
            created = True
        else:
            fig = ax.get_figure()

        # 3. Plotting
        min_j, max_j = min(jump_data), max(jump_data)
        bins = np.arange(min_j - 0.5, max_j + 1.5, 1)
        
        ax.hist(jump_data, bins=bins, color='skyblue', edgecolor='black', 
                alpha=0.7, rwidth=0.8)
        
        # 4. Formattazione
        ax.set_title(f'Jump Count Distribution {title_suffix}', fontsize=14)
        ax.set_xlabel('Number of Jumps', fontsize=12)
        ax.set_ylabel('Frequency (Elites)', fontsize=12)
        ax.set_xticks(range(int(min_j), int(max_j) + 1))
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        
        fig.tight_layout()
        
        if created:
            save_path = f'{MAIN_DIRECTORY}/plot_jumps_histogram.png'
            fig.savefig(save_path)
            print(f"Histogram saved to: {save_path}")
            plt.show()
   
    def plot_fitness_by_iteration(self, ax=None):
        if not self.all_elites:
            print("WARNING: No data available for plotting fitness.")
            return

        # 1. Preparazione dell'asse
        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(12, 7))
            created = True
        else:
            fig = ax.get_figure()

        # 2. Separazione dei dati in due gruppi: "Normali" e "Best Ever"
        x_normal, y_normal = [], []
        x_best, y_best = [], []
        
        # Tolleranza per il confronto di numeri float
        tolerance = 1e-9 

        all_fitness_values_for_norm = [] # Serve per calcolare min/max globali

        for i, iteration_list in enumerate(self.all_elites):
            current_iter = i + 1
            for elite in iteration_list:
                all_fitness_values_for_norm.append(elite.fitness)
                
                # Se la fitness è "uguale" alla migliore di sempre
                if np.isclose(elite.fitness, self.best_fit_ever, atol=tolerance):
                    x_best.append(current_iter)
                    y_best.append(elite.fitness)
                else:
                    x_normal.append(current_iter)
                    y_normal.append(elite.fitness)

        if not all_fitness_values_for_norm:
             print("WARNING: No fitness values found.")
             return

        # 3. Configurazione della Colormap (solo per i punti "normali")
        # Usiamo min/max totali per avere una scala coerente
        vmin = min(all_fitness_values_for_norm)
        vmax = max(all_fitness_values_for_norm)
        norm = Normalize(vmin=vmin, vmax=vmax)
        colors_cmap = ['green', 'orange'] 
        cmap_name = 'green_to_orange_gradient'
        custom_cmap = LinearSegmentedColormap.from_list(cmap_name, colors_cmap, N=256)
        sc = ax.scatter(x_normal, y_normal, c=y_normal, cmap=custom_cmap, norm=norm,
                        s=35, edgecolors='black', linewidths=0.3, alpha=0.8, zorder=3,
                        label='Elite Solutions (Gradient)')
        if x_best:
            ax.scatter(x_best, y_best, c='red',
                       s=50, edgecolors='black', linewidths=0.5, alpha=1.0, zorder=4,
                       label='Best Ever Reached')
        ax.set_title('Elite Fitness Distribution', fontsize=14)
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Fitness Value', fontsize=12)
        ax.axhline(y=self.best_fit_ever, color='red', linestyle='--', linewidth=1, alpha=0.5, zorder=2)
        
        ax.grid(True, linestyle=':', alpha=0.5, zorder=0)
        
        ax.legend(loc='upper right', frameon=True, fancybox=True, framealpha=0.9)
        max_iter = len(self.all_elites)
        if max_iter <= 25:
             ax.set_xticks(range(1, max_iter + 1))
        else:
            ax.xaxis.get_major_locator().set_params(integer=True)

        fig.tight_layout()

        if created:
            save_path = f'{MAIN_DIRECTORY}/plot_fitness_colormap_redbest.png'
            fig.savefig(save_path)
            print(f"Fitness plot saved to: {save_path}")
            plt.show()
    
    def plot_mesh_pc_traj(self, ax=None, show_cost=True):
        """
        Visualizza la migliore traiettoria assoluta (best_ever) sul terreno 3D.
        Flag show_cost: se True, colora i punti del terreno in base al loro costo.
        """
        
        # 1. Controllo disponibilità dati
        if not self.best_traj_ever:
            print("WARNING: Nessuna traiettoria 'best_ever' trovata.")
            return

        # Setup dell'asse 3D
        if ax is None:
            fig = plt.figure(figsize=(14, 9)) # Leggermente più grande per far spazio alla colorbar
            ax = fig.add_subplot(111, projection='3d')
            created = True
        else:
            fig = ax.get_figure()
            created = False

        # 2. Render del Terreno
        px = np.array([p['position'][0] for p in self.points_t_data])
        py = np.array([p['position'][1] for p in self.points_t_data])
        pz = np.array([p['position'][2] for p in self.points_t_data])
        
        if show_cost:
            # Estraiamo i costi per ogni punto
            costs = np.array([p['cost'] for p in self.points_t_data])
            # Creiamo lo scatter con colormap
            sc_terrain = ax.scatter(px, py, pz, c=costs,cmap='RdYlGn_r', s=1, alpha=1, 
                                   label='Terrain', zorder=1)
            # Aggiungiamo una colorbar dedicata al costo del terreno
            cbar = fig.colorbar(sc_terrain, ax=ax, pad=0.1, shrink=0.6)
            cbar.set_label('Terrain Point Cost', rotation=270, labelpad=15)
        else:
            # Grigio uniforme come prima
            ax.scatter(px, py, pz, c='gray', s=1, alpha=0.3, label='Terrain', zorder=1)

        # 3. Disegno dei Segmenti della Traiettoria
        all_x, all_y, all_z = [px], [py], [pz] 
        landing_points = []
        blue_label_added = False
        yellow_label_added = False

        for i, segment in enumerate(self.best_traj_ever):
            segment_np = np.array(segment)
            if segment_np.shape[0] == 3 and segment_np.shape[1] != 3:
                x_s, y_s, z_s = segment_np[0, :], segment_np[1, :], segment_np[2, :]
            else:
                x_s, y_s, z_s = segment_np[:, 0], segment_np[:, 1], segment_np[:, 2]
            
            displacement = np.sqrt((x_s[0]-x_s[-1])**2 + (y_s[0]-y_s[-1])**2 + (z_s[0]-z_s[-1])**2)
            
            if displacement < 1e-3:
                jump_color = 'yellow'
                label = "Stationary Jump" if not yellow_label_added else ""
                yellow_label_added = True
            else:
                jump_color = 'blue'
                label = "Active Trajectory" if not blue_label_added else ""
                blue_label_added = True
            
            ax.plot(x_s, y_s, z_s, color=jump_color, linewidth=2.5, zorder=10, label=label)
            
            all_x.append(x_s)
            all_y.append(y_s)
            all_z.append(z_s)
            landing_points.append([x_s[0], y_s[0], z_s[0]])
            if i == len(self.best_traj_ever) - 1:
                landing_points.append([x_s[-1], y_s[-1], z_s[-1]])

        lp = np.array(landing_points)

        # 4. Landing Points, Start (p0) e Goal (pf)
        ax.scatter(lp[:, 0], lp[:, 1], lp[:, 2], c='black', s=25, zorder=11, label='Contact Points')
        ax.scatter(self.p0[0], self.p0[1], self.p0[2], c='lime', s=200, marker='^', 
                   edgecolors='black', linewidths=1.5, zorder=15, label='Start (p0)')
        ax.scatter(self.pf[0], self.pf[1], self.pf[2], c='red', s=200, marker='X', 
                   edgecolors='black', linewidths=1.5, zorder=15, label='Goal (pf)')

        # 5. Scaling 1:1:1
        flat_x, flat_y, flat_z = np.concatenate(all_x), np.concatenate(all_y), np.concatenate(all_z)
        max_range = np.array([flat_x.max()-flat_x.min(), flat_y.max()-flat_y.min(), flat_z.max()-flat_z.min()]).max() / 2.0
        mid_x, mid_y, mid_z = (flat_x.max()+flat_x.min())*0.5, (flat_y.max()+flat_y.min())*0.5, (flat_z.max()+flat_z.min())*0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # 6. Titolo e Legenda
        cost_status = "with Cost Map" if show_cost else ""
        ax.set_title(f"Best Trajectory {cost_status}\n(Fitness: {self.best_fit_ever:.4f})", 
                    fontsize=13, fontweight='bold')
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend(loc='upper left', fontsize='small')
        ax.view_init(elev=30, azim=-60)

        if created:
            plt.tight_layout()
            plt.show()
   
    def plot_terrain_traj(self, ax=None, show_cost=True):
        pass
    
    def compute_edge_patches(self):
        """Compute patch contours using ConvexHull for 3D visualization."""
        self.edge_patches = []
        
        for patch in self.patches:
            points_data = patch.get('points_in_patch', [])
            if len(points_data) < 3:
                continue
            
            coords = np.array([p['position'] for p in points_data])
            
            try:
                # Project onto YZ plane for hull computation
                points_2d = coords[:, [1, 2]]
                hull = ConvexHull(points_2d)
                sorted_pts = coords[hull.vertices]
                # Close the polygon
                sorted_pts = np.vstack([sorted_pts, sorted_pts[0]])
                
                self.edge_patches.append({
                    'position': sorted_pts,
                    'patch_id': patch.get('id', -1),
                    'cost': patch.get('cost_patch', 0.0)
                })
            except Exception:
                continue
    
    def plot_2d_iterations_layout(self, ax=None, animated=False):
        """2D YZ plane visualization of contact points with patch contours and equal aspect ratio."""
        
        # 1. Setup
        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(14, 10))
            created = True
        else:
            fig = ax.get_figure()
        
        # Force equal aspect ratio for real proportions
        ax.set_aspect('equal', adjustable='box')
        
        # 2. Draw patches with cost-based coloring and contours (STATIC - only once)
        costs = [p.get('cost_patch', 0.0) for p in self.patches]
        if costs:
            norm_cost = Normalize(vmin=min(costs), vmax=max(costs))
        else:
            norm_cost = Normalize(vmin=0, vmax=1)
        
        cmap_terrain = plt.get_cmap('RdYlGn_r')
        
        for patch in self.patches:
            pts_in = patch.get('points_in_patch', [])
            if len(pts_in) < 3:
                continue
            
            coords_2d = np.array([[p['position'][1], p['position'][2]] for p in pts_in])
            
            try:
                hull = ConvexHull(coords_2d)
                hull_vertices = coords_2d[hull.vertices]
                
                poly = plt.Polygon(hull_vertices, 
                                   color=cmap_terrain(norm_cost(patch.get('cost_patch', 0.0))),
                                   alpha=0.4, ec='black', lw=1.0, zorder=1)
                ax.add_patch(poly)
            except Exception:
                continue
        
        # 3. Start and Goal markers (STATIC)
        ax.scatter(self.p0[1], self.p0[2], c='lime', s=200, marker='^',
                   edgecolors='black', linewidths=2, zorder=15, label='Start (p0)')
        ax.scatter(self.pf[1], self.pf[2], c='red', s=200, marker='X',
                   edgecolors='black', linewidths=2, zorder=15, label='Goal (pf)')
        
        # 4. Prepare iteration data
        iteration_data = []
        num_iters = len(self.all_elites)
        cmap_iters = plt.get_cmap('viridis', num_iters)
        
        for iter_idx, iteration_list in enumerate(self.all_elites):
            iter_points_y, iter_points_z = [], []
            for elite in iteration_list:
                for pt in elite.points:
                    iter_points_y.append(pt[1])
                    iter_points_z.append(pt[2])
            
            if iter_points_y:
                iteration_data.append({
                    'y': np.array(iter_points_y),
                    'z': np.array(iter_points_z),
                    'color': cmap_iters(iter_idx),
                    'iter_num': iter_idx
                })
        
        if not iteration_data:
            print("WARNING: No iteration data to plot")
            return
        
        # 5. Best trajectory (STATIC)
        if self.best_traj_ever:
            best_y, best_z = [], []
            for segment in self.best_traj_ever:
                segment_np = np.array(segment)
                if segment_np.shape[0] == 3:
                    best_y.append(segment_np[1, 0])
                    best_z.append(segment_np[2, 0])
                else:
                    best_y.append(segment_np[0, 1])
                    best_z.append(segment_np[0, 2])
            
            if self.best_traj_ever:
                last_seg = np.array(self.best_traj_ever[-1])
                if last_seg.shape[0] == 3:
                    best_y.append(last_seg[1, -1])
                    best_z.append(last_seg[2, -1])
                else:
                    best_y.append(last_seg[-1, 1])
                    best_z.append(last_seg[-1, 2])
            
            ax.plot(best_y, best_z, 'gold', linestyle='--', linewidth=2.5, 
                    alpha=0.8, zorder=13, label='Best Trajectory Path')
            ax.scatter(best_y, best_z, c='gold', s=100, marker='*',
                       edgecolors='black', linewidths=1, zorder=14, 
                       label='Best Contact Points')
        
        # 6. Formatting
        ax.set_xlabel('Y (Width)', fontsize=12)
        ax.set_ylabel('Z (Height)', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.4)
        
        if not animated:
            # Static plot: show all iterations
            for data in iteration_data:
                ax.scatter(data['y'], data['z'], c=[data['color']], s=40, 
                          alpha=0.6, edgecolors='white', linewidths=0.5, zorder=5)
            
            ax.set_title('2D Contact Points Layout - All Iterations', 
                        fontsize=14, fontweight='bold')
            ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=9)
            
            if created:
                fig.tight_layout()
                save_path = f'{MAIN_DIRECTORY}/plot_2d_iterations_static.png'
                fig.savefig(save_path, dpi=150, bbox_inches='tight')
                print(f"2D static layout saved to: {save_path}")
                plt.show()
        else:
            # Animated plot
            scatter_objects = []
            title_text = ax.text(0.5, 1.05, '', transform=ax.transAxes, 
                               ha='center', fontsize=12, fontweight='bold',
                               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            def init():
                """Initialize animation."""
                title_text.set_text('Iteration: 0')
                return [title_text]
            
            def update(frame):
                """Update function for each frame."""
                # Remove previous scatter objects
                for scat in scatter_objects:
                    scat.remove()
                scatter_objects.clear()
                
                # Show current iteration with ghosting effect
                for i in range(max(0, frame - 2), frame + 1):
                    if i < len(iteration_data):
                        data = iteration_data[i]
                        alpha_val = 1.0 if i == frame else 0.3
                        size_val = 50 if i == frame else 30
                        
                        scat = ax.scatter(data['y'], data['z'], 
                                        c=[data['color']], s=size_val,
                                        alpha=alpha_val, edgecolors='white', 
                                        linewidths=0.5, zorder=5)
                        scatter_objects.append(scat)
                
                title_text.set_text(f'Iteration: {iteration_data[frame]["iter_num"]}')
                return scatter_objects + [title_text]
            
            ax.set_title('2D Contact Points Evolution (Animated)', 
                        fontsize=14, fontweight='bold')
            ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=9)
            
            anim = FuncAnimation(fig, update, init_func=init,
                               frames=len(iteration_data), 
                               interval=400, blit=True, repeat=True)
            
            self._anim_2d = anim  # Store reference to prevent garbage collection
            
            if created:
                fig.tight_layout()
                plt.show()
    
    def plot_3d_iterations_layout(self, ax=None, animated=False):
        """3D visualization of contact points with terrain, patch contours, and optional animation."""
        
        # 1. Setup 3D axis
        created = False
        if ax is None:
            fig = plt.figure(figsize=(14, 10))
            ax = fig.add_subplot(111, projection='3d')
            created = True
        else:
            fig = ax.get_figure()
        
        # 2. Data collection for scaling
        all_x, all_y, all_z = [], [], []
        
        # 3. Plot terrain point cloud (STATIC)
        px = np.array([p['position'][0] for p in self.points_t_data])
        py = np.array([p['position'][1] for p in self.points_t_data])
        pz = np.array([p['position'][2] for p in self.points_t_data])
        costs = np.array([p['cost'] for p in self.points_t_data])
        
        all_x.extend(px)
        all_y.extend(py)
        all_z.extend(pz)
        
        ax.scatter(px, py, pz, c=costs, cmap='RdYlGn_r', s=2, alpha=0.5, 
                   zorder=1, label='Terrain')
        
        # 4. Compute and draw patch contours (STATIC)
        self.compute_edge_patches()
        
        for i, edge in enumerate(self.edge_patches):
            pts = edge['position']
            label = 'Patch Contours' if i == 0 else ""
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 
                    c='black', linewidth=1.2, alpha=0.7, zorder=3, label=label)
        
        # 5. Start and Goal markers (STATIC)
        ax.scatter(self.p0[0], self.p0[1], self.p0[2], c='lime', s=250, marker='^',
                   edgecolors='black', linewidths=2, zorder=15, label='Start (p0)')
        ax.scatter(self.pf[0], self.pf[1], self.pf[2], c='red', s=250, marker='X',
                   edgecolors='black', linewidths=2, zorder=15, label='Goal (pf)')
        
        all_x.extend([self.p0[0], self.pf[0]])
        all_y.extend([self.p0[1], self.pf[1]])
        all_z.extend([self.p0[2], self.pf[2]])
        
        # 6. Prepare iteration data
        iteration_data = []
        num_iters = len(self.all_elites)
        cmap_iters = plt.get_cmap('plasma', num_iters)
        
        for iter_idx, iteration_list in enumerate(self.all_elites):
            iter_x, iter_y, iter_z = [], [], []
            for elite in iteration_list:
                for pt in elite.points:
                    iter_x.append(pt[0])
                    iter_y.append(pt[1])
                    iter_z.append(pt[2])
            
            if iter_x:
                all_x.extend(iter_x)
                all_y.extend(iter_y)
                all_z.extend(iter_z)
                
                iteration_data.append({
                    'x': np.array(iter_x),
                    'y': np.array(iter_y),
                    'z': np.array(iter_z),
                    'color': cmap_iters(iter_idx),
                    'iter_num': iter_idx
                })
        
        if not iteration_data:
            print("WARNING: No iteration data to plot")
            return
        
        # 7. Best trajectory (STATIC)
        if self.best_traj_ever:
            best_pts = []
            for segment in self.best_traj_ever:
                segment_np = np.array(segment)
                if segment_np.shape[0] == 3:
                    best_pts.append([segment_np[0, 0], segment_np[1, 0], segment_np[2, 0]])
                else:
                    best_pts.append([segment_np[0, 0], segment_np[0, 1], segment_np[0, 2]])
            
            if self.best_traj_ever:
                last_seg = np.array(self.best_traj_ever[-1])
                if last_seg.shape[0] == 3:
                    best_pts.append([last_seg[0, -1], last_seg[1, -1], last_seg[2, -1]])
                else:
                    best_pts.append([last_seg[-1, 0], last_seg[-1, 1], last_seg[-1, 2]])
            
            best_pts_np = np.array(best_pts)
            ax.plot(best_pts_np[:, 0], best_pts_np[:, 1], best_pts_np[:, 2],
                    'b-', linewidth=3, alpha=0.8, zorder=11, label='Best Trajectory Path')
            ax.scatter(best_pts_np[:, 0], best_pts_np[:, 1], best_pts_np[:, 2],
                       c='gold', s=120, marker='*', edgecolors='black', linewidths=1,
                       zorder=12, label='Best Contact Points')
        
        # 8. Apply 1:1:1 scaling
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
        
        # 9. Labels
        ax.set_xlabel('X (m)', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_zlabel('Z (m)', fontsize=11)
        ax.view_init(elev=25, azim=-50)
        
        if not animated:
            # Static plot: show all iterations
            for data in iteration_data:
                ax.scatter(data['x'], data['y'], data['z'],
                          c=[data['color']], s=50, alpha=0.7,
                          edgecolors='white', linewidths=0.5, zorder=5)
            
            ax.set_title('3D Contact Points Layout - All Iterations', 
                        fontsize=14, fontweight='bold')
            ax.legend(loc='upper left', fontsize=9)
            
            if created:
                plt.tight_layout()
                save_path = f'{MAIN_DIRECTORY}/plot_3d_iterations_static.png'
                fig.savefig(save_path, dpi=150)
                print(f"3D static layout saved to: {save_path}")
                plt.show()
        else:
            # Animated plot
            scatter_objects = []
            title_text = ax.text2D(0.5, 0.95, '', transform=ax.transAxes,
                                  ha='center', fontsize=12, fontweight='bold',
                                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            def init():
                """Initialize animation."""
                title_text.set_text('Iteration: 0')
                return [title_text]
            
            def update(frame):
                """Update function for each frame."""
                # Remove previous scatter objects
                for scat in scatter_objects:
                    scat.remove()
                scatter_objects.clear()
                
                # Show current and previous iterations with fading effect
                for i in range(max(0, frame - 2), frame + 1):
                    if i < len(iteration_data):
                        data = iteration_data[i]
                        alpha_val = 1.0 if i == frame else 0.3
                        size_val = 80 if i == frame else 40
                        
                        scat = ax.scatter(data['x'], data['y'], data['z'],
                                        c=[data['color']], s=size_val,
                                        alpha=alpha_val, edgecolors='white',
                                        linewidths=0.7, zorder=5)
                        scatter_objects.append(scat)
                
                title_text.set_text(f'Iteration: {iteration_data[frame]["iter_num"]}')
                
                # Slowly rotate view
                azim = -50 + (frame * 2) % 360
                ax.view_init(elev=25, azim=azim)
                
                return scatter_objects + [title_text]
            
            ax.set_title('3D Contact Points Evolution (Animated)', 
                        fontsize=14, fontweight='bold')
            ax.legend(loc='upper left', fontsize=9)
            
            anim = FuncAnimation(fig, update, init_func=init,
                               frames=len(iteration_data),
                               interval=400, blit=False, repeat=True)
            
            self._anim_3d = anim  # Store reference to prevent garbage collection
            
            
            if created:
                plt.tight_layout()
                plt.show()

def main():
    """Main execution function."""
    plotter = PlotResultCemMjumps()
    print("[INFO] Plotter initialized successfully")
    
    # Static plots
    plotter.base_plot()
    plotter.count_jump_histogram(use_last_iter_only=False)
    plotter.plot_fitness_by_iteration()
    plotter.plot_mesh_pc_traj()
    plotter.plot_2d_iterations_layout(animated=True)
    plotter.plot_3d_iterations_layout(animated=True )
    
    print("[INFO] All plots completed!")

if __name__ == "__main__":
    main()