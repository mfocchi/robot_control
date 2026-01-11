import json
from matplotlib import colors
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import os
import sys
from scipy.interpolate import griddata
from scipy.spatial import ConvexHull 
from params import *
# === CONFIGURATION ===


class PlotResultCemMjumps:
    
    def __init__(self, dir_path_terrain_points, dir_path_terrain_patches, dir_path_progress, iterations_folder,dir_path_best_log):
        
        self.iterations_folder = iterations_folder
        self.data_terrain_points = self.load_json(dir_path_terrain_points)
        self.data_terrain_patches = self.load_json(dir_path_terrain_patches)
        self.data_progress = self.load_json(dir_path_progress)
        self.data_best_log = self.load_json(dir_path_best_log)
        
        # Terrain data
        self.px = None
        self.py = None
        self.pz = None
        self.points_np = None
        self.colors_np = None
        self.patch_width = None
        self.patch_height = None
        self.num_patches = None
        self.patches_data = None
        self.patches_list = []
        self.points_t = []
        self.patches_list = []
        self._extract_terrain_data()
        
        # Solution data (from progress)
        self.iterations = []
        self.fitnesses = []
        self.n_jumps = []
        self.energies = []
        self.landing_costs = []
        self.all_points = []  # List of point lists per solution
        self._extract_solution_data()
        
        # Trajectory data from best solution
        self.best_points_np = None
        self.best_traj_segments = []
        self._extract_best_trajectory_data()
        # Derived data for 3D plotting
        self.starts_np = None
        self.ends_np = None
        self.intermediates_np = None
        self._extract_trajectory_points()
        
        self.edge_patches = []
    
    def _extract_terrain_data(self):
        """
        Estrae i dati del terreno e ricostruisce i dizionari per punti e patch.
        """
        # --- 1. GESTIONE PUNTI (da actual_point_terrain.json) ---
        
        if self.data_terrain_points and 'points' in self.data_terrain_points:
            point_list = self.data_terrain_points['points']
            
            # Liste temporanee per la conversione NumPy (utile per i plot esistenti)
            positions = []
            colors = []
            
            for p in point_list:
                # Ricreazione del dizionario richiesto per ogni punto
                p_dict = {
                    'position':   p.get('position'),
                    'color':      p.get('color'),
                    'light':      p.get('light', 1.0),
                    'size_point': p.get('size_point', 1.0),
                    'cost':       p.get('cost', 0.0)
                }
                self.points_t.append(p_dict)
                
                # Popolamento dati per NumPy
                positions.append(p_dict['position'])
                colors.append(p_dict['color'])
            
            # Aggiornamento attributi NumPy per compatibilità con plot_actual_terrain
            self.points_np = np.array(positions)
            self.colors_np = np.array(colors)
            if self.points_np.size > 0:
                self.px = self.points_np[:, 0]
                self.py = self.points_np[:, 1]
                self.pz = self.points_np[:, 2]
            
            # Estrazione posizioni speciali (opzionale)
            self.start_pos_terrain = self.data_terrain_points.get('start_position')
            self.goal_pos_terrain = self.data_terrain_points.get('goal_position')
        # --- 2. GESTIONE PATCH (da actual_patch_terrain.json) ---
        
        if self.data_terrain_patches and 'patches' in self.data_terrain_patches:
            # Metadati
            meta = self.data_terrain_patches.get('metadata', {})
            self.patch_width = meta.get('patch_width')
            self.patch_height = meta.get('patch_height')
            self.num_patches = meta.get('num_patches')
            
            raw_patches = self.data_terrain_patches.get('patches', [])
            
            for patch in raw_patches:
                points_in_patch = patch.get('points_in_patch', [])
                
                # Calcolo del costo medio (mean_cost) per la patch
                if points_in_patch:
                    costs = [pt.get('cost', 0.0) for pt in points_in_patch]
                    mean_cost = sum(costs) / len(costs)
                else:
                    mean_cost = 0.0
                
                # Ricreazione del dizionario richiesto per la patch
                patch_dict = {
                    'id':              patch.get('id'),
                    'centroid':        patch.get('centroid'),
                    'points_in_patch': points_in_patch,
                    'cost_patch':      mean_cost
                }
                self.patches_list.append(patch_dict)
            self.patches_data = self.patches_list
        else:
            print("WARNING: Dati patch non trovati.")

    def _extract_solution_data(self):
        """Extract solution data from iteration reports in iterations_folder."""
        self.iterations = []
        self.fitnesses = []
        self.n_jumps = []
        self.energies = []
        self.landing_costs = []
        self.all_points = []
        
        if not os.path.exists(self.iterations_folder):
            print(f"WARNING: Iterations folder '{self.iterations_folder}' not found.")
            return
        
        # Leggi tutti i file delle iterazioni
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        
        for filename in iter_files:
            filepath = os.path.join(self.iterations_folder, filename)
            data = self.load_json(filepath)
            
            if data is None:
                continue
            
            iteration_num = data.get('iteration', 0)
            
            # Estrai elites (formato salvato in Main_cemmulti.py)
            elites = data.get('elites', [])
            
            for elite in elites:
                self.iterations.append(iteration_num)
                self.fitnesses.append(elite.get('fitness', 0.0))
                self.n_jumps.append(elite.get('n_jumps', 0))
                
                # consumed_energy e landing_cost potrebbero non essere presenti nel formato attuale
                self.energies.append(elite.get('consumed_energy', 0.0))
                self.landing_costs.append(elite.get('landing_cost', 0.0))
                
                # Estrai i punti della traiettoria
                points = elite.get('points', [])
                self.all_points.append(points)
        
        # Converti in numpy arrays per compatibilità
        self.iterations = np.array(self.iterations) if self.iterations else np.array([])
        self.fitnesses = np.array(self.fitnesses) if self.fitnesses else np.array([])
        self.n_jumps = np.array(self.n_jumps) if self.n_jumps else np.array([])
        self.energies = np.array(self.energies) if self.energies else np.array([])
        self.landing_costs = np.array(self.landing_costs) if self.landing_costs else np.array([])
        
        print(f"[INFO] Extracted {len(self.fitnesses)} solutions from {len(iter_files)} iteration files.")
        
    def _extract_trajectory_points(self):
        """Extract start, end, and intermediate points from solutions."""
        starts = []
        ends = []
        intermediates = []
        
        for pts in self.all_points:
            if not pts:
                continue
            # pts può essere una lista di liste o numpy arrays
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
        """Estrae i dati dal file best_trajectory_log.json."""
        if self.data_best_log:
            pts = self.data_best_log.get('best_jump_log_points', [])
            if pts:
                self.best_points_np = np.array(pts)
            
            # La traiettoria è una lista di array (segmenti tra i salti)
            traj = self.data_best_log.get('best_trajectory', [])
            self.best_traj_segments = [np.array(t) for t in traj if t is not None]
            print(f"[INFO] Extracted best trajectory with {len(self.best_traj_segments)} segments.")
    
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
            
    def plot_terrain_patches(self, ax=None):
            """
            Visualizza il terreno suddiviso in patch, ognuna con un colore diverso.
            """
            if not self.patches_data:
                print("WARNING: Nessun dato sulle patch disponibile in self.patches_data.")
                return

            created_fig = False
            if ax is None:
                fig = plt.figure(figsize=(12, 10))
                ax = fig.add_subplot(111, projection='3d')
                created_fig = True
            else:
                fig = ax.figure

            # Scegliamo una colormap che offra molti colori distinti (es. tab20 o rainbow)
            cmap = plt.get_cmap('tab20')
            num_patches = len(self.patches_data)

            print(f"Plotting {num_patches} patches...")

            for i, patch in enumerate(self.patches_data):
                # Get centroid from patch data (the saved format uses 'centroid')
                centroid = patch.get('centroid', None)
                if centroid is None:
                    continue
                
                centroid_np = np.array(centroid)
                
                # Assegniamo un colore basato sull'indice della patch
                color = cmap(i % 20) 

                ax.scatter(
                    centroid_np[0],
                    centroid_np[1],
                    centroid_np[2],
                    c=[color],
                    s=50,
                    alpha=0.9,
                    edgecolors='black',
                    linewidths=0.5,
                    marker='s'
                )
                
                # Optionally add patch ID label
                ax.text(centroid_np[0], centroid_np[1], centroid_np[2], 
                       f"{patch.get('id', i)}", fontsize=6, ha='center')

            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(f"Terrain Patches Visualization ({num_patches} patches)")

            # Se abbiamo poche patch, aggiungiamo una legenda (opzionale)
            if num_patches < 15:
                ax.legend([f"Patch {i}" for i in range(num_patches)], loc='best', fontsize='small')

            if created_fig:
                plt.savefig(f'{MAIN_DIRECTORY}/plot_terrain_patches.png', dpi=150)
                plt.show()
    
    def plot_fitness_by_iteration(self, ax=None):
        """
        graph 1: Fitness of top 100 solutions vs Iteration.
        """
        if len(self.fitnesses) == 0:
            print("WARNING: No fitness data available for plotting.")
            return None
            
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
        if len(self.fitnesses) > 0:
            best_idx = np.argmax(self.fitnesses)
            ax.annotate(f'Best: {self.fitnesses[best_idx]:.2f}', 
                        xy=(self.iterations[best_idx], self.fitnesses[best_idx]), 
                        xytext=(self.iterations[best_idx], self.fitnesses[best_idx] + (max(self.fitnesses)-min(self.fitnesses))*0.05),
                        arrowprops=dict(facecolor='red', shrink=0.05))

        fig.tight_layout()
        
        if created_fig:            
            fig.savefig(f'{MAIN_DIRECTORY}/plot_1_fitness_iterazione.png')
            plt.show()
        
        return scatter
        
    def plot_jumps_histogram(self, ax=None):
        """
        Graph 2: Histogram that represent which type of jumps is most used.
        """
        if len(self.n_jumps) == 0:
            print("WARNING: No jump data available for plotting.")
            return
            
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
            fig.savefig(f'{MAIN_DIRECTORY}/plot_2_istogramma_salti.png')
            plt.show()
        
    def plot_energy_vs_cost(self, ax=None):
        """
        Graph 3: Consumed Energy vs Landing Cost
        """
        if len(self.energies) == 0 or len(self.landing_costs) == 0:
            print("WARNING: No energy/cost data available for plotting.")
            return
            
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
            fig.savefig(f'{MAIN_DIRECTORY}/plot_3_energy_vs_cost.png')
            plt.show()
    
    def plot_3d_scenario(self, ax=None):    
        """
        Plot 4: Complete 3D visualization.
        """
        if self.px is None or len(self.px) == 0:
            print("WARNING: No terrain data available for 3D plotting.")
            return
            
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
        
        if self.starts_np.size > 0 and len(self.starts_np.shape) > 1:
            ax.scatter(self.starts_np[0,0], self.starts_np[0,1], self.starts_np[0,2], c='lime', s=100, marker='^', label='Start P0', depthshade=False)
        
        if self.ends_np.size > 0 and len(self.ends_np.shape) > 1:
            ax.scatter(self.ends_np[0,0], self.ends_np[0,1], self.ends_np[0,2], c='red', s=100, marker='X', label='Goal PF', depthshade=False)
            
        if self.intermediates_np.size > 0 and len(self.intermediates_np.shape) > 1:
            ax.scatter(self.intermediates_np[:,0], self.intermediates_np[:,1], self.intermediates_np[:,2], c='blue', s=20, marker='o', alpha=0.6, label='Jump Points')
            
        ax.set_title(f"Jump Visualization on Terrain", fontsize=14)
        ax.set_xlabel('X (Height/Horizontal)')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        
        ax.view_init(elev=25, azim=-45)
        
        plt.tight_layout()
        if created_fig:
            plt.savefig(f'{MAIN_DIRECTORY}/plot_3d_jumps_map.png', dpi=150)
            plt.show()
    
    def edge_patch(self, plot=True):
        self.edge_patches = [] # Reset della lista
        
        if plot:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')

        for patch in self.patches_list:
            points_data = patch.get('points_in_patch', [])
            if len(points_data) < 3:
                continue
            
            # 1. Estraiamo le coordinate
            coords = np.array([p['position'] for p in points_data]) 
            
            try:
                # 2. Calcoliamo l'involucro basandoci solo su Y e Z (il piano della mappa)
                # ConvexHull trova i vertici esterni e li ordina automaticamente
                points_2d = coords[:, [1, 2]] 
                hull = ConvexHull(points_2d)
                
                # 3. Selezioniamo i punti che formano il contorno (mantengono la X originale)
                sorted_pts = coords[hull.vertices]
                
                # Chiudiamo il poligono collegando l'ultimo punto al primo
                sorted_pts = np.vstack([sorted_pts, sorted_pts[0]])

                # Salvataggio dati
                self.edge_patches.append({
                    'position': sorted_pts.tolist(),
                    'color': [0, 0, 0]
                })

                if plot:
                    ax.plot(sorted_pts[:, 0], sorted_pts[:, 1], sorted_pts[:, 2], 
                            c='black', linewidth=1.5, alpha=0.8)
            
            except Exception:
                # Gestisce casi in cui i punti sono collineari o insufficienti
                continue

        if plot:
            ax.view_init(elev=25, azim=-45)
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
            outfile = f'{MAIN_DIRECTORY}/plot_yz_density_colored.png'
            plt.savefig(outfile, dpi=150)
            print(f"Saved: {outfile}")
            plt.show()
        
    def plot_evolution_fitness(self):
        if not os.path.exists(self.iterations_folder):
            print(colored("[WARNING] Iterations folder not found", "yellow"))
            return
        
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        
        if not iter_files:
            print(colored("[WARNING] No iteration files found", "yellow"))
            return
        
        iterations = []
        best_values = []
        best_ever_values = [] # Lista per il fitness storico
        
        current_best_ever = float('-inf') # Inizializziamo al valore minimo possibile
        
        for filename in iter_files:
            data = self.load_json(os.path.join(self.iterations_folder, filename))
            if data:
                iter_num = data.get('iteration', 0)
                best_fitness = data.get('best_fitness_this_iter', 0.0)
                
                # Calcolo del best fitness ever se non è già presente nel JSON
                # Se il tuo JSON ha già una chiave 'best_fitness_ever', usa:
                # current_best_ever = data.get('best_fitness_ever', best_fitness)
                if best_fitness > current_best_ever:
                    current_best_ever = best_fitness
                
                iterations.append(iter_num)
                best_values.append(best_fitness)
                best_ever_values.append(current_best_ever)
        
        if not iterations:
            print(colored("[WARNING] No valid data found in iteration files", "yellow"))
            return
        
        plt.figure(figsize=(12, 7))
        
        # Plot del Best Fitness della singola iterazione
        plt.plot(iterations, best_values, linewidth=2, marker='o', 
                markersize=4, color='#2E86AB', alpha=0.6, label='Best Fitness (Iter)')
        
        # Plot del Best Fitness Ever (in Verde)
        plt.plot(iterations, best_ever_values, linewidth=3, color='#27AE60', 
                label='Best Fitness Ever', zorder=5)
        
        plt.xlabel('Iteration', fontsize=12, fontweight='bold')
        plt.ylabel('Fitness Value', fontsize=12, fontweight='bold')
        plt.title('CEM Fitness Convergence', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, linestyle='--')
        plt.legend(fontsize=10, loc='lower right')
        
        plt.tight_layout()
        
        save_path = f'{MAIN_DIRECTORY}/plot_evolution_fitness.png'
        plt.savefig(save_path, dpi=150)
        print(f"Saved: {save_path}")
        plt.show()


    def plot_evolution_std_dev_cem(self):
        history_file = f"{MAIN_DIRECTORY}/cem_iteration_history.json"
        
        if not os.path.exists(history_file):
            print(colored("[WARNING] CEM iteration history file not found", "yellow"))
            return
        
        with open(history_file, 'r') as f:
            history_data = json.load(f)
        
        iteration_history = history_data.get('iteration_history', [])
        
        if not iteration_history:
            print(colored("[WARNING] No history data available in file", "yellow"))
            return
        
        iterations = [h['iteration'] for h in iteration_history]
        std_history = np.array([h['std_devs'] for h in iteration_history])
        
        plt.figure(figsize=(12, 6))
        n_continuous = std_history.shape[1]
        colors = plt.cm.tab20(np.linspace(0, 1, n_continuous))
        
        for i in range(n_continuous):
            patch_num = i // 2 + 1
            coord = 'Y' if i % 2 == 0 else 'Z'
            label = f'Patch {patch_num} - {coord}'
            plt.plot(iterations, std_history[:, i], 
                    label=label, linewidth=2, color=colors[i], marker='.')
        
        plt.xlabel('Iteration', fontsize=12, fontweight='bold')
        plt.ylabel('Standard Deviation', fontsize=12, fontweight='bold')
        plt.title('CEM Continuous Variables - Standard Deviation Convergence', 
                fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3, linestyle='--')
        plt.legend(fontsize=9, loc='best', ncol=2)
        plt.tight_layout()
        
        save_path = f'{MAIN_DIRECTORY}/plot_evolution_std_dev.png'
        plt.savefig(save_path, dpi=150)
        print(f"Saved: {save_path}")
        plt.show()

    
    def plot_3d_scenario_iterations(self, ax=None, animated=False, save_animation=False):
        
        interval = 500  # milliseconds between frames
        
        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')
            created_fig = True
        else:
            fig = ax.figure
        
        # 1. Plot Terrain (light gray background)
        if self.px is not None and len(self.px) > 0:
            ax.scatter(self.px, self.py, self.pz, c='gray', s=1, alpha=0.15, label='Terrain Cloud')
        
        # 2. Plot Patch Contours
        self.edge_patch(plot=False)
        if hasattr(self, 'edge_patches') and self.edge_patches:
            for i, edge in enumerate(self.edge_patches):
                pts = np.array(edge['position'])
                lbl = 'Patch Contour' if i == 0 else ""
                ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], c='black', 
                        linewidth=1.0, alpha=0.5, zorder=2, label=lbl)

        # 3. Plot Start and Goal (persistent markers)
        if self.start_pos_terrain:
            sp = np.array(self.start_pos_terrain)
            ax.scatter(sp[0], sp[1], sp[2], c='lime', s=200, marker='^', 
                    label='Start P0', zorder=5, edgecolors='darkgreen', linewidths=2)
        if self.goal_pos_terrain:
            gp = np.array(self.goal_pos_terrain)
            ax.scatter(gp[0], gp[1], gp[2], c='red', s=200, marker='X', 
                    label='Goal PF', zorder=5, edgecolors='darkred', linewidths=2)

        # 4. Load iteration data
        if not os.path.exists(self.iterations_folder):
            print(f"ERROR: Iterations folder '{self.iterations_folder}' not found.")
            return
        
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        cmap = plt.cm.get_cmap('tab10' if len(iter_files) <= 10 else 'rainbow', len(iter_files))
        
        if not animated:
            # --- STATIC MODE: Show all iterations at once ---
            for idx, filename in enumerate(iter_files):
                filepath = os.path.join(self.iterations_folder, filename)
                data = self.load_json(filepath)
                if data is None: 
                    continue
                
                # Extract elites from current iteration
                elites = data.get('elites', [])
                
                iter_intermediates = []
                for elite in elites:
                    pts = elite.get('points', [])
                    if len(pts) > 2:  # Only intermediate points (exclude start/end)
                        iter_intermediates.extend(pts[1:-1])
                
                if iter_intermediates:
                    iter_pts = np.array(iter_intermediates)
                    iter_num = data.get('iteration', filename.split('_')[-1].split('.')[0])
                    # Label only first and last iteration to avoid cluttering legend
                    lbl = f"Iter {iter_num}" if idx in [0, len(iter_files)-1] else ""
                    ax.scatter(iter_pts[:, 0], iter_pts[:, 1], iter_pts[:, 2], 
                            c=[cmap(idx)], s=80, alpha=0.9, label=lbl, zorder=4, 
                            edgecolors='white', linewidths=0.7, marker='o')
            
            ax.set_title("Evolution of Jumps with Patch Layout", fontsize=14, weight='bold')
            ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize=10)

        else:
            # --- ANIMATED MODE: Show iterations progressively ---
            iteration_data = []
            for idx, filename in enumerate(iter_files):
                data = self.load_json(os.path.join(self.iterations_folder, filename))
                if data:
                    elites = data.get('elites', [])
                    pts_list = []
                    for elite in elites:
                        pts = elite.get('points', [])
                        if len(pts) > 2: 
                            pts_list.extend(pts[1:-1])
                    if pts_list:
                        iter_num = data.get('iteration', filename.split('_')[-1].split('.')[0])
                        iteration_data.append({
                            'points': np.array(pts_list), 
                            'color': cmap(idx), 
                            'num': iter_num
                        })

            scatter_objects = []
            title_text = ax.text2D(0.5, 0.95, '', transform=ax.transAxes, ha='center', 
                                fontsize=12, weight='bold', bbox=dict(boxstyle='round', 
                                facecolor='wheat', alpha=0.5))

            def update(frame):
                # Clear previous scatter objects
                for s in scatter_objects: 
                    s.remove()
                scatter_objects.clear()
                
                if frame < len(iteration_data):
                    curr = iteration_data[frame]
                    scat = ax.scatter(curr['points'][:, 0], curr['points'][:, 1], curr['points'][:, 2], 
                                    c=[curr['color']], s=80, edgecolors='white', linewidths=0.7, 
                                    zorder=4, alpha=0.95, marker='o')
                    scatter_objects.append(scat)
                    
                    title_text.set_text(f"Evolution - Iteration: {curr['num']}")
                
                return scatter_objects + [title_text]

            anim = FuncAnimation(fig, update, frames=len(iteration_data), 
                            interval=interval, blit=False, repeat=True)
            
            if save_animation:
                try:
                    anim.save(f'{MAIN_DIRECTORY}/animation_iterations.gif', writer='pillow', dpi=100)
                except Exception as e:
                    print(f"WARNING: Could not save animation: {e}")
            
        # 5. Set labels and view
        ax.set_xlabel('X (Altezza)', fontsize=11, weight='bold')
        ax.set_ylabel('Y', fontsize=11, weight='bold')
        ax.set_zlabel('Z', fontsize=11, weight='bold')
        ax.view_init(elev=25, azim=-45)
        ax.grid(True, alpha=0.3)
        
        if created_fig:
            plt.tight_layout()
            plt.savefig(f'{MAIN_DIRECTORY}/plot_3d_iterations.png', dpi=150)
            
            plt.show()

    def plot_2d_iterations_layout(self, animated=False, interval=500, save_animation=False):
        
        if not self.patches_data:
            print("WARNING: Patch data missing.")
            return

        fig, ax = plt.subplots(figsize=(16, 12))

        # 1. DRAW BACKGROUND (Patches with cost-based coloring)
        costs = [p.get('cost_patch', 0) for p in self.patches_data]
        norm_cost = colors.Normalize(vmin=min(costs), vmax=max(costs))
        cmap_terrain = plt.get_cmap('Greys')  # Gray scale for terrain

        for patch in self.patches_data:
            points_in_patch = patch.get('points_in_patch', [])
            if len(points_in_patch) >= 3:
                coords_2d = np.array([[p['position'][1], p['position'][2]] for p in points_in_patch])
                try:
                    hull = ConvexHull(coords_2d)
                    face_color = cmap_terrain(norm_cost(patch.get('cost_patch', 0)))
                    poly = plt.Polygon(coords_2d[hull.vertices], color=face_color, 
                                    alpha=0.3, ec='black', lw=0.5)
                    ax.add_patch(poly)
                except:
                    pass

        # 2. LOAD ITERATION DATA
        if not os.path.exists(self.iterations_folder):
            print(f"ERROR: Folder {self.iterations_folder} not found.")
            return
        
        iter_files = sorted([f for f in os.listdir(self.iterations_folder) if f.endswith('.json')])
        cmap_iters = plt.get_cmap('jet', len(iter_files))

        # Plot Start and Goal
        if self.start_pos_terrain:
            ax.scatter(self.start_pos_terrain[1], self.start_pos_terrain[2], 
                    c='lime', s=150, marker='^', zorder=10, label='Start')
        if self.goal_pos_terrain:
            ax.scatter(self.goal_pos_terrain[1], self.goal_pos_terrain[2], 
                    c='red', s=150, marker='X', zorder=10, label='Goal')

        iteration_data = []
        for idx, filename in enumerate(iter_files):
            data = self.load_json(os.path.join(self.iterations_folder, filename))
            if not data: 
                continue
            
            elites = data.get('elites', [])
            
            pts_2d = []
            for elite in elites:
                p_list = elite.get('points', [])
                if len(p_list) > 2:
                    # Extract Y (index 1) and Z (index 2)
                    pts_2d.extend([[p[1], p[2]] for p in p_list[1:-1]])
            
            if pts_2d:
                iteration_data.append({
                    'points': np.array(pts_2d),
                    'color': cmap_iters(idx),
                    'num': data.get('iteration', filename.split('_')[-1].split('.')[0])
                })

        # 3. STATIC OR ANIMATED MODE
        if not animated:
            # Static: show all iterations at once
            for i, d in enumerate(iteration_data):
                lbl = f"Iter {d['num']}" if i % max(1, len(iteration_data)//5) == 0 else ""
                ax.scatter(d['points'][:, 0], d['points'][:, 1], color=[d['color']], 
                        s=30, alpha=0.6, label=lbl, edgecolors='white', lw=0.3)
            ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
            ax.set_title("2D Evolution of Jumps (YZ Plane)")
        else:
            # Animated: show iterations progressively
            scatter_objs = []
            title_text = ax.text(0.5, 1.05, '', transform=ax.transAxes, ha='center', fontweight='bold')

            def update(frame):
                for s in scatter_objs: 
                    s.remove()
                scatter_objs.clear()
                
                # Show last 5 iterations to see the "flow"
                start_f = max(0, frame - 5)
                for i in range(start_f, frame + 1):
                    d = iteration_data[i]
                    age = frame - i
                    alpha = max(0.2, 1.0 - (age * 0.15))
                    s = ax.scatter(d['points'][:, 0], d['points'][:, 1], color=[d['color']], 
                                s=40, alpha=alpha, edgecolors='black', lw=0.5)
                    scatter_objs.append(s)
                
                title_text.set_text(f"Iteration: {iteration_data[frame]['num']}")
                return scatter_objs + [title_text]

            anim = FuncAnimation(fig, update, frames=len(iteration_data), 
                                interval=interval, blit=False, repeat=True)
            if save_animation:
                try:
                    anim.save(f'{MAIN_DIRECTORY}/animation_2d_iterations.gif', writer='pillow', dpi=100)
                except Exception as e:
                    print(f"WARNING: Could not save animation: {e}")

        ax.set_xlabel("Y", fontsize=12)
        ax.set_ylabel("Z", fontsize=12)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        
    
    def plot_mesh_traj(self, ax=None):
        """
        Visualizza la traiettoria migliore completa (punti di salto + parabole) 
        sopra il terreno.
        """
        if self.best_points_np is None or not self.best_traj_segments:
            print("WARNING: Dati traiettoria migliore non trovati.")
            return

        created_fig = False
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')
            created_fig = True
        else:
            fig = ax.figure

        # 1. Plot Terreno di sfondo (leggero)
        if self.px is not None:
            ax.scatter(self.px, self.py, self.pz, c='gray', s=1, alpha=0.1, label='Terrain')

        # 2. Plot dei segmenti della traiettoria (le parabole dei salti)
        for i, segment in enumerate(self.best_traj_segments):
            label = "Best Trajectory" if i == 0 else ""
            ax.plot(segment[:, 0], segment[:, 1], segment[:, 2], 
                    color='blue', linewidth=2.5, zorder=10, label=label)

        # 3. Plot dei punti di landing (punti discreti scelti)
        ax.scatter(self.best_points_np[:, 0], self.best_points_np[:, 1], self.best_points_np[:, 2], 
                   c='yellow', s=60, edgecolors='black', marker='o', zorder=11, label='Landing Points')

        # 4. Highlight Start (Verde) e Goal (Rosso)
        ax.scatter(self.best_points_np[0, 0], self.best_points_np[0, 1], self.best_points_np[0, 2], 
                   c='lime', s=150, marker='^', edgecolors='black', zorder=12, label='Start')
        ax.scatter(self.best_points_np[-1, 0], self.best_points_np[-1, 1], self.best_points_np[-1, 2], 
                   c='red', s=150, marker='X', edgecolors='black', zorder=12, label='Goal')

        ax.set_title("Best Trajectory Visualization", fontsize=14, fontweight='bold')
        ax.set_xlabel("X (Altezza)")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend()
        
        # Imposta una vista favorevole
        ax.view_init(elev=30, azim=-60)

        if created_fig:
            plt.tight_layout()
            # Assicurati che MAIN_DIRECTORY sia definito (importato da params)
            plt.savefig(f'{MAIN_DIRECTORY}/best_trajectory_mesh_plot.png', dpi=150)
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
        # sc4=self.plot_3d_scenario(ax4)
        
        fig.colorbar(sc1, ax=ax2, label='Fitness Value')
        
        
        plt.tight_layout()
        plt.show()
        


# === MAIN ===
def main():
    plot_result_cem_mjumps = PlotResultCemMjumps(
        FILE_TERRAIN_POINTS, 
        FILE_TERRAIN_PATCHES, 
        FILE_PROGRESS, 
        ITERATIONS_FOLDER
    )
    print("ok all is ready")
    plot_result_cem_mjumps.plot_terrain_patches()
    plot_result_cem_mjumps.plot_actual_terrain()
    plot_result_cem_mjumps.plot_all_in_one()
    plot_result_cem_mjumps.plot_density_map()
    plot_result_cem_mjumps.plot_3d_scenario_iterations(animated=True)
    plot_result_cem_mjumps.plot_2d_iterations_layout(animated=True)
    plot_result_cem_mjumps.plot_evolution_fitness()
    plot_result_cem_mjumps.plot_evolution_std_dev_cem() 
    plot_result_cem_mjumps.plot_mesh_traj()
if __name__ == "__main__":
    main()