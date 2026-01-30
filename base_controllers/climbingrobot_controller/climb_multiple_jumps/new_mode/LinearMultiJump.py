import os
import sys
import time
import json
import numpy as np
from termcolor import colored
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D

from SingleJump import SingleJump, initialize_terrain_data
from base_controllers.components.terrain_manager import TerrainManager
from params import *


class LinearMultiJumpParams:
    """Handles multi-jump trajectory planning with linear discretization."""
    
    def __init__(self, p0, pf, terrain_manager, point_clouds, patches, num_jumps=3):
        self.p0 = np.array(p0)
        self.pf = np.array(pf)
        self.num_jumps = num_jumps
        
        self.terrain_manager = terrain_manager
        self.point_clouds = point_clouds
        self.patches = patches
        
        self.waypoints = []
        self.trajectories = []
        self.fitness_values = []
        self.jump_results = []
        
        self.p0[0] = self.terrain_manager.wall_surface_eval(
            self.p0[2], self.p0[1], 
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, 
            self.terrain_manager.mesh_z
        )
        
        self.pf[0] = self.terrain_manager.wall_surface_eval(
            self.pf[2], self.pf[1], 
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, 
            self.terrain_manager.mesh_z
        )
        
    def linear_discretization(self):
        """Generate waypoints with linear interpolation, projected onto terrain surface."""
        print(colored(f"Generating {self.num_jumps + 1} waypoints...", "cyan"))
        
        waypoints = [self.p0.copy()]
        
        for i in range(1, self.num_jumps):
            alpha = i / self.num_jumps
            intermediate_point = (1 - alpha) * self.p0 + alpha * self.pf
            
            # Project onto terrain surface
            intermediate_point[0] = self.terrain_manager.wall_surface_eval(
                intermediate_point[2], 
                intermediate_point[1],
                self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y,
                self.terrain_manager.mesh_z
            )
            
            waypoints.append(intermediate_point.copy())
        
        waypoints.append(self.pf.copy())
        print(f"Generated {len(waypoints)} waypoints (including start and goal)\n")
        
        return waypoints
    
    def execute_linear_multi_jump(self):
        """Execute multi-jump trajectory between waypoints."""
        print(colored("=== Executing Linear Multi-Jump ===\n", "cyan", attrs=['bold']))
        
        self.waypoints = self.linear_discretization()
        total_fitness = 0.0
        start_time = time.time()
        
        for i in range(len(self.waypoints) - 1):
            print(colored(f"Jump {i+1}/{len(self.waypoints)-1}: ", "yellow"), end='')
            print(f"{self.waypoints[i]} → {self.waypoints[i+1]}")
            
            jump_optimizer = SingleJump(
                self.waypoints[i], 
                self.waypoints[i+1], 
                self.terrain_manager,
                self.point_clouds,
                self.patches
            )
            
            try:
                fitness, p0_adj, pf_adj, trajectory, result = jump_optimizer.execute_jump()
                
                self.trajectories.append(trajectory)
                self.fitness_values.append(fitness)
                self.jump_results.append({
                    'jump_number': i + 1,
                    'p0': self.waypoints[i].tolist(),
                    'pf': self.waypoints[i+1].tolist(),
                    'p0_adjusted': p0_adj.tolist(),
                    'pf_adjusted': pf_adj.tolist(),
                    'fitness': float(fitness),
                    'problem_solved': int(result['problem_solved']),
                    'consumed_energy': float(result['consumed_energy']),
                    'jump_duration': float(result['Tf'])
                })
                
                total_fitness += fitness
                print(colored(f"  ✓ Fitness: {fitness:.2f}\n", "green"))
                
            except Exception as e:
                print(colored(f"  ✗ Failed: {str(e)}\n", "red"))
                self.trajectories.append(None)
                self.fitness_values.append(-1000.0)
                total_fitness -= 1000.0
        
        elapsed_time = time.time() - start_time
        
        print(colored("=== Summary ===", "cyan", attrs=['bold']))
        print(f"Jumps: {len(self.trajectories)} | Total fitness: {total_fitness:.2f} | "
              f"Avg: {total_fitness/len(self.trajectories):.2f} | Time: {elapsed_time:.1f}s\n")
        
        return {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'num_jumps': len(self.trajectories),
                'total_computation_time': elapsed_time
            },
            'waypoints': [wp.tolist() for wp in self.waypoints],
            'fitness_values': self.fitness_values,
            'total_fitness': total_fitness,
            'average_fitness': total_fitness / len(self.trajectories),
            'jump_details': self.jump_results
        }
        
    def plot_multi_jump_trajectory(self, show_plot=True):
        """Plot complete multi-jump trajectory with terrain and correct scaling."""
        x_pts = np.array([p['position'][0] for p in self.point_clouds.points_t])
        y_pts = np.array([p['position'][1] for p in self.point_clouds.points_t])
        z_pts = np.array([p['position'][2] for p in self.point_clouds.points_t])
        colors = np.array([p['color'] for p in self.point_clouds.points_t])
        sizes = np.array([p['size_point'] for p in self.point_clouds.points_t])
        
        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. Terrain (Point Cloud)
        ax.scatter(x_pts, y_pts, z_pts, c=colors, s=sizes, alpha=0.1, label='Terrain', zorder=1)
        
        # 2. Waypoints
        for i, point in enumerate(self.waypoints):
            if i == 0:
                ax.scatter(point[0], point[1], point[2], c='lime', s=200, marker='o', 
                          edgecolors='darkgreen', linewidths=3, label='Start', zorder=10)
            elif i == len(self.waypoints) - 1:
                ax.scatter(point[0], point[1], point[2], c='red', s=200, marker='o', 
                          edgecolors='darkred', linewidths=3, label='Goal', zorder=10)
            else:
                ax.scatter(point[0], point[1], point[2], c='orange', s=140, marker='o', 
                          edgecolors='darkorange', linewidths=2, 
                          label='Waypoint' if i == 1 else '', zorder=9)
                ax.text(point[0], point[1], point[2], f'  W{i}', 
                       fontsize=9, color='black', weight='bold')
        
        # 3. Planned path line (Dashed)
        wps = np.array(self.waypoints)
        ax.plot(wps[:, 0], wps[:, 1], wps[:, 2], 'k--', linewidth=1.5, 
               alpha=1.0, label='Planned Path', zorder=3)
        
        # 4. Trajectories (Jump segments)
        traj_colors = plt.cm.plasma(np.linspace(0, 1, len(self.trajectories)))
        
        for i, traj in enumerate(self.trajectories):
            if traj is not None and traj.size > 0 and traj.ndim == 2 and traj.shape[0] == 3:
                ax.plot(traj[0, :], traj[1, :], traj[2, :], color=traj_colors[i], linewidth=3.0, 
                       label=f'Jump {i+1} ({self.fitness_values[i]:.1f})' if i < 5 else '',
                       alpha=0.9, zorder=5)
                
                # Direction arrow
                mid = traj.shape[1] // 2
                if 0 < mid < traj.shape[1] - 1:
                    d = traj[:, mid+1] - traj[:, mid]
                    ax.quiver(traj[0, mid], traj[1, mid], traj[2, mid],
                             d[0], d[1], d[2], color=traj_colors[i], 
                             length=0.1, normalize=True, # Fixed arrow params
                             arrow_length_ratio=0.3, linewidth=2, zorder=6)

        # --- LOGICA DI SCALATURA (Equal Aspect Ratio) ---
        # Raccogliamo tutti i dati esistenti per calcolare il bounding box globale
        all_x = [x_pts, wps[:, 0]]
        all_y = [y_pts, wps[:, 1]]
        all_z = [z_pts, wps[:, 2]]

        for t in self.trajectories:
            if t is not None:
                all_x.append(t[0, :])
                all_y.append(t[1, :])
                all_z.append(t[2, :])

        flat_x = np.concatenate(all_x)
        flat_y = np.concatenate(all_y)
        flat_z = np.concatenate(all_z)

        max_range = np.array([
            flat_x.max() - flat_x.min(), 
            flat_y.max() - flat_y.min(), 
            flat_z.max() - flat_z.min()
        ]).max() / 2.0

        mid_x = (flat_x.max() + flat_x.min()) * 0.5
        mid_y = (flat_y.max() + flat_y.min()) * 0.5
        mid_z = (flat_z.max() + flat_z.min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        # ------------------------------------------------

        # Styling
        ax.set_xlabel('X (m) - Height', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
        ax.set_zlabel('Z (m) - Depth', fontsize=12, fontweight='bold')
        
        title = (f'Linear Multi-Jump Trajectory (Scaled)\n'
                f'Jumps: {len(self.trajectories)} | '
                f'Total Fitness: {sum(self.fitness_values):.2f}')
        ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
        
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper left', fontsize=9, framealpha=0.9)
        
        ax.view_init(elev=25, azim=50)
        ax.grid(True, alpha=0.1, linestyle='--')
        plt.tight_layout()
        
        if show_plot:
            plt.show()
    
    def plot_multi_jump_mesh(self, show_plot=True):
        """Plot complete multi-jump trajectory with a solid brown terrain mesh."""
        import matplotlib.patches as mpatches

        # 1. Recupero dati mesh dal terrain_manager
        X = self.terrain_manager.mesh_x
        Y = self.terrain_manager.mesh_y
        Z = self.terrain_manager.mesh_z

        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection='3d')

        # 2. Plot della superficie (Mesh Marrone)
        # Usiamo 'sienna' e shade=True per evidenziare i rilievi (torri/creste)
        surf = ax.plot_surface(X, Y, Z, 
                               color='sienna', 
                               edgecolor='none', 
                               alpha=0.5, 
                               linewidth=0, 
                               antialiased=True,
                               shade=True)

        # 3. Plot dei Waypoints
        for i, point in enumerate(self.waypoints):
            if i == 0:
                ax.scatter(point[0], point[1], point[2], c='lime', s=200, marker='o', 
                          edgecolors='black', linewidths=2, label='Start', zorder=10)
            elif i == len(self.waypoints) - 1:
                ax.scatter(point[0], point[1], point[2], c='red', s=200, marker='o', 
                          edgecolors='black', linewidths=2, label='Goal', zorder=10)
            else:
                ax.scatter(point[0], point[1], point[2], c='blue', s=100, marker='o', 
                          edgecolors='black', linewidths=1.5, 
                          label='Waypoint' if i == 1 else '', zorder=9)

        # 4. Traiettorie dei Salti (Rosso acceso)
        for i, traj in enumerate(self.trajectories):
            if traj is not None and traj.size > 0:
                ax.plot(traj[0, :], traj[1, :], traj[2, :], 
                        color='blue', linewidth=2.0, 
                        label=f'Jump {i+1}' if i < 3 else '',
                        alpha=1.0, zorder=15)

        # 5. Fix Legenda e Styling
        terrain_proxy = mpatches.Patch(color='sienna', alpha=0.5, label='Terrain Wall')
        handles, labels = ax.get_legend_handles_labels()
        handles.insert(0, terrain_proxy)
        ax.legend(handles=handles, loc='upper left')

        ax.set_xlabel('X (Height/Depth)', fontweight='bold')
        ax.set_ylabel('Y (Horizontal)', fontweight='bold')
        ax.set_zlabel('Z (Vertical)', fontweight='bold')
        ax.set_title('Multi-Jump Trajectory on Brown Mesh', fontsize=14, fontweight='bold')

        # 6. Equalizzazione assi (Aspect Ratio 1:1:1)
        all_x = np.concatenate([X.flatten()] + [t[0,:] for t in self.trajectories if t is not None])
        all_y = np.concatenate([Y.flatten()] + [t[1,:] for t in self.trajectories if t is not None])
        all_z = np.concatenate([Z.flatten()] + [t[2,:] for t in self.trajectories if t is not None])

        max_range = np.array([all_x.max()-all_x.min(), all_y.max()-all_y.min(), all_z.max()-all_z.min()]).max() / 2.0
        mid_x, mid_y, mid_z = (all_x.max()+all_x.min())/2, (all_y.max()+all_y.min())/2, (all_z.max()+all_z.min())/2

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        ax.view_init(elev=25, azim=45)
        plt.tight_layout()

        if show_plot:
            plt.show()

def main():
    print(colored("="*60, "cyan"))
    print(colored("  LINEAR MULTI-JUMP TRAJECTORY OPTIMIZER  ", "cyan", attrs=['bold']))
    print(colored("="*60 + "\n", "cyan"))
    
    # Initialize terrain
    print(colored("[1/3] Initializing Terrain...", "yellow"))
    terrain_manager = TerrainManager()
    point_clouds, patches, cost_grid = initialize_terrain_data ()
        
    print(colored("✓ Terrain ready\n", "green"))
    
    # Configure multi-jump
    print(colored("[2/3] Configuring Multi-Jump...", "yellow"))
    print(f"Start: {P0_INIT} | Goal: {PF_INIT} | Jumps: {MAX_JUMP}")
    
    multi_jump = LinearMultiJumpParams(
        P0_INIT, PF_INIT, terrain_manager, point_clouds, patches, num_jumps=MAX_JUMP
    )
    print(colored("✓ Configuration complete\n", "green"))
    
    # Execute
    print(colored("[3/3] Executing Multi-Jump Trajectory...", "yellow"))
    results = multi_jump.execute_linear_multi_jump()
    print(colored("✓ Execution complete\n", "green"))
    
    # Plot
    multi_jump.plot_multi_jump_trajectory(show_plot=True)
    multi_jump.plot_multi_jump_mesh(show_plot=True)
    print(colored("✓ All operations completed!\n", "green", attrs=['bold']))


if __name__ == "__main__":
    main()