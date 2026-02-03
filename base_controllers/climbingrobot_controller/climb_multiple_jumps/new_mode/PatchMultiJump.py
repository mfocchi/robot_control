import os
import sys
import time
import json
import numpy as np
from termcolor import colored
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from mpl_toolkits.mplot3d import Axes3D

from SingleJump import SingleJump, initialize_terrain_data
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.utils.matlab_conversions import (
    mat_matrix2python,
    mat_vector2python,
)
from params import *


class LinearMultiJumpParams:
    """Handles multi-jump trajectory planning with patch discretization."""
    
    def __init__(self, p0, pf, terrain_manager, point_clouds, patches, patch_selected=None):
        self.p0 = np.array(p0)
        self.pf = np.array(pf)
        self.patch_selected = patch_selected if patch_selected is not None else []
        
        self.terrain_manager = terrain_manager
        self.point_clouds = point_clouds
        self.patches = patches
        
        self.waypoints = []
        self.trajectories = []
        self.fitness_values = []
        self.jump_results = []
        
        # Project p0 onto surface
        self.p0[0] = self.terrain_manager.wall_surface_eval(
            self.p0[2], self.p0[1], 
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, 
            self.terrain_manager.mesh_z
        )
        
        # Project pf onto surface
        self.pf[0] = self.terrain_manager.wall_surface_eval(
            self.pf[2], self.pf[1], 
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, 
            self.terrain_manager.mesh_z
        )
    
    def compute_waypoints_from_patches(self):
        self.waypoints = []
        
        # Start point
        p0_adj = self.p0.copy()
        p0_adj[0] = self.terrain_manager.wall_surface_eval(
            p0_adj[2], p0_adj[1],
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y,
            self.terrain_manager.mesh_z
        )
        self.waypoints.append(p0_adj.copy())
        
        # Intermediate waypoints from patch centers
        center_relative_yz = [0.5, 0.5]
        for patch_id in self.patch_selected:
            wp = self.patches.getAbsolutePoseOfPointInsidePatch(
                patch_id, center_relative_yz[0], center_relative_yz[1], scale=1.0
            ).copy()
            # Project onto surface
            wp[0] = self.terrain_manager.wall_surface_eval(
                wp[2], wp[1],
                self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y,
                self.terrain_manager.mesh_z
            )
            self.waypoints.append(wp)
        
        # Final destination
        pf_adj = self.pf.copy()
        pf_adj[0] = self.terrain_manager.wall_surface_eval(
            pf_adj[2], pf_adj[1],
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y,
            self.terrain_manager.mesh_z
        )
        self.waypoints.append(pf_adj.copy())
        
        print(colored(f"  Computed {len(self.waypoints)} waypoints from {len(self.patch_selected)} patches", "cyan"))
      
    def execute_multi_jump_in_patch(self):
        """Execute multi-jump trajectory through selected patches to final destination."""
        print(colored("=== Executing Patch-Based Multi-Jump ===\n", "cyan", attrs=['bold']))
        
        # Compute waypoints from patches
        self.compute_waypoints_from_patches()
        
        # Initialize MATLAB engine
        eng = get_matlab_engine(self.point_clouds, None, self.terrain_manager)
        local_inner_opt_params = create_inner_opt_params_copy()
        
        total_fitness = 0.0
        total_consumed_energy = 0.0
        total_landing_cost = 0.0
        all_converged = True
        start_time = time.time()
        
        num_jumps = len(self.waypoints) - 1
        
        for i in range(num_jumps):
            print(colored(f"Jump {i+1}/{num_jumps}: ", "yellow"), end="")
            
            p0_adj = self.waypoints[i].copy()
            pf_adj = self.waypoints[i + 1].copy()
            
            # Project points onto surface
            p0_adj[0] = self.terrain_manager.wall_surface_eval(
                p0_adj[2], p0_adj[1],
                self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y,
                self.terrain_manager.mesh_z
            )
            pf_adj[0] = self.terrain_manager.wall_surface_eval(
                pf_adj[2], pf_adj[1],
                self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y,
                self.terrain_manager.mesh_z
            )
            
            # Calculate liftoff normal
            liftoff_normal = self.terrain_manager.wall_normal_eval(
                p0_adj[2], p0_adj[1],
                self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y,
                self.terrain_manager.mesh_z
            )
            local_inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)
            
            try:
                # Run MATLAB optimization
                res = eng.optimize_cpp_mex(
                    matlab.double(p0_adj.tolist()),
                    matlab.double(pf_adj.tolist()),
                    Fleg_max, Fr_max, Fr_min, mu,
                    local_inner_opt_params
                )
                
                problem_solved = int(res['problem_solved'])
                converged = problem_solved in [1, 2]
                
                if not converged:
                    all_converged = False
                    print(colored(f"Failed (solver status: {problem_solved})", "red"))
                    self.trajectories.append(None)
                    self.fitness_values.append(fitness_weights[0])
                    total_fitness += fitness_weights[0]
                    self.jump_results.append({
                        'jump_number': i + 1,
                        'p0': self.waypoints[i].tolist(),
                        'pf': self.waypoints[i + 1].tolist(),
                        'p0_adjusted': p0_adj.tolist(),
                        'pf_adjusted': pf_adj.tolist(),
                        'fitness':  fitness_weights[0],
                        'problem_solved': problem_solved,
                        'consumed_energy': 0.0,
                        'jump_duration': 0.0
                    })
                    continue
                
                # Extract trajectory
                trajectory = mat_matrix2python(res['p'])
                achieved_target = mat_vector2python(res['achieved_target'])
                consumed_energy = float(res['consumed_energy'])
                jump_duration = float(res['Tf'])
                
                # Calculate landing cost
                if i < len(self.patch_selected):
                    patch_id = self.patch_selected[i]
                else:
                    patch_id = self.patches.get_patch_id_from_point_2D(pf_adj[1], pf_adj[2])
                
                landing_cost = self.patches.get_cost_in_point(patch_id, achieved_target[1:])
                
                # Compute jump fitness (lower is better)
                jump_fitness = consumed_energy * fitness_weights[1] + landing_cost * fitness_weights[3]
                
                self.trajectories.append(trajectory)
                self.fitness_values.append(jump_fitness)
                total_fitness += jump_fitness
                total_consumed_energy += consumed_energy
                total_landing_cost += landing_cost
                
                self.jump_results.append({
                    'jump_number': i + 1,
                    'p0': self.waypoints[i].tolist(),
                    'pf': self.waypoints[i + 1].tolist(),
                    'p0_adjusted': p0_adj.tolist(),
                    'pf_adjusted': achieved_target.tolist(),
                    'fitness': float(jump_fitness),
                    'problem_solved': problem_solved,
                    'consumed_energy': consumed_energy,
                    'jump_duration': jump_duration
                })
                
                # Update waypoint with achieved target for next jump
                if i + 1 < len(self.waypoints):
                    self.waypoints[i + 1] = achieved_target.copy()
                
                print(colored(f"Fitness: {jump_fitness:.2f}, Energy: {consumed_energy:.2f}, Landing Cost: {landing_cost:.2f} ", "green"))
                
            except Exception as e:
                print(colored(f"Exception: {str(e)}", "red"))
                all_converged = False
                self.trajectories.append(None)
                self.fitness_values.append(fitness_weights[0])
                total_fitness +=  fitness_weights[0]
                self.jump_results.append({
                    'jump_number': i + 1,
                    'p0': self.waypoints[i].tolist(),
                    'pf': self.waypoints[i + 1].tolist(),
                    'p0_adjusted': p0_adj.tolist(),
                    'pf_adjusted': pf_adj.tolist(),
                    'fitness': fitness_weights[0],
                    'problem_solved': 0,
                    'consumed_energy': 0.0,
                    'jump_duration': 0.0
                })
        
        elapsed_time = time.time() - start_time
        
        print(colored("\n=== Summary ===", "cyan", attrs=['bold']))
        status = "CONVERGED" if all_converged else "FAILED (one or more jumps)"
        print(f"Status: {status}")
        print(f"Jumps: {num_jumps} | Total fitness: {total_fitness:.2f} | "
              f"Energy: {total_consumed_energy:.2f} | Landing cost: {total_landing_cost:.2f} | "
              f"Time: {elapsed_time:.1f}s\n")
        
        return {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'num_jumps': num_jumps,
                'total_computation_time': elapsed_time,
                'all_converged': all_converged
            },
            'patch_selected': self.patch_selected,
            'waypoints': [wp.tolist() for wp in self.waypoints],
            'fitness_values': self.fitness_values,
            'total_fitness': total_fitness,
            'average_fitness': total_fitness / num_jumps if num_jumps > 0 else 0.0,
            'total_consumed_energy': total_consumed_energy,
            'total_landing_cost': total_landing_cost,
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

    def plot_2d_patch(self, show_plot=True):
        """Plot 2D view of patches with their IDs using PatchCollection."""
        print(colored("\n[PLOT] Generating 2D Patch Grid with Selected Patches...", "cyan"))
        
        fig, ax = plt.subplots(figsize=(14, 10))
        
        # Geometric patch data
        width = self.patches.patch_width
        height = self.patches.patch_height
        total_patches = len(self.patches.patches)
        
        # Collect costs for colormap
        costs = np.array([p.get('cost_patch', 0.0) for p in self.patches.patches])
        
        # Get centroids and calculate corners
        centroids = np.array([p['centroid'] for p in self.patches.patches])
        y_corners = centroids[:, 1] - (width / 2.0)
        z_corners = centroids[:, 2] - (height / 2.0)
        
        # Create rectangles
        rectangles = [mpatches.Rectangle((y, z), width, height) 
                      for y, z in zip(y_corners, z_corners)]
        
        # Main PatchCollection with cost colormap
        pc = PatchCollection(rectangles, cmap='RdYlGn_r', alpha=0.7, edgecolor='grey', linewidth=1)
        pc.set_array(costs)
        ax.add_collection(pc)
        
        # Colorbar
        cbar = fig.colorbar(pc, ax=ax, shrink=0.8)
        cbar.set_label('Patch Cost', rotation=270, labelpad=15, fontsize=11)
        
        # Highlight selected patches with blue dashed border
        if self.patch_selected:
            selected_rects = []
            for patch_id in self.patch_selected:
                if 0 <= patch_id < total_patches:
                    y = y_corners[patch_id]
                    z = z_corners[patch_id]
                    selected_rects.append(mpatches.Rectangle((y, z), width, height))
            
            if selected_rects:
                pc_selected = PatchCollection(selected_rects, facecolor='none', 
                                               edgecolor='blue', linewidth=3, 
                                               linestyle='--', alpha=1.0)
                ax.add_collection(pc_selected)
        
        # Text annotations (IDs)
        for i, (y, z) in enumerate(zip(centroids[:, 1], centroids[:, 2])):
            patch_id = self.patches.patches[i]['id']
            if self.patch_selected and patch_id in self.patch_selected:
                ax.text(y, z, f'{patch_id}', ha='center', va='center', 
                       fontsize=8, fontweight='bold', color='blue')
            else:
                ax.text(y, z, f'{patch_id}', ha='center', va='center', 
                       fontsize=7, color='black')
        
        # Start & Goal points
        ax.scatter(self.p0[1], self.p0[2], c='lime', s=300, marker='o', 
                  edgecolors='darkgreen', linewidths=3, zorder=10, label='Start')
        ax.text(self.p0[1], self.p0[2] + 0.3, "START", ha='center', 
               color='darkgreen', fontweight='bold', fontsize=10, zorder=10)
        
        ax.scatter(self.pf[1], self.pf[2], c='red', s=400, marker='*', 
                  edgecolors='darkred', linewidths=2, zorder=10, label='Goal')
        ax.text(self.pf[1], self.pf[2] - 0.3, "GOAL", ha='center', 
               color='darkred', fontweight='bold', fontsize=10, zorder=10)
        
        # Plot waypoints if computed
        if len(self.waypoints) > 0:
            for i, wp in enumerate(self.waypoints):
                if i == 0 or i == len(self.waypoints) - 1:
                    continue  # Skip start/goal already plotted
                ax.scatter(wp[1], wp[2], c='orange', s=150, marker='o',
                          edgecolors='darkorange', linewidths=2, zorder=9)
                ax.text(wp[1], wp[2] + 0.2, f'W{i}', ha='center', 
                       fontsize=9, fontweight='bold', color='darkorange')
        
        # Styling
        ax.set_xlabel('Y (m)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Z (m)', fontsize=12, fontweight='bold')
        ax.set_title(f'2D Patch Grid View\n'
                    f'Total Patches: {total_patches} | '
                    f'Selected: {self.patch_selected}', 
                    fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.set_aspect('equal')
        ax.autoscale()
        ax.grid(True, alpha=0.3, linestyle='--')
        
        plt.tight_layout()
        
        if show_plot:
            plt.show()
        
        return fig, ax


def plot_patches_2d(patches, p0=None, pf=None, show_plot=True):
        """Standalone function to plot 2D patches with IDs after terrain initialization."""
        print(colored("\n[PLOT] Generating 2D Patch Grid...", "cyan"))
        
        fig, ax = plt.subplots(figsize=(14, 10))
        
        # Geometric patch data
        width = patches.patch_width
        height = patches.patch_height
        total_patches = len(patches.patches)
        
        # Collect costs for colormap
        costs = np.array([p.get('cost_patch', 0.0) for p in patches.patches])
        
        # Get centroids and calculate corners
        centroids = np.array([p['centroid'] for p in patches.patches])
        y_corners = centroids[:, 1] - (width / 2.0)
        z_corners = centroids[:, 2] - (height / 2.0)
        
        # Create rectangles
        rectangles = [mpatches.Rectangle((y, z), width, height) 
                    for y, z in zip(y_corners, z_corners)]
        
        # Main PatchCollection with cost colormap
        pc = PatchCollection(rectangles, cmap='RdYlGn_r', alpha=0.7, edgecolor='grey', linewidth=1)
        pc.set_array(costs)
        ax.add_collection(pc)
        
        # Colorbar
        cbar = fig.colorbar(pc, ax=ax, shrink=0.8)
        cbar.set_label('Patch Cost', rotation=270, labelpad=15, fontsize=11)
        
        # Text annotations (IDs)
        for i, (y, z) in enumerate(zip(centroids[:, 1], centroids[:, 2])):
            patch_id = patches.patches[i]['id']
            ax.text(y, z, f'{patch_id}', ha='center', va='center', 
                fontsize=7, color='black', fontweight='bold')
        
        # Plot start (p0) and goal (pf) if provided
        if p0 is not None:
            ax.scatter(p0[1], p0[2], c='lime', s=300, marker='o', 
                      edgecolors='darkgreen', linewidths=3, zorder=10, label='Start (p0)')
            ax.text(p0[1], p0[2] + 0.3, "START", ha='center', 
                   color='darkgreen', fontweight='bold', fontsize=10, zorder=10)
        
        if pf is not None:
            ax.scatter(pf[1], pf[2], c='red', s=400, marker='*', 
                      edgecolors='darkred', linewidths=2, zorder=10, label='Goal (pf)')
            ax.text(pf[1], pf[2] - 0.3, "GOAL", ha='center', 
                   color='darkred', fontweight='bold', fontsize=10, zorder=10)
        
        # Styling
        ax.set_xlabel('Y (m)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Z (m)', fontsize=12, fontweight='bold')
        ax.set_title(f'2D Patch Grid\n'
                    f'Total Patches: {total_patches} ({patches.number_of_patches_width}x{patches.number_of_patches_height})', 
                    fontsize=14, fontweight='bold')
        
        # Add legend if p0 or pf are shown
        if p0 is not None or pf is not None:
            ax.legend(loc='upper right', fontsize=10)
        
        ax.set_aspect('equal')
        ax.autoscale()
        ax.grid(True, alpha=0.3, linestyle='--')
        
        plt.tight_layout()
        
        if show_plot:
            plt.show()
        
        return fig, ax


def main():
    print(colored("="*60, "cyan"))
    print(colored("  LINEAR MULTI-JUMP TRAJECTORY OPTIMIZER  ", "cyan", attrs=['bold']))
    print(colored("="*60 + "\n", "cyan"))
    
    # Initialize terrain
    print(colored("[1/4] Initializing Terrain...", "yellow"))
    terrain_manager = TerrainManager()
    point_clouds, patches, cost_grid = initialize_terrain_data()
    print(colored("✓ Terrain initialized\n", "green"))
    
    # Plot 2D patches after initialization
    print(colored("[2/4] Plotting 2D Patch Grid...", "yellow"))
    plot_patches_2d(patches, p0=P0_INIT, pf=PF_INIT, show_plot=True)
    print(colored("✓ Patch grid displayed\n", "green"))
        
    # Configure multi-jump
    print(colored("[3/4] Configuring Multi-Jump...", "yellow"))
    print(f"Start: {P0_INIT} | Goal: {PF_INIT} | Jumps: {MAX_JUMP}")
    # patch_selected = [31, 54,66]
    # patch_selected = [22, 55,66]
    patch_selected = [13, 33, 46, 66]
    multi_jump = LinearMultiJumpParams(
        P0_INIT, PF_INIT, terrain_manager, point_clouds, patches, patch_selected
    )
    print(colored("✓ Configuration complete\n", "green"))
    
    # Execute
    print(colored("[4/4] Executing Multi-Jump Trajectory...", "yellow"))
    results = multi_jump.execute_multi_jump_in_patch()
    print(colored("✓ Execution complete\n", "green"))
    
    # Plot results
    multi_jump.plot_multi_jump_trajectory(show_plot=True)
    multi_jump.plot_multi_jump_mesh(show_plot=True)
    print(colored("✓ All operations completed!\n", "green", attrs=['bold']))


if __name__ == "__main__":
    main()