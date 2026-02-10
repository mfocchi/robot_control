import os
import sys
import numpy as np
from termcolor import colored
from params import *
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches 
from base_controllers.utils.matlab_conversions import (
    mat_matrix2python,
    mat_vector2python,
)

class BilevelOpt:
    
    def __init__(self, terrain_manager, p0, pf_patch, fitness_weights, point_clouds, patches, cost_grid):
        
        self.terrain_manager = terrain_manager
        self.p0 = p0
        self.pf_patch = pf_patch
        self.fitness_weights = fitness_weights
        self.point_clouds = point_clouds
        self.patches = patches
        self.cost_grid = cost_grid
    
    def eval_pop(self, input_data):
        
        # State tracking
        jump_log_points = []
        jump_log_traj = []
        total_consumed_energy = 0.0
        total_landing_cost = 0.0
        all_converged = True  # Flag to monitor overall convergence
        achieved_target = None
        
        # Initialization parameters for matlab engine
        eng = get_matlab_engine(self.point_clouds, self.cost_grid, self.terrain_manager)
        local_inner_opt_params = create_inner_opt_params_copy()
        
        # Extract discrete parameters (first array is the possible jumps) and the rest are the patch IDs
        xd = input_data[0] if isinstance(input_data, list) and len(input_data) > 0 else input_data
        n_jumps = int(xd[0])
        
        p0_adj = self.p0.copy()
        p0_adj[0] = self.terrain_manager.wall_surface_eval(
            p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        
        jump_log_points.append(p0_adj.copy())

        total_jump = n_jumps + 1  # Including final jump with +1
        
        # DOPPIONE PISELLONE CHECK
        # fitness about "duplicate" jumps inside xd
        used_patches = [(xd[1 + i]) for i in range(n_jumps)]
        if len(used_patches) != len(set(used_patches)):
            print("DOPPIONE PISELLONE!")
            all_converged = False
            fitness_score = -self.fitness_weights[0]  # Negative for maximization
            
            print("xd value: " , xd)    
            print(f"Computed Score (Fitness): {fitness_score:.4f}")
            print(f"--- Evaluation Results ---")
            print(f"Status: FAILED (DUPLICATE), Waypoints Used: 0/{MAX_JUMP}, Total Energy: 0.00, Terrain Cost: 0.00")
            print(f"--------------------------")
            
            return {
                'fitness': fitness_score,
                'points': [],  # Changed from None to empty list
                'traj': [],    # Changed from None to empty list
                'achieved_target': None,
                'n_jumps': 0,
                'consumed_energy': 0.0,
                'landing_cost': 0.0,
                'all_converged': False
            }
        
        
        for i in range(total_jump):
            if i < n_jumps: # jump btw patches
                patch_id = int(xd[1 + i])
                center_relative_patch_yz = [.5, .5]
                pf_adj = self.patches.getAbsolutePoseOfPointInsidePatch(
                    patch_id, center_relative_patch_yz[0], 
                    center_relative_patch_yz[1], scale=1.0).copy()
            else: # final jump to target
                pf_adj = self.pf_patch.copy()
                patch_id = self.patches.get_patch_id_from_point_2D(pf_adj[1], pf_adj[2]) 
                # mettere patch side a 0.1
                local_inner_opt_params['patch_side_y'] = 0.1
                local_inner_opt_params['patch_side_z'] = 0.1
            
            # Projection on the surface the points considered for optimization
            for pt in [p0_adj, pf_adj]:   
                pt[0] = self.terrain_manager.wall_surface_eval(
                    pt[2], pt[1], self.terrain_manager.mesh_x,
                    self.terrain_manager.mesh_y, self.terrain_manager.mesh_z) 

            # Calculate normal
            liftoff_normal = self.terrain_manager.wall_normal_eval(
                p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x
                , self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            
            local_inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)
            
            res = eng.optimize_cpp_mex(
                matlab.double(p0_adj), matlab.double(pf_adj), 
                Fleg_max, Fr_max, Fr_min, mu, local_inner_opt_params)
            
            # Convergence Check (1=Converged, 2=Semidefinite(other possible solutions))
            if int(res['problem_solved']) not in [1, 2]:
                all_converged = False
                break
            
            jump_landing_cost, jump_average_cost_patch = self.calc_terrain_cost(
                                            res, patch_id=patch_id, contact_abs_pos_yz=mat_vector2python(res['achieved_target'])[1:])
            
            # Update logs and metrics
            jump_log_traj.append(mat_matrix2python(res['p']))
            jump_log_points.append(mat_vector2python(res['achieved_target']).copy())
            total_consumed_energy += res['consumed_energy']
            total_landing_cost += jump_landing_cost # cost of each landing point
            
            if np.isnan(mat_vector2python(res['achieved_target'])[0]):
                print("a")
                breakpoint()
            vec = mat_vector2python(res['achieved_target'])
            if (vec[1] < 0 or vec[1] > 10) or (vec[2] > 0 or vec[2] < -10):
                print("b")
                breakpoint()
            
            vec = p0_adj
            if (vec[1] < 0 or vec[1] > 10) or (vec[2] > 0 or vec[2] < -10):
                print("c")
                breakpoint()
            # acutal target becomes next starting point
            p0_adj = mat_vector2python(res['achieved_target']) #pf_adj.copy()
            
        if not all_converged:
            fitness_score = -self.fitness_weights[0]  # Negative for maximization
            achieved_target = None
            avg_jump_landing_cost = 0.0
            avg_energy_cost = 0.0
        else:
            # waypoint_cost = (MAX_JUMP - total_jump) * self.fitness_weights[5]
            avg_energy_cost = (total_consumed_energy) * self.fitness_weights[1] # / total_jump ??
            avg_jump_landing_cost = (total_landing_cost) * self.fitness_weights[3] # / total_jump ??
            
            fitness_score = -(avg_energy_cost + avg_jump_landing_cost)  # Negative for maximization
            achieved_target = mat_vector2python(res['achieved_target']) if res['achieved_target'] is not None else None

        print("xd value: ", xd)
        print(f"Computed Score (Fitness): {fitness_score:.4f}")
        status_msg = "CONVERGED" if all_converged else "FAILED (in One or more jumps)"
        print(f"--- Evaluation Results ---")
        print(f"Total Jumps: {total_jump}/{MAX_JUMP}, Total Fitness: {fitness_score:.4f}, Energy Consumed: {avg_energy_cost:.2f}, avg_jump_landing_cost: {avg_jump_landing_cost:.4f} , Global Convergence: {status_msg}")
        print(f"--------------------------")
        
        return {
            'fitness': fitness_score,  # This is now a FITNESS (maximize this value, 0 is best)
            'points': jump_log_points,
            'traj': jump_log_traj,
            'achieved_target': achieved_target,
            'n_jumps': total_jump,
            'consumed_energy': avg_energy_cost,
            'landing_cost': avg_jump_landing_cost,
            'all_converged': all_converged
        }
               
    def calc_terrain_cost(self, res, patch_id=None, contact_abs_pos_yz=None):
        fit_average_cost_patch = 0.
        fit_landing_cost = 0.

        if (patch_id is not None and contact_abs_pos_yz is not None):
            #compute cost for landing candidate
            fit_landing_cost = self.patches.get_cost_in_point(patch_id, contact_abs_pos_yz)
            #compute average cost on patch to see how bad /good is terrain there
            fit_average_cost_patch = self.patches.get_patch_cost(patch_id)

        return fit_landing_cost, fit_average_cost_patch
    
    def plot_point_traj(self, jump_log_points, jump_log_traj):
        
        
        if jump_log_points is None or jump_log_traj is None:
            print("Warning: Cannot plot trajectory - missing data")
            return
        
        if len(jump_log_points) == 0 or len(jump_log_traj) == 0:
            print("Warning: Cannot plot trajectory - empty data")
            return
        
        x_points = np.array([point['position'][0] for point in self.point_clouds.points_t])
        y_points = np.array([point['position'][1] for point in self.point_clouds.points_t])
        z_points = np.array([point['position'][2] for point in self.point_clouds.points_t])
        color = np.array([point['color'] for point in self.point_clouds.points_t])
        size_point = np.array([point['size_point'] for point in self.point_clouds.points_t])
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.scatter(x_points, y_points, z_points, c=color, s=size_point, alpha=0.6, label='Terrain')
        
        for i, point in enumerate(jump_log_points):
            if i == 0:  
                ax.scatter(point[0], point[1], point[2], 
                        c='green', s=150, marker='o', 
                        edgecolors='black', linewidths=2,
                        label='Start Point', zorder=5)
            elif i == len(jump_log_points) - 1:  # End point
                ax.scatter(point[0], point[1], point[2], 
                        c='red', s=150, marker='o', 
                        edgecolors='black', linewidths=2,
                        label='End Point', zorder=5)
            else:  
                ax.scatter(point[0], point[1], point[2], 
                        c='blue', s=120, marker='o', 
                        edgecolors='black', linewidths=2,
                        label='Waypoint' if i == 1 else '', zorder=5)
        
        # Plot trajectories
        trajectory_colors = plt.cm.viridis(np.linspace(0, 1, len(jump_log_traj)))
        
        for i, traj in enumerate(jump_log_traj):
            if traj is not None and traj.size > 0:
                # traj is expected to be shape (3, N) where rows are [x, y, z]
                if traj.ndim == 2 and traj.shape[0] == 3:
                    ax.plot(traj[0, :], traj[1, :], traj[2, :], 
                        color=trajectory_colors[i], linewidth=2.5, 
                        label=f'Jump {i+1}' if i < 3 else '',
                        alpha=0.9, zorder=4)
                else:
                    print(f"Warning: Trajectory {i} has unexpected shape {traj.shape}")
        all_x = [x_points]
        all_y = [y_points]
        all_z = [z_points]

        # Add trajectories
        for traj in jump_log_traj:
            if traj is not None and traj.size > 0 and traj.ndim == 2 and traj.shape[0] == 3:
                all_x.append(traj[0, :])
                all_y.append(traj[1, :])
                all_z.append(traj[2, :])

        # Add waypoints
        wps = np.array(jump_log_points)
        all_x.append(wps[:, 0])
        all_y.append(wps[:, 1])
        all_z.append(wps[:, 2])

        all_x = np.concatenate(all_x)
        all_y = np.concatenate(all_y)
        all_z = np.concatenate(all_z)

        x_min, x_max = all_x.min(), all_x.max()
        y_min, y_max = all_y.min(), all_y.max()
        z_min, z_max = all_z.min(), all_z.max()

        max_range = max(
            x_max - x_min,
            y_max - y_min,
            z_max - z_min
        ) * 0.5

        mid_x = (x_max + x_min) * 0.5
        mid_y = (y_max + y_min) * 0.5
        mid_z = (z_max + z_min) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        # Set labels and title
        ax.set_xlabel('X (m) - Height', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_zlabel('Z (m)', fontsize=11)
        ax.set_title(f'Optimized Jumping Path\nTotal Jumps: {len(jump_log_traj)}', 
                    fontsize=13, fontweight='bold')
        
        # Add legend
        ax.legend(loc='upper left', fontsize=9)
        
        # Set viewing angle for better visualization
        ax.view_init(elev=20, azim=45)
        
        # Add grid
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # Print trajectory statistics
        print("\n=== Trajectory Statistics ===")
        for i, traj in enumerate(jump_log_traj):
            if traj is not None and traj.size > 0:
                traj_length = np.sum(np.sqrt(np.sum(np.diff(traj, axis=1)**2, axis=0)))
                print(f"Jump {i+1}: {traj.shape[1]} points, Length: {traj_length:.2f}m")

    def plot_mesh_traj(self, jump_log_points, jump_log_traj, best_fitness):
        
        
        if jump_log_points is None or jump_log_traj is None:
            print("Warning: Cannot plot trajectory - missing data")
            return
        
        if len(jump_log_points) == 0 or len(jump_log_traj) == 0:
            print("Warning: Cannot plot trajectory - empty data")
            return
        
        X = self.terrain_manager.mesh_x
        Y = self.terrain_manager.mesh_y
        Z = self.terrain_manager.mesh_z

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        surf = ax.plot_surface(X, Y, Z,color='sienna', 
                               edgecolor='none', alpha=0.4, 
                               linewidth=0, antialiased=False)

        pts = np.array(jump_log_points)
        # Start Point
        ax.scatter(pts[0, 0], pts[0, 1], pts[0, 2], 
                   c='lime', s=100, marker='o', edgecolors='k', label='Start', zorder=10)
        # Goal Point
        ax.scatter(pts[-1, 0], pts[-1, 1], pts[-1, 2], 
                   c='darkred', s=100, marker='*', edgecolors='k', label='Goal', zorder=10)
        # Punti intermedi
        if len(pts) > 2:
            ax.scatter(pts[1:-1, 0], pts[1:-1, 1], pts[1:-1, 2], 
                       c='blue', s=60, marker='o', edgecolors='k', label='Contacts', zorder=10)
        
        trajectory_colors = plt.cm.jet(np.linspace(0, 1, len(jump_log_traj)))
        
        for i, traj in enumerate(jump_log_traj):
            if traj is not None and traj.size > 0:
                ax.plot(traj[0, :], traj[1, :], traj[2, :], 
                        color='blue', linewidth=2, 
                        label=f'Jump {i+1}', zorder=20)

        
        terrain_proxy = mpatches.Patch(color='grey', alpha=0.4, label='Terrain Surface')
        handles, labels = ax.get_legend_handles_labels()
        handles.insert(0, terrain_proxy)
        ax.legend(handles=handles, loc='upper left')

        ax.set_title(f'Trajectory on Terrain Mesh (Best fitness: {best_fitness:.4f})')
        ax.set_xlabel('X (Depth/Height)')
        ax.set_ylabel('Y (Horizontal)')
        ax.set_zlabel('Z (Vertical)')

        all_x = np.concatenate([X.flatten()] + [t[0,:] for t in jump_log_traj if t is not None])
        all_y = np.concatenate([Y.flatten()] + [t[1,:] for t in jump_log_traj if t is not None])
        all_z = np.concatenate([Z.flatten()] + [t[2,:] for t in jump_log_traj if t is not None])

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

        ax.view_init(elev=30, azim=-45)
        plt.tight_layout()
        plt.show()