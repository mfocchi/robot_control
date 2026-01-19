import os
import sys
import numpy as np
from termcolor import colored
import threading
import matlab.engine
from params import *
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches 
from base_controllers.utils.matlab_conversions import (
    mat_matrix2python,
    mat_vector2python,
)

thread_local = threading.local()

    
class LinearOpti:
    
    def __init__(self, terrain_manager, p0, pf, fitness_weights, point_clouds, patches, cost_grid):
        
        self.terrain_manager = terrain_manager
        self.p0 = p0
        self.pf = pf
        self.fitness_weights = fitness_weights
        
        # terrain creation done outside to avoid recomputation at each thread
        self.point_clouds = point_clouds
        self.patches = patches
        self.cost_grid = cost_grid
    
    def eval_pop(self, input_data):
        
        # Extract discrete parameters (first array is the possible jumps) and the rest are the patch IDs
        xd = input_data[0] if isinstance(input_data, list) and len(input_data) > 0 else input_data
        n_jumps = int(xd[0])
        
        # State tracking
        jump_log_points = []
        jump_log_traj = []
        fitness = 0.0
        total_consumed_energy = 0.0
        total_landing_cost = 0.0
        all_converged = True  # Flag to monitor overall convergence
        
        print(colored(f"[EVAL] Evaluating individual with {n_jumps + 1} total jumps ({n_jumps} intermediate + 1 final)", "blue"))
        
        p0_adj = self.p0.copy()
        p0_adj[0] = self.terrain_manager.wall_surface_eval(
            p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        
        jump_log_points.append(p0_adj.copy())
        
        total_jump = n_jumps + 1  # Including final jump
        for i in range(total_jump):
            if i < n_jumps:
                patch_id = int(xd[1 + i])
                contact_relative_to_patch_yz = [0.5, 0.5] # center of the patch
                pf_adj = self.patches.getAbsolutePoseOfPointInsidePatch(
                    patch_id, contact_relative_to_patch_yz[0],
                    contact_relative_to_patch_yz[1], scale=1.0).copy()
            else: 
                pf_adj = self.pf.copy()
                patch_id = None
            
            # Projection on the surface the points considered for optimization
            for pt in [p0_adj, pf_adj]:
                pt[0] = self.terrain_manager.wall_surface_eval(
                    pt[2], pt[1], self.terrain_manager.mesh_x,
                    self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            
            # Compute normal at liftoff
            liftoff_normal = self.terrain_manager.wall_normal_eval(
                p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            
            # Run optimization
            #res = self.linear_computation(p0_adj, pf_adj)
            res = self.linear_computation_exponential(p0_adj, pf_adj, a=.50)
            # Convergence Check
            if int(res['problem_solved']) != 1:
                all_converged = False
            
            # Update logs and metrics
            jump_log_traj.append(res['linear_trajectory'])
            jump_log_points.append(pf_adj.copy())
            total_consumed_energy += res['consumed_energy']
            
            jump_fitness, jump_landing_cost = self.calc_linear_fitness(
                res, patch_id=patch_id, contact_abs_pos_yz=pf_adj[1:])
            fitness += jump_fitness
            total_landing_cost += jump_landing_cost
            
            # The landing point becomes the new starting point
            p0_adj = pf_adj.copy()
        
        print("xd value: " , xd)    
        print ("fitness: " , fitness)
        
        
        status_msg = "CONVERGED" if all_converged else "FAILED (One or more jumps did not converge)"
        print(f"--- Evaluation Results ---")
        print(f"Total Jumps: {n_jumps + 1}, Total Fitness: {fitness:.4f}, Energy Consumed: {total_consumed_energy:.2f}, Global Convergence: {status_msg}")
        print(f"--------------------------")
        # breakpoint()
        # Plot trajectories if requested
        #self.plot_point_traj(jump_log_points, jump_log_traj)
        
        return {
            'fitness': fitness,
            'points': jump_log_points,
            'traj': jump_log_traj,
            'n_jumps': n_jumps + 1,
            'consumed_energy': total_consumed_energy,
            'landing_cost': total_landing_cost,
            'all_converged': all_converged
        }
    
    def linear_computation(self, p0, pf):
        distance = np.linalg.norm(pf - p0)
        
        problem_solved = 1 if distance is not None else 0
        num_points = 50 
        t = np.linspace(0, 1, num_points)
        # interpolation
        trajectory = np.zeros((3, num_points))
        for i in range(3): 
            trajectory[i, :] = p0[i] + t * (pf[i] - p0[i])
            
        res = {
            'consumed_energy': float(distance),
            'problem_solved': problem_solved,
            'linear_trajectory': trajectory
        }
        
        return res
    
    def linear_computation_exponential(self, p0, pf, a=0.2):
        
        distance = np.linalg.norm(pf - p0)

        problem_solved = 1 if distance > 0 else 0
        num_points = 50
        t = np.linspace(0, 1, num_points)

        trajectory = np.zeros((3, num_points))
        for i in range(3):
            trajectory[i, :] = p0[i] + t * (pf[i] - p0[i])

        # E(d) = d * exp(d - a)
        consumed_energy = distance * np.exp(distance - a)

        res = {
            'consumed_energy': float(consumed_energy),
            'problem_solved': problem_solved,
            'linear_trajectory': trajectory
        }

        return res
           
    def calc_linear_fitness(self, res, patch_id=None, contact_abs_pos_yz=None):
        fit_average_cost_patch = 0.
        fit_landing_cost = 0.

        if (patch_id is not None and contact_abs_pos_yz is not None):
            #compute cost for landing candidate
            fit_landing_cost = -self.patches.get_cost_in_point(patch_id, contact_abs_pos_yz)
            #compute average cost on patch to see how bad /good is terrain there
            fit_average_cost_patch = -self.patches.get_patch_cost(patch_id)

        fit_consumed_energy = -res['consumed_energy']
        if (res['problem_solved']) == 1:
            fit_problem_converged = 0
        else: #problem did not converge
            fit_problem_converged = -5000
        
        # print(f"convergence: {self.fitness_weights[0]*fit_problem_converged}, energy: {self.fitness_weights[1]*fit_consumed_energy}, avg_cost: {self.fitness_weights[2]*fit_average_cost_patch}, land_cost: {self.fitness_weights[3]*fit_landing_cost}")
        
        fitness = (self.fitness_weights[0] * fit_problem_converged + 
                   self.fitness_weights[1] * fit_consumed_energy +
                   self.fitness_weights[2] * fit_average_cost_patch + 
                   self.fitness_weights[3] * fit_landing_cost)
        
        return fitness, fit_landing_cost
    
    def plot_point_traj(self, jump_log_points, jump_log_traj):
        # Conversione sicura dei punti della nuvola
        x_points = np.array([point['position'][0] for point in self.point_clouds.points_t])
        y_points = np.array([point['position'][1] for point in self.point_clouds.points_t])
        z_points = np.array([point['position'][2] for point in self.point_clouds.points_t])
        color = np.array([point['color'] for point in self.point_clouds.points_t])
        size_point = np.array([point['size_point'] for point in self.point_clouds.points_t])
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Terreno (alpha ridotta per far risaltare la traiettoria)
        ax.scatter(x_points, y_points, z_points, c=color, s=size_point, alpha=0.4, label='Terrain')
        
        # Plot dei Waypoints
        pts = np.array(jump_log_points)
        for i, point in enumerate(pts):
            if i == 0:  
                ax.scatter(point[0], point[1], point[2], c='green', s=150, marker='o', 
                           edgecolors='black', linewidths=2, label='Start Point', zorder=10)
            elif i == len(pts) - 1:
                ax.scatter(point[0], point[1], point[2], c='red', s=150, marker='X', 
                           edgecolors='black', linewidths=2, label='End Point', zorder=10)
            else:
                ax.scatter(point[0], point[1], point[2], c='cyan', s=100, marker='o', 
                           edgecolors='black', label='Jump Contact' if i == 1 else '', zorder=10)
        
        # Plot delle Traiettorie lineari
        colors = plt.cm.jet(np.linspace(0, 1, len(jump_log_traj)))
        for i, traj in enumerate(jump_log_traj):
            if traj is not None and traj.ndim == 2:
                ax.plot(traj[0, :], traj[1, :], traj[2, :], color=colors[i], 
                        linewidth=3, alpha=1.0, zorder=15, label=f'Segment {i+1}')
        
        # Scaling degli assi uguale (fondamentale per non deformare la linea)
        all_pts = np.vstack([pts] + [t.T for t in jump_log_traj if t is not None])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

        ax.set_xlabel('X (Depth/Height)')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Path Visualization (Points)\nSegments: {len(jump_log_traj)}')
        ax.legend(loc='upper right')
        plt.show()
        
    def plot_mesh_traj(self, jump_log_points, jump_log_traj, best_fitness):
        
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