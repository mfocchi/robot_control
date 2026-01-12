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

def get_matlab_engine(point_clouds=None, cost_grid=None, terrain_manager=None):
    if not hasattr(thread_local, 'engine'):
        eng = matlab.engine.start_matlab()
        eng.addpath('../../codegen_mesh_landing', nargout=0)
        
        thread_local.engine = eng
        print(f"Created Engine and uploaded Mesh for thread {threading.current_thread().name}")
    return thread_local.engine

def close_matlab_engines():
    if hasattr(thread_local, 'engine'):
        thread_local.engine.quit()
        del thread_local.engine
        print("MATLAB engine closed")
        
def shutdown_engine():
    close_matlab_engines()

def create_inner_opt_params_copy():
    return {
        'm': inner_opt_params['m'],
        'num_params': inner_opt_params['num_params'],
        'int_method': inner_opt_params['int_method'],
        'N_dyn': inner_opt_params['N_dyn'],
        'FRICTION_CONE': inner_opt_params['FRICTION_CONE'],
        'int_steps': inner_opt_params['int_steps'],
        'b': inner_opt_params['b'],
        'p_a1': matlab.double([0., 0., 0.]).reshape(3, 1),
        'p_a2': matlab.double([0., inner_opt_params['b'], 0.]).reshape(3, 1),
        'g': inner_opt_params['g'],
        'w1': inner_opt_params['w1'],
        'w2': inner_opt_params['w2'],
        'w3': inner_opt_params['w3'],
        'T_th': inner_opt_params['T_th'],
        'obstacle_avoidance': inner_opt_params['obstacle_avoidance'],
        'jump_clearance': inner_opt_params['jump_clearance'],
        'mesh_x': inner_opt_params['mesh_x'],
        'mesh_y': inner_opt_params['mesh_y'],
        'mesh_z': inner_opt_params['mesh_z'],
        'cost_x': inner_opt_params['cost_x'],
        'cost_y': inner_opt_params['cost_y'],
        'cost_z': inner_opt_params['cost_z'],
        'patch_side': inner_opt_params['patch_side'],
        'contact_normal': None,
    }
    
class BilevelOpt:
    
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
        # n_iteration matlab for n_thread
        eng = get_matlab_engine(self.point_clouds, self.cost_grid, self.terrain_manager)
        
        local_inner_opt_params = create_inner_opt_params_copy()
        
        jump_log_points = []
        jump_log_traj = []
        xd = input_data[0]  # xd is the discrete variables and contains the patches
        xc = input_data[1]  # xc is the continuous variables and contains the jump coordinates
        n_jumps = xd[0]     # first discrete variable is number of jumps, the next ones are the of the patches
        ids = []
        fitness = 0.0
        total_consumed_energy = 0.0
        total_landing_cost = 0.0
        
        print(colored(f"[EVAL] Evaluating individual with {n_jumps + 1} total jumps ({n_jumps} intermediate + 1 final)", "blue"))
        
        p0_adj = self.p0.copy()
        
        p0_adj[0] = self.terrain_manager.wall_surface_eval(
            p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        
        jump_log_points.append(p0_adj.copy())
        
        for i in range(n_jumps):
            
            patch_id = xd[1 + i] 
            # the continue variables contain the X and Y normalized coordinate of the candidate contact landing points inside the candidate patches
            contact_relative_to_patch_yz = xc[i * 2:i * 2 + 2]  # tra 0 - 1  upper left corner patch

            # computes 0, Y, Z  absolute coordinates of candidate landing location
            landing_abs_pos = self.patches.getAbsolutePoseOfPointInsidePatch(patch_id, contact_relative_to_patch_yz[0],
                                                                             contact_relative_to_patch_yz[1], scale=1.0)
            pf_adj = landing_abs_pos.copy()
            
            # adjust X coordinate to terrain shape for both liftoff and landing points
            pf_adj[0] = self.terrain_manager.wall_surface_eval(pf_adj[2], pf_adj[1], self.terrain_manager.mesh_x,
                                                               self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            p0_adj[0] = self.terrain_manager.wall_surface_eval(p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                                                               self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            
            # compute normal at liftoff
            liftoff_normal = self.terrain_manager.wall_normal_eval(p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                                                                   self.terrain_manager.mesh_y,
                                                                   self.terrain_manager.mesh_z)

            
            # Update only contact_normal for this jump
            local_inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)
            
            #tun optimization
            res = eng.optimize_cpp_mex(
                matlab.double(p0_adj), matlab.double(pf_adj), 
                Fleg_max, Fr_max, Fr_min, mu, local_inner_opt_params)
            
            jump_log_traj.append(mat_matrix2python(res['p']))
            
            # Track energy and landing cost
            total_consumed_energy += res['consumed_energy']
            landing_cost = self.patches.get_cost_in_point(patch_id, pf_adj[1:])
            total_landing_cost += landing_cost
            
            fitness += self.calc_fitness(res, patch_id=patch_id, 
                                        contact_abs_pos_yz=pf_adj[1:])
            
            # Add landing point AFTER the trajectory
            jump_log_points.append(pf_adj.copy())
            p0_adj = pf_adj.copy()
            
        # print("final jump")
        # last jump is to pf
        # absolute coordinates of FINAL landing location
        pf_adj = self.pf.copy()
        
        # adjust X coordinate to terrain shape for both liftoff and landing points
        pf_adj[0] = self.terrain_manager.wall_surface_eval(pf_adj[2], pf_adj[1], self.terrain_manager.mesh_x,
                                                           self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        p0_adj[0] = self.terrain_manager.wall_surface_eval(p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                                                           self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)

        # compute normal at liftoff
        liftoff_normal = self.terrain_manager.wall_normal_eval(p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                                                               self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        
        # Update only contact_normal for final jump
        local_inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)
        
        res = eng.optimize_cpp_mex(
            matlab.double(p0_adj), matlab.double(pf_adj), 
            Fleg_max, Fr_max, Fr_min, mu, local_inner_opt_params)
        
        # Track energy for final jump
        total_consumed_energy += res['consumed_energy']
        
        fitness += self.calc_fitness(res)
        # if (n_jumps+1) < 2:
        #     fitness -= 500.0  # Valore molto basso per "spaventare" l'algoritmo
        ref_com = mat_matrix2python(res['p'])
        jump_log_traj.append(ref_com)
        jump_log_points.append(pf_adj.copy())
        print(f"Final jump: p0_adj = {p0_adj}, pf_adj = {pf_adj} ; fitness = {fitness}")

        # Plot trajectories if requested
        #self.plot_point_traj(jump_log_points, jump_log_traj)
        
        log_result = {
            'fitness': fitness,
            'points': jump_log_points,
            'traj': jump_log_traj,
            'n_jumps': n_jumps + 1,
            'consumed_energy': total_consumed_energy,
            'landing_cost': total_landing_cost
        }
        
        return log_result
               
    def calc_fitness(self,res, patch_id=None, contact_abs_pos_yz=None):
        fit_average_cost_patch = 0.
        fit_landing_cost = 0.

        if (patch_id is not None and  contact_abs_pos_yz is not None):
            #compute cost for landing candidate
            fit_landing_cost = -self.patches.get_cost_in_point(patch_id, contact_abs_pos_yz)
            #compute average cost on patch to see how bad /good is terrain there
            fit_average_cost_patch = -self.patches.get_patch_cost(patch_id)
            #if fit_landing_costmap is None:

        fit_consumed_energy = -res['consumed_energy']
        if (res['problem_solved']) == 1 or (res['problem_solved']==2): #convergence / semidefinite solution
            fit_problem_converged = 0
        else: #problem did not converge
            fit_problem_converged = -1000
        # print("jump duration", res['Tf'])
        print(f"convergence: {self.fitness_weights[0]*fit_problem_converged}, energy: {self.fitness_weights[1]*fit_consumed_energy}, avg_cost: {self.fitness_weights[2]*fit_average_cost_patch}, land_cost: {self.fitness_weights[3]*fit_landing_cost}")
        
        fitness = ( self.fitness_weights[0] * fit_problem_converged + 
                    self.fitness_weights[1] * fit_consumed_energy +
                    self.fitness_weights[2] * fit_average_cost_patch + 
                    self.fitness_weights[3] * fit_landing_cost)
        return fitness
    
    def plot_point_traj(self, jump_log_points, jump_log_traj):
        
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

    def plot_mesh_traj(self, jump_log_points, jump_log_traj):
        
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

        ax.set_title('Trajectory on Terrain Mesh')
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