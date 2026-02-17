
import matplotlib
matplotlib.use('TkAgg')
import os
import sys
import time
import json
import numpy as np
from termcolor import colored
from datetime import datetime
import matplotlib.pyplot as plt

import matlab.engine
import matplotlib.patches as mpatches 
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface
from base_controllers.utils.matlab_conversions import mat_matrix2python, mat_vector2python


from params import *


        
class SingleJump:
    """
    Class to handle single jump optimization between two points on a terrain.
    """
    def __init__(self, p0, pf, terrain_manager, point_clouds, patches, cost_grid):
        self.p0 = np.array(p0)
        self.pf = np.array(pf)
        self.terrain_manager = terrain_manager
        self.point_clouds = point_clouds
        self.patches = patches
        self.cost_grid = cost_grid
        self.jump_opt_params = create_inner_opt_params_copy()
        
        
    def execute_jump(self, patch_id=None, contact_relative_to_patch_yz=None):
        p0_adj = self.p0.copy()
        pf_adj = self.pf.copy()
        
        eng = get_matlab_engine(self.point_clouds, self.cost_grid, self.terrain_manager)
        
        
        # If patch information provided, compute landing position
        if patch_id is not None and contact_relative_to_patch_yz is not None:
            landing_abs_pos = self.patches.getAbsolutePoseOfPointInsidePatch(
                patch_id, 
                contact_relative_to_patch_yz[0],
                contact_relative_to_patch_yz[1], 
                scale=1.0
            )
            pf_adj = landing_abs_pos.copy()
        
        # Adjust X coordinate to terrain shape for both liftoff and landing points
        pf_adj[0] = self.terrain_manager.wall_surface_eval(
            pf_adj[2], pf_adj[1], 
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, 
            self.terrain_manager.mesh_z
        )
        p0_adj[0] = self.terrain_manager.wall_surface_eval(
            p0_adj[2], p0_adj[1], 
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, 
            self.terrain_manager.mesh_z
        )
        
        # Compute normal at liftoff point
        liftoff_normal = self.terrain_manager.wall_normal_eval(
            p0_adj[2], p0_adj[1], 
            self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y,
            self.terrain_manager.mesh_z
        )
        
        # Update contact normal for this jump
        self.jump_opt_params['contact_normal'] = matlab.double(liftoff_normal)
        
        res = eng.optimize_cpp_mex(
            matlab.double(p0_adj.tolist()), 
            matlab.double(pf_adj.tolist()), 
            Fleg_max, Fr_max, Fr_min, mu,
            self.jump_opt_params
        )
        
        fitness = self.calc_fitness(res, patch_id, pf_adj[1:] if patch_id is not None else None)
        # Extract trajectory
        trajectory = mat_matrix2python(res['p'])
        print(f"Jump: p0 = {p0_adj}, pf = {pf_adj} ; fitness = {fitness:.2f}")
        
        return fitness, p0_adj, pf_adj, trajectory, res
    
    def calc_fitness(self, res, patch_id=None, contact_abs_pos_yz=None):
        fit_average_cost_patch = 0.0
        fit_landing_cost = 0.0
        
        # Compute patch-related fitness if applicable
        if patch_id is not None and contact_abs_pos_yz is not None:
            # Cost for landing candidate point
            fit_landing_cost = self.patches.get_cost_in_point(patch_id, contact_abs_pos_yz)
            # Average cost on patch
            fit_average_cost_patch = self.patches.get_patch_cost(patch_id)
        
        # Energy consumption fitness
        fit_consumed_energy = res['consumed_energy']
        
        # Convergence fitness
        if res['problem_solved'] == 1 or res['problem_solved'] == 2:
            fit_problem_converged = 0.0
        else:
            fit_problem_converged = 10000.0
        status = (res['problem_solved'])
        # Print individual fitness components
        print(f"  convergence: {fitness_weights[0]*fit_problem_converged:.2f}, "
              f"energy: {fitness_weights[1]*fit_consumed_energy:.2f}, "
              f"avg_cost: {fitness_weights[2]*fit_average_cost_patch:.2f}, "
              f"land_cost: {fitness_weights[3]*fit_landing_cost:.2f}")
        
        # Compute total weighted fitness
        fitness = (fitness_weights[0] * fit_problem_converged + 
                   fitness_weights[1] * fit_consumed_energy +
                   fitness_weights[2] * fit_average_cost_patch + 
                   fitness_weights[3] * fit_landing_cost)
        
        return fitness
    
    def plot_trajectory(self, jump_points, trajectories, show_plot=False):
        # Extract terrain point cloud data
        x_points = np.array([p['position'][0] for p in self.point_clouds.points_t])
        y_points = np.array([p['position'][1] for p in self.point_clouds.points_t])
        z_points = np.array([p['position'][2] for p in self.point_clouds.points_t])
        color = np.array([p['color'] for p in self.point_clouds.points_t])
        size_point = np.array([p['size_point'] for p in self.point_clouds.points_t])

        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Plot terrain
        ax.scatter(x_points, y_points, z_points,
                c=color, s=size_point, alpha=0.6, label='Terrain')

        # Plot jump points
        for i, p in enumerate(jump_points):
            if i == 0:
                ax.scatter(*p, c='green', s=150, marker='o',
                        edgecolors='black', linewidths=2,
                        label='Start Point', zorder=5)
            elif i == len(jump_points) - 1:
                ax.scatter(*p, c='red', s=150, marker='o',
                        edgecolors='black', linewidths=2,
                        label='End Point', zorder=5)
            else:
                ax.scatter(*p, c='blue', s=120, marker='o',
                        edgecolors='black', linewidths=2,
                        label='Waypoint' if i == 1 else '', zorder=5)

        # Plot trajectories
        trajectory_colors = plt.cm.viridis(np.linspace(0, 1, len(trajectories)))

        for i, traj in enumerate(trajectories):
            if traj is not None and traj.size > 0 and traj.shape[0] == 3:
                ax.plot(traj[0], traj[1], traj[2],
                        color=trajectory_colors[i],
                        linewidth=2.5, alpha=1.0,
                        label=f'Jump {i+1}' if i < 3 else '',
                        zorder=4)

        
        all_x = [x_points]
        all_y = [y_points]
        all_z = [z_points]

        for traj in trajectories:
            if traj is not None and traj.size > 0:
                all_x.append(traj[0])
                all_y.append(traj[1])
                all_z.append(traj[2])

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

        # --------------------------------

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Jump Trajectory\nTotal Segments: {len(trajectories)}',
                    fontsize=13, fontweight='bold')

        ax.legend(loc='upper left', fontsize=9)
        ax.view_init(elev=20, azim=45)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        if show_plot:
            plt.show()

        # Trajectory statistics
        print("\n=== Trajectory Statistics ===")
        for i, traj in enumerate(trajectories):
            if traj is not None and traj.size > 0:
                length = np.sum(np.linalg.norm(np.diff(traj, axis=1), axis=0))
                print(f"Segment {i+1}: {traj.shape[1]} points, Length: {length:.2f} m")
    
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


def eval_constraints(c, num_constr, constr_tolerance, debug=False):
    """
    Check constraint vector `c` against blocks described by `num_constr`.
    - c: sequence or 1D numpy array of constraint values
    - num_constr: object or dict with integer fields:
        wall_constraints,
        retraction_force_constraints,
        force_constraints,
        final_constraints,
        via_point
    - constr_tolerance: scalar
    - debug: bool (print detailed per-constraint info)
    """

    # ensure numpy array (1D)
    c = np.array(c).flatten()

    # compute 0-based start indices for each block (Python indexing)
    w0     = 0
    r0     = w0 + int(num_constr['wall_constraints'])
    f0     = r0 + int(num_constr['retraction_force_constraints'])
    final0 = f0 + int(num_constr['force_constraints'])
    via0   = final0 + int(num_constr['final_constraints'])

    # optional debug listing
    if debug:
        for i, val in enumerate(c):
            if i == w0:
                print('wall constraints')
            if i == r0:
                print('retraction_force_constraints')
            if i == f0:
                print('force_constraints')
            if i == final0:
                print('final_point_constraints')
            if i == via0:
                print('via_point constraints')
            print(f"{i} {float(val):.6f}")

    print(colored("Eval Constraints (positive number represents viol.)", "red"))

    # 1) wall constraints
    w_block = c[w0 : w0 + int(num_constr['wall_constraints'])]
    if w_block.size > 0 and np.any(w_block > constr_tolerance):
        print('1) wall mesh constraints violated')
        print(" ".join(f"\033[91m{v:.6f}\033[0m" if v > constr_tolerance else f"{v:.6f}" for v in w_block))

    # 2) retraction force constraints
    r_block = c[r0 : r0 + int(num_constr['retraction_force_constraints'])]
    if r_block.size > 0 and np.any(r_block > constr_tolerance):
        print('2) rope force constraints violated')
        print(" ".join(f"\033[91m{v:.6f}\033[0m" if v > constr_tolerance else f"{v:.6f}" for v in r_block))

    # 3) force constraints (unilateral, actuation, friction, ...)
    f_block = c[f0 : f0 + int(num_constr['force_constraints'])]
    if f_block.size > 0 and np.any(f_block > constr_tolerance):
        print('3) leg force constraints violated')
        # show first few entries (match MATLAB: +1, +2, +3)
        if f_block.size >= 1:
            print(f"3.1 unilateral (Fun > fmin): \033[91m{f_block[0]:.6f}\033[0m" if f_block[0] > constr_tolerance else f"{f_block[0]:.6f}")
        if f_block.size >= 2:
            print(f"3.2 actuation (Fun < fun_max): \033[91m{f_block[1]:.6f}\033[0m" if f_block[1] > constr_tolerance else f"{f_block[1]:.6f}")
        if f_block.size >= 3:
            print(f"3.3 friction (|Fut| < mu*Fun): \033[91m{f_block[2]:.6f}\033[0m" if f_block[2] > constr_tolerance else f"{f_block[2]:.6f}")

    # 4) final point constraints (several subchecks)
    final_block = c[final0 : final0 + int(num_constr['final_constraints'])]
    if final_block.size > 0 and np.any(final_block > constr_tolerance):
        print('4) final point constraint violated')
        # Each check corresponds to offsets 0..4 in MATLAB
        if final_block.size >= 1 and final_block[0] > constr_tolerance:
            print(f"4.1 p_f(y) < ymax_patch : \033[91m{final_block[0]:.6f}\033[0m" if final_block[0] > constr_tolerance else f"{final_block[0]:.6f}")
        if final_block.size >= 2 and final_block[1] > constr_tolerance:
            print(f"4.2 p_f(y) > ymin_patch : \033[91m{final_block[1]:.6f}\033[0m" if final_block[1] > constr_tolerance else f"{final_block[1]:.6f}")
        if final_block.size >= 3 and final_block[2] > constr_tolerance:
            print(f"4.3 p_f(z) < zmax_patch : \033[91m{final_block[2]:.6f}\033[0m" if final_block[2] > constr_tolerance else f"{final_block[2]:.6f}")
        if final_block.size >= 4 and final_block[3] > constr_tolerance:
            print(f"4.4 p_f(z) > zmin_patch : \033[91m{final_block[3]:.6f}\033[0m" if final_block[3] > constr_tolerance else f"{final_block[3]:.6f}")
        if final_block.size >= 5 and final_block[4] > constr_tolerance:
            print(f"4.5 ||pf(x) - wall_x|| < fixed_slack :\033[91m{final_block[4]:.6f}\033[0m" if final_block[4] > constr_tolerance else f"{final_block[4]:.6f}")

    # 5) via point constraints
    via_block = c[via0 : via0 + int(num_constr['via_point'])]
    if via_block.size > 0 and np.any(via_block > constr_tolerance):
        print('5) via point constraint violated')
        print(f"via point :\033[91m{via_block[0]:.6f}\033[0m" if via_block[0] > constr_tolerance else f"{via_block[0]:.6f}")

   
def main():
    print(colored("=== Single Jump Optimizer ===", "cyan", attrs=['bold']))
    
    # Initialize terrain
    print(colored("\n1. Initializing Terrain...", "yellow"))
    
    point_clouds, patches, cost_grid = initialize_terrain_data()
    jump_optimizer = SingleJump(
        P0_INIT, 
        PF_INIT, 
        terrain_manager, 
        point_clouds, 
        patches,
        cost_grid
    )
    print(colored("\n3. Executing Jump Optimization...", "yellow"))
    start_time = time.time()
    
    fitness, p0_adj, pf_adj, trajectory, result = jump_optimizer.execute_jump()
    
    status_map = {
        0: "converged",
        2: "semidef.converg",
        1: "not converged",
        -2: "max number of function evaluations"
    }
    status = status_map.get(result['problem_solved'], "unknown status")
    
    if status not in [0,2]:
        eval_constraints(result['c'], result['num_constr'], result['constr_tolerance'], debug=False)

    
    elapsed_time = time.time() - start_time
    print(colored(f"\n   Optimization completed in {elapsed_time:.2f} seconds!", "green"))
    
    # Display results
    print(colored("\n=== Results ===", "cyan", attrs=['bold']))
    print(f"Final Fitness: {fitness:.2f}")
    print(f"Adjusted Start: {p0_adj}")
    print(f"Adjusted Goal:  {pf_adj}")
    print(f"achived target: {result['achieved_target']}")
    
    
    status_map = {
        0: "converged",
        2: "semidef.converg",
        1: "not converged",
        -2: "max number of function evaluations"
    }
    print(f"Problem Solved: {status_map.get(int(result['problem_solved']), result['problem_solved'])}")

    print(f"Consumed Energy: {result['consumed_energy']:.2f}")
    print(f"Jump Duration: {result['Tf']:.2f}s")
    
    # Plot trajectory
    print(colored("\n5. Plotting Trajectory...", "yellow"))
    jump_optimizer.plot_trajectory([p0_adj, pf_adj], [trajectory], show_plot=False)
    jump_optimizer.plot_mesh_traj([p0_adj, pf_adj], [trajectory])
    print(colored("\n=== Single Jump Optimization Complete! ===", "cyan", attrs=['bold']))


if __name__ == "__main__":
    main()