import sys
import os
import time
from datetime import datetime
import json
import numpy as np
from termcolor import colored
import matplotlib.pyplot as plt
from multiprocessing import Manager, Lock
import threading
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed
from cem.algo import CemParams, CrossEntropyMethodMixed
from base_controllers.utils.matlab_conversions import mat_matrix2python, mat_vector2python
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface

import matlab.engine

thread_local = threading.local()

Fleg_max = 300.
Fr_max = 90.
Fr_min = 0.
mu = 0.8
mass = 5.

anchor_distance = 5.
inner_opt_params = {}
inner_opt_params['m'] = mass
inner_opt_params['num_params'] = 4.
inner_opt_params['int_method'] = 'rk4'
inner_opt_params['N_dyn'] = 30.
inner_opt_params['FRICTION_CONE'] = 1.
inner_opt_params['int_steps'] = 5.
inner_opt_params['b'] = anchor_distance
inner_opt_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
inner_opt_params['p_a2'] = matlab.double([0., inner_opt_params['b'], 0.]).reshape(3, 1)
inner_opt_params['g'] = 9.81
inner_opt_params['w1'] = 1.
inner_opt_params['w2'] = 1.
inner_opt_params['w3'] = 0.
inner_opt_params['w4'] = 0.
inner_opt_params['w5'] = 0.
inner_opt_params['w6'] = 0.
inner_opt_params['T_th'] = 0.05
inner_opt_params['obstacle_avoidance'] = 'mesh'
inner_opt_params['jump_clearance'] = 0.3

# CEM Parameters
cem_params = CemParams()
cem_params.seed = int(time.time())
cem_params.n_threads = 10  
# General CEM-MD Parameters
cem_params.cem_iters = 15
cem_params.pop_size = 150
cem_params.n_elites = int(cem_params.pop_size * 0.6)
cem_params.decrease_pop_factor = 1.0
cem_params.fraction_elites_reused = 0.0
# Discrete
cem_params.dim_discrete = 5
number_of_patches = 20
cem_params.n_values = [3] + [(number_of_patches-1) for _ in range(4)]
cem_params.init_probs = [[1.0 / cem_params.n_values[i] for _ in range(cem_params.n_values[i])] for i in range(cem_params.dim_discrete)]
cem_params.min_prob = 0.05
# Continuous
MAX_N_PATCHES = 7
cem_params.dim_continuous = 2 * MAX_N_PATCHES
cem_params.max_value_continuous = np.full(cem_params.dim_continuous, 1.0)
cem_params.min_value_continuous = np.full(cem_params.dim_continuous, 0.0)
cem_params.init_mu_continuous = np.full(cem_params.dim_continuous, 0.5)
cem_params.init_std_continuous = np.full(cem_params.dim_continuous, 1.0)
cem_params.min_std_continuous = np.full(cem_params.dim_continuous, 1e-3)

def get_matlab_engine():
    if not hasattr(thread_local, 'engine'):
        thread_local.engine = matlab.engine.start_matlab()
        thread_local.engine.addpath('../codegen_mesh', nargout=0)
        print(f"Created MATLAB engine for thread {threading.current_thread().name}")
    return thread_local.engine

def close_matlab_engines():
    if hasattr(thread_local, 'engine'):
        thread_local.engine.quit()
        del thread_local.engine
        print("MATLAB engine closed")

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
        'w4': inner_opt_params['w4'],
        'w5': inner_opt_params['w5'],
        'w6': inner_opt_params['w6'],
        'T_th': inner_opt_params['T_th'],
        'obstacle_avoidance': inner_opt_params['obstacle_avoidance'],
        'jump_clearance': inner_opt_params['jump_clearance'],
    }


class BiLevelOptimizer:
    def __init__(self, terrain_manager, p0, pf, fitness_weights, filter_weights, flag_print_plot=False):
        self.terrain_manager = terrain_manager
        self.p0 = p0
        self.pf = pf
        self.fitness_weights = fitness_weights
        self.filter_weights = filter_weights
        self.flag_print_plot = flag_print_plot
        # Point cloud initialization =================
        self.in_point_clouds = self.terrain_manager.point_cloud
        self.point_clouds = PointCloudFilter(self.in_point_clouds)
        anchor_location = np.array(inner_opt_params['p_a1']) # use to center filters
        self.point_clouds.filter_height_profile(profile="logln", x0=anchor_location[0], 
                                               weight=self.filter_weights[3], side_application="depth")
        
        # Apply filters
        # self.point_clouds.filter_process_points([self.point_clouds.smoothing_kernel], 
        #                                        weight=self.filter_weights[0], plot=False)
        kernel = [self.point_clouds.sobel_y, self.point_clouds.sobel_z] #first derivative
        self.point_clouds.filter_process_points(kernel, weight=self.filter_weights[1], plot=False)
        
        # Patches initialization =================
        pc_t = self.point_clouds.points_t
        self.patches = PatchSurface(pc_t)
        # self.patches.cost_color()
        # self.patches.plot_patches()
        # print(self.point_clouds.points_t)
        # self.point_clouds.print_map_pc()


    def eval_pop(self, input_data):
            # n_iteration matlab for n_thread
            eng = get_matlab_engine()
            
            local_inner_opt_params = create_inner_opt_params_copy()
            
            jump_log_points = []
            jump_log_traj = []
            xd = input_data[0]
            xc = input_data[1]
            
            # first discrete variable is number of jumps, the next ones are the of the patches
            n_jumps = xd[0]
            fitness = 0.0
            
            # Print total number of jumps for this individual
            print(colored(f"[EVAL] Evaluating individual with {n_jumps + 1} total jumps ({n_jumps} intermediate + 1 final)", "blue"))
            
            p0_adj = self.p0.copy()
            p0_adj[0] = self.terrain_manager.wall_surface_eval(
                p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            jump_log_points.append(p0_adj.copy())
            
            # Process intermediate jumps
            for i in range(n_jumps):
                # print(f"[JUMP {i+1}/{n_jumps}] Processing intermediate jump")
                # following discrete variables represent the id of the patches for the intermediate jumps
                patch_id = xd[1 + i]
                # the continue variables contain the X and Y normalized coordinate of the candidate contact landing points inside the candidate patches
                contact_relative_to_patch_yz = xc[i * 2:i * 2 + 2]
                
                # print(colored(f"[JUMP {i+1}] Patch ID: {patch_id}, Relative coords: {contact_relative_to_patch_yz}", "yellow"))
                # computes 0, Y, Z  absolute coordinates of candidate landing location
                landing_abs_pos = self.patches.getAbsolutePoseOfPointInsidePatch(
                    patch_id, contact_relative_to_patch_yz[0], 
                    contact_relative_to_patch_yz[1], scale=1.0)
                # print(colored(f"[JUMP {i+1}] Patch ID: {patch_id}, absolute coords: {landing_abs_pos}", "yellow"))
                pf_adj = landing_abs_pos.copy()
                
                # adjust X coordinate to terrain shape for both liftoff and landing points
                p0_adj[0] = self.terrain_manager.wall_surface_eval(
                    p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                    self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
                pf_adj[0] = self.terrain_manager.wall_surface_eval(
                    pf_adj[2], pf_adj[1], self.terrain_manager.mesh_x,
                    self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
                
                # compute normal at liftoff
                liftoff_normal = self.terrain_manager.wall_normal_eval(
                    p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                    self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
                
                # absolute point is in pf_adj
                local_inner_opt_params['mesh_x'] = self.terrain_manager.mesh_x
                local_inner_opt_params['mesh_y'] = self.terrain_manager.mesh_y
                local_inner_opt_params['mesh_z'] = self.terrain_manager.mesh_z
                local_inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)
                
                # Run optimization
                res = eng.optimize_cpp_mex(
                    matlab.double(p0_adj), matlab.double(pf_adj), 
                    Fleg_max, Fr_max, Fr_min, mu, local_inner_opt_params)
                
                jump_log_traj.append(mat_matrix2python(res['p']))
                fitness += self.calc_fitness(res, patch_id=patch_id, 
                                            contact_abs_pos_yz=pf_adj[1:])
                
                # Add landing point AFTER the trajectory
                jump_log_points.append(pf_adj.copy())
                p0_adj = pf_adj.copy()
            
    
            # last jump is to pf
            #  absolute coordinates of FINAL landing location    
            pf_adj = self.pf.copy()
            # adjust X coordinate to terrain shape for both liftoff and landing points
            pf_adj[0] = self.terrain_manager.wall_surface_eval(
                pf_adj[2], pf_adj[1], self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            p0_adj[0] = self.terrain_manager.wall_surface_eval(
                p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            
            # compute normal at liftoff
            liftoff_normal = self.terrain_manager.wall_normal_eval(
                p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
                self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            
            # Use local copy instead of global
            local_inner_opt_params['mesh_x'] = self.terrain_manager.mesh_x
            local_inner_opt_params['mesh_y'] = self.terrain_manager.mesh_y
            local_inner_opt_params['mesh_z'] = self.terrain_manager.mesh_z
            local_inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)
            
            res = eng.optimize_cpp_mex(
                matlab.double(p0_adj), matlab.double(pf_adj), 
                Fleg_max, Fr_max, Fr_min, mu, local_inner_opt_params)
            
            fitness += self.calc_fitness(res)
            ref_com = mat_matrix2python(res['p'])
            jump_log_traj.append(ref_com)
            jump_log_points.append(pf_adj.copy())
            
            if (self.flag_print_plot == True):
                print(colored(f"[RESULT] Final jump: p0={p0_adj}, pf={pf_adj}", "green"))
                print(colored(f"[RESULT] Total fitness: {fitness:.4f}", "green", attrs=['bold']))
                self.plot_point_traj(jump_log_points, jump_log_traj)
            
            return fitness, jump_log_points, jump_log_traj, n_jumps
            
    def calc_fitness(self,res, patch_id=None, contact_abs_pos_yz=None):
        fit_average_cost_patch = 0.
        fit_landing_cost = 0.
        
        # filter apply
        if (patch_id is not None and  contact_abs_pos_yz is not None):
            #compute cost for landing candidate
            fit_landing_cost = self.patches.get_cost_in_point(patch_id, contact_abs_pos_yz)
            #compute average cost on patch to see how bad /good is terrain there
            fit_average_cost_patch = self.patches.get_patch_cost(patch_id)
            #if fit_landing_costmap is None:

        fit_consumed_energy = res['consumed_energy']
        if (res['problem_solved']) == 1 or (res['problem_solved']==2): #convergence / semidefinite solution
            fit_problem_converged = 0.
        else: #problem did not converge
            fit_problem_converged = 100.
        fitness = ( self.fitness_weights[0] * fit_problem_converged + 
                    self.fitness_weights[1] * fit_consumed_energy +
                    self.fitness_weights[2] * fit_average_cost_patch + 
                    self.fitness_weights[3] * fit_landing_cost)
        
        if (self.flag_print_plot == True):
            print(colored(f"[FITNESS] Jump duration: {res['Tf']:.3f}s", "blue"))
            print(colored(f"[FITNESS] Total: {fitness:.4f} = "
                         f"convergence: {self.fitness_weights[0]*fit_problem_converged:.2f} + "
                         f"energy: {self.fitness_weights[1]*fit_consumed_energy:.2f} + "
                         f"avg_cost: {self.fitness_weights[2]*fit_average_cost_patch:.2f} + "
                         f"land_cost: {self.fitness_weights[3]*fit_landing_cost:.2f}", "blue"))
        
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
        
        # trajectory_colors = plt.cm.viridis(np.linspace(0, 1, len(jump_log_traj)))
        
        for i, traj in enumerate(jump_log_traj):
            if traj is not None and traj.size > 0:
                # traj is expected to be shape (3, N) where rows are [x, y, z]
                if traj.ndim == 2 and traj.shape[0] == 3:
                    ax.plot(traj[0, :], traj[1, :], traj[2, :], 
                        color='red', linewidth=2.5, 
                        label=f'Jump {i+1}' if i < 3 else '',
                        alpha=0.9, zorder=4)
                else:
                    print(colored(f"[WARNING] Trajectory {i} has unexpected shape {traj.shape}", "yellow"))
        
        ax.set_xlabel('X (m) - Height', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_zlabel('Z (m)', fontsize=11)
        ax.set_title(f'Optimized Jumping Path\nTotal Jumps: {len(jump_log_traj)}', 
                    fontsize=13, fontweight='bold')
        ax.legend(loc='upper left', fontsize=9)
        ax.view_init(elev=20, azim=45)

        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # Print trajectory statistics
        print(colored("\n[TRAJECTORY STATISTICS]", "cyan", attrs=['bold']))
        for i, traj in enumerate(jump_log_traj):
            if traj is not None and traj.size > 0:
                traj_length = np.sum(np.sqrt(np.sum(np.diff(traj, axis=1)**2, axis=0)))
                print(colored(f"  Jump {i+1}: Length = {traj_length:.2f}m", "cyan"))


def main():
    # ===================================================
    # INPUTS DATA:
    P0_INIT = np.array([0.0, 2.5, -5])
    PF_INIT = np.array([0.0, 2.5, -15])
    
    # WEIGHTS : [fit_problem_converged, fit_consumed_energy, fit_average_cost_patch, fit_landing_cost]
    fitness_weights = np.array([10000., 1., 1., 1.])
    filter_weights = np.array([1., 1., 1., 1.])
    flag_thread = True
    # ===================================================


    algo = CrossEntropyMethodMixed(cem_params)
    terrain_manager = TerrainManager()
    optimizer = BiLevelOptimizer(terrain_manager, P0_INIT, PF_INIT, fitness_weights=fitness_weights, filter_weights=filter_weights)
    cost_hist = np.zeros(cem_params.cem_iters)
    best_jump_log_points = None
    best_jump_log_traj = None
    best_fitness = np.inf
    
    
    start = time.time()
    
    for k in range(cem_params.cem_iters):
        iter_start = time.time()
        
        # Use ThreadPoolExecutor for parallelization    
        
        # Generate population
        algo.generate_population_discrete()
        algo.generate_population_continuous()
        xd = algo.population_discrete # shape: dim_discrete x pop_size
        xc = algo.population_continuous # shape: dim_continuous x pop_size
        
        # Organise inputs into a 2D matrix where we have as columns
        inputs = [[xd[:, i].tolist(), xc[:, i].tolist()] for i in range(cem_params.pop_size)]
        
        fitness = []
        temp_log_points = []
        temp_log_traj = []
        
        # Parallel evaluation with ThreadPoolExecutor
        if (flag_thread == True):
            
            n_workers = cem_params.n_threads
            print(colored(f"\n[PARALLEL] Using {n_workers} worker threads", "magenta", attrs=['bold']))    
            print(colored("Multi-threaded CEM Optimizer", "magenta", attrs=['bold']))
            print(colored(f"\n{'='*60}", "magenta"))
            print(colored(f"  Starting Cross Entropy Iteration {k+1}/{cem_params.cem_iters}", "magenta", attrs=['bold']))
            print(colored(f"{'='*60}", "magenta"))
            print(colored("Using ThreadPoolExecutor for parallel evaluation\n", "magenta"))
            
            with ThreadPoolExecutor(max_workers=n_workers) as executor:
                futures = {executor.submit(optimizer.eval_pop, population_inputs): i 
                        for i, population_inputs in enumerate(inputs)}
                
                completed = 0
                
                for future in as_completed(futures):
                    
                    result, log_points, log_traj, n_jumps = future.result()
                    fitness.append(result)
                    temp_log_points.append(log_points)
                    temp_log_traj.append(log_traj)
                    
                    completed += 1
                    if completed % 10 == 0 or completed == len(inputs):
                        print(colored(f"[PROGRESS] {completed}/{len(inputs)} individuals evaluated", "blue"))
                    
                    # Track best solution only if at least 3 total jumps
                    if result < best_fitness and (n_jumps + 1) >= 3:
                        best_fitness = result
                        best_jump_log_points = log_points
                        best_jump_log_traj = log_traj
                        print(colored(f"[NEW BEST] Fitness: {best_fitness:.2f} with {n_jumps + 1} jumps", "green", attrs=['bold']))
                        # optimizer.plot_point_traj(best_jump_log_points, best_jump_log_traj)
        else:
            print(colored(f"\n{'='*60}", "yellow"))
            print(colored("Using sequential evaluation", "yellow", attrs=['bold']))
            print(colored(f"{'='*60}\n", "yellow"))
            
            for i, population_inputs in enumerate(inputs, start=1):
                
                optimizer.flag_print_plot = True
                # Set plot=True to visualize each trajectory
                result, log_points, log_traj, n_jumps = optimizer.eval_pop(population_inputs)
                fitness.append(result)
                print(colored(f"\n[COMPLETE] Individual {i}/{len(inputs)} of iteration {k+1} finished, fitness = {result:.4f}\n", "red", attrs=['bold']))
                
                # Track best solution only if at least 3 total jumps
                if result < best_fitness and (n_jumps + 1) >= 3:
                    best_fitness = result
                    best_jump_log_points = log_points
                    best_jump_log_traj = log_traj
                    print(colored(f"[NEW BEST] Fitness: {best_fitness:.2f} with {n_jumps + 1} jumps", "green", attrs=['bold']))
                    # optimizer.plot_point_traj(best_jump_log_points, best_jump_log_traj)
            
        # Update distributions
        algo.evaluate_population(fitness)
        algo.update_distributions()
        cost_hist[k] = algo.log.best_value
        
        iter_time = time.time() - iter_start
        
        optimizer.plot_point_traj(best_jump_log_points, best_jump_log_traj)
        print(colored(f"\n{'='*60}", "cyan", attrs=['bold']))
        print(colored(f"  Iteration {k+1} completed in {iter_time:.2f}s", "cyan", attrs=['bold']))
        print(colored(f"  Best value this iteration: {algo.log.best_value:.4f}", "cyan", attrs=['bold']))
        print(colored(f"{'='*60}\n", "cyan", attrs=['bold']))
    
    # Save wall-time
    end = time.time()
    wall_time = end - start

    xd = algo.best_discrete
    xc = algo.best_continuous

    # Generate and save report json
    report = {
        "metadata": {
            "timestamp": datetime.now().isoformat(),
            "iterations": k + 1,
        },
        "solution": {
            "elite_cost_history": cost_hist.tolist(),
            "best_discrete": xd.tolist(),
            "best_continuous": xc.tolist(),
            "wall_time_sec": wall_time,
        },
    }

    # Save to file
    # filename = f"cem_solution.json"
    # save_path = os.path.join(os.path.abspath(os.getcwd()), filename)
    # with open(save_path, "w") as f:
    #     json.dump(report, f, indent=2)
    # print(colored(f"[SAVE] Report saved to: {save_path}", "blue"))

    # Plot best trajectory at the end
    print(colored(f"\n{'='*70}", "green", attrs=['bold']))
    print(colored(f"  OPTIMIZATION FINISHED!", "green", attrs=['bold']))
    print(colored(f"  Best Fitness: {best_fitness:.4f}", "green", attrs=['bold']))
    print(colored(f"  Total Time: {wall_time:.2f}s", "green", attrs=['bold']))
    print(colored(f"{'='*70}\n", "green", attrs=['bold']))
    
    if best_jump_log_points and best_jump_log_traj:
        optimizer.plot_point_traj(best_jump_log_points, best_jump_log_traj)
    else:
        print(colored("[ERROR] Could not plot best trajectory. No solution found or tracking issue.", "red", attrs=['bold']))

if __name__ == "__main__":
    try:
        main()
    finally:
        close_matlab_engines()