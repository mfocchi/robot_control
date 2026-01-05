import os
import sys
import time
import json
import numpy as np
from termcolor import colored
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading
import matplotlib.pyplot as plt
import matlab.engine

from cem.algo import CemParams, CrossEntropyMethodMixed
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface
from base_controllers.utils.matlab_conversions import (
    mat_matrix2python,
    mat_vector2python,
)

# =============================
# Parameters about CEM and JUMP
# =============================

# start and goal point
P0_INIT = np.array([0.0, 2.5, -5])
#PF_INIT = np.array([0.0, 4, -3])
PF_INIT = np.array([0.0, 2.5, -15])
MAX_JUMP = 4
# inner_opt_params for optimizer:
Fleg_max = 300.
Fr_max = 90.
Fr_min = 0.
mu = 0.8
number_of_patches_width = 10
number_of_patches_height = 10
mass = 5.
anchor_distance = 5.

# Create inner_opt_params in the EXACT order MATLAB expects (see struct field indices)
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
inner_opt_params['w1'] = 1.  # smooth
inner_opt_params['w2'] = 1.  # hoist work
inner_opt_params['w3'] = 0.
inner_opt_params['T_th'] = 0.05
inner_opt_params['obstacle_avoidance'] = 'mesh'
inner_opt_params['jump_clearance'] = 1.
inner_opt_params['mesh_x'] = None
inner_opt_params['mesh_y'] = None
inner_opt_params['mesh_z'] = None
inner_opt_params['cost_x'] = None
inner_opt_params['cost_y'] = None
inner_opt_params['cost_z'] = None
inner_opt_params['patch_side'] = 1.0
inner_opt_params['contact_normal'] = None

# Set up parameters OUTER LOOP
cem_params = CemParams()
cem_params.seed = int(time.time())
cem_params.n_threads = 15
# General CEM-MD Parameters
cem_params.cem_iters = 15
cem_params.pop_size = 10
cem_params.n_elites = int(cem_params.pop_size * 0.1)
cem_params.decrease_pop_factor = 1.0
cem_params.fraction_elites_reused = 0.0
# Discrete
cem_params.dim_discrete = 5
number_of_patches = number_of_patches_width * number_of_patches_height
# cem_params.n_values = [3] + [(number_of_patches-1) for _ in range(4)]
cem_params.n_values = [MAX_JUMP] + [(number_of_patches) for _ in range(4)]
cem_params.init_probs = [[1.0 / cem_params.n_values[i] for _ in range(cem_params.n_values[i])] for i in range(cem_params.dim_discrete)]
cem_params.min_prob = 0.05
# Continuous
MAX_N_PATCHES = 5
cem_params.dim_continuous = 2 * MAX_N_PATCHES
cem_params.max_value_continuous = np.full(cem_params.dim_continuous, 1.0)
cem_params.min_value_continuous = np.full(cem_params.dim_continuous, 0.0)
cem_params.init_mu_continuous = np.full(cem_params.dim_continuous, 0.5)
cem_params.init_std_continuous = np.full(cem_params.dim_continuous, 1.0)
cem_params.min_std_continuous = np.full(cem_params.dim_continuous, 1e-3)

# [ fit_problem_converged | fit_consumed_energy | fit_average_costmap_patch | fit_landing_costmap ]
fitness_weights = np.array([1., 0.1, 10., 1.])
# =============================
# =============================

# =============================
# MATLAB engine initialization and MULTI THREADS
# =============================
# eng = matlab.engine.start_matlab()
# eng.addpath('../codegen_mesh_landing', nargout=0)
# sys.path.insert(0, '../codegen_mesh_landing')

thread_local = threading.local()

def get_matlab_engine(point_clouds=None, cost_grid=None, terrain_manager=None):
    if not hasattr(thread_local, 'engine'):
        eng = matlab.engine.start_matlab()
        eng.addpath('../codegen_mesh_landing', nargout=0)
        
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

        
# =============================
# =============================

def initialize_terrain_data(terrain_manager, filter_weights):
    # === 1 POINT CLOUD INITIALIZATION ===
    in_point_clouds = terrain_manager.point_cloud
    point_clouds = PointCloudFilter(in_point_clouds)
    anchor_location = np.array(inner_opt_params['p_a1'])
    point_clouds.filter_height_profile(profile="logln", x0=anchor_location[0], weight=filter_weights[3], side_application="depth")
    point_clouds.filter_process_points_pipeline([point_clouds.smoothing_kernel], weight=filter_weights[0], plot=False)
    point_clouds.visualize_cost_map()

    # === 2 PATCHES INITIALIZATION ===
    pc_t = point_clouds.points_t
    patches = PatchSurface(pc_t,number_of_patches_width=number_of_patches_width, number_of_patches_height=number_of_patches_width)
    cost_grid = patches.get_cost_meshgrid()
    
    # Update inner_opt_params with terrain data
    inner_opt_params['mesh_x'] = terrain_manager.mesh_x
    inner_opt_params['mesh_y'] = terrain_manager.mesh_y
    inner_opt_params['mesh_z'] = terrain_manager.mesh_z
    inner_opt_params['cost_x'] = cost_grid
    inner_opt_params['cost_y'] = terrain_manager.mesh_y
    inner_opt_params['cost_z'] = terrain_manager.mesh_z
    inner_opt_params['patch_side'] = 1.0 * patches.patch_width
    
    patches.cost_color()
    
    return point_clouds, patches, cost_grid


class BiLevelOptmizer:
    def __init__(self, terrain_manager, p0, pf, fitness_weights, point_clouds, patches, cost_grid, flag_print_plot=False):
        self.terrain_manager = terrain_manager
        self.p0 = p0
        self.pf = pf
        self.fitness_weights = fitness_weights
        self.flag_print_plot = flag_print_plot
        # terrain creation done outside to avoid recomputation at each thread
        self.point_clouds = point_clouds
        self.patches = patches
        self.cost_grid = cost_grid
        
        # Create result directory
        self.result_dir = os.path.join(os.path.abspath(os.getcwd()), "result")
        os.makedirs(self.result_dir, exist_ok=True)
        
        # Save terrain data to JSON
        self.save_terrain_data()
    
    def save_terrain_data(self):
        """Save point cloud and patches information to JSON file"""
        
        # Extract point cloud data
        point_cloud_data = []
        for point in self.point_clouds.points_t:
            point_data = {
                'position': point['position'].tolist() if isinstance(point['position'], np.ndarray) else point['position'],
                'color': point['color'].tolist() if isinstance(point['color'], np.ndarray) else point['color'],
                'size_point': float(point['size_point']) if hasattr(point['size_point'], 'item') else point['size_point']
            }
            point_cloud_data.append(point_data)
        
        # Extract patches data
        patches_data = []
        for i in range(len(self.patches.patches)):
            patch = self.patches.patches[i]
            
            # Handle different possible patch structures
            patch_info = {
                'id': i,
                'cost': float(self.patches.get_patch_cost(i))
            }
            
            # Add available patch data
            if isinstance(patch, dict):
                if 'center' in patch:
                    patch_info['center'] = patch['center'].tolist() if isinstance(patch['center'], np.ndarray) else patch['center']
                if 'vertices' in patch:
                    patch_info['vertices'] = patch['vertices'].tolist() if isinstance(patch['vertices'], np.ndarray) else patch['vertices']
                if 'bounds' in patch:
                    patch_info['bounds'] = patch['bounds'].tolist() if isinstance(patch['bounds'], np.ndarray) else patch['bounds']
            
            # If patch doesn't have standard keys, try to get basic info
            if 'center' not in patch_info and hasattr(self.patches, 'get_patch_center'):
                try:
                    center = self.patches.get_patch_center(i)
                    patch_info['center'] = center.tolist() if isinstance(center, np.ndarray) else center
                except:
                    pass
            
            patches_data.append(patch_info)
        
        # Create terrain data structure
        terrain_data = {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'num_points': len(point_cloud_data),
                'num_patches': len(patches_data),
                'patch_width': float(self.patches.patch_width),
                'patch_height': float(self.patches.patch_height)
            },
            'point_cloud': point_cloud_data,
            'patches': patches_data,
            'mesh_bounds': {
                'x_min': float(np.min(self.terrain_manager.mesh_x)),
                'x_max': float(np.max(self.terrain_manager.mesh_x)),
                'y_min': float(np.min(self.terrain_manager.mesh_y)),
                'y_max': float(np.max(self.terrain_manager.mesh_y)),
                'z_min': float(np.min(self.terrain_manager.mesh_z)),
                'z_max': float(np.max(self.terrain_manager.mesh_z))
            },
            'start_position': self.p0.tolist(),
            'goal_position': self.pf.tolist()
        }
        
        # Save to file
        filename = "actual_terrain.json"
        save_path = os.path.join(self.result_dir, filename)
        
        with open(save_path, "w") as f:
            json.dump(terrain_data, f, indent=2)
        
        print(colored(f"[SAVE] Terrain data saved to: {save_path}", "cyan"))
        print(colored(f"       - Point cloud points: {len(point_cloud_data)}", "cyan"))
        print(colored(f"       - Patches: {len(patches_data)}", "cyan"))

    def eval_pop(self, input_data):
        # n_iteration matlab for n_thread
        eng = get_matlab_engine(self.point_clouds, self.cost_grid, self.terrain_manager)
        
        local_inner_opt_params = create_inner_opt_params_copy()
                
        jump_log_points = []
        jump_log_traj = []
        xd = input_data[0]
        xc = input_data[1]
        # first discrete variable is number of jumps, the next ones are the of the patches
        n_jumps = xd[0]
        
        ids = []
        fitness = 0.0
        total_consumed_energy = 0.0
        total_landing_cost = 0.0
        
        # Print total number of jumps for this individual
        print(colored(f"[EVAL] Evaluating individual with {n_jumps + 1} total jumps ({n_jumps} intermediate + 1 final)", "blue"))
        # print(f"Number of jumps {n_jumps}\n")
        p0_adj = self.p0.copy()
        p0_adj[0] = self.terrain_manager.wall_surface_eval(
            p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        
        jump_log_points.append(p0_adj.copy())
        
        
        for i in range(n_jumps):
            # print(f"Jump n:{i}\n")
            # following discrete variables represent the id of the patches for the intermediate jumps
            patch_id = xd[1 + i]
            # the continue variables contain the X and Y normalized coordinate of the candidate contact landing points inside the candidate patches
            contact_relative_to_patch_yz = xc[i * 2:i * 2 + 2]  # tra 0 - 1  upper left corner patch

            # print("jump number : ", i)
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
        if (n_jumps+1) < 3:
            fitness -= 500.0  # Valore molto basso per "spaventare" l'algoritmo
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
        
        return fitness, jump_log_points, jump_log_traj, n_jumps, log_result
        
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
            fit_problem_converged = -100
        # print("jump duration", res['Tf'])
        print(f"convergence: {fitness_weights[0]*fit_problem_converged}, energy: {fitness_weights[1]*fit_consumed_energy}, avg_cost: {fitness_weights[2]*fit_average_cost_patch}, land_cost: {fitness_weights[3]*fit_landing_cost}")
        
        fitness = ( fitness_weights[0] * fit_problem_converged + 
                    fitness_weights[1] * fit_consumed_energy +
                    fitness_weights[2] * fit_average_cost_patch + 
                    fitness_weights[3] * fit_landing_cost)
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

def main():
    # ===================================================
    # INPUTS DATA:
    P0_INIT = np.array([0.0, 2.5, -5])
    PF_INIT = np.array([0.0, 2.5, -15])
    
    # WEIGHTS : [fit_problem_converged, fit_consumed_energy, fit_average_cost_patch, fit_landing_cost]
    fitness_weights = np.array([1., 0.1, 10., 1.])
    filter_weights = np.array([1., 1., 1., 1.])
    flag_thread = True
    
    # OUTPUT TOP DATA:
    MAX_TOP_SOLUTIONS = 100
    top_solutions = []  
    
    # Create result directory at the start of main
    result_dir = os.path.join(os.path.abspath(os.getcwd()), "result")
    os.makedirs(result_dir, exist_ok=True)
    
    # ===================================================


    algo = CrossEntropyMethodMixed(cem_params)
    terrain_manager = TerrainManager()
    
    # Initialize terrain data outside of the optimizer class
    point_clouds, patches, cost_grid = initialize_terrain_data(terrain_manager, filter_weights)
    
    # Pass pre-computed data to the optimizer
    optimizer = BiLevelOptmizer(
        terrain_manager, P0_INIT, PF_INIT, 
        fitness_weights=fitness_weights,
        point_clouds=point_clouds,
        patches=patches,
        cost_grid=cost_grid
    )
    
    cost_hist = np.zeros(cem_params.cem_iters)
    best_jump_log_points = None
    best_jump_log_traj = None
    best_fitness = -np.inf
    
    
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
        # Parallel evaluation with ThreadPoolExecutor
        if (flag_thread == True):
            fitness = [0.0] * cem_params.pop_size
            all_log_points = [None] * cem_params.pop_size
            all_log_traj = [None] * cem_params.pop_size
            
            n_workers = cem_params.n_threads
            
            print(colored(f"\n[PARALLEL] Using {n_workers} worker threads", "magenta", attrs=['bold']))    
            print(colored("Multi-threaded CEM Optimizer", "magenta", attrs=['bold']))
            print(colored(f"\n{'='*60}", "magenta"))
            print(colored(f"  Starting Cross Entropy Iteration {k+1}/{cem_params.cem_iters}", "magenta", attrs=['bold']))
            print(colored(f"{'='*60}", "magenta"))
            print(colored("Using ThreadPoolExecutor for parallel evaluation\n", "magenta"))
            
            with ThreadPoolExecutor(max_workers=n_workers) as executor:
                # map futures to their input indices
                future_to_index = {executor.submit(optimizer.eval_pop, inputs[i]): i 
                                   for i in range(cem_params.pop_size)}
                
                completed = 0
                
                for future in as_completed(future_to_index):
                    idx = future_to_index[future]
                    fit_val, log_points, log_traj, n_jumps, log_result = future.result()
                    fitness[idx] = fit_val
                    
                    if (n_jumps + 1) >= 3:
                        current_sol = {
                            'fitness': log_result['fitness'],
                            'points': [p.tolist() for p in log_result['points']],
                            'traj': [t.tolist() if t is not None else None for t in log_result['traj']],
                            'n_jumps': log_result['n_jumps'],
                            'consumed_energy': log_result['consumed_energy'],
                            'landing_cost': log_result['landing_cost'],
                            'iteration': k + 1
                        }
                        top_solutions.append(current_sol)
                        top_solutions.sort(key=lambda x: x['fitness'], reverse=True)
                        
                        if len(top_solutions) > MAX_TOP_SOLUTIONS:
                            top_solutions = top_solutions[:MAX_TOP_SOLUTIONS]
                    # ----------------------------

                    completed += 1
                    # Aggiornamento del best assoluto (opzionale, per mantenere le stampe a video)
                    if fit_val > best_fitness and (n_jumps + 1) >= 3:
                            best_fitness = fit_val
                            best_jump_log_points = log_points
                            best_jump_log_traj = log_traj
                            print(colored(f"[NEW BEST] Indiv {idx}: Fitness {best_fitness:.2f} (Posizione 1 nella Top 100)", "green"))
                    
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
                if result > best_fitness and (n_jumps + 1) >= 3:
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
        if flag_thread == False:
            optimizer.plot_point_traj(best_jump_log_points, best_jump_log_traj)
        print(colored(f"\n{'='*60}", "cyan", attrs=['bold']))
        print(colored(f"  Iteration {k+1} completed in {iter_time:.2f}s", "cyan", attrs=['bold']))
        print(colored(f"  Best value this iteration: {algo.log.best_value:.4f}", "cyan", attrs=['bold']))
        print(colored(f"{'='*60}\n", "cyan", attrs=['bold']))
        
        temp_report = {
            "iteration": k + 1,
            "best_fitness_ever": best_fitness,
            "top_100_solutions": top_solutions,
            "top_100_summary": {
                "count": len(top_solutions),
                "best_fitness": top_solutions[0]['fitness'] if top_solutions else None,
                "worst_fitness": top_solutions[-1]['fitness'] if top_solutions else None,
                "avg_n_jumps": sum(s['n_jumps'] for s in top_solutions) / len(top_solutions) if top_solutions else 0,
                "avg_consumed_energy": sum(s['consumed_energy'] for s in top_solutions) / len(top_solutions) if top_solutions else 0,
                "avg_landing_cost": sum(s['landing_cost'] for s in top_solutions) / len(top_solutions) if top_solutions else 0
            }
        }
        with open(os.path.join(result_dir, "top_100_progress.json"), "w") as f:
            json.dump(temp_report, f, indent=2)
        
    n_workers = cem_params.n_threads
    
    with ThreadPoolExecutor(max_workers=n_workers) as executor:
        executor.map(lambda x: shutdown_engine(), range(n_workers))
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
        "solution_algo": {
            "elite_cost_history": cost_hist.tolist(),
            "best_discrete": xd.tolist(),
            "best_continuous": xc.tolist(),
            "wall_time_sec": wall_time,
        },
        "top_solutions": top_solutions[:10] if len(top_solutions) >= 10 else top_solutions,
        "statistics": {
            "total_solutions_found": len(top_solutions),
            "best_fitness": best_fitness,
            "avg_n_jumps": sum(s['n_jumps'] for s in top_solutions) / len(top_solutions) if top_solutions else 0,
            "avg_consumed_energy": sum(s['consumed_energy'] for s in top_solutions) / len(top_solutions) if top_solutions else 0,
            "avg_landing_cost": sum(s['landing_cost'] for s in top_solutions) / len(top_solutions) if top_solutions else 0
        }
    }

    # Save to file
    filename = f"cem_solution_final.json"
    save_path = os.path.join(result_dir, filename)
    with open(save_path, "w") as f:
        json.dump(report, f, indent=2)
    print(colored(f"[SAVE] Final report saved to: {save_path}", "blue"))
    
    # Also save complete top 100 solutions
    top_100_filename = "top_100_solutions_final.json"
    top_100_save_path = os.path.join(result_dir, top_100_filename)
    with open(top_100_save_path, "w") as f:
        json.dump({"top_solutions": top_solutions}, f, indent=2)
    print(colored(f"[SAVE] Top 100 solutions saved to: {top_100_save_path}", "blue"))

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