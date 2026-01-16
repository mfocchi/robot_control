import os
import sys
import numpy as np
import matlab
import time
from algo_patch import CemParams
import time
from datetime import datetime
import json
from termcolor import colored
import threading
import matlab.engine
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface
from base_controllers.components.terrain_manager import TerrainManager
# ================================================
# BASE DATA FOR MULTIPLE JUMPS CLIMBING CONTROLLER
# ================================================
# start and goal point
P0_INIT = np.array([0.0, 1.5, -6.4])
PF_PATCH_INIT = np.array([0.0, 3.25, -6.4])
PF_INIT = PF_PATCH_INIT
MAX_JUMP = 4
THREADS = 5
flag_thread = False
MAIN_DIRECTORY = "result_patch_11"

Fleg_max = 600.
Fr_max = 90.
Fr_min = 10.
number_of_patches_width = 10
number_of_patches_height = 10
mass = 10.
anchor_distance = 10.
# [ fit_problem_converged | fit_consumed_energy | fit_average_costmap_patch | fit_landing_costmap ]
fitness_weights = np.array([1., 1.0,10., 10.])
# weights for point cloud filtering
# filter_weights = np.array([100., 1000., 0,10.0]) #smoothing, first derivative, second derivative, weight_gauss_cost
filter_weights = np.array([100., 10., 0,100.0])
# ================================================
# INNER LOOP OPTIMIZER PARAMETERS
# ================================================
# Create inner_opt_params in the EXACT order MATLAB expects
mu = 0.8
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
inner_opt_params['w2'] = 0.  # hoist work
inner_opt_params['w3'] = 300.
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

# ================================================
# OUTER LOOP OPTIMIZER PARAMETERS
# ================================================
MAX_N_PATCHES = MAX_JUMP + 1
# Set up parameters OUTER LOOP
cem_params = CemParams()
cem_params.seed = int(time.time())
cem_params.n_threads = THREADS
# General CEM-MD Parameters
cem_params.cem_iters = 1
cem_params.pop_size =10
cem_params.n_elites = int(cem_params.pop_size * 0.3)
cem_params.decrease_pop_factor = 0.0 # NON RIDURRE LA POPOLAZIONE
cem_params.fraction_elites_reused = 0.1 
# Discrete
cem_params.dim_discrete = MAX_N_PATCHES
number_of_patches = number_of_patches_width * number_of_patches_height
# cem_params.n_values = [3] + [(number_of_patches-1) for _ in range(4)]
cem_params.n_values = [MAX_JUMP] + [(number_of_patches) for _ in range(4)]
cem_params.init_probs = [[1.0 / cem_params.n_values[i] for _ in range(cem_params.n_values[i])] for i in range(cem_params.dim_discrete)]
cem_params.min_prob = 0.01
# Continuous
cem_params.dim_continuous = 2 * MAX_N_PATCHES # ho posizioni x e y
cem_params.max_value_continuous = np.full(cem_params.dim_continuous, 1.0)
cem_params.min_value_continuous = np.full(cem_params.dim_continuous, 0.0)
cem_params.init_mu_continuous = np.full(cem_params.dim_continuous, 0.5)
cem_params.init_std_continuous = np.full(cem_params.dim_continuous, 0.20)
cem_params.min_std_continuous = np.full(cem_params.dim_continuous, 0.05)

# ================================================
# PLOTTING PARAMETERS
# ================================================

FILE_TERRAIN_POINTS = f"{MAIN_DIRECTORY}/actual_point_terrain.json"
FILE_TERRAIN_PATCHES = f"{MAIN_DIRECTORY}/actual_patch_terrain.json"
FILE_PROGRESS = f"{MAIN_DIRECTORY}/cem_iteration_history.json"
ITERATIONS_FOLDER = f"{MAIN_DIRECTORY}/iteration_reports"
FILE_BEST_LOG = f"{MAIN_DIRECTORY}/best_trajectory_log.json"
FILE_SAVE_PARAMS = f"{MAIN_DIRECTORY}/simulation_params.json"
# ================================================
# COMMON FUNCTIONS
# ================================================
result_dir = os.path.join(os.path.abspath(os.getcwd()), MAIN_DIRECTORY)
os.makedirs(result_dir, exist_ok=True)

# Terrain configuration values for rock terrain, otherwise stay in default
# wall_depth = 1            
# grid_size = 100
# max_ridge_depth = 0.5     
# seed = 30                 
# Lz = -50                  
# Ly = 10          
# terrain_manager = TerrainManager(wall_depth=wall_depth, grid_size=grid_size, max_ridge_depth=max_ridge_depth, seed=seed, Lz=Lz, Ly=Ly)
terrain_manager  = TerrainManager()

def initialize_terrain_data(terrain_manager, filter_weights, number_of_patches_width, number_of_patches_height, inner_opt_params):
    terrain_params = []
    # === 1 POINT CLOUD INITIALIZATION ===
    in_point_clouds = terrain_manager.point_cloud
    point_clouds = PointCloudFilter(in_point_clouds)
    anchor_location = np.array(inner_opt_params['p_a1'])
    
    
    # filtro con cambio di costo e colore in base all'altezza
    point_clouds.filter_height_profile(x0=0.0, scale=1.0,side_application="depth", profile="logln")
    # point_clouds.visualize_cost_map()
    # print("\n[INIT] === Smoothing Filter ===")
    # kernel = [point_clouds.smoothing_kernel] 
    # point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[0], plot=False)
    
    print("\n[INIT] === First Derivative (Gradient) ===")
    kernel = [point_clouds.sobel_y, point_clouds.sobel_z] 
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[1], plot=False)
    
    # print("\n[INIT] === Second Derivative (Laplacian) ===")
    # kernel = [point_clouds.laplacian_kernel] 
    # point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[2], plot=False)
    
    # point_clouds.visualize_cost_map()

    # === 2 PATCHES INITIALIZATION ===
    pc_t = point_clouds.points_t
    patches = PatchSurface(pc_t,number_of_patches_width=number_of_patches_width, number_of_patches_height=number_of_patches_height)
    cost_grid = patches.get_cost_meshgrid()
    # patches.plot_patch(99)
    # Update inner_opt_params with terrain data
    inner_opt_params['mesh_x'] = terrain_manager.mesh_x
    inner_opt_params['mesh_y'] = terrain_manager.mesh_y
    inner_opt_params['mesh_z'] = terrain_manager.mesh_z
    
    inner_opt_params['cost_x'] = cost_grid
    inner_opt_params['cost_y'] = terrain_manager.mesh_y
    inner_opt_params['cost_z'] = terrain_manager.mesh_z
    inner_opt_params['patch_side'] = 1.0 * patches.patch_width
    patches.gaussian_cost_all_patch(weight_gauss_cost=filter_weights[3])
    patches.visualize_full_cost_map()
    
    terrain_params.append({
        'anchor_location': point_clouds,
        'patches': patches,
        'cost_grid': cost_grid
    })
    # save the terrain data 
    save_terrain_data(terrain_manager,point_clouds, patches)
    return point_clouds, patches, cost_grid

def save_terrain_data(terrain_manager,point_clouds, patches):
    def convert_to_serializable(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, (np.integer, np.floating)):
                return obj.item()
            elif isinstance(obj, dict):
                return {k: convert_to_serializable(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert_to_serializable(item) for item in obj]
            elif isinstance(obj, tuple):
                return [convert_to_serializable(item) for item in obj]
            else:
                return obj
    # =============================================
    # FILE 1: actual_point_terrain.json
    # =============================================
    point_cloud_data = []
    for point in point_clouds.points_t:
        point_data = {
            'position': convert_to_serializable(point['position']),
            'color': convert_to_serializable(point['color']),
            'light': convert_to_serializable(point['light']),
            'size_point': convert_to_serializable(point['size_point']),
            'cost': convert_to_serializable(point['cost'])
        }
        point_cloud_data.append(point_data)
    
    terrain_points_data = {
        'metadata': {
            'timestamp': datetime.now().isoformat(),
            'num_points': len(point_cloud_data),
        },
        'mesh_bounds': {
            'x_min': float(np.min(terrain_manager.mesh_x)),
            'x_max': float(np.max(terrain_manager.mesh_x)),
            'y_min': float(np.min(terrain_manager.mesh_y)),
            'y_max': float(np.max(terrain_manager.mesh_y)),
            'z_min': float(np.min(terrain_manager.mesh_z)),
            'z_max': float(np.max(terrain_manager.mesh_z))
        },
        'start_position': P0_INIT.tolist(),
        'goal_position': PF_PATCH_INIT.tolist(),
        'points': point_cloud_data
    }
    
    # Save points file
    with open(FILE_TERRAIN_POINTS, "w") as f:
        json.dump(terrain_points_data, f, indent=2)
    
    print(colored(f"[SAVE] Point terrain data saved to: {FILE_TERRAIN_POINTS}", "cyan"))
    print(colored(f"       - Point cloud points: {len(point_cloud_data)}", "cyan"))
    
    # =============================================
    # FILE 2: actual_patch_terrain.json
    # =============================================
    patches_data = []
    for patch in patches.patches:
        patch_info = {
            'id': convert_to_serializable(patch['id']),
            'centroid': convert_to_serializable(patch['centroid']),
            'points_in_patch': convert_to_serializable(patch['points_in_patch']),
            'cost_patch': convert_to_serializable(patch['cost_patch'])
        }
        patches_data.append(patch_info)
    
    terrain_patches_data = {
        'metadata': {
            'timestamp': datetime.now().isoformat(),
            'num_patches': len(patches_data),
            'patch_width': float(patches.patch_width),
            'patch_height': float(patches.patch_height)
        },
        'patches': patches_data
    }
    
    # Save patches file
    patches_filename = "actual_patch_terrain.json"
    patches_save_path = os.path.join(result_dir, patches_filename)
    
    with open(patches_save_path, "w") as f:
        json.dump(terrain_patches_data, f, indent=2)
    
    print(colored(f"[SAVE] Patch terrain data saved to: {patches_save_path}", "cyan"))
    print(colored(f"       - Patches: {len(patches_data)}", "cyan"))
    
    # save also the parameters used
    save_params()
    
def to_json_serializable(obj):
    if isinstance(obj, dict):
        return {k: to_json_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [to_json_serializable(v) for v in obj]
    elif isinstance(obj, tuple):
        return [to_json_serializable(v) for v in obj]
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.generic):  # np.float64, np.int32, ecc.
        return obj.item()
    elif hasattr(obj, "_data"):  
        # matlab.double → lista
        return to_json_serializable(list(obj))
    else:
        return obj

def save_params():
    params = {
        'START': P0_INIT.tolist(),
        'GOAL': PF_PATCH_INIT.tolist(),        
        'inner_opt_params_order':{
            'MAX_JUMP': MAX_JUMP,
            'THREADS': THREADS,
            'Fleg_max': Fleg_max,
            'Fr_max': Fr_max,
            'Fr_min': Fr_min,
            'mass': mass,
            'anchor_distance': anchor_distance,
            'fitness_weights': fitness_weights.tolist(),
            'filter_weights': filter_weights.tolist(),
            'inner_opt_params': inner_opt_params
        },
        'cem_params': {
            'seed': cem_params.seed,
            'n_threads': cem_params.n_threads,
            'cem_iters': cem_params.cem_iters,
            'pop_size': cem_params.pop_size,
            'n_elites': cem_params.n_elites,
            'decrease_pop_factor': cem_params.decrease_pop_factor,
            'fraction_elites_reused': cem_params.fraction_elites_reused,
            'dim_discrete': cem_params.dim_discrete,
            'n_values': cem_params.n_values,
            'init_probs': cem_params.init_probs,
            'min_prob': cem_params.min_prob,
            'dim_continuous': cem_params.dim_continuous,
            'max_value_continuous': cem_params.max_value_continuous.tolist(),
            'min_value_continuous': cem_params.min_value_continuous.tolist(),
            'init_mu_continuous': cem_params.init_mu_continuous.tolist(),
            'init_std_continuous': cem_params.init_std_continuous.tolist(),
            'min_std_continuous': cem_params.min_std_continuous.tolist()
        }
    }
    
    params = to_json_serializable(params)
    
    with open(FILE_SAVE_PARAMS, "w") as f:
        json.dump(params, f, indent=2)
    print(colored(f"[SAVE] Simulation parameters saved to: {FILE_SAVE_PARAMS}", "cyan"))

# ================================================
# MATLAB SETUP
# ================================================

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
    