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
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface
# ================================================
# BASE DATA FOR MULTIPLE JUMPS CLIMBING CONTROLLER
# ================================================
# start and goal point
P0_INIT = np.array([0.0, 2.5, -5])
PF_PATCH_INIT = np.array([0.0, 4.5, -7])
PF_INIT = PF_PATCH_INIT
MAX_JUMP = 4
THREADS = 5
flag_thread = True
MAIN_DIRECTORY = "result_patch_9"

Fleg_max = 300.
Fr_max = 90.
Fr_min = 10.
number_of_patches_width = 10
number_of_patches_height = number_of_patches_width
mass = 5.
anchor_distance = 5.
# [ fit_problem_converged | fit_consumed_energy | fit_average_costmap_patch | fit_landing_costmap ]
fitness_weights = np.array([1., 1.0,10., 10.])
# weights for point cloud filtering
filter_weights = np.array([100., 1000., 0,10.0]) #smoothing, first derivative, second derivative, weight_gauss_cost

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
cem_params.cem_iters = 10
cem_params.pop_size = 100
cem_params.n_elites = int(cem_params.pop_size * 0.3)
cem_params.decrease_pop_factor = 0.0 # a ogni iterazione la popolazione decresce di ...
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
# ================================================
# COMMON FUNCTIONS
# ================================================
result_dir = os.path.join(os.path.abspath(os.getcwd()), MAIN_DIRECTORY)
os.makedirs(result_dir, exist_ok=True)

def initialize_terrain_data(terrain_manager, filter_weights, number_of_patches_width, number_of_patches_height, inner_opt_params):
    terrain_params = []
    # === 1 POINT CLOUD INITIALIZATION ===
    in_point_clouds = terrain_manager.point_cloud
    point_clouds = PointCloudFilter(in_point_clouds)
    anchor_location = np.array(inner_opt_params['p_a1'])
    
    
    # filtro con cambio di costo e colore in base all'altezza
    point_clouds.filter_height_profile(x0=0.0, scale=1.0,side_application="depth", profile="logln")
    point_clouds.visualize_cost_map()
    # print("\n[INIT] === Smoothing Filter ===")
    # kernel = [point_clouds.smoothing_kernel] 
    # point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[0], plot=False)
    
    print("\n[INIT] === First Derivative (Gradient) ===")
    kernel = [point_clouds.sobel_y, point_clouds.sobel_z] 
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[1], plot=False)
    
    # print("\n[INIT] === Second Derivative (Laplacian) ===")
    # kernel = [point_clouds.laplacian_kernel] 
    # point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[2], plot=False)
    
    point_clouds.visualize_cost_map()

    # === 2 PATCHES INITIALIZATION ===
    pc_t = point_clouds.points_t
    patches = PatchSurface(pc_t,number_of_patches_width=number_of_patches_width, number_of_patches_height=number_of_patches_height)
    cost_grid = patches.get_cost_meshgrid()
    
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
    points_filename = "actual_point_terrain.json"
    points_save_path = os.path.join(result_dir, points_filename)
    
    with open(points_save_path, "w") as f:
        json.dump(terrain_points_data, f, indent=2)
    
    print(colored(f"[SAVE] Point terrain data saved to: {points_save_path}", "cyan"))
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

