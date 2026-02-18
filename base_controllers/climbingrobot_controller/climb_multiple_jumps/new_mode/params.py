import os
import sys
from matplotlib.collections import PatchCollection
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
import matplotlib.pyplot as plt
#import seaborn as sns
import numpy as np
import matplotlib.patches as mpatches
        
# ================================================
# BASE DATA FOR MULTIPLE JUMPS CLIMBING CONTROLLER
# ================================================
# 1. P0_INIT
p0_str = os.environ.get("P0_INIT_STR") 
if p0_str:
    P0_INIT = np.array(json.loads(p0_str))
else:
    P0_INIT = np.array([0.5, 5.5, -8.5])

pf_str = os.environ.get("PF_INIT_STR")
if pf_str:
    PF_PATCH_INIT = np.array(json.loads(pf_str))
else:
    PF_PATCH_INIT = np.array([0.5, 2.5, -1.5])

PF_INIT = PF_PATCH_INIT
MAX_JUMP = 6
THREADS = 15
flag_thread = True
patience = 7
CORRIDOR_RADIUS = 6.0 # for linear corridor warm start
# MAIN_DIRECTORY = "result/2_test"

MAIN_DIRECTORY = os.environ.get("EXPERIMENT_DIR", "result/common")
# [ fit_problem_converged | fit_consumed_energy | fit_average_costmap_patch | fit_landing_costmap | fit_linear_distance | way_point_cost ]
fitness_weights = np.array([1e7, 1., 1., 10., 0.0,  0.0]) # Optimizer
# fitness_weights = np.array([1e4, 30.0,10., 0.5, 10.0,0.0]) # Linear or parabolic
# weights for point cloud filtering
# filter_weights = np.array([100., 1000., 0,10.0]) #smoothing, first derivative, II dev v1, II dev v2, gaussian cost
filter_weights = np.array([0., 10., 10.0, 0.0, 0.0])

# ================================================
# INNER LOOP OPTIMIZER PARAMETERS
# ================================================
# Create inner_opt_params in the EXACT order MATLAB expects
Fleg_max = 200.
Fr_max = 150.
Fr_min = 15.
number_of_patches_width = 10
number_of_patches_height = 10
Ly = 10.
Lz = -10.
mass = 5.08
anchor_distance = Ly
mu = 0.8

inner_opt_params = {}
inner_opt_params['m'] = mass
inner_opt_params['num_params'] = 4.
inner_opt_params['int_method'] = 'rk4'
inner_opt_params['N_dyn'] = 30.
inner_opt_params['FRICTION_CONE'] = 1.
inner_opt_params['int_steps'] = 5.
inner_opt_params['b'] = Ly #anchor_distance
inner_opt_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
inner_opt_params['p_a2'] = matlab.double([0., inner_opt_params['b'], 0.]).reshape(3, 1)
inner_opt_params['g'] = 9.81
inner_opt_params['w1'] = 0.001  # smooth
inner_opt_params['w2'] = 0.  # hoist work
inner_opt_params['w3'] = 1000.
inner_opt_params['T_th'] = 0.05
inner_opt_params['obstacle_avoidance'] = 'mesh'
inner_opt_params['jump_clearance'] = 0.5
inner_opt_params['mesh_x'] = None
inner_opt_params['mesh_y'] = None
inner_opt_params['mesh_z'] = None
inner_opt_params['cost_x'] = None
inner_opt_params['cost_y'] = None
inner_opt_params['cost_z'] = None
inner_opt_params['contact_normal'] = None
inner_opt_params['debug'] = False
# ================================================
# OUTER LOOP OPTIMIZER PARAMETERS
# ================================================
CEM_DISCRETE_DIM = MAX_JUMP + 1

# Set up parameters OUTER LOOP
cem_params = CemParams()
cem_params.seed =int(time.time())
cem_params.n_threads = THREADS
# General CEM-MD Parameters
cem_params.cem_iters = 50
cem_params.pop_size = 500
cem_params.n_elites = int(cem_params.pop_size * 0.5)
cem_params.decrease_pop_factor = 0.0 
cem_params.fraction_elites_reused = 0.0 
cem_params.alpha = 0.5
# Discrete
cem_params.dim_discrete = CEM_DISCRETE_DIM
number_of_patches = number_of_patches_width * number_of_patches_height
# cem_params.n_values = [3] + [(number_of_patches-1) for _ in range(4)]
cem_params.n_values = [MAX_JUMP] + [(number_of_patches) for _ in range(MAX_JUMP)]
cem_params.init_probs = [[1.0 / cem_params.n_values[i] for _ in range(cem_params.n_values[i])] for i in range(cem_params.dim_discrete)]
# cem_params.min_prob = 0.01  
# Continuous
cem_params.dim_continuous = 2 * CEM_DISCRETE_DIM # x and y positions
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
ITERATIONS_FOLDER = f"{MAIN_DIRECTORY}/iteration_reports"
FILE_SAVE_PARAMS = f"{MAIN_DIRECTORY}/simulation_params.json"
FILE_DESCRIPTION = f"{MAIN_DIRECTORY}/description.txt"
FILE_FOR_GAZEBO_SIM = f"{MAIN_DIRECTORY}/info_for_gazebo.json"
# ================================================
# COMMON FUNCTIONS
# ================================================
result_dir = os.path.join(os.path.abspath(os.getcwd()), MAIN_DIRECTORY)
os.makedirs(result_dir, exist_ok=True)

terrain_type = os.environ.get("TERRAIN_TYPE", "hemisphere") #custom_gaussians | hemisphere | rock | gaussian_bumps
# Terrain configuration values for rock terrain, otherwise stay in default
wall_depth = 4           
grid_size = 100
max_ridge_depth = 0.5   
# terrain_manager = TerrainManager(wall_depth=wall_depth, grid_size=grid_size, max_ridge_depth=max_ridge_depth, Lz=Lz, Ly=Ly, terrain_type='rock')
# terrain_manager  = TerrainManager(grid_size=100,wall_depth =10,max_ridge_depth=0.5, seed="default", Lz=-10, Ly=10, generate_terrain=True, terrain_type='custom_gaussians')
terrain_manager  = TerrainManager(grid_size=grid_size,wall_depth =wall_depth,max_ridge_depth=max_ridge_depth, seed="default", Lz=Lz, Ly=Ly, generate_terrain=True, terrain_type=terrain_type)

def initialize_terrain_data(warm_start_mode=False):
    create_description_file(enable=False)
    
    terrain_params = []
    # === 1 POINT CLOUD INITIALIZATION ===
    in_point_clouds = terrain_manager.point_cloud
    point_clouds = PointCloudFilter(in_point_clouds)
    anchor_location = np.array(inner_opt_params['p_a1'])
    
    # Filter with cost change and color based on height
    # point_clouds.filter_height_profile(x0=0.0, scale=1.0,side_application="depth", profile="logln")
    # point_clouds.visualize_cost_map()
    print("\n[INIT] === Smoothing Filter ===")
    kernel = [point_clouds.smoothing_kernel] 
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[0], plot=False)
    print("\n[INIT] === First Derivative (Gradient) ===")
    kernel = [point_clouds.sobel_y, point_clouds.sobel_z] 
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[1], plot=False)
    print("\n[INIT] === Second Derivative (Laplacian) ===")
    kernel = [point_clouds.laplacian_kernel] 
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[2], plot=False)
    # point_clouds.visualize_cost_map()
    # print("\n[INIT] === Laplacian of Gaussian (LoG) ===")
    # # Usa log_kernel invece di laplacian_kernel
    # kernel = [point_clouds.log_kernel] 
    # point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[3], plot=False)
    print("\n[INIT] === Bump Detection (II derviative v2) ===")
    point_clouds.compute_bump_detection(weight=filter_weights[3])
    
    # === 2 PATCHES INITIALIZATION ===
    pc_t = point_clouds.points_t
    patches = PatchSurface(pc_t,number_of_patches_width=number_of_patches_width, number_of_patches_height=number_of_patches_height)
    patches.gaussian_cost_all_patch(weight_gauss_cost=filter_weights[4])
    # patches.visualize_full_cost_map()
    # patches.print_patch_cost_matrix(2)
    
    cost_grid, cost_y, cost_z  = patches.get_cost_meshgrid(grid_size=grid_size)
    
    
    # patches.plot_cost_meshgrid(cost_grid, cost_y, cost_z, plot_type='surface')
    # patches.plot_cost_meshgrid(cost_grid, cost_y, cost_z, plot_type='contour')
    # patches.plot_cost_meshgrid(cost_grid, cost_y, cost_z, plot_type='both')
    # patches.plot_map_with_cost_meshgrid_overlay( Cost_grid=cost_grid, Y_grid=cost_y, Z_grid=cost_z, x_offset=1.0, alpha_mesh=0.7, alpha_points=0.7 )   
    
    # Update inner_opt_params with terrain data
    inner_opt_params['mesh_x'] = terrain_manager.mesh_x
    inner_opt_params['mesh_y'] = terrain_manager.mesh_y
    inner_opt_params['mesh_z'] = terrain_manager.mesh_z
    inner_opt_params['cost_x'] = cost_grid
    inner_opt_params['cost_y'] = cost_y
    inner_opt_params['cost_z'] = cost_z
    
    # inner_opt_params['patch_side'] = 1.0 * patches.patch_width
    inner_opt_params['patch_side_z'] = patches.patch_height
    inner_opt_params['patch_side_y'] = patches.patch_width
    
    # patch_id = 25
    # cost = patches.get_patch_cost(patch_id)
    # if cost is not None:
    #     print(f"Cost of patch {patch_id} is: {cost:.4f}")
    #     patches.plot_patch(patch_id)
    
    terrain_params.append({
        'anchor_location': point_clouds,
        'patches': patches,
        'cost_grid': cost_grid
    })
        
    patch_pf = patches.get_patch_id_from_point_2D(PF_PATCH_INIT[1], PF_PATCH_INIT[2])
    patch_p0 = patches.get_patch_id_from_point_2D(P0_INIT[1], P0_INIT[2])
    
    # Update n_values to exclude p0 and pf patches
    number_of_patches = number_of_patches_width * number_of_patches_height
    all_patches = list(range(number_of_patches))
    
    # Remove p0 and pf from available patches
    valid_patches = [p for p in all_patches if p != patch_p0 and p != patch_pf]
    # Update cem_params.n_values with the new structure
    cem_params.n_values = [MAX_JUMP] + [valid_patches.copy() for _ in range(MAX_JUMP)]
    
    var_min = float(os.environ.get("CEM_MIN_PROB", 1))
    cem_params.min_prob = 1/len(patches.patches) * var_min
    
    # Update init_probs accordingly
    if warm_start_mode:
        
        patch_probs = get_warm_start_line_distance_only( patches, P0_INIT, PF_PATCH_INIT, radius=CORRIDOR_RADIUS )
        
        # patch_probs = get_warm_start_base_cost(patches)
        # patch_probs = get_warm_start_line(
        #     patches, 
        #     P0_INIT, 
        #     PF_PATCH_INIT, 
        #     radius=CORRIDOR_RADIUS,     
        #     sensitivity=5.0 )
    
        # plot_probability_heatmap(patch_probs, patches, P0_INIT, PF_PATCH_INIT)

        # Create new init_probs with filtered probabilities
        new_init_probs = []
        # First dimension remains the same (number of jumps)
        new_init_probs.append([1.0 / MAX_JUMP for _ in range(MAX_JUMP)])
        
        # For patch selections, filter out p0 and pf probabilities
        for i in range(1, len(cem_params.n_values)):
            filtered_probs = [patch_probs[idx] for idx in valid_patches]
            # Renormalize
            sum_probs = sum(filtered_probs)
            if sum_probs > 0:
                filtered_probs = [p / sum_probs for p in filtered_probs]
            else:
                filtered_probs = [1.0 / len(valid_patches) for _ in valid_patches]
            new_init_probs.append(filtered_probs)
        
        cem_params.init_probs = new_init_probs
    else:
        # Uniform distribution over valid patches
        cem_params.init_probs = [[1.0 / MAX_JUMP for _ in range(MAX_JUMP)]] + \
                                [[1.0 / len(valid_patches) for _ in valid_patches] for _ in range(MAX_JUMP)]
    # save the terrain data 
    save_terrain_data(terrain_manager,point_clouds, patches)
    
    return point_clouds, patches, cost_grid

# ================================================
# SAVE PARAMS AND TERRAIN DATA
# ================================================
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
        'MAX_JUMP': MAX_JUMP,
        'THREADS': THREADS,
        'inner_opt_params_order':{
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
            'alpha': cem_params.alpha,
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

def save_gazebo_info(best_points, terrain_manager):
    gazebo_data = {
        "terrain_info": {
            "grid_size": terrain_manager.grid_size,
            "wall_depth": terrain_manager.wall_depth,
            "max_ridge_depth": terrain_manager.max_ridge_depth,
            "seed": terrain_manager.seed,
            "Lz": terrain_manager.Lz,
            "Ly": terrain_manager.Ly,
            "generate_terrain": terrain_manager.generate_terrain,
            "terrain_type": terrain_manager.terrain_type
        },
        "target_points": [point.tolist() if isinstance(point, np.ndarray) else point 
                         for point in best_points] if best_points is not None else []
    }
    
    os.makedirs(os.path.dirname(FILE_FOR_GAZEBO_SIM), exist_ok=True)
    
    with open(FILE_FOR_GAZEBO_SIM, "w") as f:
        json.dump(gazebo_data, f, indent=2)
    
    print(colored(f"[SAVE] Gazebo simulation info saved to: {FILE_FOR_GAZEBO_SIM}", "cyan"))
    print(colored(f"       - Terrain type: {terrain_manager.terrain_type}", "cyan"))
    print(colored(f"       - Target points: {len(best_points) if best_points else 0}", "cyan"))

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
        'debug': inner_opt_params['debug'],
        'mesh_x': inner_opt_params['mesh_x'],
        'mesh_y': inner_opt_params['mesh_y'],
        'mesh_z': inner_opt_params['mesh_z'],
        'cost_x': inner_opt_params['cost_x'],
        'cost_y': inner_opt_params['cost_y'],
        'cost_z': inner_opt_params['cost_z'],
        'patch_side_z': inner_opt_params['patch_side_z'],
        'patch_side_y': inner_opt_params['patch_side_y'],
        'contact_normal': None,
        
    }

# ================================================
# WARM START TERRAIN DATA
# ================================================
# sensitivity: higher --> focus on more valid points
def get_warm_start_line(patches, p0, pf, radius=4.0, sensitivity=5.0):
    num_patches = len(patches.patches)
    
    # 1. Cost Weighting: exp(-k * cost)
    costs = np.array([p.get('cost_patch', float('inf')) for p in patches.patches])
    costs[costs == None] = float('inf') 
    
    valid_mask = np.isfinite(costs)
    if np.any(valid_mask):
        max_cost = np.max(costs[valid_mask])
        min_cost = np.min(costs[valid_mask])
        # Replace infinites with 2x Max
        costs[~valid_mask] = max_cost * 2.0
        
        range_cost = max_cost - min_cost
        norm_costs = (costs - min_cost) / range_cost if range_cost > 1e-6 else np.zeros_like(costs)
    else:
        norm_costs = np.zeros_like(costs)

    w_cost = np.exp(-sensitivity * norm_costs)
    
    # Vectors to calculate point-line distance in 3D
    # Line defined by P0 + t * (PF - P0)
    line_vec = pf - p0
    line_len_sq = np.dot(line_vec, line_vec)
    
    # Vectorized distance calculation
    centroids = np.array([p['centroid'] for p in patches.patches])
    point_vecs = centroids - p0
    cross_prods = np.cross(point_vecs, line_vec)
    distances = np.linalg.norm(cross_prods, axis=1) / np.sqrt(line_len_sq)
    
    # Inverse linear weight inside radius (Distance 0 -> Weight 1, Distance Radius -> Weight 0)
    w_line = np.maximum(0.0, 1.0 - (distances / radius))
            
    # 3. Combination and Normalization
    # Multiply the two weights: a patch must be GOOD AND CLOSE to the line
    raw_probs = w_cost * w_line
    
    # Cleanup too low values
    raw_probs[raw_probs < 1e-9] = 0.0
    
    sum_probs = np.sum(raw_probs)
    
    if sum_probs == 0:
        print(colored("[WARN] Warm start generated zero probability space. Reverting to uniform.", "yellow"))
        return np.full(num_patches, 1.0 / num_patches).tolist()
    
    return (raw_probs / sum_probs).tolist()

def get_warm_start_line_distance_only(patches, p0, pf, radius=8.0):
    """
    Compute patch probabilities based on 2D perpendicular distance from the line between p0 and pf.
    Only considers y and z coordinates (ignores x).
    """
    num_patches = len(patches.patches)
    
    # Extract only y and z coordinates (ignore x)
    p0_2d = p0[1:]  # [y, z]
    pf_2d = pf[1:]  # [y, z]
    
    # Define line vector in 2D
    line_vec_2d = pf_2d - p0_2d
    line_len_sq = np.dot(line_vec_2d, line_vec_2d)
    
    # Handle degenerate case where p0 == pf in 2D
    if line_len_sq < 1e-9:
        print(colored("[WARN] Start and goal are the same point in 2D. Using uniform distribution.", "yellow"))
        return np.full(num_patches, 1.0 / num_patches).tolist()
    
    # Extract centroids for all patches (vectorized) - only y and z
    centroids = np.array([p['centroid'] for p in patches.patches])
    centroids_2d = centroids[:, 1:]  # Take only y and z columns
    
    # Calculate perpendicular distance from each centroid to the line in 2D
    # For 2D: distance = |cross_product_z_component| / ||line_vec||
    # cross_product_z = (point_y - p0_y) * line_z - (point_z - p0_z) * line_y
    point_vecs_2d = centroids_2d - p0_2d
    
    # 2D cross product (scalar result)
    cross_prod_z = point_vecs_2d[:, 0] * line_vec_2d[1] - point_vecs_2d[:, 1] * line_vec_2d[0]
    distances = np.abs(cross_prod_z) / np.sqrt(line_len_sq)
    
    # Inverse linear weight: distance 0 -> weight 1, distance >= radius -> weight 0
    weights = np.maximum(0.0, 1.0 - (distances**2 / radius))
    
    # Cleanup very small values
    weights[weights < 1e-9] = 0.0
    
    # Normalize to probabilities
    sum_weights = np.sum(weights)
    
    if sum_weights == 0:
        print(colored("[WARN] All patches outside corridor radius. Reverting to uniform distribution.", "yellow"))
        return np.full(num_patches, 1.0 / num_patches).tolist()
    
    return (weights / sum_weights).tolist()

def get_warm_start_base_cost(patches, sensitivity=10.0): 
    # 1. Cost extraction maintaining correspondence with centroids
    num_patches = len(patches.patches)
    
    # Vectorized cost extraction
    costs = np.array([p.get('cost_patch', float('inf')) for p in patches.patches])
    costs[costs == None] = float('inf')
    
    valid_mask = np.isfinite(costs)
    
    # If no valid costs, return uniform
    if not np.any(valid_mask):
        return np.full(num_patches, 1.0 / num_patches)

    max_cost = np.max(costs[valid_mask])
    min_cost = np.min(costs[valid_mask])
    costs[~valid_mask] = max_cost * 2.0
    
    range_cost = max_cost - min_cost
    norm_costs = (costs - min_cost) / range_cost if range_cost > 1e-6 else np.zeros_like(costs)

    # Inverted Softmax
    weights = np.exp(-sensitivity * norm_costs)
    
    cutoff_value = np.max(weights) * 0.01
    weights[weights < cutoff_value] = 0.0
    # ------------------------------------------

    # Final Normalization
    sum_weights = np.sum(weights)
    if sum_weights == 0:
        return np.full(num_patches, 1.0 / num_patches)
        
    return weights / sum_weights

# ================================================
# DESCRIBE RESULT FOLDER
# ================================================

def create_description_file(enable=bool):
    if not enable:
        return
    print(colored("\n[DESCRIPTION] Insert a description for this experiment.", "cyan"))
    print(colored("Finish with ENTER. Leave empty to skip.", "cyan"))
    try:
        description = input("> ").strip()
    except KeyboardInterrupt:
        print(colored("\n[DESCRIPTION] Input cancelled by user.", "yellow"))
        return

    if description == "":
        print(colored("[DESCRIPTION] Empty description. File not created.", "yellow"))
        return

    os.makedirs(os.path.dirname(FILE_DESCRIPTION), exist_ok=True)

    with open(FILE_DESCRIPTION, "w") as f:
        f.write("=== EXPERIMENT DESCRIPTION ===\n")
        f.write(description + "\n")

    print(colored(f"[SAVE] Description saved to: {FILE_DESCRIPTION}", "cyan"))

# ================================================
# PLOTTING FUNCTIONS
# ================================================

def plot_probability_heatmap(patch_probs, patches_obj, p0, pf):
    print(colored("\n[PLOT] Generating Probability Heatmap (Sorted by Grid)...", "cyan"))
    
    # Geometric patch data
    width = patches_obj.patch_width
    height = patches_obj.patch_height
    patches = patches_obj.patches
    if len(patch_probs) != len(patches):
        print(colored(f"[ERROR] Mismatch tra numero probabilità ({len(patch_probs)}) e patch ({len(patches)})", "red"))
        return patch_probs

    # =========================================================
    # 2. PREPARAZIONE PLOT
    # =========================================================
    fig, ax = plt.subplots(figsize=(12, 10))
    
    rectangles = []
    ids_and_probs = []

    for i, patch in enumerate(patches):
        centroid = patch['centroid']
        # Calcoliamo l'angolo in basso a sinistra della patch per mpatches.Rectangle
        # Il centroide è (x, y, z), a noi servono y e z per il piano 2D
        y_corner = centroid[1] - (width / 2.0)
        z_corner = centroid[2] - (height / 2.0)
        
        rect = mpatches.Rectangle((y_corner, z_corner), width, height)
        rectangles.append(rect)
        
        # Salviamo i dati per le annotazioni testuali
        ids_and_probs.append({
            'y': centroid[1],
            'z': centroid[2],
            'id': patch['id'],
            'p': patch_probs[i]
        })

    # Creazione della collezione di rettangoli (Heatmap)
    pc = PatchCollection(rectangles, cmap='RdYlGn', alpha=0.9, edgecolor='grey', linewidth=0.5)
    pc.set_array(patch_probs) # Applica i colori in base alle probabilità
    ax.add_collection(pc)
    
    # Barra laterale (Colorbar)
    cbar = fig.colorbar(pc, ax=ax)
    cbar.set_label('Warm Start Probability', rotation=270, labelpad=15)

    # 3. Annotazioni Testuali (ID e %)
    for item in ids_and_probs:
        # Mostra l'ID e la probabilità percentuale al centro della patch
        ax.text(item['y'], item['z'], f"ID: {item['id']}\n{item['p']:.1%}", 
                ha='center', va='center', fontsize=7, color='black', fontweight='bold')

    # =========================================================
    # 4. START & GOAL (Coordinate Y, Z)
    # =========================================================
    ax.scatter(p0[1], p0[2], c='blue', s=300, marker='o', edgecolors='white', zorder=10, label='Start')
    ax.text(p0[1], p0[2] + (height * 0.6), "START", ha='center', color='blue', fontweight='bold', zorder=10)
    
    ax.scatter(pf[1], pf[2], c='gold', s=400, marker='*', edgecolors='black', zorder=10, label='Goal')
    ax.text(pf[1], pf[2] - (height * 0.6), "GOAL", ha='center', color='orange', fontweight='bold', zorder=10)

    # Configurazione Assi
    ax.set_xlabel("Terrain Y (Width)")
    ax.set_ylabel("Terrain Z (Height)")
    ax.set_title("Patch Probability Map (Top-Left Origin Row-by-Row)")
    ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.3)
    
    # Impostiamo i limiti degli assi in base alla mappa
    ax.set_xlim(patches_obj.y_min - width, patches_obj.y_max + width)
    ax.set_ylim(patches_obj.z_min - height, patches_obj.z_max + height)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.show()
