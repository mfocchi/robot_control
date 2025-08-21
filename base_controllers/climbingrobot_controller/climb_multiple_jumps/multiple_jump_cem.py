
import sys
import os
import time
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface

import numpy as np
from concurrent.futures import ThreadPoolExecutor
# Import CEM algorithm
from cem.algo import CemParams, CrossEntropyMethodMixed
import matlab.engine
#eng must not be created in a __main__ guard if you're using threads — otherwise it's not visible to other threads.
eng = matlab.engine.start_matlab()
eng.addpath('../codegen_mesh', nargout=0)
sys.path.insert(0, '../codegen_mesh')

# start and goal point
P0_INIT = np.array([0.0, 2.5, -6])
PF_INIT = np.array([0.0, 4, -4])

# params for optimizer:
Fleg_max = 300.
Fr_max = 90.
Fr_min = 0.
mu = 0.8

mass = 5.
params = {}
params['m'] = mass
anchor_distance = 5.
params['num_params'] = 4.
params['int_method'] = 'rk4'
params['N_dyn'] = 30.
params['FRICTION_CONE'] = 1.
params['int_steps'] = 5.
params['b'] = anchor_distance
params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
params['p_a2'] = matlab.double([0., params['b'], 0.]).reshape(3, 1)
params['g'] = 9.81
params['w1'] = 1.  # smooth
params['w2'] = 1.  # hoist work
params['w3'] = 0.
params['w4'] = 0.
params['w5'] = 0.
params['w6'] = 0.
params['T_th'] = 0.05
params['obstacle_avoidance'] = 'mesh'
params['jump_clearance'] = 1.
# Set up parameters OUTER LOOP
p = CemParams()
p.seed = int(time.time())
p.n_threads = 1
# General CEM-MD Parameters
p.cem_iters = 15
p.pop_size = 100
p.n_elites = int(p.pop_size * 0.8)
p.decrease_pop_factor = 1.0
p.fraction_elites_reused = 0.0
# Discrete
p.dim_discrete = 5
number_of_patches = 20
p.n_values = [3] + [(number_of_patches-1) for _ in range(4)]
p.init_probs = [[1.0 / p.n_values[i] for _ in range(p.n_values[i])] for i in range(p.dim_discrete)]
p.min_prob = 0.05
# Continuous
MAX_N_PATCHES = 5
p.dim_continuous = 2 * MAX_N_PATCHES
p.max_value_continuous = np.full(p.dim_continuous, 1.0)
p.min_value_continuous = np.full(p.dim_continuous, 0.0)
p.init_mu_continuous = np.full(p.dim_continuous, 0.5)
p.init_std_continuous = np.full(p.dim_continuous, 1.0)
p.min_std_continuous = np.full(p.dim_continuous, 1e-3)

algo = CrossEntropyMethodMixed(p)

def calc_fitness():
    pass

def eval_pop():
    pass

def inner_loop():
    pass

def outer_loop():
    pass

def save_date_json():
    pass



def main():


    # create terrain:
    terrain = TerrainManager()

    # point cloud filter
    pc_terrain = terrain.point_cloud
    pc = PointCloudFilter(pc_terrain)
    pc.print_map_pc()
    pc.filter_process_points([pc.smoothing_kernel],plot=True)

    # generation patch
    patches = PatchSurface(pc.points_t)
    patches.create_patches()
    patches.cost_color()
    patches.plot_patches()

    # test best patches
    random_indices = np.random.choice(len(patches.patches), size=5, replace=False)
    patch_list = [patches.patches[i] for i in random_indices]
    print(f"Selected 3 random patches from {len(patches.patches)} total patches")
    patches.color_targhet_patches(patch_list)
    patches.plot_patches_target()

    # test best point for planning
    random_indices = np.random.choice(len(pc.points_t), size=5, replace=False)
    point_list = [pc.points_t[i] for i in random_indices]
    print(f"Selected 3 random points from {len(pc.points_t)} total points")
    patches.color_targhet_points_jump(point_list)
    patches.plot_patches_points_target()

if __name__ == "__main__":
    main()