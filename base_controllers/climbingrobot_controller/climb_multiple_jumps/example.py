import os
import time
from datetime import datetime
import json
import numpy as np
import sys
#imports for inner loop
import matlab.engine
import numpy as np
import math
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
import matplotlib.pyplot as plt




# Run evaluations in parallel
#from concurrent.futures import ProcessPoolExecutor not working with matlab
from concurrent.futures import ThreadPoolExecutor
# Import CEM algorithm
from cem.algo import CemParams, CrossEntropyMethodMixed

#eng must not be created in a __main__ guard if you're using threads — otherwise it's not visible to other threads.
eng = matlab.engine.start_matlab()
eng.addpath('../codegen_mesh', nargout=0)
sys.path.insert(0, '../codegen_mesh')



# Setup inner loop
# jump params
P0_INIT = np.array([0.0, 2.5, -6])
PF_INIT = np.array([0.0, 4, -4])

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

# terrain
# Parameters (direct translation from MATLAB)
wall_depth = 1  # how
grid_size = 100
max_ridge_depth = 0.5
seed = "default"
Lz = -20  # Height of wall in meters
Ly = 5  # Width (horizontal extent) of wall in meters
# Generate rock wall map
terrainManager = TerrainManager()
mesh_x, mesh_y, mesh_z = terrainManager.generate_rock_wall_map(Lz, Ly, grid_size, wall_depth, max_ridge_depth,   seed, debug=False)




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


def evalTerrainParams(p0, pf):
    # Interpolator (note: z must be increasing — here from -10 to 0)
    p0[0] = terrainManager.wall_surface_eval(p0[2], p0[1], mesh_x, mesh_y, mesh_z)
    pf[0] = terrainManager.wall_surface_eval(pf[2], pf[1], mesh_x, mesh_y, mesh_z)
    # compute consistent normal
    normal = terrainManager.wall_normal_eval(p0[2], p0[1], mesh_x, mesh_y, mesh_z)
    return mesh_x, mesh_y, mesh_z,normal,  p0, pf

def calc_fitness(res, patch_id=None):
    fit_average_costmap_patch = 0.
    fit_landing_costmap = 0.

    if patch_id is not None:
        points_in_patch = terrainManager.retrievePatches(patch_id)
        Y_range = points_in_patch[0, :]
        Z_range = points_in_patch[1, :]
        # eval avergage cost
        #fit_average_costmap_patch = sum(p.evalCostMap(y, z) for y in Y_range for z in Z_range)/(terrainManager.number_of_points_in_patch)
        #eval actual cost at selected landing location
        #fit_landing_costmap = p.evalCostMap(pf[1], pf[2])

    #print("jump duration", res['Tf'])
    fit_consumed_energy = -res['consumed_energy']
    if (res['problem_solved']) == 1 or (res['problem_solved']==2): #convergence / semidefinite solution
        fit_problem_converged = 100
    else:
        fit_problem_converged = 0
    print(f"convergence: {fit_problem_converged}, energy: {fit_consumed_energy}, avg_cost: {fit_average_costmap_patch}, land_cost: {fit_landing_costmap}")
    fitness =  fit_problem_converged + fit_consumed_energy +fit_landing_costmap +fit_average_costmap_patch

    return fitness


# Function to evaluate the CEM-MD population
def eval_pop(input):
    xd = input[0]
    xc = input[1]
    #first discrete variable is number of jumps
    n_jumps = xd[0] + 1
    ids = []
    fitness = 0.0
    print(f"Number of jumps {n_jumps}\n")
    ## Run trajectory optimisation here #
    p0 = P0_INIT
    for i in range(n_jumps-1):
        # following discrete variables represent the id of the patches for the intermediate jumps
        patch_id = xd[1 + i]
        # the continue variables contain the X and Y normalized coordinate of the candidate contact points inside the candidate patches
        contact_relative_to_patch = xc[i*2:i*2+2]
        #print("jump number : ", i)
        contact_abs_pos = terrainManager.getPositionInsidePatch(patch_id,   contact_relative_to_patch[0], contact_relative_to_patch[1])
        mesh_x, mesh_y, mesh_z, normal, p0_adj, pf_adj = evalTerrainParams(p0, contact_abs_pos) #get the X consistent with terrain
        params['mesh_x'] = mesh_x
        params['mesh_y'] = mesh_y
        params['mesh_z'] = mesh_z
        params['contact_normal'] = matlab.double(normal)
        res = eng.optimize_cpp_mex(matlab.double(p0_adj), matlab.double(pf_adj), Fleg_max, Fr_max, Fr_min, mu, params)
        fitness += calc_fitness(res, patch_id)
        p0 = pf_adj

    #print("final jump")
    #last jump is to pf
    mesh_x, mesh_y, mesh_z, normal, p0_adj, pf_adj = evalTerrainParams(p0, PF_INIT)
    params['mesh_x'] = mesh_x
    params['mesh_y'] = mesh_y
    params['mesh_z'] = mesh_z
    params['contact_normal'] = matlab.double(normal)
    res = eng.optimize_cpp_mex(matlab.double(p0), matlab.double(PF_INIT), Fleg_max, Fr_max, Fr_min, mu, params)
    fitness += calc_fitness(res)

    #print("population evaluation finished")
    #to debug
    ## Calculate fitness (by default the algorithm maximises) ##
    # if xd[0] == 0:
    #     fitness += 50
    # if xd[1] == 1:
    #     fitness += 100
    # for num in xc:
    #     fitness += -((num - 1.5) ** 2)
    return fitness

if __name__ == '__main__':
    # Main loop

    # Surface plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(mesh_x, mesh_y, mesh_z, alpha=0.7, cmap='Blues', edgecolor='k', linewidth=0.2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim([0, 4])
    ax.set_ylim([0, 7])
    ax.set_zlim([-10, 2])
    # Alternative method using set_box_aspect for proportional scaling
    # ax.set_box_aspect([x_range, y_range, z_range])
    ax.view_init(elev=20, azim=9)
    plt.show()

    cost_hist = np.zeros(p.cem_iters)

    start = time.time()
    for k in range(p.cem_iters):
        # Generate population by sampling the distributions
        algo.generate_population_discrete()
        algo.generate_population_continuous()
        xd = algo.population_discrete  # shape: dim_discrete x pop_size
        xc = algo.population_continuous  # shape: dim_continuous x pop_size

        # Organise inputs to pass to process pool
        inputs = [[xd[:, i].tolist(), xc[:, i].tolist(), p] for i in range(p.pop_size)]

        # Evaluate population in parallel
        # with ProcessPoolExecutor(max_workers=p.n_threads) as executor:
        #     fitness = list(executor.map(eval_pop, inputs))

        with ThreadPoolExecutor(max_workers=p.n_threads) as executor:
            fitness = list(executor.map(eval_pop, inputs))

        # Evaluate population and update distributions
        algo.evaluate_population(fitness)
        algo.update_distributions()
        cost_hist[k] = algo.log.best_value

        ## Early exit

        # Print intermediate stats
        # print(algo.log.iterations, "(", algo.log.func_evals, "): ", algo.log.best_value)
        # print("discrete probabilities: \n", algo.probs)
        # print("mean: \n", algo.mu.T)
        # print("sigma: \n", algo.std_devs.T)
        # print(" ")

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
    filename = f"cem_solution.json"
    save_path = os.path.join(os.path.abspath(os.getcwd()), filename)

    with open(save_path, "w") as f:
        json.dump(report, f, indent=2)
