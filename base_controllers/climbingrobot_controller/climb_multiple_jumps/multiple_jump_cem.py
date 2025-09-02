
import sys
import os
import time
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface
import numpy as np
from termcolor import colored
import matplotlib.pyplot as plt
from concurrent.futures import ThreadPoolExecutor
# Import CEM algorithm
from cem.algo import CemParams, CrossEntropyMethodMixed
import matlab.engine
#eng must not be created in a __main__ guard if you're using threads — otherwise it's not visible to other threads.
eng = matlab.engine.start_matlab()
eng.addpath('../codegen_mesh', nargout=0)
sys.path.insert(0, '../codegen_mesh')

# start and goal point
P0_INIT = np.array([0.0, 2.5, -5])
PF_INIT = np.array([0.0, 4, -3])

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

fitness_weights = np.array([1., 0.1, 10., 1.])



class BiLevelOptmizer:

    def __init__(self, in_point_clouds,p0,pf):

        self.p0 = p0
        self.pf = pf
        # === point cloud initialization
        self.in_point_clouds = in_point_clouds
        self.point_clouds = PointCloudFilter(self.in_point_clouds)
        #self.point_clouds.print_map_pc()

        self.point_clouds.filter_process_points([self.point_clouds.smoothing_kernel],weight=0.5, plot=False)
        pc_t = self.point_clouds.points_t
        self.point_clouds.visualize_cost_map()


        # === patch initializaiton
        self.patches = PatchSurface(pc_t)

        self.patches.cost_color()
        self.patches.plot_patches()
        #self.patches.random_color()
        #self.patches.plot_patches()
        # ===

    def evalPatchParams(self): #evalTerrainParams

        # select y and z
        p0_y = self.p0[1]
        p0_z = self.p0[2]
        pf_y = self.pf[1]
        pf_z = self.pf[2]
        # find patch from points
        patch_p0=self.patches.get_patch_id_from_point_2D(p0_y,p0_z)
        patch_pf=self.patches.get_patch_id_from_point_2D(pf_y, pf_z)
        #update p0 point on surface
        new_p0= self.patches.get_point_t_in_surface(patch_p0 , p0_y, p0_z, plot_patch=False)
        new_pf = self.patches.get_point_t_in_surface(patch_pf , pf_y, pf_z, plot_patch=False)
        #add points to patches
        self.patches.set_new_point_in_patch(patch_p0, p0_y, p0_z, update_cost=True, plot=False, k_neighbors=5)
        self.patches.set_new_point_in_patch(patch_pf, pf_y, pf_z, update_cost=True, plot=False, k_neighbors=5)
        #extract the mesh grid
        mesh_x, mesh_y, mesh_z = self.patches.get_mesh_grid_patch(patch_p0)
        #extract the normal vector
        normal_p0 = self.patches.normal_vector_of_point_in_patch(patch_p0, new_p0,plot_normal_patch = True)

        return mesh_x, mesh_y, mesh_z, normal_p0, new_p0, new_pf,

    def eval_pop(self,input_data):
        xd = input_data[0]
        xc = input_data[1]
        #first discrete variable is number of jumps, the next ones are the of the patches
        n_jumps = xd[0] + 1
        ids = []
        fitness = 0.0
        print(f"Number of jumps {n_jumps}\n")
        ## Run trajectory optimisation here #

        #p0_current = self.p0.copy()
        for i in range(n_jumps-1):
            print(f"Jump n:{i}\n")
            # following discrete variables represent the id of the patches for the intermediate jumps
            patch_id = xd[1 + i]
            # the continue variables contain the X and Y normalized coordinate of the candidate contact points inside the candidate patches
            contact_relative_to_patch_yz= xc[i*2:i*2+2]
            #print("jump number : ", i)

            contact_abs_pos_yz = self.patches.getAbsolutePoseOfPointInsidePatch(patch_id, contact_relative_to_patch_yz[0], contact_relative_to_patch_yz[1])

            # ?????????????? perche mi serve avere contact_abs_pos_yz?????
            mesh_x, mesh_y, mesh_z, normal, p0_adj, pf_adj = self.evalPatchParams(self.p0, contact_abs_pos_yz) #get the X consistent with terrain
            params['mesh_x'] = mesh_x
            params['mesh_y'] = mesh_y
            params['mesh_z'] = mesh_z
            params['contact_normal'] = matlab.double(normal)

            # che tipo di dato vuole ? optimize_cpp_mex ?
            res = eng.optimize_cpp_mex(matlab.double(p0_adj), matlab.double(pf_adj), Fleg_max, Fr_max, Fr_min, mu, params)
            # ??????????????? perche mi serve avere contact_abs_pos_yz?????
            fitness += self.calc_fitness(res, patch_id, contact_abs_pos_yz)

            self.p0 = pf_adj

        #print("final jump")
        #last jump is to pf
        mesh_x, mesh_y, mesh_z, normal, p0_adj, pf_adj = self.evalPatchParams()
        params['mesh_x'] = mesh_x
        params['mesh_y'] = mesh_y
        params['mesh_z'] = mesh_z
        params['contact_normal'] = matlab.double(normal)
        res = eng.optimize_cpp_mex(matlab.double(self.p0), matlab.double(self.pf), Fleg_max, Fr_max, Fr_min, mu, params)
        fitness += self.calc_fitness(res)

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


    def calc_fitness(self, patch_id=None, contact_abs_pos_yz=None):
        fit_average_costmap_patch = 0.
        fit_landing_costmap = 0.

        if patch_id is not None:
            points_in_patch = terrainManager.retrievePatches(patch_id)
            Y_range = points_in_patch[0, :]
            Z_range = points_in_patch[1, :]
            # eval avergage cost
            #fit_average_costmap_patch =  evalAverageCostOfPatch(patch_id)
            #eval actual cost at selected landing location
            #fit_landing_costmap = evalCostOfPointInPatch(contact_abs_pos_yz)

        #print("jump duration", res['Tf'])
        fit_consumed_energy = -res['consumed_energy']
        if (res['problem_solved']) == 1 or (res['problem_solved']==2): #convergence / semidefinite solution
            fit_problem_converged = 100
        else:
            fit_problem_converged = 0
        print(f"convergence: {fitness_weights[0]*fit_problem_converged}, energy: {fitness_weights[1]*fit_consumed_energy}, avg_cost: {fitness_weights[2]*fit_average_costmap_patch}, land_cost: {fitness_weights[3]*fit_landing_costmap}")
        fitness =  fitness_weights[0]*fit_problem_converged + fitness_weights[1]*fit_consumed_energy +fitness_weights[2]*fit_average_costmap_patch + fitness_weights[3]*fit_landing_costmap

        return fitness



def main():
    algo = CrossEntropyMethodMixed(p) # <-- da implementare dentro la classe appena il tutto funziona
    # create terrain:
    terrain = TerrainManager()
    # point cloud filter
    pc_terrain = terrain.point_cloud
    # Optimizer part
    optimizer = BiLevelOptmizer(pc_terrain,P0_INIT,PF_INIT)
    optimizer.evalPatchParams()

    cost_hist = np.zeros(p.cem_iters)

    start = time.time()
    for k in range(p.cem_iters):
        print(colored(f"Iteration {k}\n","blue"))
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
        fitness = []
        with ThreadPoolExecutor(max_workers=p.n_threads) as executor:

            for i, result in enumerate(executor.map(optimizer.eval_pop,inputs), start=1):
                fitness.append(result)
                print(colored(f"Population {i}/{len(inputs)} finished, fitness = {result}\n","red"))

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

if __name__ == "__main__":
    main()