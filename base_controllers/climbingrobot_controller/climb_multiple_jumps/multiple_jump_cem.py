
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
from base_controllers.utils.matlab_conversions import mat_matrix2python,mat_vector2python

import matlab.engine
#eng must not be created in a __main__ guard if you're using threads — otherwise it's not visible to other threads.
eng = matlab.engine.start_matlab()
eng.addpath('../codegen_mesh', nargout=0)
sys.path.insert(0, '../codegen_mesh')

# start and goal point
P0_INIT = np.array([0.0, 2.5, -5])
#PF_INIT = np.array([0.0, 4, -3])
PF_INIT = np.array([0.0, 2.5, -15])


# inner_opt_params for optimizer:
Fleg_max = 300.
Fr_max = 90.
Fr_min = 0.
mu = 0.8

mass = 5.
inner_opt_params = {}
inner_opt_params['m'] = mass
anchor_distance = 5.
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
inner_opt_params['w4'] = 0.
inner_opt_params['w5'] = 0.
inner_opt_params['w6'] = 0.
inner_opt_params['T_th'] = 0.05
inner_opt_params['obstacle_avoidance'] = 'mesh'
inner_opt_params['jump_clearance'] = 1.
# Set up parameters OUTER LOOP
cem_params = CemParams()
cem_params.seed = int(time.time())
cem_params.n_threads = 1
# General CEM-MD Parameters
cem_params.cem_iters = 15
cem_params.pop_size = 100
cem_params.n_elites = int(cem_params.pop_size * 0.8)
cem_params.decrease_pop_factor = 1.0
cem_params.fraction_elites_reused = 0.0
# Discrete
cem_params.dim_discrete = 5
number_of_patches = 20
cem_params.n_values = [3] + [(number_of_patches-1) for _ in range(4)]
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

class BiLevelOptmizer:

    def __init__(self, terrain_manager,p0,pf):

        # point cloud filter
        self.terrain_manager = terrain_manager
        self.p0_init = p0
        self.pf_init = pf


        # === point cloud initialization
        self.in_point_clouds = self.terrain_manager.point_cloud
        self.point_clouds = PointCloudFilter(self.in_point_clouds)
        #self.point_clouds.print_map_pc()

        # [ smooth | I_derivative | II_derivative | ... ]
        self.filter_weights = np.array([1., 1., 1., 1.])
        # apply filters on point cloud
        # smoothing
        self.point_clouds.filter_process_points([self.point_clouds.smoothing_kernel],weight=self.filter_weights[0], plot=False)
        # avoid X under the anchor
        anchor_location = np.array(inner_opt_params['p_a1'])
        self.point_clouds.filter_height_profile(profile="logln", x0 = anchor_location[0], weight=self.filter_weights[3], side_application="depth")
        # first derivative
        kernel = [self.point_clouds.sobel_y, self.point_clouds.sobel_z]
        self.point_clouds.filter_process_points(kernel, weight=self.filter_weights[1], plot=False)

        pc_t = self.point_clouds.points_t
        #self.point_clouds.visualize_cost_map()
        # === patch initializaiton
        self.patches = PatchSurface(pc_t)
        self.patches.cost_color()
        #self.patches.plot_patches()
        #self.patches.random_color()
        #self.patches.plot_patches()

    def eval_pop(self,input_data):
        jump_log_points = []
        jump_log_traj = []
        xd = input_data[0]
        xc = input_data[1]
        #first discrete variable is number of jumps, the next ones are the of the patches
        n_jumps = xd[0] + 1
        ids = []
        fitness = 0.0
        #print(f"Number of jumps {n_jumps}\n")
        #reset initial final points
        self.p0 = self.p0_init
        self.pf = self.pf_init

        for i in range(n_jumps-1):
            #print(f"Jump n:{i}\n")
            # following discrete variables represent the id of the patches for the intermediate jumps
            patch_id = xd[1 + i]
            # the continue variables contain the X and Y normalized coordinate of the candidate contact landing points inside the candidate patches
            contact_relative_to_patch_yz= xc[i*2:i*2+2] # tra 0 - 1  upper left corner patch

            print("jump number : ", i)
            #computes 0, Y, Z  absolute coordinates of candidate landing location
            landing_abs_pos = self.patches.getAbsolutePoseOfPointInsidePatch(patch_id, contact_relative_to_patch_yz[0], contact_relative_to_patch_yz[1], scale=1.0)
            pf_adj = landing_abs_pos.copy()


            p0_adj = self.p0.copy()
            #adjust X coordinate to terrain shape for both liftoff and landing points
            p0_adj[0] = self.terrain_manager.wall_surface_eval(self.p0[2], self.p0[1], self.terrain_manager.mesh_x, self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
            pf_adj[0] = self.terrain_manager.wall_surface_eval(pf_adj[2], pf_adj[1], self.terrain_manager.mesh_x,self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)

            jump_log_points.append(p0_adj)
            #compute normal at liftoff
            liftoff_normal = self.terrain_manager.wall_normal_eval(self.p0[2], self.p0[1], self.terrain_manager.mesh_x,self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)

            # ora il punto assoluto è in pf_adj
            inner_opt_params['mesh_x'] = self.terrain_manager.mesh_x
            inner_opt_params['mesh_y'] = self.terrain_manager.mesh_y
            inner_opt_params['mesh_z'] = self.terrain_manager.mesh_z
            inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)


            res = eng.optimize_cpp_mex(matlab.double(p0_adj), matlab.double(pf_adj), Fleg_max, Fr_max, Fr_min, mu, inner_opt_params)
            fitness += self.calc_fitness(res, patch_id=patch_id, contact_abs_pos_yz=pf_adj[1:])
            jump_log_traj.append(mat_matrix2python(res['p']))
            self.p0 = pf_adj.copy()

        #print("final jump")
        #last jump is to pf
        #  absolute coordinates of FINAL landing location
        pf_adj = self.pf.copy()
        p0_adj = self.p0.copy()
        # adjust X coordinate to terrain shape for both liftoff and landing points
        pf_adj[0] = self.terrain_manager.wall_surface_eval(pf_adj[2], pf_adj[1], self.terrain_manager.mesh_x, self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        p0_adj[0] = self.terrain_manager.wall_surface_eval(self.p0[2], self.p0[1], self.terrain_manager.mesh_x, self.terrain_manager.mesh_y,   self.terrain_manager.mesh_z)
        # compute normal at liftoff
        liftoff_normal = self.terrain_manager.wall_normal_eval(self.p0[2], self.p0[1], self.terrain_manager.mesh_x, self.terrain_manager.mesh_y,   self.terrain_manager.mesh_z)
        # ora il punto assoluto è in pf_adj
        inner_opt_params['mesh_x'] = self.terrain_manager.mesh_x
        inner_opt_params['mesh_y'] = self.terrain_manager.mesh_y
        inner_opt_params['mesh_z'] = self.terrain_manager.mesh_z
        inner_opt_params['contact_normal'] = matlab.double(liftoff_normal)

        res = eng.optimize_cpp_mex(matlab.double(p0_adj), matlab.double(pf_adj), Fleg_max, Fr_max, Fr_min, mu, inner_opt_params)
        fitness += self.calc_fitness(res)
        ref_com = mat_matrix2python(res['p'])
        jump_log_traj.append(ref_com)
        jump_log_points.append(pf_adj)

        #plot traj
        # plot starting final points
        #ax = plt.gca()
        #for point in jump_log:
        #    ax.scatter(point[0], point[1], point[2], color='red', s=500)

        #self.point_clouds.plot_map_with_target(jump_log_points)
        self.point_clouds.animate_plot_map_with_target_and_trajectory(jump_log_points,jump_log_traj)
        return fitness

    def calc_fitness(self,res, patch_id=None, contact_abs_pos_yz=None):
        fit_average_cost_patch = 0.
        fit_landing_cost = 0.
        # filter apply
        # fare un nuovo pc filter e poi aggiungere i punti presi sopra
        if (patch_id is not None and  contact_abs_pos_yz is not None):
            #compute cost for landing candidate
            fit_landing_cost = -self.patches.get_cost_in_point(patch_id, contact_abs_pos_yz)
            #compute average cost on patch to see how bad /good is terrain there
            fit_average_cost_patch = -self.patches.get_patch_cost(patch_id)
            #if fit_landing_costmap is None:
            #    breakpoint()

        fit_consumed_energy = -res['consumed_energy']
        if (res['problem_solved']) == 1 or (res['problem_solved']==2): #convergence / semidefinite solution
            fit_problem_converged = 100
        else: #problem did not converge
            fit_problem_converged = 0
        # print("jump duration", res['Tf'])
        print(f"convergence: {fitness_weights[0]*fit_problem_converged}, energy: {fitness_weights[1]*fit_consumed_energy}, avg_cost: {fitness_weights[2]*fit_average_cost_patch}, land_cost: {fitness_weights[3]*fit_landing_cost}")
        fitness =  fitness_weights[0]*fit_problem_converged + fitness_weights[1]*fit_consumed_energy +fitness_weights[2]*fit_average_cost_patch + fitness_weights[3]*fit_landing_cost

        return fitness



def main():
    algo = CrossEntropyMethodMixed(cem_params) # <-- da implementare dentro la classe appena il tutto funziona
    # create terrain:
    terrain_manager = TerrainManager()
    # Optimizer part
    optimizer = BiLevelOptmizer(terrain_manager, P0_INIT,PF_INIT)

    cost_hist = np.zeros(cem_params.cem_iters)

    start = time.time()
    for k in range(cem_params.cem_iters):
        print(colored(f"Cross Entropy Iteration no. {k}\n","blue"))
        # Generate population by sampling the distributions
        algo.generate_population_discrete()
        algo.generate_population_continuous()
        xd = algo.population_discrete  # shape: dim_discrete x pop_size
        xc = algo.population_continuous  # shape: dim_continuous x pop_size

        # Organise inputs into a 2D matrix where hwe have as columns
        inputs = [[xd[:, i].tolist(), xc[:, i].tolist(), cem_params] for i in range(cem_params.pop_size)]

        # Evaluate population in parallel
        # with ProcessPoolExecutor(max_workers=cem_params.n_threads) as executor:
        #     fitness = list(executor.map(eval_pop, inputs))
        fitness = []
        for i, population_inputs in enumerate(inputs, start=1):
            result = optimizer.eval_pop(population_inputs)
            fitness.append(result)
            print(colored(f"Individual {i}/{len(inputs)} of population {k} finished, fitness = {result}\n", "red"))

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