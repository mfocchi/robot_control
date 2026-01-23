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
from algo_patch import CrossEntropyMethodMixed
from base_controllers.components.terrain_manager import TerrainManager
from LinearOpti import LinearOpti
from params import *
from base_controllers.climbingrobot_controller.climb_multiple_jumps.new_mode.Plot_result import PlotResultCemMjumps
from collections import Counter

def main():
    
    setting = {
        "COMPUTATION_MODE": True,
        "WARM_START_MODE": True,
        "PLOT_MODE": True
    }
    
    if setting["COMPUTATION_MODE"]:
        best_lock = threading.Lock()
        
        
        terrain_manager = TerrainManager()
        point_clouds, patches, cost_grid = initialize_terrain_data(terrain_manager, warm_start_mode=setting["WARM_START_MODE"])
        optimizer = LinearOpti(
            terrain_manager, P0_INIT, PF_PATCH_INIT, 
            fitness_weights=fitness_weights,
            point_clouds=point_clouds,
            patches=patches,
            cost_grid=cost_grid
        )
        
        algo = CrossEntropyMethodMixed(cem_params)
        
        cost_hist = np.zeros(cem_params.cem_iters)
        best_jump_log_points = None
        best_jump = None
        best_trajectory = None
        best_fitness = np.inf
        best_consumed_energy = None
        best_landing_cost = None
        
        start = time.time()
        
        for k in range(cem_params.cem_iters):
            iter_start = time.time()
            first_iteration = (k == 0)
            
            if k==3 and setting["WARM_START_MODE"]:
                algo.params.min_prob = 0.01
                for j in range(algo.params.dim_discrete):
                    n_val = algo.params.n_values[j]
                    algo.probs[j] = np.full(n_val, 1.0 / n_val)
                    
            algo.generate_population_discrete(first_iteration)
            
            if not first_iteration and algo.log.best_discrete is not None:
                # Substitute the first individual of the random population with the Best Ever (ELITE INJECTION)
                print(colored(f"[INFO] Injecting previous best solution into population (Iter {k+1})", "cyan"))
                algo.population_discrete[:, 0] = algo.log.best_discrete
            
            xd = algo.population_discrete   # shape: dim_discrete x pop_size
            # print(xd)
            counter = Counter(xd[0])
            total = len(xd[0])
            for value in sorted(counter.keys()):
                perc = counter[value] / total * 100
                # print(f"Jump {value}: {perc:.2f}%")
            
            
            # Organise inputs into a 2D matrix where we have as columns
            inputs = [[xd[:, i].tolist()] for i in range(cem_params.pop_size)]
            
            
            fitness = [0.0] * cem_params.pop_size #-xd[0].copy() # 
            all_log_points = [None] * cem_params.pop_size
            all_log_traj = [None] * cem_params.pop_size
            all_consumed_energy = [0.0] * cem_params.pop_size
            all_landing_cost = [0.0] * cem_params.pop_size
            all_n_jumps = [0] * cem_params.pop_size
            n_workers = cem_params.n_threads
            
            patch_ids_esplorati = xd[1:, :].flatten().astype(int)
            patches.plot_population_density(patch_ids_esplorati)
            
            
            # ============================
            # flag_thread == TRUE: multi-threaded evaluation
            # ============================
            if (flag_thread == True):
                
                print(colored(f"\n{'='*60}", "yellow"))
                print(colored(f"{n_workers} Thread evaluation with ThreadPoolExecutor, Iteration {k+1}/{cem_params.cem_iters}", "yellow", attrs=['bold']))
                print(colored(f"{'='*60}\n", "yellow"))
                
                with ThreadPoolExecutor(max_workers=n_workers) as executor:
                    # map futures to their input indices
                    future_to_index = {executor.submit(optimizer.eval_pop, inputs[i]): i 
                                    for i in range(cem_params.pop_size)}
                    
                    for future in as_completed(future_to_index):
                        idx = future_to_index[future]
                        log_result = future.result()
                        fitness[idx] = log_result['fitness']
                        all_log_points[idx] = log_result['points']
                        all_log_traj[idx] = log_result['traj']
                        all_consumed_energy[idx] = log_result['consumed_energy']
                        all_landing_cost[idx] = log_result['landing_cost']
                        all_n_jumps[idx] = log_result['n_jumps']
                        
                        with best_lock:
                            
                            if log_result['fitness'] < best_fitness: #and (n_jumps + 1) >= 3:
                                best_fitness = log_result['fitness']
                                best_consumed_energy = log_result['consumed_energy']
                                best_landing_cost = log_result['landing_cost']
                                best_jump_log_points = log_result['points']
                                best_trajectory = log_result['traj']
                                best_jump = log_result['n_jumps']
                                print(colored(f"[NEW BEST] Indiv {idx}: Fitness {best_fitness:.2f}", "green"))
                        print(colored(f"complete individual {idx}, Iteration {k+1}", "yellow")) #scrvi in arancione
                
                print() 
                print(colored(f"[ITERATION END] Best fitness: {best_fitness:.2f}", "green"))
            
            # ============================
            # flag_thread == FALSE: sequential evaluation
            # ============================
            else:
                print(colored(f"\n{'='*60}", "yellow"))
                print(colored("Sequential evaluation", "yellow", attrs=['bold']))
                print(colored(f"{'='*60}\n", "yellow"))
                
                for i, population_inputs in enumerate(inputs):
                    log_result = optimizer.eval_pop(population_inputs)
                    
                    
                    print(colored(f"\n[COMPLETE] Individual {i+1}/{len(inputs)} of iteration {k+1} finished, fitness = {log_result['fitness']:.4f}\n", "red", attrs=['bold']))

                    fitness[i] = log_result['fitness']
                    all_log_points[i] = log_result['points']
                    all_log_traj[i] = log_result['traj']
                    all_consumed_energy[i] = log_result['consumed_energy']
                    all_landing_cost[i] = log_result['landing_cost']
                    all_n_jumps[i] = log_result['n_jumps']
                    
                    if log_result['fitness'] < best_fitness:
                        best_fitness = log_result['fitness']
                        best_consumed_energy = log_result['consumed_energy']
                        best_landing_cost = log_result['landing_cost']
                        best_jump_log_points = log_result['points']
                        best_jump = log_result['n_jumps']
                        best_trajectory = log_result['traj']
                        
                        print(colored(f"[NEW BEST] Fitness: {best_fitness:.2f} with {best_jump} jumps", "green", attrs=['bold']))

                    
                    
            # Update distributions
            algo.evaluate_population(fitness)
            algo.update_distributions()
            cost_hist[k] = algo.log.best_value 
                   
            print ("finish iteration ", k+1)
            iter_time = time.time() - iter_start
            
            # if flag_thread == False:
                # optimizer.plot_point_traj(best_jump_log_points, best_trajectory)
                # optimizer.plot_mesh_traj(best_jump_log_points, best_trajectory,best_fitness)
                
            print(colored(f"\n{'='*60}", "cyan", attrs=['bold']))
            print(colored(f"  Iteration {k+1} completed in {iter_time:.2f}s", "cyan", attrs=['bold']))
            print(colored(f"  Best value this iteration: {algo.log.best_value:.4f}", "cyan", attrs=['bold']))
            print(colored(f"{'='*60}\n", "cyan", attrs=['bold']))
            
            
            # =======================
            # SAVE PARTS
            # =======================
            
            # 1. Save elites of current iteration
            num_elites = cem_params.n_elites
            sorted_indices = np.argsort(fitness)
            elite_indices = sorted_indices[:num_elites]
            
            current_iteration_elites = []
            for idx in elite_indices:
                elite_sol = {
                    'fitness': float(fitness[idx]),
                    'n_jumps':   int(all_n_jumps[idx]),
                    'consumed_energy': float(all_consumed_energy[idx]),
                    'landing_cost': float(all_landing_cost[idx]),
                    'points': [p.tolist() for p in all_log_points[idx]],
                    'traj': [t.tolist() if t is not None else None for t in all_log_traj[idx]],
                    'iteration': k + 1,
                    'patch_ids': xd[1:, idx].tolist(),        # Gli ID delle patch scelti (saltando il primo che è N_jump)
                }
                current_iteration_elites.append(elite_sol)
            
            iteration_report = {
                "iteration": k + 1,
                "best_fitness_ever": float(best_fitness),
                "best_consumed_energy_ever": float(best_consumed_energy) if best_consumed_energy is not None else None,
                "best_landing_cost_ever": float(best_landing_cost) if best_landing_cost is not None else None,
                "best_trajectory_ever": [t.tolist() if t is not None else None for t in best_trajectory] if best_trajectory is not None else None,
                "best_fitness_this_iter": float(current_iteration_elites[0]['fitness']),
                "elites": current_iteration_elites
            }
            
            subdir = "iteration_reports"
            iteration_filename = f"iteration_{k+1:03d}_report.json"
            
            save_dir = os.path.join(result_dir, subdir)
            os.makedirs(save_dir, exist_ok=True)
            with open(os.path.join(save_dir, iteration_filename), "w") as f:
                json.dump(iteration_report, f, indent=2)
            
            print(colored(f"[SAVE] Iteration saved to: {iteration_filename}", "cyan"))
            
        n_workers = cem_params.n_threads
        
        # with ThreadPoolExecutor(max_workers=n_workers) as executor:
        #     executor.map(lambda x: close_matlab_engines(), range(n_workers))
            
        # Save wall-time
        end = time.time()
        wall_time = end - start

        xd = algo.best_discrete
        
        # =======================
        # FINAL SAVE
        # =======================
        print(f"Best discrete solution: {xd}")
        # Generate and save report json
        
        iteration_history = []
        for h in algo.history:
            iter_data = {
                "iteration": h["iter"],
                "func_evals": h["func_evals"],
                "best_value": h["best_value"],
                "best_discrete": h["best_discrete"].tolist(),
                "probs": [p.tolist() for p in h["probs"]],
            }
            iteration_history.append(iter_data)
            
        history_filename = "cem_iteration_history.json"
        history_save_path = os.path.join(result_dir, history_filename)
        with open(history_save_path, "w") as f:
            json.dump({
                "metadata": {
                    "timestamp": datetime.now().isoformat(),
                    "total_iterations": len(iteration_history)
                },
                "iteration_history": iteration_history
            }, f, indent=2)
        print(colored(f"[SAVE] Iteration history saved to: {history_save_path}", "blue"))
        
        #save best log_points and trajectory
        best_log_data={
            "best_jump_log_points": [p.tolist() for p in best_jump_log_points] if best_jump_log_points is not None else None,
            "best_trajectory": [t.tolist() if t is not None else None for t in best_trajectory] if best_trajectory is not None else None
        }
        # best_log_filename = "best_trajectory_log.json"
        # best_log_save_path = os.path.join(result_dir, best_log_filename)
        # with open(best_log_save_path, "w") as f:
        #     json.dump(best_log_data, f, indent=2)
        # print(colored(f"[SAVE] Best trajectory log saved to: {best_log_save_path}", "blue"))
        
        # Plot best trajectory at the end
        print(colored(f"\n{'='*70}", "green", attrs=['bold']))
        print(colored(f"  OPTIMIZATION FINISHED!", "green", attrs=['bold']))
        print(colored(f"  Best Fitness: {best_fitness:.4f}", "green", attrs=['bold']))
        print(colored(f"  Total Time: {wall_time:.2f}s", "green", attrs=['bold']))
        print(colored(f"{'='*70}\n", "green", attrs=['bold']))
        
        if best_jump_log_points and best_trajectory:
            optimizer.plot_point_traj(best_jump_log_points, best_trajectory)
            optimizer.plot_mesh_traj(best_jump_log_points, best_trajectory,best_fitness)
        else:
            print(colored("[ERROR] Could not plot best trajectory. No solution found or tracking issue.", "red", attrs=['bold']))
    if setting["PLOT_MODE"]:
        
        print("plot da stampare")
if __name__ == "__main__":
    # try:
        main()
    # finally:
    #     close_matlab_engines()