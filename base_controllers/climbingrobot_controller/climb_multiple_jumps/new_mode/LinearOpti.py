import os
import sys
import numpy as np
from termcolor import colored
import threading
import matlab.engine
from params import *
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches 
from base_controllers.utils.matlab_conversions import (
    mat_matrix2python,
    mat_vector2python,
)

thread_local = threading.local()

    
class LinearOpti:
    
    def __init__(self, terrain_manager, p0, pf, fitness_weights, point_clouds, patches, cost_grid):
        
        self.terrain_manager = terrain_manager
        self.p0 = p0
        self.pf = pf
        self.fitness_weights = fitness_weights
        
        # terrain creation done outside to avoid recomputation at each thread
        self.point_clouds = point_clouds
        self.patches = patches
        self.cost_grid = cost_grid
    
    
    def eval_pop(self, input_data):
        
        jump_log_points = []
        jump_log_traj = []
        xd = input_data[0]  # xd is the discrete variables and contains the patches
        xc = input_data[1]  # xc is the continuous variables and contains the jump coordinates
        n_jumps = xd[0]     # first discrete variable is number of jumps, the next ones are the of the patches
        ids = []
        fitness = 0.0
        total_consumed_energy = 0.0
        total_landing_cost = 0.0
        
        print(colored(f"[EVAL] Evaluating individual with {n_jumps + 1} total jumps ({n_jumps} intermediate + 1 final)", "blue"))
        
        p0_adj = self.p0.copy()
        
        p0_adj[0] = self.terrain_manager.wall_surface_eval(
            p0_adj[2], p0_adj[1], self.terrain_manager.mesh_x,
            self.terrain_manager.mesh_y, self.terrain_manager.mesh_z)
        
        jump_log_points.append(p0_adj.copy())
        
        for i in range(n_jumps):
            
            patch_id = xd[1 + i] 
            # the continue variables contain the X and Y normalized coordinate of the candidate contact landing points inside the candidate patches
            contact_relative_to_patch_yz = xc[i * 2:i * 2 + 2]  # tra 0 - 1  upper left corner patch

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

            
            
            #tun optimization
            res = self.linear_computation(p0_adj, pf_adj)
            
            jump_log_traj.append(res['linear_trajectory'])
            
            # Track energy and landing cost
            total_consumed_energy += res['consumed_energy']
            landing_cost = self.patches.get_cost_in_point(patch_id, pf_adj[1:])
            total_landing_cost += landing_cost
            
            fitness += self.calc_linear_fitness(res, patch_id=patch_id, 
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
        
        
        res = self.linear_computation(p0_adj, pf_adj)
        
        # Track energy for final jump
        total_consumed_energy += res['consumed_energy']
        
        fitness += self.calc_linear_fitness(res)
        
        if (n_jumps+1) < 2:
            fitness -= 500.0  # Valore molto basso per "spaventare" l'algoritmo
        
        jump_log_traj.append(res['linear_trajectory'])
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
        
        return log_result
    
    def linear_computation(self, p0, pf):
        """
        Calcola i parametri per una traiettoria lineare tra due punti.
        """
        # 1. Calcolo della distanza (Euclidean distance) come misura di energia consumata
        distance = np.linalg.norm(pf - p0)
        
        # 2. Verifica se il problema è risolto (se i punti sono validi e la distanza è calcolabile)
        # Assumiamo che se la distanza è definita (anche 0), il problema è risolto.
        problem_solved = 1 if distance is not None else 0
        
        # 3. Generazione della traiettoria lineare
        # Creiamo N punti tra p0 e pf. Il formato richiesto dai plot è (3, N)
        num_points = 50 
        t = np.linspace(0, 1, num_points)
        
        # Interpolazione lineare: P(t) = P0 + t * (Pf - P0)
        # Usiamo np.outer o broadcasting per ottenere la matrice 3x50
        trajectory = np.zeros((3, num_points))
        for i in range(3): # Per X, Y, Z
            trajectory[i, :] = p0[i] + t * (pf[i] - p0[i])
            
        res = {
            'consumed_energy': float(distance),
            'problem_solved': problem_solved,
            'linear_trajectory': trajectory
        }
        
        return res
           
    def calc_linear_fitness(self,res, patch_id=None, contact_abs_pos_yz=None):
        fit_average_cost_patch = 0.
        fit_landing_cost = 0.

        if (patch_id is not None and  contact_abs_pos_yz is not None):
            #compute cost for landing candidate
            fit_landing_cost = -self.patches.get_cost_in_point(patch_id, contact_abs_pos_yz)
            #compute average cost on patch to see how bad /good is terrain there
            fit_average_cost_patch = -self.patches.get_patch_cost(patch_id)
            #if fit_landing_costmap is None:

        fit_consumed_energy = -res['consumed_energy']
        if (res['problem_solved']) == 1 :
            fit_problem_converged = 0
        else: #problem did not converge
            fit_problem_converged = -100
        # print("jump duration", res['Tf'])
        print(f"convergence: {self.fitness_weights[0]*fit_problem_converged}, energy: {self.fitness_weights[1]*fit_consumed_energy}, avg_cost: {self.fitness_weights[2]*fit_average_cost_patch}, land_cost: {self.fitness_weights[3]*fit_landing_cost}")
        
        fitness = ( self.fitness_weights[0] * fit_problem_converged + 
                    self.fitness_weights[1] * fit_consumed_energy +
                    self.fitness_weights[2] * fit_average_cost_patch + 
                    self.fitness_weights[3] * fit_landing_cost)
        return fitness
    def plot_point_traj(self, jump_log_points, jump_log_traj):
        # Conversione sicura dei punti della nuvola
        x_points = np.array([point['position'][0] for point in self.point_clouds.points_t])
        y_points = np.array([point['position'][1] for point in self.point_clouds.points_t])
        z_points = np.array([point['position'][2] for point in self.point_clouds.points_t])
        color = np.array([point['color'] for point in self.point_clouds.points_t])
        size_point = np.array([point['size_point'] for point in self.point_clouds.points_t])
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Terreno (alpha ridotta per far risaltare la traiettoria)
        ax.scatter(x_points, y_points, z_points, c=color, s=size_point, alpha=0.4, label='Terrain')
        
        # Plot dei Waypoints
        pts = np.array(jump_log_points)
        for i, point in enumerate(pts):
            if i == 0:  
                ax.scatter(point[0], point[1], point[2], c='green', s=150, marker='o', 
                           edgecolors='black', linewidths=2, label='Start Point', zorder=10)
            elif i == len(pts) - 1:
                ax.scatter(point[0], point[1], point[2], c='red', s=150, marker='X', 
                           edgecolors='black', linewidths=2, label='End Point', zorder=10)
            else:
                ax.scatter(point[0], point[1], point[2], c='cyan', s=100, marker='o', 
                           edgecolors='black', label='Jump Contact' if i == 1 else '', zorder=10)
        
        # Plot delle Traiettorie lineari
        colors = plt.cm.jet(np.linspace(0, 1, len(jump_log_traj)))
        for i, traj in enumerate(jump_log_traj):
            if traj is not None and traj.ndim == 2:
                ax.plot(traj[0, :], traj[1, :], traj[2, :], color=colors[i], 
                        linewidth=3, alpha=1.0, zorder=15, label=f'Segment {i+1}')
        
        # Scaling degli assi uguale (fondamentale per non deformare la linea)
        all_pts = np.vstack([pts] + [t.T for t in jump_log_traj if t is not None])
        max_range = np.ptp(all_pts, axis=0).max() / 2.0
        mid = np.mean(all_pts, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

        ax.set_xlabel('X (Depth/Height)')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Path Visualization (Points)\nSegments: {len(jump_log_traj)}')
        ax.legend(loc='upper right')
        plt.show()
        
    def plot_mesh_traj(self, jump_log_points, jump_log_traj):
        X, Y, Z = self.terrain_manager.mesh_x, self.terrain_manager.mesh_y, self.terrain_manager.mesh_z

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Superficie del terreno
        surf = ax.plot_surface(X, Y, Z, color='gray', edgecolor='none', alpha=0.3, antialiased=True)

        pts = np.array(jump_log_points)
        # Waypoints
        ax.scatter(pts[0, 0], pts[0, 1], pts[0, 2], c='lime', s=120, marker='o', edgecolors='k', label='Start', zorder=20)
        ax.scatter(pts[-1, 0], pts[-1, 1], pts[-1, 2], c='darkred', s=150, marker='*', edgecolors='k', label='Goal', zorder=20)
        
        if len(pts) > 2:
            ax.scatter(pts[1:-1, 0], pts[1:-1, 1], pts[1:-1, 2], c='blue', s=80, marker='o', label='Contacts', zorder=20)
        
        # Traiettorie
        for i, traj in enumerate(jump_log_traj):
            if traj is not None:
                ax.plot(traj[0, :], traj[1, :], traj[2, :], color='blue', linewidth=2.5, zorder=25)

        # Labels e legende
        ax.set_title('Linear Trajectory on Terrain Mesh')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        # Impostazione limiti assi proporzionali
        all_coords = np.concatenate([pts] + [t.T for t in jump_log_traj if t is not None])
        mid = (all_coords.max(axis=0) + all_coords.min(axis=0)) / 2
        span = (all_coords.max(axis=0) - all_coords.min(axis=0)).max() / 2
        ax.set_xlim(mid[0] - span, mid[0] + span)
        ax.set_ylim(mid[1] - span, mid[1] + span)
        ax.set_zlim(mid[2] - span, mid[2] + span)

        ax.view_init(elev=35, azim=-60)
        plt.tight_layout()
        plt.show()