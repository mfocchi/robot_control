import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from typing import List
from dataclasses import dataclass

from algo_patch import CrossEntropyMethodMixed, CemParams

# Grid configuration
GRID_SIZE = 10
N_TILES = GRID_SIZE * GRID_SIZE
START_POS = np.array([0, 0])
END_POS = np.array([9, 9])
EXP_A = 0.1

# Cost weights
MISSING_WAYPOINT_PENALTY = 40.0  # Reduced to encourage more waypoints
ENERGY_LOG_WEIGHT = 15.0  # Reduced to balance cost
LINEAR_DIST_WEIGHT = 5.0  # Reduced weight

# Wall obstacles
WALLS = {
    (1, 1), (1, 2), (1, 3),
    (3, 3), (3, 4), (3, 5), (3, 6),
    (5, 0), (5, 1), (5, 2),
    (7, 5), (7, 6), (7, 7), (7, 8), (7, 9),
    (8, 2), (4, 8), (5, 8), (6, 8)
}

def linear_computation_exponential(p0, pf, a=EXP_A):
    """Calculate energy and trajectory between two points using exponential model."""
    dist_vec = pf - p0
    distance = np.linalg.norm(dist_vec)
    problem_solved = 1 if distance > 0 else 0
    
    # Generate linear trajectory points
    t = np.linspace(0, 1, 30)
    trajectory = np.array([p0[i] + t * (pf[i] - p0[i]) for i in range(len(p0))])
    
    # Energy: E(d) = d * exp(d - a)
    consumed_energy = distance * np.exp(distance - a)
    
    return {
        'consumed_energy': float(consumed_energy),
        'problem_solved': problem_solved,
        'linear_trajectory': trajectory
    }

def check_collision(trajectory_points):
    """Check if trajectory intersects walls or goes out of bounds."""
    for i in range(trajectory_points.shape[1]):
        r, c = int(np.round(trajectory_points[0, i])), int(np.round(trajectory_points[1, i]))
        if not (0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE) or (r, c) in WALLS:
            return True
    return False

def decode_individual(individual: np.ndarray, max_waypoints: int) -> List[np.ndarray]:
    """Decode genome to path: [num_waypoints, tile1, tile2, ...]."""
    n_active_waypoints = np.clip(int(individual[0]), 0, max_waypoints)
    
    path = [START_POS]
    for i in range(1, n_active_waypoints + 1):
        tile_idx = int(individual[i])
        path.append(np.array([tile_idx // GRID_SIZE, tile_idx % GRID_SIZE]))
    path.append(END_POS)
    return path

def evaluate_fitness(population: np.ndarray, max_waypoints: int) -> np.ndarray:
    """Evaluate fitness for population. Lower is better."""
    pop_size = population.shape[1]
    scores = np.zeros(pop_size)
    
    for i in range(pop_size):
        path = decode_individual(population[:, i], max_waypoints)
        total_energy = 0.0
        total_linear_dist = 0.0
        collision = False
        
        # Evaluate each segment
        for k in range(len(path) - 1):
            p0, pf = path[k], path[k+1]
            dist = np.linalg.norm(pf - p0)
            total_linear_dist += dist
            
            res = linear_computation_exponential(p0, pf, a=EXP_A)
            if check_collision(res['linear_trajectory']):
                collision = True
                break
            total_energy += res['consumed_energy']
        
        if collision:
            scores[i] = -1e7  # Penalty for collision
        else:
            n_intermediate = len(path) - 2
            waypoint_cost = (max_waypoints - n_intermediate) * MISSING_WAYPOINT_PENALTY
            cost_dist = total_linear_dist * LINEAR_DIST_WEIGHT
            energy_cost = np.log(total_energy + 1) * ENERGY_LOG_WEIGHT
            scores[i] = -(waypoint_cost + energy_cost + cost_dist)
            
    return scores

def plot_results(best_path_nodes, scores, all_chosen_tiles):
    """Visualize best path on grid."""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Create grid with walls
    map_matrix = np.array([[1 if (r, c) in WALLS else 0 
                            for c in range(GRID_SIZE)] 
                           for r in range(GRID_SIZE)])
    
    cmap = colors.ListedColormap(['white', 'black'])
    ax.imshow(map_matrix, cmap=cmap, origin='upper')
    ax.set_xticks(np.arange(-.5, GRID_SIZE, 1), minor=True)
    ax.set_yticks(np.arange(-.5, GRID_SIZE, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    
    # Plot trajectory segments
    for k in range(len(best_path_nodes) - 1):
        p0, pf = best_path_nodes[k], best_path_nodes[k+1]
        res = linear_computation_exponential(p0, pf, a=EXP_A)
        traj = res['linear_trajectory']
        ax.plot(traj[1, :], traj[0, :], color='blue', linewidth=2, alpha=0.8)
        ax.scatter(traj[1, 0], traj[0, 0], c='red', s=30, zorder=5)
    
    # Mark start and end
    ax.text(START_POS[1], START_POS[0], 'S', ha='center', va='center', 
            color='green', fontweight='bold', fontsize=14)
    ax.text(END_POS[1], END_POS[0], 'E', ha='center', va='center', 
            color='green', fontweight='bold', fontsize=14)
    ax.set_title(f"Best Path (score: {scores:.2f})\nIntermediate waypoints: {len(best_path_nodes)-2}")
    
    plt.tight_layout()
    plt.show()

def plot_waypoint_histogram(cem_history, max_waypoints, ax=None, use_last_iter_only=False):
    """Analyze distribution of waypoint counts (gene 0)."""
    if not cem_history:
        print("WARNING: No history data found.")
        return
    

    data = [it['best_discrete'][0] for it in cem_history]
    title_suffix = "(Best Evolution Across Iterations)"
    
    if not data:
        return

    fig, ax = plt.subplots(figsize=(10, 6))

    bins = np.arange(-0.5, max_waypoints + 1.5, 1)
    counts, _, patches = ax.hist(
        data, bins=bins, color='lightcoral',
        edgecolor='black', alpha=0.8, rwidth=0.8
    )

    ax.set_title(f'Waypoint Count Distribution {title_suffix}', fontsize=14)
    ax.set_xlabel('Number of Intermediate Waypoints', fontsize=12)
    ax.set_ylabel('Frequency', fontsize=12)
    ax.set_xticks(range(0, max_waypoints + 1))
    ax.set_xlim(-0.5, max_waypoints + 0.5)
    ax.grid(axis='y', linestyle='--', alpha=0.4)

    # Add count labels
    for i, count in enumerate(counts):
        if count > 0:
            ax.text(i, count, str(int(count)), ha='center', va='bottom', fontsize=10)

    plt.tight_layout()
    plt.show()

def run_cem_trajectory():
    """Main optimization routine using Cross-Entropy Method."""
    MAX_WAYPOINTS = 10
    n_values = [MAX_WAYPOINTS + 1] + [N_TILES] * MAX_WAYPOINTS
    
    # Initialize probability distributions - more uniform for better exploration
    p_len = np.ones(MAX_WAYPOINTS + 1) / (MAX_WAYPOINTS + 1)
    # Give slight bias to middle range
    for i in range(3, 8):
        p_len[i] *= 1.5
    p_len = p_len / np.sum(p_len)
    
    init_probs = [p_len] + [np.ones(N_TILES) / N_TILES for _ in range(MAX_WAYPOINTS)]
    
    print("Initial waypoint count distribution:")
    for i, prob in enumerate(p_len):
        print(f"  {i} waypoints: {prob:.3f}")
    
    # CEM parameters - adjusted for better exploration
    params = CemParams(
        seed=0,
        pop_size=2500,
        n_elites=30,
        cem_iters=50,
        dim_discrete=len(n_values),
        n_values=n_values,
        init_probs=init_probs,
        min_prob=0.01,  # Increased for more exploration
        fraction_elites_reused=0.2,  # Added elite reuse
    )
    
    cem = CrossEntropyMethodMixed(params,patch_p0=None, patch_pf=None)
    all_chosen_tiles = []
    
    print("\nStarting optimization...")
    
    for it in range(params.cem_iters):
        cem.generate_population()
        all_chosen_tiles.extend(cem.population_discrete[1:, :].flatten())
        
        scores = evaluate_fitness(cem.population_discrete, MAX_WAYPOINTS)
        cem.evaluate_population(scores)
        cem.update_distributions()
        
        # Store population in history
        if hasattr(cem, 'history') and cem.history:
            cem.history[-1]['population'] = cem.population_discrete.copy()
        
        if (it + 1) % 5 == 0:
            print(f"Iter {it+1}/{params.cem_iters} | Score: {cem.fit_best:.2f} | Best waypoints: {cem.best_discrete[0]}")
    
    # Extract and display results
    best_path = decode_individual(cem.best_discrete, MAX_WAYPOINTS)
    print(f"Intermediate waypoints: {len(best_path) - 2}")
    print(f"Complete path ({len(best_path)} nodes): {best_path}")
    
    plot_results(best_path, cem.fit_best, all_chosen_tiles)
    
    plot_waypoint_histogram(cem.history, MAX_WAYPOINTS)
    

if __name__ == "__main__":
    run_cem_trajectory()