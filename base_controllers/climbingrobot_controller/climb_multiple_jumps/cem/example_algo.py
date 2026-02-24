import numpy as np
from algo import CrossEntropyMethodMixed, CemParams
import matplotlib.pyplot as plt

def functions(x_discrete, x_continuous):
    """Evaluate different objective functions based on discrete choice."""
    x1, x2 = x_continuous[0], x_continuous[1]
    
    if x_discrete[0] == 0:
        # Inverted sphere function (peak at center)
        return 100 - (x1**2 + x2**2)
    elif x_discrete[0] == 1:
        # Modified Rosenbrock function
        return 100 - (100*(x2 - x1**2)**2 + (1 - x1)**2) / 100
    else:
        # Sinusoidal function
        return 50 + 30*np.sin(x1) + 30*np.cos(x2)


def plot_convergence(cem):
    """Plot fitness, discrete probabilities, and continuous std evolution."""
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    
    # Fitness evolution
    best_values = [h['best_value'] for h in cem.history]
    axes[0].plot(best_values, linewidth=2)
    axes[0].set(xlabel='Iteration', ylabel='Best Fitness', title='Fitness Convergence')
    axes[0].grid(True, alpha=0.3)
    
    # Discrete probabilities evolution
    probs_history = np.array([h['probs'][0] for h in cem.history])
    for i in range(3):
        axes[1].plot(probs_history[:, i], label=f'Function {i}', linewidth=2)
    axes[1].set(xlabel='Iteration', ylabel='Probability', title='Discrete Probabilities Evolution')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Standard deviation evolution
    std_history = np.array([h['std_devs'] for h in cem.history])
    axes[2].plot(std_history[:, 0], label='x1', linewidth=2)
    axes[2].plot(std_history[:, 1], label='x2', linewidth=2)
    axes[2].set(xlabel='Iteration', ylabel='Standard Deviation', title='Continuous Distributions Convergence')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def main():
    params = CemParams(
        seed=42,
        parallel=False,
        n_threads=1,
        cem_iters=15,
        pop_size=100,
        n_elites=20,
        decrease_pop_factor=1.0,
        fraction_elites_reused=0.2,
        # Discrete parameters
        dim_discrete=1,
        n_values=[3],
        init_probs=np.array([[1/3, 1/3, 1/3]]),
        min_prob=0.01,
        # Continuous parameters
        dim_continuous=2,
        min_value_continuous=np.array([-5.0, -5.0]),
        max_value_continuous=np.array([5.0, 5.0]),
        init_mu_continuous=np.array([0.0, 0.0]),
        init_std_continuous=np.array([2.0, 2.0]),
        min_std_continuous=np.array([0.1, 0.1])
    )
    
    cem = CrossEntropyMethodMixed(params)
    
    print("Starting optimization with Cross-Entropy Method")
    print("=" * 60)
    
    # Main optimization loop
    for iteration in range(params.cem_iters):
        cem.generate_population()
        
        # Evaluate fitness for all population members
        fitness_values = np.array([
            functions(cem.population_discrete[:, i], cem.population_continuous[:, i])
            for i in range(params.pop_size)
        ])
        
        cem.evaluate_population(fitness_values)
        cem.update_distributions()
        
        # Progress update every 10 iterations
        if (iteration + 1) % 10 == 0:
            print(f"Iteration {iteration + 1:3d} | Best Fitness: {cem.fit_best:8.3f} | "
                  f"Function: {cem.best_discrete[0]} | "
                  f"x1={cem.best_continuous[0]:6.3f}, x2={cem.best_continuous[1]:6.3f}")
    
    # Print final results
    print("=" * 60)
    print(f"\nFinal Results:")
    print(f"Best fitness: {cem.fit_best:.4f}")
    print(f"Selected function: {cem.best_discrete[0]}")
    print(f"Optimal parameters: x1={cem.best_continuous[0]:.4f}, x2={cem.best_continuous[1]:.4f}")
    print(f"\nFinal function selection probabilities:")
    for i, prob in enumerate(cem.probs[0]):
        print(f"  Function {i}: {prob:.4f}")
    
    plot_convergence(cem)


if __name__ == "__main__":
    main()