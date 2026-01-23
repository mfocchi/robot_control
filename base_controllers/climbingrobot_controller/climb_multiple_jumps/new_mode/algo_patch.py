from typing import List, Callable, Tuple, Optional
from dataclasses import dataclass

import numpy as np

@dataclass
class CemParams:
    seed: int = None

    # CEM parameters
    parallel: bool = None
    n_threads: int = None
    cem_iters: int = None
    pop_size: int = None
    n_elites: int = None
    decrease_pop_factor: float = None
    fraction_elites_reused: float = None
    alpha: float = 0.5
    # Discrete
    dim_discrete: int = None
    n_values: List[int] = None
    init_probs: np.ndarray = None
    min_prob: float = None
    
    # Continuous
    dim_continuous: int = None
    min_value_continuous: np.ndarray = None
    max_value_continuous: np.ndarray = None
    init_mu_continuous: np.ndarray = None
    init_std_continuous: np.ndarray = None
    min_std_continuous: np.ndarray = None

@dataclass
class IterationLog:
    iterations: int = 0
    func_evals: int = 0
    best_discrete: Optional[np.ndarray] = None
    best_value: float = np.inf
    


class CrossEntropyMethodMixed:
    def __init__(self, params: CemParams):
        
        
        self.params = params
        self.log = IterationLog()
        self.update_coeff = 1.0 / float(params.n_elites)
        self.elites_reuse_size = max(
            0,
            min(
                params.n_elites,
                int(params.n_elites * params.fraction_elites_reused),
            ),
        )
        

        self.rng = np.random.default_rng(params.seed)
        

        assert params.pop_size > 0
        assert params.dim_discrete > 0
        assert params.n_elites > 0 and params.n_elites <= params.pop_size

        self.allocate_data_discrete()

        self.population_fit = np.full(params.pop_size, np.inf)
        self.fit_best = np.inf
        self.history = []
        
    def allocate_data_discrete(self):
        p = self.params
        self.population_discrete = np.zeros((p.dim_discrete, p.pop_size), dtype=int)
        self.elites_discrete = np.zeros((p.dim_discrete, p.n_elites), dtype=int)
        self.best_discrete = np.zeros(p.dim_discrete, dtype=int)
        self.probs = p.init_probs.copy()


    def generate_population(self):
        self.generate_population_discrete()

    def evaluate_population(self, population_fit):
        # Store calculated fitness score
        self.population_fit = np.copy(population_fit)

        # Update global best
        for i in range(self.params.pop_size):
            if self.population_fit[i] < self.fit_best:
                self.fit_best = np.copy(self.population_fit[i])
                self.best_discrete = np.copy(self.population_discrete[:, i])

    def update_distributions(self):
        self.update_distribution_discrete()

        self.log.iterations += 1
        self.log.func_evals += self.params.pop_size
        self.log.best_discrete = np.copy(self.best_discrete)
        self.log.best_value = np.copy(self.fit_best)
        
        self.history.append({
                "iter": self.log.iterations,
                "func_evals": self.log.func_evals,
                "best_value": float(self.fit_best),
                "best_discrete": np.copy(self.best_discrete),
                "probs": [np.array(p, copy=True) for p in self.probs],
            })
        

    def generate_population_discrete(self,first_iteration=False) -> None:
        for i in range(self.params.pop_size):
            for j in range(self.params.dim_discrete):
                if j == 0 and i == 0 and first_iteration == True:
                    self.population_discrete[j, i] = 0
                else:
                    p = self.rng.random()
                    s = 0.0
                    chosen_k = self.params.n_values[j] - 1
                    for k in range(1, self.params.n_values[j]):
                        
                        n_prob = self.probs[j][k] / (1.0 - self.probs[j][0])
                        s += n_prob
                        if p < s:
                            chosen_k = k
                            break
                    self.population_discrete[j, i] = chosen_k

    # def update_distribution_discrete(self):
    #     p = self.params
    #     # Sort individuals by their perfomance (best first!)
    #     idx = np.argsort(self.population_fit)[::-1]

    #     # Add elites to population
    #     self.elites_discrete = self.population_discrete[:, idx[: p.n_elites]]

    #     # Update probabilities using the elites
    #     for j in range(self.params.dim_discrete):
    #         new_probs_elites = np.zeros(p.n_values[j])
    #         for i in range(p.n_elites):
    #             val = self.elites_discrete[j, i]
    #             new_probs_elites[val] += 1.0
            
    #         new_probs_elites = new_probs_elites / p.n_elites
            
    #         updated_probs = (new_probs_elites * self.alpha) + (np.array(self.probs[j]) * (1.0 - self.alpha))
    #         updated_probs += p.min_prob
    #         self.probs[j] = updated_probs / np.sum(updated_probs)
    
    def update_distribution_discrete(self):
        p = self.params
        # Sort individuals by their perfomance (best first!)
        idx = np.argsort(self.population_fit)#[::-1]

        # Add elites to population
        self.elites_discrete = self.population_discrete[:, idx[: p.n_elites]]

        # Update probabilities using the elites
        for j in range(self.params.dim_discrete):
            counter = [0.0 for _ in range(p.n_values[j])]
            for i in range(p.n_elites):
                counter[self.elites_discrete[j, i]] += 1
            for k in range(p.n_values[j]):
                self.probs[j][k] = counter[k] / p.n_elites + p.min_prob

            self.probs[j] = self.probs[j] / np.sum(self.probs[j])
    