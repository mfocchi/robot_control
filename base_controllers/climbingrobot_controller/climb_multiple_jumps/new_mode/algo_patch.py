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
    best_value: float = -np.inf  # Changed from np.inf to -np.inf for maximization


class CrossEntropyMethodMixed:
    def __init__(self, params: CemParams, patch_p0, patch_pf):
        
        self.patch_p0 = patch_p0
        self.patch_pf = patch_pf
        
        self.alpha = params.alpha
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

        self.population_fit = np.full(params.pop_size, -np.inf)  # Changed from np.inf to -np.inf
        self.fit_best = -np.inf  # Changed from np.inf to -np.inf
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

        # Update global best (now looking for maximum)
        for i in range(self.params.pop_size):
            if self.population_fit[i] > self.fit_best:  # Changed from < to >
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
        

    def generate_population_discrete(self, first_iteration=False) -> None:
        for i in range(self.params.pop_size):
            if not first_iteration and self.log.iterations > 0 and i < self.elites_reuse_size:
                self.population_discrete[:, i] = self.elites_discrete[:, i]
                continue
            
            for j in range(self.params.dim_discrete):
                if j == 0 and i == 0 and first_iteration == True:
                    self.population_discrete[j, i] = 0
                else:
                    # Check if n_values[j] is an integer or a list
                    if isinstance(self.params.n_values[j], int): # hanlde number of jumps
                        p = self.rng.random()
                        s = 0.0
                        for k in range(self.params.n_values[j]):
                            s += self.probs[j][k]
                            if p < s:
                                break
                        self.population_discrete[j, i] = k +1
                    else: # hanlde list of patches
                        valid_indices = self.params.n_values[j]
                        p = self.rng.random()
                        s = 0.0
                        for idx, valid_k in enumerate(valid_indices):
                            s += self.probs[j][idx]
                            if p < s:
                                break
                        self.population_discrete[j, i] = valid_k
    
    
    
    def update_distribution_discrete(self):
        p = self.params
        # Sort individuals by their perfomance (best first - now descending for maximization!)
        idx = np.argsort(self.population_fit)[::-1]  # Added [::-1] to reverse sort
        # Add elites to population
        self.elites_discrete = self.population_discrete[:, idx[: p.n_elites]]
        
        # Update probabilities using the elites
        for j in range(self.params.dim_discrete):
            if isinstance(p.n_values[j], int): # hanlde number of jumps
                counter = [0.0 for _ in range(p.n_values[j])]
                for i in range(p.n_elites):
                    counter[self.elites_discrete[j, i] -1] += 1
                for k in range(p.n_values[j]):
                    self.probs[j][k] = counter[k] / p.n_elites + p.min_prob
            else: # hanlde list of patches
                
                valid_indices = p.n_values[j]
                num_valid = len(valid_indices)
                counter = [0.0 for _ in range(num_valid)]
                for i in range(p.n_elites):
                    actual_patch_id = self.elites_discrete[j, i]
                    # Find the index in valid_indices list
                    
                    idx_in_list = valid_indices.index(actual_patch_id)
                    counter[idx_in_list] += 1
                
                # Update probabilities
                for k in range(num_valid):
                    self.probs[j][k] = counter[k] / p.n_elites + p.min_prob

            self.probs[j] = self.probs[j] / np.sum(self.probs[j])
                     
    # def generate_population_discrete(self,first_iteration=False) -> None:
    #     # Generate random gaussian values from pure Normal distribution (mean=0, std=1)
    #     for i in range(self.params.pop_size):
    #         if not first_iteration and self.log.iterations > 0 and i < self.elites_reuse_size:
    #             self.population_discrete[:, i] = self.elites_discrete[:, i]
    #             continue
    #         for j in range(self.params.dim_discrete):
    #             if j == 0 and i == 0 and first_iteration == True:
    #                 self.population_discrete[j, i] = 0
    #             else: 
    #                 p = self.rng.random()
    #                 s = 0.0
    #                 for k in range(1, self.params.n_values[j]):  # Start from 1 instead of 0
    #                     s += self.probs[j][k]
    #                     if p < s:
    #                         break
    #                 self.population_discrete[j, i] = k

    # def generate_population_discrete(self,first_iteration=False) -> None:
    #     for i in range(self.params.pop_size):
    #         # Inject elites from previous iteration (skip if first iteration or if i >= elites_reuse_size)
    #         if not first_iteration and self.log.iterations > 0 and i < self.elites_reuse_size:
    #             self.population_discrete[:, i] = self.elites_discrete[:, i]
    #             continue
            
    #         used_patches = set()
    #         forbidden_patches = {self.patch_p0, self.patch_pf}
    #         for j in range(self.params.dim_discrete):
    #             if j == 0 and i == 0 and first_iteration == True:
    #                 self.population_discrete[j, i] = 0
    #                 continue
                
    #             current_probs = self.probs[j].copy()
                
    #             # For all cases except the first individual in first iteration, mask index 0
    #             if j == 0:
    #                 current_probs[0] = 0.0
                
    #             if j > 0:
    #                 all_to_mask = used_patches.union(forbidden_patches)
    #                 for forbidden in all_to_mask:
    #                     if forbidden is not None and 0 <= forbidden < len(current_probs):
    #                         current_probs[forbidden] = 0.0
                
    #             # Renormalize
    #             total_prob = np.sum(current_probs)
    #             if total_prob > 0:
    #                 current_probs = current_probs / total_prob
    #             else:
    #                 current_probs[:] = 1.0
    #                 if j == 0:
    #                     current_probs[0] = 0.0
    #                 if j > 0:
    #                     for forbidden in all_to_mask:
    #                         if forbidden is not None and 0 <= forbidden < len(current_probs):
    #                             current_probs[forbidden] = 0.0
    #                 current_probs /= np.sum(current_probs)
                
    #             candidates = np.arange(len(current_probs))
    #             chosen_k = self.rng.choice(candidates, p=current_probs)
                
    #             self.population_discrete[j, i] = chosen_k
    #             if j > 0:
    #                 used_patches.add(chosen_k)

    
    
    # def update_distribution_discrete(self):
    #     p = self.params
    #     # Sort individuals by their perfomance (best first!)
    #     idx = np.argsort(self.population_fit)#[::-1]

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
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    # def update_distribution_discrete(self):
    #     p = self.params
    #     # Sort individuals by their perfomance (best first!)
    #     idx = np.argsort(self.population_fit)#[::-1]

    #     # Add elites to population
    #     self.elites_discrete = self.population_discrete[:, idx[: p.n_elites]]

    #     # Update probabilities using the elites
    #     for j in range(self.params.dim_discrete):
    #         counts = np.bincount(self.elites_discrete[j, :], minlength=p.n_values[j])
    #         mle_probs = counts / p.n_elites
            
    #         if hasattr(p, 'alpha') and p.alpha is not None:
    #             current_probs = np.array(self.probs[j])
    #             updated_probs = (p.alpha * mle_probs) + ((1.0 - p.alpha) * current_probs)
    #         else:
    #             updated_probs = mle_probs

    #         # C. Enforce Constraints (Exploration/Min Prob)
    #         # Ensure no probability drops below min_prob to maintain exploration
    #         if p.min_prob is not None and p.min_prob > 0:
    #             updated_probs = updated_probs + p.min_prob
    #             # Renormalize to ensure sum equals 1.0
    #             updated_probs = updated_probs / np.sum(updated_probs)
            
    #         self.probs[j] = updated_probs
    
    # def update_distribution_discrete(self, thershold_min=0.001):
    #     p = self.params
    #     idx = np.argsort(self.population_fit)
    #     self.elites_discrete = self.population_discrete[:, idx[: p.n_elites]]

    #     for j in range(p.dim_discrete):
    #         # 1. Conta quante volte ogni patch è stata scelta dagli elite
    #         counter = np.zeros(p.n_values[j])
    #         for i in range(p.n_elites):
    #             valore_scelto = int(self.elites_discrete[j, i])
    #             counter[valore_scelto] += 1
            
    #         # 2. Calcola la probabilità base (frequenza degli elite)
    #         # Non aggiungere min_prob qui a tappeto se vuoi che il threshold funzioni!
    #         new_probs = counter / p.n_elites
            
    #         # 3. Applica il THRESHOLD
    #         # Se una patch non è stata scelta da nessuno o quasi, va a zero
    #         new_probs[new_probs < thershold_min] = 0.0
            
    #         # 4. Aggiungi un min_prob minuscolo solo a chi è sopravvissuto (opzionale)
    #         # o lascia che la rinormalizzazione faccia il suo corso.
            
    #         # 5. Rinormalizzazione
    #         s = np.sum(new_probs)
    #         if s > 0:
    #             self.probs[j] = new_probs / s
    #         else:
    #             # Se tutti sono a zero (estremamente raro), reset uniforme
    #             self.probs[j] = np.full(p.n_values[j], 1.0 / p.n_values[j])
    
    
    # def update_distribution_discrete(self):
    #     p = self.params
    #     # Sort individuals by their perfomance (best first!)
    #     idx = np.argsort(self.population_fit)#[::-1]

    #     # Add elites to population
    #     self.elites_discrete = self.population_discrete[:, idx[: p.n_elites]]

    #     # Update probabilities using the elites
    #     for j in range(self.params.dim_discrete):
    #         # A. Calculate Maximum Likelihood Estimate (MLE) from Elites
    #         # We count occurrences of each value in the elites for dimension j
    #         # np.bincount is faster than a python for-loop
    #         counts = np.bincount(self.elites_discrete[j, :], minlength=p.n_values[j])
    #         mle_probs = counts / p.n_elites
            
    #         # B. Apply Smoothing (The Standard CEM Update Rule)
    #         # new_param = alpha * mle_estimate + (1 - alpha) * old_param
    #         # This incorporates history to stabilize learning.
    #         if hasattr(p, 'alpha') and p.alpha is not None:
    #             current_probs = np.array(self.probs[j])
    #             updated_probs = (p.alpha * mle_probs) + ((1.0 - p.alpha) * current_probs)
    #         else:
    #             updated_probs = mle_probs

    #         # C. Enforce Constraints (Exploration/Min Prob)
    #         # Ensure no probability drops below min_prob to maintain exploration
    #         if p.min_prob is not None and p.min_prob > 0:
    #             updated_probs = updated_probs + p.min_prob
    #             # Renormalize to ensure sum equals 1.0
    #             updated_probs = updated_probs / np.sum(updated_probs)
            
    #         self.probs[j] = updated_probs
