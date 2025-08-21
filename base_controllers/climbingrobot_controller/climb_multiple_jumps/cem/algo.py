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
    best_continuous: Optional[np.ndarray] = None
    best_value: float = -np.inf


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
        assert params.dim_continuous > 0
        assert params.n_elites > 0 and params.n_elites <= params.pop_size

        self.allocate_data_discrete()
        self.allocate_data_continuous()

        self.population_fit = np.full(params.pop_size, -np.inf)
        self.fit_evals = [None] * params.pop_size
        self.fit_best = -np.inf

    def allocate_data_discrete(self):
        p = self.params
        self.population_discrete = np.zeros((p.dim_discrete, p.pop_size), dtype=int)
        self.elites_discrete = np.zeros((p.dim_discrete, p.n_elites), dtype=int)
        self.best_discrete = np.zeros(p.dim_discrete, dtype=int)
        self.probs = p.init_probs.copy()

    def allocate_data_continuous(self):
        p = self.params
        self.population_continuous = np.zeros((p.dim_continuous, p.pop_size))
        self.elites_continuous = np.zeros((p.dim_continuous, p.n_elites))
        self.best_continuous = np.full(p.dim_continuous, -np.inf)
        self.mu = np.copy(p.init_mu_continuous)
        self.std_devs = np.copy(p.init_std_continuous)

    def generate_population(self):
        self.generate_population_discrete()
        self.generate_population_continuous()

    def evaluate_population(self, population_fit):
        # Store calculated fitness score
        self.population_fit = np.copy(population_fit)

        # Update global best
        for i in range(self.params.pop_size):
            if self.population_fit[i] > self.fit_best:
                self.fit_best = np.copy(self.population_fit[i])
                self.best_discrete = np.copy(self.population_discrete[:, i])
                self.best_continuous = np.copy(self.population_continuous[:, i])

    def update_distributions(self):
        self.update_distribution_discrete()
        self.update_distribution_continuous()

        self.log.iterations += 1
        self.log.func_evals += self.params.pop_size
        self.log.best_discrete = np.copy(self.best_discrete)
        self.log.best_continuous = np.copy(self.best_continuous)
        self.log.best_value = np.copy(self.fit_best)

    def generate_population_discrete(self) -> None:
        # Generate random gaussian values from pure Normal distribution (mean=0, std=1)
        for i in range(self.params.pop_size):
            for j in range(self.params.dim_discrete):
                p = self.rng.random()
                s = 0.0
                for k in range(self.params.n_values[j]):
                    s += self.probs[j][k]
                    if p < s:
                        break
                self.population_discrete[j, i] = k

    def generate_population_continuous(self, inject_mean_to_population=False) -> None:
        p = self.params
        # Classic generation of population
        for i in range(p.pop_size):
            for j in range(p.dim_continuous):
                self.population_continuous[j, i] = self.mu[j] + (
                    self.rng.standard_normal() * self.std_devs[j]
                )

        # Clamp inside min/max
        for i in range(p.pop_size):
            for j in range(p.dim_continuous):
                self.population_continuous[j, i] = max(
                    p.min_value_continuous[j],
                    min(p.max_value_continuous[j], self.population_continuous[j, i]),
                )

        # Insert elites from previous inner iteration
        if self.log.iterations > 0:
            for i in range(self.elites_reuse_size):
                self.population_continuous[:, i] = self.elites_continuous[:, i]
        if inject_mean_to_population:
            self.population_continuous[:, :elites_reuse_size] = self.mu

    def update_distribution_discrete(self):
        p = self.params
        # Sort individuals by their perfomance (best first!)
        idx = np.argsort(self.population_fit)[::-1]

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

    def update_distribution_continuous(self):
        p = self.params
        # Sort individuals by their perfomance (best first!)
        idx = np.argsort(self.population_fit)[::-1]

        # Add elites to population
        self.elites_continuous = self.population_continuous[:, idx[: p.n_elites]]

        # Update mean/variance using the elites
        self.mu = self.elites_continuous.mean(axis=1)

        elites_std = self.elites_continuous.std(axis=1)
        for i in range(p.dim_continuous):
            if elites_std[i] > p.min_std_continuous[i]:
                self.std_devs[i] = elites_std[i]
            else:
                self.std_devs[i] = p.min_std_continuous[i]
