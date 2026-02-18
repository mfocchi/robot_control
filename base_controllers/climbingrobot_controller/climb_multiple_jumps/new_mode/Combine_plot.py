"""
Combine_plot.py
───────────────
Load N result folders and overlay / compare them on a single set of plots:
  1. count_jump_histogram           – grouped-bar histogram
  2. plot_fitness_by_iteration      – per-folder best-fitness curve (values >= 1e6 excluded)
  3. plot_mesh_pc_traj_interactive  – all best trajectories on the terrain with
                                      interactive legend (click to show/hide)

Usage
-----
  python Combine_plot.py                         (uses FOLDERS list below)
  FOLDER_PLOT='["result/a","result/b"]' python Combine_plot.py
"""

import json
import os
from typing import Any, List, Optional

import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.patches import Patch
from attr import dataclass

# ──────────────────────────────────────────────
#  CONFIGURATION  –  list of result folders
# ──────────────────────────────────────────────
plot_str = os.environ.get("FOLDER_PLOT")
if plot_str:
    FOLDERS: List[str] = json.loads(plot_str)
    if isinstance(FOLDERS, str):
        FOLDERS = [FOLDERS]
else:
    # ── Default: edit this list to add / remove folders ──
    FOLDERS = [
        "result/test_final_v1_1_gaussian_bumps",
        "result/test_final_v1_2_gaussian_bumps",
        "result/test_final_v1_3_gaussian_bumps",
        "result/test_final_v1_4_gaussian_bumps",
        
    ]

# Where combined plots are saved
COMBINED_OUTPUT = "result/combined_plots"


# ──────────────────────────────────────────────
#  DATA CLASSES  (same as Plot_result.py)
# ──────────────────────────────────────────────
@dataclass
class InnerParams:
    Fleg_max: float
    Fr_max: float
    Fr_min: float
    mass: float
    anchor_distance: float
    fitness_weights: List[float]
    filter_weights: List[float]
    inner_opt_params: Any

@dataclass
class CemParams:
    seed: int
    n_threads: int
    cem_iters: int
    pop_size: int
    n_elites: int
    decrease_pop_factor: float
    fraction_elites_reused: float
    dim_discrete: int
    n_values: int
    init_probs: List[float]
    min_prob: float
    dim_continuous: int
    max_value_continuous: List[float]
    min_value_continuous: List[float]
    init_mu_continuous: List[float]
    init_std_continuous: List[float]
    min_std_continuous: List[float]
    alpha: float

@dataclass
class eliteData:
    fitness: float
    n_jumps: int
    consumed_energy: float
    landing_cost: float
    points: List[List[float]]
    traj: List[List[List[float]]]
    patch_ids: List[int]
    achieved_target: Optional[List[float]]


# ──────────────────────────────────────────────
#  SINGLE-FOLDER LOADER
# ──────────────────────────────────────────────
class SingleResult:
    """Loads one result folder (light version of PlotResultCemMjumps)."""

    def __init__(self, folder: str):
        self.folder = folder
        self.label = os.path.basename(folder.rstrip("/"))

        # Paths
        file_params  = f"{folder}/simulation_params.json"
        file_points  = f"{folder}/actual_point_terrain.json"
        iter_folder  = f"{folder}/iteration_reports"

        # ── params ──
        with open(file_params, "r") as f:
            dp = json.load(f)
        self.p0 = dp["START"]
        self.pf = dp["GOAL"]
        self.n_jumps = dp["MAX_JUMP"]

        # ── terrain ──
        with open(file_points, "r") as f:
            dtp = json.load(f)
        self.points_t_data = dtp["points"]

        # ── iteration history ──
        self.best_fit_ever = None
        self.best_energy_ever = None
        self.best_land_cost_ever = None
        self.best_traj_ever = []
        self.best_achieved_target_ever = None
        self.best_fit_each_iter: List[float] = []
        self.all_elites: List[List[eliteData]] = []

        iter_files = sorted(
            [f for f in os.listdir(iter_folder)
             if f.startswith("iteration_") and f.endswith(".json")],
            key=lambda x: int(x.split("_")[1].split(".")[0]),
        )

        for fname in iter_files:
            with open(os.path.join(iter_folder, fname), "r") as f:
                it = json.load(f)
            self.best_fit_each_iter.append(it["best_fitness_this_iter"])
            elites = []
            for e in it["elites"]:
                elites.append(eliteData(
                    fitness=e["fitness"],
                    n_jumps=e["n_jumps"],
                    consumed_energy=e["consumed_energy"],
                    landing_cost=e["landing_cost"],
                    points=e["points"],
                    traj=e["traj"],
                    patch_ids=e["patch_ids"],
                    achieved_target=e.get("achieved_target", None),
                ))
            self.all_elites.append(elites)
            self.best_fit_ever = it["best_fitness_ever"]
            self.best_energy_ever = it["best_consumed_energy_ever"]
            self.best_land_cost_ever = it["best_landing_cost_ever"]
            self.best_traj_ever = it["best_trajectory_ever"]
            self.best_achieved_target_ever = it.get("best_achieved_target_ever", None)

        # Project start / goal onto terrain
        self.p0 = self._project(self.p0)
        self.pf = self._project(self.pf)

    def _project(self, point):
        ty, tz = point[1], point[2]
        tol_y, tol_z = 0.1, 0.1
        cands = [p["position"] for p in self.points_t_data
                 if abs(p["position"][1] - ty) < tol_y and abs(p["position"][2] - tz) < tol_z]
        if not cands:
            tol_y *= 2; tol_z *= 2
            cands = [p["position"] for p in self.points_t_data
                     if abs(p["position"][1] - ty) < tol_y and abs(p["position"][2] - tz) < tol_z]
        if not cands:
            return point
        cands = np.array(cands)
        return cands[np.argmin(np.abs(cands[:, 0] - point[0]))].tolist()


# ──────────────────────────────────────────────
#  COMBINED PLOTTER
# ──────────────────────────────────────────────
class CombinePlot:

    def __init__(self, folders: List[str]):
        print(f"[INFO] Loading {len(folders)} result folder(s) …")
        self.results: List[SingleResult] = []
        for fld in folders:
            print(f"  -> {fld}")
            self.results.append(SingleResult(fld))
        self.n = len(self.results)
        self.cmap = cm.get_cmap("tab10") if self.n <= 10 else cm.get_cmap("tab20")
        os.makedirs(COMBINED_OUTPUT, exist_ok=True)
        print(f"[INFO] All folders loaded ({self.n} results).\n")

    # ──────────────────────────────
    # 1. JUMP COUNT HISTOGRAM
    # ──────────────────────────────
    def count_jump_histogram(self, ax=None, use_last_iter_only=False):
        """Grouped bar histogram – one colour per folder."""

        all_jump_data = []
        for r in self.results:
            if use_last_iter_only:
                jd = [e.n_jumps for e in r.all_elites[-1]]
            else:
                jd = [e.n_jumps for it in r.all_elites for e in it]
            all_jump_data.append(jd)

        # Global range of jump values
        global_min = min(min(jd) for jd in all_jump_data if jd)
        global_max = max(max(jd) for jd in all_jump_data if jd)
        jump_values = list(range(int(global_min), int(global_max) + 1))

        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 6))
            created = True
        else:
            fig = ax.get_figure()

        bar_width = 0.8 / self.n
        x_base = np.arange(len(jump_values))

        for k, (jd, r) in enumerate(zip(all_jump_data, self.results)):
            counts = [jd.count(v) for v in jump_values]
            offset = (k - self.n / 2 + 0.5) * bar_width
            color_k = self.cmap(k % 20)
            ax.bar(x_base + offset, counts, width=bar_width,
                   color=color_k, edgecolor="black", alpha=0.8,
                   label=r.label)

        title_suf = "(Last Iteration)" if use_last_iter_only else "(All Iterations)"
        ax.set_title(f"Jump Count Distribution – Combined {title_suf}", fontsize=14)
        ax.set_xlabel("Number of Jumps", fontsize=12)
        ax.set_ylabel("Frequency (Elites)", fontsize=12)
        ax.set_xticks(x_base)
        ax.set_xticklabels(jump_values)
        ax.legend(fontsize=9, framealpha=0.9)
        ax.grid(axis="y", linestyle="--", alpha=0.5)
        fig.tight_layout()

        if created:
            sp = os.path.join(COMBINED_OUTPUT, "combined_jump_histogram.png")
            fig.savefig(sp, dpi=150, bbox_inches="tight")
            sp_pdf = os.path.join(COMBINED_OUTPUT, "combined_jump_histogram.pdf")
            fig.savefig(sp_pdf, bbox_inches="tight", pad_inches=0.23)
            print(f"[SAVE] {sp}")
            plt.show()

    # ──────────────────────────────
    # 2. FITNESS BY ITERATION (curve, ignoring >= 1e6)
    # ──────────────────────────────
    def plot_fitness_by_iteration(self, ax=None):
        """
        One curve per folder showing the best fitness found up to each
        iteration.  Values >= 1e6 are excluded (treated as infeasible).
        """
        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(12, 7))
            created = True
        else:
            fig = ax.get_figure()

        FITNESS_CUTOFF = 1e6

        for k, r in enumerate(self.results):
            color_k = self.cmap(k % 20)

            # Build the "running-best" curve, filtering out values >= 1e6
            running_best = []
            current_best = None
            for i, iter_elites in enumerate(r.all_elites):
                # Best fitness in this iteration (excluding infeasible)
                valid = [e.fitness for e in iter_elites if e.fitness < FITNESS_CUTOFF]
                if valid:
                    iter_best = min(valid)
                    if current_best is None or iter_best < current_best:
                        current_best = iter_best
                running_best.append(current_best)

            if not running_best or current_best is None:
                print(f"[WARNING] No feasible fitness (<{FITNESS_CUTOFF}) in {r.label}")
                continue

            iters = np.arange(1, len(running_best) + 1)
            ax.plot(iters, running_best, color=color_k, linewidth=2.2,
                    marker="o", markersize=4, markeredgecolor="black",
                    markeredgewidth=0.4, label=f"{r.label}  (best={current_best:.4f})")

            # Light scatter of all valid elite fitness values per iteration
            for i, iter_elites in enumerate(r.all_elites):
                vals = [e.fitness for e in iter_elites if e.fitness < FITNESS_CUTOFF]
                if vals:
                    ax.scatter([i + 1] * len(vals), vals,
                               c=[color_k], s=12, alpha=0.25, edgecolors="none", zorder=2)

        ax.set_title("Best-So-Far Fitness Convergence (values < 1e6)", fontsize=14, fontweight="bold")
        ax.set_xlabel("Iteration", fontsize=12)
        ax.set_ylabel("Fitness", fontsize=12)
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.legend(fontsize=10, frameon=True, fancybox=True, framealpha=0.9)

        # Integer x-ticks when few iterations
        max_iter = max(len(r.all_elites) for r in self.results)
        if max_iter <= 25:
            ax.set_xticks(range(1, max_iter + 1))
        else:
            ax.xaxis.get_major_locator().set_params(integer=True)

        fig.tight_layout()

        if created:
            sp = os.path.join(COMBINED_OUTPUT, "combined_fitness_convergence.png")
            fig.savefig(sp, dpi=150, bbox_inches="tight")
            sp_pdf = os.path.join(COMBINED_OUTPUT, "combined_fitness_convergence.pdf")
            fig.savefig(sp_pdf, bbox_inches="tight", pad_inches=0.23)
            print(f"[SAVE] {sp}")
            plt.show()

    # ──────────────────────────────
    # 3. 3-D TRAJECTORIES (interactive legend)
    # ──────────────────────────────
    def plot_mesh_pc_traj_interactive(self, ax=None, show_cost=True):
        """
        Plot terrain + best trajectory from each folder.
        Each folder's trajectory can be toggled via the interactive legend
        (same pattern as Plot_time.py).
        """
        created = False
        if ax is None:
            fig = plt.figure(figsize=(16, 10))
            ax = fig.add_subplot(111, projection="3d")
            created = True
        else:
            fig = ax.get_figure()

        # ── Terrain (use first folder's point cloud) ──
        ref = self.results[0]
        px = np.array([p["position"][0] for p in ref.points_t_data])
        py = np.array([p["position"][1] for p in ref.points_t_data])
        pz = np.array([p["position"][2] for p in ref.points_t_data])

        if show_cost:
            costs = np.array([p["cost"] for p in ref.points_t_data])
            sc_t = ax.scatter(px, py, pz, c=costs, cmap="RdYlGn_r", s=1, alpha=1, zorder=1)
            cbar = fig.colorbar(sc_t, ax=ax, pad=0.1, shrink=0.6)
            cbar.set_label("Terrain Point Cost", rotation=270, labelpad=15)
        else:
            ax.scatter(px, py, pz, c="gray", s=1, alpha=0.3, zorder=1)

        all_x, all_y, all_z = [px], [py], [pz]

        # ── Per-folder trajectory artists ──
        traj_artists = []       # list of lists of artists for each folder
        legend_elements = []
        legend_map = {}         # id(Patch) -> list_of_artists

        for k, r in enumerate(self.results):
            color_k = self.cmap(k % 20)

            if not r.best_traj_ever:
                print(f"[WARNING] No trajectory for {r.label}")
                continue

            folder_artists = []

            # Draw trajectory segments
            for i, segment in enumerate(r.best_traj_ever):
                seg = np.array(segment)
                if seg.shape[0] == 3 and seg.shape[1] != 3:
                    xs, ys, zs = seg[0], seg[1], seg[2]
                else:
                    xs, ys, zs = seg[:, 0], seg[:, 1], seg[:, 2]
                line, = ax.plot(xs, ys, zs, color=color_k, linewidth=2.5, zorder=10)
                folder_artists.append(line)
                all_x.append(xs); all_y.append(ys); all_z.append(zs)

            # Landing points
            landing = []
            for i, segment in enumerate(r.best_traj_ever):
                seg = np.array(segment)
                if seg.shape[0] == 3 and seg.shape[1] != 3:
                    xs, ys, zs = seg[0], seg[1], seg[2]
                else:
                    xs, ys, zs = seg[:, 0], seg[:, 1], seg[:, 2]
                landing.append([xs[0], ys[0], zs[0]])
                if i == len(r.best_traj_ever) - 1:
                    landing.append([xs[-1], ys[-1], zs[-1]])
            lp = np.array(landing)
            sc_lp = ax.scatter(lp[:, 0], lp[:, 1], lp[:, 2],
                               c=[color_k], s=30, edgecolors="black",
                               linewidths=0.5, zorder=11)
            folder_artists.append(sc_lp)

            # Start marker
            sc_s = ax.scatter(r.p0[0], r.p0[1], r.p0[2],
                              c=[color_k], s=160, marker="^",
                              edgecolors="black", linewidths=1.2, zorder=15)
            folder_artists.append(sc_s)

            # Goal marker
            sc_g = ax.scatter(r.pf[0], r.pf[1], r.pf[2],
                              c="red", s=160, marker="X",
                              edgecolors="black", linewidths=1.2, zorder=15)
            folder_artists.append(sc_g)

            # Achieved target
            if r.best_achieved_target_ever:
                ach = np.array(r.best_achieved_target_ever).flatten()
                sc_a = ax.scatter(ach[0], ach[1], ach[2],
                                  c="orange", s=160, marker="D",
                                  edgecolors="black", linewidths=1.2, zorder=16)
                folder_artists.append(sc_a)
                line_err, = ax.plot([r.pf[0], ach[0]], [r.pf[1], ach[1]], [r.pf[2], ach[2]],
                                    "--", color=color_k, linewidth=1.5, alpha=0.6, zorder=14)
                folder_artists.append(line_err)

            traj_artists.append(folder_artists)

            # Legend patch
            fit_str = f"{r.best_fit_ever:.4f}" if r.best_fit_ever is not None else "N/A"
            patch = Patch(facecolor=color_k, alpha=0.85, edgecolor="black", linewidth=1,
                          label=f"{r.label}  (fit={fit_str})")
            legend_elements.append(patch)
            legend_map[id(patch)] = folder_artists

        # ── Scaling 1:1:1 ──
        fx = np.concatenate(all_x)
        fy = np.concatenate(all_y)
        fz = np.concatenate(all_z)
        mr = np.array([fx.ptp(), fy.ptp(), fz.ptp()]).max() / 2.0
        mx, my, mz = (fx.max() + fx.min()) * 0.5, (fy.max() + fy.min()) * 0.5, (fz.max() + fz.min()) * 0.5
        ax.set_xlim(mx - mr, mx + mr)
        ax.set_ylim(my - mr, my + mr)
        ax.set_zlim(mz - mr, mz + mr)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("Combined Best Trajectories\n(Click legend to show/hide)",
                      fontsize=14, fontweight="bold")
        ax.view_init(elev=30, azim=-60)

        # ── Interactive legend ──
        leg = ax.legend(handles=legend_elements, loc="upper left",
                        bbox_to_anchor=(1.05, 1), fontsize=9,
                        framealpha=0.9, edgecolor="black",
                        title="Click to show/hide", title_fontsize=10,
                        fancybox=True, shadow=True)

        patch_to_data = {}
        for lg_patch, lg_text, le in zip(leg.get_patches(), leg.get_texts(), legend_elements):
            lg_patch.set_picker(True)
            lg_text.set_picker(True)
            artists = legend_map[id(le)]
            patch_to_data[id(lg_patch)] = (artists, lg_patch, lg_text)
            patch_to_data[id(lg_text)] = (artists, lg_patch, lg_text)

        def on_pick(event):
            key = id(event.artist)
            if key not in patch_to_data:
                return
            arts, lp, lt = patch_to_data[key]
            vis = not arts[0].get_visible()
            for a in arts:
                a.set_visible(vis)
            if vis:
                lp.set_alpha(0.85)
                lt.set_alpha(1.0)
            else:
                lp.set_alpha(0.15)
                lt.set_alpha(0.3)
            fig.canvas.draw_idle()

        fig.canvas.mpl_connect("pick_event", on_pick)

        fig.tight_layout()

        if created:
            sp = os.path.join(COMBINED_OUTPUT, "combined_mesh_pc_traj.png")
            fig.savefig(sp, dpi=150, bbox_inches="tight")
            sp_pdf = os.path.join(COMBINED_OUTPUT, "combined_mesh_pc_traj.pdf")
            fig.savefig(sp_pdf, bbox_inches="tight", pad_inches=0.23)
            print(f"[SAVE] {sp}")
            plt.show()


# ──────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────
def main():
    cp = CombinePlot(FOLDERS)

    cp.count_jump_histogram(use_last_iter_only=False)
    cp.plot_fitness_by_iteration()
    cp.plot_mesh_pc_traj_interactive(show_cost=True)

    print("[INFO] All combined plots completed!")


if __name__ == "__main__":
    main()

