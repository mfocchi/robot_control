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
        "result/common_1",
        "result/common_2",
        "result/common_3"
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

        # Lazy-loaded convergence data
        self.all_comb_data: List[dict] = []

    def load_all_comb_history(self):
        """Load all_comb_in_iter_X_report.json files from the iteration_reports folder."""
        if self.all_comb_data:
            return  # already loaded

        iter_folder = f"{self.folder}/iteration_reports"
        if not os.path.exists(iter_folder):
            print(f"[WARNING] Iter folder not found: {iter_folder}")
            return

        files = [f for f in os.listdir(iter_folder)
                 if f.startswith("all_comb_in_iter_") and f.endswith(".json")]
        if not files:
            print(f"[WARNING] No all_comb_in_iter_*.json files in {iter_folder}")
            return

        def _extract_num(fname):
            try:
                return int(fname.split("all_comb_in_iter_")[1].split("_report")[0])
            except Exception:
                return -1

        files.sort(key=_extract_num)

        for fname in files:
            fpath = os.path.join(iter_folder, fname)
            try:
                with open(fpath, "r") as f:
                    self.all_comb_data.append(json.load(f))
            except Exception as e:
                print(f"[WARNING] Could not load {fpath}: {e}")

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
    # 2. FITNESS BY ITERATION
    # ──────────────────────────────
    def plot_fitness_by_iteration(self, ax=None):
        """
        Scatter plot of elite fitness values across iterations for every folder.
        Only elites with fitness > -1e5 are shown.
        Each folder uses its own colour; best-ever points are highlighted in red
        and a dashed horizontal line marks each folder's best fitness.
        """
        from matplotlib.colors import Normalize, LinearSegmentedColormap

        fitness_threshold = -1e4
        tolerance = 1e-8

        # Check at least one folder has data
        if not any(r.all_elites for r in self.results):
            print("[WARNING] No elite data available for plot_fitness_by_iteration.")
            return

        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(14, 7))
            created = True
        else:
            fig = ax.get_figure()

        legend_elements = []

        for k, r in enumerate(self.results):
            if not r.all_elites:
                continue

            color_k = self.cmap(k % 20)

            x_normal, y_normal = [], []
            x_best,   y_best   = [], []

            for i, iteration_list in enumerate(r.all_elites):
                current_iter = i + 1
                for elite in iteration_list:
                    if elite.fitness < fitness_threshold:
                        continue
                    if np.isclose(elite.fitness, r.best_fit_ever, atol=tolerance):
                        x_best.append(current_iter)
                        y_best.append(elite.fitness)
                    else:
                        x_normal.append(current_iter)
                        y_normal.append(elite.fitness)

            if not x_normal and not x_best:
                continue

            # Normal elites: semi-transparent scatter with folder colour
            ax.scatter(x_normal, y_normal,
                       color=color_k, s=30,
                       edgecolors='black', linewidths=0.3,
                       alpha=0.6, zorder=3)

            # Best-ever elites: same colour but fully opaque + red edge
            if x_best:
                ax.scatter(x_best, y_best,
                           color=color_k, s=60,
                           edgecolors='red', linewidths=1.2,
                           alpha=1.0, zorder=4, marker='*')

            # Horizontal dashed line at best_fit_ever
            if r.best_fit_ever is not None:
                ax.axhline(y=r.best_fit_ever,
                           color=color_k, linestyle='--',
                           linewidth=1.2, alpha=0.7, zorder=2)

            fit_str = f"{r.best_fit_ever:.4f}" if r.best_fit_ever is not None else "N/A"
            legend_elements.append(
                Patch(facecolor=color_k, edgecolor='black', alpha=0.85,
                      label=f"{r.label}  (best={fit_str})")
            )

        # x-ticks: show every integer up to 25, then auto
        max_iter = max((len(r.all_elites) for r in self.results if r.all_elites), default=0)
        if max_iter <= 25:
            ax.set_xticks(range(1, max_iter + 1))
        else:
            ax.xaxis.get_major_locator().set_params(integer=True)

        ax.set_title('Elite Fitness by Iteration – Combined', fontsize=14, fontweight='bold')
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Fitness Value', fontsize=12)
        ax.grid(True, linestyle=':', alpha=0.5, zorder=0)
        ax.legend(handles=legend_elements, fontsize=9,
                  frameon=True, fancybox=True, framealpha=0.9,
                  loc='lower right')

        fig.tight_layout()

        if created:
            sp = os.path.join(COMBINED_OUTPUT, "combined_fitness_by_iteration.png")
            fig.savefig(sp, dpi=150, bbox_inches="tight")
            sp_pdf = os.path.join(COMBINED_OUTPUT, "combined_fitness_by_iteration.pdf")
            fig.savefig(sp_pdf, bbox_inches="tight", pad_inches=0.23)
            print(f"[SAVE] {sp}")
            plt.show()

    # ──────────────────────────────
    # 2b. BEST FITNESS LINE PER ITERATION
    # ──────────────────────────────
    def plot_best_fitness_line(self, ax=None):
        """
        For each folder, plot a line that connects the best (maximum) fitness
        value found among the elites at each iteration.
        Only elites with fitness > -1e5 are considered.
        One line per folder, colour-coded.
        """
        fitness_threshold = -1e5

        if not any(r.all_elites for r in self.results):
            print("[WARNING] No elite data available for plot_best_fitness_line.")
            return

        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(14, 7))
            created = True
        else:
            fig = ax.get_figure()

        legend_elements = []

        for k, r in enumerate(self.results):
            if not r.all_elites:
                continue

            color_k = self.cmap(k % 20)

            iters, best_per_iter = [], []
            for i, iteration_list in enumerate(r.all_elites):
                values = [
                    elite.fitness
                    for elite in iteration_list
                    if elite.fitness > fitness_threshold
                ]
                if not values:
                    continue
                iters.append(i + 1)
                best_per_iter.append(max(values))

            if not iters:
                continue

            ax.plot(iters, best_per_iter,
                    color=color_k, linewidth=2.2,
                    marker='o', markersize=5,
                    markeredgecolor='black', markeredgewidth=0.5,
                    zorder=3)

            fit_str = f"{r.best_fit_ever:.4f}" if r.best_fit_ever is not None else "N/A"
            legend_elements.append(
                Patch(facecolor=color_k, edgecolor='black', alpha=0.85,
                      label=f"{r.label}  (best={fit_str})")
            )

        max_iter = max((len(r.all_elites) for r in self.results if r.all_elites), default=0)
        if max_iter <= 25:
            ax.set_xticks(range(1, max_iter + 1))
        else:
            ax.xaxis.get_major_locator().set_params(integer=True)

        ax.set_title('Best Elite Fitness per Iteration – Combined', fontsize=14, fontweight='bold')
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Best Fitness Value', fontsize=12)
        ax.grid(True, linestyle=':', alpha=0.5, zorder=0)
        ax.legend(handles=legend_elements, fontsize=9,
                  frameon=True, fancybox=True, framealpha=0.9,
                  loc='lower right')

        fig.tight_layout()

        if created:
            sp = os.path.join(COMBINED_OUTPUT, "combined_best_fitness_line.png")
            fig.savefig(sp, dpi=150, bbox_inches="tight")
            sp_pdf = os.path.join(COMBINED_OUTPUT, "combined_best_fitness_line.pdf")
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

    # ──────────────────────────────
    # 4. CONVERGENCE HISTOGRAM
    # ──────────────────────────────
    def plot_convergence_histogram(self):
        """
        Grouped stacked bar chart – all folders side by side for each iteration
        (same style as count_jump_histogram), plus a convergence-rate line plot.

        Layout:
          - Top panel : grouped stacked bars (Converged / Failed) per iteration,
                        one group-slot per folder (colour-coded, same as other plots)
          - Bottom panel: convergence-rate (%) per iteration, one line per folder
        """
        # Load comb data for every folder
        for r in self.results:
            r.load_all_comb_history()

        # Keep only folders that actually have data
        valid = [r for r in self.results if r.all_comb_data]
        if not valid:
            print("[WARNING] No all_comb_in_iter data found in any folder – skipping plot.")
            return

        n_valid = len(valid)

        # ── Build per-folder dicts: iteration -> (n_conv, n_fail) ──
        all_iter_sets = []
        folder_data = []  # list of dicts {iter: (conv, fail)}
        for r in valid:
            d = {}
            for entry in r.all_comb_data:
                it = entry["iteration"]
                steps = entry.get("steps", [])
                n_conv = sum(1 for s in steps if s.get("converged", False) is True)
                n_fail = len(steps) - n_conv
                d[it] = (n_conv, n_fail)
            folder_data.append(d)
            all_iter_sets.append(set(d.keys()))

        # Global sorted iteration list (union of all folders)
        all_iters = sorted(set().union(*all_iter_sets))
        x_base = np.arange(len(all_iters))
        bar_width = 0.8 / n_valid

        fig, (ax_bars, ax_rate) = plt.subplots(2, 1, figsize=(max(12, len(all_iters) * 1.2), 12))

        # ── Top: grouped stacked bars ──
        for k, (r, d) in enumerate(zip(valid, folder_data)):
            color_k = self.cmap(k % 20)
            offset = (k - n_valid / 2 + 0.5) * bar_width

            convs = [d.get(it, (0, 0))[0] for it in all_iters]
            fails = [d.get(it, (0, 0))[1] for it in all_iters]
            xs = x_base + offset

            ax_bars.bar(xs, convs, bar_width,
                        color=color_k, edgecolor="black", alpha=0.85,
                        label=r.label, hatch="")
            ax_bars.bar(xs, fails, bar_width, bottom=convs,
                        color=color_k, edgecolor="black", alpha=0.35,
                        hatch="///")

            # Convergence % label above each bar
            for x, conv, fail in zip(xs, convs, fails):
                total = conv + fail
                if total > 0:
                    perc = (conv / total) * 100
                    ax_bars.text(x, total + total * 0.015, f"{perc:.0f}%",
                                 ha="center", va="bottom", fontsize=6,
                                 fontweight="bold", color=color_k)

        # Legend: solid patch = converged, hatched = failed
        from matplotlib.patches import Patch as _Patch
        legend_handles = [r2.label and _Patch(facecolor=self.cmap(k2 % 20),
                                              edgecolor="black", alpha=0.85,
                                              label=valid[k2].label)
                          for k2, r2 in enumerate(valid)]
        legend_handles = [_Patch(facecolor=self.cmap(k2 % 20), edgecolor="black",
                                 alpha=0.85, label=valid[k2].label)
                          for k2 in range(n_valid)]
        legend_handles += [
            _Patch(facecolor="white", edgecolor="black", alpha=0.35,
                   hatch="///", label="Hatched = Failed"),
        ]
        ax_bars.set_title("Convergence per Iteration – Combined (Grouped)",
                          fontsize=13, fontweight="bold")
        ax_bars.set_xlabel("Iteration", fontsize=11)
        ax_bars.set_ylabel("Population Count", fontsize=11)
        ax_bars.set_xticks(x_base)
        ax_bars.set_xticklabels(all_iters)
        ax_bars.legend(handles=legend_handles, fontsize=9,
                       framealpha=0.9, loc="upper right")
        ax_bars.grid(axis="y", linestyle="--", alpha=0.5)

        # ── Bottom: convergence-rate line plot ──
        for k, (r, d) in enumerate(zip(valid, folder_data)):
            color_k = self.cmap(k % 20)
            iters = sorted(d.keys())
            rates = [100.0 * d[it][0] / max(d[it][0] + d[it][1], 1) for it in iters]
            ax_rate.plot(iters, rates, marker="o", linewidth=2, markersize=5,
                         markeredgecolor="black", markeredgewidth=0.4,
                         color=color_k, label=r.label)

        ax_rate.set_title("Convergence Rate (%) – All Folders",
                          fontsize=13, fontweight="bold")
        ax_rate.set_xlabel("Iteration", fontsize=11)
        ax_rate.set_ylabel("Convergence Rate (%)", fontsize=11)
        ax_rate.set_ylim(0, 110)
        ax_rate.grid(True, linestyle=":", alpha=0.5)
        ax_rate.legend(fontsize=9, frameon=True, fancybox=True, framealpha=0.9)
        if len(all_iters) <= 25:
            ax_rate.set_xticks(all_iters)

        fig.suptitle("Combined Convergence Histogram", fontsize=15, fontweight="bold")
        fig.tight_layout()

        sp = os.path.join(COMBINED_OUTPUT, "combined_convergence_histogram.png")
        fig.savefig(sp, dpi=150, bbox_inches="tight")
        sp_pdf = os.path.join(COMBINED_OUTPUT, "combined_convergence_histogram.pdf")
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
    cp.plot_best_fitness_line()
    cp.plot_mesh_pc_traj_interactive(show_cost=True)
    cp.plot_convergence_histogram()

    print("[INFO] All combined plots completed!")


if __name__ == "__main__":
    main()

