"""
plot_result_multi_step.py
=========================
Visualizer for multi-step jump optimization results produced by Main_cemmulti_step.py.

Data layout expected under FOLDER_MAIN:
    simulation_params.json
    actual_point_terrain.json
    actual_patch_terrain.json
    jump_1/
        jump_summary.json
        cem_iteration_history.json
        timing_report.json
    jump_2/ ...
    ...

NOTE: global_summary.json is NOT required. n_jumps is auto-detected from jump_N/
      directories; waypoints and global_fitness are derived from the per-jump
      jump_summary.json files.

Usage:
    python plot_result_multi_step.py
    FOLDER_PLOT="result/my_run" python plot_result_multi_step.py
"""

import json
import os
import sys

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import Normalize, LinearSegmentedColormap
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
from scipy.spatial import ConvexHull

# ─── component imports (same as Plot_result.py) ──────────────────────────────
from base_controllers.components.point_cloud_filter import PointCloudFilter
from base_controllers.components.patch_surface import PatchSurface

# ─── result folder ────────────────────────────────────────────────────────────
plot_str = os.environ.get("FOLDER_PLOT")
FOLDER_MAIN = plot_str if plot_str else "result/multi_step_2"

FILE_TERRAIN_POINTS  = os.path.join(FOLDER_MAIN, "actual_point_terrain.json")
FILE_TERRAIN_PATCHES = os.path.join(FOLDER_MAIN, "actual_patch_terrain.json")
FILE_SAVE_PARAMS     = os.path.join(FOLDER_MAIN, "simulation_params.json")


# ─────────────────────────────────────────────────────────────────────────────
def _set_axes_equal_3d(ax):
    """Force 1:1:1 aspect ratio on a 3-D axis."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    mid    = limits.mean(axis=1)
    radius = 0.5 * (limits[:, 1] - limits[:, 0]).max()
    ax.set_xlim3d(mid[0] - radius, mid[0] + radius)
    ax.set_ylim3d(mid[1] - radius, mid[1] + radius)
    ax.set_zlim3d(mid[2] - radius, mid[2] + radius)


def _extract_traj_xyz(traj_list):
    """
    Given a list of trajectory segments (each a 3×N or N×3 nested list),
    return (xs, ys, zs) as flat numpy arrays – one entry per segment start,
    finishing with the last point of the last segment.
    """
    xs, ys, zs = [], [], []
    for seg in traj_list:
        s = np.array(seg)
        if s.ndim == 2 and s.shape[0] == 3:
            xs.append(s[0, 0]); ys.append(s[1, 0]); zs.append(s[2, 0])
        elif s.ndim == 2:
            xs.append(s[0, 0]); ys.append(s[0, 1]); zs.append(s[0, 2])
    if traj_list:
        s = np.array(traj_list[-1])
        if s.shape[0] == 3:
            xs.append(s[0, -1]); ys.append(s[1, -1]); zs.append(s[2, -1])
        else:
            xs.append(s[-1, 0]); ys.append(s[-1, 1]); zs.append(s[-1, 2])
    return np.array(xs), np.array(ys), np.array(zs)


def _traj_segments(traj_list):
    """
    Yield (x_arr, y_arr, z_arr) for each complete segment in traj_list.
    Handles both 3×N and N×3 layouts.
    """
    for seg in traj_list:
        s = np.array(seg)
        if s.ndim == 2 and s.shape[0] == 3:
            yield s[0, :], s[1, :], s[2, :]
        elif s.ndim == 2:
            yield s[:, 0], s[:, 1], s[:, 2]


# ─────────────────────────────────────────────────────────────────────────────
class PlotResultMultiStep:
    """
    Plotter for multi-step jump data saved by Main_cemmulti_step.py.
    """

    # ── construction / loading ────────────────────────────────────────────────
    def __init__(self):
        print(f"[INFO] Loading data from: {os.path.abspath(FOLDER_MAIN)}")

        # ── auto-detect number of jumps from jump_N/ directories ─────────────
        j = 1
        while os.path.isdir(os.path.join(FOLDER_MAIN, f"jump_{j}")):
            j += 1
        self.n_jumps = j - 1
        if self.n_jumps == 0:
            raise FileNotFoundError(f"No jump_N/ folders found in {FOLDER_MAIN}")
        print(f"[INFO] Found {self.n_jumps} jump folder(s).")

        # per-jump data
        self.jump_summaries   = []   # jump_summary.json  for each jump
        self.cem_histories    = []   # cem_iteration_history.json list for each jump
        self.timing_reports   = []   # timing_report.json for each jump

        for j in range(1, self.n_jumps + 1):
            jdir = os.path.join(FOLDER_MAIN, f"jump_{j}")
            self.jump_summaries.append(self._load_json(os.path.join(jdir, "jump_summary.json")))
            self.cem_histories.append(self._load_json(os.path.join(jdir, "cem_iteration_history.json")))
            timing_path = os.path.join(jdir, "timing_report.json")
            self.timing_reports.append(self._load_json(timing_path) if os.path.exists(timing_path) else {})

        # ── derive global metadata from per-jump summaries ────────────────────
        self.waypoints = (
            [np.array(js["p0"]) for js in self.jump_summaries]
            + [np.array(self.jump_summaries[-1]["pf"])]
        )
        self.global_fitness  = sum(js["best_fitness"] for js in self.jump_summaries)
        self.all_best_points = [js.get("best_points", []) for js in self.jump_summaries]

        # terrain – shared across all jumps
        self.data_terrain_points  = self._load_json(FILE_TERRAIN_POINTS)
        self.data_terrain_patches = self._load_json(FILE_TERRAIN_PATCHES)
        self.points_t_data = self.data_terrain_points["points"]
        self.patches       = self.data_terrain_patches["patches"]
        self.mesh_bounds   = self.data_terrain_points.get("mesh_bounds", None)

        self.pc_plotter = PointCloudFilter(pc=[p["position"] for p in self.points_t_data])

        pw = self.data_terrain_patches["metadata"]["patch_width"]
        ph = self.data_terrain_patches["metadata"]["patch_height"]
        self.patch_plotter = PatchSurface(
            points_t=self.points_t_data,
            number_of_patches_width=int(pw),
            number_of_patches_height=int(ph),
        )

        # ── patch physical size in Y and Z (from simulation_params.json) ─────
        self.patch_side_y = 1.0   # default fallback
        self.patch_side_z = 1.0
        if os.path.exists(FILE_SAVE_PARAMS):
            try:
                sp = self._load_json(FILE_SAVE_PARAMS)
                iop = sp.get("inner_opt_params_order", {}).get("inner_opt_params", {})
                self.patch_side_y = float(iop.get("patch_side_y", 1.0))
                self.patch_side_z = float(iop.get("patch_side_z", 1.0))
            except Exception:
                pass
        print(f"[INFO] Patch size: Y={self.patch_side_y}  Z={self.patch_side_z}")

        print(f"[INFO] Loaded {self.n_jumps} jumps,  "
              f"{len(self.points_t_data)} terrain points,  "
              f"{len(self.patches)} patches.")

    # ── helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _load_json(path):
        with open(path, "r") as f:
            return json.load(f)

    def _terrain_xyz(self):
        px = np.array([p["position"][0] for p in self.points_t_data])
        py = np.array([p["position"][1] for p in self.points_t_data])
        pz = np.array([p["position"][2] for p in self.points_t_data])
        costs = np.array([p["cost"] for p in self.points_t_data])
        return px, py, pz, costs

    def _draw_terrain_3d(self, ax, alpha=0.5, s=1):
        px, py, pz, costs = self._terrain_xyz()
        ax.scatter(px, py, pz, c=costs, cmap="RdYlGn_r", s=s, alpha=alpha,
                   zorder=1, label="Terrain")

    def _draw_terrain_2d(self, ax, alpha=0.5, s=1):
        _, py, pz, costs = self._terrain_xyz()
        ax.scatter(py, pz, c=costs, cmap="RdYlGn_r", s=s, alpha=alpha,
                   zorder=1, label="Terrain")

    def _draw_patches_2d(self, ax, norm_cost=None, cmap_terrain=None):
        costs = [p.get("cost_patch", 0.0) for p in self.patches]
        if norm_cost is None:
            norm_cost = Normalize(vmin=min(costs) if costs else 0,
                                  vmax=max(costs) if costs else 1)
        if cmap_terrain is None:
            cmap_terrain = plt.get_cmap("RdYlGn_r")

        half_y = self.patch_side_y / 2.0
        half_z = self.patch_side_z / 2.0

        for patch in self.patches:
            pts_in = patch.get("points_in_patch", [])
            if not pts_in:
                continue
            coords = np.array([p["position"] for p in pts_in])
            cy = coords[:, 1].mean()
            cz = coords[:, 2].mean()
            rect = plt.Rectangle(
                (cy - half_y, cz - half_z),
                self.patch_side_y, self.patch_side_z,
                color=cmap_terrain(norm_cost(patch.get("cost_patch", 0.0))),
                alpha=0.45, ec="black", lw=0.5, zorder=1,
            )
            ax.add_patch(rect)

    def _jump_color(self, jump_idx):
        """Return a distinct colour for jump jump_idx (0-based)."""
        cmap = plt.get_cmap("tab10")
        return cmap(jump_idx % 10)

    @staticmethod
    def _disp_fit(v):
        """Negate stored (negative) fitness to get a positive display value."""
        return -v

    def _save(self, fig, name):
        for ext in ("png", "pdf"):
            path = os.path.join(FOLDER_MAIN, f"{name}.{ext}")
            fig.savefig(path, dpi=150, bbox_inches="tight", pad_inches=0.2)
            print(f"[SAVE] {path}")

    # ══════════════════════════════════════════════════════════════════════════
    # 1.  Global trajectory – 3-D and 2-D side by side in one window
    # ══════════════════════════════════════════════════════════════════════════
    def plot_global_3d_trajectory(self, elev=25, azim=-55):
        """Genera due finestre separate: una per il 3D e una per il 2D (YZ), senza titoli né legende."""
        
        # ── FINESTRA 1: 3-D Trajectory ──────────────────────────────────────────
        fig3d = plt.figure(figsize=(12, 10))
        ax3d = fig3d.add_subplot(1, 1, 1, projection="3d")
        
        self._draw_terrain_3d(ax3d, alpha=0.4, s=1)

        for j_idx, js in enumerate(self.jump_summaries):
            color  = self._jump_color(j_idx)
            traj   = js.get("best_trajectory", [])
            points = js.get("best_points", [])

            # Disegno traiettoria
            for x_s, y_s, z_s in _traj_segments(traj):
                ax3d.plot(x_s, y_s, z_s, color=color, linewidth=2.5, zorder=6)

            # Punti intermedi
            if points:
                pts = np.array(points)
                ax3d.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                            color=color, s=60, edgecolors="black",
                            linewidths=0.8, zorder=8)

            # Target raggiunto
            at = js.get("best_achieved_target")
            if at:
                at_np = np.array(at).flatten()
                ax3d.scatter(at_np[0], at_np[1], at_np[2], c="orange",
                            s=120, marker="D", edgecolors="black",
                            linewidths=1.2, zorder=10)

            # Waypoints Inizio/Fine
            p0, pf = np.array(js["p0"]), np.array(js["pf"])
            ax3d.scatter(p0[0], p0[1], p0[2], c="lime",  s=160, marker="^", 
                        edgecolors="black", linewidths=1.2, zorder=12)
            ax3d.scatter(pf[0], pf[1], pf[2], c="red",   s=160, marker="X", 
                        edgecolors="black", linewidths=1.2, zorder=12)

        ax3d.set_xlabel("X (m)", fontsize=12)
        ax3d.set_ylabel("Y (m)", fontsize=12)
        ax3d.set_zlabel("Z (m)", fontsize=12)
        ax3d.view_init(elev=elev, azim=azim)
        _set_axes_equal_3d(ax3d)
        
        self._save(fig3d, "ms_global_3d_only")

        # ── FINESTRA 2: 2-D Trajectory (YZ) ─────────────────────────────────────
        fig2d = plt.figure(figsize=(12, 10))
        ax2d = fig2d.add_subplot(1, 1, 1)
        ax2d.set_aspect("equal", adjustable="box")
        
        self._draw_patches_2d(ax2d)

        for j_idx, js in enumerate(self.jump_summaries):
            color  = self._jump_color(j_idx)
            traj   = js.get("best_trajectory", [])
            points = js.get("best_points",     [])
            p0, pf = np.array(js["p0"]), np.array(js["pf"])

            # Disegno traiettoria YZ
            for _, y_s, z_s in _traj_segments(traj):
                ax2d.plot(y_s, z_s, color=color, linewidth=2.0, zorder=6)

            # Punti intermedi
            if points:
                pts = np.array(points)
                ax2d.scatter(pts[:, 1], pts[:, 2], color=color, s=55,
                            edgecolors="black", linewidths=0.6, zorder=8)

            # Waypoints
            ax2d.scatter(p0[1], p0[2], c="lime", s=160, marker="^",
                        edgecolors="black", linewidths=1.5, zorder=12)
            ax2d.scatter(pf[1], pf[2], c="red",  s=160, marker="X",
                        edgecolors="black", linewidths=1.5, zorder=12)

            # Target raggiunto e linea di errore
            at = js.get("best_achieved_target")
            if at:
                at_np = np.array(at).flatten()
                ax2d.scatter(at_np[1], at_np[2], c="orange", s=120, marker="D",
                            edgecolors="black", linewidths=1.2, zorder=10)
                ax2d.plot([pf[1], at_np[1]], [pf[2], at_np[2]],
                        "--", color=color, linewidth=1.2, alpha=0.6, zorder=9)

        ax2d.set_xlabel("Y (m)", fontsize=12)
        ax2d.set_ylabel("Z (m)", fontsize=12)
        ax2d.grid(True, linestyle="--", alpha=0.4)

        self._save(fig2d, "ms_global_2d_only")

        # Mostra entrambe le finestre
        plt.show()
    # ══════════════════════════════════════════════════════════════════════════
    # 2.  Global 2-D trajectory – YZ plane
    # ══════════════════════════════════════════════════════════════════════════
    def plot_global_2d_trajectory(self):
        """YZ-plane view of the best trajectory for every jump with patch cost map."""
        fig, ax = plt.subplots(figsize=(14, 10))
        ax.set_aspect("equal", adjustable="box")

        self._draw_patches_2d(ax)

        for j_idx, js in enumerate(self.jump_summaries):
            color  = self._jump_color(j_idx)
            traj   = js.get("best_trajectory", [])
            points = js.get("best_points",     [])
            p0     = np.array(js["p0"])
            pf     = np.array(js["pf"])

            # trajectory
            for x_s, y_s, z_s in _traj_segments(traj):
                ax.plot(y_s, z_s, color=color, linewidth=2.0, zorder=6)

            # contact points
            if points:
                pts = np.array(points)
                ax.scatter(pts[:, 1], pts[:, 2], color=color, s=55,
                           edgecolors="black", linewidths=0.6, zorder=8)

            # waypoints
            ax.scatter(p0[1], p0[2], c="lime", s=160, marker="^",
                       edgecolors="black", linewidths=1.5, zorder=12)
            ax.scatter(pf[1], pf[2], c="red",  s=160, marker="X",
                       edgecolors="black", linewidths=1.5, zorder=12)

            # achieved target
            at = js.get("best_achieved_target")
            if at:
                at = np.array(at).flatten()
                ax.scatter(at[1], at[2], c="orange", s=120, marker="D",
                           edgecolors="black", linewidths=1.2, zorder=10)
                ax.plot([pf[1], at[1]], [pf[2], at[2]],
                        "--", color=color, linewidth=1.2, alpha=0.6, zorder=9)

            # jump label in the middle of the path
            if points and len(points) >= 2:
                mid = np.mean(np.array(points), axis=0)
                ax.text(mid[1], mid[2], f"J{j_idx+1}", fontsize=9,
                        ha="center", va="center", color="white",
                        bbox=dict(boxstyle="round,pad=0.2", fc=color, alpha=0.8))

        legend_handles = [
            mpatches.Patch(color=self._jump_color(j),
                           label=f"Jump {j+1}  (fit={self._disp_fit(self.jump_summaries[j]['best_fitness']):.1f})")
            for j in range(self.n_jumps)
        ] + [
            plt.Line2D([0], [0], marker="^", color="w",
                       markerfacecolor="lime", markersize=10, label="Waypoint start"),
            plt.Line2D([0], [0], marker="X", color="w",
                       markerfacecolor="red", markersize=10, label="Waypoint goal"),
            plt.Line2D([0], [0], marker="D", color="w",
                       markerfacecolor="orange", markersize=9, label="Achieved target"),
        ]
        ax.legend(handles=legend_handles, loc="upper left",
                  bbox_to_anchor=(1.01, 1), fontsize=9, borderaxespad=0)

        ax.set_title(f"Global Multi-Step Trajectory  –  2-D (YZ plane)\n"
                     f"{self.n_jumps} jumps  |  Total Fitness: {self._disp_fit(self.global_fitness):.4f}",
                     fontsize=13, fontweight="bold")
        ax.set_xlabel("Y (m)"); ax.set_ylabel("Z (m)")
        ax.grid(True, linestyle="--", alpha=0.4)

        fig.tight_layout()
        self._save(fig, "ms_global_2d_trajectory")
        plt.show()

    # ══════════════════════════════════════════════════════════════════════════
    # 3.  Per-jump CEM convergence
    # ══════════════════════════════════════════════════════════════════════════
    def plot_cem_convergence_per_jump(self):
        """
        One subplot per jump: best fitness vs. CEM iteration.
        Also overlays all jumps on a single axes for easy comparison.
        """
        n = self.n_jumps
        ncols = min(n, 3)
        nrows = int(np.ceil(n / ncols))
        fig, axes = plt.subplots(nrows, ncols,
                                 figsize=(6 * ncols, 4 * nrows),
                                 squeeze=False)
        fig.suptitle("CEM Convergence per Jump", fontsize=15, fontweight="bold")

        fig2, ax2 = plt.subplots(figsize=(10, 6))
        ax2.set_title("CEM Convergence – All Jumps Overlaid", fontsize=13, fontweight="bold")

        for j_idx, cem_hist in enumerate(self.cem_histories):
            ax = axes[j_idx // ncols][j_idx % ncols]
            color = self._jump_color(j_idx)

            history = cem_hist.get("iteration_history", [])
            iters   = [h["iteration"]  for h in history]
            bvals   = [self._disp_fit(h["best_value"]) for h in history]

            ax.plot(iters, bvals, color=color, linewidth=2.0, marker="o",
                    markersize=4)
            ax.set_title(f"Jump {j_idx+1}", fontsize=11, fontweight="bold")
            ax.set_xlabel("CEM Iteration"); ax.set_ylabel("Best Fitness")
            ax.grid(True, linestyle="--", alpha=0.5)

            # add early-stop annotation
            total_iters = cem_hist["metadata"].get("total_iterations", len(history))
            if len(history) < total_iters:
                ax.axvline(x=len(history), color="red", linestyle="--",
                           linewidth=1.2, alpha=0.7, label="Early stop")
                ax.legend(fontsize=8)

            # combined plot
            ax2.plot(iters, bvals, color=color, linewidth=1.8,
                     marker="o", markersize=3,
                     label=f"Jump {j_idx+1} (fit={bvals[-1]:.1f})")

        # hide unused subplots
        for idx in range(n, nrows * ncols):
            axes[idx // ncols][idx % ncols].set_visible(False)

        ax2.set_xlabel("CEM Iteration"); ax2.set_ylabel("Best Fitness")
        ax2.legend(loc="lower right", fontsize=9)
        ax2.grid(True, linestyle="--", alpha=0.5)

        fig.tight_layout()
        fig2.tight_layout()
        self._save(fig,  "ms_cem_convergence_per_jump")
        self._save(fig2, "ms_cem_convergence_overlay")
        plt.show()

    # ══════════════════════════════════════════════════════════════════════════
    # 4.  Per-jump 3-D trajectory panels
    # ══════════════════════════════════════════════════════════════════════════
    def plot_jump_panels_3d(self, elev=25, azim=-55):
        """One 3-D subplot per jump showing each trajectory on the terrain."""
        n = self.n_jumps
        ncols = min(n, 3)
        nrows = int(np.ceil(n / ncols))
        fig = plt.figure(figsize=(7 * ncols, 6 * nrows))
        fig.suptitle("Per-Jump 3-D Trajectory Panels", fontsize=15, fontweight="bold")

        px, py, pz, costs = self._terrain_xyz()

        for j_idx, js in enumerate(self.jump_summaries):
            ax  = fig.add_subplot(nrows, ncols, j_idx + 1, projection="3d")
            color = self._jump_color(j_idx)

            ax.scatter(px, py, pz, c=costs, cmap="RdYlGn_r", s=0.8,
                       alpha=0.35, zorder=1)

            traj   = js.get("best_trajectory", [])
            points = js.get("best_points",     [])
            p0     = np.array(js["p0"])
            pf     = np.array(js["pf"])
            at     = js.get("best_achieved_target")

            for x_s, y_s, z_s in _traj_segments(traj):
                ax.plot(x_s, y_s, z_s, color=color, linewidth=2.2, zorder=6)

            if points:
                pts = np.array(points)
                ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                           color=color, s=50, edgecolors="black",
                           linewidths=0.7, zorder=8)

            ax.scatter(p0[0], p0[1], p0[2], c="lime",  s=80, marker="^",
                       edgecolors="black", zorder=12)
            ax.scatter(pf[0], pf[1], pf[2], c="red",   s=80, marker="X",
                       edgecolors="black", zorder=12)

            if at:
                at_np = np.array(at).flatten()
                ax.scatter(at_np[0], at_np[1], at_np[2], c="orange",
                           s=90, marker="D", edgecolors="black", zorder=11)
                dist = np.linalg.norm(pf - at_np)
                ax.plot([pf[0], at_np[0]], [pf[1], at_np[1]], [pf[2], at_np[2]],
                        "r--", linewidth=1.0, alpha=0.6)

            fit_str = f"{self._disp_fit(js['best_fitness']):.2f}"
            t_str   = f"{js.get('wall_time_s', 0):.0f}s"
            ax.set_title(f"Jump {j_idx+1}\nfitness={fit_str}  time={t_str}",
                         fontsize=10, fontweight="bold")
            ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
            ax.view_init(elev=elev, azim=azim)
            _set_axes_equal_3d(ax)

        fig.tight_layout()
        self._save(fig, "ms_jump_panels_3d")
        plt.show()

    # ══════════════════════════════════════════════════════════════════════════
    # 5.  Summary bar charts: fitness, energy, landing cost, wall time
    # ══════════════════════════════════════════════════════════════════════════
    def plot_summary_bars(self):
        """4-panel bar chart on a single row: fitness, energy, landing cost, wall time."""
        n = self.n_jumps
        jump_labels = [f"J{j+1}" for j in range(n)]
        fitness  = [self._disp_fit(js["best_fitness"]) for js in self.jump_summaries]
        energy   = [js.get("best_consumed_energy",  0) or 0 for js in self.jump_summaries]
        land     = [js.get("best_landing_cost",     0) or 0 for js in self.jump_summaries]
        times    = [js.get("wall_time_s",           0) or 0 for js in self.jump_summaries]
        colors   = [self._jump_color(j) for j in range(n)]

        fig, axes = plt.subplots(1, 4, figsize=(18, 5))
        # fig.suptitle("Per-Jump Optimization Summary", fontsize=17, fontweight="bold")

        data = [
            (axes[0], fitness, "Best Fitness",    "Fitness"),
            (axes[1], energy,  "Consumed Energy", "Energy (J)"),
            (axes[2], land,    "Landing Cost",    "Landing Cost"),
            (axes[3], times,   "Wall Time",       "Time (s)"),
        ]

        for ax, vals, title, ylabel in data:
            bars = ax.bar(jump_labels, vals, color=colors, edgecolor="black", alpha=0.85)
            ax.set_title(title, fontsize=14, fontweight="bold")
            ax.set_ylabel(ylabel, fontsize=12)
            ax.tick_params(axis="both", labelsize=11)
            ax.grid(axis="y", linestyle="--", alpha=0.5)
            for bar, v in zip(bars, vals):
                ax.text(bar.get_x() + bar.get_width() / 2,
                        bar.get_height() * 1.01,
                        f"{v:.1f}", ha="center", va="bottom", fontsize=10)

        fig.tight_layout()
        self._save(fig, "ms_summary_bars")
        plt.show()

    # ══════════════════════════════════════════════════════════════════════════
    # 6.  Waypoint chain overview (2-D)
    # ══════════════════════════════════════════════════════════════════════════
    def plot_waypoint_chain_2d(self):
        """
        2-D (YZ) overview of the waypoint chain annotated with jump index and
        best fitness for each segment.
        """
        fig, ax = plt.subplots(figsize=(13, 9))
        ax.set_aspect("equal", adjustable="box")

        self._draw_patches_2d(ax)

        # draw waypoints
        for w_idx, wp in enumerate(self.waypoints):
            ax.scatter(wp[1], wp[2], c="white", s=180, edgecolors="black",
                       linewidths=2, zorder=15, marker="o")
            ax.text(wp[1], wp[2] + 0.4, f"W{w_idx}", fontsize=9,
                    ha="center", fontweight="bold", zorder=16)

        # draw jump arrows + best contact points
        for j_idx, js in enumerate(self.jump_summaries):
            color  = self._jump_color(j_idx)
            p0     = np.array(js["p0"])
            pf     = np.array(js["pf"])
            points = js.get("best_points", [])
            traj   = js.get("best_trajectory", [])

            # trajectory
            for _, y_s, z_s in _traj_segments(traj):
                ax.plot(y_s, z_s, color=color, linewidth=1.8, alpha=0.7, zorder=6)

            # contact points
            if points:
                pts = np.array(points)
                ax.scatter(pts[:, 1], pts[:, 2], color=color, s=45,
                           edgecolors="black", linewidths=0.5, zorder=8)

            # jump label at midpoint
            mid_y = (p0[1] + pf[1]) / 2
            mid_z = (p0[2] + pf[2]) / 2
            ax.text(mid_y, mid_z, f" J{j_idx+1}\n({self._disp_fit(js['best_fitness']):.0f})",
                    fontsize=8, color=color, ha="left", va="center",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", alpha=0.6), zorder=14)

        ax.set_title(
            f"Waypoint Chain Overview  –  2-D (YZ)\n"
            f"{self.n_jumps} jumps  |  Total Fitness: {self._disp_fit(self.global_fitness):.4f}",
            fontsize=13, fontweight="bold",
        )
        ax.set_xlabel("Y (m)"); ax.set_ylabel("Z (m)")
        ax.grid(True, linestyle="--", alpha=0.4)

        # legend
        legend_handles = [
            mpatches.Patch(color=self._jump_color(j), label=f"Jump {j+1}")
            for j in range(self.n_jumps)
        ] + [
            plt.Line2D([0], [0], marker="o", color="w",
                       markerfacecolor="white", markeredgecolor="black",
                       markersize=10, label="Waypoint"),
        ]
        ax.legend(handles=legend_handles, loc="upper left",
                  bbox_to_anchor=(1.01, 1), fontsize=9, borderaxespad=0)

        fig.tight_layout()
        self._save(fig, "ms_waypoint_chain_2d")
        plt.show()

    # ══════════════════════════════════════════════════════════════════════════
    # 7.  Jump number distribution per jump (histogram)
    # ══════════════════════════════════════════════════════════════════════════
    def plot_jump_count_per_step(self):
        """
        For each jump step, show the distribution of n_jumps across the last
        iteration's best_discrete values read from cem_iteration_history.
        """
        fig, axes = plt.subplots(1, self.n_jumps,
                                 figsize=(5 * self.n_jumps, 4),
                                 squeeze=False)
        fig.suptitle("n_jumps Distribution (last CEM iteration) per step",
                     fontsize=13, fontweight="bold")

        for j_idx, cem_hist in enumerate(self.cem_histories):
            ax = axes[0][j_idx]
            history = cem_hist.get("iteration_history", [])
            if not history:
                ax.set_visible(False)
                continue

            last_iter = history[-1]
            # best_discrete[0] is n_jumps
            n_jumps_val = last_iter["best_discrete"][0] if last_iter.get("best_discrete") else None

            # collect all best_discrete[0] across iterations to show evolution
            nj_per_iter = [h["best_discrete"][0] for h in history if h.get("best_discrete")]
            iters = list(range(1, len(nj_per_iter) + 1))

            ax.plot(iters, nj_per_iter, color=self._jump_color(j_idx),
                    linewidth=2, marker="o", markersize=5)
            ax.set_title(f"Jump {j_idx+1}", fontsize=11, fontweight="bold")
            ax.set_xlabel("CEM Iteration")
            ax.set_ylabel("Best n_jumps")
            ax.yaxis.get_major_locator().set_params(integer=True)
            ax.grid(True, linestyle="--", alpha=0.5)

        fig.tight_layout()
        self._save(fig, "ms_njumps_evolution")
        plt.show()

    # ══════════════════════════════════════════════════════════════════════════
    # 8.  Timing breakdown
    # ══════════════════════════════════════════════════════════════════════════
    def plot_timing(self):
        """Wall time breakdown per jump with per-iteration detail if available."""
        times_total = [js.get("wall_time_s", 0) or 0 for js in self.jump_summaries]

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        fig.suptitle("Timing Report", fontsize=14, fontweight="bold")

        # ── left: total wall time ──
        ax = axes[0]
        jump_labels = [f"Jump {j+1}" for j in range(self.n_jumps)]
        colors = [self._jump_color(j) for j in range(self.n_jumps)]
        bars = ax.bar(jump_labels, times_total, color=colors, edgecolor="black", alpha=0.85)
        ax.set_title("Total Wall Time per Jump", fontsize=11, fontweight="bold")
        ax.set_ylabel("Time (s)"); ax.grid(axis="y", linestyle="--", alpha=0.5)
        for bar, t in zip(bars, times_total):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() * 1.02,
                    f"{t:.0f}s", ha="center", va="bottom", fontsize=9)

        # ── right: per-iteration average time if available ──
        ax2 = axes[1]
        for j_idx, tr in enumerate(self.timing_reports):
            iters_data = tr.get("iterations", [])
            if not iters_data:
                # try cem_iteration_history func_evals as proxy
                continue
            iter_nums  = [d.get("iteration", i + 1)       for i, d in enumerate(iters_data)]
            eval_times = [d.get("eval_total_time_s", None) for d in iters_data]
            eval_times = [t for t in eval_times if t is not None]
            if eval_times:
                ax2.plot(range(1, len(eval_times) + 1), eval_times,
                         color=self._jump_color(j_idx), linewidth=1.8,
                         marker="o", markersize=3,
                         label=f"Jump {j_idx+1}")

        ax2.set_title("Per-Iteration Eval Time (if logged)", fontsize=11, fontweight="bold")
        ax2.set_xlabel("CEM Iteration"); ax2.set_ylabel("Time (s)")
        ax2.legend(fontsize=9); ax2.grid(True, linestyle="--", alpha=0.5)

        fig.tight_layout()
        self._save(fig, "ms_timing")
        plt.show()


# ─────────────────────────────────────────────────────────────────────────────
def main():
    """Run all plots."""
    plotter = PlotResultMultiStep()
    print("[INFO] Plotter ready.\n")

    # ── global views (3-D + 2-D combined in one window) ─────────────────────
    plotter.plot_global_3d_trajectory(10,-20)
    # plotter.plot_waypoint_chain_2d()

    # # ── per-jump detail ───────────────────────────────────────────────────────
    # # plotter.plot_jump_panels_3d()
    # # plotter.plot_cem_convergence_per_jump()
    # # plotter.plot_jump_count_per_step()

    # # ── summary statistics ────────────────────────────────────────────────────
    plotter.plot_summary_bars()
    # plotter.plot_timing()

    print("[INFO] All plots completed!")


if __name__ == "__main__":
    main()
