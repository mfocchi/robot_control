import json
import os
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.patches import Patch
from termcolor import colored

# Use same env variable / fallback as Plot_result.py
plot_str = os.environ.get("FOLDER_PLOT")
if plot_str:
    FOLDER_MAIN = np.array(json.loads(plot_str))
else:
    FOLDER_MAIN = "result/test_r_0_hemisphere"

TIMING_FILE = os.path.join(FOLDER_MAIN, "timing_report.json")


class PlotTimingReport:

    def __init__(self, timing_file=TIMING_FILE):
        if not os.path.exists(timing_file):
            raise FileNotFoundError(f"Timing report not found: {timing_file}")

        with open(timing_file, "r") as f:
            self.data = json.load(f)

        self.iterations = self.data.get("iterations", [])
        self.total_wall_time = self.data.get("total_wall_time_s", None)
        self.total_iters = self.data.get("total_iterations_completed", 0)

        # Derived arrays
        self.iter_nums = [it["iteration"] for it in self.iterations]
        self.iter_times = [it["iteration_time_s"] for it in self.iterations]
        self.mean_ind_times = [it["mean_individual_time_s"] for it in self.iterations]
        self.all_ind_times = [it["individual_times_s"] for it in self.iterations]

    # =====================
    # PRINT SUMMARY
    # =====================
    def print_summary(self):
        """Print a formatted timing summary to console."""
        print(colored(f"\n{'='*70}", "cyan", attrs=["bold"]))
        print(colored("  TIMING REPORT SUMMARY", "cyan", attrs=["bold"]))
        print(colored(f"{'='*70}", "cyan", attrs=["bold"]))

        if self.total_wall_time is not None:
            print(colored(f"  Total wall time          : {self.total_wall_time:.2f} s  "
                          f"({self.total_wall_time/60:.2f} min)", "green"))
        print(colored(f"  Iterations completed     : {self.total_iters}", "green"))

        # --- Iteration-level stats ---
        it = np.array(self.iter_times)
        print(colored(f"\n  --- Iteration times (s) ---", "yellow", attrs=["bold"]))
        print(colored(f"  Mean   : {np.mean(it):.4f}", "yellow"))
        print(colored(f"  Std    : {np.std(it):.4f}", "yellow"))
        print(colored(f"  Min    : {np.min(it):.4f}  (iter {self.iter_nums[int(np.argmin(it))]})", "yellow"))
        print(colored(f"  Max    : {np.max(it):.4f}  (iter {self.iter_nums[int(np.argmax(it))]})", "yellow"))
        print(colored(f"  Median : {np.median(it):.4f}", "yellow"))

        # --- Individual (population) level stats ---
        flat = np.concatenate(self.all_ind_times)
        print(colored(f"\n  --- Individual (population member) times (s) ---", "magenta", attrs=["bold"]))
        print(colored(f"  Mean   : {np.mean(flat):.4f}", "magenta"))
        print(colored(f"  Std    : {np.std(flat):.4f}", "magenta"))
        print(colored(f"  Min    : {np.min(flat):.4f}", "magenta"))
        print(colored(f"  Max    : {np.max(flat):.4f}", "magenta"))
        print(colored(f"  Median : {np.median(flat):.4f}", "magenta"))

        # Per-iteration individual stats
        print(colored(f"\n  --- Per-iteration individual stats ---", "blue", attrs=["bold"]))
        header = f"  {'Iter':>5} | {'Mean':>8} | {'Min':>8} | {'Max':>8} | {'Iter Time':>10}"
        print(colored(header, "blue"))
        print(colored(f"  {'-'*50}", "blue"))
        for it_data in self.iterations:
            ind = np.array(it_data["individual_times_s"])
            line = (f"  {it_data['iteration']:>5} | {np.mean(ind):>8.3f} | "
                    f"{np.min(ind):>8.3f} | {np.max(ind):>8.3f} | "
                    f"{it_data['iteration_time_s']:>10.3f}")
            print(colored(line, "blue"))

        print(colored(f"{'='*70}\n", "cyan", attrs=["bold"]))

    # =====================
    # PLOT 1: Iteration times
    # =====================
    def plot_iteration_times(self, ax=None):
        """Bar chart of iteration wall times."""
        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(12, 6))
            created = True
        else:
            fig = ax.get_figure()

        bars = ax.bar(self.iter_nums, self.iter_times,
                       color="steelblue", edgecolor="black", alpha=0.85)

        # Highlight max and min
        idx_max = int(np.argmax(self.iter_times))
        idx_min = int(np.argmin(self.iter_times))
        bars[idx_max].set_color("tomato")
        bars[idx_min].set_color("limegreen")

        # Mean line
        mean_val = np.mean(self.iter_times)
        ax.axhline(y=mean_val, color="orange", linestyle="--", linewidth=1.5,
                    label=f"Mean: {mean_val:.2f} s")

        ax.set_xlabel("Iteration", fontsize=12)
        ax.set_ylabel("Time (s)", fontsize=12)
        ax.set_title("Iteration Wall Time", fontsize=14, fontweight="bold")
        ax.set_xticks(self.iter_nums)
        ax.legend(fontsize=10)
        ax.grid(axis="y", linestyle="--", alpha=0.4)

        fig.tight_layout()

        if created:
            save_path = os.path.join(FOLDER_MAIN, "plot_iteration_times.png")
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            save_path_pdf = os.path.join(FOLDER_MAIN, "plot_iteration_times.pdf")
            fig.savefig(save_path_pdf, bbox_inches="tight", pad_inches=0.23)
            print(f"[SAVE] Iteration times plot saved to: {save_path}")
            plt.show()

    # =====================
    # PLOT 2: Individual times per iteration (interactive legend)
    # =====================
    def plot_individual_times_interactive(self, ax=None):
        """
        Line plot of individual evaluation times for every iteration.
        Each iteration is a separate series that can be toggled on/off
        by clicking on the interactive legend (same pattern as
        plot_map_with_cost_meshgrid_overlay).
        """
        created = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(14, 7))
            created = True
        else:
            fig = ax.get_figure()

        n_iters = len(self.iterations)
        cmap = cm.get_cmap("tab10") if n_iters <= 10 else cm.get_cmap("tab20")

        plot_objects = []      # matplotlib artist per iteration
        legend_elements = []   # Patch per legenda
        legend_map = {}        # legend_element -> artist

        pop_size = len(self.all_ind_times[0]) if self.all_ind_times else 0

        for k, it_data in enumerate(self.iterations):
            ind_times = it_data["individual_times_s"]
            x_positions = np.arange(len(ind_times))
            color_k = cmap(k % 20)

            # Use line plot instead of scatter
            line = ax.plot(x_positions, ind_times,
                          color=color_k, linewidth=2, alpha=0.8,
                          marker='o', markersize=4, markeredgecolor='black', markeredgewidth=0.3,
                          zorder=3, label=f"Iter {it_data['iteration']}")[0]

            plot_objects.append(line)

            patch = Patch(facecolor=color_k, alpha=0.8, edgecolor="black", linewidth=1,
                          label=f"Iter {it_data['iteration']}  "
                                f"(mean {it_data['mean_individual_time_s']:.2f}s)")
            legend_elements.append(patch)
            legend_map[id(patch)] = line

        # Mean per-individual line (global)
        flat = np.concatenate(self.all_ind_times)
        ax.axhline(y=np.mean(flat), color="red", linestyle="--", linewidth=1.5,
                    label=f"Global mean: {np.mean(flat):.2f} s", zorder=2)

        ax.set_xlabel("Individual index (population member)", fontsize=12)
        ax.set_ylabel("Evaluation time (s)", fontsize=12)
        ax.set_title("Individual Evaluation Times per Iteration\n"
                      "(Click legend to show/hide iterations)",
                      fontsize=14, fontweight="bold")
        
        # Set X-axis ticks every 50 individuals
        if pop_size > 0:
            tick_positions = range(0, pop_size, 50)
            ax.set_xticks(tick_positions)
        ax.grid(axis="y", linestyle="--", alpha=0.4)

        # ---- Interactive legend (same pattern) ----
        leg = ax.legend(handles=legend_elements, loc="upper left",
                        bbox_to_anchor=(1.02, 1), fontsize=9,
                        framealpha=0.9, edgecolor="black",
                        title="Click to show/hide", title_fontsize=10,
                        fancybox=True, shadow=True)

        # Enable picking
        patch_to_artist = {}
        for leg_patch, leg_text, le in zip(leg.get_patches(), leg.get_texts(), legend_elements):
            leg_patch.set_picker(True)
            leg_text.set_picker(True)
            artist = legend_map[id(le)]
            patch_to_artist[id(leg_patch)] = (artist, leg_patch, leg_text)
            patch_to_artist[id(leg_text)] = (artist, leg_patch, leg_text)

        def on_pick(event):
            key = id(event.artist)
            if key not in patch_to_artist:
                return
            artist, lpatch, ltext = patch_to_artist[key]
            visible = not artist.get_visible()
            artist.set_visible(visible)
            if visible:
                lpatch.set_alpha(0.8)
                ltext.set_alpha(1.0)
            else:
                lpatch.set_alpha(0.15)
                ltext.set_alpha(0.3)
            fig.canvas.draw_idle()

        fig.canvas.mpl_connect("pick_event", on_pick)

        fig.tight_layout()

        if created:
            save_path = os.path.join(FOLDER_MAIN, "plot_individual_times_interactive.png")
            fig.savefig(save_path, dpi=150, bbox_inches="tight")
            save_path_pdf = os.path.join(FOLDER_MAIN, "plot_individual_times_interactive.pdf")
            fig.savefig(save_path_pdf, bbox_inches="tight", pad_inches=0.23)
            print(f"[SAVE] Individual times interactive plot saved to: {save_path}")
            plt.show()



def main():
    print("[Plot_time] Loading timing report...")
    plotter = PlotTimingReport()

    # 1. Print summary
    plotter.print_summary()

    # 2. Iteration times bar chart
    plotter.plot_iteration_times()

    # 3. Individual times with interactive legend
    plotter.plot_individual_times_interactive()


    print("[Plot_time] All timing plots completed!")


if __name__ == "__main__":
    main()
