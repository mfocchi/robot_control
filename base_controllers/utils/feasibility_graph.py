import numpy as np
from collections import deque
from typing import Optional, Tuple
import csv
from termcolor import colored
import pandas as pd
import os

def make_uniform_grid_yz(y_min, y_max, z_min, z_max, ny, nz):
    """
    Returns array of shape (N,2) with columns [y, z]
    """
    ys = np.linspace(y_min, y_max, ny)
    zs = np.linspace(z_min, z_max, nz)
    Y, Z = np.meshgrid(ys, zs, indexing="ij")
    pts = np.column_stack([Y.ravel(), Z.ravel()])
    return pts


def build_directed_jump_graph(pts, is_feasible, csv_path="feasible_edges.csv", R_max=None):
    pts = np.asarray(pts, dtype=float)
    N = pts.shape[0]

    edge_list = []

    # Resume from existing CSV
    if os.path.exists(csv_path) and os.path.getsize(csv_path) > 0:
        try:
            df = pd.read_csv(csv_path)
            print(colored(f"CSV {csv_path} exists... continuing from where you left", "blue"))

            if len(df) > 0:
                edge_list = df[["i0", "if"]].to_numpy(dtype=int).tolist()
                last_i = int(df.iloc[-1]["i0"])
                last_j = int(df.iloc[-1]["if"])
            else:
                last_i, last_j = 0, -1

            write_header = False

        except Exception as e:
            print(colored(f"Could not read CSV, starting fresh. Error: {e}", "red"))
            df = pd.DataFrame(columns=["i0", "if", "y0", "z0", "yf", "zf"])
            last_i, last_j = 0, -1
            write_header = True
    else:
        print(colored(f"CREATING NEW CSV {csv_path}", "blue"))
        df = pd.DataFrame(columns=["i0", "if", "y0", "z0", "yf", "zf"])
        last_i, last_j = 0, -1
        write_header = True

    for i in range(last_i, N):
        p0 = pts[i]

        if R_max is not None:
            diff = pts - p0
            dist2 = np.sum(diff**2, axis=1)
            mask = dist2 <= R_max**2
        else:
            mask = np.ones(N, dtype=bool)

        mask[i] = False
        candidate_indices = np.nonzero(mask)[0]

        p0_sim = np.concatenate(([0.0], p0))

        for j in candidate_indices:
            # resume inside inner loop too
            if i == last_i and j <= last_j:
                continue

            pf_sim = np.concatenate(([0.0], pts[j]))
            print(f"p0: {p0_sim}, pf: {pf_sim}")

            if is_feasible(p0_sim, pf_sim):
                print(colored(f"idx: ({i},{j}), Feasible", "red"))

                edge_list.append([i, j])

                row = pd.DataFrame([{
                    "i0": i,
                    "if": j,
                    "y0": p0_sim[1],
                    "z0": p0_sim[2],
                    "yf": pf_sim[1],
                    "zf": pf_sim[2],
                }])

                row.to_csv(
                    csv_path,
                    mode="a",
                    header=write_header,
                    index=False
                )
                write_header = False

    print(colored("Done!", "green"))

    if len(edge_list) == 0:
        return np.empty((0, 2), dtype=int)

    return np.array(edge_list, dtype=int)

# -----------------------------
# Example feasibility function
# -----------------------------
def is_feasible_jump(p0, pf):
    """
    Replace this with your real jump optimization.
    Example:
    - upward reach limited
    - downward reach larger
    - total distance limited
    """
    dy = pf[1] - p0[1]
    dz = pf[2] - p0[2]

    MAX_UP = 1.5
    MAX_DOWN = 3.0
    MAX_DIST = 3.5

    if dz > MAX_UP:
        return False
    if dz < -MAX_DOWN:
        return False
    if np.hypot(dy, dz) > MAX_DIST:
        return False

    return True


def save_edges_to_csv(csv_path: str, pts: np.ndarray, edges: np.ndarray):
    """
    Save all directed feasible edges.

    Output columns:
    i0, i1, y0, z0, y1, z1
    """
    pts = np.asarray(pts, dtype=float)
    edges = np.asarray(edges, dtype=int)

    with open(csv_path, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        writer.writerow(["i0", "i1", "y0", "z0", "y1", "z1"])

        for i0, i1 in edges:
            y0, z0 = pts[i0]
            y1, z1 = pts[i1]
            writer.writerow([i0, i1, y0, z0, y1, z1])


def append_edges_to_csv(csv_path, pts, edge_buffer, write_header=False):
    """
    Append edges to CSV file.
    edge_buffer: list of [i,j]
    """
    mode = "w" if write_header else "a"

    with open(csv_path, mode, newline="") as f:
        writer = csv.writer(f)

        if write_header:
            writer.writerow(["i0", "i1", "y0", "z0", "y1", "z1"])

        for i0, i1 in edge_buffer:
            y0, z0 = pts[i0]
            y1, z1 = pts[i1]
            writer.writerow([i0, i1, y0, z0, y1, z1])



# -----------------------------
# Example usage
# -----------------------------
if __name__ == "__main__":
    #1 generate uniform grid
    pts = make_uniform_grid_yz(
        y_min=0.0, y_max=5.0,
        z_min=-20.0, z_max=0.0,
        ny=5, nz=20)
    print("Number of grid points:", pts.shape[0])

    #2 generate the directed feasibility graphs
    edges = build_directed_jump_graph(
        pts,
        is_feasible=is_feasible_jump,
        csv_path="feasible_edges_resume2.csv",
        #R_max=3.5   # optional prefilter
    )
    print("Directed feasible jumps:", edges.shape[0])
    print("Edges array shape:", edges.shape)

    save_edges_to_csv("feasible_edges.csv", pts, edges)
    print("Saved feasible_edges.csv")

