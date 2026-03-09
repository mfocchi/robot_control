"""
Patched halton_walker.py

Features:
- Halton proposals (low-discrepancy)
- Hilbert sort of proposals (spatial locality)
- Jump optimizer (relaxation + projection)
- Robust emitter that walks optimizer polyline and emits exact-R hops
  (every emitted hop will be exactly R except possibly the final residual <= R)
- Per-run printed step statistics and saved plot PNG so you can confirm changes

Dependencies: numpy, matplotlib
Run: python3 halton_walker.py
"""

import math
from typing import List, Tuple, Optional, Iterable
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# Halton sequence (2D)
# ---------------------------
def halton_single(index: int, base: int) -> float:
    """Return index-th (1-based) Halton coordinate for given base."""
    result = 0.0
    f = 1.0 / base
    i = index
    while i > 0:
        result += f * (i % base)
        i //= base
        f /= base
    return result

def halton_2d(n: int, start_index: int = 1) -> np.ndarray:
    """Return n points of 2D Halton sequence starting at start_index (1-based)."""
    pts = np.empty((n, 2), dtype=float)
    for k in range(n):
        idx = start_index + k
        pts[k, 0] = halton_single(idx, 2)  # base 2
        pts[k, 1] = halton_single(idx, 3)  # base 3
    return pts

# ---------------------------
# Hilbert curve helpers
# ---------------------------
def rot(n: int, x: int, y: int, rx: int, ry: int):
    # rotate/flip a quadrant appropriately (standard Hilbert routine)
    if ry == 0:
        if rx == 1:
            x = n - 1 - x
            y = n - 1 - y
        # swap x and y
        x, y = y, x
    return x, y

def xy2d(n: int, x: int, y: int) -> int:
    """
    Convert (x,y) to Hilbert d index for grid size n (n must be power of 2).
    Classic implementation: iterate over bit-planes.
    """
    d = 0
    s = n // 2
    while s > 0:
        rx = 1 if (x & s) != 0 else 0
        ry = 1 if (y & s) != 0 else 0
        d += s * s * ((3 * rx) ^ ry)
        x, y = rot(s, x, y, rx, ry)
        s //= 2
    return d

# ---------------------------
# Geometry helpers
# ---------------------------
def point_in_polygon(pt: Tuple[float,float], polygon: List[Tuple[float,float]]) -> bool:
    """Ray-casting point-in-polygon test. Returns True if inside or on boundary."""
    x, y = pt
    inside = False
    n = len(polygon)
    for i in range(n):
        x0,y0 = polygon[i]
        x1,y1 = polygon[(i+1)%n]
        # horizontal edge check
        if (y0 == y1) and (y == y0) and (min(x0,x1) <= x <= max(x0,x1)):
            return True
        intersect = ((y0 > y) != (y1 > y)) and (x < (x1-x0)*(y-y0)/(y1-y0+1e-16) + x0)
        if intersect:
            inside = not inside
    return inside

# ---------------------------
# Jump optimizer (iterative relaxation + projection)
# ---------------------------
def attempt_jump_via_optimizer(cur: Tuple[float,float],
                               target: Tuple[float,float],
                               R: float,
                               max_waypoints: int = 500,
                               max_iters: int = 300,
                               tol: float = 1e-4,
                               domain_min: Optional[Tuple[float,float]] = None,
                               domain_max: Optional[Tuple[float,float]] = None,
                               polygon: Optional[List[Tuple[float,float]]] = None
                              ) -> Optional[List[Tuple[float,float]]]:
    """
    Try to find a sequence of waypoints from cur -> target s.t. every step <= R,
    using iterative relaxation and projection. Returns list of points (including endpoints)
    if converged, otherwise None.
    """
    def inside_domain(pt):
        if polygon is not None:
            return point_in_polygon(pt, polygon)
        if domain_min is not None and domain_max is not None:
            x,y = pt
            return (domain_min[0] - 1e-12 <= x <= domain_max[0] + 1e-12) and (domain_min[1] - 1e-12 <= y <= domain_max[1] + 1e-12)
        return True

    dx = target[0] - cur[0]
    dy = target[1] - cur[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-12:
        return [cur]

    n_steps = int(math.ceil(dist / R))
    if n_steps > max_waypoints:
        return None

    # Initial polyline: start + internal linear interpolation + target
    pts = [np.array(cur, dtype=float)]
    for k in range(1, n_steps):
        alpha = k / n_steps
        pts.append(np.array([cur[0] + alpha*dx, cur[1] + alpha*dy], dtype=float))
    pts.append(np.array(target, dtype=float))

    # iterative relaxation & projection
    for it in range(max_iters):
        max_move = 0.0
        # relaxation: internal nodes -> damped average of neighbors
        for i in range(1, len(pts)-1):
            neigh_avg = 0.5 * (pts[i-1] + pts[i+1])
            new_pos = 0.5 * (pts[i] + neigh_avg)  # damped move towards neighbor average
            # clamp to bbox if available
            if not inside_domain((new_pos[0], new_pos[1])):
                if domain_min is not None and domain_max is not None:
                    new_pos[0] = min(max(new_pos[0], domain_min[0]), domain_max[0])
                    new_pos[1] = min(max(new_pos[1], domain_min[1]), domain_max[1])
                else:
                    # if polygon provided and new_pos outside, keep it (we'll check feasibility later)
                    pass
            move = math.hypot(new_pos[0] - pts[i][0], new_pos[1] - pts[i][1])
            pts[i] = new_pos
            if move > max_move:
                max_move = move

        # projection pass: enforce pairwise distances <= R
        # forward
        for i in range(1, len(pts)):
            a = pts[i-1]
            b = pts[i]
            d = np.linalg.norm(b - a)
            if d > R:
                dir_vec = (b - a) / d
                pts[i] = a + dir_vec * R
        # backward
        for i in range(len(pts)-2, -1, -1):
            a = pts[i]
            b = pts[i+1]
            d = np.linalg.norm(b - a)
            if d > R:
                dir_vec = (a - b) / d
                pts[i] = b + dir_vec * R

        if max_move < tol:
            # final feasibility check: all consecutive distances <= R and all points inside domain
            ok = True
            for i in range(1, len(pts)):
                if np.linalg.norm(pts[i] - pts[i-1]) > R + 1e-8:
                    ok = False
                    break
            if ok:
                # ensure all points are inside domain; if polygon check fails, treat as failure
                if polygon is not None:
                    all_inside = all(point_in_polygon((float(p[0]), float(p[1])), polygon) for p in pts)
                    if not all_inside:
                        return None
                return [ (float(p[0]), float(p[1])) for p in pts ]
    return None

# ---------------------------
# Robust emitter: exact-R steps along polyline
# ---------------------------
def emit_R_steps_along_polyline(poly: List[Tuple[float,float]],
                                R: float,
                                domain_min: Optional[Tuple[float,float]] = None,
                                domain_max: Optional[Tuple[float,float]] = None,
                                polygon: Optional[List[Tuple[float,float]]] = None,
                                eps: float = 1e-12) -> Optional[List[Tuple[float,float]]]:
    """
    Walk along polyline `poly` (list of (x,y) vertices) starting at poly[0].
    Emit a sequence of points such that consecutive emitted steps are exactly R,
    except possibly the final emitted point which will be <= R from the previous.
    Returns the emitted list (including the first point = poly[0]) or None if
    any emitted point lies outside domain/polygon (treat as infeasible).
    """
    if not poly:
        return []

    def inside(pt):
        x,y = pt
        if polygon is not None:
            if not point_in_polygon((x,y), polygon):
                return False
        if domain_min is not None and domain_max is not None:
            if not (domain_min[0] - eps <= x <= domain_max[0] + eps and domain_min[1] - eps <= y <= domain_max[1] + eps):
                return False
        return True

    pts = [(float(x), float(y)) for x,y in poly]
    # Precompute segments and their lengths
    segs = []
    for i in range(len(pts)-1):
        x0,y0 = pts[i]
        x1,y1 = pts[i+1]
        dx = x1 - x0
        dy = y1 - y0
        L = math.hypot(dx, dy)
        if L > 0:
            segs.append({'x0':x0,'y0':y0,'dx':dx,'dy':dy,'L':L})
        else:
            segs.append({'x0':x0,'y0':y0,'dx':0.0,'dy':0.0,'L':0.0})

    emitted = [pts[0]]
    cur_x, cur_y = emitted[0]

    # cursor: index into segs and offset distance along that segment already consumed
    seg_i = 0
    offset = 0.0

    while True:
        # compute remaining length along polyline from current cursor
        rem = 0.0
        if seg_i < len(segs):
            rem += max(0.0, segs[seg_i]['L'] - offset)
            for j in range(seg_i+1, len(segs)):
                rem += segs[j]['L']
        if rem < 1e-12:
            # nothing more to traverse; append final endpoint if not duplicate
            final = pts[-1]
            if math.hypot(final[0]-emitted[-1][0], final[1]-emitted[-1][1]) > eps:
                emitted.append(final)
                if not inside(final):
                    return None
            break

        # we need to place next emitted point at distance R from (cur_x,cur_y)
        need = R
        # walk segments starting from current cursor to locate where the next point lands
        j = seg_i
        pos_in_seg = offset
        landed = False
        while j < len(segs):
            seg = segs[j]
            avail = seg['L'] - pos_in_seg
            if avail + 1e-14 >= need:
                # point lies on this segment
                t = (pos_in_seg + need) / seg['L'] if seg['L'] > 0 else 0.0
                nx = seg['x0'] + seg['dx'] * t
                ny = seg['y0'] + seg['dy'] * t
                # domain containment check
                if not inside((nx, ny)):
                    return None
                emitted.append((nx, ny))
                # update cursor to new location
                seg_i = j
                offset = pos_in_seg + need
                # if reached end of this segment, advance to next and zero offset
                if offset + 1e-12 >= segs[seg_i]['L']:
                    seg_i += 1
                    offset = 0.0
                cur_x, cur_y = nx, ny
                landed = True
                break
            else:
                need -= avail
                j += 1
                pos_in_seg = 0.0
        if not landed:
            # remaining polyline shorter than need: append final endpoint (residual < R)
            final = pts[-1]
            if math.hypot(final[0]-emitted[-1][0], final[1]-emitted[-1][1]) > eps:
                if not inside(final):
                    return None
                emitted.append(final)
            break

    return emitted

# ---------------------------
# Core walker: Halton -> Hilbert sort -> optimizer acceptance -> emitter
# ---------------------------
def online_halton_hilbert_walk(
    domain_min: Tuple[float,float],
    domain_max: Tuple[float,float],
    start_point: Tuple[float,float],
    R: float,
    n_proposals: int = 4000,
    cover_tol: float = 0.0,
    polygon: Optional[List[Tuple[float,float]]] = None,
    optimizer_max_waypoints: int = 500,
    optimizer_max_iters: int = 300,
    optimizer_tol: float = 1e-4,
    hilbert_grid_res_power: int = 10  # use 2^power resolution for Hilbert mapping; e.g., 10 -> 1024
) -> Tuple[List[Tuple[float,float]], dict]:
    """
    Perform the walk:
     - generate n_proposals Halton points
     - map to domain
     - sort by Hilbert index (grid resolution 2^hilbert_grid_res_power)
     - for each proposal: skip if within cover_tol to existing samples;
         else run optimizer; if converged run emitter; if emitter works append emitted R-steps
    Returns: (samples_list, stats)
    """
    # scale halton to domain
    hal = halton_2d(n_proposals, start_index=1)
    scaled = np.empty_like(hal)
    scaled[:,0] = domain_min[0] + hal[:,0] * (domain_max[0] - domain_min[0])
    scaled[:,1] = domain_min[1] + hal[:,1] * (domain_max[1] - domain_min[1])

    # optional polygon filter (pre-filter proposals outside polygon to reduce work)
    if polygon is not None:
        mask = np.array([point_in_polygon((float(x), float(y)), polygon) for x,y in scaled])
        scaled = scaled[mask]

    # prepare Hilbert grid resolution (power of two)
    grid_side = 1 << hilbert_grid_res_power  # 2^power
    # map coordinates to integer grid indices [0, grid_side-1]
    eps = 1e-12
    ix = np.clip(((scaled[:,0] - domain_min[0]) / (domain_max[0] - domain_min[0] + eps) * grid_side).astype(int), 0, grid_side-1)
    iy = np.clip(((scaled[:,1] - domain_min[1]) / (domain_max[1] - domain_min[1] + eps) * grid_side).astype(int), 0, grid_side-1)

    # compute Hilbert indices
    hilbert_indices = [xy2d(grid_side, int(x), int(y)) for x,y in zip(ix,iy)]
    order = np.argsort(hilbert_indices)
    proposals = scaled[order]

    # samples and stats
    samples: List[Tuple[float,float]] = []
    cur = tuple(start_point)
    samples.append(cur)
    failed = 0
    accepted = 0
    skipped = 0
    total_proposals = len(proposals)

    # helper: nearest dist squared brute-force
    def nearest_dist_sq(pt, S):
        if not S:
            return float('inf')
        arr = np.array(S)
        dx = arr[:,0] - pt[0]
        dy = arr[:,1] - pt[1]
        dsq = dx*dx + dy*dy
        return float(np.min(dsq))

    for i in range(total_proposals):
        t = (float(proposals[i,0]), float(proposals[i,1]))
        # skip if already covered
        if cover_tol > 0.0:
            if nearest_dist_sq(t, samples) <= cover_tol * cover_tol:
                skipped += 1
                continue

        # try optimizer
        seq = attempt_jump_via_optimizer(cur, t, R,
                                         max_waypoints=optimizer_max_waypoints,
                                         max_iters=optimizer_max_iters,
                                         tol=optimizer_tol,
                                         domain_min=domain_min,
                                         domain_max=domain_max,
                                         polygon=polygon)
        if seq is None:
            failed += 1
            continue

        # robust emission: produce exact-R hops along seq polyline
        resampled = emit_R_steps_along_polyline(seq, R, domain_min=domain_min, domain_max=domain_max, polygon=polygon)
        if resampled is None:
            # emission produced a point outside domain -> treat as failure
            failed += 1
            continue

        # Sanity check on chunk distances (optional debug)
        # chunk_dists = [math.hypot(resampled[i+1][0]-resampled[i][0], resampled[i+1][1]-resampled[i][1]) for i in range(len(resampled)-1)]
        # if len(chunk_dists) > 0:
        #     cd_min = min(chunk_dists); cd_max = max(chunk_dists); cd_mean = sum(chunk_dists)/len(chunk_dists)
        #     # print(f"proposal {i}: emitted steps min/mean/max = {cd_min:.6g}/{cd_mean:.6g}/{cd_max:.6g}")

        # append but skip first (it's the current location)
        for p in resampled[1:]:
            samples.append(p)
        cur = samples[-1]
        accepted += 1

    stats = {'n_proposals': total_proposals, 'accepted': accepted, 'failed': failed, 'skipped': skipped}
    return samples, stats

# ---------------------------
# Coverage estimator (approximate fill distance)
# ---------------------------
def approximate_fill_distance(samples: List[Tuple[float,float]],
                              domain_min: Tuple[float,float],
                              domain_max: Tuple[float,float],
                              polygon: Optional[List[Tuple[float,float]]] = None,
                              grid_res: int = 200) -> float:
    xs = np.linspace(domain_min[0], domain_max[0], grid_res)
    ys = np.linspace(domain_min[1], domain_max[1], grid_res)
    maxd = 0.0
    arr_samples = np.array(samples)
    for yi in ys:
        for xi in xs:
            pt = (xi, yi)
            if polygon is not None and not point_in_polygon(pt, polygon):
                continue
            if arr_samples.size == 0:
                d = float('inf')
            else:
                dx = arr_samples[:,0] - xi
                dy = arr_samples[:,1] - yi
                d = float(np.min(np.hypot(dx, dy)))
            if d > maxd:
                maxd = d
    return maxd

# ---------------------------
# Plot helper
# ---------------------------
def plot_path_and_samples(domain_min, domain_max, samples, polygon=None, title=None):
    samples = np.array(samples)
    plt.figure(figsize=(6,6))
    if polygon is not None:
        poly = np.array(polygon)
        plt.fill(poly[:,0], poly[:,1], facecolor=(0.95,0.95,0.95), edgecolor='k', linewidth=1)
    x0,y0 = domain_min
    x1,y1 = domain_max
    plt.xlim(x0, x1)
    plt.ylim(y0, y1)
    plt.plot(samples[0, 0], samples[0, 1], '-o',color="red", linewidth=0.6, markersize=8)
    if samples.size > 0:
        plt.plot(samples[:,0], samples[:,1], '-o', linewidth=0.6, markersize=2)
    plt.gca().set_aspect('equal', 'box')
    if title:
        plt.title(title)
    plt.tight_layout()
    plt.grid()

    plt.show()

# ---------------------------
# Experiment runner
# ---------------------------
def get_halton_samples(domain_min, domain_max, start_point, R_values,margin =(0., 0.),
                    n_proposals=4000, cover_tol=0.0, polygon=None,
                    optimizer_max_waypoints=500, optimizer_max_iters=300, optimizer_tol=1e-4,
                    hilbert_grid_res_power=10, grid_res_cov=200, show_plots=True, save_plots_prefix=None):


    print(f"\n--- Running R = {R_values} ---")

    shrinked_domain_min = tuple(np.array(domain_min) + margin)
    shrinked_domain_max = tuple(np.array(domain_max) - margin)

    samples, stats = online_halton_hilbert_walk(
        shrinked_domain_min, shrinked_domain_max, start_point, R_values,
        n_proposals=n_proposals,
        cover_tol=cover_tol,
        polygon=polygon,
        optimizer_max_waypoints=optimizer_max_waypoints,
        optimizer_max_iters=optimizer_max_iters,
        optimizer_tol=optimizer_tol,
        hilbert_grid_res_power=hilbert_grid_res_power
    )
    fill = approximate_fill_distance(samples, shrinked_domain_min, shrinked_domain_max, polygon=polygon, grid_res=grid_res_cov)
    stats.update({'R': R_values, 'n_samples': len(samples), 'fill_dist': fill})

    print(f"R={R_values:.3g}  proposals={stats['n_proposals']}, accepted={stats['accepted']}, failed={stats['failed']}, skipped={stats['skipped']}")
    print(f"samples={len(samples)}, approx_fill={fill:.6g}")

    # compute and print global step stats
    if len(samples) > 1:
        steps = np.array([math.hypot(samples[i+1][0]-samples[i][0], samples[i+1][1]-samples[i][1]) for i in range(len(samples)-1)])
        print(f"EMITTED STEPS — count={len(steps)}  min={steps.min():.6g} mean={steps.mean():.6g} max={steps.max():.6g}")
    else:
        print("No emitted steps.")

    if show_plots:
        title = f"R={R_values}, n={len(samples)}, fill={fill:.3g}"
        plot_path_and_samples(domain_min, domain_max, samples, polygon=polygon, title=title)

    return {'stats': stats, 'samples': samples}

# ---------------------------
# Example usage
# ---------------------------
if __name__ == "__main__":

    # bounding box and start
    domain_min = (0.0, -20.0)
    domain_max = (5.0, 0)
    start = (2.5, -6)
    # Edit this list to try different R values
    R_list = [3.]  # <<-- set to the R you want to test
    results = get_halton_samples(
        domain_min, domain_max, start,
        R_values=R_list[0],
        margin=(0.5,1.5),
        n_proposals=3000,
        cover_tol=1.5,
        polygon=None,
        optimizer_max_waypoints=500,
        optimizer_max_iters=300,
        optimizer_tol=1e-4,
        hilbert_grid_res_power=10,
        grid_res_cov=160,
        show_plots=True,
    )

    # Print the first 20 step lengths for inspection
    desired_target = []
    samples = results['samples']
    for sample in samples:
        desired_target.append(np.array([0.28, sample[0], sample[1]]))

    if len(samples) > 1:
        steps = [math.hypot(samples[i + 1][0] - samples[i][0], samples[i + 1][1] - samples[i][1]) for i in range(len(samples) - 1)]
        print("First 20 emitted step lengths:", steps[:20])
    else:
        print("No emitted samples to report.")