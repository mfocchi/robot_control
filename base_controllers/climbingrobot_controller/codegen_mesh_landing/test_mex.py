#!/usr/bin/env python
"""
Sample script that uses the MagicSquarePkg package created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""
import matlab.engine
import numpy as np
import math
from base_controllers.components.terrain_manager import TerrainManager
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
from termcolor import colored


np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
import matplotlib.pyplot as plt
eng = matlab.engine.start_matlab()

def plot_patch(landing_patch_center, params, wallSurfaceEval, axis):
    """
    Python translation of the MATLAB function plot_patch.
    - landing_patch_center : array-like (3,)
    - params : object with fields 'patch_side'
    - wallSurfaceEval : function(z, y, params) -> x
    """

    # Ensure column vector (flatten)
    landing_patch_center = np.array(landing_patch_center).flatten()

    resolution = 0.05
    n_points_z = int(np.floor(params['patch_side_y'] / resolution))
    n_points_y = int(np.floor(params['patch_side_z'] / resolution))

    dy = np.linspace(
        landing_patch_center[1] - params['patch_side_y'] / 2,
        landing_patch_center[1] + params['patch_side_y'] / 2,
        n_points_y
    )

    dz = np.linspace(
        landing_patch_center[2] - params['patch_side_z'] / 2,
        landing_patch_center[2] + params['patch_side_z']/ 2,
        n_points_z
    )

    # Plot each point like MATLAB plot3
    for i in range(n_points_y):
        for j in range(n_points_z):
            x = wallSurfaceEval(dz[j], dy[i], params['mesh_x'], params['mesh_y'] ,params['mesh_z']  )
            y = dy[i]
            z = dz[j]
            axis.scatter(x, y, z, marker='.', s=40, color='blue')  # s approx = MarkerSize 20

def generateCostMapGaussian(Lz, Ly, grid_size, gaussian_center, max_cost):
    """
    Python equivalent of the MATLAB function:
    [X, Y, Z] = generateCostMap(Lz, Ly, gridSize, gaussian_center, max_cost)
    """

    # Physical grid in meters
    z = np.linspace(Lz, 0, grid_size)
    y = np.linspace(0, Ly, grid_size)
    Z, Y = np.meshgrid(z, y)   # Z: vertical, Y: horizontal
    
    # Gaussian bump
    radius = 0.25
    bulge = np.exp(-((Z - gaussian_center[2])**2 +
                     (Y - gaussian_center[1])**2) / (2 * radius**2))

    # Height map (X in MATLAB code)
    X = max_cost -bulge * max_cost

    return X, Y, Z

def initOptim(p0, pf, Ly, patch_side_z, patch_side_y,   mesh_x, mesh_y, mesh_z):
    mass = 5.08
    params = {}
    params['m'] = mass
    anchor_distance = Ly
    params['num_params'] = 4.
    params['int_method'] = 'rk4'
    params['N_dyn'] = 30.
    params['FRICTION_CONE'] = 1.
    params['int_steps'] = 5.
    params['b'] = anchor_distance
    params['p_a1'] = matlab.double([0.,0.,0.]).reshape(3,1)
    params['p_a2'] = matlab.double([0.,params['b'],0.]).reshape(3,1)
    params['g'] = 9.81
    params['w1']= 0.001 # smooth
    params['w2']= 0. # hoist work
    params['w3']= 1000. # landing patch cost
    params['T_th'] =  0.05
    params['obstacle_avoidance'] = 'mesh'
    params['jump_clearance'] = 0.5
    params['debug'] = False #enables print of cost

    # Interpolator (note: z must be increasing — here from -10 to 0)
    p0[0] = terrainManager.wall_surface_eval(p0[2],p0[1],  mesh_x, mesh_y, mesh_z)
    pf[0] = terrainManager.wall_surface_eval(pf[2],pf[1],  mesh_x, mesh_y, mesh_z)
    # compute consistent normal
    normal = terrainManager.wall_normal_eval(p0[2],p0[1],  mesh_x, mesh_y, mesh_z )
    params['mesh_x'] = mesh_x
    params['mesh_y'] = mesh_y
    params['mesh_z'] = mesh_z

    #cost map parameters
    params['cost_x'] = cost_x
    params['cost_y'] = cost_y
    params['cost_z'] = cost_z
    params['patch_side_z'] =  patch_side_z
    params['patch_side_y'] = patch_side_y

    params['contact_normal'] = matlab.double(normal)
    return p0, pf, params


def plotStuff():
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    p_a1 = np.zeros(3)
    p_a2 = np.array([0, params['b'],0.])
    # Anchor 1 line
    ax.plot(  [p_a1[0], p0[0]],  [p_a1[1], p0[1]],  [p_a1[2], p0[2]],   'k--' )
    # Anchor 1 point
    ax.plot([p_a1[0]],   [p_a1[1]],   [p_a1[2]], marker='*', color='k', markersize=10,   linestyle='None')
    # Anchor 2 line
    ax.plot([p_a2[0], p0[0]],[p_a2[1], p0[1]], [p_a2[2], p0[2]],'k--' )
    # Anchor 2 point
    ax.plot([p_a2[0]],  [p_a2[1]],  [p_a2[2]],  marker='*',    color='k',     markersize=10,     linestyle='None')
    #plot X=0 wall

    # MESH Surface plot
    ax.plot_surface(params['mesh_x'], params['mesh_y'], params['mesh_z'], alpha=0.4, cmap='Blues', edgecolor='k', linewidth=0.2)
    # plot landing patch in blue
    plot_patch(pf, params, terrainManager.wall_surface_eval, ax)
    # plot surface
    ax.plot_surface(params['cost_x'] / 5, params['cost_y'], params['cost_z'], alpha=0.4, cmap='Blues', edgecolor='k', linewidth=0.2)

    # limit plot as in matlab
    min_x = min(np.min(ref_com[0, :]), pf[0]) - 3
    max_x = max(np.max(ref_com[0, :]), pf[0]) + 3
    min_y = 0
    max_y = p_a2[1]
    min_z = min(p0[2], pf[2]) - 4
    max_z = 2

    #plot wall
    # Create 2x2 grid for a rectangular wall at X = 0
    Yw, Zw = np.meshgrid(  [min_y, max_y],  [min_z, max_z]  )
    Xw = np.zeros_like(Yw)  # X = 0 plane
    # Draw the wall
    ax.plot_surface( Xw,  Yw,    Zw,    color='b',    alpha=0.5 )

    ax.set_xlim([min_x, max_x])
    ax.set_ylim([min_y, max_y])
    ax.set_zlim([min_z, max_z])
    # Alternative method using set_box_aspect for proportional scaling
    # ax.set_box_aspect([x_range, y_range, z_range])
    ax.set_box_aspect([1, 1, 1])  # axis equal
    ax.grid(True)
    # Camera position equivalent
    ax.view_init(elev=20, azim=60)  # Adjust to approximate MATLAB CameraPosition

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    # plot initial location
    ax.scatter(p0_adj[0], p0_adj[1], p0_adj[2], color='blue', s=300)  # s = size
    # plot center of landing patch
    ax.scatter(pf_adj[0], pf_adj[1], pf_adj[2], color='green', s=200)  # s = size
    # plot reference jump trajectory
    ax.plot3D(ref_com[0, :], ref_com[1, :], ref_com[2, :], color='red', linewidth=2.5)
    # plot achieved target
    ax.scatter(achieved_target[0], achieved_target[1], achieved_target[2], color='red', s=300)  # s = size

    plt.show()


def eval_constraints(c, num_constr, constr_tolerance, debug=False):
    """
    Check constraint vector `c` against blocks described by `num_constr`.
    - c: sequence or 1D numpy array of constraint values
    - num_constr: object or dict with integer fields:
        wall_constraints,
        retraction_force_constraints,
        force_constraints,
        final_constraints,
        via_point
    - constr_tolerance: scalar
    - debug: bool (print detailed per-constraint info)
    """

    # ensure numpy array (1D)
    c = np.array(c).flatten()

    # compute 0-based start indices for each block (Python indexing)
    w0     = 0
    r0     = w0 + int(num_constr['wall_constraints'])
    f0     = r0 + int(num_constr['retraction_force_constraints'])
    final0 = f0 + int(num_constr['force_constraints'])
    via0   = final0 + int(num_constr['final_constraints'])

    # optional debug listing
    if debug:
        for i, val in enumerate(c):
            if i == w0:
                print('wall constraints')
            if i == r0:
                print('retraction_force_constraints')
            if i == f0:
                print('leg force_constraints')
            if i == final0:
                print('final_point_constraints')
            if i == via0:
                print('via_point constraints')
            print(f"{i} {float(val):.6f}")

    print(colored("Eval Constraints (positive number represents viol.)", "red"))

    # 1) wall constraints
    w_block = c[w0 : w0 + int(num_constr['wall_constraints'])]
    if w_block.size > 0 and np.any(w_block > constr_tolerance):
        print('1) wall mesh constraints violated')
        print(" ".join(f"\033[91m{v:.6f}\033[0m" if v > constr_tolerance else f"{v:.6f}" for v in w_block))

    # 2) retraction force constraints
    r_block = c[r0 : r0 + int(num_constr['retraction_force_constraints'])]
    if r_block.size > 0 and np.any(r_block > constr_tolerance):
        print('2) rope force constraints violated')
        print(" ".join(f"\033[91m{v:.6f}\033[0m" if v > constr_tolerance else f"{v:.6f}" for v in r_block))

    # 3) force constraints (unilateral, actuation, friction, ...)
    f_block = c[f0 : f0 + int(num_constr['force_constraints'])]

    if f_block.size > 0 and np.any(f_block > constr_tolerance):
        print('3) leg force constraints violated')
        # show first few entries (match MATLAB: +1, +2, +3)
        if f_block.size >= 1:
            print(f"3.1 unilateral (Fun > fmin): \033[91m{f_block[0]:.6f}\033[0m" if f_block[0] > constr_tolerance else f"{f_block[0]:.6f}")

        if f_block.size >= 2:
            print(f"3.2 actuation (Fun < fun_max): \033[91m{f_block[1]:.6f}\033[0m" if f_block[1] > constr_tolerance else f"{f_block[1]:.6f}")
        if f_block.size >= 3:
            print(f"3.3 friction (|Fut| < mu*Fun): \033[91m{f_block[2]:.6f}\033[0m" if f_block[2] > constr_tolerance else f"{f_block[2]:.6f}")

    # 4) final point constraints (several subchecks)
    final_block = c[final0 : final0 + int(num_constr['final_constraints'])]

    if final_block.size > 0 and np.any(final_block > constr_tolerance):
        print('4) final point constraint violated')
        # Each check corresponds to offsets 0..4 in MATLAB
        if final_block.size >= 1 and final_block[0] > constr_tolerance:
            print(f"4.1 p_f(y) < ymax_patch : \033[91m{final_block[0]:.6f}\033[0m" if final_block[0] > constr_tolerance else f"{final_block[0]:.6f}")
        if final_block.size >= 2 and final_block[1] > constr_tolerance:
            print(f"4.2 p_f(y) > ymin_patch : \033[91m{final_block[1]:.6f}\033[0m" if final_block[1] > constr_tolerance else f"{final_block[1]:.6f}")
        if final_block.size >= 3 and final_block[2] > constr_tolerance:
            print(f"4.3 p_f(z) < zmax_patch : \033[91m{final_block[2]:.6f}\033[0m" if final_block[2] > constr_tolerance else f"{final_block[2]:.6f}")
        if final_block.size >= 4 and final_block[3] > constr_tolerance:
            print(f"4.4 p_f(z) > zmin_patch : \033[91m{final_block[3]:.6f}\033[0m" if final_block[3] > constr_tolerance else f"{final_block[3]:.6f}")
        if final_block.size >= 5 and final_block[4] > constr_tolerance:
            print(f"4.5 ||pf(x) - wall_x|| < fixed_slack :\033[91m{final_block[4]:.6f}\033[0m" if final_block[4] > constr_tolerance else f"{final_block[4]:.6f}")

    # 5) via point constraints
    via_block = c[via0 : via0 + int(num_constr['via_point'])]
    if via_block.size > 0 and np.any(via_block > constr_tolerance):
        print('5) via point constraint violated')
        print(f"via point :\033[91m{via_block[0]:.6f}\033[0m" if via_block[0] > constr_tolerance else f"{via_block[0]:.6f}")

def generateCostMap(terrain_manager, number_of_patches_width, number_of_patches_height):
    from base_controllers.components.point_cloud_filter import PointCloudFilter
    from base_controllers.components.patch_surface import PatchSurface
    filter_weights = np.array([0., 10., 10.0, 0.0, 0.0])

    # === 1 POINT CLOUD INITIALIZATION ===
    in_point_clouds = terrain_manager.point_cloud
    point_clouds = PointCloudFilter(in_point_clouds)


    # Filter with cost change and color based on height
    # point_clouds.filter_height_profile(x0=0.0, scale=1.0,side_application="depth", profile="logln")
    # point_clouds.visualize_cost_map()
    print("\n[INIT] === Smoothing Filter ===")
    kernel = [point_clouds.smoothing_kernel]
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[0], plot=False)
    print("\n[INIT] === First Derivative (Gradient) ===")
    kernel = [point_clouds.sobel_y, point_clouds.sobel_z]
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[1], plot=False)
    print("\n[INIT] === Second Derivative (Laplacian) ===")
    kernel = [point_clouds.laplacian_kernel]
    point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[2], plot=False)
    # point_clouds.visualize_cost_map()
    # print("\n[INIT] === Laplacian of Gaussian (LoG) ===")
    # # Usa log_kernel invece di laplacian_kernel
    # kernel = [point_clouds.log_kernel]
    # point_clouds.filter_process_points_pipeline(kernel, weight=filter_weights[3], plot=False)
    print("\n[INIT] === Bump Detection (II derviative v2) ===")
    point_clouds.compute_bump_detection(weight=filter_weights[3])

    # === 2 PATCHES INITIALIZATION ===
    pc_t = point_clouds.points_t
    patches = PatchSurface(pc_t, number_of_patches_width=number_of_patches_width, number_of_patches_height=number_of_patches_height)

    patches.gaussian_cost_all_patch(weight_gauss_cost=filter_weights[4])

    patches.visualize_full_cost_map()

    patches.print_patch_cost_matrix(2)

    patches.get_cost_meshgrid(grid_size=grid_size)

    cost_grid, cost_y, cost_z = patches.get_cost_meshgrid(grid_size=grid_size)
    return cost_grid, cost_y, cost_z

#######################################
status_map = {
    0: "converged",
    2: "semidef.converg",
    1: "not converged",
    -2: "max number of function evaluations"
}
#terrain
# Parameters (direct translation from MATLAB)
wall_depth = 1  # how
grid_size = 100
max_ridge_depth = 0.5
seed = "default"



#TERRAIN 1:Generate rock wall map
Lz = -10.  # Height of wall in meters
Ly = 10.  # Width (horizontal extent) of wall in meters
terrainManager = TerrainManager(generate_terrain=False)
mesh_x, mesh_y, mesh_z  = terrainManager.generate_rock_wall_map(Lz, Ly, grid_size, wall_depth, max_ridge_depth, seed) #keep default offset
##jump params
p0 = np.array([0.5, 5.5, -6]) #unit test ,  there is singularity for px = 0!
pf=  np.array([0.5, 8.5,-4])

#TERRAIN 2
# Lz = -10.  # Height of wall in meters
# Ly = 10.  # Width (horizontal extent) of wall in meters
# terrainManager   = TerrainManager(grid_size=grid_size,wall_depth =wall_depth,max_ridge_depth=max_ridge_depth, seed="default", Lz=Lz, Ly=Ly, generate_terrain=True, terrain_type="hemisphere")
# mesh_x, mesh_y, mesh_z  = terrainManager.get_mesh()
# #mesh_x, mesh_y, mesh_z  = terrainManager.generate_hemisferic_map(Lz, Ly, cz=Lz / 2, cy=Ly / 2, radius=1.5, grid_size=grid_size, x_offset = 0.01)
# #jump params
# p0 = np.array([0.5, 4.306384528321335,-2.9831819570167184]) #unit test ,  there is singularity for px = 0!
# pf=  np.array([0.5, 1.5,-7.5])


# p0 = np.array([-0.019, 3.058, -3.24]) #unit test ,  there is singularity for px = 0!
# pf=  np.array([0.19, 0.5,-1.5])

#cost map
point_lowest_cost = pf + np.array([0, 0.5, 0.5])
max_cost = 20.
cost_x, cost_y, cost_z = generateCostMapGaussian(Lz, Ly, grid_size=grid_size, gaussian_center=point_lowest_cost, max_cost = max_cost)
# cost_x, cost_y, cost_z = generateCostMap(terrainManager, number_of_patches_width=10, number_of_patches_height=10)
print(f"[DEBUG] Any NaN in cost_x: {np.any(np.isnan(cost_x))}")
print(f"[DEBUG] Any NaN in cost_y: {np.any(np.isnan(cost_y))}")
print(f"[DEBUG] Any NaN in cost_z: {np.any(np.isnan(cost_z))}")

Fleg_max = 300. #100 not converges with hemispheric
Fr_max = 190.
Fr_min = 15.
mu = 0.8
p0_adj, pf_adj, params = initOptim(p0, pf, Ly, patch_side_z=1., patch_side_y=1. , mesh_x=mesh_x,mesh_y=mesh_y, mesh_z=mesh_z)
solution = eng.optimize_cpp_mex(matlab.double(p0_adj), matlab.double(pf_adj), Fleg_max, Fr_max, Fr_min, mu, params)
ref_com  = mat_matrix2python(solution['p'])
achieved_target = mat_matrix2python(solution['achieved_target'])

print("p0 ", p0_adj)
print("target (rought integration) ", ref_com[:,-1] )
print("achieved target (fine integration)", solution['achieved_target'])
print("Fleg ", solution['Fleg'])
print("cost ", solution['cost'])
print("target error", pf-solution['achieved_target'].reshape(1,3))
print("jump duration", solution['Tf'])
print("consumed_energy", solution['consumed_energy'])
status = status_map.get(solution['problem_solved'], "unknown status")
print(f"problem converged?: {status}")

if solution['problem_solved'] not in [0,2]:
    eval_constraints(solution['c'], solution['num_constr'], solution['constr_tolerance'], debug=False)

plotStuff()




# Properly close MATLAB engine
eng.quit()
