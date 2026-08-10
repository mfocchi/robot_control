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
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
import matplotlib.pyplot as plt
eng = matlab.engine.start_matlab()


def initOptim(p0, pf):
    mass = 5.
    params = {}
    params['m'] = mass
    anchor_distance = 5.
    params['num_params'] = 4.
    params['int_method'] = 'rk4'
    params['N_dyn'] = 30.
    params['FRICTION_CONE'] = 1.
    params['int_steps'] = 5.
    params['b'] = anchor_distance
    params['p_a1'] = matlab.double([0.,0.,0.]).reshape(3,1)
    params['p_a2'] = matlab.double([0.,params['b'],0.]).reshape(3,1)
    params['g'] = 9.81
    params['w1']= 1. # smooth
    params['w2']= 1. # hoist work
    params['w3']= 0.
    params['w4']= 0.
    params['w5']= 0.
    params['w6']= 0.
    params['T_th'] =  0.05
    params['obstacle_avoidance'] = 'mesh'
    params['jump_clearance'] = 1.

    # Interpolator (note: z must be increasing — here from -10 to 0)
    p0[0] = terrainManager.wall_surface_eval(p0[2],p0[1],  mesh_x, mesh_y, mesh_z)
    pf[0] = terrainManager.wall_surface_eval(pf[2],pf[1],  mesh_x, mesh_y, mesh_z)
    # compute consistent normal
    normal = terrainManager.wall_normal_eval(p0[2],p0[1],  mesh_x, mesh_y, mesh_z )
    params['mesh_x'] = mesh_x
    params['mesh_y'] = mesh_y
    params['mesh_z'] = mesh_z
    params['contact_normal'] = matlab.double(normal)
    return p0, pf, params


#terrain
# Parameters (direct translation from MATLAB)
wall_depth = 1  # how
grid_size = 100
max_ridge_depth = 0.5
seed = "default"
Lz = -20  # Height of wall in meters
Ly = 5  # Width (horizontal extent) of wall in meters
# Generate rock wall map
terrainManager = TerrainManager(generate_terrain=False)

#TERRAIN 1
mesh_x, mesh_y, mesh_z  = terrainManager.generate_rock_wall_map(Lz, Ly, grid_size, wall_depth, max_ridge_depth, seed, x_offset=-0.5)
###jump params
p0 = np.array([0.0, 2.5, -6]) #unit test ,  there is singularity for px = 0!
#p0 =  matlab.double([0.27753 , 2.51893, -6.09989]) # actual used p0 = np.array([0.28,  2.5, -6.10104])
pf=  np.array([0.0, 4,-4])

#TERRAIN 2
<<<<<<< HEAD
# mesh_x, mesh_y, mesh_z  = terrainManager.generate_hemisferic_map(Lz, Ly, cz=Lz / 2, cy=Ly / 2, radius=1.5, grid_size=grid_size, x_offset = 0.1)
# # #jump params
=======
terrainManager   = TerrainManager(grid_size=grid_size,wall_depth =wall_depth,max_ridge_depth=max_ridge_depth, seed="default", Lz=Lz, Ly=Ly, generate_terrain=True, terrain_type="hemisphere")
mesh_x, mesh_y, mesh_z  = terrainManager.get_mesh()
# mesh_x, mesh_y, mesh_z  = terrainManager.generate_hemisferic_map(Lz, Ly, cz=Lz / 2, cy=Ly / 2, radius=1.5, grid_size=grid_size, x_offset = 0.1)
# #jump params
p0 = np.array([0.0,2.5,-7.5])
pf=  np.array([0.0,2.5,-15.5])
>>>>>>> 915e06e (update: splite in two folder , the optimization script inside patch and without the optimization inside patch (opti patch))
# p0 = np.array([0.0, 2.5, -6]) #unit test ,  there is singularity for px = 0!
# pf=  np.array([0.0, 4,-12])

Fleg_max = 300.
Fr_max = 90.
Fr_min = 0.
mu = 0.8
p0_adj, pf_adj, params = initOptim(p0, pf)
solution = eng.optimize_cpp_mex(matlab.double(p0_adj), matlab.double(pf_adj), Fleg_max, Fr_max, Fr_min, mu, params)
ref_com  = mat_matrix2python(solution['p'])
achieved_target = mat_matrix2python(solution['achieved_target'])
print("p0 ", p0_adj)
print("target (rought integration) ", ref_com[:,-1] )
print("achieved target (fine integration)", solution['achieved_target'])
print("Fleg ", solution['Fleg'])
print("target error", pf-solution['achieved_target'].reshape(1,3))
print("jump duration", solution['Tf'])
print("consumed_energy", solution['consumed_energy'])
print("problem converged: ", solution['problem_solved'] in [0, 2])

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Surface plot
ax.plot_surface(params['mesh_x'], params['mesh_y'], params['mesh_z'],  alpha=0.7, cmap='Blues', edgecolor='k', linewidth=0.2)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_xlim([0, 4])
ax.set_ylim([0, 7])
ax.set_zlim([-10, 2])
# Alternative method using set_box_aspect for proportional scaling
# ax.set_box_aspect([x_range, y_range, z_range])
ax.view_init(elev=20, azim=9)
ax.scatter(p0_adj[0], p0_adj[1], p0_adj[2], color='blue', s=500)  # s = size
ax.scatter(achieved_target[0], achieved_target[1], achieved_target[2], color='red', s=500)  # s = size
ax.plot3D(ref_com[0,:], ref_com[1,:],ref_com[2,:],color='red', linewidth=2.5)
plt.show() 
eng.quit()