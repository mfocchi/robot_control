#!/usr/bin/env python
"""
Sample script that uses the MagicSquarePkg package created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""
import matlab.engine
import numpy as np
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python

np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)

eng = matlab.engine.start_matlab()


mass = 4.976936060000001
Fleg_max = 300.
Fr_max = 90.
Fr_max_mpc =50.

#landing
# Fleg_max = 600.
# Fr_max = 300.
# mass = 15.0246
#Fr_max_mpc =150.

# first optimize the traj
mu = 0.8
params = {}
params['jump_clearance'] = 1.
params['m'] = mass
params['obstacle_avoidance'] = False
anchor_distance = 5.
params['num_params'] = 4.
params['int_method'] = 'rk4'
params['N_dyn'] = 30.
params['FRICTION_CONE'] = 5.
params['int_steps'] = 5.
params['contact_normal'] = matlab.double([1.,0.,0.]).reshape(3,1)
params['b'] = anchor_distance
params['p_a1'] = matlab.double([0.,0.,0.]).reshape(3,1)
params['p_a2'] = matlab.double([0.,params['b'],0.]).reshape(3,1)
params['g'] = 9.81
params['m']= mass
params['w1']= 1.
params['w2']= 1.
params['w3']= 1.
params['w4']= 1.
params['w5']= 1.
params['w6']= 1.
params['T_th'] =  0.05

#jump params
p0 =  matlab.double([0.5, 2.5, -6]) # there is singularity for px = 0!
pf=  matlab.double([0.5, 4,-4])
solution = eng.optimize_cpp_mex(p0, pf, Fleg_max, Fr_max, mu, params)
print(solution['achieved_target'])
print(solution['Tf'])
print(solution['Fr_l'])
print(solution['Fr_r'])


# then call the mpc
start_mpc= 1
mpc_N = int(0.4*params['N_dyn'])

ref_time = matlab.double(mat_matrix2python(solution['time']).tolist())
actual_t = ref_time[0][start_mpc]

ref_com = matlab.double(mat_matrix2python(solution['p'])[:, start_mpc:start_mpc + mpc_N].tolist())
Fr_l0 = matlab.double(mat_matrix2python(solution['Fr_l'])[:, start_mpc:start_mpc + mpc_N].tolist())
Fr_r0 = matlab.double(mat_matrix2python(solution['Fr_r'])[:, start_mpc:start_mpc + mpc_N].tolist())
actual_state = matlab.double([0.0937,    6.6182,    6.5577,    0.1738,    1.1335,    1.3561]).reshape(6,1)


params_mpc = {}
params_mpc['int_method'] = 'rk4'
params_mpc['int_steps'] = 5.
params_mpc['contact_normal'] = matlab.double([1.,0.,0.]).reshape(3,1)
params_mpc['b'] = 5.
params_mpc['p_a1'] = matlab.double([0.,0.,0.]).reshape(3,1)
params_mpc['p_a2'] = matlab.double([0.,params['b'],0.]).reshape(3,1)
params_mpc['g'] = 9.81
params_mpc['m']= mass
params_mpc['w1']= 1.
params_mpc['w2']= 1.
params_mpc['mpc_dt'] = matlab.double(solution['Tf'] / (params['N_dyn']-1))

x = eng.optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max_mpc, mpc_N, params_mpc)
print("x", x)

# for i in range(10):
#
#     start = time.time()
#     x = eng.optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params)
#     #print("x", x)
#     duration = time.time() - start
#     print("duration", duration)

eng.exit
eng.quit()
