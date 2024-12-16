#!/usr/bin/env python
"""
Sample script that uses the MagicSquarePkg package created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""
import matlab.engine

import time
import scipy.io.matlab as mio
import numpy as np

np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)

eng = matlab.engine.start_matlab()

#jump params
p0 =  matlab.double([0.5, 0.5, -6]) #unit test ,  there is singularity for px = 0!
#p0 =  matlab.double([0.27753 , 2.51893, -6.09989]) # actual used p0 = np.array([0.28,  2.5, -6.10104])
#pf=  matlab.double([0.5, 4,-6])
pf=  matlab.double([1.5, 2.5,-6])

Fleg_max = 300.
Fr_max = 120.
mu = 0.8
mass = 4.976936060000001

params = {}
params['jump_clearance'] = 1.
params['m'] = mass
params['obstacle_avoidance'] = True
#params['obstacle_location'] = matlab.double([-0.5, 3.,-7.5]).reshape(3,1)
params['obstacle_location'] = matlab.double([0, 2.5,-6.]).reshape(3,1)
params['obstacle_size'] = matlab.double([1.5, 1.5, 0.866]).reshape(3,1)
params['num_params'] = 4.
params['int_method'] = 'rk4'
params['N_dyn'] = 30.
params['FRICTION_CONE'] = 1.
params['int_steps'] = 5.
params['contact_normal'] = matlab.double([1.,0.,0.]).reshape(3,1)
anchor_distance = 5.
params['b'] = anchor_distance
params['p_a1'] = matlab.double([0.,0.,0.]).reshape(3,1)
params['p_a2'] = matlab.double([0.,params['b'],0.]).reshape(3,1)
params['g'] = 9.81
params['w1']= 1.
params['w2']= 0.
params['w3']= 0.
params['w4']= 0.
params['w5']= 0.
params['w6']= 0.
params['T_th'] =  0.05




solution = eng.optimize_cpp_mex(p0, pf, Fleg_max, Fr_max, mu, params)
print("TARGET:",solution['achieved_target'])
print("FLEG:",solution['Fleg'])
print("Tf:",solution['Tf'])
print(solution['Fr_l'])
print(solution['Fr_r'])


eng.exit
eng.quit()
