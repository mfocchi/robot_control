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

mass = 4.976936060000001
Fleg_max = 300.
Fr_max = 90.

#landing
# Fleg_max = 600.
# Fr_max = 300.
# mass = 15.0246

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
# the result of this test should not be compared with the matlab one

# for i in range(10):
#     start = time.time()
#     x = eng.optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, params)
#     duration = time.time() - start
#     print("duration", duration)


eng.exit
eng.quit()
