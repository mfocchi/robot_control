#!/usr/bin/env python
"""
Sample script that uses the MagicSquarePkg package created using
MATLAB Compiler SDK.

Refer to the MATLAB Compiler SDK documentation for more information.
"""
import matlab.engine
import numpy as np
import math
from base_controllers.utils.math_tools import Math

np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)

eng = matlab.engine.start_matlab()

mass = 4.976936060000001
Fleg_max = 300.
Fr_max = 90.
Fr_min = 0.
#landing
# Fleg_max = 600.
# Fr_max = 300.
# mass = 15.07

wall_inclination = 0.0
math_utils = Math()
w_R_wall= math_utils.eul2Rot(np.array([0, -wall_inclination, 0]))
normal = w_R_wall[:, 0].copy()

mu = 0.8
params = {}
params['jump_clearance'] = 1.
params['m'] = mass
params['obstacle_avoidance'] = False
params['obstacle_location'] = matlab.double([0, 2.5,-6]).reshape(3,1)
params['obstacle_size'] = matlab.double([1.5, 1.5, 0.866]).reshape(3,1)
anchor_distance = 5.
params['num_params'] = 4.
params['int_method'] = 'rk4'
params['N_dyn'] = 30.
params['FRICTION_CONE'] = 1.
params['int_steps'] = 5.
params['contact_normal'] = matlab.double(normal).reshape(3,1)
params['b'] = anchor_distance
params['p_a1'] = matlab.double([0.,0.,0.]).reshape(3,1)
params['p_a2'] = matlab.double([0.,params['b'],0.]).reshape(3,1)
params['g'] = 9.81
params['w1']= 1. # smooth
params['w2']= 1. # hoist work
params['w3']= 1.
params['w4']= 1.
params['w5']= 1.
params['w6']= 1.
params['T_th'] =  0.05

#jump params
p0 = np.array([0.5, 2.5, -6]) #unit test ,  there is singularity for px = 0!
#p0 =  matlab.double([0.27753 , 2.51893, -6.09989]) # actual used p0 = np.array([0.28,  2.5, -6.10104])
pf=  np.array([0.5, 4,-4])
#compute wall consistent target
pf[0]+= (-pf[2])*math.tan(wall_inclination)

if params['obstacle_avoidance'] == True:
    p0 =  np.array([0.5, 0.5, -6]) #unit test ,  there is singularity for px = 0!
    #p0 =  matlab.double([0.27753 , 2.51893, -6.09989]) # actual used p0 = np.array([0.28,  2.5, -6.10104])
    pf = np.array([0.5, 4.5,-6])

solution = eng.optimize_cpp_mex(matlab.double(p0), matlab.double(pf), Fleg_max, Fr_max, Fr_min, mu, params)
print(solution['achieved_target'])
print(solution['Tf'])
print(solution['Fr_l'])
print(solution['Fr_r'])
print(solution['Fleg'])
# the result of this test should not be compared with the matlab one
# [[0.5039011688405084],[4.018643091498619],[-3.9477347274974726]]
# 1.1599999540379786
# [[-0.003306769589496066,-0.005148804077568659,-0.019113438855408192,-0.0025370799282028985,-0.06386635140485786,-0.025641544364741087,-0.0017809105177177963,-1.5544602625505282,-0.03427849840657173,-14.558766348654311,-21.74595121978507,-29.61805872099773,-41.434107366871466,-54.129601936206434,-64.85955944426546,-57.608905857262386,-55.28159799756689,-55.298948520430606,-45.21023405989006,-40.70626109824182,-41.721593061602725,-33.3450545964003,-27.744614447747843,-17.54607734747951,-19.82745296019226,-10.546434980635754,-4.822912020542907,-2.2664912126144414,-10.43211929940204,-16.654712458669607]]

# for i in range(10):
#     start = time.time()
#     x = eng.optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, params)
#     duration = time.time() - start
#     print("duration", duration)


eng.exit
eng.quit()
