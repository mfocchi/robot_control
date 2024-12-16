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

solution = eng.optimize_cpp_mex(matlab.double(p0), matlab.double(pf), Fleg_max, Fr_max, mu, params)
print(solution['achieved_target'])
print(solution['Tf'])
print(solution['Fr_l'])
print(solution['Fr_r'])
print(solution['Fleg'])
# the result of this test should not be compared with the matlab one

# [[0.5278959907336792],[3.9950093586138196],[-4.000958931560958]]
# 1.2888886080707584
# [[-5.82144346061948e-15,-5.430617221704646,-5.121549567550131,-6.908234851938786,-4.761284592588307,-4.475701122230117,-4.649864058893552,-5.534857473002324,-10.440252175922122,-15.854026169455482,-21.89840115719842,-28.36877511399877,-35.03087214645125,-38.71759681295521,-40.27857405868862,-39.46949486453046,-38.5155028450158,-37.428288164772546,-36.21273765974311,-34.88011508964135,-33.45756403431701,-31.94450380965398,-30.344184412664248,-28.73719512146946,-27.08554055673233,-25.49135641346178,-23.91504146369696,-22.022936882451145,-19.773536029945017,-90.0]]
# [[0.0,-39.301932291121986,-40.42180415399788,-41.79009064565616,-43.4057965184382,-45.20906575923831,-47.228084924985765,-49.43085431515452,-51.79505409635683,-54.29667007004444,-56.93580524047053,-59.612048858442485,-62.35959842318748,-65.14285270818064,-66.43032416976334,-60.71996925755153,-59.4019793140216,-58.72024636402011,-57.622641463539416,-54.88758182468992,-51.93417602927097,-48.75123513508619,-45.376890931414046,-41.8136827416848,-38.09470837086405,-34.27131285727154,-30.352453347303054,-26.371337228196566,-22.420359935210552,-90.0]]
# [[115.27361606655302],[27.068135456492676],[17.715704567779838]]

# for i in range(10):
#     start = time.time()
#     x = eng.optimize_cpp_mex(p0,  pf, Fleg_max, Fr_max, mu, params)
#     duration = time.time() - start
#     print("duration", duration)


eng.exit
eng.quit()
