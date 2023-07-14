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

#1 - call a script
#r = eng.pow_script(nargout=0)#Specifying nargout=0 is required. Although the script prints output, it returns no output arguments to Python.

# 2 - call a function
# base = np.asarray([1.0,2.0,3.0,4.0])
# exp = 2.0
# ret = eng.pow_fun(matlab.double(base.tolist()),exp)
# print(ret)

#3call the mex
matvars = mio.loadmat('../test_matlab2.mat', squeeze_me=True, struct_as_record=False)
print("mat loaded")
start_mpc= 3-1
N_dyn = len(matvars['solution'].time)
mpc_N = int(0.4*N_dyn)
Fr_max =50.
ref_com = matlab.double(matvars['solution'].p[:,start_mpc:start_mpc+mpc_N].tolist())
Fr_l0 = matlab.double(matvars['solution'].Fr_l[start_mpc:start_mpc+mpc_N].tolist())
Fr_r0 = matlab.double(matvars['solution'].Fr_r[start_mpc:start_mpc+mpc_N].tolist())
actual_t = matlab.double(matvars['solution'].time[start_mpc])
actual_state = matlab.double([0.0937,    6.6182,    6.5577,    0.1738,    1.1335,    1.3561]).reshape(6,1)


params = {}
params['int_method'] = 'rk4'
params['int_steps'] = 5.
params['contact_normal'] = matlab.double([1.,0.,0.]).reshape(3,1)
params['b'] = 5.
params['p_a1'] = matlab.double([0.,0.,0.]).reshape(3,1)
params['p_a2'] = matlab.double([0.,params['b'],0.]).reshape(3,1)
params['g'] = 9.81
params['m']= 5.08
params['w1']= 1.
params['w2']= 1.
params['mpc_dt'] = matlab.double(matvars['solution'].Tf / (N_dyn-1))

x = eng.optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params)
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
