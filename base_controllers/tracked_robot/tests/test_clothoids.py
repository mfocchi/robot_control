import Clothoids
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
curve = Clothoids.ClothoidCurve("curve")
curve.build_G1(0.0, 0.0, 0.0, 4.0, 4.0, 0.0)

#1) description
# 2) clone git@github.com:SebastianoTaddei/Clothoids.git
# 3) commit b744b9f21213c75c183f1b9966794ac9baf3832a (HEAD -> python_bindings, origin/python_bindings)
# Author: Sebastiano Taddei <sebastianotaddei@gmail.com>
# Date:   Thu Nov 14 10:32:11 2024 +0100
# 4) you nee to add elements to clothoids_interface.hh
# 5) pip install -e .

values = np.arange(0, curve.length(), 0.01, dtype=np.float64)
xy = np.zeros((values.size, 2))
dxdy = np.zeros((values.size, 2))
theta = np.zeros((values.size, 1))
dtheta = np.zeros((values.size, 1))
for i in range(values.size):
    xy[i, :] = curve.eval(values[i])
    theta[i] = curve.theta(values[i])
    dxdy[i, :] = curve.eval_D(values[i])
    dtheta[i] = curve.theta_D(values[i])

plt.figure(1)
plt.plot(xy[:, 0], xy[:, 1])
plt.title('xy plot')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
plt.figure(2)
plt.plot(theta)
plt.xlabel('sample (discretization 0.01 m)')
plt.title('orientation')
plt.figure(3)
plt.plot(dtheta)
plt.title('curvature (linear)')
plt.xlabel('sample')
plt.show()
#
#
#map to time
long_v = 0.4
T_tot = curve.length()/long_v
dt = 0.001
number_of_samples = int(np.floor(T_tot/dt))
values = np.arange(0, curve.length(), curve.length()/number_of_samples, dtype=np.float64)
xy = np.zeros((values.size, 2))
dxdy = np.zeros((values.size, 2))
theta = np.zeros((values.size, 1))
dtheta = np.zeros((values.size, 1))
dtheta = np.zeros((values.size, 1))
time = np.zeros((values.size, 1))
for i in range(values.size):
    xy[i, :] = curve.eval(values[i])
    theta[i] = curve.theta(values[i])
    dxdy[i, :] = curve.eval_D(values[i])
    dtheta[i] = curve.theta_D(values[i])
    time[i] = i*dt
#map to time (only velocities are affected)
dx_dy = dxdy* long_v
omega = dtheta*long_v #omega

#integrate back states
xy_int = [0,0]
theta_int = 0
xy_log = np.zeros((values.size, 2))
theta_log = np.zeros((values.size, 1))
for i in range(number_of_samples):
    xy_int += dt*dx_dy[i]
    theta_int += dt*omega[i]
    xy_log[i,:] = xy_int
    theta_log[i, :] = theta_int

plt.figure(5)

plt.plot(xy_log[:, 0], xy_log[:, 1], '-o')
plt.title('xy plot after time integration')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

