import Clothoids
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
curve = Clothoids.ClothoidCurve("curve")
curve.build_G1(0.0, 0.0, 0.0, 4.0, 4.0, 0.0)

#https://ebertolazzi.github.io/Clothoids/doxygen/html/class_g2lib_1_1_clothoid_curve.html#a519e61f7a826e77d199a5298ef05b3c9
#add elements to clothoids_interface.hh
# pip install -e .

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

# plt.figure()
# plt.plot(xy[:, 0], xy[:, 1])
# plt.show()
# plt.figure()
# plt.plot(dxdy[:, 0], dxdy[:, 1])
# plt.show()
# plt.figure()
# plt.plot(theta)
# plt.show()
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

plt.figure()
plt.plot(xy_log[:, 0], xy_log[:, 1])
plt.show()

