# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
from scipy.stats import qmc
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
#generate in polar coords
radius_min = 1.
radius_max = 4.

#quasi montecarlo
sampler = qmc.Halton(d=3, scramble=True, seed=0)
#since we are going to remove the samples out of the circle I need to generate a bit more samples
n_samples = int(np.floor(100*math.sqrt(2)))
sample = sampler.random(n=n_samples)
xy = radius_max * (2*sample[:, :2]-1)
phi = sample[:, 2] *2*np.pi
plt.figure(0)
plt.plot(xy[:,0], xy[:,1], "or")
plt.show()

# plt.figure(1)
# np.random.seed(0)
# #comparison with uniform sampling
# for i in range(100):
#     x = radius_max * np.random.uniform(low=0, high=1, size=1)
#     y = radius_max * np.random.uniform(low=0, high=1, size=1)
#     sample[i,0] = x
#     sample[i,1] = y
# plt.plot(sample[:, 0], sample[:, 1], "ob")
# plt.show()

plt.figure(1)
sample_ok = np.zeros((2))
phi_ok = np.zeros((1))
#remove samples out of the cilinder
for i in range(sample.shape[0]):
    # remove sample out of the circle
    radius = np.linalg.norm(xy[i,:])
    if (radius>radius_min) and (radius<radius_max):
        sample_ok = np.vstack((sample_ok, xy[i, :2]))
        phi_ok = np.vstack((phi_ok, phi[i]))
        #plot orientation
        plt.quiver(xy[i, 0], xy[i, 1], np.cos(phi[i]), np.sin(phi[i]))

#plot circles
angle = np.linspace(0, np.pi*2, 100)
p_max = np.array([radius_max*np.cos(angle), radius_max*np.sin(angle)])
p_min = np.array([radius_min*np.cos(angle), radius_min*np.sin(angle)])

plt.figure(1)
plt.plot(sample_ok[:, 0], sample_ok[:, 1], "ob")
plt.plot(p_max[0, :], p_max[1,:], "r")
plt.plot(p_min[0, :], p_min[1,:], "r")
plt.show()

