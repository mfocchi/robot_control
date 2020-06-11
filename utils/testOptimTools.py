# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 14:15:50 2020

@author: mfocchi
"""

from optimTools import quadprog_solve_qp
import numpy as np

#∥Mx−b∥2# 
# P = Mt*M
# q = MT*b
#0.5 xT*P*x + qT*x

M = np.array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.array([3., 2., 3.])
P = np.dot(M.T, M)
q = -np.dot(M.T, b)
G = np.array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
h = np.array([3., 2., -2.]).reshape((3,))

result = quadprog_solve_qp(P, q, G, h)

print result 

#do the same with matrix array
M = np.matrix([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.matrix([3., 2., 3.]).T
P = M.T*M
q = -M.T*b
G = np.matrix([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
h = np.matrix([3., 2., -2.]).reshape((3,))
result = quadprog_solve_qp(P, q, G, h)

print result