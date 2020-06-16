# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 14:15:50 2020

@author: mfocchi
"""

from optimTools import quadprog_solve_qp
import numpy as np

#∥Mx−b∥2# 
# G = Mt*M
# g = MT*b
#0.5 xT*G*x + gT*x
#s.t. Cx≤d
#     Ax=b




#do this with numpy array
M = np.array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.array([3., 2., 3.])

G = np.dot(M.T, M)
g = -np.dot(M.T, b)

C = np.array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
d = np.array([3., 2., -2.]).reshape((3,))

result = quadprog_solve_qp(G, g, C, d)

print result 

#do the same with matrix array
M = np.matrix([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.matrix([3., 2., 3.]).T
G = M.T*M
g = -M.T*b
C = np.matrix([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
d = np.matrix([3., 2., -2.]).reshape((3,))
result = quadprog_solve_qp(G, g, C, d)

print result

#with equalituies and inequalities 
M = np.matrix([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
b = np.matrix([3., 2., 3.]).T
G = M.T*M
g = -M.T*b

#3 ineq constraints 
C = np.matrix([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
d = np.matrix([3., 2., -2.]).reshape((3,))

#1 ineq constraint
A = np.matrix([[0.5, 1., 2.]])
b = np.matrix([3. ]).reshape((1,))

result = quadprog_solve_qp(G, g, C, d,A , b)

print result