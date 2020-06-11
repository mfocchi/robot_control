# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 14:12:33 2020

@author: mfocchi
"""

import numpy as np
from quadprog import solve_qp  
#
#minimize (1/2)x.T*P*x+q.Tx
#s.t. Gx≤h
#     Ax=b


def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
    qp_G = .5 * (P + P.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -numpy.vstack([A, G]).T
        qp_b = -numpy.hstack([b, h])
        meq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        meq = 0
	
    if isinstance(qp_a, np.matrix) or isinstance(qp_b, np.matrix):								
        return solve_qp(qp_G, qp_a.A1, qp_C, qp_b.A1, meq)[0]     
    else:
        return solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]					