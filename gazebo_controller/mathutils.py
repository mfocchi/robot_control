import numpy as np
import scipy as sp
import math as math
import matplotlib.pyplot as plt
from utils import *

def cross_mx(v):
    mx =np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return mx

def skew_simToVec(Ra):
    # This is similar to one implemented in the framework
    v = np.zeros(3)
    v[0] = 0.5*(Ra[2,1] - Ra[1,2])
    v[1] = 0.5*(Ra[0,2] - Ra[2,0])
    v[2] = 0.5*(Ra[1,0] - Ra[0,1])

    return v

def rotMatToRotVec(Ra):
    c = 0.5 * (Ra[0, 0] + Ra[1, 1] + Ra[2, 2] - 1)
    w = -skew_simToVec(Ra)
    s = np.linalg.norm(w) # w = sin(theta) * axis

    if abs(s) <= 1e-10:
        err = np.zeros(3)
    else:
        angle = math.atan2(s, c)
        axis = w / s
        err = angle * axis
    return err

def rpyToEarInv(r,p,y):

    #convention yaw pitch roll

    phi = r
    theta = p
    psi = y

    #Inverse of Euler angle rates matrix to get rate of change of Euler angles in the base coords
    EarInv = np.array([[math.cos(psi)/math.cos(theta),        math.sin(psi)/math.cos(theta),         0],
                       [-math.sin(psi),                  math.cos(psi),                  0],
                       [math.cos(psi)*math.tan(theta),        math.sin(psi)*math.tan(theta) ,        1]])
    return EarInv


def invincasadi(A):
    # Determinant of matrix A
    sb1=A[0,0]*((A[1,1]*A[2,2])-(A[1,2]*A[2,1]))
    sb2=A[0,1]*((A[1,0]*A[2,2])-(A[1,2]*A[2,0]))
    sb3=A[0,2]*((A[1,0]*A[2,1])-(A[1,1]*A[2,0]))

    Adetr=sb1-sb2+sb3
#    print(Adetr)
    # Transpose matrix A
    TransA=A.T

    # Find determinant of the minors
    a01=(TransA[1,1]*TransA[2,2])-(TransA[2,1]*TransA[1,2])
    a02=(TransA[1,0]*TransA[2,2])-(TransA[1,2]*TransA[2,0])
    a03=(TransA[1,0]*TransA[2,1])-(TransA[2,0]*TransA[1,1])
    
    a11=(TransA[0,1]*TransA[2,2])-(TransA[0,2]*TransA[2,1])
    a12=(TransA[0,0]*TransA[2,2])-(TransA[0,2]*TransA[2,0])
    a13=(TransA[0,0]*TransA[2,1])-(TransA[0,1]*TransA[2,0])

    a21=(TransA[0,1]*TransA[1,2])-(TransA[1,1]*TransA[0,2])
    a22=(TransA[0,0]*TransA[1,2])-(TransA[0,2]*TransA[1,0])
    a23=(TransA[0,0]*TransA[1,1])-(TransA[0,1]*TransA[1,0])

    # Inverse of determinant
    invAdetr=(float(1)/Adetr)
#    print(invAdetr)
    # Inverse of the matrix A
    invA=np.array([[invAdetr*a01, -invAdetr*a02, invAdetr*a03], [-invAdetr*a11, invAdetr*a12, -invAdetr*a13], [invAdetr*a21, -invAdetr*a22, invAdetr*a23]])

    # Return the matrix
    return invA


#/**
#brief motionVectorTransform Tranforms twists from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#
def motionVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd["AX"]:utils.sp_crd["AX"] + 3 ,   utils.sp_crd["AX"]: utils.sp_crd["AX"] + 3] = rotationMx
    b_X_a[utils.sp_crd["LX"]:utils.sp_crd["LX"] + 3 ,   utils.sp_crd["AX"]: utils.sp_crd["AX"] + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd["LX"]:utils.sp_crd["LX"] + 3 ,   utils.sp_crd["LX"]: utils.sp_crd["LX"] + 3] = rotationMx
    return b_X_a


#
#brief forceVectorTransform Tranforms wrenches from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#


def forceVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd["AX"]:utils.sp_crd["AX"] + 3 ,   utils.sp_crd["AX"]: utils.sp_crd["AX"] + 3] = rotationMx
    b_X_a[utils.sp_crd["AX"]:utils.sp_crd["AX"] + 3 ,   utils.sp_crd["LX"]: utils.sp_crd["LX"] + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd["LX"]:utils.sp_crd["LX"] + 3 ,   utils.sp_crd["LX"]: utils.sp_crd["LX"] + 3] = rotationMx
    return b_X_a

def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print "Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds."
    else:
        print "Toc: start time not set"
def close_all():
    plt.close('all')