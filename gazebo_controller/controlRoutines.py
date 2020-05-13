# Description
# File contains some necessary control algorithms for HyQ
# Author: Niraj Rathod
# Date: 19-11-2019

# Standard packages
import scipy.io
import scipy.sparse as sparse
import numpy as np
import yaml
from mathutils import *
# User defined packages
from math_tools import Math
from utils import Utils

# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
#every vector is in the wf
def quasiStaticController(conf, act_com_pose, act_com_twist,  W_contacts,  des_com_pose, des_com_twist, des_com_acc, stance_legs, ffwdOn):
    util = Utils()

    # The inertia matrix
    Inertia = np.array([[4.0745, 0.1458, -0.2245],
                             [0.1458, 11.3576, -0.0133],
                             [-0.2245, -0.0133, 12.5675]])

    # Load math functions from jet-leg
    mathJet = Math()

    Rmeas = mathJet.rpyToRot(util.angPart(act_com_pose)[util.crd["X"]], util.angPart(act_com_pose)[util.crd["Y"]], util.angPart(act_com_pose)[util.crd["Z"]])
    b_R_w =  Rmeas    
    
    # - Rotation matrix for the desired values of the orientations
    Rdes = mathJet.rpyToRot(des_com_pose[util.sp_crd["AX"]], des_com_pose[util.sp_crd["AY"]], des_com_pose[util.sp_crd["AZ"]])
    Re = Rdes.dot(Rmeas.transpose())

    err = rotMatToRotVec(Re)  # errors of the orientation w.r.t desire values

    # This is a skew symmetric matrix for (xfi-xc) in the second matrix of equation 1.3
    # corressponding to omega_dot (difference between the foothold locations
    # and COM trajectories)
    d1 = cross_mx(W_contacts[:,util.leg_map["LF"]] - util.linPart(act_com_pose))
    d2 = cross_mx(W_contacts[:,util.leg_map["RF"]] - util.linPart(act_com_pose))
    d3 = cross_mx(W_contacts[:,util.leg_map["LH"]] - util.linPart(act_com_pose))
    d4 = cross_mx(W_contacts[:,util.leg_map["RH"]] - util.linPart(act_com_pose))


    Kpcom = np.diag([conf.Kpcomx, conf.Kpcomy, conf.Kpcomz])
    Kdcom = np.diag([conf.Kdcomx, conf.Kdcomy, conf.Kdcomz])
    
    Kpbase = np.diag([conf.KpbRoll, conf.KpbPitch, conf.KpbYaw])
    Kdbase = np.diag([conf.Kdbasex, conf.Kdbasey, conf.Kdbasez])

    # Virtual model based control (Feedback Wrench)
    Wfbk = np.zeros(6)
    Wfbk[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = Kpcom.dot(util.linPart(des_com_pose) - util.linPart(act_com_pose)) + Kdcom.dot(util.linPart(des_com_twist) - util.linPart(act_com_twist))
    #the orient error is expressed in the base_frame so it should be rotated wo have the wrench in the world frame

    #TODO Jomega*util.angPart(des_com_twist)
    Wfbk[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = Kpbase.dot(b_R_w.transpose().dot(err)) + Kdbase.dot(util.angPart(des_com_twist) - util.angPart(act_com_twist))

    # FFd linear
    if (ffwdOn):    
        ffdLinear = conf.robotMass * util.linPart(des_com_acc) 
        #TODO des_omega_dot = R * (J_omega * des_euler_rate_dot + J_omega_dot*des_euler_rate);
#        ffdAngular = (b_R_w.transpose().dot(Inertia)).dot(b_R_w.dot(util.angPart(des_com_acc)))   
        ffdAngular =Inertia.dot(util.angPart(des_com_acc))
        Wffwd = - np.hstack([ffdLinear, ffdAngular])
    else:
        Wffwd = np.zeros(6)

    # Gravity Wrench
    Wg = np.zeros(6)
    mg = conf.robotMass * np.array([0, 0, conf.gravity])
    Wcom = (b_R_w.T).dot(np.array([conf.Bcom_x, conf.Bcom_y, conf.Bcom_z]))    
    Wg[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = - mg
    Wg[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = - np.cross(Wcom, mg)
    
    
    # Total Wrench
    TotWrench =   Wg + Wffwd +  Wfbk 
 
    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map["LF"]] * np.ones(3), stance_legs[util.leg_map["RF"]] * np.ones(3),
                              stance_legs[util.leg_map["LH"]] * np.ones(3), stance_legs[util.leg_map["RH"]] * np.ones(3)]))

    # A matrix used in equation 1.17 only for the stance feet
    AtmPre = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    Atm = AtmPre.dot(S_mat)

    # Map the total Wrench to grf
    u_vpd = np.linalg.pinv(Atm).dot(TotWrench)

    # Mutliply with stance matrix
    u = u_vpd

    return u