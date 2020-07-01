# Description
# File contains some necessary control algorithms for HyQ
# Author: Niraj Rathod
# Date: 19-11-2019

# Standard packages
import scipy.io
import scipy.sparse as sparse
import numpy as np
import yaml
from gazebo_controller.math_tools import *
# User defined packages
from gazebo_controller.math_tools import Math
from gazebo_controller.utils import Utils

from optimTools import quadprog_solve_qp
from scipy.linalg import block_diag

def computeVirtualImpedanceWrench(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, B_base_to_com, ffwdOn, isBaseControlled):
    util = Utils()

    # The inertia matrix
    Inertia = np.array([     [conf.robotInertia.Ixx, conf.robotInertia.Ixy, conf.robotInertia.Ixz],
                             [conf.robotInertia.Ixy, conf.robotInertia.Iyy,  conf.robotInertia.Iyz],
                             [conf.robotInertia.Ixz,  conf.robotInertia.Iyz, conf.robotInertia.Izz]])
    # Load math functions from jet-leg
    mathJet = Math()

    # ESERCISE 2.1: Feedback wrench
    Kp_com = np.diag([conf.Kpcomx, conf.Kpcomy, conf.Kpcomz])
    Kd_com = np.diag([conf.Kdcomx, conf.Kdcomy, conf.Kdcomz])
    Kp_trunk = np.diag([conf.KpRoll, conf.KpPitch, conf.KpYaw])
    Kd_trunk = np.diag([conf.KdRoll, conf.KdPitch, conf.KdYaw])

    Wfbk = np.zeros(6)
    # linear part                
    Wfbk[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = Kp_com.dot(util.linPart(des_pose) - util.linPart(act_pose)) + Kd_com.dot(util.linPart(des_twist) - util.linPart(act_twist))
    
    # angular part                
    # actual orientation 
    Rmeas = mathJet.rpyToRot(util.angPart(act_pose))
    b_R_w =  Rmeas    
    # - Rotation matrix for the desired values of the orientations
    Rdes = mathJet.rpyToRot(util.angPart(des_pose))
    Re = Rdes.dot(Rmeas.transpose())
    # errors of the orientation                 
    err = rotMatToRotVec(Re)                  
                
    #the orient error is expressed in the base_frame so it should be rotated wo have the wrench in the world frame
    #TODO Jomega*util.angPart(des_com_twist)
    Wfbk[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = Kp_trunk.dot(b_R_w.transpose().dot(err)) + Kd_trunk.dot(util.angPart(des_twist) - util.angPart(act_twist))
#
    #EXERCISE 4: Compute graviy wrench
    Wg = np.zeros(6)            
    mg = conf.robotMass * np.array([0, 0, conf.gravity])
    Wg[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = - mg
    #in case you are closing the loop on base frame
    if (isBaseControlled):                 
        W_base_to_com = (b_R_w.T).dot(B_base_to_com)       
        Wg[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = - np.cross(W_base_to_com, mg)
                
    # EXERCISE 5: Feed-forward wrench
    if (ffwdOn):    
        ffdLinear = conf.robotMass * util.linPart(des_acc) 
        #TODO des_omega_dot = R * (J_omega * des_euler_rate_dot + J_omega_dot*des_euler_rate);
#        ffdAngular = (b_R_w.transpose().dot(Inertia)).dot(b_R_w.dot(util.angPart(des_acc)))   
        ffdAngular =Inertia.dot(util.angPart(des_acc))
        Wffwd = - np.hstack([ffdLinear, ffdAngular])
    else:
        Wffwd = np.zeros(6)

    return  Wffwd, Wfbk, Wg           
                 

# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
#every vector is in the wf
def projectionBasedController(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, com, ffwdOn, isBaseControlled):
    util = Utils()
                       
    # compute virtual impedances  
    Wffwd, Wfbk, Wg = computeVirtualImpedanceWrench(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, com, ffwdOn, isBaseControlled)                
    # Total Wrench
    TotWrench =  Wffwd+ Wfbk+ Wg                     
                
    # EXERCISE 3.2 : Compute mapping to grfs
    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map["LF"]] * np.ones(3), stance_legs[util.leg_map["RF"]] * np.ones(3),
                              stance_legs[util.leg_map["LH"]] * np.ones(3), stance_legs[util.leg_map["RH"]] * np.ones(3)]))
    # This is a skew symmetric matrix for (xfi-xc)  corressponding  toe difference between the foothold locations
    # and COM trajectories)
    d1 = cross_mx(W_contacts[:,util.leg_map["LF"]] - util.linPart(act_pose))
    d2 = cross_mx(W_contacts[:,util.leg_map["RF"]] - util.linPart(act_pose))
    d3 = cross_mx(W_contacts[:,util.leg_map["LH"]] - util.linPart(act_pose))
    d4 = cross_mx(W_contacts[:,util.leg_map["RH"]] - util.linPart(act_pose))
    # Compute Jb^T
    JbT = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    #nullify columns relative to legs that are not in contact                                                                                                #
    JbT = JbT.dot(S_mat)

    # Map the total Wrench to grf
    des_grf = np.linalg.pinv(JbT, 1e-04).dot(TotWrench)
                
                                                                
                                                                
                                                                
                                                                
    return des_grf, Wffwd, Wfbk, Wg

# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
#every vector is in the wf                
def QPController(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, com, ffwdOn, isBaseControlled,  normals, f_min, mu):
    util = Utils()
                
    # compute virtual impedances                
    Wffwd, Wfbk, Wg = computeVirtualImpedanceWrench(conf, act_pose, act_twist,  W_contacts,  des_pose, des_twist, des_acc, stance_legs, com, ffwdOn, isBaseControlled)                
    # Total Wrench
    TotWrench =    Wfbk+ Wg  +Wffwd   
          
    
    #(Ax-b)T*(Ax-b)
    # G = At*A
    # g = -At*b
    #0.5 xT*G*x + gT*x
    #s.t. Cx<=d
        
    # compute cost function
    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map["LF"]] * np.ones(3), stance_legs[util.leg_map["RF"]] * np.ones(3),
                              stance_legs[util.leg_map["LH"]] * np.ones(3), stance_legs[util.leg_map["RH"]] * np.ones(3)]))
    # This is a skew symmetric matrix for (xfi-xc)  corressponding  toe difference between the foothold locations
    # and COM trajectories)
    d1 = cross_mx(W_contacts[:,util.leg_map["LF"]] - util.linPart(act_pose))
    d2 = cross_mx(W_contacts[:,util.leg_map["RF"]] - util.linPart(act_pose))
    d3 = cross_mx(W_contacts[:,util.leg_map["LH"]] - util.linPart(act_pose))
    d4 = cross_mx(W_contacts[:,util.leg_map["RH"]] - util.linPart(act_pose))
    # Compute Jb^T
    JbT = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    #nullify columns relative to legs that are not in contact                                                                                                #
    JbT = JbT.dot(S_mat)
    
    W =  np.eye(12) * 1e-4           
                                                                
    G = JbT.T.dot(JbT) + np.eye(12) * 1e-4  #regularize and make it definite positive
    g = -JbT.T.dot(TotWrench)                 
    
    #compute inequalities - unilateral constraints Cx >= f_min => -Cx <= -f_min
#    C =  - block_diag( normals[util.leg_map["LF"]] , 
#                                         normals[util.leg_map["RF"]], 
#                                         normals[util.leg_map["LH"]], 
#                                         normals[util.leg_map["RH"]]    )                                                 
#    #not need to nullify columns relative to legs that are not in contact because the QP solver does not like 0 = 0 constraints                                                                                           #
#    d = -f_min.reshape((4,))
                
    # EXERCISE 11: compute friction cones inequalities A_f x <= 0
    C_leg = [None]*4            
    for leg in range(4):
        #compute tangential components
        ty = np.cross(normals[leg], np.array([1,0,0]))
        tx = np.cross(ty, normals[leg])
                                        
        C_leg[leg] = np.array([
            tx  - mu[leg]*normals[leg] ,
            -tx - mu[leg]*normals[leg] ,
            ty  - mu[leg]*normals[leg] ,
            -ty - mu[leg]*normals[leg] ])
        
                   
    C =   block_diag( C_leg[util.leg_map["LF"]] , 
                   C_leg[util.leg_map["RF"]], 
                       C_leg[util.leg_map["LH"]], 
                       C_leg[util.leg_map["RH"]]    )   
    d = np.zeros(C.shape[0]) 
                                             

                
                                                                                                                                                                                    
    #not need to nullify columns relative to legs that are not in contact bec                
    des_grf = quadprog_solve_qp(G, g, C, d, None , None)   
 
    #compute constraint violations (take smallest distance from constraint)
    constr_viol = np.zeros(4)
    for leg in range(4): 
        distance_to_violation = np.amin( - C[3*leg:3*leg+4,:].dot(des_grf))  #d should be zeros so does not affect
       							
        constr_viol[leg] = 1/(1.0 + distance_to_violation*distance_to_violation) #goes to 1 when constraints are violated                                
    
    return des_grf, Wffwd, Wfbk, Wg, constr_viol
