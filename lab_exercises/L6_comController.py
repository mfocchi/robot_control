# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

#!/usr/bin/env python

#inherit from base controller
from base_controllers.base_controller import BaseController
import rospy as ros 
import numpy as np
from numpy import nan
import copy

# L5 Controller specific
from base_controllers.components.controlRoutines import projectionBasedController, QPController
from base_controllers.utils.common_functions import plotCoM, plotGRFs, plotConstraitViolation, plotJoint
from scipy.linalg import block_diag
from base_controllers.utils.math_tools import motionVectorTransform
from base_controllers.utils.common_functions import State
import matplotlib.pyplot as plt

import base_controllers.params as conf
import L6_conf as lab_conf
robotName = "hyq"

class Params:
    pass 
des_state = State(desired = True)
act_state = State() 

class AdvancedController(BaseController): 

    def __init__(self, robot_name="hyq"):
        super().__init__(robot_name=robot_name)

    def initVars(self):
        super().initVars()

        self.des_basePoseW_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.des_baseTwistW_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.des_baseAccW_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.des_forcesW_log = np.empty((12,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.Wffwd_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.Wfbk_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.Wg_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.constr_viol_log = np.empty((4,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.two_pi_f             = 2*np.pi*lab_conf.freq   # 2 PI * frequency
        self.two_pi_f_amp         = np.multiply(p.two_pi_f, lab_conf.amp) # A * 2 PI * frequency
        self.two_pi_f_squared_amp = np.multiply(p.two_pi_f, p.two_pi_f_amp)  # A * (2 PI * frequency)^2


    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.des_basePoseW_log[:, self.log_counter] = self.des_pose
            self.des_baseTwistW_log[:, self.log_counter] =  self.des_twist           
            self.des_baseAccW_log[:, self.log_counter] =  self.des_acc       
            self.des_forcesW_log[:, self.log_counter] =  self.des_forcesW            
            self.Wffwd_log[:, self.log_counter] =  self.Wffwd            
            self.Wfbk_log[:, self.log_counter] =  self.Wfbk            
            self.Wg_log[:, self.log_counter] =  self.Wg            
            self.constr_viol_log[:, self.log_counter] =  self.constr_viol
        super().logData()

def talker(p):
    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()          
    p.startupProcedure()

    rate = ros.Rate(1/lab_conf.dt) # 10hz
    
                                
    # Reset reference to actual value  
    p.x0 = copy.deepcopy(p.basePoseW)
    p.des_pose  = p.x0
    p.des_twist = np.zeros(6)
    p.des_acc = np.zeros(6)       

    # Control loop               
    while  (p.time  < lab_conf.exp_duration) or (lab_conf.CONTINUOUS and not ros.is_shutdown()):
        #update the kinematics
        p.updateKinematics()
                                
        # EXERCISE 1: Sinusoidal Reference Generation
        # Reference Generation
        p.des_pose  = p.x0 +  lab_conf.amp*np.sin(p.two_pi_f*p.time + lab_conf.phi)
        p.des_twist  = p.two_pi_f_amp * np.cos(p.two_pi_f*p.time + lab_conf.phi)
        p.des_acc = - p.two_pi_f_squared_amp * np.sin(p.two_pi_f*p.time + lab_conf.phi)
        #use this to compute acceleration for a custom trajectory
        #des_acc = np.subtract(des_twist, p.des_twist_old)/p.Ts
        #p.des__twist_old = des_base_twist
 
        # EXERCISE 6: Check static stability, move CoM out of the polygon   
        #p.des_pose[p.u.sp_crd["LY"]] +=0.0004
    
        # EXERCISE 8.a: Swift the Com on triangle of LF, RF, LH
#        p.des_pose[p.u.sp_crd["LX"]] = 0.1
#        p.des_pose[p.u.sp_crd["LY"]] = 0.1 
        # EXERCISE 8.b: Unload RH leg 
#        if p.time > 2.0:                            
#            p.stance_legs[p.u.leg_map["RH"]] = False       
   

        des_state.pose.set(p.des_pose)
        des_state.twist.set(p.des_twist)
        des_state.accel.set(p.des_acc)         
        act_state.pose.set(p.basePoseW)
        act_state.twist.set(p.baseTwistW)   

        # offset of the com wrt base origin in WF 
        params = Params()  
        params.gravityComp = False                                                  
        params.W_base_to_com = p.u.linPart(p.comPoseW)   -   p.u.linPart(p.basePoseW) 
        params.robot = p.robot
        params.robotInertiaB = p.compositeRobotInertiaB

        #################################################################
        # compute desired contact forces from the whole-body controller                      
        #################################################################
          

        # EXERCISE 2: Projection-based controller (base frame)
#        params.isCoMControlled = False 
#        params.gravityComp = False
#        params.ffwdOn = False        
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf, act_state, des_state, p.W_contacts, p.stance_legs, params)

        # EXERCISE 3: Add Gravity Compensation (base frame)        
#        params.isCoMControlled = False 
#        params.gravityComp = True
#        params.ffwdOn = False                                       
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf, act_state, des_state, p.W_contacts, p.stance_legs, params)
#     
        # EXERCISE 4: Add FFwd Term (base frame) 
#        params.isCoMControlled = False 
#        params.gravityComp = True
#        params.ffwdOn = True        
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf, act_state, des_state, p.W_contacts, p.stance_legs, params)
#                                
#        # EXERSISE 5: Projection-based controller (CoM)    
#        # map from base to com frame (they are aligned)
#        act_state.act_pose = p.comPoseW
#        act_state.act_twist = p.comTwistW 
#        params.isCoMControlled = True 
#        params.robotInertiaB = p.centroidalInertiaB
#        params.gravityComp = True
#        params.ffwdOn = True 
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf, act_state, des_state, p.W_contacts, p.stance_legs, params)
#        
        # EXERCISE 7: quasi-static QP controller (base frame) - unilateral constraints                
        params.normals = [None]*4                 
        params.normals[p.u.leg_map["LF"]] = np.array([0.0,0.0,1.0])
        params.normals[p.u.leg_map["RF"]] = np.array([0.0,0.0,1.0])
        params.normals[p.u.leg_map["LH"]] = np.array([0.0,0.0,1.0])
        params.normals[p.u.leg_map["RH"]] = np.array([0.0,0.0,1.0])  
        params.f_min = np.array([0.0,0.0,0.0, 0.0])    
        params.friction_coeff = np.array([0.6,0.6,0.6, 0.6])    
   
        params.isCoMControlled = False 
        params.gravityComp = True
        params.ffwdOn = True    
        params.frictionCones = False
        
        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(lab_conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, params)
        
        # EXERCISE 9: quasi-static QP controller (base frame) - friction cone constraints                                    
        #params.frictionCones = True       
        #p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(lab_conf, act_state, des_state, p.W_contacts, p.stance_legs, params)
                                
        #################################################################          
        # map desired contact forces into torques (missing gravity compensation)                      
        #################################################################                                       
        p.jacsT = block_diag(np.transpose(p.wJ[p.u.leg_map["LF"]]), 
                        np.transpose(p.wJ[p.u.leg_map["RF"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["LH"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["RH"]]  ))
        p.tau_ffwd =   p.u.mapFromRos(p.h_joints) - p.jacsT.dot(p.des_forcesW)         
 
    
    
       # send desired command to the ros controller     
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.logData()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),   3)  # to avoid issues of dt 0.0009999

        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW/(5*p.robot.robotMass)),"green")
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.des_forcesW/(5*p.robot.robotMass)),"blue")
        p.ros_pub.publishVisual()                        
                                
        #wait for synconization of the control loop
        rate.sleep()       


    # restore PD when finished
    p.pid.setPDs(400.0, 6.0, 0.0)
    ros.sleep(1.0)
    print ("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()

    # TODO fix the blocking console


if __name__ == '__main__':
    p = AdvancedController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
    finally:
        if conf.plotting:
            plotCoM('position', 0, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log,
                    p.baseTwistW_log, p.des_baseAccW_log, p.Wffwd_log + p.Wfbk_log + p.Wg_log)
            # plotCoM('wrench', 1, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, p.des_baseAccW_log, p.Wffwd_log  + p.Wfbk_log + p.Wg_log             )
            # plotGRFs(2, p.time_log, p.des_forcesW_log, p.grForcesW_log)
            # plotConstraitViolation(3,p.constr_viol_log)
            # plotJoint('torque',4, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)
