# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

#!/usr/bin/env python

#inherit from base controller
from base_controller.base_controller import BaseController
import rospy as ros 
import numpy as np
from numpy import nan
import copy

# L5 Controller specific
from base_controller.utils.controlRoutines import projectionBasedController, QPController
from base_controller.utils.common_functions import plotCoM, plotGRFs, plotConstraitViolation, plotJoint
from scipy.linalg import block_diag
from base_controller.utils.math_tools import motionVectorTransform
from base_controller.utils.common_functions import State
from optimization.ref_generation import ReferenceGenerator

# config file
import OPT_L1_walking_conf as conf

robot_name = "hyq"

des_state = State(desired = True)
act_state = State() 
initial_state = State() 

class AdvancedController(BaseController): 

    def __init__(self):  
        BaseController.__init__(self)
        
        #send data to param server
        self.verbose = conf.verbose                                                                                                          
        self.u.putIntoGlobalParamServer("verbose", self.verbose)	
        
    def initVars(self):
        BaseController.initVars(self)
								
        p.des_basePoseW_log = np.empty((6,0 ))*nan
        p.des_baseTwistW_log = np.empty((6,0 ))*nan
        p.des_baseAccW_log = np.empty((6,0 )) *nan
        p.constr_viol = np.empty((4,0 )) *nan
        
        p.des_forcesW_log = np.empty((12,0 ))  *nan       
        p.Wffwd_log = np.empty((6,0 ))  *nan                    
        p.Wfbk_log = np.empty((6,0))  *nan  
        p.Wg_log = np.empty((6,0 ))  *nan                        
        p.constr_viol_log = np.empty((4,0 ))*nan


    def logData(self):
        BaseController.logData(self)
        p.des_basePoseW_log = np.hstack((p.des_basePoseW_log , p.des_pose.reshape(6,-1)))
        p.des_baseTwistW_log = np.hstack((p.des_baseTwistW_log , p.des_twist.reshape(6,-1)))
        p.des_baseAccW_log = np.hstack((p.des_baseAccW_log , p.des_acc.reshape(6,-1)))                    
        p.des_forcesW_log = np.hstack((p.des_forcesW_log , p.des_forcesW.reshape(12,-1)))
        p.Wffwd_log = np.hstack((p.Wffwd_log , p.Wffwd.reshape(6,-1)))               
        p.Wfbk_log =  np.hstack((p.Wfbk_log , p.Wfbk.reshape(6,-1)))          
        p.Wg_log =  np.hstack((p.Wg_log , p.Wg.reshape(6,-1)))          
        p.constr_viol_log = np.hstack((p.constr_viol_log, p.constr_viol.reshape(4,-1)))      

def talker(p):
    
    p.start()
    p.register_node()
    p.initVars()          
    p.startupProcedure(robot_name) 
    rate = ros.Rate(1/conf.dt) # 10hz
    
                                
    # Reset reference to actual value  
    p.x0 = copy.deepcopy(p.basePoseW)
    p.des_pose  = p.x0
    p.des_twist = np.zeros(6)
    p.des_acc = np.zeros(6)       
   
    #refclass = ReferenceGenerator(conf)
    initial_state.set(act_state)
#    
#    #desired velocity
#    class desired_velocity():
#        pass
#    desired_velocity.lin_x = 0.05
#    desired_velocity.lin_y = 0.0
#    desired_velocity.ang_z = 0.0
#    
#    refclass.getReferenceData(initial_state, desired_velocity,  p.W_contacts,  np.logical_not(p.stance_legs), conf.robotHeight)    
    

    # Control loop               
    while (p.time  < conf.exp_duration) or conf.CONTINUOUS:
        #update the kinematics
        p.updateKinematics()
     
        des_state.pose.set(p.des_pose)
        des_state.twist.set(p.des_twist)
        des_state.accel.set(p.des_acc)         
        act_state.pose.set(p.basePoseW)
        act_state.twist.set(p.baseTwistW)  
                                                       
                              
        # set robot specific params                             
        conf.params.robot = p.robot                      
        conf.params.W_base_to_com = p.u.linPart(p.comPoseW)   -   p.u.linPart(p.basePoseW)         
        conf.params.robotInertiaB = p.compositeRobotInertiaB        
     
        #QP controller
        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(conf, act_state, des_state, p.W_contacts, p.stance_legs, conf.params)

        # map desired contact forces into torques                                     
        p.jacsT = block_diag(np.transpose(p.wJ[p.u.leg_map["LF"]]), 
                        np.transpose(p.wJ[p.u.leg_map["RF"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["LH"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["RH"]]  ))
        p.tau_ffwd =   p.u.mapFromRos(p.h_joints) - p.jacsT.dot(p.des_forcesW)         
 
        # send desired command to the ros controller     
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.logData()    
        p.time = p.time + conf.dt 
        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW/400),"green")        
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.des_forcesW/400),"blue")        
        p.ros_pub.publishVisual()                        
                                
        #wait for synconization of the control loop
        rate.sleep()       
                # stops the while loop if  you prematurely hit CTRL+C                    
        if ros.is_shutdown():
            print ("Shutting Down")                    
            break;                                                
                             
    # restore PD when finished        
    p.pid.setPDs(400.0, 6.0, 0.0) 
    ros.sleep(1.0)                
    print ("Shutting Down")                 
    ros.signal_shutdown("killed")           
    p.deregister_node()        
    
    plotCoM('position', 0, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, p.des_baseAccW_log, p.Wffwd_log  + p.Wfbk_log + p.Wg_log             )
    #plotCoM('wrench', 1, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, p.des_baseAccW_log, p.Wffwd_log  + p.Wfbk_log + p.Wg_log             )
    #plotGRFs(2, p.time_log, p.des_forcesW_log, p.grForcesW_log)
    #plotConstraitViolation(3,p.constr_viol_log)            
    #plotJoint('torque',4, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)
         
if __name__ == '__main__':
    p = AdvancedController()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        