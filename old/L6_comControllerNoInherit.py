# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

#!/usr/bin/env python

import copy
import numpy as np
import os

import rospy as ros
import sys
import time
import threading

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty, EmptyRequest
from termcolor import colored

#gazebo messages
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
#gazebo services
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

# ros utils
import roslaunch
import rosnode
import rosgraph

#other utils
from base_controller.utils.ros_publish import RosPub
from base_controller.utils.pidManager import PidManager
from base_controller.utils.utils import Utils
from base_controller.utils.math_tools import *
from numpy import nan

#robot specific 
from base_controller.hyq_kinematics.hyq_kinematics import HyQKinematics

#dynamics
from base_controller.utils.custom_robot_wrapper import RobotWrapper


# L5 Controller specific
from base_controller.utils.common_functions import plotCoM, plotGRFs, plotConstraitViolation, plotJoint, getRobotModel
from base_controller.utils.controlRoutines import projectionBasedController, QPController
from scipy.linalg import block_diag

# config file
import ex_5_conf as conf
 


class ControlThread(threading.Thread):
    def __init__(self):  
        
       #clean up previous process                

        os.system("killall gzserver gzclient")                                
        if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
            print 'ROS MASTER is active'
            nodes = rosnode.get_node_names()
            if "/rviz" in nodes:
                 print("Rviz active")
                 rvizflag=" rviz:=false"
            else:                                                         
                 rvizflag=" rviz:=true" 
        #start ros impedance controller
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)   
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/ros_impedance_controller/launch/ros_impedance_controller.launch"])
        #only available in ros lunar
#        roslaunch_args=rvizflag                             
#        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/ros_impedance_controller/launch/ros_impedance_controller_stdalone.launch"],roslaunch_args=[roslaunch_args])
        self.launch.start() 
        ros.sleep(4.0)        


        threading.Thread.__init__(self)
        # instantiate graphic utils
        self.ros_pub = RosPub(True)                    
        
        self.joint_names = ""
        self.u = Utils()
                                
        self.comPoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.stance_legs = np.array([True, True, True, True])
        
        self.q = np.zeros(12)
        self.qd = np.zeros(12)
        self.tau = np.zeros(12)                                
        self.q_des =np.zeros(12)
        self.qd_des = np.zeros(12)
        self.tau_ffwd =np.zeros(12)
        
        self.b_R_w = np.eye(3)       
                                  
        self.grForcesW = np.zeros(12)
        self.basePoseW = np.zeros(6) 
        self.J = [np.eye(3)]* 4                                   
        self.wJ = [np.eye(3)]* 4                       
                                
        self.robot_name = ros.get_param('/robot_name')
        self.sub_contact = ros.Subscriber("/"+self.robot_name+"/contacts_state", ContactsState, callback=self._receive_contact, queue_size=100)
        self.sub_pose = ros.Subscriber("/"+self.robot_name+"/ground_truth", Odometry, callback=self._receive_pose, queue_size=1)
        self.sub_jstate = ros.Subscriber("/"+self.robot_name+"/joint_states", JointState, callback=self._receive_jstate, queue_size=1)                  
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1)

        # freeze base  and pause simulation service 
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_gravity = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
                                
                                
        # Loading a robot model of HyQ (Pinocchio)
        self.robot = getRobotModel("hyq")
        
        #send data to param server
        self.verbose = conf.verbose                                                                                                          
        self.u.putIntoGlobalParamServer("verbose", self.verbose)                               

    def _receive_contact(self, msg):
        # get the ground truth from gazebo (only works with framwork, dls_hw_sim has already LF RF LH RH convention)
#        self.grForcesW[0] = msg.states[0].wrenches[0].force.x
#        self.grForcesW[1] =  msg.states[0].wrenches[0].force.y
#        self.grForcesW[2] =  msg.states[0].wrenches[0].force.z
#        self.grForcesW[3] = msg.states[1].wrenches[0].force.x
#        self.grForcesW[4] =  msg.states[1].wrenches[0].force.y
#        self.grForcesW[5] =  msg.states[1].wrenches[0].force.z
#        self.grForcesW[6] = msg.states[2].wrenches[0].force.x
#        self.grForcesW[7] =  msg.states[2].wrenches[0].force.y
#        self.grForcesW[8] =  msg.states[2].wrenches[0].force.z
#        self.grForcesW[9] = msg.states[3].wrenches[0].force.x
#        self.grForcesW[10] =  msg.states[3].wrenches[0].force.y
#        self.grForcesW[11] =  msg.states[3].wrenches[0].force.z
        pass
                                                
    def _receive_pose(self, msg):
        
        self.quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(self.quaternion)

        self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z
        self.basePoseW[self.u.sp_crd["AX"]] = euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = euler[2]

        self.baseTwistW[self.u.sp_crd["LX"]] = msg.twist.twist.linear.x
        self.baseTwistW[self.u.sp_crd["LY"]] = msg.twist.twist.linear.y
        self.baseTwistW[self.u.sp_crd["LZ"]] = msg.twist.twist.linear.z
        self.baseTwistW[self.u.sp_crd["AX"]] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd["AY"]] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd["AZ"]] = msg.twist.twist.angular.z
        
        mathJet = Math()
        # compute orientation matrix                                
        self.b_R_w = mathJet.rpyToRot(euler)
   
    def _receive_jstate(self, msg):
          #need to map to robcogen only the arrays coming from gazebo because of ROS convention is different
         self.joint_names = msg.name   
         q_ros = np.zeros(12)
         qd_ros = np.zeros(12)
         tau_ros = np.zeros(12)             
         for i in range(len(self.joint_names)):           
             q_ros[i] = msg.position[i]
             qd_ros[i] = msg.velocity[i]
             tau_ros[i] = msg.effort[i]
         #map from ROS (alphabetical) to our  LF RF LH RH convention
         self.q = self.u.mapFromRos(q_ros)
         self.qd = self.u.mapFromRos(qd_ros)                    
         self.tau = self.u.mapFromRos(tau_ros)  
         
    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd                
         self.pub_des_jstate.publish(msg)     

    def register_node(self):
        ros.init_node('controller_python', disable_signals=False, anonymous=False)

    def deregister_node(self):
        print "deregistering nodes"     
        os.system(" rosnode kill /hyq/ros_impedance_controller")    
        os.system(" rosnode kill /gazebo")    
 
    def get_contact(self):
        return self.contactsW
    def get_pose(self):
        return self.basePoseW
    def get_jstate(self):
        return self.q

        
    def freezeBase(self, flag):
        #toggle gravity
        req_reset_gravity = SetPhysicsPropertiesRequest()
        #ode config
        req_reset_gravity.time_step = 0.001
        req_reset_gravity.max_update_rate = 1000                
        req_reset_gravity.ode_config.sor_pgs_iters = 50
        req_reset_gravity.ode_config.sor_pgs_w = 1.3        
        req_reset_gravity.ode_config.contact_surface_layer = 0.001
        req_reset_gravity.ode_config.contact_max_correcting_vel = 100
        req_reset_gravity.ode_config.erp = 0.2
        req_reset_gravity.ode_config.max_contacts = 20        
        
        if (flag):
            req_reset_gravity.gravity.z =  0.0
        else:
            req_reset_gravity.gravity.z = -9.81                
        self.reset_gravity(req_reset_gravity)

        # create the message
        req_reset_world = SetModelStateRequest()
        #create model state
        model_state = ModelState()        
        model_state.model_name = "hyq"
        model_state.pose.position.x = 0.0
        model_state.pose.position.y = 0.0        
        model_state.pose.position.z = 0.8

        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0       
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0                
                                
        model_state.twist.linear.x = 0.0
        model_state.twist.linear.y = 0.0        
        model_state.twist.linear.z = 0.0
             
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0        
        model_state.twist.angular.z = 0.0
             
        req_reset_world.model_state = model_state
        # send request and get response (in this case none)
        self.reset_world(req_reset_world) 
        
    def initKinematics(self,kin):
        kin.init_homogeneous()
        kin.init_jacobians()  
                                
    def mapBaseToWorld(self, B_var):
        W_var = p.b_R_w.transpose().dot(B_var) + p.u.linPart(self.basePoseW)                            
        return W_var
                                                                                                                                
    def updateKinematics(self,kin):
        # q is continuously updated
        kin.update_homogeneous(self.q)
        kin.update_jacobians(self.q)
        self.B_contacts = kin.forward_kin(p.q) 
        # map feet contacts to wf
        self.W_contacts = np.zeros((3,4))
        for leg in range(4):
             self.W_contacts[:,leg] = self.mapBaseToWorld(self.B_contacts[leg, :].transpose())
        # update the feet jacobians
        self.J[p.u.leg_map["LF"]], self.J[p.u.leg_map["RF"]], self.J[p.u.leg_map["LH"]], self.J[p.u.leg_map["RH"]], flag = kin.getLegJacobians()
        #map jacobians to WF
        for leg in range(4):
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])
             
        # Pinocchio Update the joint and frame placements
        gen_velocities  = np.hstack((self.baseTwistW,self.qd))
        configuration = np.hstack(( self.u.linPart(self.basePoseW), self.quaternion, self.q))
        self.robot.computeAllTerms(configuration, gen_velocities)    
        self.M = self.robot.mass(self.q, False)    
        self.h = self.robot.nle(configuration, gen_velocities, False)
        self.h_joints = self.h[6:]  
        #compute contact forces                        
        self.estimateContactForces()            

    def estimateContactForces(self):           
        # estimate ground reaxtion forces from tau 
        for leg in range(4):
            grf = np.linalg.inv(self.wJ[leg].T).dot(self.u.getLegJointState(leg, self.h_joints - self.tau ))                             
            self.u.setLegJointState(leg, grf, self.grForcesW)                                  
                                  
    def startupProcedure(self):
        p.unpause_physics_client(EmptyRequest()) #pulls robot up
        ros.sleep(0.2)  # wait for callback to fill in jointmnames
                                
        p.pid = PidManager(self.joint_names) #I start after cause it needs joint names filled in by receive jstate callback
        # set joint pdi gains
        p.pid.setPDs(400.0, 6.0, 0.0)
        # GOZERO Keep the fixed configuration for the joints at the start of simulation
        p.q_des = np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4])
        p.qd_des = np.zeros(12)
        p.tau_ffwd = np.zeros(12)
                                
       # these torques are to compensate the leg gravity
        p.gravity_comp = np.array(
            [24.2571, 1.92, 50.5, 24.2, 1.92, 50.5739, 21.3801, -2.08377, -44.9598, 21.3858, -2.08365, -44.9615])
                                                
        print("reset posture...")
        p.freezeBase(1)
        start_t = ros.get_time()
        while ros.get_time() - start_t < 1.0:
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            ros.sleep(0.01)
        if p.verbose:
            print("q err prima freeze base", (p.q - p.q_des))
  
        print("put on ground and start compensating gravity...")
        p.freezeBase(0)                                                
        ros.sleep(1.0)
        if p.verbose:
            print("q err pre grav comp", (p.q - p.q_des))
                                                
        start_t = ros.get_time()
        while ros.get_time()- start_t < 1.0:
            p.send_des_jstate(p.q_des, p.qd_des, p.gravity_comp)
            ros.sleep(0.01)
        if p.verbose:
            print("q err post grav comp", (p.q - p.q_des))
                                                
        print("starting com controller (no joint PD)...")                
        p.pid.setPDs(0.0, 0.0, 0.0)
                                
    def initVars(self):
 
        p.basePoseW_log = np.empty((6,0 ))*nan
        p.baseTwistW_log = np.empty((6,0 ))*nan
        p.des_basePoseW_log = np.empty((6,0 ))*nan
        p.des_baseTwistW_log = np.empty((6,0 ))*nan
        p.des_baseAccW_log = np.empty((6,0 )) *nan
        p.constr_viol = np.empty((4,0 )) *nan
        
        p.q_des_log = np.empty((12,0 ))*nan    
        p.q_log = np.empty((12,0 )) *nan   
        p.qd_des_log = np.empty((12,0 ))*nan    
        p.qd_log = np.empty((12,0 )) *nan                                  
        p.tau_ffwd_log = np.empty((12,0 ))*nan    
        p.tau_log = np.empty((12,0 ))*nan                                  
        p.grForcesW_log = np.empty((12,0 ))  *nan 
        p.des_forcesW_log = np.empty((12,0 ))  *nan       
        p.Wffwd_log = np.empty((6,0 ))  *nan                    
        p.Wfbk_log = np.empty((6,0))  *nan  
        p.Wg_log = np.empty((6,0 ))  *nan                        
        p.time_log = np.array([])*nan
        p.constr_viol_log = np.empty((4,0 ))*nan
        
        p.time = 0.0
        p.two_pi_f             = 2*np.pi*conf.freq   # 2 PI * frequency  
        p.two_pi_f_amp         = np.multiply(p.two_pi_f, conf.amp) # A * 2 PI * frequency  
        p.two_pi_f_squared_amp = np.multiply(p.two_pi_f, p.two_pi_f_amp)  # A * (2 PI * frequency)^2

    def logData(self):
        
        p.basePoseW_log = np.hstack((p.basePoseW_log , p.basePoseW.reshape(6,-1)))
        p.baseTwistW_log = np.hstack((p.baseTwistW_log , p.baseTwistW.reshape(6,-1)))
        p.des_basePoseW_log = np.hstack((p.des_basePoseW_log , p.des_pose.reshape(6,-1)))
        p.des_baseTwistW_log = np.hstack((p.des_baseTwistW_log , p.des_twist.reshape(6,-1)))
        p.des_baseAccW_log = np.hstack((p.des_baseAccW_log , p.des_acc.reshape(6,-1)))                    
        p.q_des_log = np.hstack((p.q_des_log , p.q_des.reshape(12,-1)))   
        p.q_log = np.hstack((p.q_log , p.q.reshape(12,-1)))       
        p.qd_des_log = np.hstack((p.qd_des_log , p.qd_des.reshape(12,-1)))   
        p.qd_log = np.hstack((p.qd_log , p.qd.reshape(12,-1)))                                      
        p.tau_ffwd_log = np.hstack((p.tau_ffwd_log , p.tau_ffwd.reshape(12,-1)))                                
        p.tau_log = np.hstack((p.tau_log , p.tau.reshape(12,-1)))                                  
        p.grForcesW_log = np.hstack((p.grForcesW_log , p.grForcesW.reshape(12,-1)))    
        p.des_forcesW_log = np.hstack((p.des_forcesW_log , p.des_forcesW.reshape(12,-1)))
        p.Wffwd_log = np.hstack((p.Wffwd_log , p.Wffwd.reshape(6,-1)))               
        p.Wfbk_log =  np.hstack((p.Wfbk_log , p.Wfbk.reshape(6,-1)))          
        p.Wg_log =  np.hstack((p.Wg_log , p.Wg.reshape(6,-1)))          
        p.time_log = np.hstack((p.time_log, p.time))
        p.constr_viol_log = np.hstack((p.constr_viol_log, p.constr_viol.reshape(4,-1)))      
                  
def talker(p):
    
    p.start()
    p.register_node()
    kin = HyQKinematics()
    p.initKinematics(kin)  
    p.initVars()          
    p.startupProcedure() 
    rate = ros.Rate(1/conf.dt) # 10hz
                                
    # Reset reference to actual value  
    p.x0 = copy.deepcopy(p.basePoseW)
    p.des_pose  = p.x0
    p.des_twist = np.zeros(6)
    p.des_acc = np.zeros(6)       

    # Control loop               
    while (p.time  < conf.exp_duration) or conf.CONTINUOUS:
        #update the kinematics
        p.updateKinematics(kin)
                                
        # EXERCISE 1: Sinusoidal Reference Generation
        # Reference Generation
        p.des_pose  = p.x0 +  conf.amp*np.sin(p.two_pi_f*p.time + conf.phi)
        p.des_twist  = p.two_pi_f_amp * np.cos(p.two_pi_f*p.time + conf.phi)
        p.des_acc = - p.two_pi_f_squared_amp * np.sin(p.two_pi_f*p.time + conf.phi)
        #use this to compute acceleration for a custom trajectory
        #des_acc = np.subtract(des_twist, p.des_twist_old)/p.Ts
        #p.des__twist_old = des_base_twist
 
        # EXERCISE 6: Check static stability, move CoM out of the polygon   
        p.des_pose[p.u.sp_crd["LY"]] +=0.0004
    
        # EXERCISE 8.a: Swift the Com on triangle of LF, RF, LH
#        p.des_pose[p.u.sp_crd["LX"]] = 0.1
#        p.des_pose[p.u.sp_crd["LY"]] = 0.1 
        # EXERCISE 8.b: Unload RH leg 
#        if p.time > 2.0:                            
#            p.stance_legs[p.u.leg_map["RH"]] = False       
                                


        # offset of the com wrt base origin in WF                                                    
        W_base_to_com = p.b_R_w.dot( np.array([conf.Bcom_x, conf.Bcom_y, conf.Bcom_z]))
        #################################################################          
        # compute desired contact forces from the whole-body controller                      
        #################################################################
        
        # EXERCISE 2: Projection-based controller (base frame) 
        isCoMControlled = False 
        gravityComp = False
        ffwdOn = False        
        #p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(conf, p.basePoseW, p.baseTwistW, p.W_contacts,  p.des_pose, p.des_twist, p.des_acc, p.stance_legs, W_base_to_com, isCoMControlled, gravityComp, ffwdOn)

       # EXERCISE 3: Add Gravity Compensation (base frame) 
        isCoMControlled = False 
        gravityComp = True
        ffwdOn = False
        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(conf, p.basePoseW, p.baseTwistW, p.W_contacts,  p.des_pose, p.des_twist, p.des_acc, p.stance_legs, W_base_to_com, isCoMControlled, gravityComp, ffwdOn)
     
       # EXERCISE 4: Add FFwd Term (base frame) 
#       isCoMControlled = False 
#        gravityComp = True
#        ffwdOn = True                
        #p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(conf, p.basePoseW, p.baseTwistW, p.W_contacts,  p.des_pose, p.des_twist, p.des_acc, p.stance_legs, W_base_to_com, isCoMControlled, gravityComp, ffwdOn)
                                
        # EXERSISE 5: Projection-based controller (CoM)    
        # map from base to com frame (they are aligned)

#        p.comPoseW = copy.deepcopy(p.basePoseW)
#        p.comPoseW[p.u.sp_crd["LX"]:p.u.sp_crd["LX"]+3] += W_base_to_com # + np.array([0.05, 0.0,0.0])
#        p.comTwistW = np.dot( motionVectorTransform( W_base_to_com, np.eye(3)),p.baseTwistW)
#       isCoMControlled = True 
#        gravityComp = True
#        ffwdOn = True    
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(conf, p.comPoseW, p.comTwistW, p.W_contacts,  p.des_pose, p.des_twist, p.des_acc, p.stance_legs, W_base_to_com, isCoMControlled, gravityComp, ffwdOn)
#        
       # EXERCISE 7: quasi-static QP controller (base frame) - unilateral constraints                
        normals = [None]*4                 
        normals[p.u.leg_map["LF"]] = np.array([0.0,0.0,1.0])
        normals[p.u.leg_map["RF"]] = np.array([0.0,0.0,1.0])
        normals[p.u.leg_map["LH"]] = np.array([0.0,0.0,1.0])
        normals[p.u.leg_map["RH"]] = np.array([0.0,0.0,1.0])    
        f_min = np.array([0.0,0.0,0.0, 0.0])    
        friction_coeff = np.array([0.6,0.6,0.6, 0.6])    
#        isCoMControlled = False 
#        gravityComp = True
#        ffwdOn = True    
#        conesOn = false
        #p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(conf, p.basePoseW, p.baseTwistW, p.W_contacts,  p.des_pose, p.des_twist, p.des_acc, p.stance_legs, W_base_to_com, isCoMControlled, gravityComp, ffwdOn, conesOn, normals, f_min, friction_coeff)                                           
        
       # EXERCISE 9: quasi-static QP controller (base frame) - friction cone constraints                                    
        #conesOn = True       
        #p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(conf, p.basePoseW, p.baseTwistW, p.W_contacts,  p.des_pose, p.des_twist, p.des_acc, p.stance_legs, W_base_to_com, False, True, True, True, normals, f_min, friction_coeff)                                           
                                
        #################################################################          
        # map desired contact forces into torques (missing gravity compensation)                      
        #################################################################                                       
        p.jacsT = block_diag(np.transpose(p.wJ[p.u.leg_map["LF"]]), 
                        np.transpose(p.wJ[p.u.leg_map["RF"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["LH"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["RH"]]  ))
        p.tau_ffwd =   p.h_joints - p.jacsT.dot(p.des_forcesW)         
 
    
    
       # send desired command to the ros controller     
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.logData()    
        p.time = p.time + conf.dt 
        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[:,leg], p.u.getLegJointState(leg, p.grForcesW/400),"green")        
            p.ros_pub.add_arrow(p.W_contacts[:,leg], p.u.getLegJointState(leg, p.des_forcesW/400),"blue")        
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
    p = ControlThread()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        