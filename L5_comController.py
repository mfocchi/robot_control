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

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
#import utils
sys.stderr = stderr
from ros_impedance_controller.srv import set_pids
from ros_impedance_controller.srv import set_pidsRequest
from ros_impedance_controller.msg import pid

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

import roslaunch
import rosnode
import rosgraph
#important
np.set_printoptions(precision = 5, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
sys.dont_write_bytecode = True

#controller specific
from gazebo_controller.hyq_kinematics.hyq_kinematics import HyQKinematics
from gazebo_controller.controlRoutines import quasiStaticController, QPController

from scipy.linalg import block_diag
from gazebo_controller.utils import Utils
from gazebo_controller.math_tools import *
import ex_5_conf as conf
from numpy import nan
from utils.common_functions import plotCoM, plotGRFs
import example_robot_data




from ros_publish import RosPub   
#instantiate graphic utils
ros_pub = RosPub(True)

class ControlThread(threading.Thread):
    def __init__(self):  
        #start ros impedance controller
        if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
            print 'ROS MASTER is Online'
        else:
            print 'ROS MASTER is Offline, starting ros impedance controller...'
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/ros_impedance_controller/launch/ros_impedance_controller.launch"])
            self.launch.start() 
            time.sleep(4.0)         
                                
        threading.Thread.__init__(self)
        
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
        self.verbose = conf.verbose                                 
        self.grForcesW = np.zeros(12)
        self.basePoseW = np.zeros(6) 
                                
        # Loading a robot model (Pinocchio)
        self.model = example_robot_data.loadHyQ().model
        self.data = self.model.createData()    
        self.J = [np.eye(3)]* 4   
                             
        #send data to param server
        data = {"verbose" : conf.verbose  }
        self.u.putIntoParamServer(data)	
								
    def run(self):
        
        self.robot_name = ros.get_param('/robot_name')
        self.sub_contact = ros.Subscriber("/"+self.robot_name+"/contacts_state", ContactsState, callback=self._receive_contact, queue_size=1)
        self.sub_pose = ros.Subscriber("/"+self.robot_name+"/ground_truth", Odometry, callback=self._receive_pose, queue_size=1)
        self.sub_jstate = ros.Subscriber("/"+self.robot_name+"/joint_states", JointState, callback=self._receive_jstate, queue_size=1)                  
        self.pub_des_jstate = ros.Publisher("/"+self.robot_name+"/command", JointState, queue_size=1)
        self.set_pd_service = ros.ServiceProxy("/" + self.robot_name + "/set_pids", set_pids)

        #new freeze base
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_gravity = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
        

    def _receive_contact(self, msg):
        #ground truth (only works with framwork, dls_hw_sim has already out convention)
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
       
        # estimate ground reaxtion forces from tau (TODO missing the model of the leg)
        for leg in range(4):
            grf = -np.linalg.inv(self.J[leg].T).dot(self.u.getLegJointState(leg, p.tau))                             
            self.u.setLegJointState(leg, grf, self.grForcesW)  
                                                
    def _receive_pose(self, msg):
        
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)

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
         #map to our convention
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
        ros.init_node('controller_python', anonymous=False)

    def deregister_node(self):
        print "deregistering nodes"					
        os.system("killall -9 rosmaster")    
        os.system("killall -9 gzserver")    
        os.system("killall -9 gzclient")    
        os.system("pkill rviz")
        os.system("pkill roslaunch")                                
        ros.signal_shutdown("manual kill")
 
    def get_contact(self):
        return self.contactsW
    def get_pose(self):
        return self.basePoseW
    def get_jstate(self):
        return self.q
    

    def setPDs(self, kp, kd, ki):
        # create the message
        req_msg = set_pidsRequest()
        req_msg.data = []

        # fill in the message with des values for kp kd
        for i in range(len(self.joint_names)):
            joint_pid = pid()
            joint_pid.joint_name = self.joint_names[i]
            joint_pid.p_value = kp
            joint_pid.d_value = kd
            joint_pid.i_value = ki
            req_msg.data += [joint_pid]

        # send request and get response (in this case none)
        self.set_pd_service(req_msg)
        
    def freezeBase(self, flag):

        #TODO make this code independent from framework because it countinuosly sets the gravity mode to false at the beginning till you call fb! so it will override this
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
        model_state.pose.position.z = 0.7
        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0       
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0                
        req_reset_world.model_state = model_state
        # send request and get response (in this case none)
        self.reset_world(req_reset_world) 
        
    def initKinematics(self,kin):
        kin.init_homogeneous()
        kin.init_jacobians()  
                                
    def updateKinematics(self,kin):
        # q is continuously updated
        kin.update_homogeneous(self.q)
        kin.update_jacobians(self.q)
        self.actual_feetB = kin.forward_kin(self.q)
       #update leg jacobians
        self.J[p.u.leg_map["LF"]], self.J[p.u.leg_map["RF"]], self.J[p.u.leg_map["LH"]], self.J[p.u.leg_map["RH"]], flag = kin.getLegJacobians()
                                    
                                         
    def startupProcedure(self):
        p.unpause_physics_client(EmptyRequest()) #pulls robot up
        time.sleep(0.2)  # wait for callback to fill in jointmnames

        # set joint pdi gains
        p.setPDs(400.0, 10.0, 0.0)
        # GOZERO Keep the fixed configuration for the joints at the start of simulation
        p.q_des = np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4])
        p.qd_des = np.zeros(12)
        p.tau_ffwd = np.zeros(12)
        p.gravity_comp = np.array(
            [24.2571, 1.92, 50.5, 24.2, 1.92, 50.5739, 21.3801, -2.08377, -44.9598, 21.3858, -2.08365, -44.9615])
                                                
        print("reset posture...")
        p.freezeBase(1)
        start_t = time.time()
        while time.time() - start_t < 1.0:
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            time.sleep(0.01)
        if p.verbose:
            print("q err prima freeze base", (p.q - p.q_des))
  
        print("put on ground and start compensating gravity...")
        p.freezeBase(0)                                                
        time.sleep(1.0)
        if p.verbose:
            print("q err pre grav comp", (p.q - p.q_des))
                                                
        start_t = time.time()
        while time.time() - start_t < 1.0:
            p.send_des_jstate(p.q_des, p.qd_des, p.gravity_comp)
            time.sleep(0.01)
        if p.verbose:
            print("q err post grav comp", (p.q - p.q_des))
                                                
        print("starting com controller (no joint PD)...")                
        p.setPDs(0.0, 0.0, 0.0)
                                
    def initVars(self):
 
        p.basePoseW_log = np.empty((6,0 ))*nan
        p.baseTwistW_log = np.empty((6,0 ))*nan
        p.des_basePoseW_log = np.empty((6,0 ))*nan
        p.des_baseTwistW_log = np.empty((6,0 ))*nan
        p.des_baseAccW_log = np.empty((6,0 )) *nan
        p.constr_viol = np.empty((4,0 )) *nan
        
        p.q_des_log = np.empty((12,0 ))*nan    
        p.q_log = np.empty((12,0 )) *nan   
        p.tau_ffwd_log = np.empty((12,0 ))*nan                                
        p.grForcesW_log = np.empty((12,0 ))  *nan 
        p.des_forcesW_log = np.empty((12,0 ))  *nan       
        p.Wffwd_log = np.empty((6,0 ))  *nan                    
        p.Wfbk_log = np.empty((6,0))  *nan  
        p.Wg_log = np.empty((6,0 ))  *nan                        
        p.time_log = np.array([])*nan
        p.constr_viol_log = np.empty((4,0 ))*nan
        
        p.time = 0.0
                                
        p.two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
        p.two_pi_f_amp         = np.multiply(p.two_pi_f, conf.amp)
        p.two_pi_f_squared_amp = np.multiply(p.two_pi_f, p.two_pi_f_amp)     


    def logData(self):
        
        p.basePoseW_log = np.hstack((p.basePoseW_log , p.basePoseW.reshape(6,-1)))
        p.baseTwistW_log = np.hstack((p.baseTwistW_log , p.baseTwistW.reshape(6,-1)))
        p.des_basePoseW_log = np.hstack((p.des_basePoseW_log , p.des_base_pose.reshape(6,-1)))
        p.des_baseTwistW_log = np.hstack((p.des_baseTwistW_log , p.des_base_twist.reshape(6,-1)))
        p.des_baseAccW_log = np.hstack((p.des_baseAccW_log , p.des_base_acc.reshape(6,-1)))                    
        p.q_des_log = np.hstack((p.q_des_log , p.q_des.reshape(12,-1)))   
        p.q_log = np.hstack((p.q_log , p.q.reshape(12,-1)))       
        p.tau_ffwd_log = np.hstack((p.tau_ffwd_log , p.tau_ffwd.reshape(12,-1)))                                
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
    name = "Python Controller"
    kin = HyQKinematics()
    p.initKinematics(kin)  
    p.initVars()          
    p.startupProcedure() 
   
                                
    #reset reference to actual value  
    p.x0 = copy.deepcopy(p.basePoseW)
    p.des_base_pose  = p.x0
    p.des_base_twist = np.zeros(6)
    p.des_base_acc = np.zeros(6)       

           
    while (p.time  < conf.exp_duration) or conf.CONTINUOUS:
        start_loop = time.time()
        #update the kinematics
        p.updateKinematics(kin)
                                
        # EXERCISE 1: Sinusoidal Reference Generation
        # Reference Generation
        p.des_base_pose  = p.x0 +  conf.amp*np.sin(p.two_pi_f*p.time + conf.phi)
        p.des_base_twist  = p.two_pi_f_amp * np.cos(p.two_pi_f*p.time + conf.phi)
        p.des_base_acc = - p.two_pi_f_squared_amp * np.sin(p.two_pi_f*p.time + conf.phi)
        #use this for custom trajectory
        #des_base_acc = np.subtract(des_base_twist, p.des_base_twist_old)/p.Ts
        #p.des_base_twist_old = des_base_twist
 
        # EXERCISE 6: Check static stability, move CoM out of the polygon   
        #p.des_base_pose[p.u.sp_crd["LY"]] +=0.0004
	
        # EXERCISE 9.a: Swift the Com on triangle of LF, RF, LH
#        p.des_base_pose[p.u.sp_crd["LX"]] = 0.1
#        p.des_base_pose[p.u.sp_crd["LY"]] = 0.1 
        # EXERCISE 9.b: Unload RH leg 
#        if p.time > 2.0:							
#	        p.stance_legs[p.u.leg_map["RH"]] = False       
								
        #comopute des_grf from whole-body controller
        B_contacts = kin.forward_kin(p.q) 
        #map contactct to wf
        w_R_b = p.b_R_w.transpose()
        W_contacts = np.zeros((3,4))
        W_contacts[:,p.u.leg_map["LF"]] = w_R_b.dot(B_contacts[p.u.leg_map["LF"],:].transpose() ) + p.u.linPart(p.basePoseW).transpose()
        W_contacts[:,p.u.leg_map["RF"]] = w_R_b.dot(B_contacts[p.u.leg_map["RF"], :].transpose()) + p.u.linPart(p.basePoseW).transpose()
        W_contacts[:,p.u.leg_map["LH"]] = w_R_b.dot(B_contacts[p.u.leg_map["LH"], :].transpose()) + p.u.linPart(p.basePoseW).transpose()
        W_contacts[:,p.u.leg_map["RH"]] = w_R_b.dot(B_contacts[p.u.leg_map["RH"], :].transpose()) + p.u.linPart(p.basePoseW).transpose()
                                
        B_base_to_com = np.array([conf.Bcom_x, conf.Bcom_y, conf.Bcom_z])
                                
        # EXERCISE 2: Projection-based controller         
        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = quasiStaticController(conf, p.basePoseW, p.baseTwistW, W_contacts,  p.des_base_pose, p.des_base_twist, p.des_base_acc, p.stance_legs, B_base_to_com, True, True)
        
                                        
        # EXERSISE 3: TODO Projection-based controller (CoM)    
        #map from base to com    (TODO)                                                        
        # p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = quasiStaticController(conf, p.basePoseW, p.baseTwistW, W_contacts,  p.des_com_pose, p.des_com_twist, p.des_com_acc, p.stance_legs, com, True, False)
        
	   # EXERCISE 8: quasi-static QP controller						
        normals = [None]*4                 
        normals[p.u.leg_map["LF"]] = np.array([0.0,0.0,1.0])
        normals[p.u.leg_map["RF"]] = np.array([0.0,0.0,1.0])
        normals[p.u.leg_map["LH"]] = np.array([0.0,0.0,1.0])
        normals[p.u.leg_map["RH"]] = np.array([0.0,0.0,1.0])    
        f_min = np.array([10.0,10.0,10.0, 10.0])    
        friction_coeff = np.array([0.1,0.1,0.1, 0.1])    
        #p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(conf, p.basePoseW, p.baseTwistW, W_contacts,  p.des_base_pose, p.des_base_twist, p.des_base_acc, p.stance_legs, B_base_to_com, True, True, normals, f_min, friction_coeff)                                           
                                               
        p.jacsT = block_diag(np.transpose(w_R_b.dot( p.J[p.u.leg_map["LF"]] )), 
                        np.transpose(w_R_b.dot( p.J[p.u.leg_map["RF"]] )), 
                        np.transpose(w_R_b.dot( p.J[p.u.leg_map["LH"]] )), 
                        np.transpose(w_R_b.dot( p.J[p.u.leg_map["RH"]]  )))
        
        #necessary to have no offset on the Z direction 
        p.tau_ffwd =   -p.jacsT.dot(p.des_forcesW)         
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
                                
        p.logData()    
        p.time = p.time + conf.dt 

       # plot actual grfs 
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["LF"]], p.u.getLegJointState(p.u.leg_map["LF"], p.grForcesW/400),"green")        
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["RF"]], p.u.getLegJointState(p.u.leg_map["RF"], p.grForcesW/400),"green")    
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["LH"]], p.u.getLegJointState(p.u.leg_map["LH"], p.grForcesW/400),"green")    
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["RH"]], p.u.getLegJointState(p.u.leg_map["RH"], p.grForcesW/400),"green") 
       # plot desired grfs
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["LF"]], p.u.getLegJointState(p.u.leg_map["LF"], p.des_forcesW/400),"blue")        
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["RF"]], p.u.getLegJointState(p.u.leg_map["RF"], p.des_forcesW/400),"blue")    
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["LH"]], p.u.getLegJointState(p.u.leg_map["LH"], p.des_forcesW/400),"blue")    
        ros_pub.add_arrow(W_contacts[:,p.u.leg_map["RH"]], p.u.getLegJointState(p.u.leg_map["RH"], p.des_forcesW/400),"blue")                          
        ros_pub.publishVisual()                        
                                
        #wait for synconization
        elapsed_time = time.time() - start_loop
        if elapsed_time < conf.dt:
            time.sleep(conf.dt - elapsed_time)       
                                                
         # stops the while loop if  you prematurely hit CTRL+C                    
        if ros.is_shutdown():
            print ("Shutting Down") 
            p.deregister_node()                   
            break;                                                
                             

    # restore PD when finished        
    p.setPDs(400.0, 26.0, 0.0)
    p.join()            
    totWrenchW = p.Wffwd_log  + p.Wfbk_log + p.Wg_log                
    plotCoM('position', 0, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, p.des_baseAccW_log, totWrenchW)
    #plotCoM('wrench', 1, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, p.des_baseAccW_log, totWrenchW)
    plotGRFs(2, p.time_log, p.des_forcesW_log, p.grForcesW_log)
    
    plt.figure(3)				
    plt.plot(p.constr_viol_log[p.u.leg_map["LF"],:],label="LF")
    plt.plot(p.constr_viol_log[p.u.leg_map["RF"],:],label="RF")
    plt.plot(p.constr_viol_log[p.u.leg_map["LH"],:],label="LH")
    plt.plot(p.constr_viol_log[p.u.leg_map["RH"],:],label="RH")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("Constr violation", fontsize=10)
    plt.grid()				
                
if __name__ == '__main__':

    p = ControlThread()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        