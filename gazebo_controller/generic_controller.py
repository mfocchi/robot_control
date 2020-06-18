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

from hyq_kinematics.hyq_kinematics import HyQKinematics
from utils import Utils
import math
from math_tools import *


#important
np.set_printoptions(precision = 5, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
sys.dont_write_bytecode = True

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
        
        self.contactsW = np.zeros(12)
        self.basePoseW = np.zeros(6)   
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
                
        self.sim_time  = 0.0
        self.numberOfReceivedMessages = 0
        self.numberOfPublishedMessages = 0
        self.joint_names = ""
        self.u = Utils()
        self.verbose = False 
        
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
       
        # estimate ground reaxtion forces from tau
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

        self.b_R_w = mathJet.rpyToRot(euler[0], euler[1] , euler[2])
   
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
         self.numberOfPublishedMessages+=1        
        
        

    def register_node(self):
        ros.init_node('sub_pub_node_python', anonymous=False)

    def deregister_node(self):
        os.system("killall -9 rosmaster")    
        os.system("killall -9 gzserver")    
        os.system("killall -9 gzclient")    
        os.system("pkill rviz")
        os.system("pkill roslaunch")                                
        ros.signal_shutdown("manual kill")
                                
    def get_sim_time(self):
        return self.sim_time
        
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

        
def talker(p):
    

        
    p.start()
    p.register_node()
    name = "Python Controller"
    kin = HyQKinematics()
    p.initKinematics(kin)        
    p.startupProcedure() 

    #looop frequency
    dt = 0.001                
    #control loop
    while True:  
        start_loop = time.time()
       # controller                             
        p.tau_ffwd = 300.0 * np.subtract(p.q_des,   p.q)  - 10*p.qd + p.gravity_comp;
        #p.tau_ffwd  = np.zeros(12);       
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
                                
        #wait for synconization
        elapsed_time = time.time() - start_loop
        if elapsed_time < dt:
            time.sleep(dt-elapsed_time)        
                                                
        # stops the while loop if  you prematurely hit CTRL+C                    
        if ros.is_shutdown():
            print ("Shutting Down")                    
            break;                                                
                                                
    p.deregister_node()
    p.join()            
    
    
if __name__ == '__main__':

    p = ControlThread()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        