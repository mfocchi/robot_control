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
import time as tm
import threading

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty
from termcolor import colored

from jet_leg.hyq_kinematics import HyQKinematics
from utils import Utils
import math
from jet_leg.math_tools import Math
from mathutils import *

from controlRoutines import wholeBodyController
from scipy.linalg import block_diag

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


from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

#important
np.set_printoptions(precision = 5, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)

class ControlThread(threading.Thread):
    def __init__(self):  
        
        threading.Thread.__init__(self)
        
        self.contactsW = np.zeros(12)
        self.basePoseW = np.zeros(6)   
        self.comPoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)

        
        self.stance_legs = np.array([True, True, True, True])
        
        self.q = np.zeros(12)
        self.qd = np.zeros(12)
        self.q_des =np.zeros(12)
        self.qd_des = np.zeros(12)
        self.tau_ffwd =np.zeros(12)
                
        self.sim_time  = 0.0
        self.numberOfReceivedMessages = 0
        self.numberOfPublishedMessages = 0
        self.joint_names = ""
        self.u = Utils()
     
        
    def run(self):
        
        self.robot_name = ros.get_param('/robot_name')
        self.sub_contact = ros.Subscriber("/"+self.robot_name+"/contacts_state", ContactsState, callback=self._receive_contact, queue_size=1)
        self.sub_pose = ros.Subscriber("/"+self.robot_name+"/ground_truth", Odometry, callback=self._receive_pose, queue_size=1)
        self.sub_pose = ros.Subscriber("/"+self.robot_name+"/joint_states", JointState, callback=self._receive_jstate, queue_size=1)                  
        self.pub_des_jstate = ros.Publisher("/"+self.robot_name+"/ros_impedance_controller/command", JointState, queue_size=1)
        self.set_pd_service = ros.ServiceProxy("/" + self.robot_name + "/ros_impedance_controller/set_pids", set_pids)

#        ros.wait_for_service("/"+self.robot_name+"/freeze_base")    
#        self.freeze_base = ros.ServiceProxy("/"+self.robot_name+"/freeze_base",Empty)
   

        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_gravity = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
#        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
#        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
        

        
        #missing pid callback
       
    def _receive_contact(self, msg):
        
        self.contactsW[0] = msg.states[0].total_wrench.force.x
        self.contactsW[1] =  msg.states[0].total_wrench.force.y
        self.contactsW[2] =  msg.states[0].total_wrench.force.z
        self.contactsW[3] = msg.states[1].total_wrench.force.x
        self.contactsW[4] =  msg.states[1].total_wrench.force.y
        self.contactsW[5] =  msg.states[1].total_wrench.force.z
        self.contactsW[6] = msg.states[2].total_wrench.force.x
        self.contactsW[7] =  msg.states[2].total_wrench.force.y
        self.contactsW[8] =  msg.states[2].total_wrench.force.z
        self.contactsW[9] = msg.states[3].total_wrench.force.x
        self.contactsW[10] =  msg.states[3].total_wrench.force.y
        self.contactsW[11] =  msg.states[3].total_wrench.force.z
        
    def _receive_pose(self, msg):
        
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.basePoseW[self.u.sp_crd["AX"]] = euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = euler[2]
        self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z
        
        self.baseTwistW[self.u.sp_crd["AX"]] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd["AY"]] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd["AZ"]] = msg.twist.twist.angular.z
        self.baseTwistW[self.u.sp_crd["LX"]] = msg.twist.twist.linear.x
        self.baseTwistW[self.u.sp_crd["LY"]] = msg.twist.twist.linear.y
        self.baseTwistW[self.u.sp_crd["LZ"]] = msg.twist.twist.linear.z

        
        mathJet = Math()

        self.b_R_w = mathJet.rpyToRot(euler[0], euler[1] , euler[2])
   
    def _receive_jstate(self, msg):
        #need to map to robcogen only the arrays coming from gazebo
         self.q[0] = msg.position[0]
         self.q[1] = msg.position[1]                
         self.q[2] = msg.position[2]
         self.q[6] = msg.position[3]
         self.q[7] = msg.position[4]
         self.q[8] = msg.position[5]
         self.q[3] = msg.position[6]
         self.q[4] = msg.position[7]
         self.q[5] = msg.position[8]
         self.q[9] = msg.position[9]
         self.q[10] = msg.position[10]         
         self.q[11] = msg.position[11]
         
         self.qd[0] = msg.velocity[0]
         self.qd[1] = msg.velocity[1]                
         self.qd[2] = msg.velocity[2]
         self.qd[6] = msg.velocity[3]
         self.qd[7] = msg.velocity[4]
         self.qd[8] = msg.velocity[5]
         self.qd[3] = msg.velocity[6]
         self.qd[4] = msg.velocity[7]
         self.qd[5] = msg.velocity[8]
         self.qd[9] = msg.velocity[9]
         self.qd[10] = msg.velocity[10]         
         self.qd[11] = msg.velocity[11]

         self.joint_names = msg.name

         self.numberOfReceivedMessages+=1
    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
         
         msg = JointState()
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd                
         self.pub_des_jstate.publish(msg)     
         self.numberOfPublishedMessages+=1        
        
        

    def register_node(self):
        ros.init_node('sub_pub_node_python', anonymous=False)

    def deregister_node(self):
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
        # create the message
        req_reset_world = SetModelStateRequest()
        #create model state
        model_state = ModelState()        
        model_state.model_name = "hyq"
        model_state.pose.position.x = 0.0
        model_state.pose.position.y = 0.0        
        model_state.pose.position.z = 1.0
        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0       
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0                
        req_reset_world.model_state = model_state
        # send request and get response (in this case none)
        self.reset_world(req_reset_world) 
       
        #toggle gravity
        req_reset_gravity = SetPhysicsPropertiesRequest()
        if (flag):
            req_reset_gravity.gravity.z = 0.0
        else:
            req_reset_gravity.gravity.z = 9.81
        self.reset_gravity(req_reset_gravity)

        
    def initKinematics(self,kin):
        kin.init_homogeneous()
        kin.init_jacobians()        
    def updateKinematics(self,kin):
        # q is continuously updated
        kin.update_homogeneous(self.q)
        kin.update_jacobians(self.q)
        self.actual_feetB = kin.forward_kin(self.q)

        
def talker(p):
    
    p.start()
    p.register_node()
    name = "Python Controller"
    kin = HyQKinematics()
    
    start_t = tm.time()
    p.setPDs(0.0, 0.0, 0.0)
    
    time = 0.0
    dt = 0.001
    
    exp_duration = 2.0
    num_samples = (int)(exp_duration/dt)

    
    # Parameters of Joint Reference Trajectories (X,Y, Z, Roll, Pitch, Yaw)
    amplitude = np.array([ 0.0, 0.0, 0.1, 0.0, 0.2, 0.0])
    frequencies = np.array([ 0.0, 0.0, 1.0, 0.0, 1.5, 0.0])
    w_rad=2*math.pi*frequencies   
    x0 = copy.deepcopy(p.basePoseW)
    
    
    p.tau_ffwd_log = np.zeros((num_samples, 12 ))
    p.basePoseW_log = np.zeros((num_samples, 6 ))
    p.baseTwistW_log = np.zeros((num_samples, 6 ))

    p.des_basePoseW_log = np.zeros((num_samples, 6 ))
    p.des_baseTwistW_log = np.zeros((num_samples, 6 ))    
    
    p.q_des_log = np.zeros((num_samples,6))
    p.q_log = np.zeros((num_samples, 6))
    p.grForcesW_log = np.zeros((num_samples, 12))

    #####starting procedure
    #set joint pdi gains
    p.setPDs(400.0, 26.0, 0.0)

    # GOZERO Keep the fixed configuration for the joints at the start of simulation
    p.q_des = np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4])
    p.qd_des = np.zeros(12)
    p.tau_ffwd = np.zeros(12)
    gravity_comp = np.array([24.2571, 1.92,50.5,    24.2, 1.92, 50.5739, 21.3801,-2.08377,-44.9598,  21.3858,-2.08365, -44.9615 ])

    resp = p.freezeBase(1)

    print("reset posture...")

    while tm.time()-start_t < 1.5:
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        tm.sleep(0.01)
#    print "q err prima freeze base", (p.q-p.q_des)
    resp = p.freezeBase(1)
    tm.sleep(2.0)

#    print "q err pre grav comp", (p.q - p.q_des)
    print("compensating gravity...")

    while tm.time() - start_t < 2.5:
        p.send_des_jstate(p.q_des, p.qd_des,  gravity_comp)
        tm.sleep(0.01)
#    print "q err post grav comp", (p.q - p.q_des)
    tm.sleep(2.0)

    #init the kinematics (homogeneous and jacobians for feet position I guess)
    p.initKinematics(kin)
    #update the kinematics
    p.updateKinematics(kin)
        

    print("starting com controller (no joint PD)...")
    p.setPDs(0.0, 0.0, 0.0)
    
    count = 0
    while count<num_samples:
 
        # p.get_contact()
        # p.get_pose()
        # p.get_jstate()
 
        #update the kinematics
        p.updateKinematics(kin)

#        to DEBUG implement a PD controller
        p.tau_ffwd = 300.0 * np.subtract(p.q_des,   p.q) - (10.0*p.qd);

        
        # CoM controlr
        des_com_pose  = x0 + np.array([ 0.0, 0.0, amplitude[2]*np.sin(w_rad[2]*time), 0.0, amplitude[4]*np.sin(w_rad[4]*time), 0.0]).T
        des_com_twist = np.array([ 0.0, 0.0,  amplitude[2]*w_rad[2]*np.cos(w_rad[1]*time), 0.0, amplitude[4]*w_rad[4]*np.cos(w_rad[4]*time), 0.0]).T
        des_com_acc = np.array([ 0.0, 0.0, -amplitude[2]*w_rad[2]*w_rad[2]*np.sin(w_rad[2]*time), 0.0, -amplitude[4]*w_rad[4]*w_rad[4]*np.sin(w_rad[4]*time), 0.0]).T  
 
         #use this for custom trajectory
#        des_com_acc = np.subtract(des_com_twist, p.des_com_twist_old)/p.Ts
#        p.des_com_twist_old = des_com_twist
        #comopute des_grf from whole-body controller
        B_contacts = kin.forward_kin(p.q) 

        #map contactct to wf
        w_R_b = p.b_R_w.transpose()
        W_contacts = np.zeros((3,4))
        W_contacts[:,p.u.leg_map["LF"]] = w_R_b.dot(B_contacts[p.u.leg_map["LF"],:].transpose() ) + p.u.linPart(des_com_pose).transpose()
        W_contacts[:,p.u.leg_map["RF"]] = w_R_b.dot(B_contacts[p.u.leg_map["RF"], :].transpose()) + p.u.linPart(des_com_pose).transpose()
        W_contacts[:,p.u.leg_map["LH"]] = w_R_b.dot(B_contacts[p.u.leg_map["LH"], :].transpose()) + p.u.linPart(des_com_pose).transpose()
        W_contacts[:,p.u.leg_map["RH"]] = w_R_b.dot(B_contacts[p.u.leg_map["RH"], :].transpose()) + p.u.linPart(des_com_pose).transpose()


        #TODO
        des_forces = wholeBodyController(p.basePoseW, p.baseTwistW, W_contacts,  des_com_pose, des_com_twist, des_com_acc, p.stance_legs)
        J_LF, J_RF, J_LH, J_RH, flag = kin.get_jacobians()
        p.jacsT = block_diag(np.transpose(w_R_b.dot(J_LF)), np.transpose(w_R_b.dot(J_RF)), np.transpose(w_R_b.dot(J_LH)), np.transpose(w_R_b.dot(J_RH)))
#        p.tau_ffwd =   -p.jacsT.dot(des_forces )
    
        #p.tau_ffwd  = np.zeros(12);       
        
        
        
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        
        
 

        #log 
        p.tau_ffwd_log[count ,:] = p.tau_ffwd
        p.basePoseW_log[count ,:] = p.basePoseW
        p.baseTwistW_log[count ,:] = p.baseTwistW
        p.des_basePoseW_log[count ,:] = des_com_pose
        p.des_baseTwistW_log[count ,:] = des_com_twist
    
        tm.sleep(dt)
        time = time + dt 
        count = count +1

        
    p.setPDs(400.0, 26.0, 0.0)
    
    #plot
    plt.subplot(3, 1, 1)
    plt.plot(p.des_basePoseW_log[:, p.u.sp_crd["LX"]], label="des", color="red")
    plt.plot(p.basePoseW_log[:, p.u.sp_crd["LX"]], label="act",    linestyle='--', color="blue")              
    plt.ylabel("$COM_x$", fontsize=10)
    
    plt.subplot(3, 1, 2)
    plt.plot(  p.des_basePoseW_log[:, p.u.sp_crd["LY"]], label="des", color="red")
    plt.plot( p.basePoseW_log[:, p.u.sp_crd["LY"]], label="act",    linestyle='--', color="blue")              
    plt.ylabel("$COM_y$", fontsize=10)
#    
    plt.subplot(3, 1, 3)
    plt.plot(  p.des_basePoseW_log[:, p.u.sp_crd["LZ"]], label="des", color="red")
    plt.plot( p.basePoseW_log[:, p.u.sp_crd["LZ"]], label="act",    linestyle='--', color="blue")              
    plt.ylabel("$COM_z$", fontsize=10)
    
    p.deregister_node()
    
    
if __name__ == '__main__':

    p = ControlThread()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        