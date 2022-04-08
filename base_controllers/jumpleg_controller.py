# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function

import copy
import os

import rospy as ros
import sys
import time
import threading

# messages for topic subscribers
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest
from termcolor import colored

#gazebo messages
from gazebo_msgs.srv import SetModelState
#gazebo services
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetPhysicsProperties

# ros utils
import roslaunch
import rosnode
import rosgraph
import rospkg
import tf
from rospy import Time

#other utils
from utils.ros_publish import RosPub
from utils.pidManager import PidManager
from utils.utils import Utils
from utils.math_tools import *
from numpy import nan
import pinocchio as pin
from utils.common_functions import getRobotModel
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from six.moves import input # solves compatibility issue bw pyuthon 2.x and 3 for raw input that does exists in python 3
from termcolor import colored
import matplotlib.pyplot as plt
from utils.common_functions import plotJoint, plotAdmittanceTracking, plotEndeff

import  params as conf
robotName = "jumpleg"

from base_controllers.inverse_kinematics.inv_kinematics_pinocchio import  robotKinematics
from base_controllers.utils.math_tools import Math

class JumpLegController(threading.Thread):
    
    def __init__(self, robot_name="jumpleg"):
        threading.Thread.__init__(self)
        self.robot_name = robot_name

        self.base_offset = np.array([ conf.robot_params[self.robot_name]['spawn_x'],
                                      conf.robot_params[self.robot_name]['spawn_y'],
                                     conf.robot_params[self.robot_name]['spawn_z']])

        # needed to be able to load a custom world file
        print(colored('Adding gazebo model path!', 'blue'))
        custom_models_path = rospkg.RosPack().get_path('ros_impedance_controller')+"/worlds/models/"
        if os.getenv("GAZEBO_MODEL_PATH") is not None:
            os.environ["GAZEBO_MODEL_PATH"] +=":"+custom_models_path
        else:
            os.environ["GAZEBO_MODEL_PATH"] = custom_models_path

        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")

        #start ros impedance controller
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [rospkg.RosPack().get_path('ros_impedance_controller')+'/launch/ros_impedance_controller_jumpleg.launch',
                    'spawn_x:='+str(conf.robot_params[self.robot_name]['spawn_x']),
                    'spawn_y:='+ str(conf.robot_params[self.robot_name]['spawn_y']),
                    'spawn_z:='+ str(conf.robot_params[self.robot_name]['spawn_z'])]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        ros.sleep(1.0)
        print(colored('-----------------------------------------------SIMULATION', 'blue'))

        # Loading a robot model of robot (Pinocchio)
        xacro_path = rospkg.RosPack().get_path('jumpleg_description')+'/robots/jumpleg.urdf.xacro'
        self.robot = getRobotModel(self.robot_name, generate_urdf = True, xacro_path = xacro_path)

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual = True)                    
        self.u = Utils()
        self.math_utils = Math()

      
        self.contact_flag = False       
        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)
        self.q_des =np.zeros(self.robot.na)
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd =np.zeros(self.robot.na)

        self.foot_pos = np.zeros(3)
        self.foot_pos_des = np.zeros(3)

        self.contactForceW = np.zeros(3)

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)

        # freeze base  and pause simulation service 
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties) 
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)        
               
        self.broadcaster = tf.TransformBroadcaster()

        #send data to param server
        self.verbose = conf.verbose                                                                                           
        self.u.putIntoGlobalParamServer("verbose", self.verbose)

        self.sub_jstate = ros.Subscriber("/" + self.robot_name + "/joint_states", JointState,
                                         callback=self._receive_jstate, queue_size=1, tcp_nodelay=True)

        self.ikin = robotKinematics(self.robot, conf.robot_params[self.robot_name]['foot_frame'])

        self.freezeBase = True
        from gazebo_msgs.srv import ApplyBodyWrench
        self.apply_body_wrench = ros.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.EXTERNAL_FORCE = False
        print("Initialized fixed basecontroller---------------------------------------------------------------")

    def applyForce(self):

        from geometry_msgs.msg import Wrench, Point
        wrench = Wrench()
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 30
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        reference_frame = "world" # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x = 0, y = 0, z = 0)
        try:
            self.apply_body_wrench(body_name=self.robotName+"::wrist_3_link", reference_frame=reference_frame, reference_point=reference_point , wrench=wrench, duration=ros.Duration(10))
        except:
            pass


    def _receive_jstate(self, msg):
         for msg_idx in range(len(msg.name)):          
             for joint_idx in range(len(self.joint_names)):
                 if self.joint_names[joint_idx] == msg.name[msg_idx]: 
                     self.q[joint_idx] = msg.position[msg_idx]
                     self.qd[joint_idx] = msg.velocity[msg_idx]
                     self.tau[joint_idx] = msg.effort[msg_idx]
         # broadcast base world TF if they are different (not needed because we have a world link)
         #self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0),   Time.now(), '/base_link', '/world')

    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd                
         self.pub_des_jstate.publish(msg)  
                
    def deregister_node(self):
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")
                                                                                                                                     
    def updateKinematicsDynamics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        self.robot.computeAllTerms(self.q, self.qd) 
        # joint space inertia matrix                
        self.M = self.robot.mass(self.q)
        # bias terms                
        self.h = self.robot.nle(self.q, self.qd)
        #gravity terms                
        self.g = self.robot.gravity(self.q)        
        #compute ee position  in the world frame  
        frame_name = conf.robot_params[self.robot_name]['foot_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.foot_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        # TODO compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]

        #compute contact forces                        
        self.estimateContactForces() 

    def estimateContactForces(self):
        pass
        #TODO
        #self.contactForceW = np.linalg.inv(self.J.T).dot(self.h-self.tau)[:3]
                                 
    def startupProcedure(self):
        ros.sleep(1.0)  # wait for callback to fill in jointmnames          
        self.pid = PidManager(self.joint_names) #I start after cause it needs joint names filled in by receive jstate callback
        self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
        print("Startup accomplished -----------------------")
    def initVars(self): 
        self.q_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.q_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan
        self.qd_des_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.qd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan
        self.tau_ffwd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.tau_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan

        self.foot_pos_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.foot_pos_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.contactForceW_log = np.empty((3,conf.robot_params[self.robot_name]['buffer_size'] ))  *nan
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.time = 0.0
        self.log_counter = 0


    def logData(self):
        if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
            self.q_des_log[:, self.log_counter] = self.q_des
            self.q_log[:,self.log_counter] =  self.q
            self.qd_des_log[:,self.log_counter] =  self.qd_des
            self.qd_log[:,self.log_counter] = self.qd
            self.tau_ffwd_log[:,self.log_counter] = self.tau_ffwd                    
            self.tau_log[:,self.log_counter] = self.tau
            self.foot_pos_log[:, self.log_counter] = self.foot_pos
            self.foot_pos_des_log[:, self.log_counter] = self.foot_pos_des
            self.contactForceW_log[:,self.log_counter] =  self.contactForceW
            self.time_log[self.log_counter] = self.time
            self.log_counter+=1

    
def talker(p):
            
    p.start()
    p.initVars()     
    p.startupProcedure()
    ros.sleep(1.0)
    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    #control loop
    while True:
        #update the kinematics
        p.updateKinematicsDynamics()
        p.q_des = np.copy(p.q_des_q0)
        if ( p.time > 2.0):
            p.tau_ffwd[3:] = -p.J.T.dot( p.g[:3])  # gravity compensation

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.ros_pub.add_arrow(p.foot_pos + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")
        # log variables
        #p.logData() TODO
        if (p.freezeBase and p.time > 2.0):
            print("releasing base")
            p.pid.setPDjoint(0, 0.0,0.0,0.0)
            p.pid.setPDjoint(1, 0.0,0.0,0.0)
            p.pid.setPDjoint(2, 0.0,0.0,0.0)
            p.freezeBase = False


        # disturbance force
        if (p.time > 3.0 and p.EXTERNAL_FORCE):
            p.applyForce()
            p.EXTERNAL_FORCE = False

        # plot end-effector
        p.ros_pub.add_marker(p.foot_pos + p.base_offset)
        p.ros_pub.publishVisual()

        #wait for synconization of the control loop
        rate.sleep()

        p.time = p.time + conf.robot_params[p.robot_name]['dt']
       # stops the while loop if  you prematurely hit CTRL+C
        if ros.is_shutdown():
            print ("Shutting Down")
            break

    print("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()

if __name__ == '__main__':
    p = JumpLegController(robotName)
    try:
        talker(p)
    except ros.ROSInterruptException:
        # these plots are for simulated robot TODO fix these
        #  plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
        #               p.tau_ffwd_log, p.joint_names)
        #  plotJoint('torque', 1, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
        #            p.tau_ffwd_log, p.joint_names)
         plt.show(block=True)
    
        
