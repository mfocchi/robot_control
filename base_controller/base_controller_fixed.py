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

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
#from ros_impedance_controller.msg import BaseState # no longer used

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
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import GetPhysicsPropertiesRequest
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

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
import  params as conf
robotName = "ur5"

# controller manager management
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from std_msgs.msg import Float64MultiArray

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

class BaseController(threading.Thread):
    
    def __init__(self, robot_name="ur5",  launch_file=None):
        
        self.robot_name = robot_name
        #clean up previous process

        os.system("killall rosmaster rviz gzserver gzclient")      
        # needed to be able to load a custom world file
        os.system("export GAZEBO_MODEL_PATH="+rospkg.RosPack().get_path('ros_impedance_controller')+"/worlds/models/:"+os.environ['GAZEBO_MODEL_PATH'])  
                          
        if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
            print ('ROS MASTER is active')
            nodes = rosnode.get_node_names()
            if "/rviz" in nodes:
                 print("Rviz active")
                 rvizflag=" rviz:=false"
            else:                                                         
                 rvizflag=" rviz:=true" 
        #start ros impedance controller
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        if launch_file is None:
            launch_file = self.robot_name
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('ros_impedance_controller')+"/launch/ros_impedance_controller_"+launch_file+".launch"])
        self.launch.start() 
        ros.sleep(1.0)

        # Loading a robot model of robot (Pinocchio)
        xacro_path = rospkg.RosPack().get_path('ur_description')+'/urdf/ur5.xacro'
        self.robot = getRobotModel(self.robot_name, generate_urdf = True, xacro_path = xacro_path)
        threading.Thread.__init__(self)
                               
        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual = True)                    
        self.u = Utils()    
                                
        self.contact_flag = False       
        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)                                
        self.q_des =np.zeros(self.robot.na)
        # GOZERO Keep the fixed configuration for the joints at the start of simulation
        self.q_des[:self.robot.na] = conf.robot_params[self.robot_name]['q_0']   
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd =np.zeros(self.robot.na)                                         
        self.contactForceW = np.zeros(3)
      
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.sub_jstate = ros.Subscriber("/"+self.robot_name+"/joint_states", JointState, callback=self._receive_jstate, queue_size=1,  tcp_nodelay=True)                  
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)

        # freeze base  and pause simulation service 
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties) 
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)        
               
        self.broadcaster = tf.TransformBroadcaster()
        self.spawn_z = ros.get_param('/spawn_z')
        self.use_torque_control = ros.get_param('/use_torque_control')
        
        #send data to param server
        self.verbose = conf.verbose                                                                                           
        self.u.putIntoGlobalParamServer("verbose", self.verbose)   
        
        # controller manager management (only usable in non torque mode)
        self.switch_controller_srv = ros.ServiceProxy("/"+self.robot_name+"/controller_manager/switch_controller", SwitchController)
        self.load_controller_srv = ros.ServiceProxy("/"+self.robot_name+"/controller_manager/load_controller", LoadController)
        self.pub_reduced_des_jstate = ros.Publisher("/"+self.robot_name+"/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)
        self.available_controllers = ["joint_group_pos_controller", 
                                      "pos_joint_traj_controller"]        
        self.active_controller = self.available_controllers[0]
        


        print("Initialized fixed basecontroller---------------------------------------------------------------")
                             
    def _receive_jstate(self, msg):
         for msg_idx in range(len(msg.name)):          
             for joint_idx in range(len(self.joint_names)):
                 if self.joint_names[joint_idx] == msg.name[msg_idx]: 
                     self.q[joint_idx] = msg.position[msg_idx]
                     self.qd[joint_idx] = msg.velocity[msg_idx]
                     self.tau[joint_idx] = msg.effort[msg_idx]
                
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
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()
        os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")    
        os.system(" rosnode kill /gazebo")   
                                                                                                                                     
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
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        self.x_ee = np.array([0.0, 0.0, self.spawn_z]) + self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation 
        # compute jacobian of the end effector in the world frame  
        self.J6 = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
        # take first 3 rows of J6 cause we have a point contact            
        self.J = self.J6[:3,:] 
        #compute contact forces                        
        self.estimateContactForces() 
        # broadcast base world TF if they are different        
        self.broadcaster.sendTransform((0.0, 0.0, self.spawn_z), (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')  
        

    def estimateContactForces(self):  
        # estimate ground reaxtion forces from tau 
        self.contactForceW = np.linalg.inv(self.J6.T).dot(self.h-self.tau)[:3]                   
                                 
    def startupProcedure(self):
        ros.sleep(1.0)  # wait for callback to fill in jointmnames          
        self.pid = PidManager(self.joint_names) #I start after cause it needs joint names filled in by receive jstate callback
        if (self.use_torque_control):
            #set joint pdi gains     
            self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
            #only torque loop
            #self.pid.setPDs(0.0, 0.0, 0.0)          

    def initVars(self): 
        self.q_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'] ))*nan    
        self.q_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan   
        self.qd_des_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan    
        self.qd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan                                  
        self.tau_ffwd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan    
        self.tau_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan                                  
        self.contactForceW_log = np.empty((3,conf.robot_params[self.robot_name]['buffer_size'] ))  *nan 
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.time = 0.0
        self.log_counter = 0

    def logData(self):
        if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
            self.q_des_log[:, self.log_counter] =  self.q_des
            self.q_log[:,self.log_counter] =  self.q  
            self.qd_des_log[:,self.log_counter] =  self.qd_des
            self.qd_log[:,self.log_counter] = self.qd                       
            self.tau_ffwd_log[:,self.log_counter] = self.tau_ffwd                    
            self.tau_log[:,self.log_counter] = self.tau                     
            self.contactForceW_log[:,self.log_counter] =  self.contactForceW
            self.time_log[self.log_counter] = self.time
            self.log_counter+=1

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        print('Available controllers: ',self.available_controllers)
  
        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)
        print('Controller manager:Switching off  :  ',other_controllers)
        print('Controller manager: loading ',target_controller)
        
        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_controller_srv(srv)  
        
        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers 
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)           
          
       
    def send_reduced_des_jstate(self, q_des):     
        msg = Float64MultiArray()
        msg.data = q_des             
        self.pub_reduced_des_jstate.publish(msg) 

    def send_joint_trajectory(self):
        """Creates a trajectory and sends it using the selected action server"""
 
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format("/ur5/pos_joint_traj_controller"),
            FollowJointTrajectoryAction,
        )

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        # The following list are arbitrary positions
        # Change to your own needs if desired
        position_list = [[0, -1.57, -1.57, 0, 0, 0]]
        position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
        position_list.append([-0.5, 0.5, -1.2, 0, 0, 0])
        duration_list = [3.0, 7.0, 10.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = ros.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.ask_confirmation(position_list)     
        print("Executing trajectory using the {}".format("pos_joint_traj_controller"))
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        print("Trajectory execution finished in state {}".format(result.error_code))
        
    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        ros.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = raw_input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: "
            )
            valid = input_str in ["y", "n"]
            if not valid:
                ros.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                if (input_str == "y"):
                    confirmed = True
        if not confirmed:
            ros.loginfo("Exiting as requested by user.")
            sys.exit(0)

    
def talker(p):
            
    p.start()
    #p.register_node()
    p.initVars()     
    p.startupProcedure() 
         
    # to test the trajectory         
    #p.switch_controller("pos_joint_traj_controller")    
    #p.send_joint_trajectory()

    #loop frequency       
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt']) 
                           
    #control loop
    while True:  
        #update the kinematics
        p.updateKinematicsDynamics()    
    
        # set reference            
        p.q_des  = conf.robot_params[p.robot_name]['q_0']  + 0.1*np.sin(p.time)

        # controller 
        p.tau_ffwd = np.zeros(p.robot.na)       
        # only torque loop                            
        #p.tau_ffwd = conf.robot_params[p.robot_name]['kp']*(np.subtract(p.q_des,   p.q))  - conf.robot_params[p.robot_name]['kd']*p.qd     

        if (p.use_torque_control):
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            p.ros_pub.add_arrow(p.x_ee, p.contactForceW/(6*p.robot.robot_mass),"green") 
        else:   
            p.send_reduced_des_jstate(p.q_des) #no torque fb is present          

        # log variables
        p.logData()    
        p.ros_pub.add_marker(p.x_ee)
        p.ros_pub.publishVisual()                      
  
        #wait for synconization of the control loop
        rate.sleep()     
       
        p.time = p.time + conf.robot_params[p.robot_name]['dt']            
       # stops the while loop if  you prematurely hit CTRL+C                    
        if ros.is_shutdown():
            print ("Shutting Down")                    
            break;                                                
                             
    ros.sleep(1.0)                
    print ("Shutting Down")                 
    ros.signal_shutdown("killed")           
    p.deregister_node()     

    
if __name__ == '__main__':

    p = BaseController(robotName)
    try:
        talker(p)
    except ros.ROSInterruptException:
        from utils.common_functions import plotJoint      
        plotJoint('position',0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log, p.joint_names)
        plotJoint('torque',1, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log, p.joint_names)
      
    
        