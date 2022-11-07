# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

from __future__ import print_function

import os
import rospy as ros
import sys
# messages for topic subscribers
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger, TriggerRequest

# ros utils
import roslaunch
import rosnode
import rosgraph
import rospkg

#other utils
from base_controllers.utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from termcolor import colored
from base_controllers.utils.common_functions import plotJoint, plotEndeff
import  params as conf
robotName = "ur5"

# controller manager management
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from std_msgs.msg import Float64MultiArray
from base_controllers.base_controller_fixed import BaseControllerFixed
import tf
from rospy import Time
import time
from base_controllers.components.gripper_manager import GripperManager

class Ur5Generic(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_flag = True
        if (conf.robot_params[self.robot_name]['control_type'] == "torque"):
            self.use_torque_control = 1
        else:
            self.use_torque_control = 0

        if self.use_torque_control and self.real_robot:
            print(colored(
                "ERRORS: unfortunately...you cannot use ur5 in torque control mode, talk with your course coordinator to buy a better robot...:))",
                'red'))
            sys.exit()

        if conf.robot_params[self.robot_name]['gripper_sim']:
            self.gripper = True
        else:
            self.gripper = False
        self.gm = GripperManager(self.real_robot, conf.robot_params[self.robot_name]['dt'])

        #self.world_name = None # only the workbench
        self.world_name = 'empty.world'
        #self.world_name = 'palopoli.world'

        print("Initialized ur5 generic  controller---------------------------------------------------------------")

    def startRealRobot(self):
        os.system("killall rosmaster rviz gzserver gzclient")
        print(colored('------------------------------------------------ROBOT IS REAL!', 'blue'))

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = rospkg.RosPack().get_path('ur_robot_driver') + '/launch/ur5e_bringup.launch'
        cli_args = [launch_file,
                    'headless_mode:=true',
                    'robot_ip:=192.168.0.100',
                    'kinematics_config:=/home/laboratorio/my_robot_calibration_1.yaml']

        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        if (not rosgraph.is_master_online()) or (
                "/" + self.robot_name + "/ur_hardware_interface" not in rosnode.get_node_names()):
            print(colored('Launching the ur driver!', 'blue'))
            parent.start()

        # run rviz
        package = 'rviz'
        executable = 'rviz'
        args = '-d ' + rospkg.RosPack().get_path('ros_impedance_controller') + '/config/operator.rviz'
        node = roslaunch.core.Node(package, executable, args=args)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)

    def loadModelAndPublishers(self, xacro_path):
        super().loadModelAndPublishers(xacro_path)

        self.sub_ftsensor = ros.Subscriber("/" + self.robot_name + "/wrench", WrenchStamped,
                                           callback=self._receive_ftsensor, queue_size=1, tcp_nodelay=True)
        self.switch_controller_srv = ros.ServiceProxy(
            "/" + self.robot_name + "/controller_manager/switch_controller", SwitchController)
        self.load_controller_srv = ros.ServiceProxy("/" + self.robot_name + "/controller_manager/load_controller",
                                                    LoadController)
        # specific publisher for joint_group_pos_controller that publishes only position
        self.pub_reduced_des_jstate = ros.Publisher("/" + self.robot_name + "/joint_group_pos_controller/command",
                                                    Float64MultiArray, queue_size=10)

        self.zero_sensor = ros.ServiceProxy("/" + self.robot_name + "/ur_hardware_interface/zero_ftsensor", Trigger)

        #  different controllers are available from the real robot and in simulation
        if self.real_robot:
            # specific publisher for joint_group_pos_controller that publishes only position
            self.pub_reduced_des_jstate = ros.Publisher("/" + self.robot_name + "/joint_group_pos_controller/command",
                                                        Float64MultiArray, queue_size=10)
            self.available_controllers = [
                "joint_group_pos_controller",
                "scaled_pos_joint_traj_controller" ]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller" ]
        self.active_controller = self.available_controllers[0]

        self.broadcaster = tf.TransformBroadcaster()

    def _receive_ftsensor(self, msg):
        contactForceTool0 = np.zeros(3)
        contactMomentTool0 = np.zeros(3)
        contactForceTool0[0] = msg.wrench.force.x
        contactForceTool0[1] = msg.wrench.force.y
        contactForceTool0[2] = msg.wrench.force.z
        contactMomentTool0[0] = msg.wrench.torque.x
        contactMomentTool0[1] = msg.wrench.torque.y
        contactMomentTool0[2] = msg.wrench.torque.z
        self.contactForceW = self.w_R_tool0.dot(contactForceTool0)
        self.contactMomentW = self.w_R_tool0.dot(contactMomentTool0)

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
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in the base frame
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation
        self.w_R_tool0 = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).rotation
        # compute jacobian of the end effector in the base or world frame (they are aligned so in terms of velocity they are the same)
        self.J6 = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
        # take first 3 rows of J6 cause we have a point contact            
        self.J = self.J6[:3,:] 
        # broadcast base world TF
        self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')


    def startupProcedure(self):
        if (self.use_torque_control):
            #set joint pdi gains
            self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
        if (self.real_robot):
            self.zero_sensor()
        self.u.putIntoGlobalParamServer("real_robot",  self.real_robot)
        print(colored("finished startup -- starting controller", "red"))

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        print('Available controllers: ',self.available_controllers)
        print('Controller manager: loading ', target_controller)

        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)
        print('Controller manager:Switching off  :  ',other_controllers)

        srv = LoadControllerRequest()
        srv.name = target_controller

        self.load_controller_srv(srv)  
        
        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers 
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)
        self.active_controller = target_controller

    def send_reduced_des_jstate(self, q_des):
        msg = Float64MultiArray()
        msg.data = q_des
        if self.gripper and not self.real_robot:
            msg.data = np.append(q_des, self.gm.getDesGripperJoints())
        else:
            msg.data = q_des
        self.pub_reduced_des_jstate.publish(msg)

    def deregister_node(self):
        super().deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")

    def plotStuff(self):
        plotJoint('position', 0, self.time_log, self.q_log)

    def homing_procedure(self, dt, v_des, q_home, rate):
        v_ref = 0.0
        print(colored("STARTING HOMING PROCEDURE", 'red'))
        self.q_des = np.copy(self.q)
        print("Initial joint error = ", np.linalg.norm(self.q_des - q_home))
        print("q = ", self.q.T)
        print("Homing v des", v_des)
        while True:
            e = q_home - self.q_des
            e_norm = np.linalg.norm(e)
            if (e_norm != 0.0):
                v_ref += 0.005 * (v_des - v_ref)
                self.q_des += dt * v_ref * e / e_norm
                self.send_reduced_des_jstate(self.q_des)
            rate.sleep()
            if (e_norm < 0.001):
                self.homing_flag = False
                print(colored("HOMING PROCEDURE ACCOMPLISHED", 'red'))
                if self.gripper:
                    p.gm.move_gripper(100)
                break

def talker(p):
    p.start()
    if p.real_robot:
        p.startRealRobot()
    else:
        additional_args = 'gripper:=' + str(p.gripper) # +'gui:=false'
        p.startSimulator(world_name=p.world_name, use_torque_control=p.use_torque_control, additional_args =additional_args)

    # specify xacro location
    xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro'
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    p.startupProcedure()
    # sleep to avoid that the robot crashes on the table
    time.sleep(3.)

    # loop frequency
    rate = ros.Rate(1 / conf.robot_params[p.robot_name]['dt'])

    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    p.q_des = np.copy(p.q_des_q0)

    # use the point to point position controller
    if not p.use_torque_control:
        p.switch_controller("joint_group_pos_controller")

    gripper_on = 0
    #control loop
    while not ros.is_shutdown():
        p.updateKinematicsDynamics()
        # homing procedure

        if p.homing_flag:
            p.homing_procedure(conf.robot_params[p.robot_name]['dt'], 0.6, conf.robot_params[p.robot_name]['q_0'], rate)

        ## set joints here
        #p.q_des = p.q_des_q0  + 0.1 * np.sin(2*np.pi*0.5*p.time)
        ##test gripper
        # in Simulation remember to set gripper_sim : True in params.yaml!
        # if p.time>5.0 and (gripper_on == 0):
        #     print("gripper 30")
        #     p.gm.move_gripper(30)
        #     gripper_on = 1
        # if (gripper_on == 1) and p.time>10.0:
        #     print("gripper 100")
        #     p.gm.move_gripper(100)
        #     gripper_on = 2
        # need to uncomment this to be able to send joints references (leave it commented if you have an external node setting them)
        #p.send_reduced_des_jstate(p.q_des)

        if p.real_robot:
            p.ros_pub.add_arrow(p.x_ee + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")

        # log variables
        if (p.time > 1.0):
            p.logData()
        # plot end-effector
        p.ros_pub.add_marker(p.x_ee + p.base_offset)
        p.ros_pub.publishVisual()

        #wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999

if __name__ == '__main__':

    p = Ur5Generic(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if   conf.plotting:
            p.plotStuff()

    
        
