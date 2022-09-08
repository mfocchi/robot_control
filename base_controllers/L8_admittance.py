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
from docutils.nodes import label
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench, Point
from std_srvs.srv import Trigger, TriggerRequest

# ros utils
import roslaunch
import rosnode
import rosgraph
import rospkg
from rospy import Time

#other utils
from utils.math_tools import *
from numpy import nan
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from six.moves import input # solves compatibility issue bw pyuthon 2.x and 3 for raw input that does exists in python 3
from termcolor import colored
import matplotlib.pyplot as plt
from utils.common_functions import plotJoint, plotAdmittanceTracking, plotEndeff

import  params as conf
import L8_conf as lab_conf
robotName = "ur5"

# controller manager management
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from std_msgs.msg import Float64MultiArray

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

from obstacle_avoidance.obstacle_avoidance import ObstacleAvoidance
from base_controllers.base_controller_fixed import BaseControllerFixed
from admittance_controller import AdmittanceControl
from base_controllers.utils.kin_dyn_utils import fifthOrderPolynomialTrajectory as coeffTraj

import tf
from rospy import Time

def resend_robot_program():
    ros.sleep(1.5)
    ros.wait_for_service("/ur5/ur_hardware_interface/resend_robot_program")
    sos_service = ros.ServiceProxy('/ur5/ur_hardware_interface/resend_robot_program', Trigger)
    sos = TriggerRequest()
    result = sos_service(sos)
    # print(result)
    ros.sleep(0.1)

def move_gripper(diameter):
    import os
    import sys
    import socket

    HOST = "192.168.0.100"  # The UR IP address
    PORT = 30002  # UR secondary client
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    sock.settimeout(0.5)
    try:
        sock.connect((HOST, PORT))
    except:
        raise Exception("Cannot connect to end-effector socket") from None
    sock.settimeout(None)
    scripts_path =  rospkg.RosPack().get_path(robotName+'_description') + '/grpper/scripts'

    onrobot_script = scripts_path + "/onrobot_superminimal.script";
    file = open(onrobot_script, "rb")  # Robotiq Gripper
    lines = file.readlines()
    file.close()

    tool_index = 0
    blocking = True
    cmd_string = f"tfg_release({diameter},  tool_index={tool_index}, blocking={blocking})"

    line_number_to_add = 446

    new_lines = lines[0:line_number_to_add]
    new_lines.insert(line_number_to_add + 1, str.encode(cmd_string))
    new_lines += lines[line_number_to_add::]

    offset = 0
    buffer = 2024
    file_to_send = b''.join(new_lines)

    if len(file_to_send) < buffer:
        buffer = len(file_to_send)
    data = file_to_send[0:buffer]
    while data:
        sock.send(data)
        offset += buffer
        if len(file_to_send) < offset + buffer:
            buffer = len(file_to_send) - offset
        data = file_to_send[offset:offset + buffer]
    sock.close()

    print("Gripper moved, now resend robot program")
    resend_robot_program()
    return


class LabAdmittanceController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_flag = self.real_robot
        if (conf.robot_params[self.robot_name]['control_type'] == "torque"):
            self.use_torque_control = 1
        else:
            self.use_torque_control = 0



        if (lab_conf.obstacle_avoidance):
            self.world_name = 'tavolo_obstacles.world'
            if (not self.use_torque_control):
                print(colored("ERRORS: you can use obstacle avoidance only on torque control mode", 'red'))
                sys.exit()
        else:
            self.world_name = None

        if lab_conf.admittance_control and ((not self.real_robot) and (not self.use_torque_control)):
            print(colored("ERRORS: you can use admittance control only on torque control mode or in real robot (need contact force estimation or measurement)", 'red'))
            sys.exit()

        if self.use_torque_control and self.real_robot:
            print(colored(
                "ERRORS: unfortunately...you cannot use ur5 in torque control mode, talk with your course coordinator to buy a better robot...:))",
                'red'))
            sys.exit()

        print("Initialized L8 admittance  controller---------------------------------------------------------------")

    def startRealRobot(self):
        os.system("killall rviz gzserver gzclient")
        print(colored('------------------------------------------------ROBOT IS REAL!', 'blue'))
        if (not rosgraph.is_master_online()) or (
                "/" + self.robot_name + "/ur_hardware_interface" not in rosnode.get_node_names()):
            print(colored('Error: you need to launch the ur driver!', 'red'))
            sys.exit()
        else:
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
            self.available_controllers = [
                "joint_group_pos_controller",
                "scaled_pos_joint_traj_controller" ]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller" ]
        self.active_controller = self.available_controllers[0]

        self.broadcaster = tf.TransformBroadcaster()

    def applyForce(self):
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
            self.apply_body_wrench(body_name="ur5::wrist_3_link", reference_frame=reference_frame, reference_point=reference_point , wrench=wrench, duration=ros.Duration(10))
        except:
            pass

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
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation
        self.w_R_tool0 = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).rotation
        # compute jacobian of the end effector in the world frame
        self.J6 = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
        # take first 3 rows of J6 cause we have a point contact            
        self.J = self.J6[:3,:] 
        #compute contact forces                        
        self.estimateContactForces()
        # broadcast base world TF
        self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')

    def estimateContactForces(self):  
        # estimate ground reaction forces from torques tau
        if self.use_torque_control:
            self.contactForceW = np.linalg.inv(self.J6.T).dot(self.h-self.tau)[:3]
                                 
    def startupProcedure(self):
        if (self.use_torque_control):
            #set joint pdi gains
            self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
            #only torque loop
            #self.pid.setPDs(0.0, 0.0, 0.0)
        if (self.real_robot):
            self.zero_sensor()
        print(colored("finished startup -- starting controller", "red"))
        
    def initVars(self):
        super().initVars()

        # log variables relative to admittance controller
        self.q_des_adm_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_adm_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.EXTERNAL_FORCE = False
        self.payload_weight_avg = 0.0
        self.polynomial_flag = False
        self.obs_avoidance = ObstacleAvoidance()
        # position of the center of the objects is in WF
        self.obs_avoidance.setCubeParameters(0.25, np.array([0.125, 0.75,0.975]))
        self.obs_avoidance.setCylinderParameters(0.125, 0.3, np.array([0.6, 0.25, 1.0]))
        self.admit = AdmittanceControl(self.ikin, lab_conf.Kx, lab_conf.Dx, conf.robot_params[self.robot_name])

        if lab_conf.USER_TRAJECTORY:
            data = np.load('ur5_q_ref' + '.npz')
            self.q_ref = data['q']
            self.traj_duration = self.q_ref.shape[0]


    def logData(self):
        if (conf.robot_params[self.robot_name]['control_type'] == "admittance"):
            self.q_des_adm_log[:, self.log_counter] = self.q_des_adm
            self.x_ee_des_adm_log[:, self.log_counter] = self.x_ee_des_adm
        # I neeed to do after because it updates log counter
        super().logData()

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
        self.pub_reduced_des_jstate.publish(msg) 

    def send_joint_trajectory(self):

        # Creates a trajectory and sends it using the selected action server
        trajectory_client = actionlib.SimpleActionClient("{}/follow_joint_trajectory".format("/" + self.robot_name + "/"+self.active_controller), FollowJointTrajectoryAction)
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        # The following list are arbitrary positions
        # Change to your own needs if desired q0 [ 0.5, -0.7, 1.0, -1.57, -1.57, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
        print(colored("JOINTS ARE: ", 'blue'), self.q.transpose())
        # position_list = [[0.5, -0.7, 1.0, -1.57, -1.57, 0.5]]  # limits([0,-pi], [-pi/2,pi/2],  [0, -pi])
        # position_list.append([0.5, -0.7 - 0.2, 1.0 - 0.1, -1.57, -1.57, 0.5])
        # position_list.append([0.5 + 0.5, -0.7 - 0.3, 1.0 - 0.1, -1.57, -1.57, 0.5])
        # position_list.append([0.5 + 0.5, -0.7 - 0.3, 1.0 , -1., -1.57, 0.5])

        self.q0 = conf.robot_params[p.robot_name]['q_0']
        dq1 = np.array([0.2, 0,0,0,0,0])
        dq2 = np.array([0.2, -0.2, 0, 0, 0, 0])
        dq3 = np.array([0.2, -0.2, 0.4, 0, 0, 0])
        position_list = [self.q0]  # limits([0,-pi], [-pi/2,pi/2],  [0, -pi])
        position_list.append(self.q0 + dq1)
        position_list.append(self.q0 + dq2)
        position_list.append(self.q0 + dq3)
        print(colored("List of targets for joints: ",'blue'))
        print(position_list[0])
        print(position_list[1])
        print(position_list[2])
        print(position_list[3])

        duration_list = [5.0, 10.0, 20.0, 30.0]
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
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: " )
            valid = input_str in ["y", "n"]
            if not valid:
                ros.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                if (input_str == "y"):
                    confirmed = True
        if not confirmed:
            ros.loginfo("Exiting as requested by user.")
            sys.exit(0)

    def deregister_node(self):
        super().deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")

    def plotStuff(self):
        if not (conf.robot_params[p.robot_name]['control_mode'] == "trajectory"):
            if (lab_conf.admittance_control):
                plotJoint('position', 0, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
                          self.tau_ffwd_log, self.joint_names, self.q_des_adm_log)
                plotAdmittanceTracking(3, self.time_log, self.x_ee_log, self.x_ee_des_log, self.x_ee_des_adm_log, self.contactForceW_log)
            else:
                plotJoint('position', 0, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
                          self.tau_ffwd_log, self.joint_names, self.q_des_log)
            plotJoint('torque', 2, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
                      self.tau_ffwd_log, self.joint_names)
            plotEndeff('force', 1, p.time_log, p.contactForceW_log)
            plt.show(block=True)

def talker(p):
    p.start()
    if p.real_robot:
        p.startRealRobot()
    else:
        p.startSimulator(p.world_name, p.use_torque_control)

    # specify xacro location
    xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.xacro'
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    p.startupProcedure()


    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    p.q_des = np.copy(p.q_des_q0)
    p.admit.setPosturalTask(np.copy(p.q_des_q0))


    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    if (conf.robot_params[p.robot_name]['control_mode'] == "trajectory"):
        # to test the trajectory
        if (p.real_robot):
            p.switch_controller("scaled_pos_joint_traj_controller")
        else:
            p.switch_controller("pos_joint_traj_controller")
        p.send_joint_trajectory()
    else:
        if not p.use_torque_control:            
            p.switch_controller("joint_group_pos_controller")
        # reset to actual
        p.updateKinematicsDynamics()
        p.time_poly = None

        ext_traj_counter =0

        #control loop
        while True:
            # homing procedure
            if p.homing_flag:
                print(colored("STARTING HOMING PROCEDURE",'red'))
                while True:
                    joint_error = np.linalg.norm(p.q - conf.robot_params[p.robot_name]['q_0'])
                    p.q_des = p.q*0.95 + 0.05*conf.robot_params[p.robot_name]['q_0']
                    p.send_reduced_des_jstate(p.q_des)
                    rate.sleep()
                    if (joint_error<=0.001):
                        p.homing_flag = False
                        print(colored("HOMING PROCEDURE ACCOMPLISHED", 'red'))
                        break

            #update the kinematics
            p.updateKinematicsDynamics()

            if lab_conf.USER_TRAJECTORY and (p.time >6.0) and (ext_traj_counter < p.traj_duration):
                p.q_des = p.q_ref[ext_traj_counter,:]
                ext_traj_counter += 1

            # EXE L8-1.1: set constant joint reference
            #p.q_des = np.copy(p.q_des_q0)



            # EXE L8-1.2: set sinusoidal joint reference
            p.q_des  = p.q_des_q0  + lab_conf.amplitude * np.sin(2*np.pi*lab_conf.frequency*p.time)


            # EXE L8-1.3: set constant ee reference
            # p.x_ee_des = lab_conf.ee_reference
            # p.ros_pub.add_marker(p.x_ee_des + p.base_offset, color='blue')
            # p.q_des, ok, out_ws = p.ikin.endeffectorInverseKinematicsLineSearch(p.x_ee_des,
            #                                                                     conf.robot_params[p.robot_name][
            #                                                                         'ee_frame'], p.q, False, False,
            #                                                                     postural_task=True, w_postural=0.00001,
            #                                                                     q_postural=p.q_des_q0)

            # EXE L8-1.5:  set constant ee reference and create polynomial trajectory to reach it
            # p.x_ee_des = lab_conf.ee_reference
            # p.ros_pub.add_marker(p.x_ee_des + p.base_offset, color='blue')
            # if not p.polynomial_flag and p.time > 3.0:
            #     print(colored("STARTING POLYNOMIAL",'red'))
            #     p.time_poly = p.time
            #     p.x_intermediate = np.zeros(3)
            #     a = np.empty((3, 6))
            #     for i in range(3):
            #         a[i, :] = coeffTraj( lab_conf.poly_duration, p.x_ee[i], p.x_ee_des[i])
            #     p.polynomial_flag = True
            # # Polynomial trajectory for x,y,z coordinates
            # if  p.polynomial_flag and (p.time - p.time_poly) <  lab_conf.poly_duration:
            #     t = p.time - p.time_poly
            #     for i in range(3):
            #         p.x_intermediate[i] = a[i, 0] + a[i,1] * t + a[i,2] * pow(t, 2) + a[i,3] * pow(t, 3) + a[i,4]*pow(t, 4) + a[i,5]*pow(t, 5)
            # if p.polynomial_flag:
            #     p.q_des, ok, out_ws = p.ikin.endeffectorInverseKinematicsLineSearch(p.x_intermediate,  conf.robot_params[p.robot_name]['ee_frame'], p.q, False, False,
            #                                                                     postural_task=True, w_postural=0.00001,
            #                                                                     q_postural=p.q_des_q0)

            # EXE L8-1.4:  set constant ee reference and desired orientation
            # rpy_des = lab_conf.des_orient
            # w_R_e_des = p.math_utils.eul2Rot(rpy_des) # compute rotation matrix representing the desired orientation from Euler Angles
            # p.q_des, ok, out_ws = p.ikin.endeffectorFrameInverseKinematicsLineSearch(p.x_ee_des, w_R_e_des, conf.robot_params[p.robot_name]['ee_frame'], p.q)
            # TODO create polynomial for orientation!

            # EXE L8-2 - admittance control
            if (lab_conf.admittance_control):
                p.EXTERNAL_FORCE = True
                ## TODO to implement at the velocity level you need to load the VelocityInterface and use the joint_group_vel_controller
                p.x_ee_des = p.robot.framePlacement(p.q_des,p.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).translation
                p.q_des_adm, p.x_ee_des_adm = p.admit.computeAdmittanceReference(p.contactForceW, p.x_ee_des, p.q)

            # EXE L8-2.5 - load estimation
            # if (p.time > 10.0):
            #     p.payload_weight_avg = 0.99 * p.payload_weight_avg + 0.01 * (-p.contactForceW[2] / 9.81)
            #     print("estimated load: ", p.payload_weight_avg)

            # controller with gravity coriolis comp
            p.tau_ffwd = p.h + np.zeros(p.robot.na)

            # only torque loop (not used)
            #p.tau_ffwd = conf.robot_params[p.robot_name]['kp']*(np.subtract(p.q_des,   p.q))  - conf.robot_params[p.robot_name]['kd']*p.qd

            # override the position command if you use admittance controller
            if (p.time > 1.5) and (lab_conf.admittance_control):  # activate admittance control only after a few seconds to allow initialization
                q_to_send = p.q_des_adm
            else:
                q_to_send = p.q_des

            # send commands to gazebo
            if (p.use_torque_control):
                if (lab_conf.obstacle_avoidance):
                    p.tau_ffwd = p.obs_avoidance.computeTorques(p,  lab_conf.des_ee_goal)
                p.send_des_jstate(q_to_send, p.qd_des, p.tau_ffwd)
            else:
                p.send_reduced_des_jstate(q_to_send)

            if(not lab_conf.obstacle_avoidance):
                p.ros_pub.add_arrow(p.x_ee + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")

            # log variables
            if (p.time > 1.0):
                p.logData()

            # disturbance force
            if (p.time > 3.0 and p.EXTERNAL_FORCE):
                p.applyForce()
                p.EXTERNAL_FORCE = False

            # plot end-effector
            p.ros_pub.add_marker(p.x_ee + p.base_offset)
            p.ros_pub.publishVisual()

            #wait for synconization of the control loop
            rate.sleep()

            p.time = p.time + conf.robot_params[p.robot_name]['dt']
           # stops the while loop if  you prematurely hit CTRL+C
            if ros.is_shutdown():
                p.plotStuff()
                print ("Shutting Down")
                break

    print("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()



if __name__ == '__main__':

    p = LabAdmittanceController(robotName)

    try:
        talker(p)
    except ros.ROSInterruptException:
        # these plots are for simulated robot
        p.plotStuff()
    
        
