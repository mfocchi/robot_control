# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function

import os
import rospy as ros
import sys
# messages for topic subscribers
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench, Point
from std_srvs.srv import Trigger

# ros utils
import roslaunch
import rosnode
import rosgraph
import rospkg
import tf
from rospy import Time

#other utils
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
robotName = "ur5"

# controller manager management
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from std_msgs.msg import Float64MultiArray

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

#from motionplanner import MoveItCartesianPath
from obstacle_avoidance.obstacle_avoidance import ObstacleAvoidance
from base_controllers.base_controller_fixed import BaseControllerFixed


class AdmittanceControl():
    
    def __init__(self, ikin, Kp, Kd, conf):
        self.ikin = ikin
        self.conf = conf

        self.dx = np.zeros(3)
        self.dx_1_old = np.zeros(3)
        self.dx_2_old = np.zeros(3)
        self.Kp = Kp
        self.Kd = Kd
        self.q_postural =   self.conf['q_0']

    def setPosturalTask(self, q_postural):
        self.q_postural = q_postural

    def computeAdmittanceReference(self, Fext, x_des, q_guess):
        # only Kp, Kd
        self.dx = np.linalg.inv(self.Kp + 1/self.conf['dt'] * self.Kd).dot(Fext + 1/self.conf['dt']*self.Kd.dot(self.dx_1_old))
        #only Kp (unstable)
        #self.dx = np.linalg.inv(self.Kp).dot(Fext)
        self.dx_2_old = self.dx_1_old
        self.dx_1_old = self.dx
        # you need to remember to remove the base offset! because pinocchio us unawre of that!
        q_des, ik_success, out_of_workspace = self.ikin.endeffectorInverseKinematicsLineSearch(x_des   + self.dx,
                                                                                               self.conf['ee_frame'],
                                                                                               q_guess, False, False,
                                                                                               postural_task=True,
                                                                                               w_postural=0.00001,
                                                                                               q_postural= self.q_postural)

        return q_des, x_des + self.dx

class LabAdmittanceController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_flag = self.real_robot
        if (conf.robot_params[self.robot_name]['control_type'] == "torque"):
            self.use_torque_control = 1
        else:
            self.use_torque_control = 0
        print("Initialized L8 admittance  controller---------------------------------------------------------------")

    def startReadlRobot(self):
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

    def loadModelAndPublishersSon(self, xacro_path):
        super().loadModelAndPublishers(xacro_path)
        self.sub_ftsensor = ros.Subscriber("/" + self.robot_name + "/wrench", WrenchStamped,
                                           callback=self._receive_ftsensor, queue_size=1, tcp_nodelay=True)

        self.switch_controller_srv = ros.ServiceProxy(
            "/" + self.robot_name + "/controller_manager/switch_controller", SwitchController)
        self.load_controller_srv = ros.ServiceProxy("/" + self.robot_name + "/controller_manager/load_controller",
                                                    LoadController)
        self.pub_reduced_des_jstate = ros.Publisher("/" + self.robot_name + "/joint_group_pos_controller/command",
                                                    Float64MultiArray, queue_size=10)

        self.zero_sensor = ros.ServiceProxy("/" + self.robot_name + "/ur_hardware_interface/zero_ftsensor", Trigger)
        if self.real_robot:
            self.available_controllers = [
                "joint_group_pos_controller",
                "scaled_pos_joint_traj_controller"
            ]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller"
                                          ]
        self.active_controller = self.available_controllers[0]

        self.admit = AdmittanceControl(self.ikin, 600 * np.identity(3), 300 * np.identity(3),
                                       conf.robot_params[self.robot_name])


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
        # broadcast base world TF if they are different
        self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')

    def estimateContactForces(self):  
        # estimate ground reaxtion forces from tau
        if self.use_torque_control:
            self.contactForceW = np.linalg.inv(self.J6.T).dot(self.h-self.tau)[:3]
                                 
    def startupProcedure(self):
        if (self.use_torque_control):
            #set joint pdi gains
            self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
            #only torque loop
            #self.pid.setPDs(0.0, 0.0, 0.0)
        if (p.real_robot):
            self.zero_sensor()

    def initVars(self):
        super().initVars()
        # log variables relative to admittance controller
        self.q_des_old = np.zeros(self.robot.model.nq)
        self.q_des_adm_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.xee_des_adm_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.obs_avoidance = ObstacleAvoidance()
        # position of the center is in WF
        self.obs_avoidance.setCubeParameters(0.25, np.array([0.125, 0.75,0.975]))
        self.obs_avoidance.setCylinderParameters(0.125, 0.3, np.array([0.6, 0.25, 1.0]))

    def logData(self):
        if (conf.robot_params[self.robot_name]['control_type'] == "admittance"):
            self.q_des_adm_log[:, self.log_counter] = self.q_des_adm
            self.xee_des_adm_log[:, self.log_counter] = self.x_ee_des_adm
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
          
       
    def send_reduced_des_jstate(self, q_des):     
        msg = Float64MultiArray()
        msg.data = q_des             
        self.pub_reduced_des_jstate.publish(msg) 

    def send_joint_trajectory(self):
        """Creates a trajectory and sends it using the selected action server"""
        if self.real_robot:
            trajectory_client = actionlib.SimpleActionClient(
                "{}/follow_joint_trajectory".format("/" + self.robot_name + "/scaled_pos_joint_traj_controller"),
                FollowJointTrajectoryAction,
            )
        else:
            trajectory_client = actionlib.SimpleActionClient(
                "{}/follow_joint_trajectory".format("/" + self.robot_name + "/pos_joint_traj_controller"),
                FollowJointTrajectoryAction,
            )

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
    
def talker(p):
            
    p.start()

    if p.real_robot:
        p.startReadlRobot()
    else:
        p.startSimulator(p.use_torque_control)

    # specify xacro location
    xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.xacro'
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    p.startupProcedure()
    ros.sleep(1.0)

    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
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

            # EXE L7-1: set constant joint reference
            p.q_des = np.copy(p.q_des_q0)

            # EXE L7-2  set constant ee reference
            # p.x_ee_des = np.array([-0.3, 0.5, -0.6])
            # p.q_des, ok, out_ws = p.ikin.endeffectorInverseKinematicsLineSearch(p.x_ee_des,
            #                                                                     conf.robot_params[p.robot_name][
            #                                                                         'ee_frame'], p.q, False, False,
            #                                                                     postural_task=True, w_postural=0.00001,
            #                                                                     q_postural=p.q_des_q0)

            # EXE L7-3  set constant ee reference and desired orientation
            # rpy_des = np.array([ -1.9, -0.5, -0.1])
            # w_R_e_des = p.math_utils.eul2Rot(rpy_des) # compute rotation matrix representing the desired orientation from Euler Angles
            # p.q_des, ok, out_ws = p.ikin.endeffectorFrameInverseKinematicsLineSearch(p.x_ee_des, w_R_e_des, conf.robot_params[p.robot_name]['ee_frame'], p.q)

            # EXE L7-4 - polynomial trajectory (TODO)

            # EXE L7-5.3: set sinusoidal joint reference
            #p.q_des  = p.q_des_q0  +0.2*np.sin(2*3.14*p.time)

            # EXE 5 - admittance control
            # p.x_ee_des = p.robot.framePlacement(p.q_des,p.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).translation
            # p.q_des_adm, p.x_ee_des_adm = p.admit.computeAdmittanceReference(p.contactForceW, p.x_ee_des, p.q)

            # EXE L7-6 - load estimation
            # Fload = np.linalg.pinv(p.J.T).dot(p.tau - p.g)
            # payload_weight = -Fload[2]/9.81

            # controller with gravity coriolis comp
            p.tau_ffwd = p.h + np.zeros(p.robot.na)
            # only torque loop
            #p.tau_ffwd = conf.robot_params[p.robot_name]['kp']*(np.subtract(p.q_des,   p.q))  - conf.robot_params[p.robot_name]['kd']*p.qd

            # override the position command if you use admittance controller
            if (p.time > 1.5) and (conf.robot_params[p.robot_name]['control_type'] == "admittance"):  # activate admittance control only after a few seconds to allow initialization
                q_to_send = p.q_des_adm
            else:
                q_to_send = p.q_des

            # send commands to gazebo
            if (p.use_torque_control):
                if  (p.time > 1.5):
                    p.pid.setPDs(0.0, 10.2, 0.0)
                    # since the obstacles are defined in the WF I need to add the offset
                    d_cyl, cyl_closest_point = p.obs_avoidance.getCylinderDistance(p.x_ee + p.base_offset)
                    p.ros_pub.add_marker(cyl_closest_point, radius=0.05, color="blue")
                    d_cube, cube_closest_point = p.obs_avoidance.getCubeDistance(p.x_ee + p.base_offset)
                    p.ros_pub.add_marker(cube_closest_point, radius=0.05, color="blue")
                    goal = np.array([0.7, 0.8, 1.1])
                    tau_field, f_repulsive_cube, f_repulsive_cyl, f_attractive = p.obs_avoidance.evaluatePotentials(goal,                                                                                                                   p.base_offset,
                                                                                                                    p.robot,
                                                                                                                    p.q)
                    p.ros_pub.add_marker(goal, radius=0.15, color="green")
                    p.ros_pub.add_arrow(p.x_ee + p.base_offset, f_repulsive_cube[0], "red")
                    p.ros_pub.add_arrow(p.x_ee + p.base_offset, f_repulsive_cyl[0], "red")
                    p.ros_pub.add_arrow(p.x_ee + p.base_offset, f_attractive, "green")
                    p.tau_ffwd =  p.h + np.copy(tau_field)
                p.send_des_jstate(q_to_send, p.qd_des, p.tau_ffwd)

            else:
                p.send_reduced_des_jstate(q_to_send)

            #p.ros_pub.add_arrow(p.x_ee + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")
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
                print ("Shutting Down")
                break

    print("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()
    # this is to plot on REAL robot that is running a different rosnode (TODO SOLVE)
    if (conf.robot_params[p.robot_name]['control_type'] == "admittance"):
        plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                  p.tau_ffwd_log, p.joint_names,p.q_des_adm_log)
        plotAdmittanceTracking(2, p.time_log, p.xee_log, p.xee_des_log, p.xee_des_adm_log, p.contactForceW_log)
    else:
        plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
              p.tau_ffwd_log, p.joint_names,p.q_des_log)
    plotJoint('torque', 2, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
               p.tau_ffwd_log, p.joint_names)
    plotEndeff('force', 1, p.time_log, p.xee_log,  f_log=p.contactForceW_log)
    plt.show(block=True)


if __name__ == '__main__':

    p = LabAdmittanceController(robotName)

    try:
        talker(p)
    except ros.ROSInterruptException:
        # these plots are for simulated robot

        if (conf.robot_params[p.robot_name]['control_type'] == "admittance"):
            plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, p.joint_names, p.q_des_adm_log)
            plotAdmittanceTracking(2, p.time_log, p.xee_log, p.xee_des_log, p.xee_des_adm_log, p.contactForceW_log)
        else:
            plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, p.joint_names)
        if (p.use_torque_control):
            plotJoint('torque', 1, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                   p.tau_ffwd_log, p.joint_names)
        plt.show(block=True)
    
        
