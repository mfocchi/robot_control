# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

import rospy as ros
from base_controllers.utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from numpy import nan
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear, spawnMesh
from termcolor import colored
import os
import tf
from base_controllers.base_controller_fixed import BaseControllerFixed
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.msg import ContactsState
import scipy.io.matlab as mio
import rospkg
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
import matlab.engine
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import sys
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
from base_controllers.utils.common_functions import checkRosMaster
import  base_controllers.params as conf
robotName = "climbingrobot2"
# real robot msgs
import std_msgs, geometry_msgs
from climbingrobot_description.msg import RopeCommand
from climbingrobot_description.msg import PropellerCommand
from climbingrobot_description.msg import RopeTelemetry
from climbingrobot_description.msg import AlpineBodyTelemetry
# real robot services
from climbingrobot_description.srv import AlpineBodyCommand, AlpineBodyCommandRequest
from climbingrobot_description.srv import RopeControlMode, RopeControlModeRequest
from base_controllers.utils.common_functions import startNode, checkRosMaster, launchFileNode
from base_controllers.utils.math_tools import quaternion_matrix
from base_controllers.utils.ros_publish import RosPub
from orientation_controller import OrientationController
from sensor_msgs.msg import JointState

class ClimbingrobotController(BaseControllerFixed):
    def __init__(self, robot_name="ur5"):
        self.EXTERNAL_FORCE = False
        self.landing = True #do landing
        self.MPC_control = True
        self.PLOT_MPC = False

        self.SAVE_BAG = False # does not show rope vectors
        self.rope_index = np.array([2, 8]) #'wire_base_prismatic_r', 'wire_base_prismatic_l',
        self.leg_index = np.array([12, 13, 14])
        self.wheel_index = np.array([16, 18]) #'wheel_joint_l',  'wheel_joint_r'
        self.hip_pitch_joint = 12
        self.hip_roll_joint = 13
        self.base_passive_joints = np.array([3,4,5, 9,10,11])
        self.anchor_passive_joints = np.array([0,1, 6,7])
        self.OBSTACLE_AVOIDANCE = 'mesh' #'none', 'mesh'

        if self.MPC_control:
            sys.path.insert(0, './codegen_mpc')

        if self.OBSTACLE_AVOIDANCE=='mesh':
            sys.path.insert(0, './codegen_mesh')
            from base_controllers.components.terrain_manager import TerrainManager
            # generate terrain
            # Parameters (direct translation from MATLAB)
            wall_depth = 1  # how
            grid_size = 100
            max_ridge_depth = 0.5
            seed = "default"
            Lz = -20  # Height of wall in meters
            Ly = 5  # Width (horizontal extent) of wall in meters
            # Generate rock wall map
            self.terrainManager = TerrainManager()
            self.mesh_x, self.mesh_y, self.mesh_z = self.terrainManager.generate_rock_wall_map(Lz, Ly, grid_size, wall_depth, max_ridge_depth, seed, x_offset=-0.5)
        else:
            sys.path.insert(0, './codegen')

        self.force_scale = 60.
        self.mountain_thickness = 0.1 # TODO call the launch file passing this parameter
        self.r_leg = 0.3
        self.real_robot = conf.robot_params[robot_name]['real_robot']
        super().__init__(robot_name=robot_name)
        print("Initialized climbingrobot controller---------------------------------------------------------------")


    def apply_propeller_command(self, prop_thrusts=[0.,0.,0.,0.]):
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[0],
                               self.orientControl.b_propeller_axes[0] * prop_thrusts[0]/self.force_scale , "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[1],
                               self.orientControl.b_propeller_axes[1] * prop_thrusts[1] / self.force_scale, "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[2],
                               self.orientControl.b_propeller_axes[2] * prop_thrusts[2] / self.force_scale, "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[3],
                               self.orientControl.b_propeller_axes[3] * prop_thrusts[3] / self.force_scale, "blue", scale=1.5)
        msg =  PropellerCommand()
        msg.propeller_thrust_0 = prop_thrusts[0]
        msg.propeller_thrust_1 = prop_thrusts[1]
        msg.propeller_thrust_2 = prop_thrusts[2]
        msg.propeller_thrust_3 = prop_thrusts[3]
        self.pub_propeller_command.publish(msg)

    def getRobotMass(self):
        total_robot_mass = 5.0 #todo hardcode this
        return total_robot_mass

    def estimateRobotVelFromStates(self, l1, l2, psi, l1d, l2d, psid):
        if l1 != 0 and l2 != 0:
            # first estimate position
            px = l1 * np.sin(psi) * np.sqrt(1 - (self.anchor_distance_y ** 2 + l1 ** 2 - l2 ** 2) ** 2 / (4 * self.anchor_distance_y ** 2 * l1 ** 2))
            py = (self.anchor_distance_y ** 2 + l1 ** 2 - l2 ** 2) / (2 * self.anchor_distance_y)
            pz = -l1 * np.cos(psi) * np.sqrt(1 - (self.anchor_distance_y ** 2 + l1 ** 2 - l2 ** 2) ** 2 / (4 * self.anchor_distance_y ** 2 * l1 ** 2))

            #temp vars to simplify equation
            px_l1 = px / l1
            n_pz_l1 = -pz / l1
            px_l1_sinpsi = px / l1 / math.sin(psi+0.00001) # to avoid division by zero
            py2b = py * 2 * self.anchor_distance_y

            pdx = l1d * px_l1 + l1 * n_pz_l1 * psid + (py2b * math.sin(psi) * (l1d * pow(self.anchor_distance_y, 2) - l1d * pow(l1, 2) + 2 * l2d * l1 * l2 - l1d * pow(l2, 2))) / (4 * pow(self.anchor_distance_y, 2) * pow(l1, 2) * px_l1_sinpsi)
            pdy = (l1 * l1d - l2 * l2d) / self.anchor_distance_y
            pdz = l1 * psid * px_l1 - l1d * n_pz_l1 - (py2b * math.cos(psi) * (l1d * pow(self.anchor_distance_y, 2) - l1d * pow(l1, 2) + 2 * l2d * l1 * l2 - l1d * pow(l2, 2))) / (4 * pow(self.anchor_distance_y, 2) * pow(l1, 2) * px_l1_sinpsi)
            base_vel = np.array([pdx, pdy, pdz])
        else:
            base_vel = np.zeros(3)

        return base_vel

    def updateKinematicsDynamics(self):
        # get measured quantities
        self.w_R_rope = (quaternion_matrix(self.rope_l_imu_orientation))[:3,:3]

        self.w_R_b = quaternion_matrix(self.body_imu_orientation)[:3, :3]
        self.w_omega_b = self.body_imu_angular_velocity #TODO double check

        #rope gravity terms TO BE RECOMPUTED TODO
        #self.g #

        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.anchor_pos = np.array([conf.robot_params[self.robot_name]['spawn_x'], conf.robot_params[self.robot_name]['spawn_y'], conf.robot_params[self.robot_name]['spawn_z']])
        self.anchor_pos2 = np.array([conf.robot_params[self.robot_name]['spawn_2x'], conf.robot_params[self.robot_name]['spawn_2y'], conf.robot_params[self.robot_name]['spawn_2z']])
        self.anchor_distance_y = conf.robot_params[self.robot_name]['spawn_2y'] -  conf.robot_params[self.robot_name]['spawn_y']

        #estimate base position in WF throungh odometry from (l_1, psi)
        base_width = 0.1 #TODO
        com_offset = self.w_R_b[2]*0.1 #TODO
        leg_length = 0.3 #TODO
        x_rope_l_attach = self.anchor_pos + self.w_R_rope[0]*self.l_1
        self.base_pos = x_rope_l_attach + self.w_R_b[1] *(base_width/2) + com_offset

        #debug
        self.base_pos = np.array([1.5, 2.5, 16])

        self.base_rpy = self.math_utils.rot2eul(self.w_R_b)



        #compute ee position  in the world frame
        self.x_ee = self.base_pos - self.w_R_b[0] * leg_length

        self.hoist_l_pos = self.base_pos +  self.w_R_b.dot(np.array([0.0, -base_width/2, 0.]))
        self.hoist_r_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, base_width/2, 0.0]))
        self.rope_direction = (p.hoist_l_pos - p.anchor_pos) / np.linalg.norm(p.hoist_l_pos  - p.anchor_pos)
        self.rope_direction2 = (p.hoist_r_pos - p.anchor_pos2) / np.linalg.norm(p.hoist_r_pos - p.anchor_pos2)

        self.mat2Gazebo = self.anchor_pos
        self.base_pos_mat = self.base_pos - self.mat2Gazebo
        # offset between hoists
        hoist_distance = np.linalg.norm(self.hoist_l_pos - self.hoist_r_pos)

        #compute missin state variable phi / psi_d
        # old way (wrong)
        #self.psi = math.atan2(self.base_pos_mat[0], -self.base_pos_mat[2])
        # the psi variable is the extrinsic pitch wrt the world Y axis obtained expanding w_R_rope with extrinsic formula = Rx Ry Rz
        self.psi = math.atan2(self.w_R_rope[0,2], np.sqrt(math.pow(self.w_R_rope[0,0],2) + math.pow(self.w_R_rope[0,1],2)))
        # to get the derivative I need also the
        extr_roll =  math.atan2(-self.w_R_rope[1,2], self.w_R_rope[2,2])
        self.psi_d = np.cos(extr_roll) * self.w_omega_b[1] -np.sin(extr_roll) * self.w_omega_b[2]

        # now that we have also psid we can  estimate base velocity (in WF)
        self.base_vel = self.estimateRobotVelFromStates(self.l_1, self.l_2, self.psi, self.l_1d, self.l_2d, self.psi_d)

        # use geometric intuition for psid
        n_par = (self.anchor_pos - self.anchor_pos2) / np.linalg.norm(self.anchor_pos - self.anchor_pos2)
        rope2_axis = (self.base_pos - self.anchor_pos2) / np.linalg.norm(self.base_pos - self.anchor_pos2)
        self.n_bar = np.cross(n_par, rope2_axis) / np.linalg.norm(np.cross(n_par, rope2_axis))

        # the mountain is always wrt to world
        mountain_pos = np.array([-self.mountain_thickness/2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
        self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/wall', '/world')

        self.broadcaster.sendTransform(self.base_pos, self.body_imu_orientation, ros.Time.now(), '/base_link', '/world')

        #rviz
        self.q_des = self.solver.computeJointVariables(self.base_pos, self.w_R_b, self.q_des_q0, debug=False)
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.position = self.q_des
        self.pub_joints.publish(msg)

    def initVars(self):

        self.n_joints = len(conf.robot_params[self.robot_name]['joint_names'])

        self.q = np.zeros(self.n_joints)
        self.qd = np.zeros(self.n_joints)
        self.tau = np.zeros(self.n_joints)
        self.q_des = np.zeros(self.n_joints)
        self.qd_des = np.zeros(self.n_joints)
        self.tau_ffwd = np.zeros(self.n_joints)
        self.l_1 = 0
        self.l_2 = 0
        self.l_1d = 0
        self.l_2d = 0

        self.g = np.zeros(self.n_joints)
        self.x_ee = np.zeros(3)
        self.x_ee_des = np.zeros(3)

        self.contactForceW = np.zeros(3)
        self.contactMomentW = np.zeros(3)

        self.time = 0.
        self.rope_l_imu_orientation = np.array([0,0,0,1])
        self.rope_l_imu_angular_velocity = np.zeros((3))
        self.rope_l_imu_rpy = np.zeros((3))
        self.rope_l_imu_rpy_d = np.zeros((3))
        self.body_imu_orientation = np.array([0,0,0,1])
        self.body_imu_rpy = np.zeros((3))
        self.body_imu_angular_velocity = np.zeros((3))
        self.w_base_vel =np.zeros((3)) #TODO implement in odometry

        # log vars
        self.q_des_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_des_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_ffwd_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.x_ee_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.log_counter = 0


        self.qdd_des =  np.zeros(self.n_joints)
        self.base_accel = np.zeros(3)
        self.base_rpy = np.zeros(3)
        self.Fr_l_fbk = 0
        self.Fr_r_fbk = 0
        self.Fr_l = 0
        self.Fr_r = 0
        self.prop_force_x = 0
        self.touch_down_detected_l = False
        self.touch_down_detected_r = False
        self.optimal_control_traj_finished = False
        self.MPC_tracking_error = []

        # init new logged vars here
        self.com_log =  np.empty((3, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.simp_model_state_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        #self.ldot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.base_pos_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_rpy_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.time_jump_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_1d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_2d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.psid_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_vel_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.prop_force_x_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.contactForceW_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        w_R_wall = self.math_utils.eul2Rot(np.array([0,-conf.robot_params[p.robot_name]['wall_inclination'],0]))
        self.wall_normal = w_R_wall[:,0].copy() #take X axis, I need to use copy otherwise matlab complains is not contiguous

        self.mpc_index = 0
        self.mpc_index_old = 0
        self.mpc_index_ffwd = 0 # updated only when we stop recomputing mpc

        self.targetReceived = True # in sim just give hardcoded target

        self.prop_thrusts = [0]*4
        self.prop_thrusts_log = np.empty((4, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        propeller_orient = np.array([0.25 * np.pi, 0.75 * np.pi, np.pi + 0.25 * np.pi, np.pi + 0.75 * np.pi])
        self.orientControl = OrientationController(base_line_x = 0.1, base_line_y = 0.2, propeller_orient=propeller_orient)

        #rviz
        from closed_loop_inverse_kinematics import ClosedLoopKinSolver
        self.solver = ClosedLoopKinSolver(robot_name=self.robot_name)

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                self.simp_model_state_log[:, self.log_counter] = np.array([self.psi, self.l_1, self.l_2])
                # self.ldot_log[self.log_counter] = self.ldot
                self.base_pos_log[:, self.log_counter] = self.base_pos
                self.base_rpy_log[:, self.log_counter] = self.base_rpy
                self.Fr_l_log[self.log_counter] = self.Fr_l
                self.Fr_r_log[self.log_counter] = self.Fr_r
                self.Fr_l_fbk_log[self.log_counter] = self.Fr_l_fbk
                self.Fr_r_fbk_log[self.log_counter] = self.Fr_r_fbk
                self.l_1d_log[self.log_counter] =  self.l_1d
                self.l_2d_log[self.log_counter] =  self.l_2d
                self.psid_log[self.log_counter] = self.psi_d
                self.base_vel_log[:,self.log_counter] = self.w_base_vel

                self.prop_force_x_log[self.log_counter] = self.prop_force_x
                self.prop_thrusts_log[:, self.log_counter] = self.prop_thrusts

                #self.time_jump_log[self.log_counter] = self.time - self.end_thrusting

            super().logData()

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill -a")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")



    def plotStuff(self):
        print("PLOTTING")
        print(colored("The initial p0_x and mountain_pitch can be different by the desired ones computed by optim, even if we started optim from actual p0, "
                      "because the robot sags a bit due to leg reorientation","red"))
        # plotFrameLinear('com position', 1, p.time_log, None, p.com_log)
        # plotFrameLinear('contact force', 2, p.time_log, None, p.contactForceW_log)
        actual_com= p.base_pos_log - p.mat2Gazebo.reshape(3, 1) # mat2Gazebo is WF in matlab
        time_gazebo = p.time_log - p.start_logging
        plotJoint('position', time_gazebo, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
        plot3D('basePos', 2,  ['X', 'Y', 'Z'], time_gazebo, actual_com, p.ref_time, p.ref_com)
        plot3D('matlab states', 3, ['psi', 'l1', 'l2'], time_gazebo, p.simp_model_state_log, p.ref_time, np.vstack((p.ref_psi, p.ref_l_1, p.ref_l_2)) )

        # plot rope forces
        # plt.figure()
        # plt.subplot(2, 1, 1)
        # plt.ylabel("Fr_l")
        # plt.plot(p.ref_time, p.Fr_l0, color='red')
        # plt.plot(time_gazebo, p.Fr_l_log, color='blue')
        # plt.grid()
        # plt.subplot(2, 1, 2)
        # plt.ylabel("Fr_r")
        # plt.plot(p.ref_time, p.Fr_r0, color='red')
        # plt.plot(time_gazebo, p.Fr_r_log, color='blue')
        # plt.grid()

        # plot propeller thrusts
        plt.figure()
        plt.ylabel("prop_thrusts")
        plt.subplot(4, 1, 1)
        plt.grid()
        plt.plot(p.time_log, p.prop_thrusts_log[0,:], label="prop1", color='blue')
        plt.subplot(4, 1, 2)
        plt.grid()
        plt.plot(p.time_log, p.prop_thrusts_log[1,:], label="prop2", color='blue')
        plt.subplot(4, 1, 3)
        plt.grid()
        plt.plot(p.time_log, p.prop_thrusts_log[2,:], label="prop3", color='blue')
        plt.subplot(4, 1, 4)
        plt.plot(p.time_log, p.prop_thrusts_log[3,:], label="prop4", color='blue')
        plt.legend()
        plt.grid()
        plotFrameLinear('position', time_log=p.time_log, Pose_log=p.base_rpy_log)

        #save data
        filename = f'test_gazebo_MPC_{p.MPC_control}.mat'
        mio.savemat(filename, {'ref_time': p.ref_time, 'ref_com': p.ref_com,
                                'time_gazebo': time_gazebo, 'actual_com': actual_com,
                                'ref_psi':p.ref_psi,'ref_l_1':p.ref_l_1, 'ref_l_2':p.ref_l_2,
                                'psi': p.simp_model_state_log[0,:], 'l_1': p.simp_model_state_log[1,:], 'l_2': p.simp_model_state_log[2,:],
                                'psid': p.psid_log, 'l_1d': p.l_1d_log,'l_2d': p.l_2d_log,
                                'mu': p.mu , 'Fleg': p.Fleg,'Fr_max': p.Fr_max,
                                'Fr_l0': p.Fr_l0, 'Fr_r0': p.Fr_r0,
                                'Fr_l': p.Fr_l_log, 'Fr_r': p.Fr_r_log })

    def getIndex(self,t):
        try:
            # get index
            a_bool = self.jumps[self.jumpNumber]["time"] >= t
            idx = min([i for (i, val) in enumerate(a_bool) if val])-1
            if idx == -1:
                return 0
            else:
                return idx
        except:
            return  -1

    def getImpulseAngle(self):
        angle_hip_roll =  math.atan2(self.jumps[self.jumpNumber]["Fleg"][1],
                                self.jumps[self.jumpNumber]["Fleg"][0])
        angle_hip_pitch =  math.atan2(self.jumps[self.jumpNumber]["Fleg"][2], self.jumps[self.jumpNumber]["Fleg"][0])
        print(colored(f"Start orienting leg to (pitch, roll)  : {angle_hip_pitch, angle_hip_roll}", "blue"))
        angle_hip_pitch +=-1.57
        return angle_hip_pitch, angle_hip_roll

    # compute the passive and rope joints reference from the matlab position referred to a world frame located in between anchors
    def computeJointVariables(self, p):
        # mountain_wire_pitch_l = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_x'], -p[2])
        # mountain_wire_pitch_r = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_2x'], -p[2])
        if conf.robot_params[self.robot_name]['wall_inclination']>0.: #TODO missing normal in matlab wall_constraint!
            p[0] = (-p[2]) * math.tan(conf.robot_params[self.robot_name]['wall_inclination'])  #spawn_x is for the anchor point which is shifted wrt the wall
            print(f"adjusting initial position to be consistent with wall: {p}")

        mountain_wire_pitch_l = math.atan2(p[0] , -p[2])
        mountain_wire_pitch_r = math.atan2(p[0] , -p[2])

        mountain_wire_roll_l = -math.atan2(-p[2], p[1])
        mountain_wire_roll_r = math.atan2(-p[2], self.anchor_distance_y-p[1])
        # this is an approximation cause I shuould compute the real rope lenght considering the hoist distance so this function is only useful for init but it is inaccurate!
        wire_base_prismatic_l = np.linalg.norm(p) -self.anchor_distance_y*0.5
        wire_base_prismatic_r = math.sqrt(p[0]*p[0] +(self.anchor_distance_y - p[1])*(self.anchor_distance_y - p[1]) + p[2] * p[2])-self.anchor_distance_y*0.5

        wire_base_roll_l = -mountain_wire_roll_l
        wire_base_roll_r = -mountain_wire_roll_r
        return [mountain_wire_pitch_r, mountain_wire_roll_r,  wire_base_prismatic_r, 0., wire_base_roll_r, 0.,
                mountain_wire_pitch_l, mountain_wire_roll_l,  wire_base_prismatic_l, 0., wire_base_roll_l, 0.]

    def detectTouchDown(self):
        force_th = 10. #TODO implement a strategy based on accelerometer
        # if not self.touch_down_detected and (self.wall_normal.dot(self.contactForceW_l) > force_th):
        #     self.touch_down_detected = True
        # if self.touch_down_detected:
        #     print(colored("TouchDown Detected", "blue"))
        #     # sample com pos
        #     self.x_tilde0 =  self.wall_normal.reshape(1, 3) @ (self.com)# - self.x_p)
        #     return True
        # else:
        #     return False

    def resetRope(self):
        print(colored(f"Start Position Mode", "red"))
        # enable PD for rope and reset the PD reference to the new estension
        # sample the new elongation
        self.q_des[p.rope_index[0]] = np.copy(p.q[p.rope_index[0]])
        self.q_des[p.rope_index[1]] = np.copy(p.q[p.rope_index[1]])
        #print("resetting rope joints qdes : ", self.q_des[p.rope_index])
        self.Fr_r = 0.
        self.Fr_l = 0.
        self.tau_ffwd[p.rope_index] = np.zeros(2)
        self.setRopeControlMode('closed_loop_position')

    def printParams(self, p0, pf):
        print(colored(f"p0: {p0}","red"))
        print(colored(f"pf: {pf}","red"))
        print(colored(f"Fleg_max: {self.Fleg_max}","red"))
        print(colored(f"Fr_max: {self.Fr_max}", "red"))
        print(colored(f"mu: {self.mu}", "red"))
        print(colored(f"jump_clearance: {self.optim_params['jump_clearance']}", "red"))
        print(colored(f"mass: {self.optim_params['m']}", "red"))
        print(colored(f"num_params: {self.optim_params['num_params'] }", "red"))
        print(colored(f"int_method: {self.optim_params['int_method']}", "red"))
        print(colored(f"N_dyn: {self.optim_params['N_dyn']}", "red"))
        print(colored(f"FRICTION_CONE: {self.optim_params['FRICTION_CONE']}", "red"))
        print(colored(f"int_steps: {self.optim_params['int_steps']}", "red"))
        print(colored(f"contact_normal: {self.optim_params['contact_normal']}", "red"))
        print(colored(f"b: {self.optim_params['b']}", "red"))
        print(colored(f"p_a1: {self.optim_params['p_a1']}", "red"))
        print(colored(f"p_a2: {self.optim_params['p_a2']}", "red"))
        print(colored(f"g: {self.optim_params['g'] }", "red"))
        print(colored(f"w1: {self.optim_params['w1']}", "red"))
        print(colored(f"w2: {self.optim_params['w2']}", "red"))
        print(colored(f"w3: {self.optim_params['w3']}", "red"))
        print(colored(f"w4: {self.optim_params['w4']}", "red"))
        print(colored(f"w5: {self.optim_params['w5']}", "red"))
        print(colored(f"w6: {self.optim_params['w6']}", "red"))
        print(colored(f"T_th: {self.optim_params['T_th']}", "red"))

    def initOptim(self, p0, pf):
        ##offline optim vars
        self.Fleg_max = 300.
        self.Fr_max = 90.  # had to increas because of slopes downward jumps it used tp be 90
        self.Fr_min = 0.  # had to increas because of slopes downward jumps it used tp be 0
        # down ward jumps
        #self.Fr_max = 190.  # had to increas because of slopes downward jumps it used tp be 90
        #self.Fr_min = 15.  # had to increas because of slopes downward jumps it used tp be 0
        self.mu = 0.8
        self.optim_params = {}

        if self.OBSTACLE_AVOIDANCE=="mesh":
            self.optim_params['m'] = self.getRobotMass()
            self.optim_params['num_params'] = 4.
            self.optim_params['int_method'] = 'rk4'
            self.optim_params['N_dyn'] = 30.
            self.optim_params['FRICTION_CONE'] = 1.
            self.optim_params['int_steps'] = 5.
            self.optim_params['b'] = self.anchor_distance_y
            self.optim_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
            self.optim_params['p_a2'] = matlab.double([0., self.optim_params['b'], 0.]).reshape(3, 1)
            self.optim_params['g'] = 9.81
            self.optim_params['w1'] = 1.  # smooth
            self.optim_params['w2'] = 1.  # hoist work 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
            self.optim_params['w3'] = 0.
            self.optim_params['w4'] = 0.
            self.optim_params['w5'] = 0.
            self.optim_params['w6'] = 0.
            self.optim_params['T_th'] = 0.05
            self.optim_params['obstacle_avoidance'] = 'mesh'
            self.optim_params['jump_clearance'] = 1.
            # Interpolator (note: z must be increasing — here from -10 to 0)
            #correct initial and final positions
            p0[0] = self.terrainManager.wall_surface_eval(p0[2], p0[1],  self.mesh_x,  self.mesh_y,  self.mesh_z)
            pf[0] =  self.terrainManager.wall_surface_eval(pf[2], pf[1],  self.mesh_x,  self.mesh_y,  self.mesh_z)
            #does not work non matching with test_mex TODO
            # p0= np.array([0.99103, 2.5, -6.])
            # pf= np.array([0.40632, 4., -4.])

            # compute consistent normal
            normal = self.terrainManager.wall_normal_eval(p0[2], p0[1], self.mesh_x, self.mesh_y, self.mesh_z)
            self.optim_params['mesh_x'] = self.mesh_x
            self.optim_params['mesh_y'] = self.mesh_y
            self.optim_params['mesh_z'] = self.mesh_z
            self.optim_params['contact_normal'] = matlab.double(normal)
        else:
            self.optim_params['jump_clearance'] = 1.
            self.optim_params['m'] = self.getRobotMass()
            #if terrain is inclined we consider only the Y,Z component of the pf and we need to compute a target point consistent with the wall!
            if conf.robot_params[p.robot_name]['wall_inclination']>0.: #TODO missing normal in matlab wall_constraint!
                pf[0] = (-pf[2]) * math.tan(conf.robot_params[p.robot_name]['wall_inclination']) +  conf.robot_params[p.robot_name]['spawn_x'] #spawn_x is for the anchor point which is shifted wrt the wall
                print(f"adjusting landing target to be consistent with wall: {pf}")
            #no longer used
            self.optim_params['obstacle_avoidance'] = False
            self.optim_params['obstacle_location'] = matlab.double(np.zeros(3)).reshape(3, 1)
            self.optim_params['obstacle_size'] = matlab.double(np.zeros(3)).reshape(3, 1)
            self.optim_params['num_params'] = 4.
            self.optim_params['int_method'] = 'rk4'
            self.optim_params['N_dyn'] = 30.
            self.optim_params['FRICTION_CONE'] = 1.
            self.optim_params['int_steps'] = 5.
            self.optim_params['contact_normal'] = matlab.double([1,0,0]).reshape(3, 1)
            self.optim_params['b'] = self.anchor_distance_y
            self.optim_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
            self.optim_params['p_a2'] = matlab.double([0., self.optim_params['b'], 0.]).reshape(3, 1)
            self.optim_params['g'] = 9.81
            self.optim_params['w1'] = 1. # smooth
            self.optim_params['w2'] = 0. # hoist work 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
            self.optim_params['w3'] = 0.
            self.optim_params['w4'] = 0.
            self.optim_params['w5'] = 0.
            self.optim_params['w6'] = 0.
            self.optim_params['T_th'] = 0.05

        try:
            self.matvars = self.eng.optimize_cpp_mex(matlab.double(p0), matlab.double(pf), self.Fleg_max, self.Fr_max,  self.Fr_min, self.mu, self.optim_params)
        except:
            print(colored("Regenerate matlab code issues in calling optimize_cpp_mex","red"))
        # extract variables
        self.ref_com  = mat_matrix2python(self.matvars['p'])
        self.ref_psi = mat_vector2python(self.matvars['psi'])
        self.ref_l_1 = mat_vector2python(self.matvars['l1'])
        self.ref_l_2 = mat_vector2python(self.matvars['l2'])
        self.ref_time = mat_vector2python(self.matvars['time'])
        self.Fr_l0 = mat_vector2python(self.matvars['Fr_l'])
        self.Fr_r0 = mat_vector2python(self.matvars['Fr_r'])
        self.Fleg = mat_vector2python(self.matvars['Fleg'])
        #this is computed integrating the dynamics with dt and can be different from the reference, we should use the reference at the end of the horizon
        #self.targetPos = mat_vector2python(self.matvars['achieved_target'])
        self.targetPos = self.ref_com[:,-1] #output of optumization
        self.targetPosIdeal = self.ref_com[:, -1]
        print(colored(f"offline optimization accomplished, p0:{p0}, target(rough integr):{self.targetPos}", "blue"))
        print(colored(f"target to be compared with text_mex_x.py (fine integr. ) is:{self.matvars['achieved_target']}", "blue"))
        self.jumps = [{"time": self.ref_time, "thrustDuration" : self.matvars['T_th'], "p0": p0,
                    "targetPos": self.targetPos,  "Fleg":self.Fleg,
                    "Fr_r": self.Fr_r0, "Fr_l": self.Fr_l0,  "Tf": self.matvars['Tf'] }]

        # MPC vars (need to perform before normal optim to know Tf)
        self.mpc_N = int(0.4 * self.optim_params['N_dyn'])
        self.Fr_max_mpc = 100.

        self.optim_params_mpc = {}
        self.optim_params_mpc['int_method'] = 'rk4'
        self.optim_params_mpc['int_steps'] = 5.
        self.optim_params_mpc['contact_normal'] = matlab.double([1., 0., 0.]).reshape(3, 1)
        self.optim_params_mpc['b'] = self.anchor_distance_y
        self.optim_params_mpc['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
        self.optim_params_mpc['p_a2'] = matlab.double([0., self.optim_params_mpc['b'], 0.]).reshape(3, 1)
        self.optim_params_mpc['g'] = 9.81
        self.optim_params_mpc['m'] = self.getRobotMass()
        self.optim_params_mpc['w1'] = 1.
        self.optim_params_mpc['w2'] = 0.000001
        self.optim_params_mpc['mpc_dt'] = matlab.double(self.matvars['Tf'] / (self.optim_params['N_dyn'] - 1))
        self.deltaFr_l =np.zeros((int(self.mpc_N)))
        self.deltaFr_r = np.zeros((int(self.mpc_N)))
        self.propeller_force = np.zeros((int(self.mpc_N)))

        if self.PLOT_MPC:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)

    def computeMPC(self, delta_t):
        # after the thrust we start MPC,  it will start from time 0.05 so the index will start from  2
        if self.getIndex(delta_t) != -1:
            self.mpc_index = self.getIndex(delta_t)
        else:  # whenever the MPC should not be updated anymore use delta_t to imncrement mpc_index_ffwd
            #print("delta_t MOD dtMpc", (delta_t % self.optim_params_mpc['mpc_dt']))
            if (delta_t % self.optim_params_mpc['mpc_dt']) < 0.001:  # increment mpc_index_ffwd every mpc_dt
                self.mpc_index_ffwd += 1
                if self.mpc_index_ffwd > (self.mpc_N-1): # reference is finished keep the last computed one
                    self.mpc_index_ffwd = self.mpc_N-1
                #debug
                #print("stop mpc, applying ffwd, mpc_index_ffwd: ", self.mpc_index_ffwd)
        # This is better for const dist cause it keeps optimizing till the end!!!
        if (self.mpc_index != self.mpc_index_old): # do optim only every dtMPC  not every dt
            # reduce MPC horizon gradually at the end
            if ((self.mpc_index + self.mpc_N) >=len(self.ref_time)):
                self.mpc_N -=1
            # eval ref
            ref_com = matlab.double(self.ref_com[:, self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_l0 = matlab.double(self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_r0 = matlab.double(self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            actual_t = matlab.double(self.ref_time[self.mpc_index])
            actual_state = matlab.double([ self.psi, self.l_1, self.l_2, self.psi_d, self.l_1d, self.l_2d]).reshape(6,1)

            #perform optimization
            x = mat_vector2python(self.eng.optimize_cpp_mpc_propellers_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N, self.optim_params_mpc))
            # extract optim vars
            self.deltaFr_l = x[:self.mpc_N]
            self.deltaFr_r = x[self.mpc_N:2*self.mpc_N]
            self.propeller_force = x[2*self.mpc_N:3*self.mpc_N]


            # store tracking error for RMSE computation
            tracking_error = self.ref_com[:, self.mpc_index] - (self.base_pos - p.anchor_pos)
            self.MPC_tracking_error.append(np.linalg.norm(tracking_error))
            #online plot MPC
            if self.PLOT_MPC:
                self.onlinePlotMPC(self.deltaFr_l, self.deltaFr_r)

        self.mpc_index_old = self.mpc_index

        return self.deltaFr_l[self.mpc_index_ffwd],self.deltaFr_r[self.mpc_index_ffwd], self.propeller_force[self.mpc_index_ffwd]

    def onlinePlotMPC(self,deltaFr_l, deltaFr_r):
        # debug
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.set_label("delta Frl")
        self.ax2.set_label("delta Frr")
        self.ax1.grid()
        self.ax2.grid()
        #MPC action (red)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_l, "or-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_r, "or-")
        # full action (black)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_l, "ok-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_r, "ok-")
        # plot Fr limits Does not work
        # self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
        #               -self.Fr_max * np.ones((1, self.mpc_N)), "r-")
        # self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
        #               -self.Fr_max * np.ones((1, self.mpc_N)), "r-")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def computeJumpEnergyConsumption(self):
        # get index
        #a_bool = self.jumps[self.jumpNumber]["time"] > self.jumps[self.jumpNumber]["thrustDuration"]
        #lift_off_idx = min([i for (i, val) in enumerate(a_bool) if val]) - 1
        lift_off_idx = np.max(np.where((self.time_log - self.start_logging) <=self.jumps[self.jumpNumber]["thrustDuration"]))
        impulse_work= 0.5 * self.optim_params['m'] *self.base_vel_log[:, lift_off_idx].dot(self.base_vel_log[:, lift_off_idx]) # ekin at liftoff
        # this integral is done on a rough discretization dt
        touch_down_idx = np.max(np.where( (self.time_log - self.start_logging)  < p.jumps[p.jumpNumber]["Tf"]))
        hoist_work = 0.
        for i in range(touch_down_idx):
            hoist_work = hoist_work + (
                        abs(self.Fr_r_log[i] * self.l_2d_log[i]) + abs(self.Fr_l_log[i] * self.l_1d_log[i])) * conf.robot_params[p.robot_name]['dt']
        return impulse_work + hoist_work

    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        if self.real_robot:
            msg = RopeCommand()
            msg.rope_force = p.tau_ffwd[p.rope_index[0]]
            msg.rope_force = p.q_des[p.rope_index[0]]
            msg.rope_velocity = p.qd_des[p.rope_index[0]]
            msg.stamp = ros.Time.now()
            self.pub_rope_command_r()
            msg.rope_force = p.tau_ffwd[p.rope_index[1]]
            msg.rope_force = p.q_des[p.rope_index[1]]
            msg.rope_velocity = p.qd_des[p.rope_index[1]]
            self.pub_rope_command_l()

    # REAL ROBOT FUNCTIONS
    def startRealRobot(self):
        os.system("killall rviz gzserver gzclient")
        print(colored('------------------------------------------------ROBOT IS REAL!', 'blue'))
        checkRosMaster()
        #loads robot_description
        launchFileNode(package='climbingrobot_description',launch_file='upload.launch')
        startNode(package ='robot_state_publisher', executable='robot_state_publisher')
        startNode(package='rviz', executable='rviz', args='-d ' + rospkg.RosPack().get_path('climbingrobot_description') + '/rviz/conf.rviz')

    def startRealRobotPublisherSubscribers(self):


        self.ros_pub = RosPub(self.robot_name, only_visual=True, markers_time_to_live=0)
        self.broadcaster = tf.TransformBroadcaster()

        # this is for the matlab optim
        self.eng = matlab.engine.start_matlab()

        if self.OBSTACLE_AVOIDANCE=='mesh':
            self.eng.addpath('./codegen_mesh', nargout=0)
        else:
            self.eng.addpath('./codegen', nargout=0)

        if self.MPC_control:
            self.eng.addpath('./codegen_mpc', nargout=0)

        if self.SAVE_BAG:
            self.recorder = RosbagControlledRecorder(record_from_startup_=False)

        self.sub_rope_telemetry_l = ros.Subscriber("/winch/left/telemetry", RopeTelemetry,  callback=self._receive_rope_telemetry_l, queue_size=1,  tcp_nodelay=True)
        self.sub_rope_telemetry_r = ros.Subscriber("/winch/right/telemetry", RopeTelemetry, callback=self._receive_rope_telemetry_r, queue_size=1,  tcp_nodelay=True)
        self.pub_rope_command_l = ros.Publisher("/winch/left/command", RopeCommand,   queue_size=1,  tcp_nodelay=True)
        self.pub_rope_command_r = ros.Publisher("/winch/right/command", RopeCommand,   queue_size=1,  tcp_nodelay=True)
        self.rope_control_mode_l = ros.ServiceProxy('/winch/left/set_control_mode', RopeControlMode)
        self.rope_control_mode_r = ros.ServiceProxy('/winch/right/set_control_mode', RopeControlMode)
        # to orchestrator
        self.sub_des_target = ros.Subscriber("/planner/desired_target", geometry_msgs.msg.Vector3, callback=self._receive_target, queue_size=1, tcp_nodelay=True)
        self.pub_goal_status = ros.Subscriber("/planner/goal_status", std_msgs.msg.String,   queue_size=1, tcp_nodelay=True)

        #communication to alpine
        self.sub_alpine_telemetry = ros.Subscriber("/alpine_body/telemetry", AlpineBodyTelemetry, callback=self._receive_alpine_telemetry, queue_size=1, tcp_nodelay=True)
        self.pub_propeller_command = ros.Publisher("/alpine_body/propeller_command", PropellerCommand,   queue_size=1,  tcp_nodelay=True)
        self.alpine_command_service = ros.ServiceProxy('/alpine_body/command', AlpineBodyCommand) #TODO

        #for rviz

        self.pub_joints = ros.Publisher("/joint_states", JointState, queue_size=1, tcp_nodelay=True)

    def _receive_rope_telemetry_l(self, msg):
        self.Fr_l_meas = msg.rope_force
        self.l_1 = msg.rope_length
        self.l_1d = msg.rope_velocity
        self.brake_status_l = msg.brake_status
        self.q[p.rope_index[1]] =  self.l_1  + self.hoist_distance/2 - self.anchor_distance_y/2
        self.qd[p.rope_index[1]] = self.l_1d

    def _receive_rope_telemetry_r(self, msg):
        self.Fr_r_meas = msg.rope_force
        self.l_2 = msg.rope_length
        self.l_2d = msg.rope_velocity
        self.brake_status_r = msg.brake_status
        self.q[p.rope_index[0]] = self.l_2 + self.hoist_distance / 2 - self.anchor_distance_y / 2
        self.qd[p.rope_index[0]] = self.l_2d

    def _receive_target(self, msg):
        self.target = np.array([msg.x,msg.y,msg.z])
        self.targetReceived = True
        print(colored("received target {self.target}", "red"))

    def _receive_alpine_telemetry(self, msg):
        #rope
        self.rope_l_imu_orientation = np.array([
            msg.rope_imu_orientation.x,
            msg.rope_imu_orientation.y,
            msg.rope_imu_orientation.z,
            msg.rope_imu_orientation.w
        ])
        self.rope_l_imu_angular_velocity = np.array([ msg.rope_imu_angular_velocity.x, msg.rope_imu_angular_velocity.y,msg.rope_imu_angular_velocity.z])
        self.rope_l_imu_rpy = np.array([msg.rope_imu_rpy.x, msg.rope_imu_rpy.y, msg.rope_imu_rpy.z])
        self.rope_l_imu_rpy_d = np.array([msg.rope_imu_rpy_d.x, msg.rope_imu_rpy_d.y, msg.rope_imu_rpy_d.z])
        #body
        self.body_imu_orientation =  np.array([
                                        msg.body_imu_orientation.x,
                                        msg.body_imu_orientation.y,
                                        msg.body_imu_orientation.z,
                                        msg.body_imu_orientation.w
                                    ])
        self.body_imu_rpy = np.array([msg.body_imu_rpy.x, msg.body_imu_rpy.y, msg.body_imu_rpy.z])
        self.body_imu_angular_velocity =np.array([msg.body_imu_angular_velocity.x, msg.body_imu_angular_velocity.y, msg.body_imu_angular_velocity.z])

    def print_message(self, message = "", decimate = 1000):
        if not hasattr(self, 'print_counter'):
            self.print_counter = 0
        if np.mod(self.print_counter, decimate) == 0:
            print(colored(message, "red"))
        self.print_counter += 1

    def setRopeControlMode(self, mode='idle'):
        req = RopeControlModeRequest()
        req.mode = mode
        # Call the service
        try:
            resp = self.rope_control_mode_l(req)
            ros.loginfo("Service response: ack = %s", resp.ack)
        except ros.ServiceException as e:
            ros.logerr("Service call failed: %s" % e)
            return False
        try:
            resp = self.rope_control_mode_r(req)
            ros.loginfo("Service response: ack = %s", resp.ack)
        except ros.ServiceException as e:
            ros.logerr("Service call failed: %s" % e)
            return False

    def stateMachineLoop(self):
        terminateFlag = False
        # jump state machine
        if (p.stateMachine == 'idle') and (p.time >= p.startJump):
            # first run optim and fill in jump variable

            p.initOptim(p.base_pos - p.mat2Gazebo, p.target)

            # set the end of thrusting phase
            p.end_thrusting = p.startJump + p.jumps[p.jumpNumber]["thrustDuration"]
            p.start_logging = p.end_thrusting
            p.stateMachine = 'thrusting'  # this phase only waits is not doing anything

            if p.SAVE_BAG:
                p.recorder.start_recording_srv()

            print("\033[34m" + "---------Starting jump  number ", p.jumpNumber, " to optimized target: ",
                  p.jumps[p.jumpNumber]["targetPos"], " from actual p0 : ", p.base_pos - p.mat2Gazebo)
            print(colored(f"Start trusting", "blue"))
            p.tau_ffwd = np.zeros(p.n_joints)
            p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravitu in the virtual joint to go exactly there
            print(colored(f"Start Torque mode", "red"))
            p.setRopeControlMode('close_loop_torque')

            p.stateMachine = 'thrusting'
            p.w_Fleg = p.jumps[p.jumpNumber]["Fleg"]

        if (p.stateMachine == 'thrusting'):

            # service call to send thrust force and contact normal to ALPINE this will initiate the jump
            # Fill the request
            req = AlpineBodyCommandRequest()
            req.leg_force = np.linalg.norm(p.w_Fleg)
            req.contact_normal = geometry_msgs.msg.Vector3(x=p.wall_normal[0], y=p.wall_normal[1], z=p.wall_normal[2])
            # plot Fleg
            p.ros_pub.add_arrow(p.x_ee, np.linalg.norm(p.w_Fleg) * p.wall_normal / p.force_scale, "red", scale=2.5)

            # apply leg inpulse for thust duration
            try:
                resp = p.alpine_command_service(req)
                ros.loginfo("Service response: ack = %s", resp.ack)
            except ros.ServiceException as e:
                ros.logerr("Service call failed: %s" % e)

            # start also applying forces to ropes
            delta_t = p.time - p.end_thrusting
            p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]
            p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l

            if (p.time > p.end_thrusting):
                print(colored("Stop Trhusting", "blue"))
                p.tau_ffwd[p.leg_index] = np.zeros(len(p.leg_index))
                # retract leg for landing
                if p.landing:
                    p.stateMachine = 'flying_and_wait_for_touchdown'
                else:
                    p.stateMachine = 'flying'
                print(colored("Start " + p.stateMachine, "blue"))

        if (p.stateMachine == 'flying'):
            # after the thrust we start MPC it will start from time 0.05 so the index should be 12
            # applying forces to ropes
            delta_t = p.time - p.end_thrusting
            if p.MPC_control:
                # compute orientation control TODO
                # p.prop_force_x, p.prop_force_y, p.prop_moment_z = computeOrientationControl()
                deltaFr_l0, deltaFr_r0, p.prop_force_x = p.computeMPC(delta_t)
                p.apply_propeller_command(p.prop_force_x, p.prop_force_y, p.prop_moment_z)

            else:
                deltaFr_l0 = 0.
                deltaFr_r0 = 0.

            p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)] + deltaFr_l0
            p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)

            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            end_flying = p.startJump + p.jumps[p.jumpNumber]["Tf"]

            if (p.time >= end_flying):
                print(colored("Stop Flying", "blue"))
                # reset the qdes
                # we need to reset the rope PD because the Fr are finished and I would get the final value repeated  that is not the good thing to do
                # this will start again the position loop
                p.resetRope()
                energy = p.computeJumpEnergyConsumption()
                p.jumpNumber += 1
                if (p.jumpNumber < p.numberOfJumps):
                    p.stateMachine = 'idle'
                    # reset for multiple jumps
                    p.startJump = p.time
                else:

                    p.printLandingInfo()
                    if p.SAVE_BAG:
                        p.recorder.stop_recording_srv()
                    terminateFlag = True

            return terminateFlag
        # this is the same as flying but with the lander
        if (p.stateMachine == 'flying_and_wait_for_touchdown'):
            # applying forces to ropes, when time is finished just rset rope length (only once!) and wait for tf
            delta_t = p.time - p.end_thrusting

            if p.MPC_control:
                deltaFr_l0, deltaFr_r0, p.prop_force_x = p.computeMPC(delta_t)

                # compute orientation control TODO
                prop_forceW = p.n_bar * p.prop_force_x
                # compute thrust for orientation
                p.prop_thrusts, w_wrench = p.orientControl.computeThrust(des_orient=np.array([0, 0, 0.7]),
                                                                         act_orient=p.base_rpy,
                                                                         w_omega_b=p.w_omega_b,
                                                                         Ko=conf.robot_params[p.robot_name]['Ko'],
                                                                         Do=conf.robot_params[p.robot_name]['Do'], w_additional_force=prop_forceW)
                p.apply_propeller_command(p.prop_thrusts)
            else:
                deltaFr_l0 = 0.
                deltaFr_r0 = 0.

            if not p.optimal_control_traj_finished:
                if p.getIndex(delta_t) == -1:
                    p.optimal_control_traj_finished = True
                    print(colored("Trajectory finished, expecting delayed TD", "blue"))
                    # start again pid gains and reset qdes
                    p.resetRope()

                else:
                    p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)] + deltaFr_l0
                    p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0

                # check for early td and in case reset rope
                if p.detectTouchDown():
                    p.resetRope()
                    print(colored("Early TD detected, Start landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time
            else:  # you are checking for delayed TD you have already reset rope and restored PD
                if p.detectTouchDown():
                    print(colored("Start landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l

        if (p.stateMachine == 'landing'):
            print(colored("Start landing", "blue"))
            p.prop_force = (-25.)  # push against the wall
            p.apply_propeller_force(p.prop_force)
            landing_error = p.printLandingInfo()
            msg = std_msgs.msg.String()
            if np.linalg.norm(landing_error) < 0.5:
                msg.data = 'achieved'
            else:
                msg.data = 'error'
            p.pub_goal_status(msg)

            ####TODO
            pass

def talker(p):
    p.start()
    p.startRealRobot()
    p.startRealRobotPublisherSubscribers()

    # jump params
    # jump starting position
    p0 = np.array([0.28, 2.5, -6.10104])  # there is singularity for px = 0!
    # jump landing position
    p.target = np.array([0.28, 4, -4])


    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.updateKinematicsDynamics()


    p.startJump = 2.5
    p.stateMachine = 'idle'
    p.jumpNumber  = 0
    p.numberOfJumps = 1
    p.start_logging = np.inf

    # set the rope base joint variables to initialize in p0 position, the leg ones are defined in params.yaml
    p.q_des[:12] = p.computeJointVariables(p0)
    p.setRopeControlMode('close_loop_position')
    #the robot should go to the setpoint of the ropes

    #  wait for a target
    while True:
        if p.targetReceived:
            print(colored(f"---------------Ideal Target landing: {p.target}", "green"))
            break
        else:
            p.print_message("waiting for target", decimate=1000)
            rate.sleep()

    while not ros.is_shutdown():

        # update the kinematics
        p.updateKinematicsDynamics()

        print("AAA")
        # stop = p.stateMachineLoop()
        # if stop:
        #     break

        # plot ropes as green arrows only when you not save bags because they are ugly
        if not p.SAVE_BAG:
            p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
            p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos-p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo

        # plot contact force on retractable leg
        p.ros_pub.add_arrow(p.x_ee, p.contactForceW / p.force_scale, "blue", scale=2.5)

        #plot target position (whenever is available)
        try:
            p.ros_pub.add_marker(p.mat2Gazebo + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.3, alpha=1.)
            p.ros_pub.add_marker(p.mat2Gazebo + p.targetPosIdeal, color="green", radius=0.5, alpha=0.5)
        except:
            pass
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        p.ros_pub.add_mesh(mesh_path=os.environ['LOCOSIM_DIR'] + '/robot_descriptions/climbingrobot_description/meshes/runtime_mesh.obj', position=p.mat2Gazebo, color=None, alpha=1.0)
        p.ros_pub.publishVisual(delete_markers=False)

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),4)  # to avoid issues of dt 0.0009999
        if (p.time > p.start_logging):
            p.logData()
        # wait for synconization of the control loop
        rate.sleep()

    def printLandingInfo(self):
        landing_location = self.base_pos - self.mat2Gazebo
        print(colored(f" real landing (in matlab convention) is: {landing_location}", "blue"))
        print(colored(f" while from optim it should be  {self.targetPos}", "blue"))

        print(colored(f" the landing error is  {np.linalg.norm(landing_location - self.targetPos)}", "blue"))
        jump_length = np.linalg.norm(p0[:2] - self.targetPos[:2])
        MSE = np.square(np.array(p.MPC_tracking_error)).mean()
        RMSE = math.sqrt(MSE)
        print(colored(
            f" the relative landing error (norm per jump lenghth)  is {100 * np.linalg.norm(landing_location - self.targetPos) / jump_length}%",
            "blue"))
        print(colored(f" the energy consumption is  {energy}", "blue"))
        print(colored(f" the rmse of MPC tracking error is  {RMSE}", "blue"))
        print(colored(f" the leg impulse  is  {self.Fleg}", "blue"))
        print(colored(f" the norm of the leg impulse  is  {np.linalg.norm(self.Fleg)}", "blue"))
        self.plotStuff()
        return self.targetPos - landing_location

def plot3D(name, figure_id, label, time_log, var, time_mat = None, var_mat = None):
    fig = plt.figure()
    fig.suptitle(name, fontsize=20)
    plt.subplot(3,1,1)
    plt.ylabel(label[0])
    plt.plot(time_log, var[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    plt.grid(True)
    plt.legend(['act', 'ref'])

    plt.subplot(3,1,2)
    plt.ylabel(label[1])
    plt.plot(time_log, var[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])

    plt.subplot(3,1,3)
    plt.ylabel(label[2])
    plt.plot(time_log, var[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])

if __name__ == '__main__':
    p = ClimbingrobotController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        if p.landing: # for the landing test you should press Ctrl C to stop everything
            p.plotStuff()
            if p.SAVE_BAG:
                p.recorder.stop_recording_srv()


        
