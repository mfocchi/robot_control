from controller import Controller

import numpy as np

import rospy as ros

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

from utils.ros_publish import RosPub

from common_functions import checkRosMaster

import params as conf

class ExpController(Controller):
    def __init__(self, robot_name):
        checkRosMaster()
        super().__init__(robot_name)

        self.dt = conf.robot_params[robot_name]['dt_exp']
        self.ros_pub = RosPub(robot_name, only_visual=False, visual_frame = "base_link")

        # Additional variables
        self.IMU_quat = np.empty(4)*np.nan
        self.IMU_ang_vel = np.empty(3)*np.nan
        self.IMU_lin_accel = np.empty(3)*np.nan

        # Subscribers
        self.sub_jstate = ros.Subscriber("/joint_states", JointState, callback=self._receive_jstate, queue_size=1,
                                         tcp_nodelay=True)

        self.sub_jstate_des = ros.Subscriber("/joint_states", JointState, callback=self._receive_jstate_des, queue_size=1,
                                         tcp_nodelay=True)

        self.sub_imu = ros.Subscriber("/imu", Imu, callback=self._receive_imu, queue_size=1,
                                      tcp_nodelay=True)


    def _receive_jstate_des(self, msg):
        self.joint_names = msg.name
        q_des_ros = np.zeros(self.robot.na)
        qd_des_ros = np.zeros(self.robot.na)
        tau_des_ros = np.zeros(self.robot.na)
        for i in range(na):
            q_des_ros[i] = msg.position[i]
            qd_des_ros[i] = msg.velocity[i]
            tau_des_ros[i] = msg.effort[i]

        # map from ROS (alphabetical) to our  LF RF LH RH convention
        self.q_des = self.u.mapFromRos(q_des_ros)
        self.qd_des = self.u.mapFromRos(qd_des_ros)
        self.tau_des = self.u.mapFromRos(tau_des_ros)

    def _recieve_imu(self, msg):
        self.IMU_quat[0] = msg.orientation.x
        self.IMU_quat[1] = msg.orientation.y
        self.IMU_quat[2] = msg.orientation.z
        self.IMU_quat[3] = msg.orientation.w

        self.IMU_ang_vel[0] = msg.angular_velocity.x
        self.IMU_ang_vel[1] = msg.angular_velocity.y
        self.IMU_ang_vel[2] = msg.angular_velocity.z

        self.IMU_lin_accel[0] = msg.linear_acceleration.x
        self.IMU_lin_accel[1] = msg.linear_acceleration.y
        self.IMU_lin_accel[2] = msg.linear_acceleration.z

        # TODO update base pose and twist
        euler = euler_from_quaternion(self.quaternion)
        self.basePoseW[0] = 0
        self.basePoseW[1] = 0
        self.basePoseW[2] = 0
        self.basePoseW[3] = 0
        self.basePoseW[4] = 0
        self.basePoseW[5] = 0

        self.baseTwistW[0] = 0
        self.baseTwistW[1] = 0
        self.baseTwistW[2] = 0
        self.baseTwistW[3] = 0
        self.baseTwistW[4] = 0
        self.baseTwistW[5] = 0

        # compute orientation matrix
        self.b_R_w = self.mathJet.rpyToRot(euler)


    def estimateContacts(self):
        q_ros = self.u.mapToRos(self.q)
        qd_ros = self.u.mapToRos(self.qd)
        tau_ros = self.u.mapToRos(self.tau)
        # Pinocchio Update the joint and frame placements
        configuration = np.hstack((self.basePoseW[0:3], self.quaternion, q_ros))
        gen_velocities = np.hstack((self.baseTwistW, qd_ros))

        gravity_torques = self.robotPin.gravity(configuration)
        joint_gravity_torques = gravity_torques[6:]

        self.contacts_state[:] = False


        for leg in range(4):
            grf = np.linalg.inv(self.bJ[leg].T).dot(self.u.getLegJointState(leg, gravity_torques - tau_ros)) # TODO: maybe there is an error here
            self.u.setLegJointState(leg, grf, self.grForcesB)
            self.contacts_state[leg] = grf[2] > self.force_th


        # estimate ground reaction forces from tau
        for leg in range(4):
            grf = np.linalg.inv(self.bJ[leg].T).dot(self.u.getLegJointState(leg, joint_gravity_torques - self.tau))
            ros_pub.add_arrow(self.B_contact[leg], grf / (6 * self.robotPin.robot_mass), "green")

            self.contacts_state[leg] = grf[2] > force_th
            # print(leg, "   ", joints.contacts_state[leg] )
            if (self.contacts_state[leg]):
                ros_pub.add_marker(B_contact, radius=0.1)
            else:
                ros_pub.add_marker(B_contact, radius=0.001)


    def visualizeContacts(self):
        for leg in range(4):
            self.ros_pub.add_arrow(self.B_contacts[leg],
                                   self.u.getLegJointState(leg, self.grForcesB / (6 * self.robot.robot_mass)), "green")
            if self.contacts_state[leg]:
                ros_pub.add_marker(self.B_contacts, radius=0.1)
            else:
                ros_pub.add_marker(self.B_contacts, radius=0.001)
        self.ros_pub.publishVisual()




    def computeCOM(self):
        self.com, self.v_com = self.leg_odom.estimate_com(self.IMU_quat, self.IMU_ang_vel, self.q, self.qd, self.contacts_state, self.bJ)








