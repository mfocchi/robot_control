from controller import Controller

import numpy as np
import pinocchio as pin
import rospy as ros

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import tf

from utils.ros_publish import RosPub

from common_functions import checkRosMaster

import params as conf


class ExpController(Controller):
    def __init__(self, robot_name):
        checkRosMaster()
        super().__init__(robot_name)

        self.dt = conf.robot_params[robot_name]['dt']   #['dt_exp']
        self.ros_pub = RosPub(robot_name, only_visual=False, visual_frame = "base_link")
        self.rate = ros.Rate(1 / self.dt)

        # Additself.quaternion = np.empty(4)*np.nanional variables

        self.IMU_ang_vel = np.empty(3)*np.nan
        self.IMU_lin_accel = np.empty(3)*np.nan

        self.b_p_IMU = np.array([-0.10407, 0, 0])#-0.00635, 0.01540]) # y and z values may be wrong

        self._w_vl_imu = np.zeros(3)
        self.w_vl_b = np.zeros(3)
        self.w_p_b = np.zeros(3)

        # Subscribers
        self.sub_jstate = ros.Subscriber("/joint_states", JointState, callback=self._receive_jstate, queue_size=1, tcp_nodelay=True)

        self.sub_jstate_des = ros.Subscriber("/joint_states_des", JointState, callback=self._receive_jstate_des, queue_size=1,
                                         tcp_nodelay=True)

        self.sub_imu = ros.Subscriber("/imu", Imu, callback=self._receive_imu, queue_size=1, tcp_nodelay=True)

        self.sub_ready = ros.Subscriber("/robot_ready", Bool, callback=self._recieve_ready, queue_size=1,
                                         tcp_nodelay=True)

        self.sub_started = ros.Subscriber("/task_active", Bool, callback=self._recieve_task_active, queue_size=1,
                                        tcp_nodelay=True)

        self.pub_baseTwist = ros.Publisher("/base_twist", Twist, queue_size=1, tcp_nodelay=True)


        self.broadcaster = tf.TransformBroadcaster()

        self.isRobotReady = False
        self.task_active = False
        self.elapsed_time = 0
        self.time = 0

        self.Kp = conf.robot_params[robot_name]['kp']
        self.Kd = conf.robot_params[robot_name]['kd']

    def _recieve_ready(self, msg):
        self.isRobotReady = msg.data

    def _recieve_task_active(self, msg):
        self.task_active = msg.data

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



    def _receive_imu(self, msg):
        self.quaternion[0] = msg.orientation.x
        self.quaternion[1] = msg.orientation.y
        self.quaternion[2] = msg.orientation.z
        self.quaternion[3] = msg.orientation.w

        self.IMU_ang_vel[0] = msg.angular_velocity.x
        self.IMU_ang_vel[1] = msg.angular_velocity.y
        self.IMU_ang_vel[2] = msg.angular_velocity.z

        self.IMU_lin_accel[0] = msg.linear_acceleration.x
        self.IMU_lin_accel[1] = msg.linear_acceleration.y
        self.IMU_lin_accel[2] = msg.linear_acceleration.z
        # # TODO: update base pose and twist
        # euler = euler_from_quaternion(self.quaternion)
        # self.basePoseW[0] = 0
        # self.basePoseW[1] = 0
        # self.basePoseW[2] = 0
        # self.basePoseW[3] = 0
        # self.basePoseW[4] = 0
        # self.basePoseW[5] = 0
        #
        # self.baseTwistW[0] = 0
        # self.baseTwistW[1] = 0
        # self.baseTwistW[2] = 0
        # self.baseTwistW[3] = 0
        # self.baseTwistW[4] = 0
        # self.baseTwistW[5] = 0
        # compute orientation matrix
        self.b_R_w = self.mathJet.rpyToRot(euler)

    def publishBaseTwist(self):
        msg = Twist()

        msg.linear.x = self.w_vl_b[0]
        msg.linear.y = self.w_vl_b[1]
        msg.linear.z = self.w_vl_b[2]

        msg.angular.x = self.IMU_ang_vel[0]
        msg.angular.y = self.IMU_ang_vel[1]
        msg.angular.z = self.IMU_ang_vel[2]

        self.pub_baseTwist.publish(msg)


    def estimateContacts(self):
        q_ros = self.u.mapToRos(self.q)
        # Pinocchio Update the joint and frame placements
        configuration = np.hstack(([0,0,0], self.quaternion, q_ros))
        neutral_configuration = np.hstack((pin.neutral(self.robot.model)[0:7], q_ros))

        pin.forwardKinematics(self.robot.model, self.robot.data, neutral_configuration)
        pin.computeJointJacobians(self.robot.model, self.robot.data)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        gravity_torques = self.robot.gravity(configuration)
        joint_gravity_torques = gravity_torques[6:]

        self.contacts_state[:] = False

        for leg in range(4):
            self.B_contacts[leg] = self.robot.framePlacement(neutral_configuration,
                                                     self.robot.model.getFrameId(self.ee_frames[leg])).translation
            leg_joints = range(6 + self.u.mapIndexToRos(leg) * 3, 6 + self.u.mapIndexToRos(leg) * 3 + 3)
            self.bJ[leg] = self.robot.frameJacobian(neutral_configuration,
                                                       self.robot.model.getFrameId(self.ee_frames[leg]),
                                                       pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, leg_joints]
            grf = np.linalg.inv(self.bJ[leg].T).dot(self.u.getLegJointState(leg, joint_gravity_torques - self.tau))

            self.u.setLegJointState(leg, grf, self.grForcesB)
            self.contacts_state[leg] = grf[2] > force_th



    def visualizeContacts(self):
        for leg in range(4):
            self.ros_pub.add_arrow(self.B_contacts[leg],
                                   self.u.getLegJointState(leg, self.grForcesB/ self.robot.robotMass() ), "green")
            if self.contacts_state[leg]:
                self.ros_pub.add_marker(self.B_contacts[leg], radius=0.1)
            else:
                self.ros_pub.add_marker(self.B_contacts[leg], radius=0.001)
        self.ros_pub.publishVisual()


    def updateBase(self):
        w_vl_b_old = self.w_vl_b.copy()
        alpha = 0.1
        self._w_vl_imu = alpha * self._w_vl_imu + (1-alpha) * (self._w_vl_imu + self.IMU_lin_accel * self.dt) # usa media mobile invece che filtro integrale
        self.w_vl_b = self._w_vl_imu #- pin.skew(self.IMU_ang_vel) @ self.b_R_w.T @ self.b_p_IMU
        self.w_p_b += self.w_vl_b*self.dt
        #print('base velocity in world: ', self.w_vl_b)
        #self.w_p_b += self.w_vl_b * self.dt

        self.broadcaster.sendTransform( (0,0,0), self.quaternion, ros.Time.now(), '/base_link', '/world')

        if w_vl_b_old[2] >= 0 and self.w_vl_b[2] < 0:
            print('APEX DETECTED: ', w_vl_b_old[2], self.w_vl_b[2])

        self.publishBaseTwist()



    def computeCOM(self):
        pass
        #self.com, self.v_com = self.leg_odom.estimate_com(self.quaternion, self.IMU_ang_vel, self.q, self.qd, self.contacts_state, self.bJ)

    def testApex(self):
        pass

    def bag(self):
        stamp = ros.Time.now()
        msg_task = Bool()
        msg_task.data = self.task_active
        self.sensors_bag.write('/task_active', msg_task)

        msg_js = JointState()
        msg_js.header.stamp = stamp
        index_of_actuated_joints = 2
        # remove universe and base joints that are not actuated
        msg_js.name = self.ros_joints_name[index_of_actuated_joints:]
        msg_js.position = self.u.mapToRos(self.q)
        msg_js.velocity = self.u.mapToRos(self.qd)
        msg_js.effort = self.u.mapToRos(self.tau)

        self.sensors_bag.write('/joint_states', msg_js)

        msg_imu = Imu()
        msg_imu.header.stamp = stamp
        msg_imu.orientation.x = self.quaternion[0]
        msg_imu.orientation.y = self.quaternion[1]
        msg_imu.orientation.z = self.quaternion[2]
        msg_imu.orientation.w = self.quaternion[3]

        msg_imu.angular_velocity.x = self.IMU_ang_vel[0]
        msg_imu.angular_velocity.y = self.IMU_ang_vel[1]
        msg_imu.angular_velocity.z = self.IMU_ang_vel[2]

        msg_imu.linear_acceleration.x = self.IMU_lin_accel[0]
        msg_imu.linear_acceleration.y = self.IMU_lin_accel[1]
        msg_imu.linear_acceleration.z = self.IMU_lin_accel[2]

        self.sensors_bag.write('/imu', msg_imu)


    def jointFeedback(self, q, q_des, qd, qd_des):
        tau_fb = self.Kp @ (q_des-q) + self.Kd @ (qd_des-qd)
        return tau_fb