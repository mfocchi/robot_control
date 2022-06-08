from base_controllers.controller import Controller, conf

import numpy as np
import pinocchio as pin
import rospy
# ros utils
import roslaunch
import rosnode
import rosgraph

from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.pidManager import PidManager
from base_controllers.utils.math_tools import *

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
#from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


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


from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty

from gazebo_msgs.srv import ApplyBodyWrench

from scipy.io import savemat

import os

import time



class SimController(Controller):
    def __init__(self, robot_name, launch_file=None):
        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")
        if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
            print('ROS MASTER is active')
            nodes = rosnode.get_node_names()
            if "/rviz" in nodes:
                print("Rviz active")
                rvizflag = " rviz:=false"
            else:
                rvizflag = " rviz:=true"

        self.robot_name = robot_name
        # start ros impedance controller
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        if launch_file is None:
            launch_file = self.robot_name
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/ros_impedance_controller/launch/ros_impedance_controller_"+launch_file+".launch"])
        self.launch.start()
        rospy.sleep(1.5)

        super().__init__(self.robot_name)
        self.dt = conf.robot_params[self.robot_name]['dt']
        self.robot.q_home = conf.robot_params[self.robot_name]['q_0']
        self.ros_pub = RosPub(self.robot_name, only_visual=True, visual_frame = "world")
        self.rate = rospy.Rate(1 / self.dt)

        # Subscribers
        self.use_ground_truth_contacts = True
        if self.use_ground_truth_contacts:
            self.sub_contact_lf = rospy.Subscriber("/" + self.robot_name + "/lf_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_lf, queue_size=1, buff_size=2**24,
                                                 tcp_nodelay=True)
            self.sub_contact_rf = rospy.Subscriber("/" + self.robot_name + "/rf_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_rf, queue_size=1, buff_size=2**24,
                                                 tcp_nodelay=True)
            self.sub_contact_lh = rospy.Subscriber("/" + self.robot_name + "/lh_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_lh, queue_size=1, buff_size=2**24,
                                                 tcp_nodelay=True)
            self.sub_contact_rh = rospy.Subscriber("/" + self.robot_name + "/rh_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_rh, queue_size=1, buff_size=2**24,
                                                 tcp_nodelay=True)

        self.sub_pose = rospy.Subscriber("/" + self.robot_name + "/ground_truth", Odometry, callback=self._receive_pose,
                                       queue_size=1, tcp_nodelay=True)
        self.sub_jstate = rospy.Subscriber("/" + self.robot_name + "/joint_states", JointState, callback=self._receive_jstate,
                                         queue_size=1, tcp_nodelay=True)


        # Publisher
        self.pub_des_jstate = rospy.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)

        # freeze base  and pause simulation service
        self.reset_world = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)

        self.pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)



        # send data to param server
        self.verbose = conf.verbose
        self.u.putIntoGlobalParamServer("verbose", self.verbose)
        self.initVars()


        self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.APPLY_EXTERNAL_WRENCH = False
        self.TIME_EXTERNAL_WRENCH = 0.6

        self.g_mag = 9.81

    def _receive_pose(self, msg):
        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w
        euler = euler_from_quaternion(self.quaternion)

        self.basePoseW[0] = msg.pose.pose.position.x
        self.basePoseW[1] = msg.pose.pose.position.y
        self.basePoseW[2] = msg.pose.pose.position.z
        self.basePoseW[3] = euler[0]
        self.basePoseW[4] = euler[1]
        self.basePoseW[5] = euler[2]

        self.baseTwistW[0] = msg.twist.twist.linear.x
        self.baseTwistW[1] = msg.twist.twist.linear.y
        self.baseTwistW[2] = msg.twist.twist.linear.z
        self.baseTwistW[3] = msg.twist.twist.angular.x
        self.baseTwistW[4] = msg.twist.twist.angular.y
        self.baseTwistW[5] = msg.twist.twist.angular.z

        # compute orientation matrix
        self.b_R_w = self.mathJet.rpyToRot(euler)

    def _receive_contact_lf(self, msg):
    # grForcesLocal_gt is used temporarely for saving force expressed in local frame
    # grForcesW_gt is computed in estimate forces and is expressed in world
        self.grForcesLocal_gt[0] =  msg.states[0].wrenches[0].force.x
        self.grForcesLocal_gt[1] =  msg.states[0].wrenches[0].force.y
        self.grForcesLocal_gt[2] =  msg.states[0].wrenches[0].force.z
    def _receive_contact_rf(self, msg):
        self.grForcesLocal_gt[3] = msg.states[0].wrenches[0].force.x
        self.grForcesLocal_gt[4] = msg.states[0].wrenches[0].force.y
        self.grForcesLocal_gt[5] = msg.states[0].wrenches[0].force.z
    def _receive_contact_lh(self, msg):
        self.grForcesLocal_gt[6] = msg.states[0].wrenches[0].force.x
        self.grForcesLocal_gt[7] = msg.states[0].wrenches[0].force.y
        self.grForcesLocal_gt[8] = msg.states[0].wrenches[0].force.z
    def _receive_contact_rh(self, msg):
        self.grForcesLocal_gt[9] = msg.states[0].wrenches[0].force.x
        self.grForcesLocal_gt[10] = msg.states[0].wrenches[0].force.y
        self.grForcesLocal_gt[11] = msg.states[0].wrenches[0].force.z

    def _send_des_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd
         self.pub_des_jstate.publish(msg)

    def applyForce(self, Fx, Fy, Fz, Mx, My, Mz, duration):
        from geometry_msgs.msg import Wrench, Point
        wrench = Wrench()
        wrench.force.x = Fx
        wrench.force.y = Fy
        wrench.force.z = Fz
        wrench.torque.x = Mx
        wrench.torque.y = My
        wrench.torque.z = Mz
        reference_frame = "world"  # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x=0, y=0, z=0)
        try:
            self.apply_body_wrench(body_name="solo::base_link", reference_frame=reference_frame,
                                   reference_point=reference_point, wrench=wrench, duration=rospy.Duration(duration))
        except:
            pass


    def send_command(self, q_des=None, qd_des=None, tau_ffwd=None):
        # q_des, qd_des, and tau_ffwd have dimension 12                                                         
        # and are ordered as on the robot                                                                       

        if (q_des is None) and (qd_des is None) and (tau_ffwd is None):
            raise RuntimeError('Cannot have both states (q_des and qd_des) and controls (tau_ffwd) as None')

        if q_des is not None:
            self.q_des = q_des
        else:
            self.q_des = np.zeros(self.robot.na)

        if qd_des is not None:
            self.qd_des = qd_des
        else:
            self.qd_des = np.zeros(self.robot.na)

        if tau_ffwd is not None:
            self.tau_ffwd = tau_ffwd
        else:
            self.tau_ffwd = np.zeros(self.robot.na)

        self._send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)

        if (self.APPLY_EXTERNAL_WRENCH and self.time > self.TIME_EXTERNAL_WRENCH):
            print("START APPLYING EXTERNAL WRENCH")
            self.applyForce(0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.05)
            self.APPLY_EXTERNAL_WRENCH = False

        # log variables
        self.rate.sleep()
        self.logData()


    def register_node(self):
        rospy.init_node('controller_python', disable_signals=False, anonymous=False)

    def deregister_node(self):
        print("deregistering nodes")
        os.system(" rosnode kill /" + self.robot_name + "/ros_impedance_controller")
        os.system(" rosnode kill /gazebo")

    def get_contact(self):
        return self.contactsW

    def get_pose(self):
        return self.basePoseW

    def get_jstate(self):
        return self.q

    def resetGravity(self, g_mag=None):
        # get actual configs
        physics_props = self.get_physics_client()

        req_reset_gravity = SetPhysicsPropertiesRequest()
        # ode config
        req_reset_gravity.time_step = physics_props.time_step
        req_reset_gravity.max_update_rate = physics_props.max_update_rate
        req_reset_gravity.ode_config = physics_props.ode_config
        req_reset_gravity.gravity = physics_props.gravity

        if not g_mag is None:
            self.g_mag = -g_mag
            self.robot.model.gravity.linear[2] = -g_mag
            req_reset_gravity.gravity.z = -g_mag
        else:
            self.robot.model.gravity.linear[2] = -9.81
            req_reset_gravity.gravity.z = -9.81
        self.set_physics_client(req_reset_gravity)

    def freezeBase(self, g_mag = None):

        self.resetGravity(g_mag)
        # create the message
        req_reset_world = SetModelStateRequest()
        # create model state
        model_state = ModelState()
        model_state.model_name = self.robot_name
        model_state.pose.position.x = 0.0
        model_state.pose.position.y = 0.0
        model_state.pose.position.z = 0.8

        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0

        model_state.twist.linear.x = 0.0
        model_state.twist.linear.y = 0.0
        model_state.twist.linear.z = 0.0

        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0
        model_state.twist.angular.z = 0.0

        req_reset_world.model_state = model_state
        # send request and get response (in this case none)
        self.reset_world(req_reset_world)

    def estimateContacts_KKT(self):
        q_ros = self.u.mapToRos(self.q)
        v_ros = self.u.mapToRos(self.qd)
        tau_ros = self.u.mapToRos(self.tau)

        B_R_W = pin.Quaternion(self.quaternion).toRotationMatrix().T

        full_conf = np.hstack((self.basePoseW[0:3], self.quaternion, q_ros))
        full_vel  = np.hstack((self.baseTwistW[0:6], v_ros))

        full_tau = np.hstack(([0.]*6, tau_ros))
        full_h = self.robot.nle(full_conf, full_vel)

        C_qd = full_h - self.robot.gravity(full_conf)

        self.tau_minus_h = full_tau - full_h
        self.C_qd = C_qd
        bias = np.hstack((self.tau_minus_h, [0., 0., 0.] * 4))

        dJdq = np.zeros(12)
        for leg in range(0, 4):
            dJdq[3*leg:3*(leg+1)] = self.robot.dJdq(full_conf, full_vel, self.robot.model.getFrameId(self.ee_frames[leg]), 'linear')
        bias[self.robot.nv:] = self.u.mapToRos(dJdq)

        KKT_inv = self.robot.KKTMatrixAtEndEffectorsInv(full_conf, 'linear')
        tmp = KKT_inv @ bias
        full_a = tmp[0:self.robot.nv]
        full_grfW = -tmp[self.robot.nv:]

        self.grForcesW = self.u.mapToRos(full_grfW)
        self.base_acc_W = full_a[:6]
        self.qdd = self.u.mapToRos(full_a[6:])

        for leg in range(4):

            self.grForcesB[3*leg:3*(leg+1)] = B_R_W @ self.grForcesW[3*leg:3*(leg+1)]

            self.contacts_state[leg] = self.grForcesW[3*(leg+1)-1] > self.force_th


    def estimateContacts(self):
        q_ros = self.u.mapToRos(self.q)
        v_ros = self.u.mapToRos(self.qd)

        # Pinocchio Update the joint and frame placements
        configuration = np.hstack(([0,0,0], self.quaternion, q_ros))
        neutral_configuration = np.hstack((pin.neutral(self.robot.model)[0:7], q_ros))
        velocity = np.hstack(([0,0,0], self.baseTwistW[3:6], v_ros))

        pin.forwardKinematics(self.robot.model, self.robot.data, neutral_configuration)
        pin.computeJointJacobians(self.robot.model, self.robot.data)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        #gravity_torques = self.robot.gravity(configuration)
        gravity_torques = self.robot.nle(configuration, velocity)
        joint_gravity_torques = gravity_torques[6:]

        self.contacts_state[:] = False


        self.robot.forwardKinematics(neutral_configuration)
        # leg index represents lf rf lh rh
        for leg in range(4):
            index = self.robot.model.getFrameId(self.ee_frames[leg])
            self.B_contacts[leg] = pin.updateFramePlacement(self.robot.model, self.robot.data, index).translation
            leg_joints = range(6 + self.u.mapIndexToRos(leg) * 3, 6 + self.u.mapIndexToRos(leg) * 3 + 3)
            self.bJ[leg] = self.robot.frameJacobian(neutral_configuration,
                                                       self.robot.model.getFrameId(self.ee_frames[leg]),
                                                       pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, leg_joints]
            grf = np.linalg.inv(self.bJ[leg].T).dot(self.u.getLegJointState(leg, joint_gravity_torques - self.tau))

            R = pin.rpy.rpyToMatrix(self.basePoseW[3:6])
            self.u.setLegJointState(leg, grf, self.grForcesB)
            self.u.setLegJointState(leg, R @ grf, self.grForcesW)
            self.contacts_state[leg] = grf[2] > self.force_th

        full_configuration = np.hstack((self.u.linPart(self.basePoseW), self.quaternion, self.u.mapToRos(self.q)))
        self.robot.forwardKinematics(full_configuration) 
        for leg in range(4):
            foot_index = self.robot.model.getFrameId(self.ee_frames[leg])
            lower_leg_index =  self.robot.model.getFrameId(self.lower_leg_frames[leg])

            self.W_contacts[leg] = pin.updateFramePlacement(self.robot.model, self.robot.data, foot_index).translation

            if self.use_ground_truth_contacts:
                grf_gt = pin.updateFramePlacement(self.robot.model, self.robot.data, lower_leg_index).rotation.dot(self.grForcesLocal_gt[3*leg:3*(leg+1)])
                self.u.setLegJointState(leg, grf_gt, self.grForcesW_gt)




    def visualizeContacts(self):
        for leg in range(4):
            self.ros_pub.add_arrow(self.W_contacts[leg],
                                   self.u.getLegJointState(leg, self.grForcesB/ (6*self.robot.robot_mass )),
                                   "green")
            if (self.use_ground_truth_contacts):
                self.ros_pub.add_arrow(self.W_contacts[leg],
                                    self.u.getLegJointState(leg, self.grForcesW_gt / (6*self.robot.robot_mass)),
                                    "red")
            if self.contacts_state[leg]:
                self.ros_pub.add_marker(self.W_contacts[leg], radius=0.1)
            else:
                self.ros_pub.add_marker(self.W_contacts[leg], radius=0.001)
        self.ros_pub.publishVisual()



    # def estimateContacts(self):
    #     q_ros = self.u.mapToRos(self.q)
    #     qd_ros = self.u.mapToRos(self.qd)
    #     tau_ros = self.u.mapToRos(self.tau)
    #     # Pinocchio Update the joint and frame placements
    #     configuration = np.hstack((self.basePoseW[0:3], self.quaternion, q_ros))
    #     gen_velocities = np.hstack((self.baseTwistW, qd_ros))
    # 
    #     self.h = pin.nonLinearEffects(self.robot.model, self.robot.data, configuration, gen_velocities)
    #     self.h_joints = self.h[6:]
    # 
    #     # estimate ground reaction forces from tau
    #     for leg in range(4):
    #         grf = np.linalg.inv(self.wJ[leg].T).dot(
    #             self.u.getLegJointState(leg, self.u.mapFromRos(self.h_joints) - self.tau))
    #         self.u.setLegJointState(leg, grf, self.grForcesW)
    #         self.contacts_state[leg] = grf[2] > self.force_th

    # def visualizeContacts(self):
    #     for leg in range(4):
    #         self.ros_pub.add_arrow(self.W_contacts[leg],
    #                                self.u.getLegJointState(leg, self.grForcesW / (6 * self.robot.robot_mass)), "green")
    #         if self.contacts_state[leg]:
    #             ros_pub.add_marker(self.W_contacts, radius=0.1)
    #         else:
    #             ros_pub.add_marker(self.W_contacts, radius=0.001)
    #     self.ros_pub.publishVisual()

    def computeCom(self): #deprecated
        q_ros = self.u.mapToRos(self.q)
        qd_ros = self.u.mapToRos(self.qd)
        configuration = np.hstack((self.basePoseW[0:3], self.quaternion, q_ros))
        gen_velocities = np.hstack((self.baseTwistW, qd_ros))

        # compute com
        self.comPosW, self.comVelW = self.robot.robotComW(configuration, gen_velocities)
        self.comPosB, self.comVelB = self.robot.robotComW(configuration, gen_velocities)

    def computeInertia(self):
        q_ros = self.u.mapToRos(self.q)
        qd_ros = self.u.mapToRos(self.qd)
        configuration = np.hstack((self.basePoseW[0:3], self.quaternion, q_ros))
        gen_velocities = np.hstack((self.baseTwistW, qd_ros))

        # robot inertias
        self.centroidalInertiaB = self.robot.centroidalInertiaB(configuration, gen_velocities)
        self.compositeRobotInertiaB = self.robot.compositeRobotInertiaB(configuration)

    def startupProcedure(self):
        rospy.sleep(0.5)  # wait for callback to fill in jointmnames

        self.pid = PidManager(self.joint_names)
        self.pid.setPDs(conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], 0.0)

        if (self.robot_name == 'hyq'):
            # these torques are to compensate the leg gravity
            self.gravity_comp = np.array(
                [24.2571, 1.92, 50.5, 24.2, 1.92, 50.5739, 21.3801, -2.08377, -44.9598, 21.3858, -2.08365, -44.9615])

            print("reset posture...")
            self.freezeBase(1)
            start_t = rospy.get_time()
            while rospy.get_time() - start_t < 1.0:
                self._send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                rospy.sleep(0.01)
            if self.verbose:
                print("q err prima freeze base", (self.q - self.q_des))

            print("put on ground and start compensating gravity...")
            self.freezeBase(0)
            rospy.sleep(0.5)
            if self.verbose:
                print("q err pre grav comp", (self.q - self.q_des))

            start_t = rospy.get_time()
            while rospy.get_time() - start_t < 1.0:
                self._send_des_jstate(self.q_des, self.qd_des, self.gravity_comp)
                rospy.sleep(0.01)
            if self.verbose:
                print("q err post grav comp", (self.q - self.q_des))

            print("starting com controller (no joint PD)...")
            self.pid.setPDs(0.0, 0.0, 0.0)

        if (self.robot_name == 'solo'):
            start_t = rospy.get_time()
            # fb_conf = np.hstack([pin.neutral(self.robot.model)[:7], self.q_des])
            # self.gravity_comp = -self.robot.gravity(fb_conf)[6:]

            self.q_des = self.robot.q_home
            self.qd_des = np.zeros(self.robot.na)
            # self.tau_ffwd = self.gravity_comp

            while rospy.get_time() - start_t < 0.3:
                fb_conf = np.hstack([pin.neutral(self.robot.model)[:7], self.q])
                self.tau_ffwd = self.gravityCompensation()+self.self_weightCompensation()
                self._send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                rospy.sleep(0.01)
        #self.pid.setPDs(0.0, 0.0, 0.0)
        print("finished startup")

    def save_reference(self, filename):
        DATA = {}
        DATA['last_sample'] = self.log_counter
        DATA['q_des'] = self.q_des_log[:, :self.log_counter]
        DATA['qd_des'] = self.qd_des_log[:, :self.log_counter]
        DATA['tau_ffwd'] = self.tau_ffwd_log[:, :self.log_counter]

        savemat(filename, DATA)

        print('Reference saved in', filename)




if __name__ == '__main__':
    p = SimController('solo')

    p.startupProcedure()
    q_des = p.robot.q_home
    qd_des = np.zeros_like(q_des)
    try:

        while not rospy.is_shutdown():

            p.estimateContacts()
            p.visualizeContacts()
            tau_ffwd = p.gravityCompensation()+p.self_weightCompensation()
            p.send_command(q_des, qd_des, tau_ffwd)



    except (rospy.ROSInterruptException, rospy.service.ServiceException):
        rospy.signal_shutdown("killed")
        p.deregister_node()
        # stats = pstats.Stats(profiler).sort_stats('cumtime')
        # stats.print_stats()

