import rospy as ros
from base_controllers.utils.pid_tuner import PIDTuningGui
import threading
import numpy as np
from termcolor import colored
from base_controllers.utils.math_tools import Math
from base_controllers.utils.utils import Utils
import pinocchio as pin
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
import os
from gazebo_ros import gazebo_interface

class QuadrupedTasks:
    def __init__(self, task='pushup', robot_conf = None, gui = True, quadruped = None):
        """Simple timer utility similar to force_timer_ in your C++ code."""
        self.DEBUG = task  # 'none', 'pushup','swim','step'
        self.debug_gui = gui
        self.quadruped = quadruped
        self.robot_conf = robot_conf
        #init for step task
        self.switch_on = False
        self.t0 = None
        self.q_retraction = robot_conf['q_retraction']
        self.q_final = robot_conf['q_final']
        self.transition_time = 0.1

        self.pid_tuning_gui = PIDTuningGui(self.quadruped, mode=self.DEBUG, tuning_type='PID', init_freq=0.5)



        if self.debug_gui and self.DEBUG != 'none':
            self.quadruped.stop_thread = False
            self.thread_pid = threading.Thread(target=self.pid_tuning_gui.init_pid_tuning_ui)
            self.thread_pid.daemon = True
            self.thread_pid.start()

        self.math_utils = Math()

        self.u = Utils()

        self.phi = 0.

    def computeJcb(self, feetW, com, stance_legs):
        Jb = np.zeros([3 * self.quadruped.robot.nee, 6])  # Newton-Euler matrix
        for leg in range(self.quadruped.robot.nee):
            start_row = 3 * leg
            end_row = 3 * (leg + 1)
            if stance_legs[leg]:
                # ---> linear part
                # identity matrix (I avoid to rewrite zeros)
                Jb[start_row:end_row, :3] = np.identity(3)
                # ---> angular part
                # all in a function
                Jb[start_row:end_row, 3:] = -pin.skew(feetW[leg] - com)
            else:
                Jb[start_row:end_row, 3:] = np.zeros(3)
                Jb[start_row:end_row, :3] = np.zeros(3)
        return Jb


    def resetRobot(self, basePoseDes=np.array([0, 0, 0.3, 0., 0., 0.])):
        # this sets the position of the joints
        gazebo_interface.set_model_configuration_client(self.quadruped.robot_name, '', self.quadruped.joint_names, self.quadruped.qj_0, '/gazebo')
        self.quadruped.send_des_jstate(self.quadruped.q_des, self.quadruped.qd_des, self.quadruped.tau_ffwd)
        # this sets the position of the base
        self.quadruped.freezeBase(True,  basePoseW=basePoseDes)

    def cerp(self, start, end, weight, start_tangent: float = 1e-3, end_tangent: float = 1e-3):
        # Hermite basis functions
        h00 = (2 * weight**3) - (3 * weight**2) + 1
        h10 = weight**3 - 2 * weight**2 + weight
        h01 = (-2 * weight**3) + (3 * weight**2)
        h11 = weight**3 - weight**2

        # Interpolation
        return (h00 * start) + (h10 * start_tangent) + (h01 * end) + (h11 * end_tangent)

    def stop(self):
        print("Stopping Tuning GUI...")

        # stop the GUI thread
        self.quadruped.stop_thread = True

        # join it safely
        if hasattr(self, "thread_pid") and self.thread_pid.is_alive():
            self.thread_pid.join(timeout=1.0)

        print("Tuning GUI stopped.")

    def startUp(self, time = 0.):
        self.start_time = time

        self.initial_com = np.copy(self.quadruped.basePoseW[:3].copy())
        print(colored(f"IMPORTANT: you cannot control both pitch and Z and expect 0 error on comX, only on base, because it is an impossible task!", "red"))

        self.W_feetRelPosDes = np.copy(self.quadruped.W_contacts - self.initial_com)
        self.W_contacts_sampled = np.copy(self.quadruped.W_contacts)
        self.des_robot = RobotWrapper.BuildFromURDF(os.environ.get('LOCOSIM_DIR') + "/robot_urdf/generated_urdf/" + self.quadruped.robot_name + ".urdf",
                                                    root_joint=pin.JointModelFreeFlyer())


        if self.DEBUG == 'step' or self.DEBUG == 'swim':
            if self.quadruped.real_robot:
                print(colored('!!!!!!!!!!!!!! PULL UP THE ROBOT !!!!!!!!!!!!!!', 'red'))
                ros.sleep(1.)  # wait for user tu pull up the robot
            print(colored(f'STARTING {self.DEBUG} MODE', 'red'))
            self.quadruped.pid.setPDjoints(self.robot_conf['kp_real_swing'],
                                 self.robot_conf['kd_real_swing'],
                                 self.robot_conf['ki_real_swing'])
        if self.DEBUG == 'step' or self.DEBUG == 'swim':
            self.resetRobot(basePoseDes=np.array([0, 0, self.robot_conf['spawn_z'], 0., 0., 0.]))

        # DEFINE A ros SHUTDOWN HOOK
        ros.on_shutdown(self.stop)


    def generateReference(self, time):
        t_ = time - self.start_time

        # generate a reference sin trajectory for COM
        freq = self.pid_tuning_gui.debug_freq
        self.phi += 2*np.pi*self.pid_tuning_gui.debug_freq*self.robot_conf['dt']

        amp_lin = np.array([0., 0., 0.05])
        amp_ang = np.array([0., 0.1, 0])

        com = self.initial_com + np.multiply(amp_lin, np.sin(self.phi))
        comd = np.multiply(2*np.pi*freq*amp_lin,  np.cos(self.phi))
        comdd = np.multiply(np.power(2*np.pi*freq*amp_lin, 2), -np.sin(self.phi))

        eul = np.array([0., 0.0, 0]) + np.multiply(amp_ang, np.sin(self.phi))
        euld = np.multiply(2 * np.pi * freq * amp_ang, np.cos(self.phi))
        euldd = np.multiply(np.power(2 * np.pi * freq * amp_ang, 2), -np.sin(self.phi))


        #######################
        Jb = self.computeJcb(self.W_contacts_sampled, com, self.quadruped.stance_legs)

        W_des_basePose = np.empty(6)
        W_des_basePose[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = com
        W_des_basePose[self.u.sp_crd['AX']:self.u.sp_crd['AX'] + 3] = eul

        W_des_baseTwist = np.empty(6)
        W_des_baseTwist[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = comd
        Jomega = self.math_utils.Tomega(eul)
        W_des_baseTwist[self.u.sp_crd['AX']:self.u.sp_crd['AX'] +
                                            3] = self.math_utils.Tomega(eul).dot(euld)

        W_des_baseAcc = np.empty(6)
        W_des_baseAcc[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = comdd
        # compute w_omega_dot =  Jomega* euler_rates_dot + Jomega_dot*euler_rates (Jomega already computed, see above)
        Jomega_dot = self.math_utils.Tomega_dot(eul, euld)
        W_des_baseAcc[self.u.sp_crd['AX']:self.u.sp_crd['AX'] + 3] = Jomega @ euldd + Jomega_dot @ euld

        # map base twist into feet relative vel (wrt com/base)
        W_feetRelVelDes = -Jb.dot(W_des_baseTwist)
        w_R_b_des = self.math_utils.eul2Rot(eul)

        grf_ffwd = np.zeros(12)
        tau_ffwd = np.zeros(12)
        qd_des = np.zeros(12)
        q_des = np.zeros(12)
        fbjoints = pin.neutral(self.quadruped.robot.model)
        w_J = self.u.listOfArrays(4, np.zeros((3, 3)))

        # integrate relative Velocity
        for leg in range(self.quadruped.robot.nee):
            if self.DEBUG != 'step':
                # with this you do not have proper tracking of com and trunk orientation, I think there is a bug in the ik
                # self.W_feetRelPosDes[leg] += W_feetRelVelDes[3 * leg:3 * (leg+1)]*self.dt
                # this has better tracking
                # should use desired values to generate traj otherwise if it is unstable it detroys the ref signal
                self.W_feetRelPosDes[leg] = self.W_contacts_sampled[leg] - com

                q_des[3 * leg:3 * (leg + 1)], isFeasible = self.quadruped.IK.ik_leg(w_R_b_des.T.dot(self.W_feetRelPosDes[leg]),
                                                                          self.quadruped.leg_names[leg],
                                                                          self.quadruped.legConfig[self.quadruped.leg_names[leg]][0],
                                                                          self.quadruped.legConfig[self.quadruped.leg_names[leg]][1])
                # for joint velocity we need to recompute the Jacobian (in the world frame) for the computed joint position q_des
                # you need to fill in also the floating base part
                quat_des = pin.Quaternion(w_R_b_des)
                fbjoints[:3] = com
                fbjoints[3:7] = np.array([quat_des.x, quat_des.y, quat_des.z, quat_des.w])
                fbjoints[7:] = q_des

                pin.forwardKinematics(self.des_robot.model, self.des_robot.data, fbjoints, np.zeros(
                    self.des_robot.model.nv), np.zeros(self.des_robot.model.nv))
                pin.computeJointJacobians(
                    self.des_robot.model, self.des_robot.data)
                pin.computeFrameJacobian(self.des_robot.model, self.des_robot.data,
                                         fbjoints, self.des_robot.model.getFrameId(self.quadruped.ee_frames[leg]))
                w_J[leg] = pin.getFrameJacobian(self.des_robot.model, self.des_robot.data,
                                                self.des_robot.model.getFrameId(
                                                    self.quadruped.ee_frames[leg]),
                                                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 6 + leg * 3:6 + leg * 3 + 3]
                # compute joint variables
                qd_des[3 * leg:3 * (leg + 1)] = np.linalg.pinv(w_J[leg]).dot(W_feetRelVelDes[3 * leg:3 * (leg + 1)])

            if self.DEBUG != 'swim' and self.DEBUG != 'step':
                tau_ffwd, self.grForcesW_des = self.quadruped.wbc.computeWBC(self.quadruped.W_contacts, self.quadruped.wJ, self.quadruped.h_joints, self.quadruped.basePoseW, self.quadruped.comPoseW, self.quadruped.baseTwistW, self.quadruped.comTwistW,
                                                                   W_des_basePose, W_des_baseTwist, W_des_baseAcc, self.quadruped.centroidalInertiaB,
                                                                   comControlled=False, type='projection', stance_legs=self.quadruped.stance_legs)
            else:
                tau_ffwd = np.zeros(12)

            if self.DEBUG == 'step':
                # self.qj_switch  = self.q_0_lo
                self.qj_switch = self.q_retraction
                switching_signal = 0.5 * (1. + np.sin(2*np.pi*self.pid_tuning_gui.debug_freq * t_))
                if (switching_signal > 0.75) and not self.switch_on:
                    print(colored("SWITCH ON", "red"))
                    self.q_1 = self.quadruped.q_des.copy()
                    self.q_2 = self.qj_switch.copy()
                    self.switch_on = True
                    self.t0 = t_
                if (switching_signal < 0.25) and self.switch_on:
                    print(colored("SWITCH OFF", "red"))
                    self.q_1 = self.qj_switch.copy()
                    self.q_2 = self.quadruped.qj_0.copy()
                    self.switch_on = False
                    self.t0 = t_
                if self.t0 is not None:
                    elapsed_time = t_ - self.t0
                    elapsed_ratio = np.clip(elapsed_time / self.transition_time, 0, 1)
                    q_des = self.cerp(self.q_1, self.q_2, elapsed_ratio).copy()
                else: #first time
                    q_des = self.quadruped.qj_0
                    # tau_ffwd = self.quadruped.h_joints #rovina il tracking del KFE

        return q_des, qd_des, tau_ffwd, W_des_basePose, W_des_baseTwist
