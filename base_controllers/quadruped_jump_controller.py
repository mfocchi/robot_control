import sys
import os


dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path+"/../")
sys.path.append(dir_path+"/../../")
sys.path.append(dir_path+"/../../../")


from jump_policy.jump_agent import JumpAgent
from utils.pid_tuner import PIDTuningGui
from base_controllers.quadruped_controller import QuadrupedController
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
from base_controllers.utils.common_functions import *
from landing_controller.controller.landingManager import LandingManager
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelConfigurationRequest
from gazebo_msgs.srv import SetModelConfiguration
from base_controllers.utils.pidManager import PidManager
from gazebo_ros import gazebo_interface
from termcolor import colored
from base_controllers.utils.math_tools import *
import base_controllers.params as conf
import pinocchio as pin
import numpy as np
import rospy as ros
import scipy.io.matlab as mio
import threading


# utility functions

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)
np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)


class QuadrupedJumpController(QuadrupedController):
    def __init__(self, robot_name="go1", launch_file=None):
        super(QuadrupedJumpController, self).__init__(robot_name, launch_file)
        self.use_gui = False
        self.DEBUG = 'none' # 'none', 'pushup','swim','step'
        self.debug_gui = True
        self.jumpAgent = JumpAgent(self.robot_name)
        self.go0_conf = 'standDown'
        # self.q_0_td = conf.robot_params[self.robot_name]['q_0_td']
        # self.q_0_lo = conf.robot_params[self.robot_name]['q_0_lo']
        self.q_retraction = conf.robot_params[self.robot_name]['q_retraction']
        self.q_land = conf.robot_params[self.robot_name]['q_land']
        self.q_final = conf.robot_params[self.robot_name]['q_final']
        self.use_landing_controller = False

        if self.real_robot:
            print(colored(
                "Real robot TRUE: you should launch your lab alias with xhost +; lab -u root"))

        user = os.popen('whoami').read()

        #if user=='root':
        pid = os.getpid()
        print(colored("USER IS ROOT: USING RNICE FOR THREAD PRIORITY","red"))
        # NOTE: use chrt -r 99 command for rt
        # need to launch docker as root
        os.system(f'sudo renice -n -21 -p {str(pid)}')
        # need to launch docker as root
        os.system(f'sudo echo -20 > /proc/{str(pid)}/autogroup')

        print("Initialized Quadruped Jump controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        self.intermediate_com_position = []
        self.intermediate_flight_com_position = []
        self.firstTime = True
        self.detectedApexFlag = False
        self.rearingFlag = False
        self.ideal_landing = np.zeros(3)
        self.landing_position = np.zeros(3)
        self.landing_orientation = np.zeros(3)
        self.landing_error = 0.
        self.orient_error = 0.
        self.touchdown_detected = False
        # for debug
        self.switch_on = False
        self.t0 = None

        self.reset_joints = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.set_state = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        if not self.real_robot:
            # if you spawn it starts to publish gt in wf rather than in lowerleg frame
            self.publish_contact_gt_in_wf = True
            spawnModel('go1_description', 'jump_platform')

        # instance of pid gui for tuning
        self.pid_tuning_gui = PIDTuningGui(self, mode=self.DEBUG, init_freq=0.5, real_robot_=self.real_robot)

        if self.debug_gui and self.DEBUG != 'none':
            self.stop_thread = False
            self.thread_pid = threading.Thread(target=self.pid_tuning_gui.init_pid_tuning_ui)
            self.thread_pid.daemon = True
            self.thread_pid.start()

    def detectApex(self, threshold=-3):
        # foot tradius is 0.015
        foot_lifted_off = np.array([False, False, False, False])
        for leg in range(4):
            foot_lifted_off[leg] = self.W_contacts[leg][2] > 0.017
        # if not self.detectedApexFlag and np.all(foot_lifted_off):

        if not self.detectedApexFlag and self.time >= p.startTrust + p.T_th_total + p.T_apex:
            # Try tp use only the linear acceleration
            if not self.detectedApexFlag:
                print(colored(f"APEX detected at t={self.time} setting new treshold", "red"))
                self.q_apex = self.q_des.copy()
                self.qd_apex = self.qd_des.copy()
                self.t_apex = self.time
                self.detectedApexFlag = True
                # if self.real_robot:
                #     #if self.baseLinAccW[2] < threshold:
                #     self.detectedApexFlag = True
                #     print(colored(f"APEX detected at t={self.time} setting new treshold", "red"))
                #     self.q_apex = self.q_des.copy()
                #     self.qd_apex = self.qd_des.copy()
                #     self.t_apex = self.time
                # else:
                #     if self.baseTwistW[2] < 0.0:
                #         self.detectedApexFlag = True
                #         # move floating base for landing
                #         # if not self.real_robot:
                #         #     self.pause_physics_client()
                #         #     for i in range(10):
                #         #         self.setJumpPlatformPosition(
                #         #             self.target_position, com_0)
                #         #     self.unpause_physics_client()
                #         print(colored(f"APEX detected at t={self.time}", "red"))
                #         self.q_apex = self.q_des.copy()
                #         self.qd_apex = self.qd_des.copy()
                #         self.t_apex = self.time

    def detectTouchDown(self):
        # if np.all(self.contact_state):
        #     return True
        # else:
        #     return False
        return self.time >= p.startTrust + p.T_th_total + p.T_fl
        # return False

    def resetRobot(self, basePoseDes=np.array([0, 0, 0.3, 0., 0., 0.])):
        # this sets the position of the joints
        gazebo_interface.set_model_configuration_client(self.robot_name, '', self.joint_names, self.qj_0, '/gazebo')
        self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
        # this sets the position of the base
        if self.DEBUG == 'none' or self.DEBUG == 'pushup':
            self.freezeBase(False,  basePoseW=basePoseDes)
        else:
            self.freezeBase(True,  basePoseW=basePoseDes)


    def bernstein_pol(self, k, n, x):
        v = (np.math.factorial(n)/(np.math.factorial(k) *(np.math.factorial(n-k))))*np.power(x, k)*np.power(1-x, n-k)
        return v

    def Bezier3(self, w, t_, t_tot):
        t = t_/t_tot
        if w.ndim == 1:
            return  w[0] * self.bernstein_pol(0, 3, t) + \
                    w[1] * self.bernstein_pol(1, 3, t) + \
                    w[2] * self.bernstein_pol(2, 3, t) + \
                    w[3] * self.bernstein_pol(3, 3, t)
        else:
            return  w[:, 0]*self.bernstein_pol(0, 3, t) +\
                    w[:, 1]*self.bernstein_pol(1, 3, t) +\
                    w[:, 2]*self.bernstein_pol(2, 3, t) +\
                    w[:, 3]*self.bernstein_pol(3, 3, t)

    def Bezier2(self, w, t_, t_tot):
        t = t_/t_tot
        return  ((w[:, 1]-w[:, 0]))*(3/t_tot)*self.bernstein_pol(0, 2, t) +\
                ((w[:, 2]-w[:, 1]))*(3/t_tot)*self.bernstein_pol(1, 2, t) +\
                ((w[:, 3]-w[:, 2]))*(3/t_tot)*self.bernstein_pol(2, 2, t)

    def Bezier1(self, w, t_, t_tot):
        t = t_ / t_tot
        return  ((w[:, 2] - 2 * w[:, 2] + w[:, 0])) * (6 / np.power(t_tot, 2)) * self.bernstein_pol(0, 1, t) + \
                ((w[:, 3] - 2 * w[:, 2] + w[:, 1])) * \
                (6 / np.power(t_tot, 2)) * self.bernstein_pol(1, 1, t)

    def computeHeuristicSolutionBezierLinear(self, com_0, com_lo_b, comd_lo_b, T_th):
        self.bezier_weights_lin = np.zeros([3, 4])
        comd_0 = np.zeros(3)
        self.bezier_weights_lin = np.zeros([3, 4])
        self.bezier_weights_lin[:, 0] = com_0
        self.bezier_weights_lin[:, 1] = (comd_0*(T_th/3.)) + com_0
        self.bezier_weights_lin[:, 2] = com_lo_b - (comd_lo_b*(T_th/3.))
        self.bezier_weights_lin[:, 3] = com_lo_b

    def computeHeuristicSolutionBezierAngular(self, eul_0, eul_lo, euld_lo, T_th):
        self.bezier_weights_ang = np.zeros([3, 4])
        euld_0 = np.zeros(3)
        self.bezier_weights_ang = np.zeros([3, 4])
        self.bezier_weights_ang[:, 0] = eul_0
        self.bezier_weights_ang[:, 1] = (euld_0*(T_th/3.)) + eul_0
        self.bezier_weights_ang[:, 2] = eul_lo - (euld_lo*(T_th/3.))
        self.bezier_weights_ang[:, 3] = eul_lo

    def computeTrajectoryBezier(self, T_th_b, com_lo_e, n_blobs=10):
        self.number_of_blobs = n_blobs

        t = np.linspace(0, T_th_b, self.number_of_blobs-1)
        for t_ in t:
            self.intermediate_com_position.append(self.Bezier3(self.bezier_weights_lin, t_, T_th_b))

        self.intermediate_com_position.append(com_lo_e)

        print(com_lo_e)

    def plotTrajectoryBezier(self):
        # plot com intermediate positions
        for blob in self.intermediate_com_position:
            self.ros_pub.add_marker(blob, color=[0.5, 0.5, 0.5], radius=0.02)
    

    def computeJcb(self, feetW, com, stance_legs):
        Jb = np.zeros([3 * self.robot.nee, 6])  # Newton-Euler matrix
        for leg in range(self.robot.nee):
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

    def evalThrust(self, t_, T_th_b, T_th_e, com_lo_b, comd_lo_b, com_lo_e, comd_lo_e):

        # compute total thrust time (Bezier + Explosive)
        T_th_total = T_th_b + T_th_e

        if t_ < T_th_b:
            # Bezier from 0 to T_th
            com = np.array(self.Bezier3(self.bezier_weights_lin, t_, T_th_b))
            comd = np.array(self.Bezier2(self.bezier_weights_lin, t_, T_th_b))
            comdd = np.array(self.Bezier1(self.bezier_weights_lin, t_, T_th_b))
        else:
            # Explosive from T_th to T_th_total
            # use euler integration
            t = t_ - T_th_b  # normalize time for explosive part
            t_0 = (t-self.dt) / T_th_e
            t_1 = np.clip(t / T_th_e, 0, 1)

            com = self.lerp(com_lo_b, com_lo_e, t_1)
            comd = self.lerp(comd_lo_b, comd_lo_e, t_1)
            comd_ = self.lerp(comd_lo_b, comd_lo_e, t_0)
            comdd = (comd-comd_)/self.dt

        # angle part is computed only 
        eul = np.array(self.Bezier3(self.bezier_weights_ang, t_, T_th_total))
        euld = np.array(self.Bezier2(self.bezier_weights_ang, t_, T_th_total))
        euldd = np.array(self.Bezier1(self.bezier_weights_ang, t_, T_th_total))

        if self.DEBUG=='pushup' or self.DEBUG=='swim':
            # generate a sin trajectory
            freq = self.pid_tuning_gui.debug_freq
            amp_lin = np.array([0., 0., 0.05])
            amp_ang = np.array([0., 0.1, 0])

            com = self.initial_com + np.multiply(amp_lin, np.sin(2*np.pi*freq * t_))
            comd = np.multiply(2*np.pi*freq*amp_lin,  np.cos(2*np.pi*freq * t_))
            comdd = np.multiply(np.power(2*np.pi*freq*amp_lin, 2), -np.sin(2*np.pi*freq * t_))

            eul = np.array([0., 0.0, 0]) + np.multiply(amp_ang, np.sin(2 * np.pi * freq * t_))
            euld = np.multiply(2 * np.pi * freq * amp_ang, np.cos(2 * np.pi * freq * t_))
            euldd = np.multiply(np.power(2 * np.pi * freq * amp_ang, 2), -np.sin(2 * np.pi * freq * t_))

        Jb = p.computeJcb(self.W_contacts_sampled, com, self.stance_legs)

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
        fbjoints = pin.neutral(self.robot.model)
        w_J = self.u.listOfArrays(4, np.zeros((3, 3)))
        # integrate relative Velocity

        for leg in range(self.robot.nee):
            if self.DEBUG != 'step':
                # with this you do not have proper tracking of com and trunk orientation, I think there is a bug in the ik
                # self.W_feetRelPosDes[leg] += W_feetRelVelDes[3 * leg:3 * (leg+1)]*self.dt
                # this has better tracking
                # should use desired values to generate traj otherwise if it is unstable it detroys the ref signal
                self.W_feetRelPosDes[leg] = self.W_contacts_sampled[leg] - com

                q_des[3 * leg:3 * (leg+1)], isFeasible = self.IK.ik_leg(w_R_b_des.T.dot(self.W_feetRelPosDes[leg]),
                                                                        self.leg_names[leg],
                                                                        self.legConfig[self.leg_names[leg]][0],
                                                                        self.legConfig[self.leg_names[leg]][1])
                # for joint velocity we need to recompute the Jacobian (in the world frame) for the computed joint position q_des
                # you need to fill in also the floating base part
                quat_des = pin.Quaternion(w_R_b_des)
                fbjoints[:3] = com
                fbjoints[3:7] = np.array([quat_des.x, quat_des.y, quat_des.z, quat_des.w])
                fbjoints[7:] = q_des

                pin.forwardKinematics(self.des_robot.model, self.des_robot.data, fbjoints, np.zeros(
                    self.des_robot.model.nv),   np.zeros(self.des_robot.model.nv))
                pin.computeJointJacobians(
                    self.des_robot.model, self.des_robot.data)
                pin.computeFrameJacobian(self.des_robot.model, self.des_robot.data,
                                        fbjoints, p.des_robot.model.getFrameId(self.ee_frames[leg]))
                w_J[leg] = pin.getFrameJacobian(self.des_robot.model, self.des_robot.data,
                                                p.des_robot.model.getFrameId(
                                                    self.ee_frames[leg]),
                                                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 6 + leg * 3:6 + leg * 3 + 3]
                # compute joint variables
                qd_des[3 * leg:3 * (leg+1)] = np.linalg.pinv(w_J[leg]).dot(W_feetRelVelDes[3 * leg:3 * (leg+1)])
                
            if self.DEBUG!='swim' and self.DEBUG!='step':
                tau_ffwd, self.grForcesW_wbc = self.wbc.computeWBC(self.W_contacts, self.wJ, self.h_joints,  self.basePoseW, self.comPoseW, self.baseTwistW, self.comTwistW,
                                                            W_des_basePose, W_des_baseTwist, W_des_baseAcc, self.centroidalInertiaB,
                                                            comControlled=False, type='projection', stance_legs=self.stance_legs)
            # OLD
            # tau_ffwd, self.grForcesW_wbc = self.wbc.gravityCompensationBase(self.B_contacts, self.wJ, self.h_joints,  self.basePoseW)
            
            else:
                tau_ffwd = np.zeros(12)
            
            if self.DEBUG=='step':
                p.qj_switch  = p.q_0_lo
                switching_signal =  0.5*(1. + np.sin(2*np.pi*self.pid_tuning_gui.debug_freq * t_) )   
                if (switching_signal  > 0.75) and not p.switch_on:
                    print(colored("SWITCH ON","red"))
                    p.q_1 = p.q_des.copy()
                    p.q_2 = p.qj_switch.copy()            
                    p.switch_on = True
                    p.t0 = t_
                if  (switching_signal < 0.25)  and p.switch_on:
                    print(colored("SWITCH OFF","red"))
                    p.q_1 = p.qj_switch.copy()
                    p.q_2 = p.qj_0.copy()
                    p.switch_on = False
                    p.t0 = t_
                if p.t0 is not None:
                    elapsed_time = t_-p.t0
                    elapsed_ratio = np.clip(elapsed_time / p.lerp_time, 0, 1)
                    q_des = p.cerp(p.q_1, p.q_2,  elapsed_ratio).copy()
                    qd_des = np.zeros(12) #p.lerp(p.q_t_th, np.zeros_like(p.q_t_th), elapsed_ratio).copy()
                    #tau_ffwd = p.h_joints #rovina il tracking del KFE
        
        return q_des, qd_des, tau_ffwd, W_des_basePose, W_des_baseTwist

    def computeIdealLanding(self, com_lo, comd_lo, target_position):
        # get time of flight
        arg = comd_lo[2] * comd_lo[2] - 2 * 9.81 * (target_position[2] - com_lo[2])
        if arg < 0:
            print(colored("Point Beyond Reach, tagret too high", "red"))
            return False
        else:  # beyond apex
            self.T_apex = (comd_lo[2]) / 9.81  # we take the highest value
            self.T_fl = (comd_lo[2] + math.sqrt(arg)) / 9.81  # we take the highest value
            self.ideal_landing = np.hstack((com_lo[:2] + self.T_fl * comd_lo[:2], target_position[2]))
            print( 'ta',p.startTrust + p.T_th_total + p.T_apex)
            print( 'td',p.startTrust + p.T_th_total + p.T_fl)
            # t = np.linspace(0, self.T_fl, self.number_of_blobs)
            # com = np.zeros(3)
            # for blob in range(self.number_of_blobs):
            #     com[2] = com_lo[2] + comd_lo[2] * t[blob] + 0.5 * (-9.81) * t[blob] * t[blob]
            #     com[:2] = com_lo[:2] + comd_lo[:2] * t[blob]
            #     self.intermediate_flight_com_position.append(com.copy())
            return True

    def plotTrajectoryFlight(self):
        # plot com intermediate positions
        for blob in self.intermediate_flight_com_position:
            self.ros_pub.add_marker(blob, color=[0.5,0.5,0.5], radius=0.02)

    def setJumpPlatformPosition(self, target, com0):
        # create the message
        set_platform_position = SetModelStateRequest()
        # create model state
        model_state = ModelState()
        model_state.model_name = 'jump_platform'
        model_state.pose.position.x = target[0]
        model_state.pose.position.y = target[1]
        model_state.pose.position.z = target[2]-com0[2]
        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        set_platform_position.model_state = model_state
        # send request and get response (in this case none)
        self.set_state(set_platform_position)

    def customStartupProcedure(self):
        print(colored("Custom Startup Procedure", "red"))
        self.q_des = self.qj_0
        self.pid = PidManager(self.joint_names)
        # set joint pdi gains
        self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                            conf.robot_params[self.robot_name]['kd'],
                            conf.robot_params[self.robot_name]['ki'])
        p.resetRobot(basePoseDes=np.array([0, 0, conf.robot_params[self.robot_name]['spawn_z'],  0., 0., 0.]))
        while self.time <= self.startTrust:
            self.updateKinematics()
            if p.DEBUG!='swim' and p.DEBUG!='step' :
                self.tau_ffwd, self.grForcesW_des = self.wbc.gravityCompensation(self.W_contacts, self.wJ, self.h_joints,
                                                                        self.basePoseW, self.comPoseW)
            self.send_command(self.q_des, self.qd_des, self.tau_ffwd)

    def lerp(self, start, end, weight):
        return start + weight * (end - start)

    def cerp(self, start, end, weight, start_tangent: float = 1e-3, end_tangent: float = 1e-3):
        # Hermite basis functions
        h00 = (2 * weight**3) - (3 * weight**2) + 1
        h10 = weight**3 - 2 * weight**2 + weight
        h01 = (-2 * weight**3) + (3 * weight**2)
        h11 = weight**3 - weight**2

        # Interpolation
        return (h00 * start) + (h10 * start_tangent) + (h01 * end) + (h11 * end_tangent)


if __name__ == '__main__':
    p = QuadrupedJumpController('go1')
    world_name = 'fast.world'

    try:
        # p.startController(world_name='slow.world')
        p.startController(world_name=world_name,
                          use_ground_truth_pose=True,
                          use_ground_truth_contacts=True,
                          additional_args=['gui:='+str(p.use_gui),
                                           'go0_conf:='+p.go0_conf,
                                           'rviz:='+str(not p.real_robot)])
        # initialize data stucture to use them with desired values
        p.des_robot = RobotWrapper.BuildFromURDF(
            os.environ.get('LOCOSIM_DIR') + "/robot_urdf/generated_urdf/" + p.robot_name + ".urdf",
            root_joint=pinocchio.JointModelFreeFlyer())

        if p.real_robot:
            p.startTrust = 15.  # the startup procedure should be shorter than this! otherwise the time is wrong
            p.startupProcedure()
            p.updateKinematics()  # neeeded to call legodom to have an initial estimate of com position
        else:
            p.startTrust = 1.
            p.customStartupProcedure()

        # initial pose
        com_0 = p.basePoseW[:3].copy()
        eul_0 = p.basePoseW[3:].copy()

        # define jump action (relative)
        p.jumpDeltaStep = np.array([-0.4, 0.0, 0.])
        p.jumpDeltaOrient = np.array([0.0, 0., 0.0])

        # get the action from the policy (use p.jumpAgent to get values)
        p.jumpAgent.act(p.jumpDeltaStep, p.jumpDeltaOrient)

        # compute target pose
        p.target_position = com_0 + p.jumpDeltaStep
        p.target_orientation = eul_0 + p.jumpDeltaOrient

        # we have to do this because the training was done for height 0.3 TODO
        default_start = np.array([0., 0., 0.3])

        # extract liftoff position orientation from action
        # linear
        p.com_lo_b = (com_0-default_start) + p.jumpAgent.trunk_x_lo_b[0]
        p.comd_lo_b = p.jumpAgent.trunk_xd_lo_b[0]
        p.com_lo_e = (com_0-default_start) + p.jumpAgent.trunk_x_lo_e[0]
        p.comd_lo_e = p.jumpAgent.trunk_xd_lo_e[0]
        # angular
        p.eul_lo = p.jumpAgent.trunk_o_lo
        p.euld_lo = p.jumpAgent.trunk_od_lo
        # time
        p.T_th_b = p.jumpAgent.t_th_b[0].item()
        p.T_th_e = p.jumpAgent.t_th_e[0].item()
        p.T_th_total = p.T_th_b + p.T_th_e

        p.lerp_time = p.jumpAgent.lerp_time

        if p.DEBUG != 'none':
            p.initial_com = np.copy(com_0)
            print(f"Initial Com Position is {p.initial_com}")
            print(f"Initial Joint Position is {p.q}")
            print(f"Initial Joint torques {p.tau_ffwd}")
            print(colored(f"IMPORTANT: you cannot control both pitch and Z and expect 0 error on comX, only on base, because it is an impossible task!", "red"))
            p.T_th = np.inf
            p.T_th_total = np.inf

        if p.DEBUG=='step' or p.DEBUG=='swim':
            print(colored('!!!!!!!!!!!!!! PULL UP THE ROBOT !!!!!!!!!!!!!!','red'))
            ros.sleep(5.) #wait for user tu pull up the robot
            print(colored(f'STARTING {p.DEBUG} MODE','red'))
            p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp_real_swing'],
                                          conf.robot_params[p.robot_name]['kd_real_swing'], 
                                          conf.robot_params[p.robot_name]['ki_real_swing'] )
        
        p.computeHeuristicSolutionBezierLinear(com_0, p.com_lo_b, p.comd_lo_b, p.T_th_b)
        # we have the explosive part only for the lineat part
        p.computeHeuristicSolutionBezierAngular(eul_0, p.eul_lo, p.euld_lo, p.T_th_total)

        # this is for visualization
        if not p.DEBUG:
            p.computeTrajectoryBezier(p.T_th_b, p.com_lo_e)
        p.stance_legs = [True, True, True, True]

        # reset integration of feet
        p.W_feetRelPosDes = np.copy(p.W_contacts - com_0)
        p.W_contacts_sampled = np.copy(p.W_contacts)

        # if not p.real_robot:
        #      p.setSimSpeed(dt_sim=0.001, max_update_rate=50, iters=1500)

        p.lm = LandingManager(p)

        while not ros.is_shutdown():
            if p.lm.lc is not None:
                p.updateKinematics(update_legOdom=p.lm.lc.lc_events.touch_down.detected)
            else:
                p.updateKinematics()

            # release base
            if p.firstTime:
                print(colored(f'Start of the control loop {p.time}'))
                p.firstTime = False
                p.trustPhaseFlag = True
                p.T_fl = None
                p.startTrust = p.time
                p.computeIdealLanding(p.com_lo_e, p.comd_lo_e, p.target_position)
                # if (p.computeIdealLanding(com_lo, comd_lo, p.target_position)):
                # if (p.computeIdealLanding(com_exp, comd_exp, p.target_position)):
                #     error = np.linalg.norm(
                #         p.ideal_landing - p.target_position)
                # else:
                #     break
            # compute joint reference
            if (p.trustPhaseFlag):
                t = p.time - p.startTrust
                p.q_des, p.qd_des, p.tau_ffwd, p.basePoseW_des, p.baseTwistW_des = p.evalThrust(t, p.T_th_b, p.T_th_e, p.com_lo_b, p.comd_lo_b, p.com_lo_e, p.comd_lo_e)

                # if p.time >= (p.startTrust + p.T_th):
                if p.time >= (p.startTrust + p.T_th_total):
                    p.trustPhaseFlag = False
                    # we se this here to have enough retraction (important)
                    p.q_t_th = p.q_des.copy()
                    p.qd_t_th = p.qd_des.copy()
                    p.tau_ffwd = np.zeros(12)
                    print(colored(f"thrust completed! at time {p.time}", "red"))
                    # Reducing gains for more complaint landing  ATTENTIONNN!!! THIS MIGHT LEAD TO INSTABILITIES AND BREAK THE REAL ROBOT
                    real_str = '_real' if p.real_robot else ''
                    p.pid.setPDjoints(conf.robot_params[p.robot_name][f'kp{real_str}_swing'],
                                        conf.robot_params[p.robot_name][f'kd{real_str}_swing'],
                                        conf.robot_params[p.robot_name][f'ki{real_str}_swing'])
                    print(colored(f"pdi: {p.pid.joint_pid}"))

                    if p.DEBUG != 'none':
                        print('time is over: ', p.time, 'tot_time:',
                              p.startTrust + p.T_th_total)
                        break
            else:
                # if not p.detectedApexFlag:
                #     elapsed_time = p.time - (p.startTrust + p.T_th_total)
                #     elapsed_ratio = np.clip(elapsed_time / p.lerp_time, 0, 1)
                #     p.q_des = p.cerp(p.q_t_th, conf.robot_params[p.robot_name]['q_land'],  elapsed_ratio).copy()
                #     p.qd_des = p.cerp(p.qd_t_th, np.zeros_like(p.qd_t_th), elapsed_ratio).copy()

                p.detectApex()
                if (p.detectedApexFlag):
                    elapsed_time_apex = p.time - p.t_apex
                    elapsed_ratio_apex = elapsed_time_apex / (p.lerp_time)
                    if p.use_landing_controller:
                        if elapsed_ratio_apex <= 1.0:
                            pass
                            # p.q_des = p.lerp(p.q_apex, p.qj_0,
                            #                     elapsed_ratio_apex).copy()
                            # p.qd_des = p.lerp(p.qd_apex, np.zeros_like(p.qd_apex),
                            #                     elapsed_ratio_apex).copy()
                        else:
                            p.q_des, p.qd_des, p.tau_ffwd, finished = p.lm.runAtApex(
                                p.basePoseW, p.baseTwistW, useIK=True, useWBC=True, naive=False)
                            if finished:
                                # break
                                p.tau_ffwd, p.grForcesW_des = p.wbc.gravityCompensationBase(p.B_contacts,
                                                                                            p.wJ,
                                                                                            p.h_joints,
                                                                                            p.basePoseW)
                    else:
                        # Simple landing strategy, interploate to extension
                        # set jump position (avoid collision in jumping)
                        elapsed_ratio_apex = np.clip(
                            elapsed_ratio_apex, 0, 1)
                        p.q_des = p.lerp(p.q_apex, p.q_land,
                                            elapsed_ratio_apex).copy()
                        p.qd_des = p.lerp(p.qd_apex, np.zeros_like(p.qd_apex),
                                            elapsed_ratio_apex).copy()
                        
                        if not p.touchdown_detected:
                            p.touchdown_detected = p.detectTouchDown()
                            if p.touchdown_detected:
                                real_str = '_real' if p.real_robot else ''
                                p.pid.setPDjoints(  conf.robot_params[p.robot_name][f'kp{real_str}_land'],
                                                    conf.robot_params[p.robot_name][f'kd{real_str}_land'],
                                                    conf.robot_params[p.robot_name][f'ki{real_str}_land'])
                                print(colored(f"pdi: {p.pid.joint_pid}"))
                                p.landing_position = p.u.linPart(p.basePoseW)
                                p.landing_orientation = p.u.angPart(
                                    p.basePoseW)
                                p.landing_error = p.target_position - p.landing_position
                                p.orient_error = p.target_orientation - p.landing_orientation
                                # it does not make sense to compute perc error considering variable Z either succeed or not
                                perc_err_xy = 100. * np.linalg.norm(p.landing_error[:2]) / np.linalg.norm(com_0 - p.target_position)
                                perc_err_orient = 100. * np.linalg.norm(p.orient_error) / np.linalg.norm(eul_0 - p.target_orientation)
                                print(colored(f"TOUCHDOWN detected at t {p.time}", "red"))
                                print(colored(f"landed at {p.basePoseW} with  perc.  error xy {perc_err_xy} and perc orient_error {perc_err_orient}", "green"))
                                print(colored(f"started at {com_0} and orient {eul_0}", "green"))
                                print(colored(f"target at {p.target_position}, {p.target_orientation}", "green"))
                                # if p.real_robot:
                                #     p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp_real'],
                                #                       conf.robot_params[p.robot_name]['kd_real'],
                                #                       conf.robot_params[p.robot_name]['ki_real'])
                                #     print(colored(f"pdi: {p.pid.joint_pid}"))
                                
                                p.time_td = p.time 
                                p.qdes_td = p.q_des.copy()   
                        else:
                            # break
                            p.tau_ffwd, p.grForcesW_des = p.wbc.gravityCompensationBase(p.B_contacts,
                                                                                        p.wJ,
                                                                                        p.h_joints,
                                                                                        p.basePoseW)
                            # if p.real_robot:
                            elapsed_time = p.time - p.time_td
                            elapsed_ratio = np.clip(elapsed_time / 0.4, 0, 1)
                            p.q_des = p.cerp(p.qdes_td, p.q_final,  elapsed_ratio).copy()

                else:

                    # Interpolate for retraction
                    elapsed_time = p.time - (p.startTrust + p.T_th_total)
                    elapsed_ratio = np.clip(
                        elapsed_time / p.lerp_time, 0, 1)
                    p.q_des = p.cerp(p.q_t_th, p.q_retraction, elapsed_ratio).copy()
                    p.qd_des = p.lerp(p.qd_t_th, np.zeros_like(p.qd_t_th),
                                        elapsed_ratio).copy()
                    # NOTE: this is temporary made to swich immediatly to q0
                    # needed also for the landing controller
                    # p.q_des = p.lerp(p.q_t_th, p.qj_0,
                    #                             elapsed_ratio).copy()
                    # p.qd_des = p.lerp(p.qd_t_th, np.zeros_like(p.qd_t_th),
                    #                     elapsed_ratio).copy()

            if not p.real_robot:
                if p.DEBUG == 'none':
                    p.plotTrajectoryBezier()
                    p.plotTrajectoryFlight()
                # plot target
                p.ros_pub.add_marker(
                    p.target_position, color="blue", radius=0.1)

                if (np.linalg.norm(p.ideal_landing) > 0.):
                    p.ros_pub.add_marker(p.ideal_landing, color="purple", radius=0.1)

            if not p.real_robot:
                p.visualizeContacts()
            p.logData()

            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd, soft_limits=1.0, clip_commands=True)

            # log variables
            p.rate.sleep()
            p.sync_check()

            p.time = np.round(p.time + p.dt, 4)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        filename = f'quadruped_jump.mat'
        mio.savemat(filename, {'time': p.time_log,
                               'q': p.q_log, 'q_des': p.q_des_log,
                               'qd': p.qd_log, 'qd_des': p.qd_des_log,
                               'tau': p.tau_log, 'tau_des': p.tau_des_log, 'tau_ffw': p.tau_ffwd_log,
                               'basePoseW': p.basePoseW_log, 'basePoseW_des': p.basePoseW_des_log,
                               'baseTwistW': p.baseTwistW_log, 'baseTwistW_des': p.baseTwistW_des_log,
                               'grf': p.grForcesW_log, 'grf_des': p.grForcesW_des_log,
                               'contact': p.contact_state_log,
                               'landing_position': p.landing_position,
                               'landing_orientation': p.landing_orientation,
                               'landing_error': p.landing_error,
                               'orient_error': p.orient_error})

    print("end control!!")
    p.deregister_node()

    if conf.plotting:
        plotJoint('position', time_log=p.time_log,
                  q_log=p.q_log, q_des_log=p.q_des_log)
        plotJoint('velocity', time_log=p.time_log,
                  qd_log=p.qd_log, qd_des_log=p.qd_des_log)
        plotJoint('torque', time_log=p.time_log,
                  tau_log=p.tau_log,   tau_des_log=p.tau_des_log)
        plotJoint('torque', time_log=p.time_log,
                  tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log)

        # COM
        plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log,
                  Pose_log=p.basePoseW_log,  title='Base', frame='W')
        plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log,
                  Twist_log=p.baseTwistW_log,  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotContacts('GRFs', time_log=p.time_log, des_Forces_log=p.grForcesW_des_log,
                     Forces_log=p.grForcesW_gt_log, contact_states=p.contact_state_log, frame='W')
