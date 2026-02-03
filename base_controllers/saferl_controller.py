# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)
from base_controllers.quadruped_controller import QuadrupedController
from base_controllers.utils.common_functions import plotFrame, plotJoint
from base_controllers.components.rl_velocity_controller.rl_controller import RlVelocityController
from base_controllers.components.safe_rl.value_function.value_function_manager import ValueFunctionManager
from base_controllers.utils.common_functions import launchFileNode
import params as conf
import numpy as np
import os
from base_controllers.utils.joyManager import JoyManager
robotName = "aliengo"  # needs to inherit BaseController
from termcolor import colored
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder

class SafeRLController(QuadrupedController):

    def __init__(self, robot_name="myrobot"):
        super().__init__(robot_name=robot_name)
        print("Initialized   controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']


    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            ## add your logs here
            pass
        super().logData()


if __name__ == '__main__':
    p = SafeRLController('aliengo')
    world_name = 'fast.world'
    use_gui=False#True
    p.state_estimation = 'pronto'  # 'odometry','imu', 'pronto', 'ground_truth' (only sim)
    rl_controller = RlVelocityController(p.robot_name, p.dt)
    p.SAVE_BAG = False  #
    vf_frequency = 100  # Hz
    vf_decimation = (1 / p.dt) / (vf_frequency)
    step = 0
    isrec = True
    use_joy = True
    sim_push = False

    if use_joy:
        joy = JoyManager()

    #load value function NN
    vf = ValueFunctionManager()
    try:
        # p.startController(world_name='slow.world')
        p.startController(world_name=world_name,
                          use_ground_truth_contacts=True,
                          additional_args=['gui:=' + str(use_gui),
                                           'go0_conf:=standDown'])
        if p.SAVE_BAG:
            p.recorder = RosbagControlledRecorder(bag_name="saferl.bag", record_from_startup_=False)
            p.recorder.start_recording_srv()
        #p.setSimSpeed(max_update_rate=300)
        p.startupProcedure()
        if p.state_estimation=='pronto':
            launchFileNode("pronto_aliengo", "pronto_aliengo.launch", additional_args=['pronto_conf:='+p.pronto_config,
                                                                                       'use_sim_time:='+str(not p.real_robot)])
        p.pid.setPDjoints(rl_controller.kp, rl_controller.kd, np.full(12, 0))
        p.counter = 0
        p.startTime = p.time

        #profiler = Profiler(function_name=vf.computeValueFnc)
        #to reduce simulation frequency
        #p.setSimSpeed(dt_sim=0.001, max_update_rate=300, iters=1500)
        while not ros.is_shutdown():
            p.updateKinematics()

            if p.gracefulCollapseFlag:
                if p.gracefulCollapse():
                    break
            if use_joy:
                axes, buttons = joy.get_commands()
                # use a scaling to make the joy input less reactive
                long_x = 0.3 * axes[0]
                long_y = 0.3 * axes[1]
                rot_z = 0.4 * axes[2]
                # safety layer
                if buttons[0] and not p.gracefulCollapseFlag:
                    print(colored("start Graceful collapse", "red"))
                    p.kp_act, p.kd_act, p.ki_act = p.pid.getPDjoints()
                    print(colored(f"Storing actual pd: {p.kp_act}, {p.kd_act},{p.ki_act}"), "red")
                    p.gracefulCollapseFlag = True
                if buttons[1]:
                    print(colored("Severe shutdown!", "red"))
                    if p.state_estimation == 'pronto':
                        os.system(" rosnode kill /aliengo_joint_swapper")
                        os.system(" rosnode kill /pronto_aliengo")
                    ros.signal_shutdown("killed")
                    p.deregister_node()
                    break
            if  (p.time > (p.startTime + 3.)):
                #rl_controller.velocity_cmd = np.array([0.5, 0.0, 0.0])
                if use_joy:
                    rl_controller.velocity_cmd = np.array([long_x, long_y, rot_z])
                else:
                    rl_controller.velocity_cmd = np.array([0.2, 0.0, 0.0])

                p.baseTwistW_des[:3] = p.b_R_w.T @ np.append(rl_controller.velocity_cmd[:2], 0.0)
                p.baseTwistW_des[5] = rl_controller.velocity_cmd[2]
                lin_vel_b = p.b_R_w.dot(p.baseTwistW[:3])
                ang_vel_b = p.b_R_w.dot(p.baseTwistW[3:6])
                proj_gravity_b = p.b_R_w.dot(np.array([0, 0, -1]))
                # pushes of increasing entity
                if not p.real_robot and p.time % 2. == 0 and isrec and sim_push:
                     p.applyForce(0, 50*p.counter, 0, 0, 0, 0, 0.25)
                     print(50*p.counter)
                     p.counter+=1
                if isrec:
                     # nominal policy
                     p.rl_q_des = rl_controller.action(lin_vel_b, ang_vel_b, proj_gravity_b, p.q, p.qd, policy_type="default")
                else:
                    # backup policy
                    #print(colored("I am executing backup policy!","red"))
                    rl_controller.velocity_cmd = np.array([0.0, 0.0, 0.0])
                    p.rl_q_des = rl_controller.action(lin_vel_b, ang_vel_b, proj_gravity_b, p.q, p.qd, policy_type="safe")
                #check this
                #print('ang_vel_b A',ang_vel_b)
                #print('proj_gravity A',proj_gravity)
                if step % decimation == 0:# and isrec:
                     isrec, V_safe = vf.computeValueFnc(body_ang_vel=ang_vel_b, proj_gravity=proj_gravity_b, joint_pos=p.q, joint_vel=p.qd, threshold=0.6, vf_additional_term = 0.0)
                #     #isrec = True #just record value functtion but dont use
                #     #print(V_safe)
                # '''elif step % decimation == 0:
                #     if np.all(np.abs(lin_vel_b < 10e-2)) and np.all(np.abs(p.qd) < 10e-2):
                #         #print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
                #         isrec = True
                #         rl_controller.velocity_cmd = np.array([0.5, 0.0, 0.0])
                #         vf.count = 0
                #         vf.VF = True'''
                step += 1

                # time based switch
                # if p.time > (p.startTime + 3.): #backup policy
                #     rl_controller.velocity_cmd = np.array([0.0, 0.0, 0.0])
                #     p.rl_q_des = rl_controller.action(lin_vel_b, ang_vel_b, proj_gravity_b, p.q, p.qd, policy_type="safe")
                # else:
                #     p.rl_q_des = rl_controller.action(lin_vel_b, ang_vel_b, proj_gravity_b, p.q, p.qd, policy_type="default")

                # switch off wbc
                p.grForcesW_des = np.zeros((12))
                p.tau_ffwd = np.zeros(12)
                p.send_command(p.rl_q_des, np.zeros(12), np.zeros(12), log_data_in_send_command=False)

            else:
                p.tau_ffwd, p.grForcesW_des = p.wbc.gravityCompensationBase(p.B_contacts,    p.wJ,
                                                                                p.h_joints,
                                                                                p.comPoseW)
                p.send_command(p.q_des, p.qd_des, p.alphaCollapse * p.tau_ffwd, log_data_in_send_command=False)

            p.visualizeContacts()

    except (ros.ROSInterruptException, ros.service.ServiceException):
        if p.SAVE_BAG:
            p.recorder.stop_recording_srv()
        ros.signal_shutdown("killed")
        p.deregister_node()

    if conf.plotting:
        plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, sharex=True, sharey=False,
                  start=0, end=-1)
        plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log, Twist_log=p.baseTwistW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
    if p.SAVE_BAG:
        p.recorder.stop_recording_srv()