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
import params as conf
import numpy as np
robotName = "aliengo"  # needs to inherit BaseController

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
    use_gui=True
    p.state_estimation = 'ground_truth'  # 'odometry','imu', 'pronto', 'ground_truth' (only sim)
    rl_controller = RlVelocityController(p.robot_name, p.dt)

    #load value function NN
    vf = ValueFunctionManager()
    try:
        # p.startController(world_name='slow.world')
        p.startController(world_name=world_name,
                          use_ground_truth_contacts=True,
                          additional_args=['gui:=' + str(use_gui),
                                           'go0_conf:=standDown'])

        p.startupProcedure()
        p.pid.setPDjoints(rl_controller.kp, rl_controller.kd, np.full(12, 0))
        p.counter = 0
        p.startTime = p.time
        while not ros.is_shutdown():
            p.updateKinematics()
            rl_controller.velocity_cmd = np.array([0.5, 0.0, 0.0])
            p.baseTwistW_des[:3] = p.b_R_w.T @ np.append(rl_controller.velocity_cmd[:2], 0.0)
            p.baseTwistW_des[5] = rl_controller.velocity_cmd[2]
            lin_vel_b = p.b_R_w.dot(p.baseTwistW[:3])
            ang_vel_b = p.b_R_w.dot(p.baseTwistW[3:6])
            proj_gravity = p.b_R_w.dot(np.array([0, 0, -1]))
            # pushes of increasing entity
            if p.time % 2. == 0:
                p.applyForce(0, 50*p.counter, 0, 0, 0, 0, 0.25)
                print(50*p.counter)
                p.counter+=1
            #nominal policy
            p.rl_q_des = rl_controller.action(lin_vel_b, ang_vel_b, proj_gravity, p.q, p.qd, policy_type="default")
            #check this
            isrec, V_safe = vf.computeValueFnc(body_ang_vel=ang_vel_b, proj_gravity=proj_gravity, joint_pos=p.q, joint_vel=p.qd, threshold=0.3)
            print(V_safe)

            # time based switch
            # if p.time > (p.startTime + 3.): #backup policy
            #     rl_controller.velocity_cmd = np.array([0.0, 0.0, 0.0])
            #     p.rl_q_des = rl_controller.action(lin_vel_b, ang_vel_b, proj_gravity, p.q, p.qd, policy_type="safe")
            # else:
            #     p.rl_q_des = rl_controller.action(lin_vel_b, ang_vel_b, proj_gravity, p.q, p.qd, policy_type="default")

            # switch off wbc
            p.grForcesW_des = np.zeros((12))
            p.tau_ffwd = np.zeros(12)
            p.send_command(p.rl_q_des, np.zeros(12), np.zeros(12), log_data_in_send_command=True)
            #p.visualizeContacts()

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    if conf.plotting:
        plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, sharex=True, sharey=False,
                  start=0, end=-1)
        plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log, Twist_log=p.baseTwistW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
