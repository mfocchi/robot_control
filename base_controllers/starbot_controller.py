# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function
import rospy as ros
from utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from base_controller import BaseController
from base_controllers.utils.common_functions import plotCoM, plotJoint

import  params as conf
robotName = "solo"

class StarbotController(BaseController):
    
    def __init__(self, robot_name="solo"):
        super().__init__(robot_name=robot_name)
        self.freezeBaseFlag = False
        print("Initialized starbot controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                pass
            super().logData()

def talker(p):
    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()
    p.startupProcedure()
    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    p.q_des = np.copy(p.q_des_q0)

    while not ros.is_shutdown():
        # update the kinematics
        p.updateKinematics()
        p.tau_ffwd = np.zeros(p.robot.na)
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

        # plot actual (green) and desired (blue) contact forces
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW / (6 * p.robot.robot_mass)),
                                "green")
            if (p.use_ground_truth_contacts):
                p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW_gt / (6 * p.robot.robot_mass)),
                                    "red")
        p.ros_pub.publishVisual()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

        # stops the while loop if  you prematurely hit CTRL+C
        if ros.is_shutdown():
            print("Shutting Down")
            break

            # restore PD when finished
    p.pid.setPDs(conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.0)
    ros.sleep(1.0)
    print("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()

if __name__ == '__main__':

    p = StarbotController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if conf.plotting:
            plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, p.joint_names)


