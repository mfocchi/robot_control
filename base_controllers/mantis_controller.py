# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from base_controllers.base_controller import BaseController

from base_controllers.utils.common_functions import plotFrame, plotJoint

import params as conf
robotName = "mantis" # needs to inherit BaseController

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="myrobot"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.freezeBaseFlag = False
        self.apply_external_wrench = True
        self.time_external_wrench = 2.
        self.dist_duration = 5.
        print("Initialized murobot controller---------------------------------------------------------------")

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
    additional_args = None
    p.startSimulator(additional_args = additional_args)
    p.loadModelAndPublishers()
    p.initSubscribers()
    p.initVars()
    p.startupProcedure()

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    # use this if rostopic hz /command it gives frequency < rate
    #p.setSimSpeed(max_update_rate=100)

    #initial configuration
    p.q_des = np.copy(p.q_des_q0)
    # control loop
    while not ros.is_shutdown():
        # update the kinematics
        p.updateKinematics()

        p.tau_ffwd = np.zeros(p.robot.na)
        #p.q_des = p.q_des_q0  + 0.3 * np.sin(2*np.pi*0.5*p.time)

        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        if (p.apply_external_wrench and p.time > p.time_external_wrench):
            print("START APPLYING EXTERNAL WRENCH")
            p.base_dist = np.array([200,0,0])
            p.applyForce(p.base_dist[0],p.base_dist[1], p.base_dist[2], 0.0, 0.0, 0.0, p.dist_duration)
            p.apply_external_wrench = False


        # log variables
        p.logData()


        # visual
        if (p.time > p.time_external_wrench) and (p.time < (p.time_external_wrench + p.dist_duration)):
            p.ros_pub.add_arrow(p.basePoseW[:3], p.base_dist / 100., "blue", scale=4.5)

        # plot actual (green) and desired (blue) contact forces
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg],
                                p.contact_state[leg] * p.u.getLegJointState(leg, p.grForcesW / (6 * p.robot.robotMass)),
                                "green")
            if (p.use_ground_truth_contacts):
                p.ros_pub.add_arrow(p.W_contacts[leg],
                                    p.u.getLegJointState(leg, p.grForcesW_gt / (6 * p.robot.robotMass)), "red")
        p.ros_pub.publishVisual(delete_markers=True)

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

if __name__ == '__main__':
    p = GenericSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names = p.joint_names)
        plotFrame('position', time_log=p.time_log, Pose_log=p.basePoseW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log, joint_names=p.joint_names)


