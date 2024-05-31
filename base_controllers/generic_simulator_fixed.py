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
from base_controllers.base_controller_fixed import BaseControllerFixed
from base_controllers.utils.common_functions import plotFrame, plotJoint
import rospkg
import params as conf
robotName = "z1"  # needs to inherit BaseControllerFixed

class GenericSimulator(BaseControllerFixed):
    
    def __init__(self, robot_name="myrobot"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.freezeBaseFlag = False
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
    p.startSimulator(launch_file='standard') #load start_framework.launch
    #TODO fix this in the future to unify fixed and floating base robots description
    xacro_path = rospkg.RosPack().get_path(p.robot_name+'_description') + '/robots/' + p.robot_name + '.urdf.xacro'
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    p.startupProcedure()
    # To implement your own controller decomment this
    #p.pid.setPDs(0, 0, 0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    p.q_des = np.copy(p.q_des_q0)

    while not ros.is_shutdown():
        #p.tau_ffwd = np.zeros(p.robot.na)
        p.q_des = p.q_des_q0 + 0.05 * np.sin(2 * np.pi * 0.5 * p.time)

        #here you can implement your own controller
        #p.tau_ffwd = 20*(p.q_des-p.q) + 1*(p.qd_des-p.qd)
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

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


