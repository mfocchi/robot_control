# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function
import rospy as ros
from utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from numpy import nan
from utils.common_functions import plotJoint, plotCoMLinear
from termcolor import colored
import os
from base_controller_fixed import BaseControllerFixed

import  params as conf
robotName = "climbingrobot"

class ClimbingrobotController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.EXTERNAL_FORCE = False
        self.freezeBaseFlag = False
        print("Initialized climbingrobot controller---------------------------------------------------------------")

    def applyForce(self):
        from geometry_msgs.msg import Wrench, Point
        wrench = Wrench()
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 30
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        reference_frame = "world" # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x = 0, y = 0, z = 0)
        try:
            self.apply_body_wrench(body_name=self.robot_name+"::base_link", reference_frame=reference_frame, reference_point=reference_point , wrench=wrench, duration=ros.Duration(10))
        except:
            pass

    def updateKinematicsDynamics(self):
        # q is continuously updated
        self.robot.computeAllTerms(self.q, self.qd )
        # joint space inertia matrix                
        self.M = self.robot.mass(self.q )
        # bias terms                
        self.h = self.robot.nle(self.q  , self.qd )
        #gravity terms                
        self.g = self.robot.gravity(self.q )
        #compute ee position  in the world frame  
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.anchor_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor')).translation
        self.base_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).translation
        self.x_ee =  self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J6 = self.robot.frameJacobian(self.q , self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
        self.J = self.J6[:, 3:]
        self.dJdq = self.robot.frameClassicAcceleration(self.q , self.qd , None,  self.robot.model.getFrameId(frame_name), False).linear

        # compute com variables accordint to a frame located at the foot
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data)

        # from ground truth
        self.com = self.q[:3] + robotComB
        self.comd = self.qd[:3]

        #compute contact forces TODO
        #self.estimateContactForces()

    def estimateContactForces(self):
        self.contactForceW = np.linalg.inv(self.J.T).dot( (self.h-self.tau)[3:] )

    def initVars(self):
        super().initVars()
        self.qdd_des =  np.zeros(self.robot.na)
        self.base_accel = np.zeros(3)
        # init new logged vars here
        self.com_log =  np.empty((3, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.comd_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        #self.comdd_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qdd_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']


    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                pass
            super().logData()

    def freezeBase(self, flag):
        if (self.freezeBaseFlag):
            self.freezeBaseFlag = flag
            print("releasing base")
            self.tau_ffwd[2] = 0.
            # set base joints PD to zero
            self.pid.setPDjoint(0, 0., 0., 0.)
            self.pid.setPDjoint(1, 0., 0., 0.)
            self.pid.setPDjoint(2, 0., 0., 0.)


        if (not self.freezeBaseFlag) and (flag):
            self.freezeBaseFlag = flag
            print("freezing base")
            self.q_des = np.copy(self.q_des_q0)
            self.qd_des = np.copy(np.zeros(self.robot.na))
            self.tau_ffwd = np.copy(np.zeros(self.robot.na))
            self.tau_ffwd[2] = self.g[2]  # compensate gravitu in the virtual joint to go exactly there
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'],
                                 np.zeros(self.robot.na))


    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

def talker(p):

    p.start()
    p.startSimulator("climb_wall.world")
    #p.startSimulator()
    p.loadModelAndPublishers()
    p.startupProcedure()

    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    #p.updateKinematicsDynamics()
    #p.freezeBase(True)
    p.thrustingFlag = False
    p.thrustDuration = 0.5
    p.q_des = np.copy(p.q_des_q0)

    while True:
        p.tau_ffwd = np.zeros(p.robot.na)
        # update the kinematics
        p.updateKinematicsDynamics()

        if (p.time > 3.0) and  (p.time < 3.0 +p.thrustDuration )  and (not p.thrustingFlag):
            p.pid.setPDjoint(4, 0., 2., 0.)
            p.pid.setPDjoint(5, 0., 2., 0.)
            p.pid.setPDjoint(3, 0., 0., 0.)
            p.pid.setPDjoint(9, 0., 2., 0.)
            p.thrustingFlag = True
            print(colored("Start Trhusting", "blue"))

        if (p.thrustingFlag):
            p.tau_ffwd[9] = -20.
            p.tau_ffwd[3] = -60.5 * (p.q[3] - conf.robot_params[p.robot_name]['q_0'][3])
            if (p.time > (3.0 + p.thrustDuration)):
                p.thrustingFlag = False
                p.pid.setPDjoint(3, conf.robot_params[p.robot_name]['kp'][3], conf.robot_params[p.robot_name]['kd'][3], 0.)
                p.pid.setPDjoint(4, conf.robot_params[p.robot_name]['kp'][4], conf.robot_params[p.robot_name]['kd'][4],  0.)
                p.pid.setPDjoint(5, conf.robot_params[p.robot_name]['kp'][5], conf.robot_params[p.robot_name]['kd'][5],
                                 0.)
                p.pid.setPDjoint(9, conf.robot_params[p.robot_name]['kp'][9], conf.robot_params[p.robot_name]['kd'][9],
                                 0.)
                print(colored("Stop Trhusting", "blue"))
                p.q_des[3] = p.q[3]

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        # log variables
        p.logData()



        # disturbance force
        # if (p.time > 3.0 and p.EXTERNAL_FORCE):
        #     p.applyForce()
        #     p.EXTERNAL_FORCE = False

        # plot  rope bw base link and anchor
        p.ros_pub.add_arrow(p.anchor_pos, (p.base_pos - p.anchor_pos), "green")
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        p.ros_pub.publishVisual()

        # wait for synconization of the control loop
        rate.sleep()

        p.time = p.time + conf.robot_params[p.robot_name]['dt']

        # stops the while loop if  you prematurely hit CTRL+C
        if ros.is_shutdown():
            print("Shutting Down")
            break

    print("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()




if __name__ == '__main__':
    p = ClimbingrobotController(robotName)

    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
        # print("PLOTTING")
        # plotCoMLinear('com position', 1, p.time_log, None, p.com_log)
        # plotCoMLinear('contact force', 2, p.time_log, None, p.contactForceW_log)
        # plotJoint('position', 3, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, p.qdd_des_log, None,
        #           joint_names=conf.robot_params[p.robot_name]['joint_names'])
        # plotJoint('velocity', 4, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, p.qdd_des_log, None,
        #           joint_names=conf.robot_params[p.robot_name]['joint_names'])
        # plotJoint('acceleration', 5, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, p.qdd_des_log, None,
        #           joint_names=conf.robot_params[p.robot_name]['joint_names'])
        # plt.show(block=True)


        
