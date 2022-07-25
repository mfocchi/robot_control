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

        self.rope_index = 3
        self.leg_index = np.array([7,8,9])
        self.base_passive_joints = np.array([4,5,6])
        self.anchor_passive_joints = np.array([0,1,2])

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
        self.w_R_b = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).rotation

        self.x_ee =  self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q , self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
        self.Jleg = self.J[:, -3:]
        self.dJdq = self.robot.frameClassicAcceleration(self.q , self.qd , None,  self.robot.model.getFrameId(frame_name), False).linear


        self.l = self.q[p.rope_index]
        self.theta = self.q[0]
        self.phi = self.q[2]

        # compute com variables accordint to a frame located at the foot
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data, self.q)

        # from ground truth
        self.com = self.base_pos + robotComB

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
        #self.comdd_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qdd_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']


    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                pass
            super().logData()



    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

def talker(p):

    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.startupProcedure()

    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    p.updateKinematicsDynamics()

    # jump parameters
    p.startJump = 3.0
    p.jumps = [{"Fun": 8.9, "Fut": 0., "K_rope": 60, "Tf": 1.0},
               {"Fun": 8.9, "Fut": 0., "K_rope": 60, "Tf": 1.0},
               {"Fun": 8.9, "Fut": 0., "K_rope": 60, "Tf": 1.0}]
    p.numberOfJumps = len(p.jumps)
    p.thrustDuration = 0.05
    p.stateMachine = 'idle'
    p.l_0 = conf.robot_params[p.robot_name]['q_0'][p.rope_index]
    p.jumpNumber  = 0

    while True:
        p.tau_ffwd = np.zeros(p.robot.na)
        # update the kinematics
        p.updateKinematicsDynamics()

        #multiple jumps state machine
        if ( p.stateMachine == 'idle') and (p.time > p.startJump) and (p.jumpNumber<p.numberOfJumps):
            print(colored("-------------------------------", "blue"))
            print(colored(f"Start trusting Jump number : {p.jumpNumber}" , "blue"))
            p.pid.setPDjoint(p.base_passive_joints, 0., 2., 0.)
            p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
            p.pid.setPDjoint(p.leg_index, 0., 0., 0.)
            p.b_Fu = np.array([p.jumps[p.jumpNumber]["Fun"], p.jumps[p.jumpNumber]["Fut"], 0.0])
            p.w_Fu = p.w_R_b.dot(p.b_Fu)
            p.stateMachine = 'thrusting'

        if (p.stateMachine == 'thrusting'):
            #p.tau_ffwd[p.leg_index] = - np.linalg.inv(p.Jleg.T).dot(p.w_Fu)
            p.tau_ffwd[9] = -20
            p.tau_ffwd[p.rope_index] = - p.jumps[p.jumpNumber]["K_rope"] * (p.l - p.l_0)
            p.ros_pub.add_arrow( p.x_ee, p.w_Fu, "red")
            if (p.time > (p.startJump + p.thrustDuration)):
                print(colored("Stop Trhusting", "blue"))
                p.stateMachine = 'flying'
                # reenable  the PDs of default values for landing
                p.pid.setPDjoint(p.base_passive_joints, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'] ,  0.)
                p.pid.setPDjoint(p.leg_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'],  0.)
                print(colored("Start Flying", "blue"))

        if (p.stateMachine == 'flying'):
            # keep extending rope
            p.tau_ffwd[p.rope_index] = - p.jumps[p.jumpNumber]["K_rope"] * (p.l - p.l_0)
            if (p.time > (p.startJump + p.thrustDuration + p.jumps[p.jumpNumber]["Tf"])):
                print(colored("Stop Flying", "blue"))
                # reset the qdes
                p.stateMachine = 'idle'

                # enable PD for rope and reset the PD reference to the new estension
                p.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'][p.rope_index], conf.robot_params[p.robot_name]['kd'][p.rope_index], 0.)
                p.q_des[p.rope_index] = p.q[p.rope_index]
                p.l_0 = p.l
                p.jumpNumber += 1
                p.startJump = p.time

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


        
