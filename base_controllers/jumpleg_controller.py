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
from utils.common_functions import plotJoint, plotAdmittanceTracking, plotEndeff

from base_controller_fixed import BaseControllerFixed
import  params as conf
robotName = "jumpleg"

class JumpLegController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)

        self.freezeBase = True
        print("Initialized jump leg controller---------------------------------------------------------------")

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
            self.apply_body_wrench(body_name=self.robot_name+"::wrist_3_link", reference_frame=reference_frame, reference_point=reference_point , wrench=wrench, duration=ros.Duration(10))
        except:
            pass

    def updateKinematicsDynamics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        self.robot.computeAllTerms(self.q, self.qd) 
        # joint space inertia matrix                
        self.M = self.robot.mass(self.q)
        # bias terms                
        self.h = self.robot.nle(self.q, self.qd)
        #gravity terms                
        self.g = self.robot.gravity(self.q)        
        #compute ee position  in the world frame  
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        # TODO compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]

         #compute contact forces
        self.estimateContactForces()

    def estimateContactForces(self):
        pass
        #TODO
        #self.contactForceW = np.linalg.inv(self.J.T).dot(self.h-self.tau)[:3]
                                 
    def initVars(self):
        super().initVars()

        # init new logged vars here


    def logData(self):
            # init  new logged vars here (before the call logdata)

            super().logData()

def talker(p):

    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()     
    p.startupProcedure()
    ros.sleep(1.0)
    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    #control loop
    while True:
        #update the kinematics
        p.updateKinematicsDynamics()
        p.q_des = np.copy(p.q_des_q0)
        if ( p.time > 2.0):
            p.tau_ffwd[3:] = -p.J.T.dot( p.g[:3])  # gravity compensation

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.ros_pub.add_arrow(p.x_ee + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")
        # log variables
        #p.logData() TODO
        if (p.freezeBase and p.time > 2.0):
            print("releasing base")
            p.pid.setPDjoint(0, 0.0,0.0,0.0)
            p.pid.setPDjoint(1, 0.0,0.0,0.0)
            p.pid.setPDjoint(2, 0.0,0.0,0.0)
            p.freezeBase = False


        # disturbance force
        if (p.time > 3.0 and p.EXTERNAL_FORCE):
            p.applyForce()
            p.EXTERNAL_FORCE = False

        # plot end-effector
        p.ros_pub.add_marker(p.x_ee + p.base_offset)
        p.ros_pub.publishVisual()

        #wait for synconization of the control loop
        rate.sleep()

        p.time = p.time + conf.robot_params[p.robot_name]['dt']
       # stops the while loop if  you prematurely hit CTRL+C
        if ros.is_shutdown():
            print ("Shutting Down")
            break

    print("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()

if __name__ == '__main__':
    p = JumpLegController(robotName)

    try:
        talker(p)
    except ros.ROSInterruptException:
        # these plots are for simulated robot TODO fix these
        #  plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
        #               p.tau_ffwd_log, p.joint_names)
        #  plotJoint('torque', 1, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
        #            p.tau_ffwd_log, p.joint_names)
         plt.show(block=True)
    
        
