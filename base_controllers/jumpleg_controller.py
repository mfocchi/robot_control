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
            self.apply_body_wrench(body_name=self.robot_name+"::base_link", reference_frame=reference_frame, reference_point=reference_point , wrench=wrench, duration=ros.Duration(10))
        except:
            pass

    def updateKinematicsDynamics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        self.q_fixed = np.hstack((np.zeros(3), self.q[3:]))
        self.qd_fixed = np.hstack((np.zeros(3), self.qd[3:]))


        self.robot.computeAllTerms(self.q_fixed , self.qd_fixed)
        # joint space inertia matrix                
        self.M = self.robot.mass(self.q_fixed)
        # bias terms                
        self.h = self.robot.nle(self.q_fixed , self.qd_fixed)
        #gravity terms                
        self.g = self.robot.gravity(self.q_fixed)
        #compute ee position  in the world frame  
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.x_ee = self.robot.framePlacement(self.q_fixed, self.robot.model.getFrameId(frame_name)).translation


        # TODO compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]
        self.dJdq = self.robot.frameClassicAcceleration(self.q, self.qd, None,  self.robot.model.getFrameId(frame_name), False).linear

        # TODO fix this
        # Moore-penrose pseudoinverse of A = J^T => (A^TA)^-1 * A^T
        #JTpinv = np.linalg.inv(self.J.dot(self.J.T)).dot(self.J)
        M_inv = np.linalg.inv(self.M[3:, 3:])
        # lambda_  inertia mapped at the end effector
        lambda_ = np.linalg.inv(self.J.dot(M_inv).dot(self.J.T))
        JTpinv = lambda_.dot(self.J).dot(M_inv)
        # null-space projector of J^T#
        self.N = (np.eye(3) - (self.J.T).dot(JTpinv))

        # compute com variables accordint to a frame located at the foot
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data)

        # only for real robot
        #self.com = -self.x_ee + robotComB
        #self.comd = -self.J.dot(self.qd[3:])
        #self.comdd = -self.J.dot(self.qdd)

        # from ground truth
        self.com = self.q[:3] + robotComB
        self.comd = self.qd[:3]

        #compute contact forces
        self.estimateContactForces()

    def estimateContactForces(self):
        self.contactForceW = np.linalg.inv(self.J.T).dot( (self.h-self.tau)[3:] )

    def initVars(self):
        super().initVars()
        self.qdd_des =  np.zeros(self.robot.na)
        # init new logged vars here
        self.com_log =  np.empty((3, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.comd_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        #self.comdd_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                self.com_log[:, self.log_counter] = self.com
                self.comd_log[:, self.log_counter] = self.comd
                #self.comdd_log[:, self.log_counter] = self.comdd
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
            pd  =  np.multiply( conf.robot_params[p.robot_name]['kp'][3:], np.subtract(p.q_des[3:],   p.q[3:]) ) \
                          -np.multiply(conf.robot_params[p.robot_name]['kd'][3:], p.qd[3:])
            p.tau_ffwd[3:] = pd -p.J.T.dot( p.g[:3])  # gravity compensation
            # remove virtual joint gravity compensation previously set
            p.tau_ffwd[2] = 0.

            # TODO fix this
            # p.qdd_ref = p.qdd_des[3:] + pd
            # p.tau_ffwd[3:] = np.linalg.pinv(p.N) @ p.N @ (p.M[3:, 3:] @ p.qdd_ref + p.h[3:])
        else:
            # trick to have zero error  on Z direction at startup
            p.tau_ffwd[2] = p.g[2]

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

        if (p.freezeBase and p.time > 2.0):
            print("releasing base")
            # p.pid.setPDjoint(0, 0.0,0.0,0.0)
            # p.pid.setPDjoint(1, 0.0,0.0,0.0)
            # p.pid.setPDjoint(2, 0.0,0.0,0.0)
            # not using the ros impedance controller PD
            p.pid.setPDs(0.,0.,0.)
            p.freezeBase = False


        # disturbance force
        # if (p.time > 3.0 and p.EXTERNAL_FORCE):
        #     p.applyForce()
        #     p.EXTERNAL_FORCE = False

        # plot end-effector and contact force
        p.ros_pub.add_arrow(p.base_offset + p.q[:3] + p.x_ee, p.contactForceW / (10 * p.robot.robot_mass), "green")
        p.ros_pub.add_marker( p.base_offset + p.q[:3] + p.x_ee, radius=0.05)
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
        #plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, None, None, p.joint_names)
        plotCoMLinear('position',1, p.time_log, None, p.com_log, None, p.comd_log)
        plotCoMLinear('position', 2,p.time_log, None, p.contactForceW_log)
        plt.show(block=True)
    
        
