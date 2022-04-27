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
import os

from base_controller_fixed import BaseControllerFixed
import  params as conf
robotName = "jumpleg"

class JumpLegController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)

        self.freezeBaseFlag = False
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

    def freezeBase(self, flag):

        if (self.freezeBaseFlag) and (not flag):
            self.freezeBaseFlag = flag
            print("releasing base")
            p.tau_ffwd[2] = 0.
            self.pid.setPDjoint(0, 0., 0., 0.)
            self.pid.setPDjoint(1, 0., 0., 0.)
            self.pid.setPDjoint(2, 0., 0., 0.)

            # debug
            #self.q_des[2] = 0.3

        if (not self.freezeBaseFlag) and (flag):
            self.freezeBaseFlag = flag
            print("freezing base")
            self.q_des = np.copy(p.q_des_q0)
            self.tau_ffwd = np.zeros(self.robot.na)
            self.tau_ffwd[2] = p.g[2]
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'],
                                 np.zeros(self.robot.na))


    def invKinFoot(self, ee_pos_des_BF, frame_name, q0_leg=np.zeros(3), verbose = False):
        # Error initialization
        niter = 0
        # Recursion parameters
        epsilon = 1e-06  # Tolerance
        # alpha = 0.1
        alpha = 1  # Step size
        lambda_ = 0.0000001  # Damping coefficient for pseudo-inverse
        max_iter = 200  # Maximum number of iterations
        out_of_workspace = False

        # Inverse kinematics with line search
        while True:
            # compute foot position
            q0 = np.hstack((np.zeros(3), q0_leg))
            self.robot.computeAllTerms(q0, np.zeros(6))
            ee_pos0 = self.robot.framePlacement(q0, self.robot.model.getFrameId(frame_name)).translation
            # get the square matrix jacobian that is smaller (3x6)
            J_ee = self.robot.frameJacobian(q0, self.robot.model.getFrameId(frame_name), True,
                                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]

            # computed error wrt the des cartesian position
            e_bar = ee_pos_des_BF - ee_pos0
            Jpinv = (np.linalg.inv(J_ee.T.dot(J_ee) + lambda_ * np.identity(J_ee.shape[1]))).dot(J_ee.T)
            grad = J_ee.T.dot(e_bar)
            dq = Jpinv.dot(e_bar)
            if np.linalg.norm(grad) < epsilon:
                IKsuccess = True
                if verbose:
                    print("IK Convergence achieved!, norm(grad) :", np.linalg.norm(grad))
                    print("Inverse kinematics solved in {} iterations".format(niter))
                    if np.linalg.norm(e_bar) > 0.1:
                        print("THE END EFFECTOR POSITION IS OUT OF THE WORKSPACE, norm(error) :", np.linalg.norm(e_bar))
                        out_of_workspace = True
                break

            if niter >= max_iter:
                if verbose:
                    print(
                        "\n Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                        np.linalg.norm(e_bar))
                IKsuccess = False
                break

            q0_leg += alpha*dq
            niter += 1
        return q0_leg, IKsuccess, out_of_workspace


    def thirdOrderPolynomialTrajectory(self, tf, q0, qf):
        # Matrix used to solve the linear system of equations for the polynomial trajectory
        polyMatrix = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [1, tf, np.power(tf, 2), np.power(tf, 3)],
                               [0, 1, 2 * tf, 3 * np.power(tf, 2)]])

        polyVector = np.array([q0, 0,  qf, 0])
        matrix_inv = np.linalg.inv(polyMatrix)
        polyCoeff = matrix_inv.dot(polyVector)

        return polyCoeff

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

def talker(p):

    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()     
    p.startupProcedure()
    ros.sleep(1.0)
    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    p.q_des = np.copy(p.q_des_q0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    # compute coeff first time
    # initial com posiiton
    T_f = 0.3
    com_0 = np.array([0., 0., 0.25])
    com_f = np.array([0.1, 0., 0.25])
    comd_f = np.array([0.1, 0., 0.5])

    q_0_leg, ik_success, out_of_workspace = p.invKinFoot(-com_0, conf.robot_params[p.robot_name]['ee_frame'], p.q_des_q0[3:].copy(), verbose = False)
    q_f_leg, ik_success, out_of_workspace = p.invKinFoot(-com_f, conf.robot_params[p.robot_name]['ee_frame'], p.q_des_q0[3:].copy(), verbose = False)
    a = np.empty((3, 4))
    for i in range(3):
         a[i,:] = p.thirdOrderPolynomialTrajectory(T_f, q_0_leg[i], q_f_leg[i])


    # here the RL loop...

    p.time = 0
    startTrust = 2.0
    p.updateKinematicsDynamics()
    p.freezeBase(True)
    counter = 0
    #control loop
    while p.time < (startTrust + 2.):
        #update the kinematics
        p.updateKinematicsDynamics()
        if (p.time > startTrust) and (p.time < startTrust + T_f):
            t = p.time - startTrust
            for i in range(3):
                p.q_des[3+i] = a[i, 0] + a[i,1] * t + a[i,2] * pow(t, 2) + a[i,3] * pow(t, 3)
                p.qd_des[3+i] = a[i, 1] + 2 * a[i,2] * t + 3 * a[i,3] * pow(t, 2)
            #     #qdd = 2 * a[2] + 6 * a[3] * time + 12 * a[4] * time ** 2 + 20 * a[5] * time ** 3

        if (p.time > startTrust):
            # pd  =  np.multiply( conf.robot_params[p.robot_name]['kp'][3:], np.subtract(p.q_des[3:],   p.q[3:]) ) \
            #               +np.multiply(conf.robot_params[p.robot_name]['kd'][3:], np.subtract(p.qd_des[3:],   p.qd[3:]))
            p.tau_ffwd[3:] = -p.J.T.dot( p.g[:3])  # gravity compensation

            # TODO fix this
            # p.qdd_ref = p.qdd_des[3:] + pd
            # p.tau_ffwd[3:] = np.linalg.pinv(p.N) @ p.N @ (p.M[3:, 3:] @ p.qdd_ref + p.h[3:])

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

        if ( p.time > startTrust):
            p.freezeBase(False)

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

    plotCoMLinear('position', 1, p.time_log, None, p.com_log, None, p.comd_log)
    plotCoMLinear('position', 2, p.time_log, None, p.contactForceW_log)
    plotJoint('position', 3, p.time_log, p.q_log, p.q_des_log,  p.qd_log, p.qd_des_log,   joint_names=conf.robot_params[p.robot_name]['joint_names'])
    plt.show(block=True)



if __name__ == '__main__':
    p = JumpLegController(robotName)

    try:
        talker(p)
    except ros.ROSInterruptException:
        print("PLOTTING")
        #


        
