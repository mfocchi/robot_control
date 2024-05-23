from __future__ import print_function

import rospy as ros
import numpy as np
import rospkg
import pinocchio as pin
# utility functions
from base_controllers.utils.math_tools import *
from termcolor import colored
from base_controllers.quadruped_controller import Controller
import base_controllers.params as conf
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
import os, sys

class QuadrupedJumpController(Controller):
    def __init__(self, robot_name="hyq", launch_file=None):
        super(QuadrupedJumpController, self).__init__(robot_name, launch_file)
        self.DEBUG = True

    #####################
    # OVERRIDEN METHODS OF BASECONTROLLER#
    #####################
    # initVars
    # logData
    # startupProcedure

    def computeHeuristicSolutionBezierLinear(self, com_0, com_lo, comd_lo, T_th):
        self.bezier_weights_lin = np.zeros([3, 4])
        comd_0 = np.zeros(3)
        self.bezier_weights_lin = np.zeros([3, 4])
        self.bezier_weights_lin[:, 0] = com_0
        self.bezier_weights_lin[:, 1] = (comd_0*(T_th/3.)) + com_0
        self.bezier_weights_lin[:, 2] = com_lo - (comd_lo*(T_th/3.))
        self.bezier_weights_lin[:, 3] = com_lo

    def computeHeuristicSolutionBezierAngular(self, eul_0, eul_lo, euld_lo, T_th):
        self.bezier_weights_ang = np.zeros([3, 4])
        euld_0 = np.zeros(3)
        self.bezier_weights_ang = np.zeros([3, 4])
        self.bezier_weights_ang[:, 0] = eul_0
        self.bezier_weights_ang[:, 1] = (euld_0*(T_th/3.)) + eul_0
        self.bezier_weights_ang[:, 2] = eul_lo - (euld_lo*(T_th/3.))
        self.bezier_weights_ang[:, 3] = eul_lo


    def evalBezier(self, t_, T_th):
        com = np.array(self.Bezier3(self.bezier_weights_lin,t_,T_th))
        comd = np.array(self.Bezier2(self.bezier_weights_lin,t_,T_th))
        comdd = np.array(self.Bezier1(self.bezier_weights_lin, t_, T_th))
        eul = np.array(self.Bezier3(self.bezier_weights_ang,t_,T_th))
        euld = np.array(self.Bezier2(self.bezier_weights_ang,t_,T_th))
        euldd = np.array(self.Bezier1(self.bezier_weights_ang, t_, T_th))

        if self.DEBUG:
            freq = 0.5
            amp_lin = np.array([0., 0.,0.05])
            amp_ang = np.array([0., 0.0, 0])
            com = self.initial_com + np.multiply(amp_lin, np.sin(2*np.pi*freq * self.time ))
            comd = np.multiply(2*np.pi*freq*amp_lin, np.cos(2*np.pi*freq * self.time))
            comdd = np.multiply(np.power(2*np.pi*freq*amp_lin, 2), -np.sin(2*np.pi*freq * self.time))
            eul = np.array([0., 0.0, 0]) + np.multiply(amp_ang, np.sin(2 * np.pi * freq * self.time))
            euld = np.multiply(2 * np.pi * freq * amp_ang, np.cos(2 * np.pi * freq * self.time))
            euldd = np.multiply(np.power(2 * np.pi * freq * amp_ang, 2), -np.sin(2 * np.pi * freq * self.time))

        Jb = p.computeJcb(self.W_contacts, com, self.stance_legs)


        W_des_basePose = np.empty(6)
        W_des_basePose[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = com
        W_des_basePose[self.u.sp_crd['AX']:self.u.sp_crd['AX'] + 3] = eul

        W_des_baseTwist = np.empty(6)
        W_des_baseTwist[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = comd
        Jomega = self.math_utils.Tomega(eul)
        W_des_baseTwist[self.u.sp_crd['AX']:self.u.sp_crd['AX'] + 3] = self.math_utils.Tomega(eul).dot(euld)

        W_des_baseAcc = np.empty(6)
        W_des_baseAcc[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = comdd
        # compute w_omega_dot =  Jomega* euler_rates_dot + Jomega_dot*euler_rates (Jomega already computed, see above)
        Jomega_dot = self.math_utils.Tomega_dot(eul, euld)
        W_des_baseAcc[self.u.sp_crd['AX']:self.u.sp_crd['AX'] + 3] = Jomega @ euldd + \
                                                            Jomega_dot @ euld

        #map base twist into feet relative vel (wrt com/base)
        W_feetRelVelDes = -Jb.dot(W_des_baseTwist)
        w_R_b_des = self.math_utils.eul2Rot(eul)

        qd_des = np.zeros(12)
        q_des = np.zeros(12)
        fbjoints = pin.neutral(self.robot.model)
        w_J = self.u.listOfArrays(4, np.zeros((3, 3)))
        #integrate relative Velocity
        for leg in range(self.robot.nee):
            # with this you do not have proper tracking of com and trunk orientation, I think there is a bug in the ik
            #self.W_feetRelPosDes[leg] += W_feetRelVelDes[3 * leg:3 * (leg+1)]*self.dt
            # this has better tracking
            self.W_feetRelPosDes[leg] = self.W_contacts[leg] -com
            #now we can do Ik
            q_des[3 * leg:3 * (leg+1)], isFeasible = self.IK.ik_leg(w_R_b_des.T.dot(self.W_feetRelPosDes[leg]),
                                                   self.leg_names[leg],
                                                   self.legConfig[self.leg_names[leg]][0],
                                                   self.legConfig[self.leg_names[leg]][1])
            #for joint velocity we need to recompute the Jacobian (in the world frame) for the computed joint position q_des
            # you need to fill in also the floating base part
            quat_des = pin.Quaternion(w_R_b_des)
            fbjoints[:3] = com
            fbjoints[3:7] = np.array([quat_des.x, quat_des.y, quat_des.z, quat_des.w])
            fbjoints[7:] = q_des

            pin.forwardKinematics(self.des_robot.model, self.des_robot.data, fbjoints, np.zeros(self.des_robot.model.nv),   np.zeros(self.des_robot.model.nv))
            pin.computeJointJacobians(self.des_robot.model, self.des_robot.data)
            pin.computeFrameJacobian(self.des_robot.model, self.des_robot.data,fbjoints, p.des_robot.model.getFrameId(self.ee_frames[leg]))
            w_J[leg] = pin.getFrameJacobian(self.des_robot.model, self.des_robot.data,
                                     p.des_robot.model.getFrameId(self.ee_frames[leg]),
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 6 + leg * 3:6 + leg * 3 + 3]
            #compute joint variables
            qd_des[3 * leg:3 *(leg+1)] = np.linalg.pinv(w_J[leg]).dot(W_feetRelVelDes[3 * leg:3 *(leg+1)])

        print(q_des)
        tau_ffwd = self.WBC(W_des_basePose, W_des_baseTwist, W_des_baseAcc, comControlled = False, type='projection', stance_legs=self.stance_legs)
        #OLD
        #tau_ffwd = self.gravityCompensation()

        #check unloading of front legs
        grf_lf = self.u.getLegJointState(0, self.grForcesW)
        grf_rf = self.u.getLegJointState(2, self.grForcesW)
        #if unloaded raise front legs and compute Jb for a subset of lefs
        if not self.rearingFlag  and (grf_lf[2] < 5.) and (grf_rf[2] < 5.): # LF RF
            self.rearingFlag = True
            self.stance_legs = [False, True, False, True]
            print(colored(f"rearing front legs at time {self.time}", "red"))

        #overwrite front legs
        if self.rearingFlag:
            for leg in range(self.robot.nee):
                if self.stance_legs[leg]:
                    q_des[3 * leg:3 * (leg+1)] = p.qj_0[3 * leg:3 * (leg+1)]
                    qd_des[3 * leg:3 * (leg+1)] = np.zeros(3)
                    #the small torques to compensate self weight of the legs in the air have been already computed by WBC
        return q_des, qd_des, tau_ffwd, W_des_basePose, W_des_baseTwist

    def plotTrajectoryBezier(self):
        # plot com intermediate positions
        for blob in range(len(self.intermediate_com_position)):
            self.ros_pub.add_marker(self.intermediate_com_position[blob], color=[
                blob * 1. / self.number_of_blobs, blob * 1. / self.number_of_blobs, blob * 1. / self.number_of_blobs],
                                 radius=0.05)

    def computeTrajectoryBezier(self, T_th):
        self.number_of_blobs = 30
        t = np.linspace(0, T_th, self.number_of_blobs)
        self.intermediate_com_position = []
        for blob in range(self.number_of_blobs):
            self.intermediate_com_position.append(
                self.Bezier3(self.bezier_weights_lin, t[blob], T_th))

    def bernstein_pol(self,k,n,x):
        v = (np.math.factorial(n)/(np.math.factorial(k)*(np.math.factorial(n-k))))*np.power(x,k)*np.power(1-x,n-k)
        return v

    def Bezier3(self,w,t_ex,t_th):
        t = t_ex/t_th
        return  w[:,0]*self.bernstein_pol(0,3,t)+\
                w[:,1]*self.bernstein_pol(1,3,t)+\
                w[:,2]*self.bernstein_pol(2,3,t)+\
                w[:,3]*self.bernstein_pol(3,3,t)

    def Bezier2(self,w,t_ex,t_th):
        t = t_ex/t_th
        return ((w[:,1]-w[:,0]))*(3/t_th)*self.bernstein_pol(0,2,t)+\
            ((w[:,2]-w[:,1]))*(3/t_th)*self.bernstein_pol(1,2,t)+\
            ((w[:,3]-w[:,2]))*(3/t_th)*self.bernstein_pol(2,2,t)

    def Bezier1(self, w, t_ex, t_th):
        t = t_ex / t_th
        return ((w[:, 2] - 2 * w[:, 2] + w[:, 0])) * (6 / np.power(t_th, 2)) * self.bernstein_pol(0, 1, t) + \
               ((w[:, 3] - 2 * w[:, 2] + w[:, 1])) * (6 / np.power(t_th, 2)) * self.bernstein_pol(1, 1, t)

    def computeJcb(self, feetW, com, stance_legs):
        Jb = np.zeros([3 * self.robot.nee, 6])  # Newton-Euler matrix
        for leg in range(self.robot.nee):
            start_row = 3 * leg
            end_row = 3 * (leg + 1)
            if stance_legs[leg]:
                # ---> linear part
                # identity matrix (I avoid to rewrite zeros)
                Jb[start_row:end_row, :3] = np.identity(3)
                # ---> angular part
                # all in a function
                Jb[start_row:end_row, 3:]= -pin.skew(feetW[leg] - com)
            else:
                Jb[start_row:end_row, 3:] = np.zeros(3)
                Jb[start_row:end_row, :3] = np.zeros(3)
        return Jb

if __name__ == '__main__':
    p = QuadrupedJumpController('go1')
    world_name = 'fast.world'
    use_gui = True
    try:
        #p.startController(world_name='slow.world')
        p.startController(world_name=world_name,
                          use_ground_truth_pose=True,
                          use_ground_truth_contacts=False,
                          additional_args=['gui:='+str(use_gui),
                                           'go0_conf:=standDown'])
        # initialize data stucture to use them with desired values
        p.des_robot = RobotWrapper.BuildFromURDF(
            os.environ.get('LOCOSIM_DIR') + "/robot_urdf/generated_urdf/" + p.robot_name + ".urdf")
        p.startupProcedure()


        #initial pose
        #linear
        com_lo = np.array([0.2,0.,0.4])
        comd_lo = np.array([0.8, 0.,1.3])
        com_0 = p.basePoseW[:3]
        #angular
        eul_0 = p.basePoseW[3:]
        eul_lo = np.array([0., -0.2, 0.])
        euld_lo = np.array([0., 0.1, 0.])
        if p.DEBUG:
            p.initial_com = np.copy(com_0)
            print(f"Initial Com Position is {p.initial_com}")
            print(f"Initial Joint Position is {p.q}")
            print(f"Initial Joint torques {p.tau_ffwd}")
            p.T_th = 5.
        else:
            p.T_th = 0.6
        p.computeHeuristicSolutionBezierLinear(com_0, com_lo, comd_lo,p.T_th)
        p.computeHeuristicSolutionBezierAngular(eul_0, eul_lo, euld_lo, p.T_th)

        #this is for visualization
        p.computeTrajectoryBezier(p.T_th)
        p.time = 0. # reset time for logging
        startTrust  = 0.
        p.trustPhaseFlag = True
        p.rearingFlag = False

        p.stance_legs = [True, True, True, True]
        #print(p.computeJcb(p.W_contacts, com_0))
        #reset integration of feet
        p.W_feetRelPosDes = np.copy(p.W_contacts - com_0)
        #p.setSimSpeed(dt_sim=0.001, max_update_rate=200, iters=1500)
        #p.pid.setPDs(0.0, 0.0, 0.0)

        while not ros.is_shutdown():
            p.updateKinematics()

            if (p.time > startTrust):

                 # compute joint reference
                if (p.trustPhaseFlag):
                    t = p.time - startTrust
                    p.q_des, p.qd_des, p.tau_ffwd, p.basePoseW_des, p.baseTwistW_des = p.evalBezier(t, p.T_th)

                    if p.time >= (startTrust + p.T_th):
                        p.trustPhaseFlag = False
                        #we se this here to have enough retraction (important)
                        p.q_des = p.qj_0
                        p.qd_des = np.zeros(12)
                        p.tau_ffwd = np.zeros(12)
                        print(colored(f"thrust completed! at time {p.time}","red"))
                        if p.DEBUG:
                            break
            p.plotTrajectoryBezier()
            p.visualizeContacts()
            p.logData()
            p.send_command(p.q_des, p.qd_des, p.tau_ffwd)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        
    from base_controllers.utils.common_functions import *

    if conf.plotting:
        plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, sharex=True, sharey=False,
                  start=0, end=-1)
        plotJoint('velocity', time_log=p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, sharex=True, sharey=False,
                  start=0, end=-1)
        plotJoint('torque', time_log=p.time_log, tau_log=p.tau_ffwd_log,sharex=True, sharey=False, start=0, end=-1)
        plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log, Twist_log=p.baseTwistW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
