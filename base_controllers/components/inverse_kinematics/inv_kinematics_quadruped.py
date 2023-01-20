import time

import numpy as np
import pinocchio as pin
from base_controllers.utils.utils import Utils
from base_controllers.utils.common_functions import getRobotModel

import scipy.optimize
import scipy.linalg

# these macro are useful for choosing the solution (four are possible)
# meanings:
#    HIP is DOWN when if -pi/2 < HAA < pi/2, is UP otherwise
#    KNEE is INWARD if it point toward the central part of the robot, is OUTWARD otherwise
# normal solution for X-shaped robots (like solo):
#    lf = {HIP_DOWN, KNEE_INWARD}
#    lh = {HIP_DOWN, KNEE_INWARD}
#    rf = {HIP_DOWN, KNEE_INWARD}
#    rh = {HIP_DOWN, KNEE_INWARD}
# normal solution for CC-shaped robots (like aliengo):
#    lf = {HIP_DOWN, KNEE_INWARD}
#    lh = {HIP_DOWN, KNEE_OUTWARD}
#    rf = {HIP_DOWN, KNEE_INWARD}
#    rh = {HIP_DOWN, KNEE_OUTWARD}


HIP_UP = 'HipUp'
HIP_DOWN = 'HipDown'
KNEE_INWARD = 'KneeInward'
KNEE_OUTWARD = 'KneeOutward'


class InverseKinematics:
    '''
        This clss computes the inverse kinematic of a leg of a quadruped having joints {HAA, HFE, KFE}
        The class modifies data field of the RobotWrapper (because it calls forwardKinematics and updateFramePlacements)
        Requires Pinocchio model with frames [leg]_hip, [leg]_upperleg, [leg]_loweleg, [leg]_foot
        for leg in ['lf', 'rf', 'lh', 'rh'] for computing the relative distance between reference frames
    '''

    def __init__(self, robot):
        self.robot = robot
        self.u = Utils()

        pin.forwardKinematics(self.robot.model, self.robot.data, pin.neutral(self.robot.model))
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        # indeces
        self._hip_id_dict = {}
        self._upperleg_id_dict = {}
        self._lowerleg_id_dict = {}
        self._foot_id_dict = {}

        legs = ['lf', 'rf', 'lh', 'rh']

        for leg in legs:
            self._hip_id_dict[leg] = self.robot.model.getFrameId(leg + '_hip')
            self._upperleg_id_dict[leg] = self.robot.model.getFrameId(leg + '_upperleg')
            self._lowerleg_id_dict[leg] = self.robot.model.getFrameId(leg + '_lowerleg')
            self._foot_id_dict[leg] = self.robot.model.getFrameId(leg + '_foot')

        # Compute the relative distance from the reference frames
        self.measures = {}
        self.upper_limits = {}
        self.lower_limits = {}

        for leg in legs:
            self.measures[leg] = {}

            self.measures[leg]['base_2_HAA_x'] = self.robot.data.oMf[self._hip_id_dict[leg]].translation[0]
            self.measures[leg]['base_2_HAA_y'] = self.robot.data.oMf[self._hip_id_dict[leg]].translation[1]

            self.measures[leg]['HAA_2_HFE_y'] = self.robot.data.oMf[self._upperleg_id_dict[leg]].translation[1] - \
                                                self.robot.data.oMf[self._hip_id_dict[leg]].translation[1]

            self.measures[leg]['HFE_2_KFE_y'] = self.robot.data.oMf[self._lowerleg_id_dict[leg]].translation[1] - \
                                                self.robot.data.oMf[self._upperleg_id_dict[leg]].translation[1]

            self.measures[leg]['HFE_2_KFE_z'] = self.robot.data.oMf[self._lowerleg_id_dict[leg]].translation[2] - \
                                                self.robot.data.oMf[self._upperleg_id_dict[leg]].translation[2]

            self.measures[leg]['KFE_2_FOOT_y'] = self.robot.data.oMf[self._foot_id_dict[leg]].translation[1] - \
                                                 self.robot.data.oMf[self._lowerleg_id_dict[leg]].translation[1]

            self.measures[leg]['KFE_2_FOOT_z'] = self.robot.data.oMf[self._foot_id_dict[leg]].translation[2] - \
                                                 self.robot.data.oMf[self._lowerleg_id_dict[leg]].translation[2]

            self.measures[leg]['HAA_2_FOOT_y'] = self.robot.data.oMf[self._foot_id_dict[leg]].translation[1] - \
                                                 self.robot.data.oMf[self._hip_id_dict[leg]].translation[1]

            self.upper_limits[leg] = self.u.getLegJointState(leg.upper(), self.robot.model.upperPositionLimit[7:])
            self.lower_limits[leg] = self.u.getLegJointState(leg.upper(), self.robot.model.lowerPositionLimit[7:])

        self._leg_joints={}

        for i in range(4):
            self._leg_joints[legs[i]] = range(6 + self.u.mapIndexToRos(i) * 3, 6 + self.u.mapIndexToRos(i) * 3 + 3)

        self._q_neutral = pin.neutral(self.robot.model)

        self.KneeInward = KNEE_INWARD
        self.KneeOutward = KNEE_OUTWARD

    def ik_leg(self, foot_pos, foot_idx, hip=HIP_DOWN, knee=KNEE_INWARD, verbose = False):
        # import warnings
        # warnings.filterwarnings("error")
        q = np.zeros(3)
        isFeasible = False
        leg = self.robot.model.frames[foot_idx].name[:2]

        HAA_foot_x = foot_pos[0] - self.measures[leg]['base_2_HAA_x']
        HAA_foot_y = foot_pos[1] - self.measures[leg]['base_2_HAA_y']

        sq = foot_pos[2] ** 2 + HAA_foot_y ** 2 - self.measures[leg]['HAA_2_FOOT_y'] ** 2
        # if sq < 0.:
        #     print('foot_idx', foot_idx)
        #     print('foot_pos', foot_pos)
        #     print('HAA_foot_x', HAA_foot_x)
        #     print('HAA_foot_y', HAA_foot_y)
        #     print("self.measures[leg]['HAA_2_FOOT_y']", self.measures[leg]['HAA_2_FOOT_y'])
        #     print('sq', sq)

        if sq < 0:
            if verbose:
                print('foot is higher that hip!')
            return q, isFeasible
        HAA_foot_z = np.sqrt(sq)


        if hip == HIP_DOWN:
            HAA_foot_z *= -1
        # Verify if the foot is inside the work space of the leg (WS1)
        if (HAA_foot_x ** 2 + HAA_foot_z ** 2) > (
                self.measures[leg]['HFE_2_KFE_z'] + self.measures[leg]['KFE_2_FOOT_z']) ** 2:
            if verbose:
                print('Foot position is out of the workspace')
            return q, isFeasible

        #################
        # Compute qHAA #
        #################

        ratio = 1 / (self.measures[leg]['HAA_2_FOOT_y'] ** 2 + HAA_foot_z ** 2)

        cos_qHAA = ratio * (self.measures[leg]['HAA_2_FOOT_y'] * HAA_foot_y + HAA_foot_z * foot_pos[2])
        cos_qHAA = np.round(cos_qHAA, 10)
        sin_qHAA = ratio * (-HAA_foot_z * HAA_foot_y + self.measures[leg]['HAA_2_FOOT_y'] * foot_pos[2])
        sin_qHAA = np.round(sin_qHAA, 10)

        qHAA = np.arctan2(sin_qHAA, cos_qHAA)

        #################
        # Compute qKFE #
        #################

        num = HAA_foot_x ** 2 + HAA_foot_z ** 2 - self.measures[leg]['HFE_2_KFE_z'] ** 2 - self.measures[leg][
            'KFE_2_FOOT_z'] ** 2
        den = 2 * self.measures[leg]['HFE_2_KFE_z'] * self.measures[leg]['KFE_2_FOOT_z']

        cos_qKFE = num / den
        cos_qKFE = np.round(cos_qKFE, 10)

        sin_qKFE = np.sqrt(1 - cos_qKFE ** 2)
        sin_qKFE = np.round(sin_qKFE, 10)

        if foot_idx == self._foot_id_dict['lf'] or foot_idx == self._foot_id_dict['rf']:
            if knee == KNEE_INWARD:
                sin_qKFE *= -1
        else:
            if knee == KNEE_OUTWARD:
                sin_qKFE *= -1
        qKFE = np.arctan2(sin_qKFE, cos_qKFE)

        #################
        # Compute qHFE #
        #################

        c_num0 = - self.measures[leg]['KFE_2_FOOT_z'] * sin_qKFE * HAA_foot_x
        c_num1 = -(self.measures[leg]['HFE_2_KFE_z'] + self.measures[leg]['KFE_2_FOOT_z'] * cos_qKFE) * HAA_foot_z
        den = -(self.measures[leg]['KFE_2_FOOT_z'] ** 2 + self.measures[leg]['HFE_2_KFE_z'] ** 2 + 2 *
                self.measures[leg]['KFE_2_FOOT_z'] * self.measures[leg]['HFE_2_KFE_z'] * cos_qKFE)

        cos_qHFE = (c_num0 + c_num1) / den
        cos_qHFE = np.round(cos_qHFE, 10)

        s_num0 = -(self.measures[leg]['HFE_2_KFE_z'] + self.measures[leg]['KFE_2_FOOT_z'] * cos_qKFE) * HAA_foot_x
        s_num1 = self.measures[leg]['KFE_2_FOOT_z'] * sin_qKFE * HAA_foot_z

        sin_qHFE = (s_num0 + s_num1) / den
        sin_qHFE = np.round(sin_qHFE, 10)

        qHFE = np.arctan2(sin_qHFE, cos_qHFE)

        # Save the solution
        q[0] = qHAA
        q[1] = qHFE
        q[2] = qKFE

        cond_upper = q < self.upper_limits[leg]
        cond_lower = q > self.lower_limits[leg]

        if cond_upper.all() and cond_lower.all():
            isFeasible = True
        else:
            outROMidx = np.hstack([np.where(cond_upper == False)[0],np.where(cond_lower == False)[0]])
            if verbose:
                print('IK produced a solution for leg '+ leg+ ' out of ROM for joint(s) '+str(outROMidx))

        return q, isFeasible


    def diff_ik_leg(self, q_des, B_v_foot, foot_idx, damp, update=True):
        leg = self.robot.model.frames[foot_idx].name[:2]
        self._q_neutral[7:] = self.u.mapToRos(q_des)
        B_J = self.robot.frameJacobian(self._q_neutral, foot_idx, update, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, self._leg_joints[leg]]
        for i in range(3):
            B_J[i,i] += damp
        qd_leg =  np.linalg.inv(B_J) @ B_v_foot
        return qd_leg




class InverseKinematicsOptimization:
    def __init__(self, robot):
        self.robot = robot
        self.u = Utils()
        self._q_floating = pin.neutral(self.robot.model)
        self.frame_names = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

        self.q_range = {}
        self.J_range = {}
        self.frame_id = {}
        self.bounds = {}

        for leg, name in enumerate(self.frame_names):
            self.q_range[name] = range(7 + self.u.mapIndexToRos(leg) * 3, 7 + self.u.mapIndexToRos(leg) * 3 + 3)
            self.J_range[name] = range(6 + self.u.mapIndexToRos(leg) * 3, 6 + self.u.mapIndexToRos(leg) * 3 + 3)
            self.frame_id[name] = self.robot.model.getFrameId(name)
            self.bounds[name] = scipy.optimize.Bounds(self.robot.model.lowerPositionLimit[self.q_range[name]],
                                                      self.robot.model.upperPositionLimit[self.q_range[name]])

    def leg_forwardKinematcs(self, q_leg, frame_name, ret_J):
        self._q_floating[7:] = 0.
        self._q_floating[self.q_range[frame_name]] = q_leg

        pin.forwardKinematics(self.robot.model, self.robot.data, self._q_floating)
        pin.framesForwardKinematics(self.robot.model, self.robot.data, self._q_floating)

        pos = self.robot.data.oMf[self.frame_id[frame_name]].translation
        if ret_J:
            J = pin.computeFrameJacobian(self.robot.model, self.robot.data, self._q_floating, self.frame_id[frame_name],
                                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, self.J_range[frame_name]]
            return pos, J

        return pos

    def cost(self, q_leg, pos_des, frame_name):
        pos = self.leg_forwardKinematcs(q_leg, frame_name, ret_J=False)
        err = pos-pos_des#).reshape(3,1)

        return 0.5*err.T@err

    def jac(self, q_leg, pos_des, frame_name):
        pos, J = self.leg_forwardKinematcs(q_leg, frame_name, ret_J=True)
        err = pos-pos_des

        return err.T @ J



    def solve(self, pos_des, frame_name, q0=np.zeros(3)):
        bounds = self.bounds[frame_name]
        sol = scipy.optimize.minimize(self.cost,
                                      args=(pos_des, frame_name),
                                      x0=q0,
                                      method='SLSQP',
                                      jac=self.jac,
                                      bounds=self.bounds[frame_name])
        return sol







if __name__ == '__main__':
    from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import  robotKinematics
    # robot_name = 'solo'
    # qj = np.array([ 0.2,  np.pi / 4, -np.pi / 2,   # lf
    #                 0.2, -np.pi / 4,  np.pi / 2,   # lh
    #                -0.2,  np.pi / 4, -np.pi / 2,   # rf
    #                -0.2, -np.pi / 4,  np.pi / 2])  # rh
    # hip = [HIP_DOWN]*4
    # knee = [KNEE_INWARD] * 4

    robot_name = 'go1'
    qj = np.array([ 0.2, 0.7, -1.4,   # lf
                   0.2, 0.7, -1.4,    # lh
                   -0.2, 0.7, -1.4,   # rf
                   -0.2, 0.7, -1.4])  # rh
    hip = [HIP_DOWN] * 4
    knee = [KNEE_INWARD] * 2 + [KNEE_OUTWARD] * 2

    robot = getRobotModel(robot_name, generate_urdf=True)
    q = pin.neutral(robot.model)
    q[7:12 + 7] = qj

    IK = InverseKinematics(robot)
    IKOpt = InverseKinematicsOptimization(robot)
    RK = robotKinematics(robot, ['lf_foot', 'rf_foot','lh_foot', 'rh_foot'])
    pin.forwardKinematics(robot.model, robot.data, q)
    pin.updateFramePlacements(robot.model, robot.data)
    legs = ['lf', 'rf','lh', 'rh']
    feet_id = [robot.model.getFrameId(leg + '_foot') for leg in legs]
    feet_pos = [robot.data.oMf[foot].translation.copy() for foot in feet_id]

    #############################################
    # TEST 1: gives the feet positions, find qj #
    #############################################
    # I know the general configuration
    # In most of the cases, I really know it

    print('\n')
    print('##########')
    print('# TEST 1 #')
    print('##########')

    for i, foot in enumerate(feet_id):
        print('EE name:', robot.model.frames[foot].name)
        print('\tPosition:', feet_pos[i])
        print('\tReal Joint config:', IK.u.getLegJointState(legs[i].upper(), qj))

        start = time.time()
        sol, isFeasible = IK.ik_leg(feet_pos[i], foot, hip[i], knee[i])
        T = time.time() - start
        print('\n\tAnalytics time:', np.round(np.array([T*1000]),3)[0], 'ms')
        print('\tIK Solution Analytics:', sol)

        start = time.time()
        solOpt = IKOpt.solve(feet_pos[i], robot.model.frames[foot].name, q0=np.array([0.0, 0.7, -1.4]))
        T = time.time() - start
        print('\n\tOptimization time:', np.round(np.array([T*1000]),3)[0], 'ms')
        print('\tIK Solution Optimization:', solOpt.x)


        start = time.time()
        solMic, IKsuccess = RK.footInverseKinematicsFixedBaseLineSearch(feet_pos[i], robot.model.frames[foot].name, q0_leg=np.array([0.0, 0.7, -1.4]))
        T = time.time() - start
        print('\n\tMichele time:', np.round(np.array([T*1000]),3)[0], 'ms')
        print('\tIK Solution Michele:      ', solMic)







    # ##############################################################################################
    # # TEST 2: gives the feet positions, compute all the solutions and check if they are feasible #
    # ##############################################################################################
    # print('\n')
    # print('##########')
    # print('# TEST 2 #')
    # print('##########')
    #
    # from tabulate import tabulate
    # for i, foot in enumerate(feet_id):
    #     rows = []
    #     print('\nLeg:', legs[i])
    #     print('Position:', feet_pos[i])
    #     print('Real Joint config:', IK.u.getLegJointState(legs[i].upper(), qj))
    #     for hip in [HIP_UP, HIP_DOWN]:
    #         for knee in [KNEE_INWARD, KNEE_OUTWARD]:
    #             # sol, isFeasible = IK.ik_leg(feet_pos[i], foot, hip, knee)
    #             solOpt = IKOpt.solve(feet_pos[i], robot.model.frames[foot].name)
    #             ik_dict = {}
    #             # ik_dict['solutionAnalytics'] = sol
    #             ik_dict['solutionOptimization'] = solOpt.x
    #             ik_dict['configuration'] = [hip, knee]
    #             # ik_dict['in range of motion'] = isFeasible
    #             rows.append(ik_dict)
    #     print(tabulate(rows, headers='keys', tablefmt='grid'), end='\n')
    #
