import numpy as np
import pinocchio as pin
from base_controllers.utils.utils import Utils

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

        for leg in ['lf', 'rf', 'lh', 'rh']:
            self._hip_id_dict[leg] = self.robot.model.getFrameId(leg + '_hip')
            self._upperleg_id_dict[leg] = self.robot.model.getFrameId(leg + '_upperleg')
            self._lowerleg_id_dict[leg] = self.robot.model.getFrameId(leg + '_lowerleg')
            self._foot_id_dict[leg] = self.robot.model.getFrameId(leg + '_foot')

        # Compute the relative distance from the reference frames
        self.measures = {}
        self.upper_limits = {}
        self.lower_limits = {}

        for leg in ['lf', 'rf', 'lh', 'rh']:
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


        self.KneeInward = KNEE_INWARD
        self.KneeOutward = KNEE_OUTWARD

    def ik_leg(self, foot_pos, foot_idx, hip=HIP_DOWN, knee=KNEE_INWARD):
        leg = self.robot.model.frames[foot_idx].name[:2]

        HAA_foot_x = foot_pos[0] - self.measures[leg]['base_2_HAA_x']
        HAA_foot_y = foot_pos[1] - self.measures[leg]['base_2_HAA_y']
        HAA_foot_z = np.sqrt(foot_pos[2] ** 2 + HAA_foot_y ** 2 - self.measures[leg]['HAA_2_FOOT_y'] ** 2)

        if hip == HIP_DOWN:
            HAA_foot_z *= -1
        # Verify if the foot is inside the work space of the leg (WS1)
        if (HAA_foot_x ** 2 + HAA_foot_z ** 2) > (
                self.measures[leg]['HFE_2_KFE_z'] + self.measures[leg]['KFE_2_FOOT_z']) ** 2:
            raise ValueError('Foot position is out of the workspace')

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
                self.measures[leg][
                    'KFE_2_FOOT_z'] * self.measures[leg]['HFE_2_KFE_z'] * cos_qKFE)

        cos_qHFE = (c_num0 + c_num1) / den
        cos_qHFE = np.round(cos_qHFE, 10)

        s_num0 = -(self.measures[leg]['HFE_2_KFE_z'] + self.measures[leg]['KFE_2_FOOT_z'] * cos_qKFE) * HAA_foot_x
        s_num1 = self.measures[leg]['KFE_2_FOOT_z'] * sin_qKFE * HAA_foot_z

        sin_qHFE = (s_num0 + s_num1) / den
        sin_qHFE = np.round(sin_qHFE, 10)

        qHFE = np.arctan2(sin_qHFE, cos_qHFE)

        # Save the solution
        q = np.array([qHAA, qHFE, qKFE])
        if (q < self.upper_limits[leg]).all() and (q > self.lower_limits[leg]).all():
            inROM = True
        else:
            inROM = False

        return q, inROM


if __name__ == '__main__':
    from base_controllers.utils.common_functions import getRobotModel
    # robot_name = 'solo'
    # qj = np.array([ 0.2,  np.pi / 4, -np.pi / 2,   # lf
    #                 0.2, -np.pi / 4,  np.pi / 2,   # lh
    #                -0.2,  np.pi / 4, -np.pi / 2,   # rf
    #                -0.2, -np.pi / 4,  np.pi / 2])  # rh
    # hip = [HIP_DOWN]*4
    # knee = [KNEE_INWARD] * 4

    robot_name = 'aliengo'
    qj = np.array([ 0.2, 0.7, -1.4,  # lf
                    0.2, 0.7, -1.4,  # lh
                   -0.2, 0.7, -1.4,  # rf
                   -0.2, 0.7, -1.4])  # rh
    hip = [HIP_DOWN] * 4
    knee = [KNEE_INWARD, KNEE_OUTWARD] * 2

    robot = getRobotModel(robot_name, generate_urdf=True)
    q = pin.neutral(robot.model)
    q[7:12 + 7] = qj

    IK = InverseKinematics(robot)
    pin.forwardKinematics(robot.model, robot.data, q)
    pin.updateFramePlacements(robot.model, robot.data)
    legs = ['lf', 'lh', 'rf', 'rh']
    feet_id = [robot.model.getFrameId(leg + '_foot') for leg in legs]
    feet_pos = [robot.data.oMf[foot].translation for foot in feet_id]


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
        sol, inROM = IK.ik_leg(feet_pos[i], foot, hip[i], knee[i])
        print('Leg:', legs[i])
        print('\tPosition:', feet_pos[i])
        print('\tIn Range of Motion:', inROM)
        print('\tIK Solution:      ', sol)
        print('\tReal Joint config:', IK.u.getLegJointState(legs[i].upper(), qj))



    ##############################################################################################
    # TEST 2: gives the feet positions, compute all the solutions and check if they are feasible #
    ##############################################################################################
    print('\n')
    print('##########')
    print('# TEST 2 #')
    print('##########')

    from tabulate import tabulate
    for i, foot in enumerate(feet_id):
        rows = []
        print('\nLeg:', legs[i])
        print('Position:', feet_pos[i])
        print('Real Joint config:', IK.u.getLegJointState(legs[i].upper(), qj))
        for hip in [HIP_UP, HIP_DOWN]:
            for knee in [KNEE_INWARD, KNEE_OUTWARD]:
                sol, inROM = IK.ik_leg(feet_pos[i], foot, hip, knee)
                ik_dict = {}
                ik_dict['solution'] = sol
                ik_dict['configuration'] = [hip, knee]
                ik_dict['in range of motion'] = inROM
                rows.append(ik_dict)
        print(tabulate(rows, headers='keys', tablefmt='grid'), end='\n')

