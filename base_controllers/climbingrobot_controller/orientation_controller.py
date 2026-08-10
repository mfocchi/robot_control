import numpy as np
import math
from base_controllers.utils.math_tools import Math
from base_controllers.utils.math_tools import computeOrientationError

class OrientationController():
    def __init__(self, base_line_x = 0.5, base_line_y= 0.5, propeller_orient = np.zeros(4)):
        self.b_propeller_pos = [np.zeros(3)]*4
        self.b_propeller_pos[0] = np.array([base_line_x, base_line_y,0])
        self.b_propeller_pos[1] = np.array([-base_line_x, base_line_y, 0])
        self.b_propeller_pos[2] = np.array([-base_line_x, -base_line_y, 0])
        self.b_propeller_pos[3] = np.array([base_line_x, -base_line_y, 0])
        self.math_utils = Math()
        self.A = self.buildActuationMatrix(propeller_orient, self.b_propeller_pos)

    def buildActuationMatrix(self, propeller_orient, b_propeller_pos):
        A = np.zeros((6,4))
        self.b_propeller_axes = [np.zeros(3)] * 4
        #set Fx, Fy
        for prop in range(4):
            v = self.rotateZ_2d(propeller_orient[prop]+np.pi/2) @ np.array([1., 0.])
            self.b_propeller_axes[prop] = np.array([v[0], v[1], 0.])
            A[:3, prop] = self.b_propeller_axes[prop]
        # set Gamma
        for prop in range(4):
            A[3:, prop] = np.cross(b_propeller_pos[prop], self.b_propeller_axes[prop])
        np.set_printoptions(precision=5, suppress=True)
        #print("A =\n", A)
        return A

    def rotateZ_2d(self, alpha):
        return  np.array([[math.cos(alpha), -math.sin(alpha) ], [math.sin(alpha), math.cos(alpha) ]])

    def computeThrust(self, des_orient, act_orient, w_omega_b, Ko, Do, w_additional_force = None):
        # compute desired orientation
        w_R_des = self.math_utils.eul2Rot(des_orient)
        #compute actual orient
        w_R_b = self.math_utils.eul2Rot(act_orient)

        w_error_o = computeOrientationError(w_R_b, w_R_des)

        W_Fdes = np.zeros(3)
        if w_additional_force is not None:
            W_Fdes += w_additional_force
        # compute the virtual moment (angular part of the wrench) to realize the orientation task
        W_Gamma_des = np.multiply(Ko, w_error_o) + np.multiply(Do, - w_omega_b)
        W_wrench_des = np.concatenate((W_Fdes, W_Gamma_des))

        #map to BF
        B_Fdes = w_R_b.T.dot(W_Fdes)
        B_Gamma_des = w_R_b.T.dot(W_Gamma_des)
        B_wrench_des = np.concatenate((B_Fdes, B_Gamma_des))

        #compute thrusts
        prop_trusts = np.linalg.pinv(self.A).dot(B_wrench_des)
        return prop_trusts, W_wrench_des

if __name__ == '__main__':
    base_line_x = 0.3
    base_line_y = 0.5
    propeller_orient = np.array([0.25*np.pi, 0.75*np.pi, np.pi + 0.25*np.pi, np.pi + 0.75*np.pi])
    from base_controllers import params as conf
    p = OrientationController(base_line_x,base_line_y,propeller_orient)
    #des orient
    des_orient =  np.array([0., 0., -0.5])
    #actual orient
    act_orient = np.array([0., 0., 0.])
    #actual twist
    w_omega_b = np.zeros(3)
    thrusts = p.computeThrust(des_orient, act_orient, w_omega_b, conf.robot_params["climbingrobot2"]['Ko'], conf.robot_params["climbingrobot2"]['Do'])
    print(f"Thsusts: {thrusts}")