import numpy as np
import math
from base_controllers.utils.math_tools import unwrap_angle
from base_controllers.doretta.utils import constants
# ------------------------------------ #
# CONTROLLER'S PARAMETERS
# K_P = 8.0
# K_THETA = 10.0
# K_P = 3
# K_THETA = 2.5
# ------------------------------------ #

class LyapunovParams:
    def __init__(self, K_P, K_THETA, DT=0.001):
        self.K_P = K_P
        self.K_THETA = K_THETA
        self.DT = DT
class Robot:
    pass

class LyapunovController:
    def __init__(self, params: LyapunovParams):

        self.K_P = params.K_P
        self.K_THETA = params.K_THETA

        self.C1 = constants.side_slip_angle_coefficients[0]
        self.C2 = constants.side_slip_angle_coefficients[1]

        self.log_e_x = []
        self.log_e_y = []
        self.log_e_theta = []
        self.goal_reached = False
        self.theta_old = 0.

        self.params = params



    def getErrors(self):
        return self.log_e_x, self.log_e_y,  self.log_e_theta

    def control_unicycle(self, robot, current_time, des_x, des_y, des_theta, v_d, omega_d, traj_finished):
        """
        ritorna i valori di linear e angular velocity
        """
        if traj_finished:
            # save errors for plotting
            self.log_e_x.append(0.0)
            self.log_e_y.append(0.0)
            self.log_e_theta.append(0.0)
            return 0.0, 0.0, 0., 0.

        # compute errors
        ex = robot.x - des_x
        ey = robot.y - des_y
        theta,self.theta_old = unwrap_angle(robot.theta, self.theta_old)
        etheta = theta-des_theta

        #compute ausiliary variables
        psi = math.atan2(ey, ex)
        beta = theta + des_theta
        exy = math.sqrt(ex**2 + ey**2)

        dv = -self.K_P * exy * math.cos(psi - theta)
        # important ! the result is 15% different for the sinc function with python from matlab
        domega = -self.K_THETA * etheta -v_d * np.sinc(0.5 * etheta) * exy * math.sin(psi - 0.5 * beta)

        v = v_d + dv
        omega = omega_d + domega

        V = 1 / 2 * (ex ** 2 + ey ** 2+ etheta**2)
        V_dot = -self.K_THETA * etheta**2  - self.K_P * exy * math.pow(math.cos(psi - theta),2)

        #domega = - self.K_THETA * etheta - 2/etheta * v_d * np.sin(0.5 * etheta)* np.sin(psi - 0.5 * beta)
        # save errors for plotting
        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta)

        # print("ERRORS -> x:%.2f, y:%.2f, theta:%.2f" % (ex, ey, etheta))
        # print("VELS -> v:%.2f, o:%.2f" % (v_ref + dv, o_ref + domega))
        return v, omega,  V, V_dot



    def control_alpha0(self, robot, current_time,  des_x, des_y, des_theta, v_d, omega_d, traj_finished):
        """
        ritorna i valori di linear e angular velocity
        """

        if traj_finished:
            # save errors for plotting
            self.log_e_x.append(0.0)
            self.log_e_y.append(0.0)
            self.log_e_theta.append(0.0)
            return 0.0, 0.0, 0., 0.,0., 0.

        # compute errors
        ex = robot.x - des_x
        ey = robot.y - des_y
        theta, self.theta_old = unwrap_angle(robot.theta, self.theta_old)
        etheta = theta - des_theta

        # compute ausiliary variables
        psi = math.atan2(ey, ex)
        beta = theta + des_theta
        exy = math.sqrt(ex ** 2 + ey ** 2)

        #estimate alpha from des values
        alpha_0 = self.alpha_exp(v_d, omega_d)

        dv = -self.K_P * exy * math.cos(psi -  (alpha_0 + theta))
        domega = -v_d / math.cos((etheta + alpha_0) / 2) * exy * math.sin(psi - ((alpha_0 + beta) / 2)) - self.K_THETA * math.sin(etheta + alpha_0)

        #compute the controls
        v =   (v_d + dv) *np.cos(alpha_0)
        omega = omega_d + domega

        V = 1 / 2 * (ex ** 2 + ey ** 2) + (1- math.cos(etheta + alpha_0))
        V_dot = - self.K_P *math.pow( exy,2) * math.pow(math.cos(psi - (theta+alpha_0)), 2) -self.K_THETA * math.pow(math.sin(etheta+alpha_0),2)
        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta+alpha_0) #etheta should converge to -alpha
        return v, omega, V, V_dot, alpha_0

    def alpha_exp(self, v, omega):
        # in the case radius is infinite, betas are zero (this is to avoid Nans)
        if abs(omega) < 1e-05:
            return  0.

        radius = v/(omega)

        if radius > 0:
            alpha = self.C1*np.exp(self.C2*radius)
        else:
            alpha = -self.C1*np.exp(-self.C2*radius)
        return alpha