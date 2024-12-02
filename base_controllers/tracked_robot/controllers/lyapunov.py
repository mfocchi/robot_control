import numpy as np
import math
from base_controllers.utils.math_tools import unwrap_angle
from base_controllers.tracked_robot.utils import constants
from scipy.optimize import fsolve
# ------------------------------------ #
# CONTROLLER'S PARAMETERS
# K_P = 8.0
# K_THETA = 10.0
# K_P = 3
# K_THETA = 2.5
# ------------------------------------ #

class LyapunovParams:
    def __init__(self, K_P, K_THETA, DT=0.001, ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = False):
        self.K_P = K_P
        self.K_THETA = K_THETA
        self.DT = DT
        self.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = ESTIMATE_ALPHA_WITH_ACTUAL_VALUES
class Robot:
    pass

class LyapunovController:
    def __init__(self, params: LyapunovParams): #, matlab_engine = None):

        self.K_P = params.K_P
        self.K_THETA = params.K_THETA

        self.C1 = constants.side_slip_angle_coefficients[0]
        self.C2 = constants.side_slip_angle_coefficients[1]
        self.SIDE_SLIP_COMPENSATION = 'NN'
        self.log_e_x = []
        self.log_e_y = []
        self.log_e_theta = []
        self.goal_reached = False
        self.theta_old = 0.
        self.v_old = 0.
        self.omega_old = 0.

        self.params = params
        self.eng = matlab_engine

    def setSideSlipCompensationType(self, SIDE_SLIP_COMPENSATION):
        self.SIDE_SLIP_COMPENSATION = SIDE_SLIP_COMPENSATION

    def getErrors(self):
        return self.log_e_x, self.log_e_y,  self.log_e_theta

    def control_unicycle(self, actual_state, current_time, des_x, des_y, des_theta, v_d, omega_d, traj_finished):
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
        ex = actual_state.x - des_x
        ey = actual_state.y - des_y

        theta,self.theta_old = unwrap_angle(actual_state.theta, self.theta_old)
        etheta = theta-des_theta

        #compute ausiliary variables
        psi = math.atan2(ey, ex)
        beta = theta + des_theta
        exy = math.sqrt(ex**2 + ey**2)

        dv = -self.K_P * exy * math.cos(psi - theta)
        # important ! the result is 15% different for the sinc function with python from matlab
        # domega = -self.K_THETA * etheta -v_d * np.sinc(0.5 * etheta) * exy * math.sin(psi - 0.5 * beta)
        domega = -v_d * exy * (1/np.cos(etheta/2)) * np.sin(psi - (beta/2)) - self.K_THETA * np.sin(etheta)

        v = v_d + dv
        omega = omega_d + domega

        # V = 1 / 2 * (ex ** 2 + ey ** 2+ etheta**2)
        # V_dot = -self.K_THETA * etheta**2  - self.K_P * exy * math.pow(math.cos(psi - theta),2)
        V = 1 / 2 * (ex ** 2 + ey ** 2) + (1 - np.cos(etheta))
        V_dot = -self.K_P * (exy ** 2) * (np.cos(theta - psi) ** 2) - self.K_THETA * (np.sin(etheta) ** 2)

        #domega = - self.K_THETA * etheta - 2/etheta * v_d * np.sin(0.5 * etheta)* np.sin(psi - 0.5 * beta)
        # save errors for plotting

        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta)

        # print("ERRORS -> x:%.2f, y:%.2f, theta:%.2f" % (ex, ey, etheta))
        # print("VELS -> v:%.2f, o:%.2f" % (v_ref + dv, o_ref + domega))
        return v, omega,  V, V_dot

    def control_alpha(self, actual_state, current_time,  des_x, des_y, des_theta, v_d, omega_d,  v_dot_d, omega_dot_d, traj_finished, model_alpha=None, approx=False):
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
        ex = actual_state.x - des_x
        ey = actual_state.y - des_y
        theta, self.theta_old = unwrap_angle(actual_state.theta, self.theta_old)
        etheta = theta - des_theta

        # compute ausiliary variables
        psi = math.atan2(ey, ex)
        beta = theta + des_theta
        exy = math.sqrt(ex ** 2 + ey ** 2)

        #initial guess for alpha from des values
        alpha_d = self.alpha_exp(v_d, omega_d, model_alpha)


        dv0 = -self.K_P * exy * math.cos(psi -  (alpha_d + theta))

        if approx:
            domega0 = -v_d / math.cos((etheta + alpha_d) / 2) * exy * math.sin(psi - ((alpha_d + beta) / 2)) - self.K_THETA * math.sin(etheta + alpha_d)
            #compute the controls
            v =   (v_d + dv0) *np.cos(alpha_d)
            omega = omega_d + domega0
            alpha = alpha_d
        else:
            alpha_dot_d = self.alpha_dot_exp(v_d, omega_d, v_dot_d, omega_dot_d)
            domega0 = -v_d / math.cos((etheta + alpha_d) / 2) * exy * math.sin(psi - ((alpha_d + beta) / 2)) - self.K_THETA * math.sin(etheta + alpha_d) - alpha_dot_d

            params = ( psi, etheta, exy, theta, alpha_d, beta, v_d, omega_d, v_dot_d, omega_dot_d)
            alpha, dv, domega = fsolve(self.equations, (alpha_d, dv0, domega0), args=params)
            #print(f"alpha: {alpha}, dv: {dv},domega: {domega}")
            v = (v_d + dv) * np.cos(alpha)
            omega = omega_d + domega

        V = 1 / 2 * (ex ** 2 + ey ** 2) + (1- math.cos(etheta + alpha))
        V_dot = - self.K_P *math.pow( exy,2) * math.pow(math.cos(psi - (theta+alpha)), 2) -self.K_THETA * math.pow(math.sin(etheta+alpha),2)
        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta+alpha) #etheta should converge to -alpha
        # save for the next loop computation of alpha from actual values
        self.v_old = v
        self.omega_old = omega
        return v, omega, V, V_dot, alpha


    def equations(self, vars, *data):
        psi, etheta, exy, theta, alpha_d, beta, v_d, omega_d, v_dot_d, omega_dot_d = data
        alpha, dv, domega = vars
        alphadot_d = self.alpha_dot_exp(v_d, omega_d, v_dot_d, omega_dot_d)
        #system of non linear equations
        eq1 = alpha - self.alpha_exp((v_d + dv) * math.cos(alpha), (omega_d + domega))
        eq2 = dv + self.K_P * exy * math.cos(psi - (alpha + theta))
        eq3 = domega + v_d / math.cos((etheta + alpha) / 2) * exy * math.sin(psi - ((alpha + beta) / 2)) + self.K_THETA * math.sin(etheta + alpha_d) + alphadot_d
        return [eq1, eq2, eq3]

    def alpha_exp(self, v_in, omega_in, model_alpha=None):
        # in the case radius is infinite, betas are zero (this is to avoid Nans)
        if self.params.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES:
            v = self.v_old
            omega = self.omega_old
        else:
            v = v_in
            omega = omega_in

        if self.SIDE_SLIP_COMPENSATION == 'EXP' or model_alpha is None:
            if abs(omega) < 1e-05:
                return  0.

            radius = v/(omega)
            if radius > 0:
                alpha = self.C1*np.exp(self.C2*radius)
            else:
                alpha = -self.C1*np.exp(-self.C2*radius)

        elif self.SIDE_SLIP_COMPENSATION=='NN':
            qd = np.zeros(2)
            qd[0] = (v - omega * constants.TRACK_WIDTH / 2) / constants.SPROCKET_RADIUS  # left front
            qd[1] = (v + omega * constants.TRACK_WIDTH / 2) / constants.SPROCKET_RADIUS  # right front
            alpha = model_alpha.predict(qd)
            #matlab
            #alpha = self.eng.feval(model_alpha['predictFcn'], qd)

        return alpha

    def alpha_dot_exp(self, v, omega, v_dot, omega_dot):
        if (v == 0) or (omega == 0):
            alpha_dot = 0.
        else:
            # deal with degenerate cases (I computed numerically the limits in situations that creatde Nans)
            if abs(omega)<0.0001:
                alpha_dot = 0.
            else:
                r = v / omega
                if r > 0:
                    alpha = self.C1*math.exp(self.C2*r)
                    alpha_dot = alpha * math.log(alpha/(self.C1))*(v_dot/v - omega_dot/omega)
                else:
                    alpha = -self.C1*math.exp(-self.C2*r)
                    alpha_dot = alpha * math.log(alpha/(-self.C1))*(v_dot/v - omega_dot/omega)

        return  alpha_dot