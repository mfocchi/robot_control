import numpy as np
import math
from base_controllers.utils.math_tools import unwrap_angle
# ------------------------------------ #
# CONTROLLER'S PARAMETERS
# K_P = 8.0
# K_THETA = 10.0
# K_P = 3
# K_THETA = 2.5
# ------------------------------------ #

class LyapunovParams:
    def __init__(self, K_P, K_THETA, DT=0.001, C1= -2.0397, C2= -5.2179):
        self.K_P = K_P
        self.K_THETA = K_THETA
        self.DT = DT
        self.C2=C2
        self.C1 =C1
class Robot:
    pass

class LyapunovController:
    def __init__(self, params: LyapunovParams):

        self.K_P = params.K_P
        self.K_THETA = params.K_THETA
        self.C2 = params.C2
        self.C1 = params.C1

        self.trajectory = None
        self.total_time = -1.0
        self.start_time = -1.0
        self.log_e_x = []
        self.log_e_y = []
        self.log_e_theta = []
        self.goal_reached = False
        self.theta_old = 0.
        self.des_theta_old = 0.
        self.params = params

    def config(self, start_time, trajectory):
        self.trajectory = trajectory
        self.total_time = len(self.trajectory.x) * self.params.DT
        self.start_time = start_time

    def getErrors(self):
        return self.log_e_x, self.log_e_y,  self.log_e_theta

    def evalTraj(self, current_time):
        elapsed_time = current_time - self.start_time
        current_index = int(elapsed_time / self.params.DT)

        # quando arrivo all'indice dell'ultimo punto della traiettoria, la traiettoria è finita
        if current_index >= len(self.trajectory.v) - 1:
            # target is considered reached
            print("Lyapunov controller: trajectory finished")
            self.goal_reached = True

            # save errors for plotting
            self.log_e_x.append(0.0)
            self.log_e_y.append(0.0)
            self.log_e_theta.append(0.0)
            traj_finished = True
            return 0,0,0,0,0, True

        assert current_index < len(self.trajectory.v) - 1, "Lyapunov controller: index out of range"

        des_x = self.trajectory.x[current_index]
        des_y = self.trajectory.y[current_index]
        des_theta, self.des_theta_old = unwrap_angle(self.trajectory.theta[current_index], self.des_theta_old)
        v_d = self.trajectory.v[current_index]
        omega_d = self.trajectory.omega[current_index]
        return des_x, des_y, des_theta, v_d, omega_d, False

    def control_unicycle(self, robot, current_time):
        """
        ritorna i valori di linear e angular velocity
        """
        des_x, des_y, des_theta, v_d, omega_d, traj_finished = self.evalTraj(current_time)

        if traj_finished:
            return 0.0, 0.0, 0., 0., 0., 0., 0., 0., 0.

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
        return v, omega, des_x, des_y, des_theta, v_d, omega_d, V, V_dot

    def alpha_exp(self, v, omega):

        radius = v/(omega+1e-10)

        if radius > 0:
            alpha = self.C1*np.exp(self.C2*radius)
        else:
            alpha = -self.C1*np.exp(-self.C2*radius)
        return alpha

    def control_alpha0(self, robot, current_time):
        """
        ritorna i valori di linear e angular velocity
        """
        des_x, des_y, des_theta, v_d, omega_d, traj_finished = self.evalTraj(current_time)
        if traj_finished:
            return 0.0, 0.0, 0., 0., 0., 0., 0., 0., 0.

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

        V = 1 / 2 * (ex ** 2 + ey ** 2 + etheta ** 2)
        V_dot = -self.K_THETA * etheta ** 2 - self.K_P * exy * math.pow(math.cos(psi - theta), 2)
        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta+alpha_0)
        return v, omega, des_x, des_y, des_theta, v_d, omega_d, V, V_dot

