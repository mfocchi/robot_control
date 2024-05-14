import time

import numpy as np


from ..environment.trajectory import Trajectory, ModelsList
from ..utils.tools import normalize_angle
import math
from base_controllers.doretta.utils.tools import unwrap_angle
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

        self.trajectory = None
        self.total_time = -1.0
        self.start_time = -1.0
        self.draw_e_x = []
        self.draw_e_y = []
        self.draw_e_theta = []
        self.goal_reached = False
        self.theta_old = 0.
        self.des_theta_old = 0.
        self.params = params

    def config(self, start_time, trajectory):
        self.trajectory = trajectory
        self.total_time = len(self.trajectory.x) * self.params.DT
        self.start_time = start_time
        self.draw_e_x.append(0.0)
        self.draw_e_y.append(0.0)
        self.draw_e_theta.append(0.0)

    def control(self, robot, current_time):
        """
        ritorna i valori di linear e angular velocity
        """
        elapsed_time = current_time - self.start_time
        current_index = int(elapsed_time / self.params.DT)

        # quando arrivo all'indice dell'ultimo punto della traiettoria, la traiettoria è finita
        if current_index >= len(self.trajectory.v)-1:
            # target is considered reached
            print("Lyapunov controller: trajectory finished")
            self.goal_reached = True

            # save errors for plotting
            self.draw_e_x.append(0.0)
            self.draw_e_y.append(0.0)
            self.draw_e_theta.append(0.0)


            return 0.0, 0.0, 0., 0., 0., 0., 0., 0.,0.

        assert current_index < len(self.trajectory.v)-1, "Lyapunov controller: index out of range"

        des_x = self.trajectory.x[current_index]
        des_y = self.trajectory.y[current_index]
        # compute errors
        ex = robot.x - self.trajectory.x[current_index]
        ey = robot.y - self.trajectory.y[current_index]

        theta,self.theta_old = unwrap_angle(robot.theta, self.theta_old)
        des_theta, self.des_theta_old = unwrap_angle(self.trajectory.theta[current_index], self.des_theta_old)
        etheta = theta-des_theta


        alpha = theta + des_theta
        v_ref = self.trajectory.v[current_index]
        o_ref = self.trajectory.omega[current_index]




        psi = math.atan2(ey, ex)
        exy = math.sqrt(ex**2 + ey**2)

        dv = -self.K_P * exy * math.cos(psi - theta)
        # important ! the result is 15% different for the sinc function with python from matlab
        domega = -self.K_THETA * etheta -v_ref * np.sinc(0.5 * etheta) * exy * math.sin(psi - 0.5 * alpha)


        V = 1 / 2 * (ex ** 2 + ey ** 2+ etheta**2)
        V_dot = -self.K_THETA * etheta**2  - self.K_P * exy * math.pow(math.cos(psi - theta),2)


        #domega = - self.K_THETA * etheta - 2/etheta * v_ref * np.sin(0.5 * etheta)* np.sin(psi - 0.5 * alpha)
        # save errors for plotting
        self.draw_e_x.append(ex)
        self.draw_e_y.append(ey)
        self.draw_e_theta.append(etheta)

        # print("ERRORS -> x:%.2f, y:%.2f, theta:%.2f" % (ex, ey, etheta))
        # print("VELS -> v:%.2f, o:%.2f" % (v_ref + dv, o_ref + domega))


        return v_ref + dv, o_ref + domega, des_x, des_y, des_theta, v_ref, o_ref, V, V_dot


