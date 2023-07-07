import time

import numpy as np

from ..utils.constants import DT
from ..environment.trajectory import Trajectory, ModelsList
from ..utils.tools import normalize_angle

# ------------------------------------ #
# CONTROLLER'S PARAMETERS
# K_P = 8.0
# K_THETA = 10.0
# K_P = 3
# K_THETA = 2.5
# ------------------------------------ #

class LyapunovParams:
    def __init__(self, K_P, K_THETA):
        self.K_P = K_P
        self.K_THETA = K_THETA

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
        self.draw_e_v = []
        self.draw_e_omega = []
        self.goal_reached = False

    def config(self, start_x, start_y, start_theta, start_time, velocity_generator):
        self.trajectory = Trajectory(ModelsList.UNICYCLE, start_x, start_y, start_theta, velocity_generator)
        self.total_time = len(self.trajectory.x) * DT
        self.start_time = start_time
        self.draw_e_x.append(0.0)
        self.draw_e_y.append(0.0)
        self.draw_e_theta.append(0.0)
        self.draw_e_v.append(0.0)
        self.draw_e_omega.append(0.0)

    def control(self, robot, current_time):
        """
        ritorna i valori di linear e angular velocity
        """
        elapsed_time = current_time - self.start_time
        current_index = int(elapsed_time / DT)
        # quando arrivo all'indice dell'ultimo punto della traiettoria, la traiettoria è finita
        if current_index >= len(self.trajectory.v)-1:
            # target is considered reached
            print("Lyapunov controller: target reached")
            self.goal_reached = True

            # save errors for plotting
            self.draw_e_x.append(0.0)
            self.draw_e_y.append(0.0)
            self.draw_e_theta.append(0.0)
            self.draw_e_v.append(0.0)
            self.draw_e_omega.append(0.0)

            return 0.0, 0.0

        assert current_index < len(self.trajectory.v)-1, "Lyapunov controller: index out of range"

        # compute errors
        ex = robot.x - self.trajectory.x[current_index]
        ey = robot.y - self.trajectory.y[current_index]
        etheta = robot.theta - self.trajectory.theta[current_index]
        etheta = normalize_angle(etheta)
        theta = robot.theta
        alpha = theta - self.trajectory.theta[current_index]
        v_ref = self.trajectory.v[current_index]
        o_ref = self.trajectory.omega[current_index]

        psi = np.arctan2(ey, ex)
        exy = np.sqrt(ex**2 + ey**2)

        dv = -self.K_P * exy * np.cos(theta - psi)
        domega = - self.K_THETA * etheta - v_ref * np.sinc(0.5 * etheta) * np.sin(psi - 0.5 * alpha)

        # save errors for plotting
        self.draw_e_x.append(ex)
        self.draw_e_y.append(ey)
        self.draw_e_theta.append(etheta)
        self.draw_e_v.append(dv)
        self.draw_e_omega.append(domega)

        # print("ERRORS -> x:%.2f, y:%.2f, theta:%.2f" % (ex, ey, etheta))
        # print("VELS -> v:%.2f, o:%.2f" % (v_ref + dv, o_ref + domega))


        return v_ref + dv, o_ref + domega


