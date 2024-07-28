import time

import numpy as np

from ..utils.constants import DT
from ..environment.trajectory import Trajectory, ModelsList
from ..utils.tools import normalize_angle
import math
# ------------------------------------ #
# CONTROLLER'S PARAMETERS
# K_P = 8.0
# K_THETA = 10.0
# K_P = 3
# K_THETA = 2.5
# ------------------------------------ #

class LyapunovParamsZou:
    def __init__(self, K_P, K_THETA):
        self.K_x = K_P
        self.K_y = K_P
        self.K_theta = K_THETA

class LyapunovControllerZou:
    def __init__(self, params: LyapunovParamsZou):

        self.K_x = params.K_x
        self.K_y = params.K_y
        self.K_theta = params.K_theta

        self.trajectory = None
        self.total_time = -1.0
        self.start_time = -1.0
        self.draw_e_x = []
        self.draw_e_y = []
        self.draw_e_theta = []
        self.draw_v = []
        self.draw_omega = []
        self.goal_reached = False



    def config(self, start_time, trajectory):
        self.trajectory = trajectory
        self.total_time = len(self.trajectory.x) * DT
        self.start_time = start_time
        self.draw_e_x.append(0.0)
        self.draw_e_y.append(0.0)
        self.draw_e_theta.append(0.0)
        self.draw_v.append(0.0)
        self.draw_omega.append(0.0)

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
            self.draw_v.append(0.0)
            self.draw_omega.append(0.0)

            return 0.0, 0.0

        assert current_index < len(self.trajectory.v)-1, "Lyapunov controller: index out of range"

        # compute errors in base frame
        e_x = math.cos(robot.theta)*(self.trajectory.x[current_index] - robot.x) + math.sin(robot.theta)*(self.trajectory.y[current_index] - robot.y)
        e_y = -math.sin(robot.theta)*(self.trajectory.x[current_index] - robot.x) + math.cos(robot.theta)*(self.trajectory.y[current_index] - robot.y)
        e_theta = self.trajectory.theta[current_index] -robot.theta
        e_theta =normalize_angle(e_theta)

        #ref traj
        v_r = self.trajectory.v[current_index]
        omega_r = self.trajectory.omega[current_index]

        #controller
        v = v_r * math.cos(e_theta) + self.K_x * e_x
        omega = omega_r + v_r * (self.K_y * e_y + self.K_theta * math.sin(e_theta))

        # error  derivative in base frame
        e_dot_x = e_y * omega - v + v_r * math.cos(e_theta)
        e_dot_y = - e_x * omega + v_r * math.sin(e_theta)
        e_dot_theta = -omega + omega_r
        # compute liapunov funcitons
        V = 1 / 2 * (e_x**2 + e_y**2) + (1 - math.cos(e_theta)) / self.K_y;
        Vdot = e_x * e_dot_x + e_y * e_dot_y + e_dot_theta * math.sin(e_theta) / self.K_y;

        # save errors for plotting
        self.draw_e_x.append(e_x)
        self.draw_e_y.append(e_y)
        self.draw_e_theta.append(e_theta)
        self.draw_v.append(v)
        self.draw_omega.append(omega)

        # print("ERRORS -> x:%.2f, y:%.2f, theta:%.2f" % (ex, ey, etheta))
        # print("VELS -> v:%.2f, o:%.2f" % (v_ref + dv, o_ref + domega))


        return v, omega


