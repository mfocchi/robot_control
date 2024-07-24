import numpy as np

from ..models.unicycle import Unicycle

from enum import Enum
from base_controllers.utils.math_tools import unwrap_angle

class ModelsList(Enum):
    UNICYCLE = 1
    TRACKED = 2


class Trajectory:
    def __init__(self, model, start_x, start_y, start_theta, velocity_generator=None, DT = 0.001,  v = None, o = None):
        self.x = [start_x]
        self.y = [start_y]
        self.theta = [start_theta]
        self.des_theta_old = 0.
        self.v = []
        self.omega = []
        self.v_dot = []
        self.omega_dot = []
        self.DT =DT
        self.start_time = 0.
        if model is ModelsList.UNICYCLE:
            self.ideal_unicycle = Unicycle(start_x, start_y, start_theta, DT)
        else:
            assert False, "Trajectory generator: model is not valid"
        if velocity_generator is not None:
            self.init_trajectory(velocity_generator)
        if (v is not None) and (o is not None):
            self.init_trajectory_with_user_vel(v, o)

    def set_initial_time(self, start_time):
        self.start_time = start_time

    def getSingleUpdate(self, x, y, theta, v, o):
        self.ideal_unicycle.set_state( x, y, theta)
        self.ideal_unicycle.update(v, o)
        return self.ideal_unicycle.x, self.ideal_unicycle.y, self.ideal_unicycle.theta

    def init_trajectory_with_user_vel(self, v, o, v_dot = None, omega_dot = None):
        """
        :param velocity_generator: function which returns a list of longitudinal velocity and a list of angular velocities
        :return: void
        """
        if v_dot is None:
            v_dot = np.zeros_like(v)
        if omega_dot is None:
            omega_dot = np.zeros_like(o)

        assert len(v) == len(o), "Trajectory generator: Invalid input (lenght)"
        for i in range(len(v) - 1):
            self.ideal_unicycle.update(v[i], o[i])
            self.x.append(self.ideal_unicycle.x)
            self.y.append(self.ideal_unicycle.y)
            self.theta.append(self.ideal_unicycle.theta)
            self.v.append(v[i])
            self.omega.append(o[i])
            self.v_dot.append(v_dot[i])
            self.omega_dot.append(omega_dot[i])

        # append finale di velocità nulle
        self.v.append(0.0)
        self.omega.append(0.0)
        self.v_dot.append(0.0)
        self.omega_dot.append(0.0)

    def init_trajectory(self, velocity_generator):
        """
        :param velocity_generator: function which returns a list of longitudinal velocity and a list of angular velocities
        :return: void
        """
        v, o, v_dot, omega_dot, _ = velocity_generator()
        assert len(v) == len(o), "Trajectory generator: Invalid input (lenght)"
        for i in range(len(v) - 1):
            self.ideal_unicycle.update(v[i], o[i])
            self.x.append(self.ideal_unicycle.x)
            self.y.append(self.ideal_unicycle.y)
            self.theta.append(self.ideal_unicycle.theta)
            self.v.append(v[i])
            self.omega.append(o[i])
            self.v_dot.append(v_dot[i])
            self.omega_dot.append(omega_dot[i])
        
        # append finale di velocità nulle
        self.v.append(0.0)
        self.omega.append(0.0)
        self.v_dot.append(0.0)
        self.omega_dot.append(0.0)

    def evalTraj(self, current_time):
        elapsed_time = current_time - self.start_time
        current_index = int(elapsed_time / self.DT)

        # quando arrivo all'indice dell'ultimo punto della traiettoria, la traiettoria è finita
        if current_index >= len(self.v) - 1:
            # target is considered reached
            print("Lyapunov controller: trajectory finished")
            return 0, 0, 0, 0, 0, 0,0,True

        assert current_index < len(self.v) - 1, "Lyapunov controller: index out of range"

        des_x = self.x[current_index]
        des_y = self.y[current_index]
        des_theta, self.des_theta_old = unwrap_angle(self.theta[current_index], self.des_theta_old)
        v_d = self.v[current_index]
        omega_d = self.omega[current_index]
        v_dot_d = self.v_dot[current_index]
        omega_dot_d = self.omega_dot[current_index]
        return des_x, des_y, des_theta, v_d, omega_d,v_dot_d, omega_dot_d, False

