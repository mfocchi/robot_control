import numpy as np

from ..utils import constants as const
from ..utils.tools import normalize_angle


class Unicycle:
    """
        Class representing the state of a vehicle.

        :param x: (float) x-coordinate
        :param y: (float) y-coordinate
        :param theta: (float) theta angle
        :param v: (float) linear velocity
        :param omega: (float) angular velocity
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, omega=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega

    def update(self, vel, omega):
        """
        Update the state of the vehicle.

        :param vel: (float) Linear velocity
        :param omega: (float) Angular velocity
        """

        # TODO limit linear velocity
        omega = np.clip(omega, -const.MAX_ANGULAR_VELOCITY, const.MAX_ANGULAR_VELOCITY)  # limit the steering angle

        self.x += self.v * np.cos(self.theta) * const.DT
        self.y += self.v * np.sin(self.theta) * const.DT
        self.theta += omega * const.DT
        self.theta = normalize_angle(self.theta)
        self.v = vel
        self.omega = omega

    def state(self):
        return self.x, self.y, self.theta, self.v, self.omega
