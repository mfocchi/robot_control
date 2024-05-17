import numpy as np

from base_controllers.utils.math_tools import unwrap_angle

class Unicycle:
    """
        Class representing the state of a vehicle.

        :param x: (float) x-coordinate
        :param y: (float) y-coordinate
        :param theta: (float) theta angle
        :param v: (float) linear velocity
        :param omega: (float) angular velocity
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, DT=0.01):
        self.x = x
        self.y = y
        self.theta = theta

        self.theta_old = theta
        self.DT = DT

    def set_state(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def update(self, vel, omega):
        """
        Update the state of the vehicle.

        :param vel: (float) Linear velocity
        :param omega: (float) Angular velocity
        """


        # eulers integration
        # self.x += vel * np.cos(self.theta) * const.DT
        # self.y += vel * np.sin(self.theta) * const.DT
        # self.theta += omega * const.DT

        # proper integration with ZOH updtes considering the constant evolution of theta across the steps with constant omega and v
        self.x += vel * self.DT  * np.sinc(omega*self.DT/2) * np.cos(self.theta + omega* self.DT/2)
        self.y += vel * self.DT  * np.sinc(omega*self.DT/2) * np.sin(self.theta + omega* self.DT/2)
        self.theta += omega * self.DT
        # this is not needed here
        # self.theta,self.theta_old = unwrap_angle(self.theta, self.theta_old)

    def state(self):
        return self.x, self.y, self.theta
