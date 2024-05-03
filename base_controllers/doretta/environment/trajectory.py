from ..models.unicycle import Unicycle

from enum import Enum


class ModelsList(Enum):
    UNICYCLE = 1
    TRACKED = 2


class Trajectory:
    def __init__(self, model, start_x, start_y, start_theta, velocity_generator, DT):
        self.x = [start_x]
        self.y = [start_y]
        self.theta = [start_theta]
        self.v = []
        self.omega = []

        if model is ModelsList.UNICYCLE:
            self.robot = Unicycle(start_x, start_y, start_theta, velocity_generator, DT)
        else:
            assert False, "Trajectory generator: model is not valid"

        self.init_trajectory(velocity_generator)

    def init_trajectory(self, velocity_generator):
        """
        :param velocity_generator: function which returns a list of longitudinal velocity and a list of angular velocities
        :return: void
        """
        v, o, _ = velocity_generator()
        assert len(v) == len(o), "Trajectory generator: Invalid input (lenght)"
        for i in range(len(v) - 1):
            self.robot.update(v[i], o[i])
            self.x.append(self.robot.x)
            self.y.append(self.robot.y)
            self.theta.append(self.robot.theta)
            self.v.append(v[i])
            self.omega.append(o[i])
        
        # append finale di velocità nulle
        self.v.append(0.0)
        self.omega.append(0.0)

