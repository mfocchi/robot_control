import numpy as np
import math
from base_controllers.utils.math_tools import unwrap_angle
from base_controllers.doretta.utils import constants
from scipy.optimize import fsolve
from tracked_vehicle import TrackedVehicle
from tracked_vehicle import VehicleParam


class TrackeSimulator:
    def __init__(self):
        sigma = np.ones(track_param.parts_longitudinal, track_param.parts_lateral) * vehicle_param.weight / (2 * track_param.A);
        self.TrackedRobot = self.TrackedVehicle(vehicle_param, track_param, ground);
        [patch_pos_long_l, patch_pos_lat_l] = TrackedRobot.getLeftPatchesPositions();
        [patch_pos_long_r, patch_pos_lat_r] = TrackedRobot.getRightPatchesPositions();

    def simStep:
        pass






if __name__ == '__main__':
