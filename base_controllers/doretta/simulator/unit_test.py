import numpy as np

import unittest
from base_controllers.doretta.simulator.tracked_vehicle_simulator import TrackParams, Ground, SimParam
from base_controllers.doretta.simulator.tracked_vehicle import TrackedVehicle, VehicleParam
from base_controllers.doretta.simulator.track import TrackParams

np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)

#to run: python3 -m unittest test_climbing_robot


class TestTrack(unittest.TestCase):

    def test_track(self):
        self.vehicle_param = VehicleParam()
        self.track_param = TrackParams()
        self.sim_param = SimParam()
        self.ground = Ground()
        self.sigma = np.ones((self.track_param.parts_longitudinal, self.track_param.parts_lateral)) * self.vehicle_param.weight / (
                                 2 * self.track_param.A)
        self.tracked_robot = TrackedVehicle(self.vehicle_param, self.track_param, self)
        omega_left =
        Fx_l, Fy_l, M_long_l, M_lat_l = self.tracked_robot.track_left.computeTerrainInteractions(self.state, omega_left, self.track_param,       self.sigma,
                                                                                                 self.ground,
                                                                                                 self.patch_pos_long_l,
                                                                                                 self.patch_pos_lat_l)



        print("Hull", hull.vertices)
        self.assertEqual(Fx_l, 14)
        self.assertEqual(Fy_l, 14)
        self.assertEqual(M_long_l, 14)
        self.assertEqual(M_lat_l, 14)

    def test_dynamics(self):