
import numpy as np
from base_controllers.tracked_robot.simulator.track import Track
from  base_controllers.tracked_robot.utils import constants as constants

class VehicleParam:
    def __init__(self):
        self.mass = constants.mass; #[kg] vehicle mass
        self.Izz = constants.inertia_moment; #[kg m^2] vehicle inertia
        self.width = constants.TRACK_WIDTH; #[m]
        self.height = 0.25; #[m]
        self.weight  = self.mass * 9.81


class TrackedVehicle:
    def __init__(self, vehicle_param, track_param, ground_param):
    # Simulates a tracked vehicle. Formulas are taken from "theory of
    # ground vehicles" of Wong.
        self.vehicle_param = vehicle_param
        #using Biral model
        self.track_left  = Track([0.0, vehicle_param.width / 2], track_param)
        self.track_right = Track([0.0,-vehicle_param.width / 2], track_param)
        self.F_left =  np.array([0.0, 0.0])
        self.F_right =  np.array([0.0, 0.0])
        self.M_long_left = 0.0
        self.M_long_right= 0.0
        self.M_lat_left= 0.0
        self.M_lat_right= 0.0

        # normal load for a single track. No load transfer is considered
        sigma = np.ones(self.track_left.getSizePatches()) * vehicle_param.weight / (2 * track_param.A) 

        self.track_left.setNormalStress( sigma) 
        self.track_right.setNormalStress(sigma) 

    def computeShearDisplacement(self, inputs):
        omega_l = inputs.omega_sprocket_left 
        omega_r = inputs.omega_sprocket_right 
        self.track_left.computePatchesShearDisplacement(inputs,  omega_l) 
        self.track_right.computePatchesShearDisplacement(inputs, omega_r) 

    #TODO this function is not implemented
    def computeShearStress(self, ground):
        self.track_left.computePatchesShearStress()
        self.track_right.computePatchesShearStress()

    #TODO this function is not implemented
    def computeShearVelocitys(self,  inputs):
        omega_l = inputs.omega_sprocket_left 
        omega_r = inputs.omega_sprocket_right 
        self.track_left.computeShearVelocitys(inputs,  omega_l) 
        self.track_right.computeShearVelocitys(inputs, omega_r) 

    #TODO this function is not implemented
    def computeShearAngles(self):
        self.track_left.computeShearAngle() 
        self.track_right.computeShearAngle() 

    #TODO this function is not implemented
    def computeTractiveForces(self, ground):
        self.F_left = self.track_left.computeTractiveForce() 
        self.F_right = self.track_right.computeTractiveForce() 

    #TODO this function is not implemented
    def computeResistiveTurningMoments(self):
        self.M_long_left, self.M_lat_left  = self.track_left.computeResistiveTruningMoments() 
        self.M_long_right,self.M_lat_right = self.track_right.computeResistiveTruningMoments() 

    #getters
    def getLeftPatchesPositions(self):
        # compute the x,y position of each discretized patch on the track with respect to the track center
        patches_longitudinal_position = self.track_left.getPatchesLongitudinalPosition()
        patches_lateral_position      = self.track_left.getPatchesLateralPosition()
        return patches_longitudinal_position, patches_lateral_position

    def getRightPatchesPositions(self):
        # compute the x,y position of each discretized patch on the track with respect to the track center
        patches_longitudinal_position = self.track_right.getPatchesLongitudinalPosition()
        patches_lateral_position      = self.track_right.getPatchesLateralPosition()
        return patches_longitudinal_position, patches_lateral_position

    def getPatchesShearDisplacement(self):
        j_left  = self.track_left.shear_displacement 
        j_right = self.track_right.shear_displacement 
        return j_left,j_right

    def getPatchesShearStress(self):
        tau_left  = self.track_left.shear_stress
        tau_right = self.track_right.shear_stress
        return tau_left, tau_right

    def printForces(self):
        print("Tractive forces")
        print(f"F_left [{self.F_left[0]}, {self.F_left[1]}], F_right [{self.F_left[0]}, {self.F_left[1]}]")

    def printTurningMoments(self):
        print("Resistive turning moments")
        print(f"M_long_left: {self.M_long_left}, M_long_right: {self.M_long_right}")
        print(f"M_lat_left: {self.M_lat_left}, M_lat_right: {self.M_lat_right}")