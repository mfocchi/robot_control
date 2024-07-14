
import numpy as np
from track import Track

class VehicleParam:



class TrackedVehicle:
    def __init__(self, vehicle_param, track_param, ground_param):
    # Simulates a tracked vehicle. Formulas are taken from "theory of
    # ground vehicles" of Wong.
        self.vehicle_param = vehicle_param 
        self.track_left  = Track([0.0, vehicle_param.width / 2], track_param, ground_param)
        self.track_right = Track([0.0,-vehicle_param.width / 2], track_param, ground_param)
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


    def computeShearStress(self):
        self.track_left.computePatchesShearStress()
        self.track_right.computePatchesShearStress()

    def computeShearVelocitys(self,  inputs):
        omega_l = inputs.omega_sprocket_left 
        omega_r = inputs.omega_sprocket_right 
        self.track_left.computeShearVelocitys(inputs,  omega_l) 
        self.track_right.computeShearVelocitys(inputs, omega_r) 

    def computeShearAngles(self):
        self.track_left.computeShearAngle() 
        self.track_right.computeShearAngle() 

    def computeTractiveForces(self):
        self.F_left = self.track_left.computeTractiveForce() 
        self.F_right = self.track_right.computeTractiveForce() 

    def computeResistiveTurningMoments(self):
        self.M_long_left, self.M_lat_left  = self.track_left.computeResistiveTruningMoments() 
        self.M_long_right,self.M_lat_right = self.track_right.computeResistiveTruningMoments() 

    def getLeftPatchesPositions(self, self):
        # compute the x,y position of each discretized patch on the track with respect to the track center
        pataches_longitudinal_position = self.track_left.getPatchesLongitudinalPosition()
        pataches_lateral_position      = self.track_left.getPatchesLateralPosition()
        return pataches_longitudinal_position, pataches_lateral_position

    def getRightPatchesPositions(self, self):
        # compute the x,y position of each discretized patch on the track with respect to the track center
        pataches_longitudinal_position = self.track_right.getPatchesLongitudinalPosition()
        pataches_lateral_position      = self.track_right.getPatchesLateralPosition()
        return pataches_longitudinal_position, pataches_lateral_position

    def getPatchesShearDisplacement(self):
        j_left  = self.track_left.shear_displacement 
        j_right = self.track_right.shear_displacement 
        return j_left,j_right

    def getPatchesShearStress(self):
            tau_left  = self.track_left.shear_stress 
            tau_right = self.track_right.shear_stress 


    # def printForces(self):
    #         disp("Tractive forces")
    #         disp(strcat("F_{left} [ ", string(self.F_left(1)), " , ", string(self.F_left(2)),...
    #             " ] F_{right} [ ", string(self.F_right(1)), " , ", string(self.F_right(2))))
    #
    # def printTurningMoments(self):
    #         disp("Resistive turning moments")
    #         disp(strcat("M_long_left: ", string(self.M_long_left), " M_long_right ", string(self.M_long_right)))
    #         disp(strcat("M_lat_left: ", string(self.M_lat_left), " M_lat_right ", string(self.M_lat_right)))
