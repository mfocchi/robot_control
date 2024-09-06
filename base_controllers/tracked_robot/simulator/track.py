import math
import numpy as np
from base_controllers.tracked_robot.simulator.contact_patch import ContactPatch
from  base_controllers.tracked_robot.utils import constants as constants

class TrackParams:
    def __init__(self):
        self.length = 0.5  #[m] track l.ength
        self.width = 0.1  #[m] track width
        self.sprocket_radius = constants.SPROCKET_RADIUS #[m]
        self.A = self.length * self.width  # [m^2] contact area
        self.parts_longitudinal = 60 #4
        self.parts_lateral = 6 #2
        self.d_longitudinal = self.length / self.parts_longitudinal 
        self.d_lateral      = self.width / self.parts_lateral 
        self.dA = self.d_longitudinal * self.d_lateral 

class Track:
    def __init__(self, position, track_param):
        # create and manages the array of contact_patch to simulate the interaction with the terrain

        # self.name
        # self.parts_longitudinal
        # self.parts_lateral
        # self.contact_patches
        # self.position
        # self.dA
        # self.length
        # self.width
        # self.sprocket_radius
        # self.normal_stress
        # self.shear_displacement
        # self.shear_stress


        self.parts_longitudinal = track_param.parts_longitudinal 
        self.parts_lateral = track_param.parts_lateral 
        self.position = position 
        self.length =  track_param.length 
        self.width  = track_param.width 
        self.sprocket_radius = track_param.sprocket_radius 

        self.normal_stress = np.zeros(self.getSizePatches())
        self.shear_displacement = np.zeros(self.getSizePatches())
        #self.shear_velocity = np.np.zeros(self.getSizePatches())
        self.shear_stress = np.zeros(self.getSizePatches())
            
        d_longitudinal = self.length / self.parts_longitudinal
        d_lateral      = self.width / self.parts_lateral

        self.dA = d_longitudinal * d_lateral
        # populate the contact_patches
        self.contact_patches = [None] * ( self.parts_longitudinal * self.parts_lateral)

        for i in range(self.parts_longitudinal):
            for j in range(self.parts_lateral):
                #matlab
                #patch_position_in_track = np.array([self.length/2 - (i-1/2) * d_longitudinal,      self.width/2 - (j-1/2) * d_lateral ])
                #python
                patch_position_in_track = np.array([self.length / 2 - (i + 1 / 2) * d_longitudinal, self.width / 2 - (j + 1 / 2) * d_lateral])

                patch_position = self.position +patch_position_in_track
                index = self.getIndex(i,j)
                self.contact_patches[index] = ContactPatch(patch_position, d_lateral, d_longitudinal)


    def getIndex(self, i, j): #i = 0 j = 0 index =0
        # access a patch in a matrix-like indexing
        #index = (i-1)*self.parts_lateral + j #matlab
        index = i * self.parts_lateral + j  # PYTHON
        return index

    def getParam(self):
        class param:
            pass
        param.length = self.length
        param.width = self.width
        param.sprocket_radius = self.sprocket_radius
        return param

    def computePatchesShearDisplacement(self, inputs, omega_sprocket):
        track_param = self.getParam()

        for i in range(self.parts_longitudinal):
            for j  in range(self.parts_lateral):
                index = (i-1)*self.parts_lateral + j
                self.shear_displacement[i,j] = self.contact_patches[index].computeShearDisplacement(inputs, omega_sprocket, track_param)

    def computePatchesShearStress(self, inputs, omega_sprocket, ground_param):
        track_param = self.getParam()

        for i in range(self.parts_longitudinal):
            for j in range(self.parts_lateral):
                index = self.getIndex(i,j)
                sigma = self.normal_stress[i,j]
                self.shear_stress[i,j] = self.contact_patches[index].computeShearStressFirmGround(inputs, omega_sprocket, track_param, ground_param, sigma)

    def computePatchesCosSinShearAngle(self, inputs, omega_sprocket):
        cos_shear_angles = np.zeros(self.getSizePatches())
        sin_shear_angles = np.zeros(self.getSizePatches())
        track_param = self.getParam()

        for i in range(self.parts_longitudinal):
            for j in range(self.parts_lateral):
                index = self.getIndex(i,j)
                cos_shear_angles[i,j], sin_shear_angles[i,j] = self.contact_patches[index].computeCosSinShearAngle(inputs, omega_sprocket, track_param)
        return  cos_shear_angles, sin_shear_angles

    def computeTractiveForce(self, inputs, omega_sprocket, ground):
        # compute the total tractive force that the track can apply on
        # the ground given all the parameters and control inputs

        [cos_shear_angles, sin_shear_angles] = self.computePatchesCosSinShearAngle(inputs, omega_sprocket)
        self.computePatchesShearStress(inputs, omega_sprocket, ground)

        dFx = -self.dA * np.multiply(self.shear_stress, cos_shear_angles)
        dFy = -self.dA * np.multiply(self.shear_stress, sin_shear_angles)

        Fx = sum(dFx, "all")
        Fy = sum(dFy, "all")
        return Fx, Fy

    def computeResistiveTruningMoments(self, inputs, omega_sprocket, ground):
        # The moments of turning resistance Mr Ml due to the lateral shear
        # forces acting on the track

        cos_shear_angles, sin_shear_angles = self.computePatchesCosSinShearAngle(inputs, omega_sprocket)
        self.computePatchesShearStress(inputs, omega_sprocket, ground)

        patches_y = self.getPatchesLongitudinalPosition()
        patches_x = self.getPatchesLateralPosition()


        dM_long = -self.dA * (patches_y) * self.shear_stress * cos_shear_angles
        dM_lat  = -self.dA * (patches_x) * self.shear_stress * sin_shear_angles

        M_long = sum(dM_long, "all")
        M_lat  = sum(dM_lat, "all")
        return M_long, M_lat

    # SETTERS
    def setNormalStress(self, sigma):
        self.normal_stress = sigma

    # GETTERS

    def getPatchesLongitudinalPosition(self):
        x = np.zeros(self.getSizePatches())
        for i in range(self.parts_longitudinal):
            for j in range(self.parts_lateral):
                index = self.getIndex(i,j)
                x[i,j] = self.contact_patches[index].getX()

        return x

    def getPatchesLateralPosition(self):
        y = np.zeros(self.getSizePatches())
        for i in range(self.parts_longitudinal):
            for j in range(self.parts_lateral):
                index = self.getIndex(i,j)
                y[i,j] = self.contact_patches[index].getY()
        return y


    def getSizePatches(self):
        cols = self.parts_lateral
        rows = self.parts_longitudinal
        size = [rows, cols]
        return size

    def computeTerrainInteractions(self, state, omega_sprocket, track_param, sigma, ground, patch_pos_long, patch_pos_lat):
        r = track_param.sprocket_radius
        L = track_param.length
        mu = ground.friction_coefficient
        K  = ground.K
        dA = track_param.dA

        u = state[0]
        v = state[1]
        omega = state[2]

        if ((u == 0.0) and (v == 0.0) and (omega == 0.0) and (omega_sprocket == 0.0)): # robot idle
            M_long = 0.0
            M_lat  = 0.0
            Fx = 0.0
            Fy = 0.0
            return Fx, Fy, M_long, M_lat

        x = patch_pos_long
        y = patch_pos_lat

        #compute shear displacements  in x y direction (TODO SHOUD BE DIFFERENT FOR LEFT RIGHT TRACK!)
        v_sprocket = omega_sprocket*r
        theta = (omega*(L/2 - x))/(v_sprocket)
        if(omega == 0.0 and omega_sprocket != 0.0):
            j_x = (u - v_sprocket - omega * y) * (L/2 - x) / v_sprocket
            j_y = (v * (L/2 - x) + (omega/2)*(math.pow(L,2)/4 - np.power(x,2))) / v_sprocket
        elif(omega_sprocket == 0.0): # track is locked
            j_x = np.ones_like(sigma) * 1e5
            j_y = np.ones_like(sigma) * 1e5
        else:
            j_x = v_sprocket*x*np.cos(theta) - (L*v_sprocket)/2 - (v_sprocket*v)/omega + (v_sprocket*v*np.cos(theta))/omega + (v_sprocket*np.sin(theta)*(u - omega*y))/omega
            j_y = (v_sprocket*(u - omega*y))/omega + v_sprocket*x*np.sin(theta) + (v_sprocket*v*np.sin(theta))/omega - (v_sprocket*np.cos(theta)*(u - omega*y))/omega

        # compute shear velocities in x y direction
        shear_velocitys_x = u - omega * y - v_sprocket
        shear_velocitys_y = v + omega * x

        #norm of shear displacement
        shear_disp = np.sqrt(np.power(j_x,2) + np.power(j_y,2))
        # norm of shear velocity
        shear_velocitys = np.sqrt(np.power(shear_velocitys_x,2) + np.power(shear_velocitys_y,2))

        shear_angles_sin = shear_velocitys_y / shear_velocitys
        shear_angles_cos = shear_velocitys_x / shear_velocitys

        #compute shear stress
        shear_stress = sigma * mu * (1 - np.exp(-shear_disp / K))

        #compute tractive forces
        dFx = -dA * shear_stress * shear_angles_cos
        dFy = -dA * shear_stress * shear_angles_sin
        #compute turning moments
        dM_long = -y * dFx
        dM_lat  = x * dFy
        #print(dFx)
        #integrate tractive forces and moments
        M_long = np.sum(dM_long)
        M_lat  = np.sum(dM_lat)
        Fx = np.sum(dFx)
        Fy = np.sum(dFy)

        return Fx, Fy, M_long, M_lat
