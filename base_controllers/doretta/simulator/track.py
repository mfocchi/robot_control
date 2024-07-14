import numpy as np

class Track:
    def __init__(self, position, track_param, ground):
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

        self.normal_stress = np.np.zeros(self.getSizePatches()) 
        self.shear_displacement = np.np.zeros(self.getSizePatches()) 
        #self.shear_velocity = np.np.zeros(self.getSizePatches())
        self.shear_stress = np.np.zeros(self.getSizePatches()) 
            
        self.ground_param = ground
        d_longitudinal = self.length / self.parts_longitudinal
        d_lateral      = self.width / self.parts_lateral

        self.dA = d_longitudinal * d_lateral
        # populate the contact_patches
        self.contact_patches = np.ones((1, self.parts_longitudinal * self.parts_lateral))

        for i in range(self.parts_longitudinal):
            for j in range(self.parts_lateral):
                patch_position = self.position + np.array([self.length/2 - (i-1/2) * d_longitudinal,      self.width/2 - (j-1/2) * d_lateral ])
                index = self.getIndex(i,j)
                self.contact_patches[index] = contact_patch_Biral(patch_position, d_lateral, d_longitudinal)


    def index = getIndex(self, i, j):
        # access a patch in a matrix-like indexing
        index = (i-1)*self.parts_lateral + j

    def getParam(self)
        class param:
            pass
        param.length = self.length
        param.width = self.width
        param.sprocket_radius = self.sprocket_radius

    def computePatchesShearDisplacement(self, inputs, omega_sprocket):
        track_param = self.getParam()

        for i in range(self.parts_longitudinal):
            for j  in range(self.parts_lateral):
                index = (i-1)*self.parts_lateral + j
                self.shear_displacement(i,j) = self.contact_patches{index}.computeShearDisplacement(inputs, omega_sprocket, track_param)

    def computePatchesShearStress(self, inputs, omega_sprocket):
        track_param = self.getParam()

        for i in range(self.parts_longitudinal):
            for j in range(self.parts_lateral):
                index = self.getIndex(i,j)
                sigma = self.normal_stress(i,j)
                self.shear_stress(i,j) = self.contact_patches{index}.computeShearStressFirmGround(inputs, omega_sprocket, track_param, self.ground_param, sigma)

        def computePatchesCosSinShearAngle(self, inputs, omega_sprocket)
            cos_shear_angles = np.zeros(self.getSizePatches()) 
            sin_shear_angles = np.zeros(self.getSizePatches()) 
            track_param = self.getParam() 

            for i in range(self.parts_longitudinal):
                for j in range(self.parts_lateral):
                    index = self.getIndex(i,j) 
                    [cos_shear_angles(i,j), sin_shear_angles(i,j)] = ...
                        self.contact_patches{index}.computeCosSinShearAngle(...
                        inputs, omega_sprocket, track_param) 
            return  cos_shear_angles, sin_shear_angles

        def computeTractiveForce(self, inputs, omega_sprocket):
            # compute the total tractive force that the track can apply on
            # the ground given all the parameters and control inputs

            [cos_shear_angles, sin_shear_angles] = self.computePatchesCosSinShearAngle(inputs, omega_sprocket) 
            self.computePatchesShearStress(inputs, omega_sprocket) 
            
            dFx = -self.dA * self.shear_stress .* cos_shear_angles 
            dFy = -self.dA * self.shear_stress .* sin_shear_angles 
            
            Fx = sum(dFx, "all") 
            Fy = sum(dFy, "all") 
            return Fx, Fy

        def computeResistiveTruningMoments(self, inputs, omega_sprocket)
            # The moments of turning resistance Mr Ml due to the lateral shear
            # forces acting on the track

            cos_shear_angles, sin_shear_angles = self.computePatchesCosSinShearAngle(inputs, omega_sprocket)
            self.computePatchesShearStress(inputs, omega_sprocket) 

            patches_y = self.getPatchesLongitudinalPosition() 
            patches_x = self.getPatchesLateralPosition() 
            

            dM_long = -self.dA * (patches_y) .* self.shear_stress .* cos_shear_angles 
            dM_lat  = -self.dA * (patches_x) .* self.shear_stress .* sin_shear_angles 
            
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
                    x[i,j] = self.contact_patches{index}.getX()
            return x

        def getPatchesLateralPosition(self):
            y = np.zeros(self.getSizePatches()) 
            for i = 1:self.parts_longitudinal
                for j = 1:self.parts_lateral
                    index = self.getIndex(i,j) 
                    y[i,j] = self.contact_patches{index}.getY()
            return y

            
        def getSizePatches(self):
            cols = self.parts_lateral 
            rows = self.parts_longitudinal 
            size = [rows, cols] 
            return size