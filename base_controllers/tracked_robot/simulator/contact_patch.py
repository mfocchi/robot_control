
import numpy as np

class ContactPatch:
    def __init__(self, position,length_x, length_y):

    # simulate a contact area of custom size. This should be use to
    # discretize the integral of Wong's general theory in "Theory of ground
    # vehicles"
        
        self.position = position
        self.length_x = length_x
        self.length_y = length_y
        self.area  = length_x * length_y 

    def computeShearDisplacement(self, inputs, omega_sprocket, track_param):
        x = self.position[0] 
        y = self.position[1] 
        r = track_param.sprocket_radius 
        l = track_param.length 
        Omega = inputs.Omega 
        u = inputs.u 
        v = inputs.v 

        common_term = 1/(omega_sprocket*r) * (l/2 - x) 
#             j_x = common_term * (u - Omega * y - 1) 
#             j_y = common_term * (v + Omega / 2 * (l/2 - x)) 
        if(self.isLeftTrack()):
            j_x = common_term * (u - Omega * y - omega_sprocket*r) 
            j_y = common_term * (v + Omega / 2 * (l/2 - x)) 
        elif(self.isRightTrack()):
            j_x = common_term * (u + Omega * y - omega_sprocket*r) 
            j_y = common_term * (v + Omega / 2 * (l/2 - x)) 
        shear_displacement = np.sqrt(np.power(j_x,2) + np.power(j_y,2))

    def computeShearStressFirmGround(self, inputs, omega_sprocket, track_param, ground_param, simga):
        # compute the shear stress in case of firm ground. In this
        # case, the maximum shear stress depends on the normal stress (sigma)
        # and the friction coefficient
        mu = ground_param.friction_coefficient
        K  = ground_param.K

        shear_displacement = self.computeShearDisplacement(inputs, omega_sprocket, track_param)

        shear_stress = simga * mu * (1 - np.exp(-shear_displacement / K))
        return shear_stress

    def  computeCosSinShearAngle(self, inputs, omega_sprocket, track_param):
        # calculate the cosine and sine of the shear velocity

        [shear_velocity_x, shear_velocity_y] = self.computeShearVelocity(inputs, omega_sprocket, track_param)

        shear_velocity   = np.sqrt(np.power(shear_velocity_x,2) + np.power(shear_velocity_y,2))

        shear_angle_sin = shear_velocity_y / shear_velocity
        shear_angle_cos = shear_velocity_x / shear_velocity
        return shear_angle_cos,shear_angle_sin
        
    def computeShearVelocity(self, inputs, omega_sprocket, track_param):
        # compute the shear velocity of every discretized area (patch)
        # ofthe track
        x = self.position[0]
        y = self.position[1]
        r = track_param.sprocket_radius
        Omega = inputs.Omega
        u = inputs.u
        v = inputs.v
            
#             shear_velocity_x = u - Omega * y - r * omega_sprocket 
#             shear_velocity_y = v + Omega * x 
        if(self.isLeftTrack()):
            shear_velocity_x = u - Omega * y - r * omega_sprocket
            shear_velocity_y = v + Omega * x
        elif(self.isRightTrack()):
            shear_velocity_x = u + Omega * y - r * omega_sprocket
            shear_velocity_y = v + Omega * x
        return shear_velocity_x, shear_velocity_y

    def isRightTrack(self):
        return self.position[1] <= 0


    def isLeftTrack(self):
        return self.position[1] > 0


    def getY(self):
        return self.position[1]


    def getX(self):
        return self.position[0]

