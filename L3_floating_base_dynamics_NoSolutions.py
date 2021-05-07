
import pinocchio as pin
import numpy as np
import example_robot_data
from pinocchio.utils import * #rand
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *


# console print options to see matrix nicely
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)

# Loading a robot model
model = example_robot_data.loadHyQ().model
data = model.createData()

#start configuration
v  = np.array([0.0   ,  0.0 , 0.0,  0.0,  0.0,       0.0, #underactuated 	
		     0.0,  0.0,  0.0,  0.0,     0.0,  0.0,  0.0,  0.0,  0.0,    0.0,  0.0,  0.0]) #actuated
q = example_robot_data.loadHyQ().q0

# Update the joint and frame placements
pin.forwardKinematics(model,data,q,v)
pin.updateFramePlacements(model,data)

M =  pin.crba(model, data, q)
H = pin.nonLinearEffects(model, data, q, v)
G = pin.computeGeneralizedGravity(model,data, q)

#EXERCISE 1: Compute the com of the robot (in WF)
#.....

# compare the result by using native pinocchio function
com_test = pin.centerOfMass(model, data, q, v)
print "Com Position (pinocchio): ", com_test

# EXERCIZE 2: Compute robot kinetic energy
#get a random generalized velocity 
v = rand(model.nv)
# Update the joint and frame placements
pin.forwardKinematics(model,data,q,v)
# compute using generalized velocities and system mass matrix
#  .....

## compare with pinocchio native function
pin.computeKineticEnergy(model,data,q,v)
#print "TEST2: ", EkinRobot - data.kinetic_energy


##EXERCISE 3: Build the transformation matrix to use com coordinates
# get the location of the base frame
w_base = data.oMi[1].translation
#compute centroidal quantitities (hg, Ag and Ig)
pin.ccrba(model, data, q, v)

# G_T_B = ...


# EXERCISE 4: Check the mass matrix becomes block diagonal if you appy the tranform
# M_g = ...

#print "\n The mass matrix expressed at the com becomes diagonal: \n", M_g


# EXERSISE 5: Check that joint gravity vector nullifies
# Gg = ...
