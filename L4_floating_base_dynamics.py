
import pinocchio
import numpy as np
import example_robot_data
from pinocchio.utils import * #rand
from pinocchio.robot_wrapper import RobotWrapper

#important
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
# Loading a robot model
model = example_robot_data.loadHyQ().model
data = model.createData()

# Generating a random configuration and velocity
q = pinocchio.randomConfiguration(model,-np.ones(model.nq),np.ones(model.nq))
#v = np.random.rand(model.nv)

v  = np.array([0.0   ,  0.0 , 1.0,  0.0,  1.0,       0.0, 	0.0,  0.0,  0.0,  0.0,     0.0,  0.0,  0.0,  0.0,  0.0,    0.0,  0.0,  0.0])
q = example_robot_data.loadHyQ().q0


#pitch 
#matrixToRpy(rotate('x',0.4) )
#q[4] = pinocchio.utils.XYZQUATToSe3(np.array([0 ,0,0]))

# Update the joint and frame placements
pinocchio.forwardKinematics(model,data,q,v)
pinocchio.updateFramePlacements(model,data)


# Get frame ID and its parent joint
frame_id = model.getFrameId("lf_foot")
frame = model.frames[frame_id]
parent_joint = frame.parent


# Get the frame SE3 (from joint to frame point)
iMf = frame.placement


# Let's compute the frame placement expressed in the world
oMf = data.oMi[parent_joint].act(iMf) # same as data.oMi[parent_joint] * iMf


# Let's compute the spatial body velocity at the frame coordinate
print("spatial velocity expressed in the frame coordinate system (f_v)")
f_v1 = pinocchio.getFrameVelocity(model, data, frame_id)
f_v2 = iMf.actInv(data.v[parent_joint]) #this is the shifting law that maps the twist from joint origin evaluatig it in foot origin
print(f_v1)
print(f_v2)


# Let's compute the spatial body velocity at the world coordinate
print("spatial velocity expressed in the world coordinate system (o_v)")
o_v1 = oMf.act(f_v1) #same as o_v1 = oMf * f_v1

o_v2 = data.oMi[parent_joint].act(data.v[parent_joint])# same oMi[parent_joint] * data.v[parent_joint]
print(o_v1)
print(o_v2)


print("euclidean frame velocity w.r.t. the world and expressed in world frame (o_v_wf)")
fiMo = pinocchio.SE3(data.oMi[parent_joint].rotation.transpose(), iMf.translation)
o_v1_wf = fiMo.actInv(data.v[parent_joint])


oMff = pinocchio.SE3(data.oMf[frame_id].rotation, np.zeros(3))
o_v2_wf = oMff.act(f_v2)
print(o_v1_wf)
print(o_v2_wf)


M =  pinocchio.crba(model, data, q)
H = pinocchio.nonLinearEffects(model, data, q, v)
G = pinocchio.computeGeneralizedGravity(model,data, q)


# EXERCIZE 1: test M * [0 0 -g  zeros()] = g
#Grav_W = np.hstack( ( np.array((0.0, 0.0, -9.81)), np.zeros( model.nv - 3)  )).T
#Gtest = -M.dot(Grav_W)
#print (Gtest - G   )
#print (G)

#EXERCIZE 2 : test kinetic energy
#EkinRobot = 0.5*v.transpose().dot(M.dot(v))
#
#EKinSystem= 0
#v = rand(model.nv)
## Update the joint and frame placements
#pinocchio.forwardKinematics(model,data,q,v)
##get the spatial velocities of each link
#twists = [f for f  in data.v]
#inertias = [f for f  in model.inertias]
#for idx, inertia in enumerate(inertias):
#	EKinSystem += inertias[idx].vtiv(twists[idx]) # twist.T I twist
#EKinSystem *= .5;
#print(EKinSystem - EkinRobot)
#
#pinocchio.computeKineticEnergy(model,data,q,v)
#print(data.kinetic_energy)


#robot = RobotWrapper(model)
#print (robot.com(q))
#EXERCIZE 3 compute the com of the robot

com_test = pinocchio.centerOfMass(model, data, q, v)
mass_robot = 0
w_com_robot =np.zeros((3))
for idx,name in enumerate(model.joints): 
	if (idx>0):			 

		mass_link = data.mass[idx]
		mass_robot+= mass_link
		#com_link = data.oMi[idx].act(model.inertias[idx].lever)				
		com_link =  model.inertias[idx].lever		
		w_com_link = data.oMi[idx].rotation.dot(com_link) + data.oMi[idx].translation		
		w_com_robot +=  data.mass[idx] * w_com_link

w_com_robot /=mass_robot
#print (w_com_robot - com_test)


print "Com Position w_com_robot: ", com_test



#EXERCIZE 4 : show that equation of motion is decoupled if you do a com coordinate transform
#build the transfoprmation matrix
from pinocchio.utils import *

#compute centroidal quantitities (hg, Ag and Ig), corresponding to the centroidal momentum, the centroidal map and the centroidal rigid inertia
pinocchio.ccrba(model, data, q, v)
w_base = data.oMi[1].translation
print "Base Position w_base  ", w_base
G_T_B = np.zeros((model.nv, model.nv))
G_Tf_B = np.zeros((model.nv, model.nv))
G_X_B = np.zeros((6, 6))

#G_X_B =
G_X_B[:3,:3] = np.eye(3)
G_X_B[3:,3:] = np.eye(3)
G_X_B[:3,3:] = pinocchio.skew(com_test - w_base)
G_Xf_B = np.linalg.inv(G_X_B.T)



F_B = M[:6, 6:]

S_G_B = np.linalg.inv(data.Ig).dot(G_Xf_B.dot(F_B))
#G_T_B
G_T_B[:6 , :6] = G_X_B
G_T_B[6: , 6:] = np.eye(12)
G_T_B[:6 , 6:] = S_G_B


#G_Tf_B#
G_Tf_B[:6 , :6] = G_Xf_B
G_Tf_B[6: , 6:] = np.eye(12)
G_Tf_B[6: , :6] = -(S_G_B.T).dot(G_Xf_B)

#check G_Tf_B = inv(G_T_B.T)
#print np.linalg.inv(G_T_B.T) - G_Tf_B

M_g = G_Tf_B * M * np.linalg.inv(G_T_B)
#print "\n The mass matrix expressed at the com becomes diagonal: \n", M_g

Grav_W = np.hstack( ( np.array((0.0, 0.0, -9.81)), np.zeros( model.nv - 3)  )).T
#G_g = -M_g.dot(Grav_W) 

#G_g = G_T_B.T.dot(G) #with this it nullifies angular part I dont know why
G_g = G_Tf_B.dot(G) 


#the fact that all become zero it makes sense if you think the com is not a point solidal woth the base link
# but moves with joints, this means that its jacobians has a part from the base and from the joints. 
# so if we try to turn the floating base robot to a fixed base applying th wrench of God we will have also an influence on the joint torques that will be cancelled
print "\n The gravity force vector at the com should be  [0 0 mg   03x1    0nx1 ]: \n", G_g
 