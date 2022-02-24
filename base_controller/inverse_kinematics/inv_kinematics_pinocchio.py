
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os


import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
import yaml
import math

import sys
sys.path.append('../utils')#allows to incude stuff on the same level
from utils import Utils

class robotKinematics():
    def __init__(self, robot, ee_frames):
        self.u = Utils()	
        self.robot = robot

        self.ik_success = False
        self.urdf_feet_names = ee_frames

        # Get feet frame names in an alphabatical order to match pinocchio kinematics
        self.urdf_feet_names_pinocchio = []
        for frame in self.robot.model.frames:
               
            if frame.name in ee_frames:
                self.urdf_feet_names_pinocchio.append(frame.name)   
        
    def getBlockIndex(self, frame_name):
        for i in range(len(self.urdf_feet_names_pinocchio)):
            if frame_name == self.urdf_feet_names_pinocchio[i]:
                idx = i * 3
                break

        return idx

    def computeFootForwardKinematics(self, q_leg, frame_name):
        q = np.zeros((self.robot.na)) #these are actuated joints
        frame_id = self.robot.model.getFrameId(frame_name)
        # Get index of frame to retreive data from Pinocchio variables
        blockIdx = self.getBlockIndex(frame_name)
        q[blockIdx:blockIdx + 3] = q_leg
        q_floating = np.hstack(( pinocchio.neutral(self.robot.model)[0:7], q))
        
        pinocchio.forwardKinematics(self.robot.model, self.robot.data, q_floating)
        pinocchio.framesForwardKinematics(self.robot.model, self.robot.data, q_floating)
        return  self.robot.data.oMf[frame_id].translation
        
    def computeFootJacobian(self, q_leg, frame_name):
        q = np.zeros((self.robot.na)) #these are actuated joints
        frame_id = self.robot.model.getFrameId(frame_name)
        # Get index of frame to retreive data from Pinocchio variables
        blockIdx = self.getBlockIndex(frame_name)
        q[blockIdx:blockIdx + 3] = q_leg
        q_floating = np.hstack(( pinocchio.neutral(self.robot.model)[0:7], q))
        
        J = pinocchio.computeFrameJacobian(self.robot.model, self.robot.data, q_floating, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
     
        return J[:3, 6 + blockIdx : 6 + blockIdx + 3]
        
    def footInverseKinematicsFixedBaseLineSearch(self, foot_pos_des, frame_name, q0_leg = np.zeros(3),  verbose = False):    

      
        # Error initialization
        e_bar = 1
        iter = 0     
             # Recursion parameters
        epsilon = 0.0001  # Tolerance
        # alpha = 0.1
        alpha = 1  # Step size
        lambda_ = 0.00001# Damping coefficient for pseudo-inverse
        max_iter = 6 # Maximum number of iterations
    
        # For line search only
        gamma = 0.5
        beta = 0.5    
       
        # Inverse kinematics with line search
        while True: 
            # compute foot position 
            foot_pos0 = self.computeFootForwardKinematics(q0_leg, frame_name)
            #get the square matrix jacobian that is smaller
            J_leg = self.computeFootJacobian(q0_leg, frame_name)         
           
            # computed error wrt the des cartesian position
            e_bar = foot_pos_des - foot_pos0
            
            
            if np.linalg.norm(e_bar) < epsilon:
                IKsuccess = True
                if verbose:
                    print("IK Convergence achieved!, norm(error) :", np.linalg.norm(e_bar) )
                    print("Inverse kinematics solved in {} iterations".format(iter))     
                break
            if iter >= max_iter:
                if verbose:
                    print(("\n Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is: ", np.linalg.norm(e_bar)))
                IKsuccess = False
                break
             
            #compute newton step
            dq = J_leg.T.dot(np.linalg.solve(J_leg.dot(J_leg.T) + lambda_ * np.identity(J_leg.shape[1]), e_bar))
         
        
            while True:
                # Update
                q1_leg = q0_leg + dq*alpha
                foot_pos1 = self.computeFootForwardKinematics(q1_leg, frame_name)
                   
                #Compute error of next step         
                e_bar_new = foot_pos_des - foot_pos1 
                # print "e_bar_new", np.linalg.norm(e_bar_new), "e_bar", np.linalg.norm(e_bar)
                    
                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)
                threshold = 0.0 # even more strict: gamma*alpha*np.linalg.norm(e_bar)
        
      
                if error_reduction < threshold:
                    alpha = beta*alpha
                    if verbose:
                        print (" line search: alpha: ", alpha) 
                else:
                    q0_leg = q1_leg
                    alpha = 1
                    break                    
            iter += 1
            
        # unwrapping prevents from outputs larger than 2pi
#        for i in range(len(q0_leg)):
#            while q0_leg[i] >= 2 * math.pi:
#                q0_leg[i] -= 2 * math.pi
#            while q0_leg[i] < -2 * math.pi:
#                q0_leg[i] += 2 * math.pi      

        return q0_leg, IKsuccess
        
   
    def fixedBaseInverseKinematics(self, feetPosDes, q0, verbose = False):

        no_of_feet = len(self.urdf_feet_names)
        print("Number of feet is :", no_of_feet)
        q = []
        leg_ik_success = np.zeros((no_of_feet))
        
        for leg in range(no_of_feet):
            '''Compute IK in similar order to feet location variable'''
            if verbose:
                print('Solving IK for leg: ',leg)
            f_p_des = np.array(feetPosDes[leg, :]).T
            q0_leg = self.u.getLegJointState(leg,  q0)   
      
            # q_leg, foot_jac, err, leg_ik_success[leg] = self.footInverseKinematicsFixedBase(f_p_des, self.urdf_feet_names[leg], q0)
            q_leg,  leg_ik_success[leg]= self.footInverseKinematicsFixedBaseLineSearch(f_p_des, self.urdf_feet_names[leg], q0_leg, verbose)
            q = np.hstack([q, q_leg])
            if not leg_ik_success[leg]:            
                print('Warning, IK failed on  leg: ',leg)

        self.ik_success = all(leg_ik_success)

        if self.ik_success is False and verbose is True:
            print('Warning, IK failed in one of the legs')
        return q


    def isOutOfJointLims(self, joint_positions, joint_limits_max, joint_limits_min):

        no_of_legs_to_check = joint_positions.size/3
        q = joint_positions.reshape((no_of_legs_to_check, 3))

        # print "q: ", q
        # print "leq than max ", np.all(np.less_equal(q, joint_limits_max))
        # print "geq than min ", np.all(np.greater_equal(q, joint_limits_min))
        return not np.all(np.less_equal(q, joint_limits_max)) \
               or not np.all(np.greater_equal(q, joint_limits_min))

    def isOutOfWorkSpace(self, contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel):
        q = self.fixedBaseInverseKinematics(contactsBF_check)
        out = self.isOutOfJointLims(q, joint_limits_max, joint_limits_min)
        
        return out