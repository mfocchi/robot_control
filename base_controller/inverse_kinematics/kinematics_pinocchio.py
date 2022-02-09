
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys

import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
import yaml
import math

class robotKinematics():
    def __init__(self, robot, robotName):

        self.PKG = os.path.dirname(os.path.abspath(__file__)) + '/{}/'.format(robotName)
        self.URDF = self.PKG + 'urdf/{}.urdf'.format(robotName)

        self.FEET = self.PKG + 'robot_data.yaml'
        self.model = robot.model
        self.data = robot.data
        self.feet_jac = None
        self.ik_success = False

        yaml_data = []
        self.urdf_feet_names = []
        self.default_q = []
        ## Can be used to compute q in an feet order similar to feet variables
        # Get feet frame names in a similar order to feet variables (position, etc...)
        with open(self.FEET, 'r') as stream:
            try:
                yaml_data = yaml.safe_load(stream)

            except yaml.YAMLError as exc:
                print(exc)
        
        self.urdf_feet_names = yaml_data['Feet_names']
        self.default_q = yaml_data['Default_q']
        self.default_q = np.vstack(self.default_q)
        self.q = self.default_q   # Last q computed
        # Get feet frame names in an alphabatical order to match pinocchio kinematics
        self.urdf_feet_names_pinocchio = []
        for frame in self.model.frames:
            if frame.name in self.urdf_feet_names:
                self.urdf_feet_names_pinocchio.append(frame.name)   

    def getBlockIndex(self, frame_name):
        for i in range(len(self.urdf_feet_names_pinocchio)):
            if frame_name == self.urdf_feet_names_pinocchio[i]:
                idx = i * 3
                break

        return idx

    def computeFootForwardKinematics(self, q_leg, frame_name):
        q = np.zeros((self.model.nq))
        frame_id = self.model.getFrameId(frame_name)
        # Get index of frame to retreive data from Pinocchio variables
        blockIdx = self.getBlockIndex(frame_name)
        
        q[blockIdx:blockIdx + 3] = q_leg
        pinocchio.forwardKinematics(self.model, self.data, q)
        pinocchio.framesForwardKinematics(self.model, self.data, q)
        return  self.data.oMf[frame_id].translation
        
    def computeFootJacobian(self, q_leg, frame_name):
        q = np.zeros((self.model.nq))
        frame_id = self.model.getFrameId(frame_name)
        # Get index of frame to retreive data from Pinocchio variables
        blockIdx = self.getBlockIndex(frame_name)
        q[blockIdx:blockIdx + 3] = q_leg
        
        J = pinocchio.computeFrameJacobian(self.model, self.data, q, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return J[:3,blockIdx:blockIdx + 3]
        
    def footInverseKinematicsFixedBaseLineSearch(self, foot_pos_des, frame_name, q0_leg = np.zeros(3), verbose = False):    

      
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
        for i in range(len(q0_leg)):
            while q0_leg[i] >= 2 * math.pi:
                q0_leg[i] -= 2 * math.pi
            while q0_leg[i] < -2 * math.pi:
                q0_leg[i] += 2 * math.pi      

        return q0_leg, J_leg, IKsuccess
        
    #### OLD                   
    def footInverseKinematicsFixedBase(self, foot_pos_des, frame_name, q0=np.zeros(12)):
        frame_id = self.model.getFrameId(frame_name)
        # Get index of frame to retreive data from Pinocchio variables
        blockIdx = self.getBlockIndex(frame_name)
        # anymal_q0 = np.vstack(self.default_q)
        # q = anymal_q0.ravel()
        eps = 0.005
        IT_MAX = 200
        DT = 1e-1
        err = np.zeros((3, 1))
        e = np.zeros((3, 1))
        damp = 1e-12

        # alpha = 1  # Step size
        # # For line search only
        # gamma = 0.5
        # beta = 0.5
        q = q0

        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, self.data, q)
            pinocchio.framesForwardKinematics(self.model, self.data, q)
            foot_pos = self.data.oMf[frame_id].translation
            # err = np.hstack([err, (foot_pos - foot_pos_des)])
            # e = err[:,-1]
            # print foot_pos_des[0], foot_pos[[0]], foot_pos[[0]] - foot_pos_des[0]
            # e = foot_pos - foot_pos_des
            e[0] = foot_pos[[0]] - foot_pos_des[0]
            e[1] = foot_pos[[1]] - foot_pos_des[1]
            e[2] = foot_pos[[2]] - foot_pos_des[2]
            J = pinocchio.computeFrameJacobian(self.model, self.data, q, frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)

            J_lin = J[:3, :]

            if np.linalg.norm(e) < eps:
                # print("IK Convergence achieved!")
                IKsuccess = True
                break
            if i >= IT_MAX:
                print((
                    "\n Warning: the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                    np.linalg.norm(e)))
                IKsuccess = False
                break
            # print J_lin
            v = - J_lin.T.dot(np.linalg.solve(J_lin.dot(J_lin.T) + damp * np.eye(3), e))
            q = pinocchio.integrate(self.model, q, v * DT)
            # q_next = q + v*alpha

            ## Not working - Alternative method to integrate q
            # #Compute error of next step
            # pinocchio.forwardKinematics(self.model, self.data, q_next)
            # pinocchio.framesForwardKinematics(self.model, self.data, q_next)
            # foot_pos_next = self.data.oMf[frame_id].translation
            # e_next = np.zeros(3)
            # e_next[0] = foot_pos_next[[0]] - foot_pos_des[0]
            # e_next[1] = foot_pos_next[[1]] - foot_pos_des[1]
            # e_next[2] = foot_pos_next[[2]] - foot_pos_des[2]
            # print("E: ", e)
            # print("E next: ", e_next)
            # e_check = np.linalg.norm(e_next) - np.linalg.norm(e)
            # threshold = gamma*alpha*np.linalg.norm(e)
            # if e_check <= threshold:
            #     alpha = beta*alpha
            # q = q_next 
            # print i
            i += 1

        q_leg = q[blockIdx:blockIdx + 3]
        J_leg = J_lin[:, blockIdx:blockIdx + 3]
        return q_leg.ravel(), J_leg, err, IKsuccess


    def fixedBaseInverseKinematics(self, feetPosDes, q0 = None, verbose = False):

        no_of_feet = len(self.urdf_feet_names)
        self.feet_jac = []
        q = []
        leg_ik_success = np.zeros((no_of_feet))
        
        if (q0 is None):
#            q0 =np.vstack((np.array([-0.2, 0.75, -1.5]), 
#                np.array([-0.2, 0.75, -1.5]), 
#                np.array([-0.2, -0.75, 1.5]),
#                np.array([-0.2, -0.75, 1.5])))  
#             q0 = [-0.1, 0.75, -1.5, -0.1, 0.75, -1.5, -0.1, 0.75, -1.5, -0.1, 0.75, -1.5]
#             q0 = np.vstack(self.default_q)
#             q0 = q0.ravel()

            q0 = self.default_q.ravel()

        for leg in range(no_of_feet):
            '''Compute IK in similar order to feet location variable'''
            f_p_des = np.array(feetPosDes[leg, :]).T
            # q_leg, foot_jac, err, leg_ik_success[leg] = self.footInverseKinematicsFixedBase(f_p_des, self.urdf_feet_names[leg], q0)
            q_leg, foot_jac, leg_ik_success[leg]= self.footInverseKinematicsFixedBaseLineSearch(f_p_des, self.urdf_feet_names[leg], q0[leg*3 : leg*3+3], verbose)

            q = np.hstack([q, q_leg])
            self.feet_jac.append(foot_jac)


        self.ik_success = all(leg_ik_success)

        if self.ik_success is False and verbose is True:
            print('Warning, IK failed in one of the legs')
        return q

    def getLegJacobians(self):
        isOutOfWS = not self.ik_success
        return self.feet_jac, isOutOfWS

    def getCurrentQ(self):

        return self.q

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
        if not out:
            self.q = q

        return self.isOutOfJointLims(q, joint_limits_max, joint_limits_min)