# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""
import numpy as np

class AdmittanceControl():
    
    def __init__(self, ikin, Kp, Kd, conf):
        self.ikin = ikin
        self.conf = conf

        self.dx = np.zeros(3)
        self.dx_1_old = np.zeros(3)
        self.dx_2_old = np.zeros(3)
        self.Kp = Kp
        self.Kd = Kd
        self.q_postural =   self.conf['q_0']

    def setPosturalTask(self, q_postural):
        self.q_postural = q_postural

    def computeAdmittanceReference(self, Fext, x_des, q_guess):
        # only Kp, Kd
        self.dx = np.linalg.inv(self.Kp + 1/self.conf['dt'] * self.Kd).dot(Fext + 1/self.conf['dt']*self.Kd.dot(self.dx_1_old))
        #only Kp (unstable)
        #self.dx = np.linalg.inv(self.Kp).dot(Fext)
        self.dx_2_old = self.dx_1_old
        self.dx_1_old = self.dx
        # you need to remember to remove the base offset! because pinocchio us unawre of that!
        q_des, ik_success, out_of_workspace = self.ikin.endeffectorInverseKinematicsLineSearch(x_des   + self.dx,
                                                                                               self.conf['ee_frame'],
                                                                                               q_guess, False, False,
                                                                                               postural_task=True,
                                                                                               w_postural=0.00001,
                                                                                               q_postural= self.q_postural)

        return q_des, x_des + self.dx

