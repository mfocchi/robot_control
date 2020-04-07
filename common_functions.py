# -*- coding: utf-8 -*-
"""
Created on Thu Apr  2 18:07:44 2020

@author: mfocchi
"""
import os
import psutil
#from pinocchio.visualize import GepettoVisualizer
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
 

REF_SPHERE_RADIUS = 0.03
EE_SPHERE_COLOR  = (1, 0.5, 0, 0.5)
EE_REF_SPHERE_COLOR  = (1, 0, 0, 0.5)

def importDisplayModel(DISPLAY, DISPLAY_FLOOR):
    
    
    # Import the model
    ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
    path      = os.environ.get('UR5_MODEL_DIR', ERROR_MSG)
    urdf      = path + "/ur_description/urdf/ur5_gripper.urdf";
    srdf      = path + '/ur5_description/srdf/ur5_gripper.srdf'
    robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
    
    if DISPLAY:
        import commands
        import gepetto
        from time import sleep
                                
#        for proc in psutil.process_iter():
#                                        
#         
#         # check whether the process name matches
#        # print(proc.name())
#    
#            if (proc.name() == 'gepetto-gui'):
#                print('killing ', proc.name())
#                proc.kill()         
 
        
        l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
        if int(l[1]) == 0:
            os.system('gepetto-gui &')
        sleep(1)
        gepetto.corbaserver.Client()
        robot.initViewer(loadModel=True)
        gui = robot.viewer.gui
        gui.addSphere('world/ee', 0.05, EE_SPHERE_COLOR)
        if(DISPLAY_FLOOR):
            gui.createSceneWithFloor('world')
            gui.setLightingMode('world/floor', 'ON')
        robot.displayCollisions(False)
        robot.displayVisuals(True)
                    
                    
    return robot                    

def plot(name, time_log, q_des_log, q_log, qd_des_log, qd_log, qdd_des_log, qdd_log, num_samples):
    plot_var_log = np.zeros((6,num_samples))
    plot_var_des_log = np.zeros((6,num_samples))

    if name == 'pos':
        plot_var_log[:,:] = q_log[:,:]
        plot_var_des_log[:,:] = q_des_log
    elif name == 'vel':
        plot_var_log[:,:] = qd_log[:,:]
        plot_var_des_log[:,:] = qd_des_log[:,:]
    else:
        plot_var_log[:,:] = qdd_log[:,:]
        plot_var_des_log[:,:] = qdd_des_log

    lw_des=7
    lw_act=4  

    plt.figure(1)
    plt.subplot(3,2,1)
    plt.title("1_joint")    
    plt.plot(time_log, plot_var_des_log[0,:],linestyle='-', lw=lw_des,color = 'red')
    plt.plot(time_log, plot_var_log[0,:],linestyle='-', lw=lw_act,color = 'blue')
    plt.grid()
    
    plt.subplot(3,2,2)
    plt.title("2_joint")
    plt.plot(time_log, plot_var_des_log[1,:],linestyle='-', lw=lw_des,color = 'red', label="q_des")
    plt.plot(time_log, plot_var_log[1,:],linestyle='-',lw=lw_act, color = 'blue', label="q")
    plt.legend(bbox_to_anchor=(-0.01, 1.115, 1.01, 0.115), loc=3, mode="expand")
    plt.grid()
    
    plt.subplot(3,2,3)
    plt.title("3_joint")    
    plt.plot(time_log, plot_var_des_log[2,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, plot_var_log[2,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()    
    
    plt.subplot(3,2,4)
    plt.title("4_joint")    
    plt.plot(time_log, plot_var_des_log[3,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, plot_var_log[3,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    
    plt.subplot(3,2,5)
    plt.title("5_joint")    
    plt.plot(time_log, plot_var_des_log[4,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, plot_var_log[4,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    
    plt.subplot(3,2,6)
    plt.title("6_joint") 
    plt.plot(time_log, plot_var_des_log[5,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, plot_var_log[5,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()