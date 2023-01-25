# -*- coding: utf-8 -*-
"""
Created on Thu Apr  2 18:07:44 2020

@author: mfocchi
"""
import os
import psutil
#from pinocchio.visualize import GepettoVisualizer
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

import sys
from termcolor import colored
import rospkg
import rospy as ros
import rosnode
import roslaunch
import rosgraph
from roslaunch.parent import ROSLaunchParent
import copy

#from urdf_parser_py.urdf import URDF
#make plot interactive
plt.ion()
plt.close() 

lw_des=7
lw_act=4   
marker_size= 0   


class Twist:
    linear = np.empty((3))*np.nan
    angular = np.empty((3))*np.nan
    def set(self, value):
        self.linear = copy.deepcopy(value[:3])
        self.angular = copy.deepcopy(value[3:])

class Pose:
    position = np.empty((3))*np.nan
    orientation = np.empty((3))*np.nan
    def set(self, value):
        self.position = copy.deepcopy(value[:3])
        self.orientation = copy.deepcopy(value[3:])
            
    
class State:
    
    def __init__(self, desired = False):
        self.pose = Pose()
        self.twist = Twist()
        if (desired):
            self.accel = Twist()
            
    def set(self, value):
        self.pose.set(value.getPose())
        self.twist.set(value.getTwist())
            
    def getPose(self):
        return np.hstack([self.pose.position, self.pose.orientation]) 
            
    def getTwist(self):
        return np.hstack([self.twist.linear, self.twist.angular])
        
def checkRosMaster():
    if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
        print(colored('ROS MASTER is Online','red'))
    else:
        print(colored('ROS MASTER is NOT Online, Starting roscore!!','red'))
        parent = ROSLaunchParent("roscore", [], is_core=True)  # run_id can be any string
        parent.start()

def startNode(node_name):
    
    nodes = rosnode.get_node_names()
    if "/reference_generator" in nodes:
        print(colored("Re Starting ref generator","red"))
        os.system("rosnode kill /"+node_name)
    package = node_name
    executable = node_name
    name = node_name
    namespace = ''
    node = roslaunch.core.Node(package, executable, name, namespace, output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)


def getRobotModel(robot_name="hyq", generate_urdf = False, xacro_path = None, additional_urdf_args = None):
    ERROR_MSG = 'You should set the environment variable LOCOSIM_DIR"\n';
    path  = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
    srdf      = path + "/robot_urdf/" + robot_name + ".srdf"

    if (generate_urdf):  
        try:       
            #old way
            if (xacro_path is None):
                xacro_path = rospkg.RosPack().get_path(robot_name+'_description')+ '/robots/'+robot_name+'.urdf.xacro'
            
            package = 'xacro'
            executable = 'xacro'
            name = 'xacro'
            namespace = '/'
            # with gazebo 11 you should set in the ros_impedance_controllerXX.launch the new_gazebo_version = true
            # note we generate the urdf with the floating base joint (new gazebo version should be false by default in the xacro of the robot! because Pinocchio needs it!
            args = xacro_path+ ' --inorder -o '+os.environ['LOCOSIM_DIR']+'/robot_urdf/generated_urdf/'+robot_name+'.urdf'
     
     
       
            try:
                flywheel = ros.get_param('/flywheel4')
                args+=' flywheel4:='+flywheel
            except:
                pass

            try:
                flywheel2 = ros.get_param('/flywheel2')
                args += ' flywheel2:=' + flywheel2
            except:
                pass

            try:
                angle = ros.get_param('/angle_deg')
                args += ' angle_deg:=' + angle
            except:
                pass

            try:
                anchorZ = ros.get_param('/anchorZ')
                args += ' anchorZ:=' + anchorZ
            except:
                pass

            if additional_urdf_args is not None:
                args += ' '+additional_urdf_args
            
            os.system("rosrun xacro xacro "+args)  
            #os.system("rosparam get /robot_description > "+os.environ['LOCOSIM_DIR']+'/robot_urdf/'+robot_name+'.urdf')  
            #urdf = URDF.from_parameter_server()
            print("URDF generated_commons")
            urdf_location      = path + "/robot_urdf/generated_urdf/" + robot_name+ ".urdf"
            print(urdf_location)
            robot = RobotWrapper.BuildFromURDF(urdf_location)
            print("URDF loaded in Pinocchio")
        except:
            print ('Issues in URDF generation for Pinocchio, did not succeed')
    else:

        urdf      = path + "/robot_urdf/" + robot_name+ ".urdf"
        robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
    
    return robot                    

def plotJoint(name, figure_id, time_log, q_log=None, q_des_log=None, qd_log=None, qd_des_log=None, qdd_log=None, qdd_des_log=None, tau_log=None, tau_ffwd_log = None, tau_des_log = None, joint_names = None, q_adm = None):
    plot_var_des_log = None
    if name == 'position':
        unit = '[rad]'
        plot_var_log = q_log
        if   (q_des_log is not None):
            plot_var_des_log = q_des_log
        else:
            plot_var_des_log = None
    elif name == 'velocity':
        unit = '[rad/s]'
        plot_var_log = qd_log
        if   (qd_des_log is not None):
            plot_var_des_log  = qd_des_log
        else:
            plot_var_des_log = None
    elif name == 'acceleration':
        unit = '[rad/s^2]'
        plot_var_log = qdd_log
        if   (qdd_des_log is not None):
            plot_var_des_log  = qdd_des_log
        else:
            plot_var_des_log = None
    elif name == 'torque':
        unit = '[Nm]'
        plot_var_log = tau_log
        if   (tau_des_log is not None):
            plot_var_des_log  = tau_des_log
        else:
          plot_var_des_log = None                                                
    else:
       print(colored("plotJoint error: wrong input string", "red") )
       return                                   

    

    njoints = min(plot_var_log.shape)

    #neet to transpose the matrix other wise it cannot be plot with numpy array    
    fig = plt.figure(figure_id)                
    fig.suptitle(name, fontsize=20)

    labels_ur = ["1 - Shoulder Pan", "2 - Shoulder Lift", "3 - Elbow", "4 - Wrist 1", "5 - Wrist 2", "6 - Wrist 3"]
    labels_hyq = ["LF_HAA", "LF_HFE","LF_KFE","RF_HAA", "RF_HFE","RF_KFE","LH_HAA", "LH_HFE","LH_KFE","RH_HAA", "RH_HFE","RH_KFE"]
    labels_flywheel2 = ["LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                        "RH_HAA", "RH_HFE", "RH_KFE", "left_wheel", "right_wheel"]
    labels_flywheel4 = ["LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                        "RH_HAA", "RH_HFE", "RH_KFE",
                        "back_wheel", "front_wheel", "left_wheel", "right_wheel"]

    if joint_names is None:
        if njoints <= 6:
            labels = labels_ur
        if njoints == 12:
            labels = labels_hyq
        if njoints == 14:
            labels = labels_flywheel2
        if njoints == 16:
            labels = labels_flywheel4
    else:
        labels = joint_names

    for jidx in range(njoints):
        if (njoints % 3 == 0): #divisible by 3
            plt.subplot(int(njoints / 3), 3, jidx + 1)
            if jidx + 3 >= njoints:
                plt.xlabel("Time [s]")
        else:  # divisible by 2
            plt.subplot(int(njoints / 2), 2, jidx + 1)
            if jidx + 2 >= njoints:
                plt.xlabel("Time [s]")
        plt.ylabel(labels[jidx] + ' '+ unit)
        if name == 'torque' and tau_ffwd_log is not None:
            plt.plot(time_log, tau_ffwd_log[jidx, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des,
                     color='green')
        if   (plot_var_des_log is not None):
             plt.plot(time_log, plot_var_des_log[jidx,:], linestyle='-', marker="o",markersize=marker_size, lw=lw_des,color = 'red')
        plt.plot(time_log, plot_var_log[jidx,:],linestyle='-',marker="o",markersize=marker_size, lw=lw_act,color = 'blue')
        if (q_adm is not None):
            plt.plot(time_log, q_adm[jidx, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='black')
        plt.grid()

    if njoints == 12:
        fig.align_ylabels(fig.axes[0:12:3])
        fig.align_ylabels(fig.axes[1:12:4])
        fig.align_ylabels(fig.axes[2:12:4])

    return fig
                
    

                
def plotEndeff(name, figure_id, time_log, plot_var_log, plot_var_des_log = None):

    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)                   
    plt.subplot(3,1,1)
    plt.ylabel("x")
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[0,:], lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[0,:], lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,2)
    plt.ylabel("y")
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[1,:], lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[1,:], lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,3)
    plt.ylabel("z")
    if   (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[2,:], lw=lw_des, color = 'red')                                        
    plt.plot(time_log, plot_var_log[2,:], lw=lw_act, color = 'blue')
    plt.grid()


def plotAdmittanceTracking(figure_id, time_log, x_log, x_des_log, x_des_log_adm, f_log):

    fig = plt.figure(figure_id)
    fig.suptitle("admittance tracking", fontsize=20)
    plt.subplot(4, 1, 1)
    plt.ylabel("end-effector x")
    plt.plot(time_log, x_log[0, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[0, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[0, :], lw=2, color='black')
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.ylabel("end-effector y")
    plt.plot(time_log, x_log[1, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[1, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[1, :], lw=2, color='black')
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.ylabel("end-effector z")
    plt.plot(time_log, x_log[2, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[2, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[2, :], lw=2, color='black')
    plt.grid()

    f_norm = []
    for i in range(f_log.shape[1]):
        f_norm.append(np.linalg.norm(f_log[:,i]))

    plt.subplot(4, 1, 4)
    plt.plot(time_log, f_norm, lw=2, color='blue')
    plt.ylabel("norm of ee force")
    plt.grid()
    
def plotCoM(name,  figure_id, time_log, des_basePoseW=None, basePoseW=None, des_baseTwistW=None, baseTwistW=None, baseAccW=None, des_baseAccW=None,  wrenchW=None, title = None):
    plot_var_des_log = None
    if name == 'position':
        labels = ["CoM X", "CoM Y", "CoM Z", "Roll", "Pitch", "Yaw"]
        lin_unit = '[m]'
        ang_unit = '[rad]'
        plot_var_log = basePoseW
        if   (des_basePoseW is not None):
            plot_var_des_log = des_basePoseW
    elif name == 'velocity':
        labels = ["CoM X", "CoM Y", "CoM Z", "Roll", "Pitch", "Yaw"]
        lin_unit = '[m/s]'
        ang_unit = '[rad/s]'
        plot_var_log = baseTwistW
        if   (des_baseTwistW is not None):
            plot_var_des_log  = des_baseTwistW
    elif name == 'acceleration':
        labels = ["CoM X", "CoM Y", "CoM Z", "Roll", "Pitch", "Yaw"]
        lin_unit = '[m/s^2]'
        ang_unit = '[rad/s^2]'
        plot_var_log = baseAccW
        if   (des_baseAccW is not None):
            plot_var_des_log  = des_baseAccW    
    elif name == 'wrench':
        labels = ["FX", "FY", "FZ", "MX", "MY", "MX"]
        lin_unit = '[N]'
        ang_unit = '[Nm]'
        plot_var_log  = wrenchW
    else:
       print("wrong choice")

    if title is None:
        title = name

    # neet to transpose the matrix other wise it cannot be plot with numpy array
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    plt.subplot(3, 2, 1)
    plt.ylabel(labels[0] + " "+lin_unit)
    plt.plot(time_log, plot_var_log[0, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    if (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[0, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des,
                 color='red')
    plt.grid()

    plt.subplot(3, 2, 3)
    plt.ylabel(labels[1] + " "+lin_unit)
    plt.plot(time_log, plot_var_log[1, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    if (plot_var_des_log is not None):
       plt.plot(time_log, plot_var_des_log[1, :], linestyle='-', lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 5)
    plt.ylabel(labels[2] + " "+lin_unit)
    plt.xlabel("Time [s]")
    plt.plot(time_log, plot_var_log[2, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    if (plot_var_des_log is not None):
       plt.plot(time_log, plot_var_des_log[2, :], linestyle='-', lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 2)
    plt.ylabel(labels[3] + " "+ang_unit)
    plt.plot(time_log, plot_var_log[3, :].T, linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    if (plot_var_des_log is not None):
       plt.plot(time_log, plot_var_des_log[3, :], linestyle='-', lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 4)
    plt.ylabel(labels[4] + " "+ang_unit)
    plt.plot(time_log, plot_var_log[4, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    if (plot_var_des_log is not None):
       plt.plot(time_log, plot_var_des_log[4, :], linestyle='-', lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 6)
    plt.ylabel(labels[5] + " "+ang_unit)
    plt.xlabel("Time [s]")
    plt.plot(time_log, plot_var_log[5, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    if (plot_var_des_log is not None):
       plt.plot(time_log, plot_var_des_log[5, :], linestyle='-', lw=lw_des, color='red')
    plt.grid()


    fig.align_ylabels(fig.axes[:3])
    fig.align_ylabels(fig.axes[3:])

    return fig



def plotCoMLinear(name, figure_id, time_log, plot_var_des_log=None, plot_var_log=None):
    # neet to transpose the matrix other wise it cannot be plot with numpy array
    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)
    plt.subplot(3, 1, 1)
    plt.ylabel("X")
    plt.plot(time_log, plot_var_log[0, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    if (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[0, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des,
                 color='red')

    plt.grid()

    plt.subplot(3, 1, 2)
    plt.ylabel("Y")
    plt.plot(time_log, plot_var_log[1, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue',
             label="q")
    if (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[1, :], linestyle='-', lw=lw_des, color='red', label="des")
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.ylabel("Z")
    plt.plot(time_log, plot_var_log[2, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    if (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[2, :], linestyle='-', lw=lw_des, color='red')
    plt.grid()



def plotGRFs(figure_id, time_log, des_forces, act_forces):
    # %% Input plots

    fig = plt.figure(figure_id)
    fig.suptitle("ground reaction forces", fontsize=20)  
    plt.subplot(6,2,1)
    plt.ylabel("$LF_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[0,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[0,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                
    plt.subplot(6,2,3)
    plt.ylabel("$LF_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[1,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[1,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                
    plt.subplot(6,2,5)
    plt.ylabel("$LF_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[2,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[2,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((0,450))

    #RF
    plt.subplot(6,2,2)
    plt.ylabel("$RF_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[3,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[3,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                
    plt.subplot(6,2,4)
    plt.ylabel("$RF_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[4,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[4,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                
    plt.subplot(6,2,6)
    plt.ylabel("$RF_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[5,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[5,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((0,450))
                
     #LH
    plt.subplot(6,2,7)
    plt.ylabel("$LH_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[6,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[6,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                
    plt.subplot(6,2,9)
    plt.ylabel("$LH_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[7,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[7,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                
                
    plt.subplot(6,2,11)
    plt.ylabel("$LH_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[8,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[8,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((0,450))
                
     #RH
    plt.subplot(6,2,8)
    plt.ylabel("$RH_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[9,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[9,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                
    plt.subplot(6,2,10)
    plt.ylabel("$RH_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[10,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[10,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((-100,100))
                        
    plt.subplot(6,2,12)
    plt.ylabel("$RH_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[11,:],linestyle='-',lw=lw_des,color = 'red')
    plt.plot(time_log, act_forces[11,:],linestyle='-',lw=lw_act,color = 'blue')
    plt.grid()
    #plt.ylim((0,450))


    fig.align_ylabels(fig.axes[0:12:4])
    fig.align_ylabels(fig.axes[1:12:4])
    fig.align_ylabels(fig.axes[2:12:4])
    fig.align_ylabels(fig.axes[3:12:4])


def plotGRFs_withGT(figure_id, time_log, des_forces, act_forces, gt_forces):
    # %% Input plots

    fig = plt.figure(figure_id)
    fig.suptitle("ground reaction forces", fontsize=20)
    plt.subplot(6, 2, 1)
    plt.ylabel("$LF_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[0, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[0, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[0, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 3)
    plt.ylabel("$LF_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[1, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[1, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[1, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 5)
    plt.ylabel("$LF_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[2, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[2, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[2, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((0,450))

    # RF
    plt.subplot(6, 2, 2)
    plt.ylabel("$RF_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[3, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[3, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[3, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 4)
    plt.ylabel("$RF_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[4, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[4, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[4, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 6)
    plt.ylabel("$RF_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[5, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[5, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[5, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((0,450))

    # LH
    plt.subplot(6, 2, 7)
    plt.ylabel("$LH_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[6, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[6, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[6, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 9)
    plt.ylabel("$LH_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[7, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[7, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[7, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 11)
    plt.ylabel("$LH_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[8, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[8, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[8, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((0,450))

    # RH
    plt.subplot(6, 2, 8)
    plt.ylabel("$RH_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[9, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[9, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[9, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 10)
    plt.ylabel("$RH_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[10, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[10, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[10, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 12)
    plt.ylabel("$RH_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[11, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[11, :], linestyle='-', lw=lw_act, color='blue')
    plt.plot(time_log, gt_forces[11, :], linestyle='-', lw=lw_act, color='green')
    plt.grid()
    # plt.ylim((0,450))

    fig.align_ylabels(fig.axes[:6])
    fig.align_ylabels(fig.axes[6:])
    return fig


def plotGRFs_withContacts(figure_id, time_log, des_forces, act_forces, contact_states):
    # %% Input plots

    fig = plt.figure(figure_id)
    fig.suptitle("ground reaction forces", fontsize=20)
    plt.subplot(6, 2, 1)
    plt.ylabel("$LF_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[0, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[0, :], linestyle='-', lw=lw_act, color='blue')
    coeff =  max(np.nanmax(des_forces[0, :]), np.nanmax(act_forces[0, :]))
    plt.plot(time_log, coeff*contact_states[0, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 3)
    plt.ylabel("$LF_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[1, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[1, :], linestyle='-', lw=lw_act, color='blue')
    coeff =  max(np.nanmax(des_forces[1, :]), np.nanmax(act_forces[1, :]))
    plt.plot(time_log, coeff * contact_states[0, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 5)
    plt.ylabel("$LF_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[2, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[2, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[2, :]), np.nanmax(act_forces[2, :]))
    plt.plot(time_log, coeff * contact_states[0, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((0,450))

    # RF
    plt.subplot(6, 2, 2)
    plt.ylabel("$RF_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[3, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[3, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[3, :]), np.nanmax(act_forces[3, :]))
    plt.plot(time_log, coeff * contact_states[1, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 4)
    plt.ylabel("$RF_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[4, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[4, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[4, :]), np.nanmax(act_forces[4, :]))
    plt.plot(time_log, coeff * contact_states[1, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 6)
    plt.ylabel("$RF_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[5, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[5, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[5, :]), np.nanmax(act_forces[5, :]))
    plt.plot(time_log, coeff * contact_states[1, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((0,450))

    # LH
    plt.subplot(6, 2, 7)
    plt.ylabel("$LH_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[6, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[6, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[6, :]), np.nanmax(act_forces[6, :]))
    plt.plot(time_log, coeff * contact_states[2, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 9)
    plt.ylabel("$LH_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[7, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[7, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[7, :]), np.nanmax(act_forces[7, :]))
    plt.plot(time_log, coeff * contact_states[2, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 11)
    plt.ylabel("$LH_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[8, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[8, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[8, :]), np.nanmax(act_forces[8, :]))
    plt.plot(time_log, coeff * contact_states[2, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((0,450))

    # RH
    plt.subplot(6, 2, 8)
    plt.ylabel("$RH_x [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[9, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[9, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[9, :]), np.nanmax(act_forces[9, :]))
    plt.plot(time_log, coeff * contact_states[3, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 10)
    plt.ylabel("$RH_y [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[10, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[10, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[10, :]), np.nanmax(act_forces[10, :]))
    plt.plot(time_log, coeff * contact_states[3, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((-100,100))

    plt.subplot(6, 2, 12)
    plt.ylabel("$RH_z [N]$", fontsize=10)
    plt.xlabel("Time [s]")
    plt.plot(time_log, des_forces[11, :], linestyle='-', lw=lw_des, color='red')
    plt.plot(time_log, act_forces[11, :], linestyle='-', lw=lw_act, color='blue')
    coeff = max(np.nanmax(des_forces[11, :]), np.nanmax(act_forces[11, :]))
    plt.plot(time_log, coeff * contact_states[3, :], linestyle='-', lw=lw_act/2, color='black')
    plt.grid()
    # plt.ylim((0,450))

    fig.align_ylabels(fig.axes[:6])
    fig.align_ylabels(fig.axes[6:])
    return fig


def plotFeet(figure_id, time_log, des_feet=None, act_feet=None, contact_states=None):
    # %% Input plots

    fig = plt.figure(figure_id)
    fig.suptitle("Feet", fontsize=20)

    legs = ['LF', 'RF', 'LH', 'RH']
    axes = ['x', 'y', 'z']

    positions = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

    for l in range(4):
        for a in range(3):
            i = 3*l+a
            plt.subplot(6, 2, positions[3*l+a])
            plt.ylabel("$"+legs[l]+"_"+axes[a]+"\, [m]$", fontsize=10)
            plt.xlabel("Time [s]")
            if des_feet is not None:
                plt.plot(time_log, des_feet[i, :], linestyle='-', lw=lw_des, color='red')
            if act_feet is not None:
                plt.plot(time_log, act_feet[i, :], linestyle='-', lw=lw_act, color='blue')
            if des_feet is not None and act_feet is not None and contact_states is not None:
                coeff =  max(np.nanmax(des_feet[i, :]), np.nanmax(act_feet[0, :]))
                plt.plot(time_log, coeff*contact_states[l, :], linestyle='-', lw=lw_act/2, color='black')
            plt.grid()
    # plt.ylim((-100,100))

    fig.align_ylabels(fig.axes[:6])
    fig.align_ylabels(fig.axes[6:])
    return fig



def plotConstraitViolation(figure_id,constr_viol_log):
    fig = plt.figure(figure_id)            
    plt.plot(constr_viol_log[0,:],label="LF")
    plt.plot(constr_viol_log[1,:],label="RF")
    plt.plot(constr_viol_log[2,:],label="LH")
    plt.plot(constr_viol_log[3,:],label="RH")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("Constr violation", fontsize=10)
    plt.grid()                                                                     

def plotEndeffImpedance(name, figure_id, x_log, x_des_log, f_log):                  
    
    title=""    
    
    if name == 'position':
        title="Force vs Displacement" 
    elif name == 'velocity':
        title="Force vs Velocity" 
    elif name == 'acceleration':
        title="Force vs Acceleration"                           
    else:
        print("wrong choice in impedance plotting")
 
    lw_act=4  
    lw_des=7
                    
#    fig = plt.figure(figure_id)    
    fig, axs = plt.subplots(3, 3)
    fig.suptitle(title, fontsize=20)
    
    axs[0, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 0].set_title('Fx vs X')
    axs[0, 0].grid()
    
    axs[0, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 1].set_title('Fx vs Y')
    axs[0, 1].grid()
    
    axs[0, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 2].set_title('Fx vs Z')
    axs[0, 2].grid()
    
    axs[1, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 0].set_title('Fy vs X')
    axs[1, 0].grid()
    
    axs[1, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 1].set_title('Fy vs Y')
    axs[1, 1].grid()
    
    axs[1, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 2].set_title('Fy vs Z')
    axs[1, 2].grid()
    
    axs[2, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 0].set_title('Fz vs X')
    axs[2, 0].grid()
    
    axs[2, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 1].set_title('Fz vs Y')
    axs[2, 1].grid()
    
    axs[2, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 2].set_title('Fz vs Z')
    axs[2, 2].grid()
    
def plotJointImpedance(name, q_log, q_des_log, tau_log):
    
    title=""
    
    if name == 'position':
        title="Torque vs Angular Displacement"      
    elif name == 'velocity':
        title="Torue vs Angular Velocity" 
    elif name == 'acceleration':
        title="Torque vs Angular Acceleration"                           
    else:
        print("wrong choice in impedance plotting")
 
    lw_act=4  
    lw_des=3

    #Number of joints
    njoints = q_log.shape[0]                                                            
    
    #neet to transpose the matrix other wise it cannot be plot with numpy array    
    fig = plt.figure()                
    fig.suptitle(name, fontsize=20)             
    labels_ur = ["1 - Shoulder Pan", "2 - Shoulder Lift","3 - Elbow","4 - Wrist 1","5 - Wrist 2","6 - Wrist 3"]
    labels_hyq = ["LF_HAA", "LF_HFE","LF_KFE","RF_HAA", "RF_HFE","RF_KFE","LH_HAA", "LH_HFE","LH_KFE","RH_HAA", "RH_HFE","RH_KFE"]

    if njoints == 6:
        labels = labels_ur         
    if njoints == 12:
        labels = labels_hyq                  
                
    
    for jidx in range(njoints):
                
        plt.subplot(njoints/2,2,jidx+1)
        plt.ylabel(labels[jidx])    
        plt.plot(q_log[jidx,:].T-q_des_log[jidx,:].T, tau_log[jidx,:].T, linestyle='-', lw=lw_des,color = 'blue')
        plt.grid()


def polar_char(name, figure_id, phase_deg, mag0, mag1=None, mag2=None):
    import matplotlib as mpl
    size_font = 24
    mpl.rcdefaults()
    mpl.rcParams['lines.linewidth'] = 10
    mpl.rcParams['lines.markersize'] = 6
    mpl.rcParams['patch.linewidth'] = 2
    mpl.rcParams['axes.grid'] = True
    mpl.rcParams['axes.labelsize'] = size_font
    mpl.rcParams['font.family'] = 'sans-serif'
    mpl.rcParams['font.size'] = size_font
    mpl.rcParams['font.serif'] = ['Times New Roman', 'Times', 'Bitstream Vera Serif', 'DejaVu Serif',
                                  'New Century Schoolbook',
                                  'Century Schoolbook L', 'Utopia', 'ITC Bookman', 'Bookman', 'Nimbus Roman No9 L',
                                  'Palatino',
                                  'Charter', 'serif']
    mpl.rcParams['text.usetex'] = True
    mpl.rcParams['legend.fontsize'] = size_font
    mpl.rcParams['legend.loc'] = 'best'
    mpl.rcParams['figure.facecolor'] = 'white'
    mpl.rcParams['figure.figsize'] = 14, 14
    mpl.rcParams['savefig.format'] = 'pdf'





    phase_rad = []
    for deg in phase_deg:
        rad = deg * np.pi/180
        phase_rad.append(rad)

    patches = []
    for mag in [mag0, mag1, mag2]:
        if mag is not None:
            poly = np.zeros((len(phase_rad), 2))
            for i in range(len(phase_rad)):
                poly[i, :] = np.array([phase_rad[i], mag[i]])
            patches.append(Polygon(poly))

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    # plt.subplots_adjust(left=0.04, bottom=0.04, top=0.96, right=0.96)

    p = PatchCollection(patches, alpha=0.7)
    fcolors = ['g', 'dodgerblue', 'coral']
    ecolors = ['darkgreen', 'b', 'r']
    p.set_edgecolor(ecolors)
    p.set_facecolor(fcolors)


    ax.set_rmax(3)
    step = np.abs(phase_deg[0]-phase_deg[1])
    phase_rad =np.arange(0,360, step)*np.pi/180
    ax.set_xticks(phase_rad)

    rticks = np.arange(0,4,0.5)
    ax.set_rticks(rticks)
    #rticks_show = np.arange(0, 4, 1)
    ax.set_yticklabels(rticks)
    ax.add_collection(p)

    plt.show()
    return fig, ax

    

