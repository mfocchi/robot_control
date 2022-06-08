# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 10:11:34 2021

@author: mfocchi
"""
import os
from utils.ros_publish import RosPub
import numpy as np
from utils.common_functions import getRobotModel
import tf
import rospy
import roslib
from rospy import Time
import pinocchio as pin

os.system("killall rosmaster rviz gzserver gzclient")  
br = tf.TransformBroadcaster()


ros_pub = RosPub("solo") #start only graphic stuff
robot = getRobotModel("solo", generate_urdf = True)
q_des = np.zeros((robot.na))

q_des[:12] = np.array([-0.0, 0.7, -1.4, -0.0, -0.7, 1.4,  -0.0, 0.7, -1.4,-0.0, -0.7, 1.4])

translation = (1.0, 0.0, 0.50)
rpy = np.array([0., -np.pi/15, 0.])
rotation = pin.Quaternion(pin.rpy.rpyToMatrix(rpy))
rate = rospy.Rate(5)  # 5hz    

while not rospy.is_shutdown():
    br.sendTransform(translation, rotation, Time.now(), '/base_link', '/world')
    ros_pub.publish(robot, q_des)
    # flywheel
    if (robot.na>12):
        q_des[12]+=0.5
        q_des[13]+=0.5
        q_des[14]+=0.5
        q_des[15]+=0.5
    rate.sleep()
    
    

    
