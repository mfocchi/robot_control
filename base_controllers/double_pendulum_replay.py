# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 10:11:34 2021

@author: mfocchi
"""
import os
from utils.ros_publish import RosPub
import numpy as np
from example_robot_data.robots_loader import load
import tf
import rospy
import roslib
from rospy import Time

os.system("killall rosmaster rviz gzserver gzclient")  
br = tf.TransformBroadcaster()


ros_pub = RosPub("double-pendulum") #start only graphic stuff
robot =  load('double_pendulum')
q_des = np.zeros((robot.nv))

q_des = np.array([0, 0])

translation = (0.0, 0.0, 1.0)
rotation = (0.0, 0.0, 0.0, 1.0)
rate = rospy.Rate(5)  # 5hz    

while not rospy.is_shutdown():
    br.sendTransform(translation, rotation, Time.now(), '/base_link', '/world')
    ros_pub.publish(robot, q_des)
  
    rate.sleep()
    
    

    
