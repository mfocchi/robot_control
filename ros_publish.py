

import rospy as ros
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Point
import math
import numpy as np

import roslaunch
import os

import time as tm

class RosPub():
    def __init__(self): 
        #launch rviz node if not yet done will start roscore too
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/visualize.launch"])
        self.launch.start()													
        ros.loginfo("RVIZ started")
        tm.sleep(2.0)							
        print("---------------------------------------------------------------")   
        #init ros node to publish joint states and vis topics                                
        ros.init_node('sub_pub_node_python', anonymous=False, log_level=ros.FATAL)
        #set joint state publisher            
        self.joint_pub = ros.Publisher("/joint_states", JointState, queue_size=1)
        self.marker_pub = ros.Publisher('vis' , MarkerArray, queue_size=1)                                
        self.markerArray = MarkerArray()
        self.markerArray.markers = []                              
                                
    def publish(self,robot, q, qd = None, tau = None):
    
        if (qd is None ):
           qd = np.zeros(robot.nv)
        if (tau is None ):
           tau = np.zeros(robot.nv)
                                
        all_names = [name for name in robot.model.names]            
        msg = JointState()
        msg.header.stamp = ros.Time.now()            
        msg.name = all_names[-6:] #remove universe joint that is not active
        msg.position = q                
        msg.velocity = qd                
        msg.effort = tau              
        
        self.joint_pub.publish(msg)
                                
        #publish also the markers if any
        if len(self.markerArray.markers)>0:                                                                                                                            
            self.marker_pub .publish(self.markerArray)  
        #reset the marker array making it ready for another round
        self.markerArray.markers = []                             

    def add_marker(self, pos):
       marker = Marker()
       marker.header.frame_id = "world"
       marker.type = marker.SPHERE
       marker.action = marker.ADD
       marker.scale.x = 0.1
       marker.scale.y = 0.1
       marker.scale.z = 0.1
       marker.color.a = 0.5
       marker.color.r = 1.0
       marker.color.g = 1.0
       marker.color.b = 0.0
       marker.pose.orientation.w = 1.0
       marker.pose.position.x = pos[0]
       marker.pose.position.y = pos[1] 
       marker.pose.position.z = pos[2] 
       self.markerArray.markers.append(marker)
       
    def add_arrow(self, start, vector):
       marker = Marker()
       marker.header.frame_id = "world"
       marker.type = marker.ARROW
       marker.action = marker.ADD                            
       marker.points.append(Point(start[0], start[1], start[2]))    
       marker.points.append(Point(start[0] + vector[0], start[1] + vector[1], start[2] + vector[2]))                                                                                      
       marker.scale.x = 0.05
       marker.scale.y = 0.05
       marker.scale.z = 0.05
       marker.color.a = 1.0
       marker.color.r = 0.0
       marker.color.g = 1.0
       marker.color.b = 1.0
       self.markerArray.markers.append(marker)
                    
    def deregister_node(self):
        print("---------------------------------------------------------------")   					
        ros.signal_shutdown("manual kill")
        self.launch.shutdown()

    def isShuttingDown(self):
       return ros.is_shutdown()
                            
