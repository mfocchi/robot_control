

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
import tf
from rospy import Time

class RosPub():
    def __init__(self, robot_name="solo", only_visual = False, visual_frame = "world"):
        
        print("Starting ros pub---------------------------------------------------------------")
        if (not only_visual):                           
            #launch rviz node if not yet done will start roscore too
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.environ['LOCOSIM_DIR'] + "/ros_impedance_controller/launch/visualize_"+robot_name+".launch"])
            self.launch.start()                                                    
            ros.loginfo("RVIZ started")
            tm.sleep(1.0)
                               
   
        #init ros node to publish joint states and vis topics
        ros.init_node('sub_pub_node_python', anonymous=False, log_level=ros.FATAL)
        #set joint state publisher 
        self.joint_pub = ros.Publisher("/joint_states", JointState, queue_size=1) 
        self.marker_pub = ros.Publisher('/vis' , MarkerArray, queue_size=1)
        self.arrow_pub = ros.Publisher('/arrow', MarkerArray, queue_size=1)
        self.markerArray = MarkerArray()
        self.markerArray.markers = []
        self.markerArray_arrows = MarkerArray()
        self.markerArray_arrows.markers = []
        self.id = 0
        self.id_arrow = 0

        self.broadcaster = tf.TransformBroadcaster()  
        self.fixedBaseRobot = False
        self.visual_frame = visual_frame
        print("Initialized ros pub---------------------------------------------------------------")
    def publish(self,robot, q, qd = None, tau = None):
    
        if (qd is None ):
           qd = np.zeros(robot.nv)
        if (tau is None ):
           tau = np.zeros(robot.nv)
                                
        all_names = [name for name in robot.model.names]            
        msg = JointState()
        msg.header.stamp = ros.Time.now() 
        # check if it is a floating base robot
        try: 
            index_of_actuated_joints = robot.na
        except:#fixed base
            index_of_actuated_joints = robot.nv
            self.fixedBaseRobot = True
            
        if (self.fixedBaseRobot):
            self.broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')  
                   
        msg.name = all_names[-index_of_actuated_joints:] #remove universe joint that is not active
        msg.position = q                
        msg.velocity = qd                
        msg.effort = tau              
        
        self.joint_pub.publish(msg)
        self.publishVisual()                                   
 
    def publishVisual(self):                                
        #publish also the markers if any
        if len(self.markerArray.markers)>0:                                                                                                                            
            self.marker_pub.publish(self.markerArray)
            # reset the marker array making it ready for another round
            self.markerArray.markers = []
            self.id = 0

        if len(self.markerArray_arrows.markers) > 0:
            self.arrow_pub.publish(self.markerArray_arrows)
            self.markerArray_arrows.markers = []
            self.id_arrow = 0
                                
    def add_marker(self, pos, radius = 0.1):
       marker = Marker()
       marker.header.frame_id = self.visual_frame
       marker.type = marker.SPHERE
       marker.action = marker.ADD
       marker.scale.x = radius
       marker.scale.y = radius
       marker.scale.z = radius
       marker.color.a = 0.5
       marker.color.r = 1.0
       marker.color.g = 0.0
       marker.color.b = 0.0                        
                               
       marker.pose.orientation.w = 1.0
       marker.pose.position.x = pos[0]
       marker.pose.position.y = pos[1] 
       marker.pose.position.z = pos[2]
       marker.lifetime = ros.Duration(0.0)
       
       marker.id = self.id       
       self.id += 1                                        
       self.markerArray.markers.append(marker)
                            
    def add_arrow(self, start, vector, color = "green"):
       marker = Marker()
       if (color == "green"):                    
           marker.color.r = 0.0
           marker.color.g = 1.0
           marker.color.b = 0.0                    
       if (color == "blue"):                    
           marker.color.r = 0.0
           marker.color.g = 0.0
           marker.color.b = 1.0                                                

       marker.header.frame_id = self.visual_frame
       marker.type = marker.ARROW
       marker.action = marker.ADD
       marker.points.append(Point(start[0], start[1], start[2]))
       marker.points.append(Point(start[0] + vector[0], start[1] + vector[1], start[2] + vector[2]))
       marker.scale.x = 0.02
       marker.scale.y = 0.04
       marker.scale.z = 0.02
       marker.color.a = 1.0
       marker.lifetime = ros.Duration(0.0)

       marker.id = self.id_arrow
       self.id_arrow += 1
       self.markerArray_arrows.markers.append(marker)
       
    def add_cone(self,  origin, normal, friction_coeff, color = "green"):     
        
       height = 0.2;
       radius = friction_coeff* height
       tail_end = origin + normal*height; 
       marker = Marker()
       if (color == "green"):                    
           marker.color.r = 0.0
           marker.color.g = 1.0
           marker.color.b = 0.0                    
       if (color == "blue"):                    
           marker.color.r = 0.0
           marker.color.g = 0.0
           marker.color.b = 1.0                                                

       marker.header.frame_id = self.visual_frame
       marker.type = marker.ARROW
       marker.action = marker.ADD      
       
       marker.points.append(Point(tail_end[0], tail_end[1], tail_end[2]))    
       marker.points.append(Point(origin[0], origin[1], origin[2]))
       marker.scale.x = 0.0
       marker.scale.y = 2*radius
       marker.scale.z = height
       marker.color.a = 0.7

       marker.id = self.id
       self.id += 1     
       self.markerArray.markers.append(marker)   
                    
    def deregister_node(self):
        print("---------------------------------------------------------------")                  
        ros.signal_shutdown("manual kill")
      

    def isShuttingDown(self):
       return ros.is_shutdown()
                            

