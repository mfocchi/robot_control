

import rospy as ros
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np

class RosPub():
    def __init__(self): 

        #initi ros node                                
        ros.init_node('sub_pub_node_python', anonymous=False)
        #set joint state publisher            
        self.joint_pub = ros.Publisher("/joint_states", JointState, queue_size=1)

        self.marker_pub = ros.Publisher('vis' , MarkerArray, queue_size=1)                                
        self.markerArray = MarkerArray()
                            
                                
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

    def publish_marker(self, pos):
       self.markerArray.markers = []                 
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
       self.marker_pub .publish(self.markerArray)                            
                    
    def deregister_node(self):
        ros.signal_shutdown("manual kill")

							
    def isShuttingDown(self):
       return ros.is_shutdown()
							

