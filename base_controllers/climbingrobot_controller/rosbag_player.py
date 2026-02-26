from base_controllers.utils.common_functions import checkRosMaster
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import os
import rospy
from termcolor import colored

if __name__ == '__main__':
    checkRosMaster()
    rospy.init_node('rosbag_controlled_recording')
    recorder = RosbagControlledRecorder()
    bag_location = os.environ["LOCOSIM_DIR"]+"/robot_control/base_controllers/climbingrobot_controller/"
    bag_name = "climbing_robot_rocky_terrain_single_jumps.bag"

    if not os.path.exists(bag_location+bag_name):
        print(colored("Bag File does not exists.", "red"))
    else:
        recorder.rosbagPlay(robot_name="climbingrobot", upload_args=" robot_name:=climbingrobot2", bag_file=bag_location+bag_name, bag_options="-r 0.5")


