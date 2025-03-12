from base_controllers.utils.common_functions import checkRosMaster
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import os
import rospy


if __name__ == '__main__':
    checkRosMaster()
    rospy.init_node('rosbag_controlled_recording')
    recorder = RosbagControlledRecorder()
    bag_location = os.environ["LOCOSIM_DIR"]+"/robot_control/base_controllers/tracked_robot/simulator/"
    bag_name = "openLoop3DModel_sloped_test_right_Distr_True.bag"
    recorder.rosbagPlay(robot_name="tractor", bag_file=bag_location+bag_name, bag_options="-r 1.5")


