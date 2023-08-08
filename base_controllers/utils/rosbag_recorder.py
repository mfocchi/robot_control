# import psutil
# import signal
# import subprocess
# import os
# import time
#
# def terminate_ros_node(p):
#   process = psutil.Process(p.pid)
#   for sub_process in process.get_children(recursive=True):
#       sub_process.send_signal(signal.SIGINT)
#   p.wait()  # we wait for children to terminate
#   p.terminate()
#
# def terminate_ros_node(s):
#   list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
#   list_output = list_cmd.stdout.read()
#   print(list_output)
#   retcode = list_cmd.wait()
#   assert retcode == 0, "List command returned %d" % retcode
#   for str in list_output.split("\n"):
#     if (str.startswith(s)):
#       os.system("rosnode kill " + str)
#
# if __name__ == '__main__':
#   dir_save_bagfile = '/home/mfocchi/'
#   rosbag_process = subprocess.Popen('rosbag record -a -j -o {}'.format("myrosbag"), stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)
#   time.sleep(2.5)
#   print("stopping rosbag")
#   terminate_ros_node(rosbag_process)

#!/usr/bin/env python

"""Node to record a rosbag with start/stop/pause control through service calls.
Example call:
    rosrun utilities rosbag_controlled_recording.py _rosbag_command:="rosbag record -o /home/foo/test_bag /bar_topic" _record_from_startup:=false
Then start/pause/resume/stop can be controlled through:
    rosservice call /rosbag_controlled_recording/start
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/stop
Note that pausing does not modify the recorded time of messages, i.e. the bag's total length is unaffected. A list of
  pause-resume times is logged when stopping, in case the paused period needs to be (manually) removed afterwards.
If this node is killed recording is also stopped. If recording was paused, it is momentarily resumed before stopping.
"""

import psutil
import subprocess
import shlex
import signal
import time
import rospy
from std_srvs.srv import Empty, EmptyResponse
from base_controllers.utils.common_functions import checkRosMaster

def signal_process_and_children(pid, signal_to_send, wait=False):
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        if signal_to_send == 'suspend':
            children.suspend()
        elif signal_to_send == 'resume':
            children.resume()
        else:
            children.send_signal(signal_to_send)
    if wait:
        process.wait()


def format_to_columns(input_list, cols):
    """Adapted from https://stackoverflow.com/questions/171662/formatting-a-list-of-text-into-columns"""
    max_width = max(map(len, input_list))
    justify_list = map(lambda x: x.ljust(max_width + 4), input_list)
    lines = (''.join(justify_list[i:i + cols]) for i in range(0, len(justify_list), cols))
    return '\n'.join(lines)


class RosbagControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop  and pause"""

    def __init__(self, rosbag_command_, record_from_startup_=False):
        self.rosbag_command = shlex.split(rosbag_command_)
        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []
        self.process_pid = None
        if record_from_startup_:
            self.start_recording_srv()

    def start_recording_srv(self, service_message=None):
        process = subprocess.Popen(self.rosbag_command)
        self.process_pid = process.pid
        self.recording_started = True
        rospy.loginfo("Started recording rosbag")
        return EmptyResponse()

    def pause_resume_recording_srv(self, service_message=None):
        if self.recording_started:
            if self.recording_paused:
                signal_process_and_children(self.process_pid, 'resume')
                self.recording_paused = False
                rospy.loginfo("Recording resumed")
            else:
                signal_process_and_children(self.process_pid, 'suspend')
                self.recording_paused = True
                rospy.loginfo("Recording paused")
            self.pause_resume_times.append(rospy.get_time())
            return EmptyResponse()
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")

    def stop_recording_srv(self, service_message=None):
        if self.process_pid is not None:
            if self.recording_paused:  # need to resume process in order to cleanly kill it
                self.pause_resume_recording_srv()
            if self.pause_resume_times:  # log pause/resume times for user's reference
                pause_resume_str = map(str, self.pause_resume_times)
                pause_resume_str[0:0] = ['PAUSE', 'RESUME']
                rospy.loginfo("List of pause and resume times:\n%s\n", format_to_columns(pause_resume_str, 2))
            signal_process_and_children(self.process_pid, signal.SIGINT, wait=True)
            self.process_pid = None
            rospy.loginfo("Stopped recording rosbag")
        self.recording_stopped = True
        return EmptyResponse()


if __name__ == '__main__':
    checkRosMaster()
    rospy.init_node('rosbag_controlled_recording')

    # Get parameters

    # Start recorder object
    recorder = RosbagControlledRecorder('rosbag record -a',  False)

    # Services
    start_service = rospy.Service('~start', Empty, recorder.start_recording_srv)
    pause_resume_service = rospy.Service('~pause_resume', Empty, recorder.pause_resume_recording_srv)
    stop_service = rospy.Service('~stop', Empty, recorder.stop_recording_srv)

    # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
    rospy.on_shutdown(recorder.stop_recording_srv)
    print("start 1st bag")
    time.sleep(2)
    recorder.start_recording_srv()
    time.sleep(5)
    recorder.stop_recording_srv()
    print("start 2nd bag")
    time.sleep(10)
    recorder.start_recording_srv()
    time.sleep(10)
    recorder.stop_recording_srv()


    while not rospy.is_shutdown():
        if recorder.recording_stopped:  # stop main node if recording has finished
            print("finish")
            break
        rospy.sleep(1.0)