#!/usr/bin/env python
import rospy as ros
import numpy as np
from std_msgs.msg import Float64MultiArray
from base_controllers.components.gripper_manager import GripperManager
import params as conf

#TODO doesn't use the filter

class JointStatePublisher():

    def __init__(self):
        self.q_des =np.zeros(6)
        self.time = 0
        self.filter_1 = np.zeros(6)
        self.filter_2 = np.zeros(6)
        self.gm = GripperManager(False, conf.robot_params['ur5']['dt'])

    def send_des_jstate(self):
        msg = Float64MultiArray()
        msg.data = np.append(self.q_des, self.gm.getDesGripperJoints())
        self.pub_des_jstate.publish(msg)

    def initFilter(self, q):
        self.filter_1 = np.copy(q)
        self.filter_2 = np.copy(q)

    def secondOrderFilter(self, input, rate, settling_time):
        dt = 1 / rate
        gain =  dt / (0.1*settling_time + dt)
        self.filter_1 = (1 - gain) * self.filter_1 + gain * input
        self.filter_2 = (1 - gain) * self.filter_2 + gain * self.filter_1
        return self.filter_2


def talker(p):
    ros.init_node('custom_joint_pub_node', anonymous=True)
    p.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=1)
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)  # 1000hz

    # init variables
    q_des0 = np.array([-0.3223527113543909, 1.7805794638446351, -2.5675506591796875, -1.6347843609251917, -3.5715253988849085, -2.0017417112933558])
    # p.initFilter(q_des0)

    while not ros.is_shutdown():
        p.q_des = q_des0 + 0.1 * np.sin(2 * np.pi * 0.5* p.time)
        p.gm.move_gripper(50)
        p.send_des_jstate()
        print(p.q_des)
        p.time = np.round(p.time + np.array([conf.robot_params['ur5']['dt']]),  3)
        loop_rate.sleep()

if __name__ == '__main__':
    myPub = JointStatePublisher()
    try:
        talker(myPub)
    except ros.ROSInterruptException:
        pass
