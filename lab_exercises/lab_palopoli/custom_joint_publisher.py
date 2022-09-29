#!/usr/bin/env python
import rospy as ros
import numpy as np
from sensor_msgs.msg import JointState


class JointStatePublisher():

    def __init__(self):
        self.q_des =np.zeros(6)
        self.qd_des = np.zeros(6)
        self.tau_ffwd = np.zeros(6)
        self.filter_1 = np.zeros(6)
        self.filter_2 = np.zeros(6)

    def send_des_jstate(self):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.position = self.q_des
        msg.velocity = self.qd_des
        msg.effort = self.tau_ffwd
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
    p.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1)

    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)  # 1000hz

    # init variables
    time = 0
    q_des0 = np.array([-0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558])
    p.initFilter(q_des0)

    amp = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])  # amplitude
    freq = np.array([0.2, 0.0, 0.0, 0.0, 0., 0.0]) # frequency


    while not ros.is_shutdown():
        # generate reference
        # 1 -fixed
        # if time < 2.:
        #     p.q_des =  q_des0
        # else:
        #     #p.q_des = q_des0 + np.array([0., 0.4, 0., 0., 0., 0])
        #     # 2- filtered
        #     p.q_des = q_des0 + p.secondOrderFilter(q_des0 + np.array([0., 0.6, 0., 0., 0., 0]), loop_frequency, 5.)

        # 3- sine
        #p.q_des = q_des0 + np.multiply(amp, np.sin(2*np.pi*freq*time))

        p.qd_des = np.zeros(6)
        p.tau_ffwd = np.zeros(6)

        p.send_des_jstate()
        print(p.q_des)
        time = np.round(time + np.array([1/loop_frequency]), 3)
        loop_rate.sleep()

if __name__ == '__main__':
    myPub = JointStatePublisher()
    try:
        talker(myPub)
    except ros.ROSInterruptException:
        pass