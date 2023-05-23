# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from base_controllers.base_controller import BaseController
from base_controllers.utils.common_functions import checkRosMaster, plotFrame, plotJoint
import time
import params as conf
robotName = "tractor" # needs to inherit BaseController
import os
from std_msgs.msg import Bool, Int32, Float32
import rospkg
import roslaunch

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.freezeBaseFlag = False
        self.torque_control = False
        print("Initialized tractor controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                pass
            super().logData()

    def startSimulator(self):
        # clean up previous process
        os.system("killall rosmaster coppeliaSim")
        # launch roscore
        checkRosMaster()
        ros.sleep(0.5)

        # launch coppeliasim
        scene = rospkg.RosPack().get_path('tractor_description') + '/CoppeliaSimModels/scene.ttt'
        file = os.getenv("LOCOSIM_DIR")+"/CoppeliaSim/coppeliaSim.sh"+" "+scene+" &"
        os.system(file)
        ros.sleep(1.5)

        # run robot state publisher
        package = 'robot_state_publisher'
        executable = 'robot_state_publisher'
        node = roslaunch.core.Node(package, executable)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)

        # run rviz
        package = 'rviz'
        executable = 'rviz'
        args = '-d ' + rospkg.RosPack().get_path('ros_impedance_controller') + '/config/operator.rviz'
        node = roslaunch.core.Node(package, executable, args=args)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)

    def loadModelAndPublishers(self):
        super().loadModelAndPublishers()
        self.startPub=ros.Publisher("/startSimulation", Bool, queue_size=1, tcp_nodelay=True)
        self.pausePub=ros.Publisher("/pauseSimulation", Bool, queue_size=1, tcp_nodelay=True)
        self.stopPub=ros.Publisher("/stopSimulation", Bool, queue_size=1, tcp_nodelay=True)
        self.enableSynModePub=ros.Publisher("/enableSyncMode", Bool, queue_size=1, tcp_nodelay=True)
        self.triggerNextStepPub=ros.Publisher("/triggerNextStep", Bool, queue_size=1, tcp_nodelay=True)
        # simStepDoneSub=ros.Subscriber("/simulationStepDone", Bool, sim_done_)
        self.simStateSub=ros.Subscriber("/simulationState", Int32, self.sim_state_)
        # simTimeSub=simROS.Subscriber('/simulationTime',Float32, time_sim_)

    def sim_state_(self, msg):
        pass

    def deregister_node(self):
        self.stopPub.publish(True)
        os.system("killall rosmaster coppeliaSim")
        super().deregister_node()

    def simulationControl(self,input):
        if input == 'start':
            print("starting simulation")
            ros.wait_for_message('/simulationState', Bool, timeout=2.)
            msg = Bool()
            msg.data = 1
            self.startPub.publish(msg)

def talker(p):
    p.start()
    additional_args = None #'gui:=false'
    #p.startSimulator(additional_args = ['spawn_Y:=3.14'])
    #p.startSimulator(world_name='tractor.world', additional_args=['spawn_Y:=3.14'])



    p.startSimulator()

    p.loadModelAndPublishers()
    p.initVars()
    p.initSubscribers()
    p.simulationControl('start')

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    p.q_des = np.copy(p.q_des_q0)
    # for torque control

    # # equivalent
    # p.pid.setPDjoint(0, 0.0, 0.0, 0.0)
    # p.pid.setPDjoint(1, 0.0, 0.0, 0.0)

    forward_speed = -0.007

    while not ros.is_shutdown():
        #p.simulationControl('start')
        p.qd_des = forward_speed * np.ones(4)
        p.q_des = p.q_des + forward_speed*np.ones(4)
        if p.torque_control:
            p.tau_ffwd = 30*conf.robot_params[p.robot_name]['kp'] * np.subtract(p.q_des, p.q) -2* conf.robot_params[p.robot_name]['kd'] * p.qd
        else:
            p.tau_ffwd = np.zeros(p.robot.na)

        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        # log variables
        p.logData()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

if __name__ == '__main__':
    p = GenericSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        plotJoint('position', p.time_log, p.q_log, p.q_des_log, joint_names = p.joint_names)
        plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, sharex=True, sharey=False,
                  start=0, end=-1)


