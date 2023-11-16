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
import params as conf
robotName = "tractor" # needs to inherit BaseController
import os
from std_msgs.msg import Bool, Int32, Float32
import rospkg
import roslaunch

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.torque_control = False
        print("Initialized tractor controller---------------------------------------------------------------")
        self.GAZEBO = True

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
        if self.GAZEBO:
            super().startSimulator(world_name='tractor.world', additional_args=['spawn_Y:=3.14'])
        else:
            # clean up previous process
            os.system("killall rosmaster rviz coppeliaSim")
            # launch roscore
            checkRosMaster()
            ros.sleep(0.5)

            # launch coppeliasim
            scene = rospkg.RosPack().get_path('tractor_description') + '/CoppeliaSimModels/scene.ttt'
            file = os.getenv("LOCOSIM_DIR")+"/CoppeliaSim/coppeliaSim.sh"+" "+scene+" &"
            os.system(file)
            ros.sleep(1.5)

            # run robot state publisher + load robot description + rviz
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('tractor_description')+"/launch/rviz_nojoints.launch"])
            launch.start()

            # launch separately
            # package = 'robot_state_publisher'
            # executable = 'robot_state_publisher'
            # node = roslaunch.core.Node(package, executable)
            # launch = roslaunch.scriptapi.ROSLaunch()
            # launch.start()
            # process = launch.launch(node)
            #
            # # run rviz
            # package = 'rviz'
            # executable = 'rviz'
            # args = '-d ' + rospkg.RosPack().get_path('ros_impedance_controller') + '/config/operator.rviz'
            # node = roslaunch.core.Node(package, executable, args=args)
            # launch = roslaunch.scriptapi.ROSLaunch()
            # launch.start()
            # process = launch.launch(node)

    def loadModelAndPublishers(self):
        super().loadModelAndPublishers()
        if not self.GAZEBO:
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
        os.system("killall rosmaster rviz coppeliaSim")
        super().deregister_node()

    def simulationControl(self,input):
        if input == 'start':
            print("starting simulation")
            ros.wait_for_message('/simulationState', Bool, timeout=5.)
            ros.sleep(1.5)
            msg = Bool()
            msg.data = True
            for i in range(30):
                self.startPub.publish(msg)

    def startupProcedure(self):
        if self.GAZEBO:
            super().startupProcedure()
            if self.torque_control:
                self.pid.setPDs(0.0, 0.0, 0.0)
        else:
            # we need to broadcast the TF world baselink in coppelia not in base controller for synchro issues
            self.broadcast_world = False
            self.simulationControl('start')
            #wait for coppelia to start
            ros.sleep(1.5)

def talker(p):
    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()
    p.initSubscribers()
    p.startupProcedure()

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.q_des = np.copy(p.q_des_q0)
    if p.GAZEBO:
        forward_speed = -0.5
        actuated_wheels = [0, 1, 2, 3]
    else:
        forward_speed = 1.
        actuated_wheels = [0, 1]

    while not ros.is_shutdown():

        p.qd_des[actuated_wheels] = forward_speed
        p.q_des = p.q_des + p.qd_des*conf.robot_params[p.robot_name]['dt']
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
        pass

    ros.signal_shutdown("killed")
    p.deregister_node()
    print("Plotting")
    plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names = p.joint_names)
    plotJoint('velocity', p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, joint_names = p.joint_names)


