# Description
# File contains some necessary control algorithms for HyQ
# Author: Michele Focchi
# Date: 04-12-2022
import rospy as ros
import numpy as np
from base_controllers.utils.common_functions import checkRosMaster, launchFileGeneric
from std_msgs.msg import Bool, Int32, Float32
from termcolor import colored
import os
import rospkg

class CoppeliaManager():
    def __init__(self, coppeliaModel = None, use_gui = True):
        self.use_gui = use_gui
        self.coppeliaModel = coppeliaModel
        self.startPub = ros.Publisher("/startSimulation", Bool, queue_size=1, tcp_nodelay=True)
        self.pausePub = ros.Publisher("/pauseSimulation", Bool, queue_size=1, tcp_nodelay=True)
        self.stopPub = ros.Publisher("/stopSimulation", Bool, queue_size=1, tcp_nodelay=True)
        self.enableSyncModePub = ros.Publisher("/enableSyncMode", Bool, queue_size=1, tcp_nodelay=True)
        self.triggerNextStepPub = ros.Publisher("/triggerNextStep", Bool, queue_size=1, tcp_nodelay=True)
        self.renderSimulationPub = ros.Publisher("/renderSimulation", Bool, queue_size=1, tcp_nodelay=True)
        # simStepDoneSub=ros.Subscriber("/simulationStepDone", Bool, sim_done_)
        self.simStateSub = ros.Subscriber("/simulationState", Int32, self.sim_state_)
        # simTimeSub=simROS.Subscriber('/simulationTime',Float32, time_sim_)

    def startSimulator(self):
        print(colored(
            f"---------To run this you need to clone https://github.com/mfocchi/CoppeliaSim inside locosim folder",
            "red"))
        # clean up previous process
        os.system("killall rosmaster rviz coppeliaSim")
        # launch roscore
        checkRosMaster()
        ros.sleep(1.5)

        # launch coppeliasim
        scene = rospkg.RosPack().get_path('tractor_description') + '/CoppeliaSimModels/' + self.coppeliaModel
        if self.use_gui:
            file = os.getenv("LOCOSIM_DIR") + "/CoppeliaSim/coppeliaSim.sh " + scene + " &"
        else:
            file = os.getenv("LOCOSIM_DIR") + "/CoppeliaSim/coppeliaSim.sh -h " + scene + " &"

        os.system(file)

        # run robot state publisher + load robot description + rviz
        launchFileGeneric(rospkg.RosPack().get_path('tractor_description') + "/launch/rviz_nojoints.launch")
        # wait for coppelia to start
        ros.sleep(1.5)

    def simulationControl(self,input):
        if input == 'start':
            print("starting simulation")
            ros.wait_for_message('/simulationState', Bool, timeout=5.)
            ros.sleep(1.5)
            msg = Bool()
            msg.data = True
            for i in range(30):
                self.startPub.publish(msg)
        if input == 'stop':
            self.stopPub.publish(True)
        if input == 'enable_sync_mode':
            self.enableSyncModePub.publish(True)
        if input=='trigger_next_step':
            self.triggerNextStepPub.publish(True)

    def sim_state_(self, msg):
        pass

