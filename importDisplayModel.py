# -*- coding: utf-8 -*-
"""
Created on Thu Apr  2 18:07:44 2020

@author: mfocchi
"""
import os
#from pinocchio.visualize import GepettoVisualizer
from pinocchio.robot_wrapper import RobotWrapper


def importDisplayModel(DISPLAY, DISPLAY_FLOOR):
	# Import the model
	ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
	path      = os.environ.get('UR5_MODEL_DIR', ERROR_MSG)
	urdf      = path + "/ur_description/urdf/ur5_gripper.urdf";
	srdf      = path + '/ur5_description/srdf/ur5_gripper.srdf'
	robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
	
	if DISPLAY:
	    import commands
	    import gepetto
	    from time import sleep
	    robot.initViewer(loadModel=True)
	    l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
	    if int(l[1]) == 0:
	        os.system('gepetto-gui &')
	    sleep(1)
	    gepetto.corbaserver.Client()
	    robot.initViewer(loadModel=True)
	    gui = robot.viewer.gui
	    if(DISPLAY_FLOOR):
	        robot.viewer.gui.createSceneWithFloor('world')
	        gui.setLightingMode('world/floor', 'ON')
	    robot.displayCollisions(False)
	    robot.displayVisuals(True)
					
	return robot					