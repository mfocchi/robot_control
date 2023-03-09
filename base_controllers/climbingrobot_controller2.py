# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function
import rospy as ros
from utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from numpy import nan
from utils.common_functions import plotJoint, plotFrameLinear
from termcolor import colored
import os
from rospy import Time
import tf
from base_controller_fixed import BaseControllerFixed
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from std_srvs.srv    import Empty, EmptyRequest
import roslaunch
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.msg import ContactsState
import scipy.io.matlab as mio
import distro
import rospkg
from base_controllers.utils.custom_robot_wrapper import RobotWrapper

import  params as conf
#robotName = "climbingrobot2landing"
robotName = "climbingrobot2landing"

class ClimbingrobotController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.EXTERNAL_FORCE = False
        self.LOAD_MATLAB_TRAJ = True
        self.MARCO_APPROACH = True

        self.rope_index = np.array([2, 8])
        self.leg_index = np.array([12, 13, 14])
        self.base_passive_joints = np.array([3,4,5, 9,10,11])
        self.anchor_passive_joints = np.array([0,1, 6,7])
        if robot_name == 'climbingrobot2landing':
            self.landing = True
        else:
            self.landing = False
        self.landing_joints = np.array([15, 17])
        self.mountain_thickness = 0.1 # TODO call the launch file passing this parameter
        self.r_leg = 0.3
        print("Initialized climbingrobot controller---------------------------------------------------------------")

    def applyForce(self, Fx, Fy, Fz, interval):
        wrench = Wrench()
        wrench.force.x = Fx
        wrench.force.y = Fy
        wrench.force.z = Fz
        wrench.torque.x = 0.
        wrench.torque.y = 0.
        wrench.torque.z = 0.
        reference_frame = "world" # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x = 0., y = 0., z = 0.)
        self.apply_body_wrench(body_name=self.robot_name+"::base_link", reference_frame=reference_frame, reference_point=reference_point, wrench=wrench, start_time=ros.Time(),  duration=ros.Duration(interval))

    def loadModelAndPublishers(self, xacro_path=None):
        xacro_path = rospkg.RosPack().get_path('climbingrobot_description') + '/urdf/' + p.robot_name + '.xacro'
        additional_urdf_args = ' anchorX:=' + str(conf.robot_params[self.robot_name]['spawn_x'])
        additional_urdf_args += ' anchorY:=' + str(conf.robot_params[self.robot_name]['spawn_y'])
        additional_urdf_args += ' anchorZ:=' + str(conf.robot_params[self.robot_name]['spawn_z'])
        additional_urdf_args += ' anchor2X:=' + str(conf.robot_params[self.robot_name]['spawn_2x'])
        additional_urdf_args += ' anchor2Y:=' + str(conf.robot_params[self.robot_name]['spawn_2y'])
        additional_urdf_args += ' anchor2Z:=' + str(conf.robot_params[self.robot_name]['spawn_2z'])
        super().loadModelAndPublishers(xacro_path=xacro_path, additional_urdf_args=additional_urdf_args)

        self.broadcaster = tf.TransformBroadcaster()
        self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_bumper", ContactsState,
                                             callback=self._receive_contact, queue_size=1, buff_size=2 ** 24,
                                             tcp_nodelay=True)
        if p.landing:
            self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_landing_l_bumper", ContactsState,
                                                 callback=self._receive_contact_landing_l, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_landing_r_bumper", ContactsState,
                                                 callback=self._receive_contact_landing_r, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)


    def updateKinematicsDynamics(self):
        # q is continuously updated
        self.robot.computeAllTerms(self.q, self.qd )
        # joint space inertia matrix                
        self.M = self.robot.mass(self.q )
        # bias terms                
        self.h = self.robot.nle(self.q  , self.qd )
        #gravity terms                
        self.g = self.robot.gravity(self.q )
        #compute ee position  in the world frame  
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.anchor_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor')).translation
        self.anchor_pos2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor_2')).translation
        self.base_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).translation

        self.w_R_b = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).rotation
        self.x_ee =  self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        self.hoist_l_pos = self.base_pos +  self.w_R_b.dot(np.array([0.0, -0.05, 0.05]))
        self.hoist_r_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, 0.05, 0.05]))

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q , self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
        self.Jleg = self.J[:, self.leg_index]
        self.dJdq = self.robot.frameClassicAcceleration(self.q , self.qd , None,  self.robot.model.getFrameId(frame_name), False).linear

        w_R_wire = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire')).rotation
        w_R_wire2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire_2')).rotation


        self.l = self.q[p.rope_index[0]]
        self.ldot = self.qd[p.rope_index[0]]
        self.l_2 = self.q[p.rope_index[1]]
        self.ldot_2 = self.qd[p.rope_index[1]]

        # WF matlab to WF Gazebo offset
        self.mat2Gazebo = self.anchor_pos

        # compute com variables accordint to a frame located at the foot
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data, self.q)

        # from ground truth
        self.com = self.base_pos + robotComB
        # the mountain is always wrt to world
        mountain_pos = np.array([- self.mountain_thickness/2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
        self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/wall', '/world')
        # self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/pillar', '/world')
        # self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/pillar2', '/world')
        if p.landing:
            self.x_landing_l = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wheel_l')).translation
            self.x_landing_r = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wheel_r')).translation

    def _receive_contact(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW = self.robot.framePlacement(self.q,  self.robot.model.getFrameId("lower_link")).rotation.dot(grf)

    def _receive_contact_landing_l(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW_l = self.robot.framePlacement(self.q, self.robot.model.getFrameId("wheel_l")).rotation.dot(grf)

    def _receive_contact_landing_r(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW_r = self.robot.framePlacement(self.q, self.robot.model.getFrameId("wheel_r")).rotation.dot(grf)


    def initVars(self):
        super().initVars()
        self.contactForceW_l = np.zeros(3)
        self.contactForceW_r = np.zeros(3)
        self.qdd_des =  np.zeros(self.robot.na)
        self.base_accel = np.zeros(3)
        # init new logged vars here
        self.com_log =  np.empty((3, conf.robot_params[self.robot_name]['buffer_size'] ))*nan

        self.simp_model_state_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.ldot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.base_pos_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.time_jump_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.rope_normal = np.zeros(3)


    def logData(self):
            # if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
            #     self.simp_model_state_log[:, self.log_counter] = np.array([self.theta, self.phi, self.l])
            #     self.ldot_log[self.log_counter] = self.ldot
            #     self.base_pos_log[:, self.log_counter] = self.base_pos
            #     #self.time_jump_log[self.log_counter] = self.time - self.end_thrusting
            #     pass
            super().logData()

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill -a")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

    # deprecated: this function cannot be used with closed kin chain
    # def resetBase(self, p0 = None):
    #     # create the message
    #     req_reset_joints = SetModelConfigurationRequest()
    #     req_reset_joints.model_name = self.robot_name
    #     req_reset_joints.urdf_param_name = 'robot_description'
    #     req_reset_joints.joint_names = self.joint_names
    #     req_reset_joints.joint_positions = conf.robot_params[self.robot_name]['q_0'].tolist()
    #
    #     # send request and get response (in this case none)
    #     self.reset_joints(req_reset_joints)
    #
    #     print(colored(f"---------Resetting Base", "blue"))

    # Deprecated
    # def spawnMountain(self):
    #     package = 'gazebo_ros'
    #     executable = 'spawn_model'
    #     name = 'spawn_climb_wall'
    #     namespace = '/'
    #     args = '-urdf -param climb_wall -model mountain -x '+ str(conf.robot_params[self.robot_name]['spawn_x'] - self.mountain_thickness/2)+ ' -y '+ str(conf.robot_params[self.robot_name]['spawn_y'])
    #     node = roslaunch.core.Node(package, executable, name, namespace,args=args,output="screen")
    #     self.launch = roslaunch.scriptapi.ROSLaunch()
    #     self.launch.start()
    #     process = self.launch.launch(node)

    def startupProcedure(self):
        #set PD gains
        super().startupProcedure()

    def plotStuff(self):
        if p.numberOfJumps < 2: # do plots only for one jump
            print("PLOTTING")
            # plotFrameLinear('com position', 1, p.time_log, None, p.com_log)
            # plotFrameLinear('contact force', 2, p.time_log, None, p.contactForceW_log)
            traj_gazebo= p.base_pos_log - p.anchor_pos.reshape(3, 1) # is in anchor frame
            time_gazebo = p.time_log - p.start_logging
            plotJoint('position', 0, time_gazebo, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            #plotJoint('torque', 1, time_gazebo, None, None, None, None, None,None, p.tau_ffwd_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            plot3D('basePos', 2,  ['X', 'Y', 'Z'], time_gazebo, traj_gazebo , p.matvars['solution'].time, p.matvars['solution'].p )
            plot3D('states', 3, ['theta', 'phi', 'l'], time_gazebo, p.simp_model_state_log)
            mio.savemat('test_gazebo.mat', {'solution': p.matvars['solution'], 'T_th': p.matvars['T_th'], 'mu': p.matvars['mu'],
                                            'Fun_max':p.matvars['Fun_max'], 'Fr_max':p.matvars['Fr_max'], 'p0':p.matvars['p0'],'pf': p.matvars['pf'],
                                            'time_gazebo': time_gazebo, 'traj_gazebo': traj_gazebo})


    def getIndex(self,t):
        try:
            # get index
            a_bool = self.jumps[self.jumpNumber]["time"] >= t
            idx = min([i for (i, val) in enumerate(a_bool) if val])-1
            if idx == -1:
                return 0
            else:
                return idx
        except:
            return  -1

    def getImpulseAngle(self):
        if p.MARCO_APPROACH:
            max_Fut = self.jumps[self.jumpNumber]["Fut"][self.getIndex(self.jumps[self.jumpNumber]["thrustDuration"]/2)]
            max_Fun = self.jumps[self.jumpNumber]["Fun"][self.getIndex(self.jumps[self.jumpNumber]["thrustDuration"]/2)]
            angle = math.atan2(max_Fut,max_Fun)
        else:
            angle =  math.atan2(self.jumps[self.jumpNumber]["Fut"],
                                self.jumps[self.jumpNumber]["Fun"])

        return angle, max_Fut, max_Fun


    # compute the passive and rope joints reference from the matlab position referred to a world frame located in between anchor
    def computeJointVariables(self, p):
        # mountain_wire_pitch_l = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_x'], -p[2])
        # mountain_wire_pitch_r = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_2x'], -p[2])
        mountain_wire_pitch_l = math.atan2(p[0] , -p[2])
        mountain_wire_pitch_r = math.atan2(p[0] , -p[2])

        anchor_distance_y = (self.anchor_pos2 - self.anchor_pos)[1]
        mountain_wire_roll_l = -math.atan2(-p[2], p[1])
        mountain_wire_roll_r = math.atan2(-p[2], anchor_distance_y-p[1])

        wire_base_prismatic_l = np.linalg.norm(p) -anchor_distance_y*0.5
        wire_base_prismatic_r = math.sqrt(p[0]*p[0] +(anchor_distance_y - p[1])*(anchor_distance_y - p[1]) + p[2] * p[2])-anchor_distance_y*0.5

        wire_base_roll_l = -mountain_wire_roll_l
        wire_base_roll_r = -mountain_wire_roll_r
        return [mountain_wire_pitch_r, mountain_wire_roll_r,  wire_base_prismatic_r, 0., wire_base_roll_r, 0.,
                mountain_wire_pitch_l, mountain_wire_roll_l,  wire_base_prismatic_l, 0., wire_base_roll_l, 0.]

def talker(p):
    p.start()
    p.startSimulator(world_name="accurate.world", additional_args= ['spawn_2x:=' + str(conf.robot_params[p.robot_name]['spawn_2x']),
                                                     'spawn_2y:=' + str(conf.robot_params[p.robot_name]['spawn_2y']),
                                                     'spawn_2z:=' + str(conf.robot_params[p.robot_name]['spawn_2z'])])
    p.loadModelAndPublishers()

    p.startupProcedure()
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.updateKinematicsDynamics()

    # jump parameters
    p.startJump = 2.5
    p.orientTime = 0.5
    # p0 is defined wrt anchor1 pos
    p0 = np.array([0.0, 2.5, -6])
    p.q_des[:12] = p.computeJointVariables(p0)
    jumpN = 0
    # while not ros.is_shutdown():
    #     p.updateKinematicsDynamics()
    #     #multiple jump test
    #     if (jumpN == 0) and (p.time >3.): #change target
    #         p.pid.setPDjoint(p.anchor_passive_joints, 0., 0., 0.)
    #         p0 = np.array([0.0, 3, -8])
    #         print(colored(f"Jumpping to {p0}", "red"))
    #         p.applyForce(100, 0, 0., 0.02)
    #         p.q_des[:12] = p.computeJointVariables(p0)
    #         print(p.q_des[:12])
    #         jumpN += 1
    #     if (jumpN == 1) and (p.time > 5.5):  # change target
    #         p0 = np.array([0.0, 4.5, -8])
    #         print(colored(f"Jumpping to {p0}", "red"))
    #         p.applyForce(100, 0, 0., 0.02)
    #         p.q_des[:12] = p.computeJointVariables(p0)
    #         print(p.q_des[:12])
    #         jumpN += 1
    #     if (jumpN == 2) and (p.time > 10):  # change target
    #         p0 = np.array([0.0, 2, -6])
    #         print(colored(f"Jumpping to {p0}", "red"))
    #         p.applyForce(100, 0, 0., 0.02)
    #         p.q_des[:12] = p.computeJointVariables(p0)
    #         print(p.q_des[:12])
    #         jumpN += 1
    #     # compensate gravity only for robpe joint
    #     p.tau_ffwd[p.rope_index] = p.g[p.rope_index]
    #     p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
    #     p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3)  # to avoid issues of dt 0.0009999
    #     p.logData()
    #     p.ros_pub.add_arrow(p.anchor_pos, (p.base_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
    #     p.ros_pub.add_arrow(p.anchor_pos2, (p.base_pos - p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo
    #     p.ros_pub.add_arrow(p.x_ee, p.contactForceW / 5., "blue", scale=4.5)
    #     p.ros_pub.add_marker(p.mat2Gazebo + p0, color="red", radius=0.2)
    #     p.ros_pub.publishVisual()
    #     rate.sleep()

    # validation test matlab
    Fr_l = 0
    Fr_r = 0
    while not ros.is_shutdown():
        p.updateKinematicsDynamics()
        if (jumpN == 0) and (p.time >p.startJump): #change target
            print(colored(f"Start Matlab Validation Test", "red"))
            p.pid.setPDjoint(p.anchor_passive_joints, 0., 0., 0.)
            p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
            if p.landing:
                Fr_l = -90
                Fr_r = -120
                Fleg = 1000
            else:
                Fr_l = -30
                Fr_r = -40
                Fleg = 200
            p.tau_ffwd[p.rope_index[0]] = Fr_l
            p.tau_ffwd[p.rope_index[1]] = Fr_r
            p.w_Fleg = np.array([Fleg, 0., 0.])
            if p.EXTERNAL_FORCE:
                p.applyForce(Fleg, 0, 0., 0.05)
            jumpN+=1
        if (jumpN ==1):
            if not p.EXTERNAL_FORCE and p.time<(p.startJump + 0.05):
                p.tau_ffwd[p.leg_index] = -p.Jleg.T.dot(p.w_Fleg)
                p.ros_pub.add_arrow(p.x_ee, p.w_Fleg / 20., "red", scale=4.5)
            else:
                p.tau_ffwd[p.leg_index] = np.zeros(3)

                if  p.landing:
                    # retract knee joint and extend landing joints
                    p.tau_ffwd[p.landing_joints] = np.zeros(2)
                    p.q_des[p.leg_index[2]] = 0.3
                    p.q_des[p.landing_joints] = np.array([-0.8, 0.8])
            if (p.base_pos[0] < conf.robot_params[p.robot_name]['spawn_x']):
                print(colored(f"target in matlab is: {p.base_pos-p.mat2Gazebo}"))
                break

        rope_direction = (p.hoist_l_pos - p.anchor_pos) / np.linalg.norm(p.hoist_l_pos  - p.anchor_pos)
        p.ros_pub.add_arrow(p.hoist_l_pos, rope_direction * Fr_l / 50., "red", scale=4.5)
        rope_direction2 = (p.hoist_r_pos - p.anchor_pos2) / np.linalg.norm(p.hoist_r_pos  - p.anchor_pos2)
        p.ros_pub.add_arrow(p.hoist_r_pos, rope_direction2 * Fr_r / 50., "red", scale=4.5)


        p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
        p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos-p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo
        if p.landing:
            p.ros_pub.add_arrow(p.x_landing_l, p.contactForceW_l / 20., "blue", scale=4.5)
            p.ros_pub.add_arrow(p.x_landing_r, p.contactForceW_r / 20., "blue", scale=4.5)
        p.ros_pub.add_arrow(p.x_ee, p.contactForceW / 20., "blue", scale=4.5)
        p.ros_pub.add_marker(p.mat2Gazebo + p0, color="red", radius=0.2)

        p.ros_pub.publishVisual()
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),
                          3)  # to avoid issues of dt 0.0009999
        p.logData()
        rate.sleep()

    import sys
    sys.exit()
    #p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravitu in the virtual joint to go exactly there
    p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)


    #override with matlab stuff
    if p.LOAD_MATLAB_TRAJ:
        if p.MARCO_APPROACH:
            # single jump
            p.matvars = mio.loadmat('test_optim_marco.mat', squeeze_me=True,struct_as_record=False)
            p.jumps = [{"time": p.matvars['solution'].time, "thrustDuration" : p.matvars['T_th'], "p0": p.matvars['p0'], "targetPos": p.matvars['pf'],  "Fun": p.matvars['solution'].Fun, "Fut": p.matvars['solution'].Fut,  "Fr": p.matvars['solution'].Fr,  "K_rope": 0.0, "Tf": p.matvars['solution'].Tf }]
            #double jump
            #p.matvars1 = mio.loadmat('test_optim_marco1.mat', squeeze_me=True, struct_as_record=False)
            #p.matvars2 = mio.loadmat('test_optim_marco2.mat', squeeze_me=True, struct_as_record=False)
            #p.jumps = [{"time": p.matvars1['solution'].time, "thrustDuration" : p.matvars1['T_th'], "p0": p.matvars['p0'], "targetPos": p.matvars1['pf'], "Fun": p.matvars1['solution'].Fun, "Fut": p.matvars1['solution'].Fut,  "Fr": p.matvars1['solution'].Fr, "K_rope": 0.0, "Tf": p.matvars1['solution'].Tf - p.matvars1['T_th']},
            #            {"time": p.matvars2['solution'].time, "thrustDuration" : p.matvars2['T_th'], "p0": p.matvars['p0'], "targetPos": p.matvars2['pf'], "Fun": p.matvars2['solution'].Fun, "Fut": p.matvars2['solution'].Fut,  "Fr": p.matvars2['solution'].Fr, "K_rope": 0.0, "Tf": p.matvars2['solution'].Tf - p.matvars2['T_th']}]

        else:#my approach
            p.matvars = mio.loadmat('test_optim.mat', squeeze_me=True, struct_as_record=False)
            p.jumps = [{"thrustDuration" : p.matvars['T_th'], "p0": p.matvars['p0'],  "targetPos": p.matvars['pf'], "Fun": p.matvars['solution'].Fun, "Fut": p.matvars['solution'].Fut,  "Fr": p.matvars['solution'].Fr,  "K_rope": 0.0, "Tf": p.matvars['solution'].Tf -  p.matvars['T_th']}]

    p.stateMachine = 'idle'
    p.jumpNumber  = 0
    p.numberOfJumps = len(p.jumps)
    p.start_logging = np.inf

    while not ros.is_shutdown():

        # update the kinematics
        p.updateKinematicsDynamics()

        #multiple jumps state machine
        if ( p.stateMachine == 'idle') and (p.time >= p.startJump) and (p.jumpNumber<p.numberOfJumps):
            print("\033[34m"+"---------Starting jump  number ", p.jumpNumber, " to target: ", p.jumps[p.jumpNumber]["targetPos"], " from p0 : ", p.jumps[p.jumpNumber]["p0"])

            p.l_0 = p.l
            # if p.jumpNumber == 0: # do only once
            #     p.resetBase(p.jumps[p.jumpNumber]["p0"])
            if (p.EXTERNAL_FORCE):
                print(colored("Start applying force", "red"))
                p.applyForce( p.jumps[p.jumpNumber]["Fun"], p.jumps[p.jumpNumber]["Fut"], 0., p.jumps[p.jumpNumber]["thrustDuration"])
                p.stateMachine = 'thrusting'
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
                p.end_thrusting = p.startJump + p.jumps[p.jumpNumber]["thrustDuration"]
                print(colored("Start Thrusting with EXT FORCE", "blue"))
            else:

                # strategy 1 - orientation hip roll joint
                #p.q_des[7] = p.getImpulseAngle()[0]
                # strategy 2 -  align the body (keep HR to 1.57)
                #p.q_des[5] = p.getImpulseAngle()[0]
                print(colored(f"Start orienting leg to  : {p.getImpulseAngle()[0]}", "blue"))
                p.stateMachine = 'orienting_leg'
                #set the end of orienting
                p.end_orienting = p.startJump + p.orientTime
                p.end_thrusting = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"]

            if p.MARCO_APPROACH:
                p.start_logging = p.end_orienting
            else:
                p.start_logging = p.end_thrusting

        if (p.stateMachine == 'orienting_leg') and (p.time >= p.end_orienting):
            print(colored(f"Start trusting", "blue"))
            p.tau_ffwd = np.zeros(p.robot.na)
            p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravitu in the virtual joint to go exactly there
            p.pid.setPDjoint(p.base_passive_joints, 0., 0., 0.)
            p.pid.setPDjoint(p.leg_index, 0., 0., 0.)
            print(colored(f"ZERO LEG AND ROPE PD", "red"))
            p.stateMachine = 'thrusting'
            if p.MARCO_APPROACH:
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)

        if (p.stateMachine == 'thrusting'):
            if (p.EXTERNAL_FORCE):
                p.ros_pub.add_arrow(p.base_pos, np.array([p.jumps[p.jumpNumber]["Fun"], p.jumps[p.jumpNumber]["Fut"], 0.]) / 50., "red", scale= 4.5)
            else:
                if not p.MARCO_APPROACH:
                    p.b_Fu_xy = np.array([p.jumps[p.jumpNumber]["Fun"], p.jumps[p.jumpNumber]["Fut"]])
                else:
                    # in marco's approach these are vectors not scalars and I start to give the rope at the beginning
                    #p.b_Fu_xy =  p.jumps[p.jumpNumber]["Fun"][p.getIndex(p.time - p.end_orienting)], p.jumps[p.jumpNumber]["Fut"][p.getIndex(p.time - p.end_orienting)]
                    # required from strategy 2 -   with body aligned with impulse direction
                    Fu = np.array([  p.jumps[p.jumpNumber]["Fun"][p.getIndex(p.time - p.end_orienting)],  p.jumps[p.jumpNumber]["Fut"][p.getIndex(p.time - p.end_orienting) ]  ])
                    Fu_norm = np.linalg.norm(Fu)
                    p.b_Fu_xy = np.hstack((Fu_norm ,0.))

                    # start already to use the rope
                    Fr_l = p.jumps[p.jumpNumber]["Fr"][p.getIndex(p.time - p.end_orienting)]
                    Fr_r = p.jumps[p.jumpNumber]["Fr"][p.getIndex(p.time - p.end_orienting)]

                    p.tau_ffwd[p.rope_index[0]] = Fr_l
                    p.tau_ffwd[p.rope_index[1]] = Fr_r
                    rope_direction = (p.hoist_l_pos - p.anchor_pos) / np.linalg.norm(p.hoist_l_pos- p.anchor_pos)
                    p.ros_pub.add_arrow(p.hoist_l_pos, rope_direction * Fr_l / 50., "red", scale=4.5)
                    rope_direction2 = (p.hoist_r_pos - p.anchor_pos2) / np.linalg.norm(p.hoist_r_pos - p.anchor_pos2)
                    p.ros_pub.add_arrow(p.hoist_r_pos, rope_direction2 * Fr_r / 50., "red", scale=4.5)
                #using all 3 leg joints to generate impulse
                p.w_Fu = p.w_R_b.dot(np.hstack((p.b_Fu_xy, 0)))
                p.tau_ffwd[p.leg_index] = - p.Jleg.T.dot(p.w_Fu)
                p.ros_pub.add_arrow(p.x_ee, p.w_Fu / 50., "red", scale = 4.5)

            if (p.time > p.end_thrusting):
                print(colored("Stop Trhusting", "blue"))
                p.stateMachine = 'flying'
                print(colored(f"RESTORING LEG PD", "red"))
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
                # reenable  the PDs of default values for landing and reset the torque on the leg
                p.pid.setPDjoint(p.base_passive_joints, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'] ,  0.)
                # reenable leg pd
                p.pid.setPDjoint(p.leg_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'],  0.)
                p.tau_ffwd[p.leg_index] = np.zeros(len(p.leg_index))
                print(colored("Start Flying", "blue"))

        if (p.stateMachine == 'flying'):
            # keep extending rope
            if p.LOAD_MATLAB_TRAJ:
                if not p.MARCO_APPROACH:
                    delta_t = p.time - p.end_thrusting
                else:
                    delta_t = p.time - p.end_orienting
                Fr =  p.jumps[p.jumpNumber]["Fr"][p.getIndex(delta_t)]

            p.tau_ffwd[p.rope_index[0]] = Fr
            p.tau_ffwd[p.rope_index[1]] = Fr
            rope_direction =  (p.base_pos - p.anchor_pos)/np.linalg.norm(p.base_pos - p.anchor_pos)
            rope_direction2 = (p.base_pos - p.anchor_pos2) / np.linalg.norm(p.base_pos - p.anchor_pos2)
            p.ros_pub.add_arrow( p.base_pos, rope_direction*Fr/50., "red", scale=4.5)
            p.ros_pub.add_arrow( p.base_pos, rope_direction2*Fr/50., "red", scale=4.5)
            if (p.EXTERNAL_FORCE):
                end_flying = p.startJump  + p.jumps[p.jumpNumber]["thrustDuration"]+  p.jumps[p.jumpNumber]["Tf"]
            else:
                end_flying = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"] + p.jumps[p.jumpNumber]["Tf"]

            if (p.time >= end_flying):
                print(colored("Stop Flying", "blue"))
                # reset the qdes
                p.stateMachine = 'idle'
                print(colored(f"RESTORING ROPE PD", "red"))
                # enable PD for rope and reset the PD reference to the new estension
                # sample the new elongation
                p.q_des[p.rope_index[0]] = np.copy(p.q[p.rope_index[0]])
                p.q_des[p.rope_index[1]] = np.copy(p.q[p.rope_index[1]])
                p.l_0 = np.copy(p.l)
                p.q_des[p.rope_index[0]] = 0
                p.q_des[p.rope_index[1]] = 0

                p.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)

                p.jumpNumber += 1

                if (p.jumpNumber < p.numberOfJumps):
                    # reset for multiple jumps
                    p.startJump = p.time
                else:
                    p.pause_physics_client()
                    break

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        if (p.time >= p.start_logging):
            p.logData()

        # plot  rope bw base link and anchor
        p.ros_pub.add_arrow(p.anchor_pos, (p.base_pos - p.anchor_pos), "green", scale = 3.)# arope, already in gazebo
        p.ros_pub.add_arrow(p.anchor_pos2, (p.base_pos - p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo
        #p.ros_pub.add_marker(p.anchor_pos, radius=0.8, color= "green") #already in gazebo
        p.ros_pub.add_marker(p.anchor_pos + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.8)

        # p.ros_pub.add_arrow(p.x_ee, p.rope_normal, color=[0, 0, 0], scale=4.5)
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        if (p.jumpNumber == p.numberOfJumps):
            p.ros_pub.add_marker(p.x_ee, color="green", radius=0.15)
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        # commented because it no longer a  nice impulse when I changed orientation strategy
        #p.ros_pub.add_arrow(p.x_ee, p.contactForceW / 50., "blue", scale = 4.5)
        p.ros_pub.publishVisual()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

def plot3D(name, figure_id, label, time_log, var, time_mat = None, var_mat = None):
    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)

    plt.subplot(3,1,1)
    plt.ylabel(label[0])
    plt.plot(time_log, var[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

    plt.subplot(3,1,2)
    plt.ylabel(label[1])
    plt.plot(time_log, var[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

    plt.subplot(3,1,3)
    plt.ylabel(label[2])
    plt.plot(time_log, var[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

if __name__ == '__main__':
    p = ClimbingrobotController(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log,
                  joint_names=conf.robot_params[p.robot_name]['joint_names'])
        plotJoint('torque', 1, p.time_log, tau_log=p.tau_log, tau_ffwd_log = p.tau_ffwd_log,
                  joint_names=conf.robot_params[p.robot_name]['joint_names'])
    finally:
        pass
        # if conf.plotting:
        #     p.plotStuff()


        
