import numpy as np
from base_controllers.tracked_robot.simulator.tracked_vehicle import TrackedVehicle, VehicleParam
from base_controllers.tracked_robot.simulator.track import TrackParams
from numpy.testing import assert_almost_equal
from matplotlib import pyplot as plt
from  base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
import base_controllers.tracked_robot.utils.constants as constants
from base_controllers.utils.math_tools import computeOrientationError
from base_controllers.utils.math_tools import Math
#to debug
from base_controllers.utils.common_functions import  launchFileGeneric
from base_controllers.utils.ros_publish import RosPub
import rospkg
import os
import pinocchio as pin
import rospy as ros
import tf
from termcolor import colored

class Ground3D():
    def __init__(self,
                 cohesion=0.0001040 * 1e6,
                 K=0.001,
                 shear_resistance_angle=20/180.*2*np.pi,
                 g=9.81,
                 friction_coefficient=0.1, # 0.1 is very low and on slopes it slips!
                 terrain_stiffness = 1e06,
                 terrain_damping = 1e04,
                 terrain_torsional_stiffness=1e05,
                 terrain_torsional_damping=1e04):
        self.cohesion = cohesion  # [Pa] #not used
        self.K = K  # [m]
        self.shear_resistance_angle = shear_resistance_angle  # [rad] #not used
        self.g = g
        self.friction_coefficient = friction_coefficient
        self.Kt_x = np.eye(3)*terrain_stiffness
        self.Dt_x = np.eye(3) * terrain_damping
        self.Kt_theta = np.eye(3) * terrain_torsional_stiffness
        self.Dt_theta = np.eye(3) * terrain_torsional_damping


class TrackedVehicleSimulator3D:
    def __init__(self, dt=0.001, ground=None, USE_MESH = False, DEBUG=False):
        self.NO_SLIPPAGE = False
        self.USE_MESH = USE_MESH
        self.DEBUG = DEBUG

        self.dt = dt
        self.vehicle_param = VehicleParam()
        self.track_param = TrackParams()
        if ground is not None:
            self.ground = ground
        else:
            self.ground = Ground3D()
        self.sigma = np.ones((self.track_param.parts_longitudinal, self.track_param.parts_lateral)) * self.vehicle_param.weight / (2 * self.track_param.A)
        self.tracked_robot = TrackedVehicle(self.vehicle_param, self.track_param,  self)
        self.patch_pos_long_l, self.patch_pos_lat_l = self.tracked_robot.getLeftPatchesPositions()
        self.patch_pos_long_r, self.patch_pos_lat_r = self.tracked_robot.getRightPatchesPositions()

        #these are just for the unit test
        self.t_end = 20.  # [s]
        self.pose_init = [0., 0., 0.0,0,0,0]  # position in WF, rpy angles
        self.twist_init = [0, 0.0, 0,0,0,0]  # in WF
        self.number_of_steps = np.int32(self.t_end / self.dt)
        self.pose_des_log = np.full((6, self.number_of_steps), np.nan)
        self.pose_log = np.full((6, self.number_of_steps), np.nan)
        self.time_log = np.full((self.number_of_steps), np.nan)
        self.math_utils = Math()

        self.q_des = np.zeros(2)
        self.qd_des = np.zeros(2)
        self.pose_des = np.zeros(3)
        self.orient_des = np.zeros(3)

    def computeGroundForcesMoments(self,pose, twist, w_R_b, pg, terr_roll, terr_pitch, terr_yaw):
        pc = pose[:3]
        pc_d = twist[:3]
        w_omega = twist[3:]
        w_R_terr = self.math_utils.eul2Rot(np.array([terr_roll, terr_pitch, terr_yaw]))
        self.terrain_normal = w_R_terr[:,2]

        #projection of com on the ground level
        pc_ = pc# - w_R_b[:,2]*self.vehicle_param.height

        #force is present only if there is linear penetration
        # print("pg", pg)
        # print("pcom_",pc_)
        if (self.terrain_normal.dot(pg - pc_) > 0.0):
            Fk = self.ground.Kt_x.dot(pg - pc_)
            #only if it is approaching
            Fd = np.zeros(3)
            if (self.terrain_normal.dot(pc_d) < 0.0):
                Fd = -self.ground.Dt_x.dot(pc_d)
            else:
                Fd = np.zeros(3)
            # project along terrain normal
            w_Fg =  self.terrain_normal*self.terrain_normal.dot(Fk + Fd)
        else:
            w_Fg = np.array([0.0, 0.0, 0.0])


        #moments due to penetration
        #compute projector on tx, ty contact plane
        N = np.eye(3) -  np.multiply.outer(self.terrain_normal.ravel(), self.terrain_normal.ravel())
        w_e_o =   computeOrientationError(w_R_b, w_R_terr)
        b_e_o = w_R_b.T.dot(w_e_o)
        #this projection is fundamental
        w_Mg = N.dot(self.ground.Kt_theta.dot(w_e_o) - self.ground.Dt_theta.dot(w_omega))

        #map to base frame
        b_Fg = w_R_b.T.dot(w_Fg)
        b_Mg = w_R_b.T.dot(w_Mg)

        return b_Fg, b_Mg, w_e_o[0],b_e_o[1]

    def dynamics3D(self, twist, w_R_b, b_Fg,b_Mg, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, vehicle_param):
        # calculate the acceleration with the vehicle dynamical model
        m = vehicle_param.mass
        bI = vehicle_param.bI

        #map everything to BF
        b_vc = w_R_b.T.dot(twist[:3])
        b_omega = w_R_b.T.dot(twist[3:])

        #compute traction forces and moments
        b_Ft = np.zeros(3)
        b_Ft[0] = Fx_l + Fx_r
        b_Ft[1] = Fy_l + Fy_r
        b_Mt = np.zeros(3)
        b_Mt[2] = M_long_l + M_lat_l + M_long_r + M_lat_r
        #compute gravity force
        b_Fgrav = w_R_b.T.dot(m*np.array([0.,0.,-self.ground.g]))
        #compute non linear terms
        NL_lin = m*(np.cross(b_omega,b_vc))
        NL_ang = np.cross(b_omega,  bI.dot(b_omega))

        #debug
        w_Ft = w_R_b.dot(b_Ft) #traction force
        w_Fg = w_R_b.dot(b_Fg) # terrain linear
        w_Mg = w_R_b.dot(b_Mg) # terrain moment
        w_Fgrav = w_R_b.dot(b_Fgrav) #gravity force
        if self.DEBUG:
            self.ros_pub.add_arrow(self.pose[:3], w_Ft / 100., "red")
            self.ros_pub.add_arrow(self.pose[:3], w_Fgrav / 1000., "blue")
            self.ros_pub.add_arrow(self.pose[:3], w_Fg / 1000., "green")
            self.ros_pub.add_arrow(self.pose[:3], w_Mg / np.linalg.norm(w_Mg), "green") #should be purely lateral flipping back forth on ramp with only pitch

        b_vc_dot =  1/m*(b_Ft + b_Fgrav + b_Fg-NL_lin)
        b_omega_dot = np.linalg.inv(bI).dot(b_Mt + b_Mg-NL_ang)

        w_twist_dot = np.concatenate((w_R_b.dot(b_vc_dot), w_R_b.dot(b_omega_dot)))
        # print("b_Ft",b_Ft)
        # print("b_Fgrav",b_Fgrav)
        #print("b_Mg", b_Mg)
        return w_twist_dot

    def integrateTwist(self, pose, twist):
        rpy = pose[3:]
        # TOD replace with RK4
        pose[:3] += twist[:3] * self.dt
        rpy_dot = np.linalg.inv(self.math_utils.Tomega(rpy)).dot(twist[3:])
        pose[3:] +=rpy_dot* self.dt
        return pose

    def initSimulation(self,pose_init =np.zeros(6),  twist_init=np.zeros(6), ros_pub = None):
        self.pose = pose_init
        self.twist = twist_init
        #self.pose[2] = self.vehicle_param.height
        if ros_pub is None:
            self.ros_pub = RosPub('tractor', only_visual=True)
        else:
            self.ros_pub =ros_pub

    def simulateOneStep(self,pg, terrain_roll, terrain_pitch, terrain_yaw, omega_left, omega_right):
        # compute base orientation
        w_R_b = self.math_utils.eul2Rot(self.pose[3:])

        #get states in base frame b_v_c
        b_v_c = w_R_b.T.dot(self.twist[:3])
        state2D = np.array([b_v_c[0],b_v_c[1], self.twist[5]])


        Fx_l, Fy_l, M_long_l, M_lat_l = self.tracked_robot.track_left.computeTerrainInteractions(state2D, omega_left, self.track_param,
                                                                                   self.sigma, self.ground, self.patch_pos_long_l, self.patch_pos_lat_l)
        Fx_r, Fy_r, M_long_r, M_lat_r= self.tracked_robot.track_right.computeTerrainInteractions(state2D, omega_right, self.track_param,
                                                                                   self.sigma, self.ground, self.patch_pos_long_r, self.patch_pos_lat_r)


        # compute ground peneration
        b_Fg, b_Mg, b_eox, b_eoy = self.computeGroundForcesMoments(self.pose, self.twist, w_R_b, pg, terrain_roll, terrain_pitch, terrain_yaw)

        if self.NO_SLIPPAGE:
            #extension of unicycle to 3d
            vel = constants.SPROCKET_RADIUS * (omega_left + omega_right) / 2
            omega = constants.SPROCKET_RADIUS / constants.TRACK_WIDTH * (omega_right - omega_left)
            self.twist[:3] = w_R_b.dot(np.array([vel,0,0]))
            self.twist[3:] = w_R_b.dot(np.array([0, 0, omega]))
        else:
            #update twist from dynamics
            self.twist += self.dynamics3D(self.twist, w_R_b, b_Fg, b_Mg, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, self.vehicle_param) * self.dt
        self.pose  = self.integrateTwist(self.pose, self.twist)
        return b_eox, b_eoy

    def computeZcomponent(self, x, y, pitch):#TODO use mesh
        return x * np.tan(-pitch)


    def simulate(self, hf_v, w_omega_z):
        self.time = 0.
        sim_counter = 0
        self.rate = ros.Rate(1 /  (self.dt))

        while self.time < self.t_end:
            if np.mod(self.time, 1) == 0:
                print(colored(f"TIME: {self.time}","red"))

            #get actual orient
            self.euler = self.pose[3:]
            self.w_R_b = self.math_utils.eul2Rot(self.euler)
            self.hf_R_b = self.math_utils.eul2Rot(np.array([self.euler[0],self.euler[1], 0.]))
            # get des pos
            des_x, des_y, des_theta, _, _, _, _, _ = self.traj.evalTraj(self.time)

            if self.USE_MESH:
                #base_x_axis = self.math_utils.eul2Rot(self.pose[3:])[:, 0]
                pg, terrain_roll, terrain_pitch, terrain_yaw = self.terrainManager.project_on_mesh(point=self.pose[:2],  direction=np.array([0., 0., 1.]))

                self.pose_des,terrain_roll_des,terrain_pitch_des, terrain_yaw_des = self.terrainManager.project_on_mesh(point=np.array([des_x, des_y]), direction=np.array([0., 0., 1.]))
                self.orient_des = np.array([0, 0, des_theta]) #first two elements are the expected values of b_eox, b_eoy

                w_R_terr = self.math_utils.eul2Rot(np.array([terrain_roll, terrain_pitch, terrain_yaw]))
                w_normal = w_R_terr.dot(np.array([0, 0, 1]))

                # print("pose ", self.pose[:3])
                # print("pg ", pg)
                #for debug
                # print("pose ", self.pose[:3])
                # print("pg ", pg)
                if self.DEBUG:
                    self.ros_pub.add_mesh("tractor_description", "/meshes/terrain.stl", position=np.array([0., 0., 0.0]), color="red", alpha=1.0)
                    self.ros_pub.add_arrow(pg, w_normal*0.5, color="white")
                    self.ros_pub.add_marker(pg, radius=0.1, color="white", alpha = 1.)
            else:
                terrain_roll = self.terrain_roll_vec[sim_counter]
                terrain_pitch = self.terrain_pitch_vec[sim_counter]
                terrain_yaw =  self.terrain_yaw_vec[sim_counter]
                pg = np.array([self.pose[0], self.pose[1], self.computeZcomponent(self.pose[0], self.pose[1], terrain_pitch)])
                self.pose_des = np.array([des_x, des_y, self.computeZcomponent(des_x, des_y, terrain_pitch)])
                self.orient_des = np.array([0, 0, des_theta])

            #project hf_v onto hf_x_b
            b_v = self.hf_R_b[0].dot(np.array([hf_v[sim_counter], 0., 0.]))
            #project w_omega_z onto w_z_b
            b_omega = self.w_R_b[2].dot(np.array([0.,0., w_omega_z[sim_counter]]))

            # convert to wheels
            r = self.track_param.sprocket_radius
            B = self.vehicle_param.width
            omega_left = (b_v - b_omega * B / 2.0) / r
            omega_right = (b_v + b_omega * B / 2.0) / r

            #updates pose and twist
            b_eox, b_eoy = self.simulateOneStep(pg, terrain_roll, terrain_pitch, terrain_yaw, omega_left, omega_right)

            #log
            self.pose_des_log[:3, sim_counter] = self.pose_des
            self.pose_des_log[3:, sim_counter] = self.orient_des
            self.pose_log[:, sim_counter] = np.concatenate((self.pose[:3], np.array([b_eox, b_eoy, self.pose[5]])))

            self.time_log[sim_counter] = self.time


            sim_counter +=1

            self.time = np.round(self.time + self.dt, 4)

            #debug
            #time.sleep(0.2)
            #ros.sleep(self.dt)
            self.euler = self.pose[3:]
            self.quaternion = pin.Quaternion(self.w_R_b)
            self.broadcaster.sendTransform(self.pose[:3],
                                           self.quaternion,
                                           ros.Time.now(), '/base_link', '/world')
            self.ros_pub.publishVisual(delete_markers=False)

            self.rate.sleep()
            if ros.is_shutdown():
                break
        os.system("rosnode kill rviz")
        os.system("pkill rosmaster")

    def getRobotState(self):
        return self.pose, self.twist

if __name__ == '__main__':
    groundParams = Ground3D()
    p = TrackedVehicleSimulator3D(dt=0.0005, ground=groundParams,USE_MESH=False, DEBUG=True)

    # to debug
    launchFileGeneric(rospkg.RosPack().get_path('tractor_description') + "/launch/rviz_nojoints.launch")
    p.broadcaster = tf.TransformBroadcaster()

    if p.USE_MESH:
        from base_controllers.tracked_robot.simulator.terrain_manager import TerrainManager
        p.terrainManager = TerrainManager(rospkg.RosPack().get_path('tractor_description') + "/meshes/terrain.stl")
        #start the robot on the mesh
        start_position, start_roll, start_pitch, start_yaw = p.terrainManager.project_on_mesh(point=p.pose_init[:2], direction=np.array([0., 0., 1.]))
        p.pose_init[:3] = start_position.copy()
        p.pose_init[3] = start_roll
        p.pose_init[4] = start_pitch
        p.pose_init[5] = start_yaw
    else:
        p.terrain_roll_vec = np.linspace(0.0, 0.0, p.number_of_steps)
        p.terrain_pitch_vec = np.linspace(0.0, -0.1, p.number_of_steps) # if you set -0.3 the robot starts to slip backwards!
        p.terrain_yaw_vec = np.linspace(0.0, -0.0, p.number_of_steps) # if you set -0.3 the robot starts to slip backwards!

    #gen traj
    hf_v = np.linspace(0.4, 0.4, p.number_of_steps)
    w_omega_z = np.linspace(0.0, 0.0, p.number_of_steps)
    #w_omega_z = np.linspace(0.4, 0.4, p.number_of_steps)

    #the trajectory is in the WF
    p.traj = Trajectory(ModelsList.UNICYCLE, start_x=p.pose_init[0],
                        start_y= p.pose_init[1], start_theta=p.pose_init[5],
                        DT=p.dt, v=hf_v, omega=w_omega_z)



    # init vars
    p.initSimulation(p.pose_init, p.twist_init)

    # simulation with internal while loop
    p.simulate(hf_v, w_omega_z)

    if not p.USE_MESH:#unit test
        print(p.pose_log[:,-1])
        assert_almost_equal(p.pose_log[0, -1], 7.729048908314892 , decimal=2)
        assert_almost_equal(p.pose_log[1, -1],0. , decimal=2)
        assert_almost_equal(p.pose_log[2, -1],     0.77491, decimal=2)
        assert_almost_equal(p.pose_log[3, -1], 0., decimal=2)
        assert_almost_equal(p.pose_log[4, -1],  -0.0005 , decimal=2)
        assert_almost_equal(p.pose_log[5, -1], 0., decimal=2)


    # xy plot
    plt.figure()
    plt.plot(p.pose_log[0, :-1], p.pose_log[1, :-1], "-b", label="actual")
    plt.plot(p.pose_des_log[0, :-1], p.pose_des_log[1, :-1], "-r", label="desired")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)

    fig = plt.figure()
    fig.suptitle("lin states", fontsize=20)
    plt.subplot(3, 1, 1)
    plt.ylabel("X")
    plt.plot(p.time_log[:-1], p.pose_log[0, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[0, :-1], linestyle='-', lw=3, color='red')
    plt.grid()
    plt.subplot(3, 1, 2)
    plt.ylabel("Y")
    plt.plot(p.time_log[:-1], p.pose_log[1, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[1, :-1], linestyle='-', lw=3, color='red')
    plt.grid()
    plt.ylim([-10, 10])
    plt.subplot(3, 1, 3)
    plt.ylabel("Z")
    plt.plot(p.time_log[:-1], p.pose_log[2, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[2, :-1], linestyle='-', lw=3, color='red')
    plt.xlabel("Time [s]")
    plt.grid()


    fig = plt.figure()
    fig.suptitle("ang states", fontsize=20)
    plt.subplot(3, 1, 1)
    plt.ylabel("roll")
    plt.plot(p.time_log[:-1], p.pose_log[3, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[3, :-1], linestyle='-', lw=3, color='red')
    plt.grid()
    plt.ylim([-0.5,0.5])
    plt.subplot(3, 1, 2)
    plt.ylabel("pitch")
    plt.plot(p.time_log[:-1], p.pose_log[4, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[4, :-1], linestyle='-', lw=3, color='red')
    plt.grid()
    plt.ylim([-0.5,0.5])
    plt.subplot(3, 1, 3)
    plt.ylabel("yaw")
    plt.plot(p.time_log[:-1], p.pose_log[5, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[5, :-1], linestyle='-', lw=3, color='red')
    plt.xlabel("Time [s]")
    plt.grid()
    plt.ylim([-5,5])
