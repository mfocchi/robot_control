import numpy as np
from base_controllers.tracked_robot.simulator.tracked_vehicle import TrackedVehicle, VehicleParam
from base_controllers.tracked_robot.simulator.track import TrackParams
from numpy.testing import assert_almost_equal
from matplotlib import pyplot as plt
from  base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
import base_controllers.tracked_robot.utils.constants as constants
from base_controllers.utils.math_tools import computeOrientationError
from base_controllers.utils.math_tools import Math

class Ground3D():
    def __init__(self,
                 cohesion=0.0001040 * 1e6,
                 K=0.001,
                 shear_resistance_angle=20/180.*2*np.pi,
                 g=9.81,
                 friction_coefficient=0.1,
                 terrain_stiffness = 1e06, 
                 terrain_damping = 1e03,
                 terrain_torsional_stiffness=1e05,
                 terrain_torsional_damping=1e02):
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
    def __init__(self, dt=0.001, ground=None):
        self.NO_SLIPPAGE = False
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
        self.t_end = 20  # [s]
        self.twist_init = [0, 0.0, 0,0,0,0]  # in WF
        self.pose_init = [0, 0.0, 0.0,0,0,0]  # position in WF, rpy angles
        self.number_of_steps = np.int32(self.t_end / self.dt)
        self.pose_des_log = np.full((6, self.number_of_steps), np.nan)
        self.pose_log = np.full((6, self.number_of_steps), np.nan)
        self.time_log = np.full((self.number_of_steps), np.nan)
        self.math_utils = Math()

    def computeGroundForcesMoments(self,pose, twist, w_R_b, pg, roll, pitch):
        pc = pose[:3]
        pc_d = twist[:3]
        w_omega = twist[3:]
        w_R_terr = self.math_utils.eul2Rot(np.array([roll, pitch, pose[2]]))
        terrain_normal = w_R_terr[:,2]

        #projection of com on the ground level
        pc_ = pose[:3] - w_R_b[:,2]*self.vehicle_param.height

        #force is present only if there is linear penetration
        if (terrain_normal.dot(pg - pc_) > 0.0):
            Fk = self.ground.Kt_x.dot(pg - pc_)
            #only if it is approaching
            if (terrain_normal.dot(pc_d) < 0.0):
                Fd = -self.ground.Dt_x.dot(pc_d)
            else:
                Fd = np.zeros(3)
            # project along terrain normal
            w_Fg =  terrain_normal*terrain_normal.dot(Fk + Fd)
        else:
            w_Fg = np.array([0.0, 0.0, 0.0])

        #moments due to penetration
        #compute projector on tx, ty contact plane
        N = np.eye(3) -  np.multiply.outer(terrain_normal.ravel(), terrain_normal.ravel())
        w_e_o =   computeOrientationError(w_R_b, w_R_terr)
        w_Mg = N.dot(self.ground.Kt_theta.dot(w_e_o) - self.ground.Dt_theta.dot(w_omega))
        #map to base frame
        b_Fg = w_R_b.dot(w_Fg)
        b_Mg = w_R_b.dot(w_Mg)
        return b_Fg, b_Mg

    def dynamics3D(self, pose, twist, w_R_b, b_Fg,b_Mg, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, vehicle_param):
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
        b_Fgrav = w_R_b.T.dot(np.array([0.,0.,-self.ground.g]))
        #compute non linear terms
        NL_lin = m*(np.cross(b_omega,b_vc))
        NL_ang = np.cross(b_omega,  bI.dot(b_omega))

        b_vc_dot =  1/m*(b_Ft + b_Fgrav +b_Fg -NL_lin)
        b_omega_dot = np.linalg.inv(bI).dot(b_Mt+b_Mg-NL_ang)

        w_twist_dot = np.concatenate((w_R_b.dot(b_vc_dot), w_R_b.dot(b_omega_dot)))
        return w_twist_dot

    def integrateTwist(self, pose, twist):
        rpy = pose[3:]
        # TOD replace with RK4
        pose[:3] += twist[:3] * self.dt
        pose[3:] +=np.linalg.inv(self.math_utils.Tomega(rpy)).dot(twist[3:])* self.dt
        return pose

    def initSimulation(self, twist_init=np.zeros(6), pose_init =np.zeros(6)):
        self.twist = twist_init
        self.pose = pose_init

    def simulateOneStep(self,pg, terrain_roll, terrain_pitch, omega_left, omega_right):
        # compute base orientation
        w_R_b = self.math_utils.eul2Rot(self.pose[3:])

        #get states in base frame b_v_c
        b_v_c = w_R_b.T.dot(self.twist[3:])
        state2D = np.array([b_v_c[0],b_v_c[1], self.twist[5]])
        Fx_l, Fy_l, M_long_l, M_lat_l = self.tracked_robot.track_left.computeTerrainInteractions(state2D, omega_left, self.track_param,
                                                                                   self.sigma, self.ground, self.patch_pos_long_l, self.patch_pos_lat_l)
        Fx_r, Fy_r, M_long_r, M_lat_r= self.tracked_robot.track_right.computeTerrainInteractions(state2D, omega_right, self.track_param,
                                                                                   self.sigma, self.ground, self.patch_pos_long_r, self.patch_pos_lat_r)


        # compute ground peneration
        b_Fg, b_Mg = self.computeGroundForcesMoments(self.pose, self.twist, w_R_b, pg, terrain_roll, terrain_pitch)

        if self.NO_SLIPPAGE:
            #extension of unicycle to 3d
            vel = constants.SPROCKET_RADIUS * (omega_left + omega_right) / 2
            omega = constants.SPROCKET_RADIUS / constants.TRACK_WIDTH * (omega_right - omega_left)
            self.twist[:3] = w_R_b.dot(np.array([vel,0,0]))
            self.twist[3:] = w_R_b.dot(np.array([0, 0, omega]))
        else:
            #update twist from dynamics
            self.twist += self.dynamics3D(self.pose, self.twist, w_R_b, b_Fg, b_Mg, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, self.vehicle_param) * self.dt
        self.pose  = self.integrateTwist(self.pose, self.twist)

    def simulate(self, omega_left_vec, omega_right_vec):
        self.time = 0.
        sim_counter = 0
        while self.time < self.t_end:

            terrain_roll = 0.
            terrain_pitch = 0.3
            pg = np.array([self.pose[0], self.pose[1], self.pose[0]*np.tan(terrain_pitch)])

            #updates pose and twist
            self.simulateOneStep(pg, terrain_roll, terrain_pitch,omega_left_vec[sim_counter], omega_right_vec[sim_counter])

            #get des pos
            des_x, des_y, des_theta,_,_,_,_,_ = self.traj.evalTraj(self.time)

            #log
            self.pose_des_log[:3, sim_counter] = np.array([des_x, des_y, des_theta])
            self.pose_log[:, sim_counter] = self.pose
            self.time_log[sim_counter] = self.time

            sim_counter +=1
            self.time = np.round(self.time + self.dt, 3)

    def getRobotState(self):
        return self.pose, self.twist

if __name__ == '__main__':
    groundParams = Ground3D()
    p = TrackedVehicleSimulator3D(dt=0.001, ground=groundParams)

    v = np.linspace(0.4, 0.4, p.number_of_steps)
    omega = np.linspace(0.2, 0.2, p.number_of_steps)
    p.traj = Trajectory(ModelsList.UNICYCLE, 0, 0, 0, DT=p.dt, v=v, omega=omega)

    r = p.track_param.sprocket_radius
    B = p.vehicle_param.width
    omega_left = (v - omega * B / 2.0) / r
    omega_right = (v + omega * B / 2.0) / r

    # init vars
    p.initSimulation(p.pose_init, p.twist_init)

    # simulation with internal while loop
    p.simulate(omega_left, omega_right)


    # assert_almost_equal(p.pose_log[0, -1], -1.02009 , decimal=2)
    # assert_almost_equal(p.pose_log[1, -1], 4.41854 , decimal=2)
    # assert_almost_equal(p.pose_log[2, -1], 3.54506, decimal=2)

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
    fig.suptitle("states", fontsize=20)
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
    plt.subplot(3, 1, 3)
    plt.ylabel("Z")
    plt.plot(p.time_log[:-1], p.pose_log[2, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[2, :-1], linestyle='-', lw=3, color='red')
    plt.xlabel("Time [s]")
    plt.grid()
