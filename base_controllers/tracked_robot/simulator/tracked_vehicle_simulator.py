import numpy as np
from base_controllers.tracked_robot.simulator.tracked_vehicle import TrackedVehicle, VehicleParam
from base_controllers.tracked_robot.simulator.track import TrackParams
from numpy.testing import assert_almost_equal
from matplotlib import pyplot as plt
from  base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
import base_controllers.tracked_robot.utils.constants as constants
from termcolor import colored

class Ground():
    def __init__(self,
                 cohesion=0.0001040 * 1e6,
                 K=0.001,
                 shear_resistance_angle=20/180.*2*np.pi,
                 mu_t=0.65,
                 g=9.81,
                 fr=0.6,
                 friction_coefficient=0.1):
        self.cohesion = cohesion  # [Pa] #not used
        self.K = K  # [m]
        self.shear_resistance_angle = shear_resistance_angle  # [rad] #not used
        self.mu_t = mu_t #not used
        self.g = g
        self.fr = fr #not used
        self.friction_coefficient = friction_coefficient
        

class TrackedVehicleSimulator:
    def __init__(self, dt=0.001, ground=None):
        self.NO_SLIPPAGE = False
        self.dt = dt
        self.vehicle_param = VehicleParam()
        self.track_param = TrackParams()
        if ground is not None:
            self.ground = ground
        else:
            self.ground = Ground()
        self.sigma = np.ones((self.track_param.parts_longitudinal, self.track_param.parts_lateral)) * self.vehicle_param.weight / (2 * self.track_param.A)
        self.tracked_robot = TrackedVehicle(self.vehicle_param, self.track_param,  self)
        self.patch_pos_long_l, self.patch_pos_lat_l = self.tracked_robot.getLeftPatchesPositions()
        self.patch_pos_long_r, self.patch_pos_lat_r = self.tracked_robot.getRightPatchesPositions()

        #these are just for the unit test
        self.t_end = 20  # [s]
        self.state_init = [0, 0.0, 0]  # [u, v, Omega]
        self.pose_init = [0, 0.05, 0.1]  # [x, y, theta]
        self.number_of_steps = np.int32(self.t_end / self.dt)
        self.pose_des_log = np.full((3, self.number_of_steps), np.nan)
        self.pose_log = np.full((3, self.number_of_steps), np.nan)
        self.time_log = np.full((self.number_of_steps), np.nan)

    def setGround(self, ground):
        self.ground = ground

    def dynamics(self, state, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, vehicle_param):
       # calculate the acceleration with the vehicle dynamical model
        m   = vehicle_param.mass
        Izz = vehicle_param.Izz
        u = state[0]
        v = state[1]
        Omega = state[2]

        u_dot = (Fx_l + Fx_r) / m + Omega * v
        v_dot = (Fy_l + Fy_r) / m - Omega * u

        Omega_dot = (M_long_l + M_lat_l + M_long_r + M_lat_r) / Izz
        v_body_dot = np.array([u_dot, v_dot, Omega_dot])
        return v_body_dot
       
    def integrateBodyVelocity(self, pose, velocities_body):
        # the world fixed reference frame
        velocities_world = np.zeros(3)
        theta = pose[2]
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        velocities_world[:2] = R.dot(velocities_body[:2])
        velocities_world[2] = velocities_body[2]
        #TOD replace with RK4
        pose += velocities_world * self.dt
        pose_der = velocities_world
        return pose, pose_der

    def initSimulation(self, vbody_init, pose_init):
        self.state = vbody_init
        self.pose = pose_init
        self.pose_der = np.zeros(3)
    
    def simulateOneStep(self, omega_left, omega_right):
        Fx_l, Fy_l, M_long_l, M_lat_l = self.tracked_robot.track_left.computeTerrainInteractions(self.state, omega_left, self.track_param,
                                                                                   self.sigma, self.ground, self.patch_pos_long_l, self.patch_pos_lat_l)
        Fx_r, Fy_r, M_long_r, M_lat_r= self.tracked_robot.track_right.computeTerrainInteractions(self.state, omega_right, self.track_param,
                                                                                   self.sigma, self.ground, self.patch_pos_long_r, self.patch_pos_lat_r)



        self.state += self.dynamics(self.state, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, self.vehicle_param) * self.dt



        if self.NO_SLIPPAGE:
            vel = constants.SPROCKET_RADIUS * (omega_left + omega_right) / 2
            omega = constants.SPROCKET_RADIUS / constants.TRACK_WIDTH * (omega_right - omega_left)
            self.pose_der[0] = vel * np.cos(self.pose[2])
            self.pose_der[1] = vel * np.sin(self.pose[2])
            self.pose_der[2] = omega
            self.pose += self.pose_der * self.dt
        else:
            self.pose, self.pose_der = self.integrateBodyVelocity(self.pose, self.state)

    def simulate(self, omega_left_vec, omega_right_vec):
        self.time = 0.
        sim_counter = 0
        while self.time < self.t_end:
            #print(colored(f"time {p.time}","red"))
            self.simulateOneStep(omega_left_vec[sim_counter], omega_right_vec[sim_counter])
            pose, pose_der = self.getRobotState()
            #get des pos
            des_x, des_y, des_theta,_,_,_,_,_ = self.traj.evalTraj(self.time)

            #log
            self.pose_des_log[:, sim_counter] = np.array([des_x, des_y, des_theta])
            self.pose_log[:, sim_counter] = self.pose
            self.time_log[sim_counter] = self.time


            sim_counter +=1
            self.time = np.round(self.time + self.dt, 3)

    def getRobotState(self):
        return self.pose, self.pose_der

    def unit_test_track(self):
        omega_left = 0.
        state = np.array([1.4901e-14,0,0])
        Fx_l, Fy_l, M_long_l, M_lat_l = self.tracked_robot.track_left.computeTerrainInteractions(state, omega_left, self.track_param,
                                                                                                 self.sigma,
                                                                                                 self.ground,
                                                                                                 self.patch_pos_long_l,
                                                                                                 self.patch_pos_lat_l)



        assert_almost_equal(Fx_l,  -29.945025)
        assert_almost_equal(Fy_l, 0)
        assert_almost_equal(M_long_l,9.073342575)
        assert_almost_equal(M_lat_l, 0)

        omega_left =     3.9595
        state = np.array([   0.39478,   -0.0016373,      0.17262])
        Fx_l, Fy_l, M_long_l, M_lat_l = self.tracked_robot.track_left.computeTerrainInteractions(state, omega_left, self.track_param,
                                                                                                 self.sigma,
                                                                                                 self.ground,
                                                                                                 self.patch_pos_long_l,
                                                                                                 self.patch_pos_lat_l)

        # print("track")
        # print(Fx_l)
        # print(Fy_l)
        # print(M_long_l)
        # print(M_lat_l)
        assert_almost_equal(Fx_l,  -5.427720569302706, decimal = 2)
        assert_almost_equal(Fy_l, 1.9137256084209593, decimal = 2)
        assert_almost_equal(M_long_l,1.4172282686978552, decimal = 2)
        assert_almost_equal(M_lat_l,   -3.0884029613580166, decimal = 2)
        #test dynamics
        state = np.array([   0.6549,   0.00025322,     0.036401])
        Fx_l=-15.477331 
        Fy_l=2.620788
        M_long_l=4.587358
        M_lat_l=-1.786085
        Fx_r=28.655311
        Fy_r=-0.013475
        M_long_r=8.596293
        M_lat_r=-0.258470
        vbody_dot  = p.dynamics(state, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, self.vehicle_param)

        assert_almost_equal(vbody_dot[0], 0.21964 , decimal=2)
        assert_almost_equal(vbody_dot[1],    0.019616  , decimal=2)
        assert_almost_equal(vbody_dot[2],       2.4754, decimal=2)

if __name__ == '__main__':
    groundParams = Ground()
    p = TrackedVehicleSimulator(dt=0.001, ground=groundParams)

    p.unit_test_track()

    v = np.linspace(0.4, 0.4, p.number_of_steps)
    omega = np.linspace(0.2, 0.2, p.number_of_steps)
    p.traj = Trajectory(ModelsList.UNICYCLE, 0, 0, 0, DT=p.dt, v=v, omega=omega)

    r = p.track_param.sprocket_radius
    B = p.vehicle_param.width
    omega_left = (v - omega * B / 2.0) / r
    omega_right = (v + omega * B / 2.0) / r

    # init vars
    p.initSimulation(p.state_init, p.pose_init)

    # simulation with internal while loop
    p.simulate(omega_left, omega_right)


    assert_almost_equal(p.pose_log[0, -1], -1.02009 , decimal=2)
    assert_almost_equal(p.pose_log[1, -1], 4.41854 , decimal=2)
    assert_almost_equal(p.pose_log[2, -1], 3.54506, decimal=2)

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
    plt.ylabel("theta")
    plt.plot(p.time_log[:-1], p.pose_log[2, :-1], linestyle='-',  lw=3, color='blue')
    plt.plot(p.time_log[:-1], p.pose_des_log[2, :-1], linestyle='-', lw=3, color='red')
    plt.xlabel("Time [s]")
    plt.grid()
