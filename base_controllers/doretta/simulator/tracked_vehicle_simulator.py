import numpy as np
from base_controllers.doretta.simulator.tracked_vehicle import TrackedVehicle, VehicleParam
from base_controllers.doretta.simulator.track import TrackParams


class Ground:
    def __init__(self):
        self.cohesion = 0.0001040 * 1e6  # [Pa]
        # self.K = 0.025  # [m]
        self.K = 0.001  # [m]
        self.shear_resistance_angle = 20/180.*2*np.pi # [rad]
        self.mu_t = 0.65 
        self.g = 9.81 
        self.fr = 0.6 
        self.friction_coefficient = 0.1
        
class SimParam:
    def __init__(self):
        self.alg_delay = 0.01  # [s]
        self.t_end = 20  # [s]
        self.dt = 0.01 
        self.decimation_factor = 1 
        self.tau_sprockets = 1e-1 
        self.tau_forces = 0e-2 
        self.state_init = [0, 0.0, 0]  # [u, v, Omega]
        self.pose_init = [0, 0.05, 0.1]  # [x, y, theta]

class CtrlParam:
    def __init__(self):
        self.Kp = 5 
        self.K_theta = 0.1 
        self.t_end = 20  # [s]    trajectory end
        self.lin_vel_coeff = 0.4 
        self.ang_vel_coeff = 0.2 
        self.traj_init_pose = [0.0, 0.0, 0.0] 
        self.type = 1  # 0  openloop 1        Lyapunov

        
class TrackedVehicleSimulator:
    def __init__(self):
        self.vehicle_param = VehicleParam()
        self.track_param = TrackParams()
        self.sim_param = SimParam()
        self.ground = Ground()
        self.sigma = np.ones((self.track_param.parts_longitudinal, self.track_param.parts_lateral)) * self.vehicle_param.weight / (2 * self.track_param.A)
        self.tracked_robot = TrackedVehicle(self.vehicle_param, self.track_param,  self)
        self.patch_pos_long_l, self.patch_pos_lat_l = self.tracked_robot.getLeftPatchesPositions()
        self.patch_pos_long_r, self.patch_pos_lat_r = self.tracked_robot.getRightPatchesPositions()


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
       
    def integrateBodyVelocity(self, pose, velocities_body):
        # the world fixed reference frame
        velocities_world = np.zeros(3)
        theta = pose[2]
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        velocities_world[:2] = R.dot(velocities_body[:2])
        velocities_world[2] = velocities_body[2]
        pose += velocities_world * self.sim_param.dt
        return pose

    def simulate(self, omega_left, omega_right):
        # init vars
        self.state = self.sim_param.state_init
        self.pose = self.sim_param.pose_init
        self.time = 0.
        sim_counter = 0
        while self.time < self.sim_param.t_end:
            Fx_l, Fy_l, M_long_l, M_lat_l = self.tracked_robot.track_left.computeTerrainInteractions(self.state, omega_left[sim_counter], self.track_param,
                                                                                       self.sigma, self.ground, self.patch_pos_long_l, self.patch_pos_lat_l)
            Fx_r, Fy_r, M_long_r, M_lat_r= self.tracked_robot.track_right.computeTerrainInteractions(self.state, omega_right[sim_counter], self.track_param,
                                                                                       self.sigma, self.ground, self.patch_pos_long_r, self.patch_pos_lat_r)
            self.state += self.dynamics(self.state, Fx_l, Fy_l, M_long_l, M_lat_l, Fx_r, Fy_r, M_long_r, M_lat_r, self.vehicle_param) * self.sim_param.dt
            self.pose = self.integrateBodyVelocity(self.pose, self.state)
            #todo log states
            sim_counter +=1
            self.time += self.sim_param.dt
        return self.pose_log

if __name__ == '__main__':
    p = TrackedVehicleSimulator()
    number_of_steps = np.int(p.sim_param.t_end/p.sim_param.dt)
    v = np.linspace(0.2, 0.2, number_of_steps)
    omega = np.linspace(0.2, 0.2, number_of_steps)
    r = p.track_param.sprocket_radius
    B = p.vehicle_param.width
    omega_left = (v - omega * B / 2.0) / r
    omega_right = (v + omega * B / 2.0) / r
    p.simulate(omega_left, omega_right)