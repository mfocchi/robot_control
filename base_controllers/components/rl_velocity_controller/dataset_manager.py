import numpy as np
import time
import os
from tqdm import tqdm
from termcolor import colored

class DatasetManager():
    def __init__(self, quadruped=None):

        self.quadruped = quadruped
        # -------------------------------
        # Simulation Thresholds and Constants
        # -------------------------------
        self.INCLINATION_THRESHOLD = 30.0  # degrees - max allowed inclination before considering robot as fallen
        self.FALL_HEIGHT_THRESHOLD = 0.2   # meters - min allowed height before considering robot as fallen
        self.CP_SAFE_RADIUS = 0.02         # meters - acceptable radius to consider CP successful
        self.G = 9.81                      # gravitational constant
        self.policy_frequency = 50 #Hz
        self.decimation = (1 / self.quadruped.dt) * (1 / self.policy_frequency)
    # -------------------------------
    # Capture Point Computation
    # -------------------------------
    def compute_capture_point(self, pos, vel, height):
        tc = np.sqrt(height / self.G)  # time constant based on height
        return pos + vel * tc

    # -------------------------------
    # Fall Condition Checker
    # -------------------------------
    def check_fallen(self, base_z_position, inclination_deg):
        return inclination_deg > self.INCLINATION_THRESHOLD or base_z_position < self.FALL_HEIGHT_THRESHOLD

    def check_termination(self):
        # -------------------------------
        # Check Falling
        # -------------------------------
        inclination_after = 2 * np.arcsin(np.sqrt(self.quadruped.quaternion[0] ** 2 + self.quadruped.quaternion[1] ** 2)) * (180 / np.pi)
        if  self.check_fallen(self.quadruped.basePoseW[2], inclination_after):
            self.fallen_flag = 1

        # -------------------------------
        # Check com inside safe radius of capture point
        # -------------------------------
        base_pos_xy = self.quadruped.basePoseW[:2].copy()
        base_vel_xy = self.quadruped.baseTwistW[:2].copy()
        z_after = self.quadruped.basePoseW[2].copy()
        cp = self.compute_capture_point(base_pos_xy, base_vel_xy, z_after)
        cp_local = cp - base_pos_xy

        #when self.capture_flag = 1 it means we are stable
        if np.linalg.norm(cp_local) < self.CP_SAFE_RADIUS and self.quadruped.time >= self.warmup_time and self.fallen_flag == 0:
            self.capture_flag = 1

        self.quadruped.ros_pub.add_marker(np.append(base_pos_xy, 0.), radius=0.1, color="blue")
        self.quadruped.ros_pub.add_marker(np.append(cp, 0.), radius=0.1, color="red")

        # -------------------------------
        # Save Observation Transition for VF Learning
        # -------------------------------
        body_ang_vel = self.quadruped.b_R_w @ self.quadruped.baseTwistW[3:]
        proj_gravity = self.quadruped.b_R_w.dot(np.array([0, 0, -1]))
        #dimension of observation vector = 3+3+12+12=30
        self.obs_tp1_ = np.concatenate((
            proj_gravity.astype(np.float32),
            body_ang_vel.astype(np.float32),
            self.quadruped.q.astype(np.float32),
            self.quadruped.qd.astype(np.float32)
        ))
        # dimension of full_obs vector = 30+30+1+1=62
        full_obs = np.concatenate([self.obs_t_, self.obs_tp1_, [float(self.fallen_flag), float(self.capture_flag)]])
        #collect data only for backup policy which is active after warmup
        if self.step % self.decimation == 0:
            self.observations.append(full_obs)

        #store last observation
        self.obs_t_ = self.obs_tp1_

        return self.capture_flag or self.fallen_flag

    # -------------------------------
    # Main Function: Single Simulation Episode
    # -------------------------------
    def run_single_simulation(self, actor_network,  max_steps=2000, noise_std=1.0, warmup_time=1.0):
        #init vars
        self.fallen_flag = 0
        self.capture_flag = 0
        self.first_time = True
        self.observations = []
        self.obs_t_ = []
        self.obs_tp1_ = []

        #reset robot
        self.warmup_time = warmup_time
        actor_network.velocity_cmd = np.array([
            np.random.uniform(-0.5, 1.0),  # vx
            np.random.uniform(-0.3, 0.3),  # vy
            np.random.uniform(-0.7, 0.7)  # yaw_rate
        ])
        #debug
        #actor_network.velocity_cmd = np.array([0.5, 0.0, 0.0])
        print(f"Nominal policy velocity command {actor_network.velocity_cmd}")
        #generate push instant
        low = warmup_time - 0.5
        high = warmup_time
        n_steps = int((high - low) / self.quadruped.dt)
        push_instant = round(low + np.random.randint(0, n_steps + 1) * self.quadruped.dt,3)
        for self.step in range(max_steps):
            self.quadruped.updateKinematics()
            if not self.first_time and self.quadruped.time > warmup_time:
                # Stop if CP was reached successfully
                if self.check_termination():
                    print(f"Termination at {self.quadruped.time}")
                    break
            else:
                self.first_time = False

            # -------------------------------
            # Observation and Action Computation
            # -------------------------------
            # Prepare observation
            lin_vel_b = self.quadruped.b_R_w.dot(self.quadruped.baseTwistW[:3])
            ang_vel_b = self.quadruped.b_R_w.dot(self.quadruped.baseTwistW[3:6])
            proj_gravity = self.quadruped.b_R_w.dot(np.array([0, 0, -1]))

            # Inference (get new action)
            # Generate random initial command (velocity and yaw) to explore nominal policy states
            if self.quadruped.time <= warmup_time: #nominal policy
                if np.mod(self.quadruped.time, 0.5) == 0:
                    print(colored(f"TIME: {self.quadruped.time}", "blue"))

                # Apply external push randomly before warmup ends
                if self.quadruped.time == push_instant:
                    #apply as a wrench
                    # Fx = np.random.uniform(-200.0, 300.0)
                    # Fy = np.random.uniform(-150, 150)
                    # self.quadruped.applyForce(Fx,0,0,0,0,0,self.quadruped.dt)  # push in x
                    # self.quadruped.applyForce(0, Fy, 0, 0, 0, 0, self.quadruped.dt)  # push in y
                    #apply as a twisch change
                    vx= np.random.uniform(-2.0, 3.0)
                    vy = np.random.uniform(-1.5, 1.5)
                    #debug makes it fall
                    # vx = -1.645
                    # vy = -1.239
                    print(f"Pushing robot at {push_instant} with twist  vx: {vx}, vy: {vy}")
                    self.quadruped.setBaseTwist(baseTwist=np.array([vx, vy, 0, 0, 0, 0]))
                self.quadruped.q_des = actor_network.action(lin_vel_b, ang_vel_b, proj_gravity, self.quadruped.q, self.quadruped.qd, policy_type="default")
            else:#switch to backup policy
                if np.mod(self.quadruped.time, 0.5) == 0:
                    print(colored(f"TIME: {self.quadruped.time}", "red"))
                actor_network.velocity_cmd = np.array([0.0, 0.0, 0.0])
                self.quadruped.q_des = actor_network.action(lin_vel_b, ang_vel_b, proj_gravity, self.quadruped.q, self.quadruped.qd, policy_type="safe")

            #for log
            self.quadruped.baseTwistW_des[:3] = self.quadruped.b_R_w.T @ np.append(actor_network.velocity_cmd[:2], 0.0)
            self.quadruped.baseTwistW_des[5] = actor_network.velocity_cmd[2]

            # Add noise to simulate real-world actuation
            self.quadruped.tau_ffwd = np.zeros(12)
            #TODO
            if self.quadruped.time >= self.warmup_time:
                noise = np.random.normal(0, noise_std, size=self.quadruped.tau_ffwd.shape)
                self.quadruped.tau_ffwd += noise
            # switch off wbc
            self.quadruped.grForcesW_des = np.zeros((12))

            #send to PD controller
            self.quadruped.send_des_jstate(self.quadruped.q_des, np.zeros(12), self.quadruped.tau_ffwd)
            self.quadruped.rate.sleep()
            self.quadruped.time = np.round(self.quadruped.time + self.quadruped.dt, 4)  # np.array([self.loop_time]), 3)
            self.quadruped.ros_pub.publishVisual(delete_markers=True)

        return np.array(self.observations), self.fallen_flag, self.capture_flag

    # -------------------------------
    # Run a Batch of Simulations and Save Results
    # -------------------------------
    def run_batch_simulations(self, actor_network, n_episodes=100, save_path="results", noise_std=1.0, seed=None):
        os.makedirs(save_path, exist_ok=True)
        all_obs = []
        stats = []
        max_len = 0

        if seed is not None:
            np.random.seed(seed)

        print("Running batch simulations...")

        #for i in tqdm(range(n_episodes)):
        for i in (range(n_episodes)):
            print(colored(f"Simulation {i}-----------------------", "blue"))
            # -------------------------------
            # Reset base
            # -------------------------------
            self.quadruped.initVars()  # resets time etc
            self.quadruped.q_des = self.quadruped.qj_0.copy()
            self.quadruped.resetRobot(basePoseDes=np.array([0, 0, 0.36, 0., 0., 0.]))

            obs, fallen, captured = self.run_single_simulation(actor_network=actor_network, noise_std=noise_std)
            print(colored(f"Fallen {fallen}, Captured {captured}", "green"))
            all_obs.append(obs)
            
            stats.append((fallen, captured, len(obs)))
            max_len = max(max_len, len(obs))

        # Pad observation arrays to same length in case of early termination (com comverget to cop)
        obs_dim = all_obs[0].shape[1]
        padded_obs = np.zeros((n_episodes, max_len, obs_dim), dtype=np.float32)

        for i, episode in enumerate(all_obs):
            padded_obs[i, :len(episode), :] = episode

        stats = np.array(stats, dtype=int)

        np.save(os.path.join(save_path, "observations_dataset.npy"), padded_obs)

        print(f"Episodi completati: {n_episodes}")
        print(f"Caduti: {np.sum(stats[:, 0])}, CP raggiunto: {np.sum(stats[:, 1])}")
        print(f"Dati salvati in: {save_path}")
        print(f"Shape of observations: {padded_obs.shape}")




