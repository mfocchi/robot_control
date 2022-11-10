import numpy as np
import pinocchio as pin

class IMU_utils:
    def __init__(self, timeout = 1000, dt = 0.002):
        self.counter = 0.
        self.timeout = timeout
        self.dt = dt
        self.IMU_accelerometer_bias = np.zeros(3)
        self.g0 = np.array([0., 0., 9.81])

        # filters data
        self.alpha_accelerometer = 0.95
        self.alpha_velocity = [0.07, 0.07, 0.05] # X Y should be equal
        self.W_lin_vel = np.zeros(3)


    def IMU_bias_estimation(self, b_R_w, IMU_accelerometer):
            self.IMU_accelerometer_bias = self.alpha_accelerometer * self.IMU_accelerometer_bias + \
                    (1 - self.alpha_accelerometer) * (IMU_accelerometer - b_R_w @ self.g0)
            self.counter += 1


    def compute_lin_vel(self, W_lin_acc, loop_dt):
        self.W_lin_vel[0] = (1 - self.alpha_velocity[0] * loop_dt) * self.W_lin_vel[0] + loop_dt * W_lin_acc[0]
        self.W_lin_vel[1] = (1 - self.alpha_velocity[1] * loop_dt) * self.W_lin_vel[1] + loop_dt * W_lin_acc[1]
        self.W_lin_vel[2] = (1 - self.alpha_velocity[2] * loop_dt) * self.W_lin_vel[2] + loop_dt * W_lin_acc[2]








