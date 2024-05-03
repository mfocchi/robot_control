from datetime import datetime
import os
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from base_controllers.doretta.utils import constants as constants
from base_controllers.doretta.environment.trajectory import Trajectory, ModelsList
from base_controllers.doretta.models.unicycle import Unicycle
from base_controllers.doretta.controllers.lyapunov import LyapunovController, LyapunovParams
from base_controllers.doretta.controllers.lyapunov_zou import LyapunovControllerZou, LyapunovParamsZou
from base_controllers.doretta.controllers.stanley import StanleyController, StanleyParams

import base_controllers.doretta.velocity_generator as vt
import sys
print(sys.setrecursionlimit(3000))
save = False

SIMULATION = True
simulation_max = 100.0

def main():

    type_of_controller='luigi'
    #type_of_controller='stanley'
    #type_of_controller = 'zou' # much more aggressive on omega
    initial_des_x = 4.
    initial_des_y = 4.
    initial_des_theta = 0.5

    initial_x = 0.
    initial_y = 0.
    initial_theta = -1.

    #if difference between initial_des_theta and initial_theta > 1.57 you can have issues (a little glitch in direction at the beginning )

    vel_gen = vt.velocity_luigi
    _, _, s_test = vel_gen()


    robot = Unicycle(x=initial_x, y=initial_y, theta=initial_theta)

    traj = Trajectory(ModelsList.UNICYCLE, initial_des_x, initial_des_y, initial_des_theta, vel_gen)
    start_time = 0.0

    if type_of_controller=='zou':
        K_P = 10.0
        K_THETA = 10.0
        params = LyapunovParamsZou(K_P=K_P, K_THETA=K_THETA)
        controller = LyapunovControllerZou(params=params)
        controller.config(start_time=start_time, trajectory=traj)

    if type_of_controller=='luigi':
        K_P = 10.0
        K_THETA = 10.0
        params = LyapunovParams(K_P=K_P, K_THETA=K_THETA)
        controller = LyapunovController(params=params)
        controller.config(start_time=start_time, trajectory=traj)

    if type_of_controller=='stanley':
        K_THETA = 1.0
        K_E = 6.0
        EPSILON_V = 0.01
        K_DELTA = 12
        params = StanleyParams(K_THETA=K_THETA, K_E=K_E, EPSILON_V=EPSILON_V, K_DELTA=K_DELTA)
        controller = StanleyController(path=traj, params=params)




    # plot results
    time_ = [start_time]
    draw_x = [robot.x]
    draw_y = [robot.y]
    draw_theta = [robot.theta]
    draw_control_vel = [robot.v]
    draw_control_omega = [robot.omega]
    draw_V = [0.]
    draw_V_dot = [0.]
    current_time = constants.DT

    while not controller.goal_reached:  # if target is not reached yet
        # controllers
        if  type_of_controller=='stanley':
            o, index = controller.control(robot=robot)
            v = controller.longitudinal_velocity_controller(index)
        if type_of_controller == 'luigi':
            v, o, _, _, V, V_dot = controller.control(robot, current_time)
        if type_of_controller == 'zou':
            v, o = controller.control(robot, current_time)

        # SAFE CHECK -> clipping velocities
        # v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        # o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)

        # send / update velocity
        robot.update(v, o)
        state = robot.state()
        draw_x.append(state[0])
        draw_y.append(state[1])
        draw_theta.append(state[2])

        draw_control_vel.append(v)
        draw_control_omega.append(o)
        draw_V.append(V)
        draw_V_dot.append(V_dot)

        time_.append(current_time )

        current_time += constants.DT
        if current_time > simulation_max:
            print("MAXIMUM TIME ALLOWED REACHED")
            break

    print("FINISH!!!")
    # plot results and spinning forever

    home_path = str(Path.home())
    my_path = home_path + "/ros2_ws/src/doretta"

    # DEBUG
    # print("Time: " + str(self.draw_time))
    # print("Omega: " + str(self.draw_control_omega))

    # PATHs XY
    plt.figure(1)
    # plt.rcParams["figure.autolayout"] = True
    # plt.text(100, 100, "K_P: %.2f  K_THETA:%.2f" % (K_P, K_THETA),fontsize = 16)
    plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
    plt.plot(traj.x, traj.y, "-r", label="desired")
    plt.plot(draw_x, draw_y, "-b", label="real")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)
    # if save:
    #     plt.savefig(os.path.join(my_path, 'test_results/SIM_LYAPUNOV/SIM_LYAP_%s_plot_path_%s.png' % (s_test, dt_string)))

    # # States
    plt.figure(2)
    plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
    plt.subplot(3, 1, 1)
    plt.plot(time_, draw_x, "-b", label="REAL")
    plt.plot(time_[:-1], traj.x, "-r", label="desired")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("X")
    # plt.axis("equal")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(time_, draw_y, "-b", label="REAL")
    plt.plot(time_[:-1], traj.y, "-r", label="desired")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("Y")
    # plt.axis("equal")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(time_, draw_theta, "-b", label="REAL")
    plt.plot(time_[:-1], traj.theta, "-r", label="desired")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("theta")
    # plt.axis("equal")
    plt.grid(True)

    # # VELOCITY
    plt.figure(3)
    plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.plot(time_, draw_control_vel, "-b", label="REAL")
    plt.plot(time_[:-1], traj.v, "-r", label="desired")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("linear velocity[m/s]")
    # plt.axis("equal")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.cla()
    plt.plot(time_, draw_control_omega, "-b", label="REAL")
    plt.plot(time_[:-1], traj.omega, "-r", label="desired")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("angular velocity[rad/s]")
    # plt.axis("equal")
    plt.grid(True)

    # # liapunov
    plt.figure(4)
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.plot(time_, draw_V, "-b", label="REAL")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("V liapunov")
    # plt.axis("equal")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.cla()
    plt.plot(time_, draw_V_dot, "-b", label="REAL")

    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("Vdot liapunov")
    # plt.axis("equal")
    plt.grid(True)

    # if save:
    #     plt.savefig(os.path.join(my_path, 'test_results/SIM_LYAPUNOV/SIM_LYAP_%s_plot_vel_%s.png' % (s_test, dt_string)))




    if not save:
        plt.show()


if __name__ == "__main__":
    main()
