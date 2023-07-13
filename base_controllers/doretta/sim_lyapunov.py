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

import base_controllers.doretta.velocity_tests as vt
import sys
print(sys.setrecursionlimit(3000))
save = False

SIMULATION = True
simulation_max = 100.0

# Lyapunov controller parameters
K_P = 10.0
K_THETA = 10.0




def main():

    type_of_controller='luigi'
    #type_of_controller = 'zou' # much more aggressive on omega
    initial_des_x = 2.
    initial_des_y = 2.
    initial_des_theta = 0.5

    initial_x = 0.
    initial_y = 0.
    initial_theta = -0.1

    vel_gen = vt.velocity_luigi
    _, _, s_test = vel_gen()


    robot = Unicycle(x=initial_x, y=initial_y, theta=initial_theta)

    traj = Trajectory(ModelsList.UNICYCLE, initial_des_x, initial_des_y, initial_des_theta, vel_gen)

    if type_of_controller=='zou':
        params = LyapunovParamsZou(K_P=K_P, K_THETA=K_THETA)
        controller = LyapunovControllerZou(params=params)

    if type_of_controller=='luigi':
        params = LyapunovParams(K_P=K_P, K_THETA=K_THETA)
        controller = LyapunovController(params=params)

    start_time = 0.0
    controller.config(start_time=start_time, trajectory=traj)

    # plot results
    time_ = [start_time]
    draw_x = [robot.x]
    draw_y = [robot.y]
    draw_theta = [robot.theta]
    draw_control_vel = [robot.v]
    draw_control_omega = [robot.omega]

    current_time = constants.DT

    while not controller.goal_reached:  # if target is not reached yet
        # controllers
        v, o,_,_,_,_ = controller.control(robot, current_time)

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

    # if save:
    #     plt.savefig(os.path.join(my_path, 'test_results/SIM_LYAPUNOV/SIM_LYAP_%s_plot_vel_%s.png' % (s_test, dt_string)))




    if not save:
        plt.show()


if __name__ == "__main__":
    main()
