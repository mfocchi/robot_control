from datetime import datetime
import os
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from doretta.utils import constants as constants
from doretta.environment.trajectory import Trajectory, ModelsList
from doretta.models.unicycle import Unicycle
from doretta.controllers.lyapunov import LyapunovController, LyapunovParams

import doretta.velocity_tests as vt

save = False

SIMULATION = True
simulation_max = 100.0

# Lyapunov controller parameters
K_P = 6.0
K_THETA = 6.0

def main():


    initial_x = 0.0
    initial_y = 1.5
    initial_theta = np.radians(180)
    vel_gen = vt.velocity_test0
    _, _, s_test = vel_gen()

    robot = Unicycle(x=initial_x, y=initial_y, theta=initial_theta)

    traj = Trajectory(ModelsList.UNICYCLE, initial_x, initial_y, initial_theta, vel_gen)

    params = LyapunovParams(K_P=K_P, K_THETA=K_THETA)
    controller = LyapunovController(params=params)
    start_time = 0.0
    controller.config(start_x=robot.x, start_y=robot.y, start_theta=robot.theta, start_time=start_time, velocity_generator=vel_gen)

    # plot results
    time_ = [start_time]
    draw_x = [robot.x]
    draw_y = [robot.y]
    draw_theta = [robot.theta]
    draw_vel = [robot.v]
    draw_omega = [robot.omega]

    current_time = constants.DT

    while not controller.goal_reached:  # if target is not reached yet
        # controllers
        v, o = controller.control(robot, current_time)

        # SAFE CHECK -> clipping velocities
        v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)

        # send / update velocity
        robot.update(v, o)
        state = robot.state()
        draw_x.append(state[0])
        draw_y.append(state[1])
        draw_theta.append(state[2])
        draw_vel.append(state[3])
        draw_omega.append(state[4])
        time_.append(current_time - time_[0])

        current_time += constants.DT
        if current_time > simulation_max:
            print("MAXIMUM TIME ALLOWED REACHED")
            break

    time_[0] = 0  # istante di inizio viene aggiornato a 0.0

    # plot results and spinning forever
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%H:%M:%S")

    home_path = str(Path.home())
    my_path = home_path + "/ros2_ws/src/doretta"

    # DEBUG
    # print("Time: " + str(self.draw_time))
    # print("Omega: " + str(self.draw_control_omega))

    # PATHs
    plt.subplots(1)
    # plt.rcParams["figure.autolayout"] = True
    # plt.text(100, 100, "K_P: %.2f  K_THETA:%.2f" % (K_P, K_THETA),fontsize = 16)
    plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
    plt.cla()
    plt.plot(traj.x, traj.y, "-r", label="desired")
    plt.plot(draw_x, draw_y, "-b", label="real")
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)
    if save:
        plt.savefig(os.path.join(my_path, 'test_results/SIM_LYAPUNOV/SIM_LYAP_%s_plot_path_%s.png' % (s_test, dt_string)))

    # # VELOCITY
    plt.subplots(1)
    plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.plot(time_, draw_vel, "-r", label="REAL")
    # plt.gcf().canvas.mpl_connect('key_release_event',
    #                             lambda event: [exit(0) if event.key == 'escape' else None])
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("linear velocity[m/s]")
    # plt.axis("equal")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.cla()
    plt.plot(time_, draw_omega, "-r", label="REAL")
    # plt.plot(self.draw_time, self.draw_control_omega, "-b", label="CONTROLLER")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("angular velocity[rad/s]")
    # plt.axis("equal")
    plt.grid(True)

    if save:
        plt.savefig(os.path.join(my_path, 'test_results/SIM_LYAPUNOV/SIM_LYAP_%s_plot_vel_%s.png' % (s_test, dt_string)))

    # # ERRORS
    plt.subplots(1)
    plt.suptitle("K_P: %.2f  K_THETA: %.2f" % (K_P, K_THETA))
    plt.subplot(3, 1, 1)
    plt.cla()
    plt.plot(time_, controller.draw_e_x, "-r", label="e_x")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("e_x [m]")
    # plt.axis("equal")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.cla()
    plt.plot(time_, controller.draw_e_y, "-r", label="e_y")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("e_y [m]")
    # plt.axis("equal")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.cla()
    plt.plot(time_, controller.draw_e_theta, "-r", label="e_theta")
    plt.legend()
    plt.xlabel("time[sec]")
    plt.ylabel("e_theta [rad]")
    # plt.axis("equal")
    plt.grid(True)

    if save:
        plt.savefig(os.path.join(my_path, 'test_results/SIM_LYAPUNOV/SIM_LYAP_%s_plot_err_%s.png' % (s_test, dt_string)))

    print("FINISH!!!")

    if not save:
        plt.show()


if __name__ == "__main__":
    main()
