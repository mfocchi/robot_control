import numpy as np
from termcolor import colored
from tracked_vehicle_simulator3d import TrackedVehicleSimulator3D
from tracked_vehicle_simulator3d import Ground3D
from matplotlib import pyplot as plt #need to import as last to avoid issues with qt
np.set_printoptions(threshold=np.inf, precision = 9, linewidth = 10000, suppress = True)
from base_controllers.utils.math_tools import forward_euler_step, backward_euler_step, RK4_step, heun_step


def volterra(y):
    a=4
    c=1
    y_next = np.zeros(2)
    y_next[0] = y[0] * (a - y[1])
    y_next[1] = y[1] * (c*y[0] - y[1])
    return y_next

if __name__ == '__main__':
    dt = 0.001
    t_end = 10.
    pose_init = np.array([2., 1.])
    number_of_steps = np.int32(t_end / dt)

    time = 0.
    sim_counter = 0
    pose_log = np.full((2, number_of_steps), np.nan)
    time_log = np.full((number_of_steps), np.nan)
    x = pose_init
    while time < t_end:
        if np.mod(time, 1) == 0:
            print(colored(f"TIME: {time}", "red"))
        x = heun_step(volterra, y = x,  h=dt)
        time = np.round(time + dt, 4)
        time_log[sim_counter] = time
        pose_log[:,sim_counter] = x
        sim_counter += 1

    fig = plt.figure()
    fig.suptitle("lin states", fontsize=20)
    plt.subplot(2, 1, 1)
    plt.ylabel("X")
    plt.plot(time_log[:-1], pose_log[0, :-1], linestyle='-', lw=3, color='blue')
    plt.grid()
    plt.subplot(2, 1, 2)
    plt.ylabel("Y")
    plt.plot(time_log[:-1], pose_log[1, :-1], linestyle='-', lw=3, color='blue')
    plt.grid()
    plt.ylim([-10, 10])

