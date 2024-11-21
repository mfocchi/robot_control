import numpy as np
from termcolor import colored
from tracked_vehicle_simulator3d import TrackedVehicleSimulator3D
from tracked_vehicle_simulator3d import Ground3D
from matplotlib import pyplot as plt #need to import as last to avoid issues with qt
np.set_printoptions(threshold=np.inf, precision = 9, linewidth = 10000, suppress = True)
if __name__ == '__main__':
    dt = 0.0005
    t_end = 20.
    pose_init = [0., 0., 0.0, 0, 0, 0]
    twist_init = [0., 0., 0.0, 0, 0, 0]
    number_of_steps = np.int32(t_end / dt)
    omega_left = np.linspace(0, 2, number_of_steps)
    omega_right = np.linspace(2, 2, number_of_steps)
    terrain_roll_vec = np.linspace(0.0, 0.0, number_of_steps)
    terrain_pitch_vec = np.linspace(0.0, -0.1, number_of_steps)  # if you set -0.3 the robot starts to slip backwards!
    terrain_yaw_vec = np.linspace(0.0, -0.0, number_of_steps)  # if you set -0.3 the robot starts to slip backwards!

    groundParams = Ground3D()
    p = TrackedVehicleSimulator3D(dt=dt, ground=groundParams, USE_MESH=False, DEBUG=False, int_method='FORWARD_EULER', enable_visuals=False)

    time = 0.
    sim_counter = 0
    pose_log = np.full((6, number_of_steps), np.nan)
    time_log = np.full((number_of_steps), np.nan)
    p.initSimulation(pose_init, twist_init)
    while time < t_end:
        if np.mod(time, 1) == 0:
            print(colored(f"TIME: {time}", "red"))
        terrain_roll = terrain_roll_vec[sim_counter]
        terrain_pitch = terrain_pitch_vec[sim_counter]
        terrain_yaw = terrain_yaw_vec[sim_counter]
        pg = np.array([p.pose[0], p.pose[1], p.computeZcomponent(p.pose[0], p.pose[1], terrain_pitch)])
        # updates pose and twist
        b_eox, b_eoy = p.simulateOneStep(pg, terrain_roll, terrain_pitch, terrain_yaw, omega_left[sim_counter], omega_right[sim_counter])
        # log
        pose_log[:, sim_counter] =  p.pose
        time_log[sim_counter] = time

        time = np.round(time + dt, 4)
        sim_counter += 1


    # xy plot
    plt.figure()
    plt.plot(pose_log[0, :-1], pose_log[1, :-1], "-b", label="actual")
    plt.legend()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(True)

    fig = plt.figure()
    fig.suptitle("lin states", fontsize=20)
    plt.subplot(3, 1, 1)
    plt.ylabel("X")
    plt.plot(time_log[:-1], pose_log[0, :-1], linestyle='-', lw=3, color='blue')
    plt.grid()
    plt.subplot(3, 1, 2)
    plt.ylabel("Y")
    plt.plot(time_log[:-1], pose_log[1, :-1], linestyle='-', lw=3, color='blue')
    plt.grid()
    plt.ylim([-10, 10])
    plt.subplot(3, 1, 3)
    plt.ylabel("Z")
    plt.plot(time_log[:-1], pose_log[2, :-1], linestyle='-', lw=3, color='blue')
    plt.xlabel("Time [s]")
    plt.grid()

    fig = plt.figure()
    fig.suptitle("ang states", fontsize=20)
    plt.subplot(3, 1, 1)
    plt.ylabel("roll")
    plt.plot(time_log[:-1], pose_log[3, :-1], linestyle='-', lw=3, color='blue')
    plt.grid()
    plt.ylim([-0.5, 0.5])
    plt.subplot(3, 1, 2)
    plt.ylabel("pitch")
    plt.plot(time_log[:-1], pose_log[4, :-1], linestyle='-', lw=3, color='blue')
    plt.grid()
    plt.ylim([-0.5, 0.5])
    plt.subplot(3, 1, 3)
    plt.ylabel("yaw")
    plt.plot(time_log[:-1], pose_log[5, :-1], linestyle='-', lw=3, color='blue')
    plt.xlabel("Time [s]")
    plt.grid()
    plt.ylim([-5, 5])
