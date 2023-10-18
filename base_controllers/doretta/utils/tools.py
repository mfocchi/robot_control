import numpy as np
import math

def unwrap(rpy_meas, rpy_old):
    rpy_unwrapped = np.zeros(3)
    for i in range(3):
        rpy_unwrapped[i] = rpy_meas[i]
        while (rpy_unwrapped[i] < rpy_old[i] - math.pi):
            rpy_unwrapped[i] += 2 * math.pi
        while (rpy_unwrapped[i] > rpy_old[i] + math.pi):
            rpy_unwrapped[i] -= 2 * math.pi
        rpy_old[i] = rpy_unwrapped[i]
    return rpy_unwrapped

def unwrap_angle(angle_meas, angle_old):
    angle_unwrapped = 0.
    for i in range(3):
        angle_unwrapped = angle_meas
        while (angle_unwrapped < angle_old - math.pi):
            angle_unwrapped += 2 * math.pi
        while (angle_unwrapped > angle_old + math.pi):
            angle_unwrapped -= 2 * math.pi
        angle_old = angle_unwrapped
    return angle_unwrapped

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle