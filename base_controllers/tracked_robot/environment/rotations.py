import numpy as np
from math import atan2


class Rotations:
    # function to compute rotation about x axis by alpha
    def Rx(alpha):
        return np.array([[1, 0, 0],
                         [0, np.cos(alpha), -np.sin(alpha)],
                         [0, np.sin(alpha), np.cos(alpha)]])

    # function to compute rotation about y axis by beta
    def Ry(beta):
        return np.array([[np.cos(beta), 0, np.sin(beta)],
                         [0, 1, 0],
                         [-np.sin(beta), 0, np.cos(beta)]])

    # function to compute rotation about z axis by gamma
    def Rz(gamma):
        return np.array([[np.cos(gamma), -np.sin(gamma), 0],
                         [np.sin(gamma), np.cos(gamma), 0],
                         [0, 0, 1]])

    # function to compute rotation matrix from euler angles
    def XYZ_euler_rotation(alpha, beta, gamma):  # wrt body frame
        return Rotations.Rx(alpha) @ Rotations.Ry(beta) @ Rotations.Rz(gamma)

    # conversion from world frame to body frame
    def optitrack_to_world_frame():
        return np.array([[0, 0, 1],
                         [1, 0, 0],
                         [0, 1, 0]])

    def compute_theta(alpha, beta, gamma):
        a = np.deg2rad(alpha)
        b = np.deg2rad(beta)
        g = np.deg2rad(gamma)
        # rotation according to XYZ Euler angles
        R_oti = Rotations.XYZ_euler_rotation(a, b, g)  # XYZ @ Identity matrix
        # change of reference frame from world to body
        R_wi = Rotations.optitrack_to_world_frame() @ R_oti

        # compute theta angle
        # x_b: x component of body frame
        x_b = R_wi @ np.array([[1], [0], [0]])
        theta = atan2(x_b[1], x_b[0])
        # print("COMPUTE THETA: " + str(theta))
        return theta