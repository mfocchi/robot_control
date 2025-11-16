import numpy as np
from utils import Utils

utility = Utils()

def computeESM(f, CoM, w_R_b, check_graphics=False):
    """
    Compute Edge Stability Margin (ESM)
    -----------------------------------
    Parameters
    ----------
    f : list of np.ndarray
        Stance foot positions in base frame (each 3x1)
    CoM : np.ndarray
        Center of mass in base frame (3x1)
    w_R_b : np.ndarray
        Rotation matrix (world <- base)
    check_graphics : bool
        Show debugging arrows and geometry

    Returns
    -------
    float : ESM value
    """
    count = len(f)
    CoMW = w_R_b @ CoM
    factor = 0.0

    # -----------------------------
    # Edge construction
    if count == 4:
        edges = [f[1]-f[0], f[2]-f[1], f[3]-f[2], f[0]-f[3]]
        edge_start = [f[0], f[1], f[2], f[3]]
        edge_startW = [w_R_b @ e for e in edge_start]

        # Check if CoM is inside polygon
        if (utility.Point2isRightOfLine(edge_startW[0], edge_startW[1], CoMW) > 0 and
            utility.Point2isRightOfLine(edge_startW[1], edge_startW[2], CoMW) > 0 and
            utility.Point2isRightOfLine(edge_startW[2], edge_startW[3], CoMW) > 0 and
            utility.Point2isRightOfLine(edge_startW[3], edge_startW[0], CoMW) > 0):
            factor = 1.0

    else:  # 3 legs
        edges = [f[1]-f[0], f[2]-f[1], f[0]-f[2]]
        edge_start = [f[0], f[1], f[2]]
        edge_startW = [w_R_b @ e for e in edge_start]

        if (utility.Point2isRightOfLine(edge_startW[0], edge_startW[1], CoMW) > 0 and
            utility.Point2isRightOfLine(edge_startW[1], edge_startW[2], CoMW) > 0 and
            utility.Point2isRightOfLine(edge_startW[2], edge_startW[0], CoMW) > 0):
            factor = 1.0

    # -----------------------------
    # Compute ESM
    esm_values = []
    for i in range(count):
        edge_n = edges[i] / np.linalg.norm(edges[i])
        r = (CoM - edge_start[i]) - np.dot((CoM - edge_start[i]), edge_n) * edge_n
        p = CoM - r

        # Gravity vector in base frame
        gz = w_R_b.T @ np.array([0.0, 0.0, 1.0])

        # Frame aligned with z and edge
        x = np.cross(edge_n, gz)
        x /= np.linalg.norm(x)
        baseM = np.column_stack((x, edge_n, gz))

        # Transform r into local edge frame, apply rotateEdge(), map back
        rp = baseM @ utility.rotateEdge(np.linalg.inv(baseM) @ r)

        # Project onto z-axis and compute ESM
        esm_val = np.dot(gz, rp) - np.dot(gz, r)
        esm_values.append(esm_val)

        if check_graphics:
            import matplotlib.pyplot as plt
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.quiver(*edge_start[i], *edges[i], color='r')
            ax.quiver(*edge_start[i], *(CoM - edge_start[i]), color='b')
            ax.scatter(*CoM, color='k', label='CoM')
            ax.scatter(0, 0, 0, color='r', label='Origin')
            ax.quiver(*p, *r, color='k')
            ax.quiver(*p, *rp, color='g')
            ax.quiver(*p, *gz, color='m')
            ax.quiver(*p, *x, color='c')
            ax.legend()
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'Edge {i+1}')
            plt.show()

    # Return minimum ESM scaled by inside-polygon factor
    return factor * np.min(esm_values)
