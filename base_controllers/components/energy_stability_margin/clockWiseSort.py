import numpy as np
from utils import Utils

utility = Utils()

def clockWiseSort(stance_vec):
    """
    Performs a clockwise radial sort of the input stance points.
    Equivalent to MATLAB's clockWiseSort().

    Parameters
    ----------
    stance_vec : list of np.ndarray
        Each element is a 3x1 foot position vector.

    Returns
    -------
    list of np.ndarray
        Sorted points (clockwise around stance_vec[0]).
    """
    stance_vec = stance_vec.copy()
    nlegs = len(stance_vec)

    # Sort clockwise: ensure each next point is to the right of the line from first to current
    for i in range(1, nlegs - 1):  # MATLAB indices start at 1
        for j in range(i + 1, nlegs):
            p0 = stance_vec[0]
            p1 = stance_vec[i]
            p2 = stance_vec[j]
            if utility.Point2isRightOfLine(p0, p1, p2) < 0.0:  # for counter-clockwise use >0
                stance_vec[i], stance_vec[j] = stance_vec[j], stance_vec[i]

    return stance_vec