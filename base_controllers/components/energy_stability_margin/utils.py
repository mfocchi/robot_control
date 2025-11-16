
import numpy as np

class Utils:
    @staticmethod
    def Point2isRightOfLine(p0, p1, p2):
        """
        Equivalent to MATLAB's Point2isRightOfLine.
        Returns >0 if p2 is to the right of line p0→p1.
        """
        return (p2[0] - p0[0]) * (p1[1] - p0[1]) - (p1[0] - p0[0]) * (p2[1] - p0[1])

    @staticmethod
    def rotateEdge(r):
        """
        Equivalent to MATLAB's rotateEdge().
        Transforms a vector r = [x, y, z] to rp = [0, y, sqrt(x^2 + z^2)].
        """
        rp = np.zeros(3)
        rp[0] = 0.0
        rp[1] = r[1]
        rp[2] = np.sqrt(r[0]**2 + r[2]**2)
        return rp

    @staticmethod
    def sortCwise(points):
        """
        Equivalent to MATLAB's sortCwise() — sort points clockwise around centroid.
        points: list of np.array 3D vectors.
        Returns a list of np.array sorted clockwise in XY plane.
        """
        pts = np.array([p[:2] for p in points])  # project to XY plane
        centroid = np.mean(pts, axis=0)
        angles = np.arctan2(pts[:, 1] - centroid[1], pts[:, 0] - centroid[0])
        order = np.argsort(-angles)  # clockwise order
        return [points[i] for i in order]