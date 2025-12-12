import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from math import cos, sin, radians
from utils import Utils
from esm import computeESM
from clockWiseSort import clockWiseSort

# ------------------------------------------------------------
# Initialize utilities
utility = Utils()

# Terrain angles (degrees)
terrain_pitch = 30
terrain_roll = 0

# Foot positions in base frame (meters)
LF = np.array([0.384751, 0.207, -0.577608])
RF = np.array([0.384751, -0.207, -0.577608])
LH = np.array([-0.384751, 0.207, -0.577608])
RH = np.array([-0.384751, -0.207, -0.577608])

# Choose stance configuration
#stance_vecB = [LF, RF, LH, RH]  # 4 legs
# stance_vecB = [LF, RF, RH]    # LH swing
stance_vecB = [LF, RF, LH]    # RH swing
# stance_vecB = [LF, LH, RH]    # RF swing
# stance_vecB = [RF, LH, RH]    # LF swing

# Clockwise sort stance vectors
stance_vecB = clockWiseSort(stance_vecB)

# Rotation matrices
def Rx(angle):
    return np.array([
        [1, 0, 0],
        [0, cos(angle), -sin(angle)],
        [0, sin(angle), cos(angle)]
    ])

def Ry(angle):
    return np.array([
        [cos(angle), 0, sin(angle)],
        [0, 1, 0],
        [-sin(angle), 0, cos(angle)]
    ])

# Rotation from base to world
w_R_b = Ry(radians(terrain_pitch)) @ Rx(radians(terrain_roll))

# ------------------------------------------------------------
# Compute support polygon in world frame
Xv, Yv = [], []
for leg in stance_vecB:
    stance_vecW = w_R_b @ leg  # map to world
    Xv.append(stance_vecW[0])
    Yv.append(stance_vecW[1])

Xv = np.array(Xv)
Yv = np.array(Yv)
points = np.vstack((Xv, Yv)).T
kW = ConvexHull(points)
polygon = points[kW.vertices]

# ------------------------------------------------------------
# Random sample generation
x = -0.75 + 1.5 * np.random.rand(10000)
y = -0.75 + 1.5 * np.random.rand(10000)

# Keep points inside convex hull (like inpolygon)
from matplotlib.path import Path
path = Path(polygon)
points_xy = np.vstack((x, y)).T
mask = path.contains_points(points_xy)
idx_in = np.where(mask)[0]

# ------------------------------------------------------------
# ESM computation
CoM = np.array([0.20633, 0.057005, -0.00131821])
esm_vals = []
xvec, yvec = [], []
esm_max = -np.inf
targetCoM = np.zeros(2)

for j in idx_in:
    CoM_test = np.array([x[j], y[j], CoM[2]])
    esm = computeESM(stance_vecB, CoM_test, w_R_b, False)
    esm_vals.append(esm)
    xvec.append(x[j])
    yvec.append(y[j])
    if esm > esm_max:
        esm_max = esm
        targetCoM = [x[j], y[j]]

# ------------------------------------------------------------
# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#plot samples
ax.scatter(xvec, yvec, esm_vals, c='b', s=5, label='Samples')
ax.scatter(targetCoM[0], targetCoM[1], esm_max, c='r', s=80, label='Max ESM')

# Plot support polygon (in world XY)
poly_x = polygon[:, 0].tolist() + [polygon[0, 0]]
poly_y = polygon[:, 1].tolist() + [polygon[0, 1]]
ax.plot3D(poly_x, poly_y, [0]*len(poly_x), 'g', linewidth=3, label='Support Polygon')

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('ESM')
ax.legend()
ax.set_title(f"Terrain pitch={terrain_pitch}°, roll={terrain_roll}°")
plt.tight_layout()
plt.show()

print("Target CoM:", targetCoM)
print("Max ESM:", esm_max)
