import numpy as np

DT = 0.001
FREQUENCY = int(1/DT)

MAX_LINEAR_VELOCITY: float = 2.0  # m/s
MAX_ANGULAR_VELOCITY = np.radians(120)  # radians per second