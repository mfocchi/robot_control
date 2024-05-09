

import matplotlib.pyplot as plt
import numpy as np

def generateDisturbanceOnSphere(amp):
    # generate samples from gaussian normal distribution with mean 0 and std dev 1
    direction = np.random.randn(3)

    #direction = np.random.uniform(low=-1, high=1, size=3)

    direction /= np.linalg.norm(direction, axis=0)
    # sample magnitude
    #amp = min + max * np.random.randn()

    return amp * direction

def generateDisturbanceOnHemiSphere(amp):
    # generate samples from gaussian normal distribution with mean 0 and std dev 1
    #direction = np.random.randn(3)
    direction = np.random.uniform(low=-1, high=1, size=3)
    print(direction)
    direction /= np.linalg.norm(direction, axis=0)
    # hemisphere 1
    if direction[2] >0:
        direction[2]*= -1
    return amp * direction


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

plt.title("hemisphere")
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
for i in range(200):
    point = generateDisturbanceOnSphere(10)
    ax.scatter(point[0], point[1], point[2], marker='o', color='g', s=20)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

for i in range(200):
    point = generateDisturbanceOnHemiSphere(10)
    ax.scatter(point[0], point[1], point[2], marker='o', color='g', s=20)



plt.show()