import numpy as np

from ..utils.tools import normalize_angle

# ------------------------------------ #
# CONTROLLER'S PARAMETERS
# K_THETA = 0.8
# K_E = 0.07
# EPSILON_V = 0.01
# K_DELTA = 2.5
# ------------------------------------ #

class StanleyParams:
    def __init__(self, K_THETA, K_E, EPSILON_V, K_DELTA):
        self.K_THETA = K_THETA
        self.K_E = K_E
        self.EPSILON_V = EPSILON_V
        self.K_DELTA = K_DELTA

class StanleyController:
    def __init__(self, path, params: StanleyParams):

        self.K_THETA = params.K_THETA
        self.K_E = params.K_E
        self.EPSILON_V = params.EPSILON_V
        self.K_DELTA = params.K_DELTA

        self.path = path
        self.theta_error = [0.0]
        self.cross_error = [0.0]
        self.last_target_idx = 0
        self.goal_target = len(path.x) -1
        self.goal_reached = False
        pass

    def control(self, robot):
        current_target_idx, error_front_axle = self.calc_target_index(robot, self.path)

        if self.last_target_idx >= current_target_idx:
            current_target_idx = self.last_target_idx

        # theta_e corrects the heading error
        theta_e = self.K_THETA * normalize_angle(self.path.theta[current_target_idx] - robot.theta)
        self.theta_error.append(theta_e)
        self.cross_error.append(error_front_axle)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.K_E * error_front_axle, robot.v + self.EPSILON_V)
        # Steering control
        delta = theta_e + theta_d
        delta = self.K_DELTA * delta
        # DEBUG print controller
        # print("THETA: %f PATH: %f" % (np.rad2deg(robot.theta), np.rad2deg(path.theta[current_target_idx])))
        # print("THETA ERROR: %f CROSS ERROR " % np.rad2deg(theta_e) + str(error_front_axle))

        # updating path index
        self.last_target_idx = current_target_idx
        if self.last_target_idx == self.goal_target:
            print("GOAL REACHED")
            self.goal_reached = True

        return delta, current_target_idx

    def longitudinal_velocity_controller(self, index):
        # vel = 0.1
        # vel = np.clip(vel, -MAX_SPEED, MAX_SPEED)
        vel = self.path.v[index]
        return vel

    def calc_target_index(self, robot, path):
        """
        Compute index in the trajectory list of the target.
        :return: (int, float)
        """
        # Calc front axle position, which is robot position

        fx = robot.x
        fy = robot.y
        cx = path.x
        cy = path.y


        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)  # calcola distanza tra robot e ogni punto della traiettoria
        target_idx = np.argmin(d)  # restituisce l'indice del punto più vicino

        # Project RMS error onto front axle vector
        # calcola l'errore tra l'orientamento del robot e l'orientamento del segmento più vicino
        front_axle_vec = [-np.cos(robot.theta + np.pi / 2),
                          -np.sin(
                              robot.theta + np.pi / 2)]
        # vettore perpendicolare all'orientamento del robot, norma = 1 dato che sono seno e coseno
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        # print("DEBUG %d %.2f" % (target_idx, error_front_axle))
        return target_idx, error_front_axle
