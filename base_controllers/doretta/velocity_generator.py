from numpy import pi
import numpy as np


class VelocityGenerator:
    def __init__(self, simulation_time=10., DT=0.001):
        self.DT = DT
        self.simulation_time = simulation_time
        self.n_samples = int(simulation_time / self.DT)

    def velocity_mir_smooth(self, t1_perc = .1, t2_perc = .8, v_max_ = 0.2, omega_max_ = 0.3):
        t = np.linspace(0., self.DT * self.n_samples, self.n_samples)
        v = []
        o = []
        t1 = t1_perc*self.simulation_time
        t2 =t2_perc*self.simulation_time
        #v_max = 0.4
        omega_max =omega_max_
        v_max = v_max_
        for i in range(t.shape[0]):
            if (t[i] < t1):
                v.append(v_max * t[i])
                o.append(0)
            elif (t[i] < t2):
                #v.append(t1 * v_max - t1 * v_max / (t2 - t1) * (t[i] - t1))
                v.append(v_max)
                o.append(omega_max)
            else:
                v.append(v_max)
                o.append(0)
        s = "mir_smooth"
        return v, o, s

    def velocity_mir_smooth1(self):
        t = np.linspace(0., self.DT * self.n_samples, self.n_samples)
        v = []
        o = []
        t1 = 5
        t2 = 10
        v_max = 0.5
        for i in range(t.shape[0]):
            if (t[i] < t1):
                v.append(v_max * t[i])
                o.append(0.5)
            elif (t[i] < t2):
                v.append(t1 * v_max - t1 * v_max / (t2 - t1) * (t[i] - t1))
                o.append(-0.5)
            else:
                v.append(0.)
                o.append(0.)
        s = "mir_smooth"
        return v, o, s

