from numpy import pi
import numpy as np


class VelocityGenerator:
    def __init__(self, simulation_time=10., DT=0.001):
        self.DT = DT
        self.n_samples = int(simulation_time / self.DT)

    def velocity_mir_smooth(self):
        t = np.linspace(0., self.DT * self.n_samples, self.n_samples)
        v = []
        o = []
        t1 = 3
        t2 = 7
        #v_max = 0.4

        v_max = 0.1
        for i in range(t.shape[0]):
            if (t[i] < t1):
                v.append(v_max * t[i])
                o.append(0)
            elif (t[i] < t2):
                v.append(t1 * v_max - t1 * v_max / (t2 - t1) * (t[i] - t1))
                o.append(-0.2)
            else:
                v.append(0.)
                o.append(0.)
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

