from numpy import pi
from base_controllers.doretta.utils.constants import FREQUENCY
from base_controllers.doretta.utils.constants import DT
import numpy as np

R_AMPIO = 1.2
R_STRETTO = 0.8
V_ALTA = 0.3
V_BASSA = 0.1
simulation_time = 10.
n_samples = int(simulation_time/DT)

def velocity_luigi():
    t =np.linspace(0., DT*n_samples,n_samples)
    v = []
    o = []
    for i in range(t.shape[0]):
        if (t[i] < 2):
            v.append(10)
            o.append(0)
        elif (t[i] < 5):
            v.append(10)
            o.append(-0.7)
        else:
            v.append(0.)
            o.append(0.)
    s = "luigi"
    return v, o, s

def velocity_mir():
    t =np.linspace(0., DT*n_samples,n_samples)
    v = []
    o = []
    for i in range(t.shape[0]):
        if (t[i] < 2):
            v.append(0.4)
            o.append(0)
        elif (t[i] < 5):
            v.append(1)
            o.append(-0.2)
        else:
            v.append(0.)
            o.append(0.)
    s = "mir"
    return v, o, s

def velocity_mir_smooth():
    t =np.linspace(0., DT*n_samples,n_samples)
    v = []
    o = []
    t1 = 3
    t2 = 7
    for i in range(t.shape[0]):
        if (t[i] < t1):
            v.append(0.4*t[i])
            o.append(0)
        elif (t[i] < t2):
            v.append(t1*0.4 - t1*0.4/(t2-t1)*(t[i] - t1))
            o.append(-0.8)
        else:
            v.append(0.)
            o.append(0.)
    s = "mir_smooth"
    return v, o, s

def velocity_test0():
    # linear velocity, angular velocity
    v = [1 / 20 * pi] * 15 * FREQUENCY
    o = [pi / 15] * 15 * FREQUENCY
    s = "vt0"
    return v, o, s


def velocity_test1():
    # linear velocity, angular velocity
    v = [3 / 32 * pi] * 8 * FREQUENCY
    o = [pi / 8] * 8 * FREQUENCY
    v = v + [3 / 32 * pi] * 8 * FREQUENCY
    o = o + [-pi / 8] * 8 * FREQUENCY
    s = "vt1"
    return v, o, s


def velocity_test2():
    # linear velocity, angular velocity
    v = [3 / 20 * pi] * 5 * FREQUENCY
    o = [pi / 5] * 5 * FREQUENCY
    v = v + [3 / 20 * pi] * 5 * FREQUENCY
    o = o + [-pi / 5] * 5 * FREQUENCY
    s = "vt2"
    return v, o, s


def velocity_test3():
    # linear velocity, angular velocity
    v = [3 / 20 * pi] * 5 * FREQUENCY
    o = [pi / 5] * 5 * FREQUENCY
    v = v + [9 / 20 * pi] * 2 * FREQUENCY
    o = o + [- 3 * pi / 5] * 2 * FREQUENCY
    s = "vt3"
    return v, o, s


def velocity_test4():
    # linear velocity, angular velocity
    t_end = 20
    n_samples = t_end * FREQUENCY
    radius = 0.8
    o = np.linspace(0.1, 1, n_samples)
    v = radius * o
    o = o.tolist()
    v = v.tolist()

    s = "velocity_test4"
    return v, o, s

def velocity_test5A():
    # linear velocity, angular velocity
    r = R_AMPIO
    v_ref = V_BASSA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt5A"
    return v, o, s

def velocity_test5B():
    # linear velocity, angular velocity
    r = R_STRETTO
    v_ref = V_BASSA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt5B"
    return v, o, s

def velocity_test6A():
    # linear velocity, angular velocity
    r = R_AMPIO
    v_ref = V_ALTA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt6A"
    return v, o, s

def velocity_test6B():
    # linear velocity, angular velocity
    r = R_STRETTO
    v_ref = V_ALTA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt6B"
    return v, o, s

def velocity_test7():
    v_ref = 0.2
    o_ref = 0.4
    delta_t = 30
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt7"
    return v, o, s

def velocity_test8():
    v_ref = 0.5
    o_ref = 0.5
    delta_t = 30
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt8"
    return v, o, s

def velocity_test9():
    # linear velocity, angular velocity
    r = 2.0
    v_ref = V_ALTA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt9"
    return v, o, s

def velocity_test10():
    # linear velocity, angular velocity
    r = 2.0
    v_ref = 0.5
    o_ref = 0.0
    delta_t = 5
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt10"
    return v, o, s

def velocity_test11():
    # linear velocity, angular velocity
    r = 2.0
    v_ref = 1.0
    o_ref = 0.0
    delta_t = 5
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt11"
    return v, o, s