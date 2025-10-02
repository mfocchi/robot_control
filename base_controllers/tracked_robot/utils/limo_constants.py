import numpy as np

MAX_LINEAR_VELOCITY = 1.0  # m/s
MAX_ANGULAR_VELOCITY = 2.0  # radians per second
TRACK_WIDTH = 0.172
TRACK_LENGTH = 0.22
SPROCKET_RADIUS = 0.0475 # without tracks
#SPROCKET_RADIUS = 0.055 # with tracks
MAXSPEED_RADS_PULLEY = 25

#identified with v = 0.2 friction coeff 0.1
side_slip_angle_coefficients = np.array([  -1.1280,   -8.2517])
#this should be used only with positive radius!
beta_slip_inner_coefficients_left =np.array([-0.0476,   -1.8231])
beta_slip_outer_coefficients_left =np.array([ 0.3244,   -9.3033])
#this should be used only with negative radius!
beta_slip_inner_coefficients_right =np.array([ -0.0477,    1.8240])
beta_slip_outer_coefficients_right =np.array([ 0.3232 ,   9.2790])

