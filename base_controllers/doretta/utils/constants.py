import numpy as np




MAX_LINEAR_VELOCITY: float = 1.0  # m/s
MAX_ANGULAR_VELOCITY = np.radians(120)  # radians per second
TRACK_WIDTH = 0.606
SPROCKET_RADIUS = 0.0856
MAXSPEED_MOTOR_RPM = 1500 #max speed of wheels (motors) is 1500 rpm and 157 rad /s => max omega is 1
GEARBOX_RATIO = 34.45
MAXSPEED_RADS_MOTOR = MAXSPEED_MOTOR_RPM*2*np.pi/60 #157 rad/s
MAXSPEED_RADS_PULLEY = MAXSPEED_RADS_MOTOR/GEARBOX_RATIO #4.55 rad/s

#friction coeff 0.3
side_slip_angle_coefficients = np.array([1.4451,   -4.0347])

#this should be used only with positive radius!
beta_slip_inner_coefficients_left =np.array([ 0.1706 ,  -4.8387])
beta_slip_outer_coefficients_left =np.array([0.2906  , -4.3906])

#this should be used only with negative radius!
beta_slip_inner_coefficients_right =np.array([ 0.1532  ,  4.0401])
beta_slip_outer_coefficients_right =np.array([0.3141   , 5.4072])
