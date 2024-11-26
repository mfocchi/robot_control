import numpy as np




MAX_LINEAR_VELOCITY: float = 1.0  # m/s
MAX_ANGULAR_VELOCITY = np.radians(120)  # radians per second
TRACK_WIDTH = 0.606
SPROCKET_RADIUS = 0.0856
MAXSPEED_MOTOR_RPM = 1500 #max speed of wheels (motors) is 1500 rpm and 157 rad /s => max omega is 1
GEARBOX_RATIO = 34.45 #USED JUST TO COMPUTE MAX SPEED
MAXSPEED_RADS_MOTOR = MAXSPEED_MOTOR_RPM*2*np.pi/60 #157 rad/s
MAXSPEED_RADS_PULLEY = 18. #MAXSPEED_RADS_MOTOR/GEARBOX_RATIO #previously was 4.5/10

#identified with v = 0.2 friction coeff 0.1
side_slip_angle_coefficients = np.array([  -1.1280,   -8.2517])
#this should be used only with positive radius!
beta_slip_inner_coefficients_left =np.array([-0.0476,   -1.8231])
beta_slip_outer_coefficients_left =np.array([ 0.3244,   -9.3033])
#this should be used only with negative radius!
beta_slip_inner_coefficients_right =np.array([ -0.0477,    1.8240])
beta_slip_outer_coefficients_right =np.array([ 0.3232 ,   9.2790])

#com base brame
mass = 61.05
inertia_moment = 4.5
#b_com = np.array([-0.082,0,-0.032])
b_left_track_start = np.array([0.35,0.35, -0.2])
b_left_track_end = np.array([-0.35,0.35,  -0.2])
b_right_track_start = np.array([0.35,-0.35,-0.2])
b_right_track_end = np.array([-0.35,-0.35,-0.2])