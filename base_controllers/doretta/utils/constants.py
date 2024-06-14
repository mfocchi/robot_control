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
#identified with v = 0.05
# side_slip_angle_coefficients = np.array([   0.8649,   -4.9221])
# #this should be used only with positive radius!
# beta_slip_inner_coefficients_left =np.array([-0.0508,   -2.2502])
# beta_slip_outer_coefficients_left =np.array([0.2506 ,  -7.8051])
# #this should be used only with negative radius!
# beta_slip_inner_coefficients_right =np.array([ -0.0518,    2.0325])
# beta_slip_outer_coefficients_right =np.array([ 0.2970,   10.0432])

#identified with v = 0.1
side_slip_angle_coefficients = np.array([    0.9460,   -4.4316])
#this should be used only with positive radius!
beta_slip_inner_coefficients_left =np.array([-0.1325 ,  -3.9040])
beta_slip_outer_coefficients_left =np.array([ 0.2826 ,  -4.2355])
#this should be used only with negative radius!
beta_slip_inner_coefficients_right =np.array([ -0.1311,    3.4685])
beta_slip_outer_coefficients_right =np.array([ 0.2723,    4.8530])

#com base brame
mass = 61.05
#b_com = np.array([-0.082,0,-0.032])
b_left_track_start = np.array([0.35,0.35, -0.2])
b_left_track_end = np.array([-0.35,0.35,  -0.2])
b_right_track_start = np.array([0.35,-0.35,-0.2])
b_right_track_end = np.array([-0.35,-0.35,-0.2])