 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
import math
import time as tm
import eigenpy
eigenpy.switchToNumpyMatrix()
import sys
import os

from utils.common_functions import *
from quadprog import solve_qp    

from ros_publish import RosPub

    
# Print options 
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
sys.dont_write_bytecode = True

#instantiate graphic utils
ros_pub = RosPub()



# Control loop interval
dt = 0.001
exp_duration_sin = 3.0 
exp_duration = 5.0
num_samples = (int)(exp_duration/dt)

## Matrix of KP gains
kp=eye(6)*200
# Matrix of KD gains
kd=eye(6)*20

# Parameters of Joint Reference Trajectories
amplitude = np.array([ 0.0, 0.6, 0.0, 0.0, 0.0, 0.0])
frequencies = np.array([ 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
phi = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

two_pi_f             = 2*np.pi*frequencies   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f,amplitude)
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init variables
zero = np.matrix([0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
time = 0.0
count = 0

robot = importDisplayModel(False, False)

J_old = np.matrix(np.zeros((3,6)))

#TODO 
#N = int(conf.T_SIMULATION/conf.dt)      # number of time steps
#tau     = np.empty((robot.na, N))*nan    # joint torques
#tau_c   = np.empty((robot.na, N))*nan    # joint Coulomb torques
#q       = np.empty((robot.nq, N+1))*nan  # joint angles
#v       = np.empty((robot.nv, N+1))*nan  # joint velocities
#dv      = np.empty((robot.nv, N+1))*nan  # joint accelerations
#x       = np.empty((nx,  N))*nan        # end-effector position
#dx      = np.empty((ndx, N))*nan        # end-effector velocity
#ddx     = np.empty((ndx, N))*nan        # end effector acceleration
#x_ref   = np.empty((nx,  N))*nan        # end-effector reference position
#dx_ref  = np.empty((ndx, N))*nan        # end-effector reference velocity
#ddx_ref = np.empty((ndx, N))*nan        # end-effector reference acceleration
#ddx_des = np.empty((ndx, N))*nan        # end-effector desired acceleration

# Init loggers
q_log = np.zeros((6,num_samples))
q_des_log = np.zeros((6,num_samples))
qd_log = np.zeros((6,num_samples))
qd_des_log = np.zeros((6,num_samples))
qdd_log = np.zeros((6,num_samples))
qdd_des_log = np.zeros((6,num_samples))
f_log = np.zeros((3,num_samples))
x_log = np.zeros((3,num_samples))
time_log = np.zeros(num_samples)

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -0.6, 0.6, -1.57-0.1, -1.57, 0.0]).T
#q0 = np.matrix([ 0. , -0.3,  0.7,  -1.57 ,  -1.57 ,  0. ]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
# EXERCISE 9: 
#qd0 = np.matrix([ 0, w_rad[1]*amplitude[1], 0.0, 0.0, w_rad[4]*amplitude[4], 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

q = q0
qd = qd0
qdd = qdd0

q_des  = q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration


f = np.matrix([0.0, 0.0,0.0]).T
#end effector ID
assert(robot.model.existFrame('ee_link'))
frame_ee = robot.model.getFrameId('ee_link')

#TODO
#robot.viewer.gui.addArrow('world/contact_arrow', 0.01, 1.0, EE_SPHERE_COLOR)

z_gnd = 0.0
ground_contact = False
counter_lo = 0
counter_td=0
# CONTROL LOOP
while True:   

    # Reference Generation
    
#    Sinusoidal reference TODO fix this
#    q_des  = q0 + amplitude*np.sin(two_pi_f*time  + phi )
#    qd_des = two_pi_f_amp * np.cos(two_pi_f*time + phi)
#    qdd_des = - two_pi_f_squared_amp* np.sin(two_pi_f*time + phi)            
    w_rad=2*math.pi*frequencies
    q_des  = q0 + np.matrix([ 0.0, amplitude[1]*np.sin(w_rad[1]*time), 0.0, 0.0,  amplitude[4]*np.sin(w_rad[4]*time), 0.0]).T
    qd_des = np.matrix([ 0.0,  amplitude[1]*w_rad[1]*np.cos(w_rad[1]*time), 0.0, 0.0, amplitude[4]*w_rad[4]*np.cos(w_rad[4]*time), 0.0]).T
    qdd_des = np.matrix([ 0.0, -amplitude[1]*w_rad[1]*w_rad[1]*np.sin(w_rad[1]*time), 0.0, 0.0, -amplitude[4]*w_rad[4]*w_rad[4]*np.sin(w_rad[4]*time), 0.0]).T  
#          
#        
    # Set constant reference after a while
    if time>exp_duration_sin:
        q_des  = q0
        qd_des=zero
        qdd_des=zero         
 
    # Decimate print of time
    #if (divmod(count ,1000)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= exp_duration:
        break
                            
    robot.computeAllTerms(q, qd)    

    M = robot.mass(q, False)
    h = robot.nle(q, qd, False)
    g = robot.gravity(q)                

    #contact at end-effector 
    #compute jacobian of the end effector and its derivative in the WF
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    J = J6[:3,:] # take first 3 rows of J6        
    dJdq = robot.frameClassicAcceleration(q, qd, None, frame_ee, False).linear    
    # compute frame end effector position and velocity in the WF   
    x = robot.framePlacement(q, frame_ee).translation                
    xd = J*qd    

    # ES 3 (precision depends on dt)    
#    Jdot = (J-J_old)/dt
#    J_old = J            
#    print (Jdot.dot(qd) - dJdq).T
               
    #ES 2 check 
#    v_frame = robot.frameVelocity(q, qd, frame_id, False)
#    xd = v_frame.linear                
#    print (J*qd     - v_frame.linear).T                
                       
                    
    #ES 4 contact at JOINT origin 
    #compute Jacobian and its derivative in the world frame 
#    frame_joint_id = robot.model.getJointId('wrist_3_joint')                    
#    J = robot.getJointJacobian(frame_joint_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]                   
#    Jdot = pin.getJointJacobianTimeVariation(robot.model, robot.data, frame_joint_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:] 
#    dJdq = Jdot*qd # I need to do so cause frameClassicAcceleration it does not compute accel for joint frames        
#    # compute end effector position and velocity  in the world frame 
#    x = robot.placement(q, frame_joint_id).translation                           
#    xd = J*qd



#    #ES 5 check shifting law     TODO
#    
##   parent link="wrist_3_link" child link="ee_link"  RPY ="0.0 0.0 1.570796325" xyz="0.0 0.0823 0.0"
#    frame_wrist = robot.model.getFrameId('wrist_3_link')                    
#    frame_ee = robot.model.getFrameId('ee_link')        
#                
#    Je = robot.frameJacobian(q, frame_ee, True, pin.ReferenceFrame.LOCAL)                
#    Jwr = robot.frameJacobian(q, frame_wrist, True, pin.ReferenceFrame.LOCAL)                        
#    
#    x_ee = robot.framePlacement(q, frame_ee).translation   
#    x_wr = robot.framePlacement(q, frame_wrist).translation                            
#    w_R_ee = robot.data.oMf[frame_ee].rotation
#    w_R_wr = robot.data.oMf[frame_wrist].rotation    
#    #express t in the wrist frame
#    wr_t = (w_R_wr).T * (x_ee - x_wr)
#    # find the rotation matrix that maps vectors from wrist frame to ee frame
#    ee_R_wr = w_R_ee.T*w_R_wr     
#            
#    ee_X_wr = pin.SE3(ee_R_wr, wr_t)       
#    Je_test =  ee_X_wr.toActionMatrix()*Jwr                        
#    print "TEST:\n", Je_test -Je
            

              


    ####################################
    #simulation           
    M_inv = np.linalg.inv(M)
                
    #normal pseudoinverse of J^T = (A^TA)^-1 * A^T
    #JTpinv = np.linalg.inv(J*J.T)*J             
 
    #lambda_ contact inertia
    lambda_= np.linalg.inv(J*M_inv* J.T + np.eye(3)*1e-09) 
    
    #dynamicaaly consistent psdinv
    JTpinv = lambda_*J*M_inv

    N_dyn = (eye(6)- J.T * JTpinv) 
    
    #tau = M*(qdd_des+kp*(q_des-q)+kd*(qd_des-qd)) + h    
    tau = kp*(q_des-q)+kd*(qd_des-qd) + g 
    
    #print N_dyn     
                
    #####simulator
    ################################            
    #detect contact

    #touchdown
    if (counter_lo==0) and not (ground_contact) and (x[2]<=z_gnd):                         
        counter_td = 30     
        ground_contact = True
        #compute nullsace for velcities to update the joint velcity accounting  for the impact
        Jpinv = M_inv*J.T*lambda_                
        N_vel = eye(6) - Jpinv*J                                        
        qd = N_vel*qd                            
        
        robot.computeAllTerms(q, qd)    
        h = robot.nle(q, qd, False)  
        dJdq = robot.frameClassicAcceleration(q, qd, None, frame_ee, False).linear 
        print("contact ON")  
    #liftoff  
    if ((counter_td==0) and(ground_contact) and (f[2]<=0.0)):#(x[2]>z_gnd)):
   
        #histeresis
        counter_lo = 200                
        ground_contact = False;    
        print ("contact OFF")
                                
    if counter_td > 0:
        counter_td =counter_td-1
    if counter_lo > 0:
        counter_lo =counter_lo-1                                
        #print counter                                   
                            
    # compute forward dynamics     
    qdd_uc = M_inv*(tau - h)              
    if (not ground_contact):
        qdd = qdd_uc
                                
        f = np.matrix([0.0, 0.0,0.0]).T                                
    else:
  
        qd = N_vel*qd #TODO I should not do this but this is what thaty stops the motion                 
        qdd = M_inv*( N_dyn*(tau -    h) - J.T*    lambda_*dJdq)    
        # compute contact force    
        f = lambda_*( -dJdq + J*M_inv*(h - tau) )
        torques_row_space =  N_dyn*(tau -    h)    
        #print  torques_row_space    #they are almost zero cause the robot is not redundant
                                
        # EXERCISE 1: check contact force disappeared when projected
        #print (N_dyn* J.T*f).T                    
                                
       
                            
       #EX 6 gauss principle of leas constraint verification
       #Minimize     1/2 x^T G x - a^T x
       #Subject to   C.T x >= b
    #                                    G : array, shape=(n, n)
    #        matrix appearing in the quadratic function to be minimized
    #    a : array, shape=(n,)
    #        vector appearing in the quadratic function to be minimized
    #    C : array, shape=(n, m)
    #        matrix defining the constraints under which we want to minimize the
    #        quadratic function
    #    b : array, shape=(m), default=None
    #        vector defining the constraints
    #    meq : int, default=0
    #        the first meq constraints are treated as equality constraints,
    #        all further as inequality constraints (defaults to 0).

        qp_C = J.T
        qp_b = -dJdq
        meq = J.shape[0]                       
        p = M*qdd_uc                        

        qdd_check = solve_qp(M,p.A1,qp_C, qp_b.A1, meq)  [0]  
                        
#        print("proj - gauss")                     
#        print( qdd.A1 - qdd_check)
#      
                    
                                
                        
                        
    # Forward Euler Integration        
    q = q + qd*dt + 0.5*dt*dt*qdd
    qd = qd + qdd*dt
    
    # Log Data into a vector
    q_log[:,count] = q.flatten()
    q_des_log[:,count] = q_des.flatten()
    qd_log[:,count] = qd.flatten()
    qd_des_log[:,count] = qd_des.flatten()
    qdd_log[:,count] = qdd.flatten()
    qdd_des_log[:,count] = qdd_des.flatten()
    time_log[count] = time 
    f_log[:,count] = f.flatten()
    x_log[:,count] = x.flatten()
    time = time + dt 
    count +=1
				
    ros_pub.add_marker(x.A1.tolist()) 
    #reduce f for plotting purposes				
    red_f = f/100 				
    ros_pub.add_arrow(x.A1.tolist(), red_f.A1.tolist()) 
    ros_pub.publish(robot, q, qd, tau)
    #stops the while when you hit CTRL+C					
    if ros_pub.isShuttingDown():
        print ("Shutting Down")					
        break;
            
                    
    tm.sleep(dt*1)
                      

ros_pub.deregister_node()
                    
                            
plot('pos', time_log,  q_des_log, q_log, qd_des_log, qd_log, qdd_des_log, qdd_log, num_samples)
#plot('vel', q_des_log, q_log, qd_des_log, qd_log, qdd_des_log, qdd_log)
#plot('acc', q_des_log, q_log, qd_des_log, qd_log, qdd_des_log, qdd_log)

plt.figure(2)
plt.subplot(3,1,1)
plt.title("fx")    
plt.plot(time_log, f_log[0,:],color = 'red')
plt.grid()

plt.subplot(3,1,2)
plt.title("fy")    
plt.plot(time_log, f_log[1,:],color = 'red')
plt.grid()

plt.subplot(3,1,3)
plt.title("fz")    
plt.plot(time_log, f_log[2,:],color = 'red')
plt.grid()

plt.figure(3)
plt.subplot(3,1,1)
plt.title("x")    
plt.plot(time_log, x_log[0,:],color = 'red')
plt.grid()

plt.subplot(3,1,2)
plt.title("y")    
plt.plot(time_log, x_log[1,:],color = 'red')
plt.grid()

plt.subplot(3,1,3)
plt.title("z")    
plt.plot(time_log, x_log[2,:],color = 'red')
plt.grid()



