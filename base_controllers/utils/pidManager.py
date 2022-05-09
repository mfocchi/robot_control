from ros_impedance_controller.srv import set_pids
from ros_impedance_controller.srv import set_pidsRequest
from ros_impedance_controller.msg import pid

import rospy as ros
import copy

class PidManager:

    def __init__(self, jnames):
        print("Initializing PID Manager")
        self.joint_names = jnames
        try:
            ros.wait_for_service("/set_pids", 5.0)
            self.set_pd_service = ros.ServiceProxy("/set_pids", set_pids)
        except (ros.ServiceException, ros.ROSException) as e:
            ros.logger("Service call non available: %s" % (e,))
        self.joint_pid = pid()
        self.joint_pid_log = len(jnames)*[pid()]
        self.req_msg = set_pidsRequest()						
	                                
    def setPDs(self, kp, kd, ki):              
        # create the message
        self.req_msg.data = []
								
        # fill in the message with des values for kp kd
        for i in range(len(self.joint_names)):
            self.joint_pid.joint_name = self.joint_names[i]
            self.joint_pid.p_value = kp
            self.joint_pid.d_value = kd
            self.joint_pid.i_value = ki
            self.req_msg.data += [copy.deepcopy(self.joint_pid)]
            self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)
												
        # send request and get response (in this case none)
        self.set_pd_service(self.req_msg)
															
								
  
    def setPDleg(self, legid, kp, kd, ki):
        # create the message
        self.req_msg.data = []

        # fill in the message with des values for kp kd
        for i in range(len(self.joint_names)):

            if (legid == 0):
                if ((self.joint_names[i] == 'lf_haa_joint') or (self.joint_names[i] == 'lf_hfe_joint') or (
                        self.joint_names[i] == 'lf_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

            if (legid == 1):
                if ((self.joint_names[i] == 'rf_haa_joint') or (self.joint_names[i] == 'rf_hfe_joint') or (
                        self.joint_names[i] == 'rf_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

            if (legid == 2):
                if ((self.joint_names[i] == 'lh_haa_joint') or (self.joint_names[i] == 'lh_hfe_joint') or (
                        self.joint_names[i] == 'lh_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

            if (legid == 3):
                if ((self.joint_names[i] == 'rh_haa_joint') or (self.joint_names[i] == 'rh_hfe_joint') or (
                        self.joint_names[i] == 'rh_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)
																				
       
        self.req_msg.data = self.joint_pid_log

        # send request and get response (in this case none)
        self.set_pd_service(self.req_msg)
        
        
        
        
    def setPDjoint(self, joint_idx, kp, kd, ki):
        # create the message
        self.req_msg.data = []

        # fill in the message with des values for kp kd
        self.joint_pid.joint_name = self.joint_names[joint_idx]
        self.joint_pid.p_value = kp
        self.joint_pid.d_value = kd
        self.joint_pid.i_value = ki
        self.joint_pid_log[joint_idx] = copy.deepcopy(self.joint_pid)
       
        self.req_msg.data =  copy.deepcopy(self.joint_pid_log)

        # send request and get response (in this case none)
        self.set_pd_service(self.req_msg)
        
    def setPDjoints(self, kp, kd, ki):
        # create the message
        self.req_msg.data = []
        
        for joint_idx in range(len(self.joint_names)):
            # fill in the message with des values for kp kd
            self.joint_pid.joint_name = self.joint_names[joint_idx]
            self.joint_pid.p_value = kp[joint_idx]
            self.joint_pid.d_value = kd[joint_idx]
            self.joint_pid.i_value = ki[joint_idx]
            self.joint_pid_log[joint_idx] = copy.deepcopy(self.joint_pid)
       
        self.req_msg.data =  copy.deepcopy(self.joint_pid_log)

        # send request and get response (in this case none)
        self.set_pd_service(self.req_msg)