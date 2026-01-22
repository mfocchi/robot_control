import numpy as np
import onnxruntime as ort
import os
import json
from termcolor import colored

class RlVelocityController():
    def __init__(self, robot_name: str, dt: float, freq: int = 50):
        
        self.robot_name = robot_name
        
        base_model_path = os.path.join(os.environ.get('LOCOSIM_DIR'),
                                       'robot_control',
                                       'base_controllers',
                                       'components',
                                       'rl_velocity_controller',
                                       'policies')

        config_path = os.path.join(base_model_path, f"{robot_name}.json")

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Policy does not exist for {self.robot_name}!!!")

        with open(config_path, 'r') as f:
            self.cfg = json.load(f)

        self.model_path = os.path.join(base_model_path, f'{robot_name}.onnx')
        self.model = ort.InferenceSession(self.model_path)
        print(f'Policy for {robot_name} loaded')

        self.model_path_safe = os.path.join(base_model_path, f'{robot_name}_safe.onnx')
        self.model_safe = ort.InferenceSession(self.model_path_safe)
        print(f'Policy for {robot_name}_safe loaded')

        self.q_def = self.cfg["q_def"]
        self.q_des = self.q_def
        self.action_scale = self.cfg["action_scale"]
        

        
        self.kp = np.full(12, self.cfg["kp"])
        self.kd = np.full(12, self.cfg["kd"])

    
        self.prev_action = np.zeros(12)
        self.velocity_cmd = np.zeros(3)
        
        self.decimation = (1/dt)*(1/freq)
        self.decimation_counter = 0
        
    def action(self, base_lin_vel, base_ang_vel, pj_gravity, q, qd, policy_type='default'):
        
        if self.decimation_counter == 0 :

            joint_pos_rel = q - self.q_def

            
            obs = np.concatenate([
                base_lin_vel,
                base_ang_vel,
                pj_gravity,
                self.velocity_cmd,
                joint_pos_rel,
                qd,
                self.prev_action
            ]).astype(np.float32)[None]

            if policy_type=='default':
                action = self.model.run(None, {'obs': obs})[0][0]
            elif policy_type=='safe':
                action = self.model_safe.run(None, {'obs': obs})[0][0]
            else:
                colored("Wrong polocy type", "red")

            self.prev_action = action
            
            final_action = self.q_def + ( self.action_scale * action) 

            
            self.q_des = final_action
            
        
        self.decimation_counter = (self.decimation_counter+1) % self.decimation
        
        return self.q_des
        