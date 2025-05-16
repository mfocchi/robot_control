import numpy as np
import onnxruntime as ort
import os
import json

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
        
        self.q_def = self.cfg["q_def"]
        self.q_des = self.q_def
        self.action_scale = self.cfg["action_scale"]
        
        self.ord = self.cfg["q_ord"]
        
        self.kp = np.full(12, self.cfg["kp"])
        self.kd = np.full(12, self.cfg["kd"])

    
        self.prev_action = np.zeros(12)
        self.velocity_cmd = np.zeros(3)
        
        self.decimation = (1/dt)*(1/freq)
        self.decimation_counter = 0
        
    def action(self, base_lin_vel, base_ang_vel, pj_gravity, q, qd):
        
        if self.decimation_counter == 0 :
            
            # fix ordering of locosim->isaac
            q_ord = np.zeros(12)
            for i in range(12):
                q_ord[i] = q[self.ord[i]]
            
            joint_pos_rel = q_ord - self.q_def
            
            obs = np.concatenate([
                base_lin_vel,
                base_ang_vel,
                pj_gravity,
                self.velocity_cmd,
                joint_pos_rel,
                qd,
                self.prev_action
            ]).astype(np.float32)[None]
            
            action = self.model.run(None, {'obs': obs})[0][0]
            self.prev_action = action
            
            final_action = self.q_def + ( self.action_scale * action) 
            
            # fix ordering of isaac->locosim
            q_ord = np.zeros(12)
            for i in range(12):
                # print(i, self.ord[i])
                q_ord[self.ord[i]] = final_action[i]
            
            # print(final_action, q_ord)
            
            self.q_des = q_ord
            
        
        self.decimation_counter = (self.decimation_counter+1) % self.decimation
        
        return self.q_des
        