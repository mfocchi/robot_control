import os

import numpy as np
import torch
# Value function network load
import flax.linen as nn_flax
import jax
from jax import numpy as jnp
from functools import partial
import pickle


def load_value(file_path):
    with open(file_path, 'rb') as f:
        return pickle.load(f)


class CriticNetwork(nn_flax.Module):
    @nn_flax.compact
    def __call__(self, x):
        x = nn_flax.Dense(512)(x)
        x = nn_flax.LayerNorm()(x)
        x = nn_flax.elu(x)
        x = nn_flax.Dense(256)(x)
        x = nn_flax.LayerNorm()(x)
        x = nn_flax.elu(x)
        x = nn_flax.Dense(128)(x)
        x = nn_flax.LayerNorm()(x)
        x = nn_flax.elu(x)
        # Output initialized to 1 (probability of survival)
        x = nn_flax.Dense(1, kernel_init=nn_flax.initializers.zeros, bias_init=nn_flax.initializers.ones)(x)
        # x = nn.Dense(1)(x)
        return x.squeeze(-1)


class CriticEvaluator:
    def __init__(self, model, params):
        self.apply_fn = model.apply
        self.params = params


def normalize_inputs(obss, mean, std):
    return (obss - mean) / (std + 1e-8)


# @jax.jit
@partial(jax.jit, static_argnames=['critic_model'])
def critic_inference(critic_model, params, obs):
    return critic_model.apply(params, obs)


class ValueFunctionManager:
    def __init__(self):
        #model_path = "components/safe_rl/value_function/models/VF_new_policies3f.pkl"#0.43 2 #e.pkl" 0.4 1
        #model_path = "components/safe_rl/value_function/models/VF_new_policies2e.pkl"
        #model_path = "components/safe_rl/value_function/models/VF_new2.pkl" #150N lateral push triggers backup
        model_path = os.environ['LOCOSIM_DIR']+"/robot_control/base_controllers/components/safe_rl/value_function/models/VF_new2b.pkl"  # 50N lateral push  triggers backup
        self.model = self.load_value(model_path)
        self.setup_value_function(self.model)
        print("Loaded model parameters.")
        self.count = 0
        self.min_switch = 1
        self.VF = True
        self.count_back = 0
        self.min_back = 200
        self.backup_trigger_counter = 0

    # Function to load value function
    def load_value(self, file_path):
        with open(file_path, 'rb') as f:
            return pickle.load(f)

    def setup_value_function(self, model):
        # Setup value function network
        self.critic_model = CriticNetwork()
        self.params, self.mean, self.std = model['model_params'], model['mean'], model['std']
        # Create object to evaluate the value function
        self.critic_network = CriticEvaluator(self.critic_model, self.params)

    def computeValueFnc(self,proj_gravity, body_ang_vel, joint_pos, joint_vel,   threshold, vf_additional_term):

        # dimension of observation vector = 3+3+12+12=30
        obs_flax_np = np.concatenate((
            proj_gravity.astype(np.float32),
            body_ang_vel.astype(np.float32),
            joint_pos.astype(np.float32),
            joint_vel.astype(np.float32)
        ))

        obs_flax = jnp.array(obs_flax_np)  # jax.numpy array

        # Normalize the observation
        obs_flax = normalize_inputs(obs_flax, self.mean, self.std)

        V_safe = critic_inference(self.critic_model, self.critic_network.params, obs_flax)
        #print(V_safe)
        #self.VF = True
        #return self.VF, V_safe
        if V_safe - vf_additional_term > threshold and self.VF:
            #  print(f"\033[92mV_safe: {V_safe:.4f}\033[0m")
            self.count = 0
            self.VF = True
        elif self.VF:
            self.count += 1
            print(f"\033[91mV_safe: {V_safe:.4f}\033[0m",self.backup_trigger_counter)
            if self.count >= self.min_switch:
                self.VF = False
                self.backup_trigger_counter+=1
            else:
                self.VF = True
        elif V_safe - vf_additional_term > threshold and not self.VF:
            self.count_back += 1
            #print('self.count_back',self.count_back)
            if self.count_back >= self.min_back:
                print('Back')
                self.VF =True
                self.count_back = 0
                self.count = 0
        elif not self.VF:
            self.count_back = 0

        return self.VF, V_safe