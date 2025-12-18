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
        model_path = "components/safe_rl/value_function/models/VF_backup_policy.pkl"
        self.model = self.load_value(model_path)
        self.setup_value_function(self.model)
        print("Loaded model parameters.")

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

    def computeValueFnc(self,proj_gravity, body_ang_vel, joint_pos, joint_vel,   threshold):

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

        if V_safe > threshold:
            #  print(f"\033[92mV_safe: {V_safe:.4f}\033[0m")
            return True, V_safe
        else:
            #  print(f"\033[91mV_safe: {V_safe:.4f}\033[0m")
            return False, V_safe