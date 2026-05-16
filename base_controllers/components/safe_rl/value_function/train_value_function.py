# ====================================
#         TD-Learning for Safety-V(x)
# ====================================

import os
import time
import copy
import math
import pickle
import numpy as np
from tqdm import tqdm

import jax
import jax.numpy as jnp
import flax.linen as nn
from flax.training import train_state
import optax

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(BASE_DIR)   # force working directory to match script directory
print("Working directory set to:", BASE_DIR)

# ====================================
#      Environment Configuration
# ====================================
os.environ["XLA_FLAGS"] = os.environ.get("XLA_FLAGS", "") + " --xla_gpu_triton_gemm_any=True"
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "True"

model_path = 'models/VF_no_backup.pkl'
load_parameters = False

# ====================================
#       Define Neural Network Entire model is in Flax, JAX's neural network library
# ====================================
class ValueNetwork(nn.Module):
    @nn.compact
    def __call__(self, x):
        # Activation is ELU for smoother gradients.
        x = nn.Dense(512)(x)
        x = nn.LayerNorm()(x)
        x = nn.elu(x)
        x = nn.Dense(256)(x)
        x = nn.LayerNorm()(x)
        x = nn.elu(x)
        x = nn.Dense(128)(x)
        x = nn.LayerNorm()(x)
        x = nn.elu(x)
        # Output initialized to 1 (probability of survival)
        x = nn.Dense(1, kernel_init=nn.initializers.zeros, bias_init=nn.initializers.ones)(x)
        # x = nn.Dense(1)(x)
        return x.squeeze(-1)


# ====================================
#           Loss Function
# ====================================
""" def loss_fn(params, batch_states, batch_next_states, batch_fallen, batch_cp, target_params):
    # loss_fn computes the TD-target and returns MSE loss:
    loss = (V(s) - target)^2
    target = (1 - fallen) * [ (1 - cp)*V(next_s) + cp ] 
"""
def loss_fn(params, batch_states, batch_next_states, batch_fallen, batch_cp, target_params):
    #forward pass
    #eval value function function of the state
    V_s = model.apply(params, batch_states)
    #compute the value estimate V(x′) of the next state
    V_next = jax.lax.stop_gradient(model.apply(target_params, batch_next_states))
    indicators = 1.0 - batch_fallen #0 if fallen  I do not continue
    #TD target computation (batch_cp: probability of capture now)
    target = indicators * ((1-batch_cp)*V_next + batch_cp)
    # jax.debug.print("target: {x}", x=target)
    # target = indicators * V_next
    #compute mean squared error
    return jnp.mean(jnp.square(V_s - target))

def fill_padding_with_last_valid(states, next_states, fallen, capt_p):
    states_filled = np.copy(states)
    next_states_filled = np.copy(next_states)
    fallen_filled = np.copy(fallen)
    capt_p_filled = np.copy(capt_p)

    num_episodes, T, _ = states.shape

    for ep in range(num_episodes):
        # Trova primo indice dove compare uno stato di solo zeri
        zero_mask = np.all(states[ep] == 0, axis=1)
        if not np.any(zero_mask):
            continue  # nessun padding, salta episodio

        first_zero_idx = np.argmax(zero_mask)

        # Prendi ultimo stato valido
        last_valid_state = states[ep, first_zero_idx - 1]

        for t in range(first_zero_idx, T):
            states_filled[ep, t] = last_valid_state
            next_states_filled[ep, t] = last_valid_state
            fallen_filled[ep, t] = 0
            capt_p_filled[ep, t] = 1.0

    return states_filled, next_states_filled, fallen_filled, capt_p_filled

@jax.jit
def train_step(state, batch_states, batch_next_states, batch_fallen, batch_cp, target_params):
    #loss = loss_fn(params, batch_states, batch_next_states, batch_fallen, batch_cp, target_params)
    loss, grads = jax.value_and_grad(loss_fn)(state.params, batch_states, batch_next_states, batch_fallen, batch_cp, target_params)
    # optimizer step to update params, Takes the gradients, Updates the parameters using the optimizer (params - learning_rate * Adam_update(grads))
    # replaces the old state with a new one containing the updated model (states.params)
    # Returns a new TrainState with updated values
    state = state.apply_gradients(grads=grads)
    return state, loss

@jax.jit
def update_target_params(target_params, params, tau):
    return jax.tree_util.tree_map(lambda t, p: tau * p + (1 - tau) * t, target_params, params)

# ====================================
#          Batch Sampling
# ====================================
def closest_power_of_2(n):
    return 2 ** (n.bit_length() - 1)

def get_batches(states, next_states, fallen, capt_p, batch_size, rng):
    batch_size = closest_power_of_2(batch_size)
    num_episodes, num_time_steps, _ = states.shape

    # Max number of episodes we can actually fit
    episodes_per_batch = min(num_episodes, math.ceil(batch_size / num_time_steps))

    # Ensure at least one batch
    num_batches = max(1, num_episodes // episodes_per_batch)

    rng, subkey = jax.random.split(rng)
    total_needed = num_batches * episodes_per_batch

    # If dataset is too small, repeat indices
    perm = jax.random.permutation(subkey, num_episodes)
    if total_needed > num_episodes:
        # Tile the permutation instead of causing slicing errors
        repeats = math.ceil(total_needed / num_episodes)
        perm = jnp.tile(perm, repeats)

    indices = perm[:total_needed].reshape(num_batches, episodes_per_batch)
    # extract(idx) produces a minibatch: s  → (batch_size, state_dim)
    def extract(idx):
        #(episodes_per_batch * T, D) then keeps only the first N samples to match batch size.
        s = states[idx].reshape(-1, states.shape[-1])[:batch_size]
        ns = next_states[idx].reshape(-1, next_states.shape[-1])[:batch_size]
        d = fallen[idx].reshape(-1)[:batch_size]
        cp = capt_p[idx].reshape(-1)[:batch_size]
        return s, ns, d, cp

    batches = jax.vmap(extract)(indices)
    return batches, rng


# ====================================
#         State Normalization
# ====================================
def normalize_states(states):
    mean = np.mean(states, axis=(0, 1))
    std = np.std(states, axis=(0, 1))
    return (states - mean) / (std + 1e-8), mean, std

# ====================================
#            Load Dataset
# ====================================
data_path = 'observation_datasets/observations_dataset_no_backup.npy'
data = np.load(data_path)
print("Dataset loaded:", data.shape)
input_dim = 30 # obst, obs_t+1, fallen, captured
states = data[:, :, :input_dim]
next_states = data[:, :, input_dim:2 * input_dim]
fallen = data[:, :, -2]
capt_p = data[:, :, -1]

states, next_states, fallen, capt_p = fill_padding_with_last_valid(states, next_states, fallen, capt_p)

""" # Esempio: trova gli indici degli episodi con almeno un 1
episode_with_fallen = np.where(np.any(fallen == 1, axis=1))[0]
print("Episodi con almeno un 'fallen':", episode_with_done)
exit() """

print("Observation dataset shape:", states.shape)
states, mean, std = normalize_states(states)
next_states, _, _ = normalize_states(next_states)

# ====================================
#         Convert to JAX tensors
# ====================================
states = jnp.array(states, dtype=jnp.float32)
next_states = jnp.array(next_states, dtype=jnp.float32)
fallen = jnp.array(fallen, dtype=jnp.float32)
capt_p = jnp.array(capt_p, dtype=jnp.float32)

# ====================================
#    Initialize / Load Model Params
# ====================================

if load_parameters:
    try:
        with open(model_path, 'rb') as f:
            loaded = pickle.load(f)
            params, mean, std = loaded['model_params'], loaded['mean'], loaded['std']
        print("Loaded model parameters.")
    except FileNotFoundError:
        print("Model file not found. Starting from scratch.")
else:
    key = jax.random.PRNGKey(42)  # seed random key for any operation involving randomness (weight initialization, random sampling, noise injection, etc.).
    # instantiate the model
    model = ValueNetwork()
    # In Flax, models are defined as pure functions and do not carry parameters inside them.
    # Parameters are stored outside the model.
    params = model.init(key, jnp.ones((1, input_dim)))  # runs the model once with the given input,but instead of producing outputs, it creates the network’s weights.
    # params It becomes a dictionary-like structure containing all initialized weights:

target_params = copy.deepcopy(params)

# ====================================
#       Optimizer & Train State
# ====================================
learning_rate = 3e-4
tau = 0.01
optimizer = optax.adam(learning_rate)

class TrainState(train_state.TrainState):
    pass

state = TrainState.create(apply_fn=model.apply, params=params, tx=optimizer)

# ====================================
#           Training Loop
# ====================================
epochs = 1000
batch_size = 512
rng = jax.random.PRNGKey(int(time.time()))
losses, min_losses, max_losses = [], [], []

with tqdm(range(epochs), desc="Epoch") as pbar:
    #for each training epoch
    for epoch in pbar:
        start_time = time.time()
        rng, subkey = jax.random.split(rng)
        #Sample batches
        batches, rng = get_batches(states, next_states, fallen, capt_p, batch_size, subkey)

        #Compute loss + gradients (JIT-compiled)
        epoch_loss = 0
        batch_losses = []
        # batches is a tuple of four big tensors, one per variable:
        # batches = (
        #     all_batch_states,  # shape: (num_batches, batch_size, dim)
        #     all_batch_next_states,  # shape: (num_batches, batch_size, dim)
        #     all_batch_fallen,  # shape: (num_batches, batch_size)
        #     all_batch_cp  # shape: (num_batches, batch_size)
        # )
        #Loop over minibatches, over every batch inside the epoch.
        for batch_states, batch_next_states, batch_fallen, batch_cp in zip(*batches):
            #produces one batch at a time: batch_states: shape (batch_size, dim)...
            # Perform one gradient update
            state, loss = train_step(state, batch_states, batch_next_states, batch_fallen, batch_cp, target_params)
            #Update target network, target_params = tau * new_params + (1 - tau) * old_target_params, This is Polyak averaging (soft target update), which stabilizes TD learning.
            target_params = update_target_params(target_params, state.params, tau)
            #Accumulate total epoch loss
            epoch_loss += loss
            batch_losses.append(loss)
        #Compute average epoch loss across minibatches
        number_of_minibatches=batches[0].shape[0]
        epoch_avg = epoch_loss / number_of_minibatches
        # Store training statistics
        losses.append(epoch_avg)
        #minimum batch loss
        min_losses.append(np.min(batch_losses))
        #maximum batch loss
        max_losses.append(np.max(batch_losses))
        #Update progress bar
        pbar.set_postfix({"Loss": f"{epoch_avg:.2e}", "Hz": f"{1 / (time.time() - start_time):.2f}"})

# ====================================
#           Save Model
# ====================================
with open(model_path, 'wb') as f:
    pickle.dump({'model_params': state.params, 'mean': mean, 'std': std}, f)
print("Model saved at", model_path)

# ====================================
#       Value Estimate Example
# ====================================
sample_state = states[5, 0, :]
v_est = model.apply(state.params, sample_state)
print(f"Estimated V(x) (prob. survival) for good state: {v_est.item():.4f}")

sample_state = states[239, 0, :]
v_est = model.apply(state.params, sample_state)
print(f"Estimated V(x) (prob. survival) for bad state: {v_est.item():.4f}")


# ====================================
#       Value Estimate Statistics
# ====================================

# Stimiamo V(x) per ogni primo stato di ogni episodio
all_v_estimates = jnp.array([model.apply(state.params, s[0]) for s in states])

# Identifica episodi che hanno avuto una terminazione
terminated_mask = jnp.any(fallen == 1, axis=1)
non_terminated_mask = ~terminated_mask

# Seleziona stime di V(x) per i due gruppi
v_est_terminated = all_v_estimates[terminated_mask]
v_est_non_terminated = all_v_estimates[non_terminated_mask]

# Calcola statistiche
mean_terminated = jnp.mean(v_est_terminated)
std_terminated = jnp.std(v_est_terminated)

mean_non_terminated = jnp.mean(v_est_non_terminated)
std_non_terminated = jnp.std(v_est_non_terminated)

# Stampa risultati
print("\n===== Value Estimate Statistics =====")
print(f"Episodes with termination   : n = {v_est_terminated.shape[0]}")
print(f"Mean V(x): {mean_terminated:.4f}, Std: {std_terminated:.4f}")
print()
print(f"Episodes without termination: n = {v_est_non_terminated.shape[0]}")
print(f"Mean V(x): {mean_non_terminated:.4f}, Std: {std_non_terminated:.4f}")

print("\n===== All V Estimates for Terminated Episodes =====")
print(v_est_terminated)
print("=======================================")
