import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

robot = 'aliengo'  # 'aliengo', 'go1', 'go2', 'b2', 'hyqreal1', 'hyqreal2', 'mini_cheetah' 

# ----------------------------------------------------------------------------------------------------------------
Kp_walking = 21.5
Kd_walking = 3.5

policy_folder_path = dir_path + "/policies/aliengo_state_est"
cuncurrent_state_est_network = policy_folder_path + "/exported/cuncurrent_state_estimator.pth"
rma_network = policy_folder_path + "/exported/rma.pth"

# Load specific training parameters
import yaml 
with open(policy_folder_path + "/params/env.yaml", "r") as file:
    training_env = yaml.unsafe_load(file)
