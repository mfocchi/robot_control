import numpy as np
import onnxruntime as ort
import os
import json

# Utils function
def cart2sph(pos, threshold=1e-5):
    # Extract x, y, z components
    x = pos[:, 0]
    y = pos[:, 1]
    z = pos[:, 2]

    # Compute spherical coordinates
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(z, hxy)
    az = np.arctan2(y, x)

    # Concatenate azimuth, elevation, and radius
    spherical = np.stack((az, el, r), axis=1)

    return spherical

def sph2cart(pos):
    # Extract az, el, r components
    az = pos[:, 0]
    el = pos[:, 1]
    r = pos[:, 2]

    rcos_theta = r * np.cos(el)
    x = rcos_theta * np.cos(az)
    y = rcos_theta * np.sin(az)
    z = r * np.sin(el)

    # Concatenate x, y, and z
    cartesian = np.stack((x, y, z), axis=1)

    return cartesian

def quat_from_euler_xyz(roll, pitch, yaw):

        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        # compute quaternion
        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        return np.stack([qw, qx, qy, qz], axis=-1)

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class JumpAgent():
    def __init__(self, robot_name: str):

        base_model_path = os.path.join(os.environ.get('LOCOSIM_DIR'),
                                       'robot_control',
                                       'base_controllers',
                                       'jump_policy')
        
        config_path = os.path.join(base_model_path, f"{robot_name}.json")

        # Load configuration from JSON file
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found at: {config_path}")

        with open(config_path, 'r') as f:
            self.cfg = json.load(f)

        self.model_path = os.path.join(base_model_path, f'{robot_name}.onnx')
        print(f'JumpAgent, policy: {robot_name}')
        self.model = ort.InferenceSession(self.model_path)

        self.min_action = self.cfg["min_action"]
        self.max_action = self.cfg["max_action"]

        self.lerp_time = self.cfg["lerp_time"]

        self.t_th_min = self.cfg["t_th_min"]
        self.t_th_max = self.cfg["t_th_max"]

        self.x_theta_min = self.cfg["x_theta_min"]
        self.x_theta_max = self.cfg["x_theta_max"]

        self.x_r_min = self.cfg["x_r_min"]
        self.x_r_max = self.cfg["x_r_max"]

        self.xd_theta_min = self.cfg["xd_theta_min"]
        self.xd_theta_max = self.cfg["xd_theta_max"]

        self.xd_r_min = self.cfg["xd_r_min"]
        self.xd_r_max = self.cfg["xd_r_max"]

        self.psi_min = self.cfg["psi_min"]
        self.psi_max = self.cfg["psi_max"]

        self.theta_min = self.cfg["theta_min"]
        self.theta_max = self.cfg["theta_max"]

        self.phi_min = self.cfg["phi_min"]
        self.phi_max = self.cfg["phi_max"]

        self.psid_min = self.cfg["psid_min"]
        self.psid_max = self.cfg["psid_max"]

        self.thetad_min = self.cfg["thetad_min"]
        self.thetad_max = self.cfg["thetad_max"]

        self.phid_min = self.cfg["phid_min"]
        self.phid_max = self.cfg["phid_max"]

        self.xd_mult_min = self.cfg["xd_mult_min"]
        self.xd_mult_max = self.cfg["xd_mult_max"]

        self.l_expl_min = self.cfg["l_expl_min"]
        self.l_expl_max = self.cfg["l_expl_max"]

    def process_actions(self, actions, target):

        self.t_th_b = map_range(
            actions[..., 0], self.min_action, self.max_action, self.t_th_min, self.t_th_max)
        self.t_th_b = self.t_th_b.reshape(-1, 1)

        x_xd_phi = cart2sph(target[None])[:, 0].item()

        # Trunk x_lo_b
        x_theta = map_range(
            actions[..., 1], self.min_action, self.max_action, self.x_theta_min, self.x_theta_max)
        x_r = map_range(
            actions[..., 2], self.min_action, self.max_action, self.x_r_min, self.x_r_max)
        print(x_xd_phi, x_theta, x_r)
        self.trunk_x_lo_b = sph2cart(
            np.stack((x_xd_phi, x_theta, x_r))[None])

        # Trunk xd_lo_b
        xd_theta = map_range(
            actions[..., 3], self.min_action, self.max_action, self.xd_theta_min, self.xd_theta_max)
        xd_r = map_range(
            actions[..., 4], self.min_action, self.max_action, self.xd_r_min, self.xd_r_max)
        self.trunk_xd_lo_b = sph2cart(
            np.stack((x_xd_phi, xd_theta, xd_r))[None])

        # Trunk o_lo
        psi = map_range(
            actions[..., 5], self.min_action, self.max_action, self.psi_min, self.psi_max)
        theta = map_range(
            actions[..., 6], self.min_action, self.max_action, self.theta_min, self.theta_max)
        phi = map_range(
            actions[..., 7], self.min_action, self.max_action, self.phi_min, self.phi_max)

        self.trunk_o_lo = np.stack((psi, theta, phi))

        # Trunk od_lo
        psid = map_range(
            actions[..., 8], self.min_action, self.max_action, self.psid_min, self.psid_max)
        thetad = map_range(
            actions[..., 9], self.min_action, self.max_action, self.thetad_min, self.thetad_max)
        phid = map_range(
            actions[..., 10], self.min_action, self.max_action, self.phid_min, self.phid_max)

        self.trunk_od_lo = np.stack((psid, thetad, phid))

        xd_mult = map_range(
            actions[..., 11], self.min_action, self.max_action, self.xd_mult_min, self.xd_mult_max)
        l_expl = map_range(
            actions[..., 12], self.min_action, self.max_action, self.l_expl_min, self.l_expl_max)

        trunk_xd_lo_un = self.trunk_xd_lo_b / np.linalg.norm(self.trunk_xd_lo_b)
        self.trunk_x_lo_e = self.trunk_x_lo_b + trunk_xd_lo_un * l_expl.reshape(-1, 1)

        self.trunk_xd_lo_e = self.trunk_xd_lo_b * xd_mult.reshape(-1, 1)

        vf_n = np.linalg.norm(self.trunk_xd_lo_e)
        v0_n = np.linalg.norm(self.trunk_xd_lo_b)
        sf_n = np.linalg.norm(self.trunk_x_lo_e)
        s0_n = np.linalg.norm(self.trunk_x_lo_b)

        self.a = 0.5 * ((np.power(vf_n, 2) - np.power(v0_n, 2)) / ((sf_n - s0_n) + 1e-15))

        self.t_th_e = ((vf_n - v0_n) / (self.a + 1e-15)).reshape(-1, 1)
        self.t_th_total = self.t_th_b + self.t_th_e

        print(f"Processed action:\n\
                    t_th_b: {self.t_th_b}\n\
                    trunk_x_lo_b: {self.trunk_x_lo_b}\n\
                    trunk_xd_lo_b: {self.trunk_xd_lo_b}\n\
                    trunk_o_lo: {self.trunk_o_lo}\n\
                    trunk_od_lo: {self.trunk_od_lo}\n\
                    xd_mult: {xd_mult}\n\
                    l_expl: {l_expl}\n\
                    trunk_x_lo_e: {self.trunk_x_lo_e}\n\
                    trunk_xd_lo_e: {self.trunk_xd_lo_e}\n\
                    t_th_exp: {self.t_th_e}\n\
                    t_th_tot: {self.t_th_total}\n\
                    ")

    def act(self, _position, _orientation):
        # remove relative z
        position = _position.copy()
        orientation = _orientation.copy()

        quat_orientation = quat_from_euler_xyz(orientation[..., 0], orientation[..., 1], orientation[..., 2])

        # get processed target
        target = np.concatenate((position, quat_orientation))
        obs = target.astype(np.float32)[None]
        # run the model
        action = self.model.run(None, {'obs': obs})[0][0]

        # call the action processing
        self.process_actions(action, position)
