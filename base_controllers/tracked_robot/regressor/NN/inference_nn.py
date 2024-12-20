#!/usr/bin/env python
# coding: utf-8

# In[79]:


import joblib
import onnxruntime as ort
import os.path as osp
import numpy as np
import os

from sklearn.preprocessing import StandardScaler
main_dir = os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/regressor/NN/'
scalers_pth = main_dir+'scalers/'



wheel_l_scaler = joblib.load(osp.join(scalers_pth, 'wheel_r_scaler.jbl'))
wheel_r_scaler = joblib.load(osp.join(scalers_pth, 'wheel_r_scaler.jbl'))
roll_scaler = joblib.load(osp.join(scalers_pth, 'roll_scaler.jbl'))
pitch_scaler = joblib.load(osp.join(scalers_pth, 'pitch_scaler.jbl'))
yaw_scaler = joblib.load(osp.join(scalers_pth, 'yaw_scaler.jbl'))
beta_l_scaler = joblib.load(osp.join(scalers_pth, 'beta_l_scaler.jbl'))
beta_r_scaler = joblib.load(osp.join(scalers_pth, 'beta_r_scaler.jbl'))
alpha_scaler = joblib.load(osp.join(scalers_pth, 'alpha_scaler.jbl'))
onnx_model = ort.InferenceSession(osp.join(main_dir, 'slippage_model_onnx.onnx'))
x = np.array([-13.30434783,  13.53915392,  -0.06243814,   0.04164394, 21.00765 ])


x_ = np.zeros((1,5), np.float32)
x_[0][0] = wheel_l_scaler.transform(x[0].reshape(-1,1)).item()
x_[0][1] = wheel_r_scaler.transform(x[1].reshape(-1,1)).item()
x_[0][2] = roll_scaler.transform(x[2].reshape(-1,1)).item()
x_[0][3] = pitch_scaler.transform(x[3].reshape(-1,1)).item()
x_[0][4] = yaw_scaler.transform(x[4].reshape(-1,1)).item()


y = onnx_model.run(None, {'modelInput':x_})[0]


y_ = np.zeros((3), np.float32)
y_[0] = beta_l_scaler.inverse_transform(y[0][0].reshape(-1,1)).item()
y_[1] = beta_r_scaler.inverse_transform(y[0][1].reshape(-1,1)).item()
y_[2] = alpha_scaler.inverse_transform(y[0][2].reshape(-1,1)).item()


class SlipNN():
    def __init__(self, scalers_pth = main_dir+'scalers/', model_pth = main_dir+'slippage_model_onnx.onnx', output='alpha'):
        self.output = output
        self.feature_names_ = ['wheel_l', 'wheel_r', 'roll', 'pitch','yaw']
        self.wheel_l_scaler = joblib.load(osp.join(scalers_pth, 'wheel_r_scaler.jbl'))
        self.wheel_r_scaler = joblib.load(osp.join(scalers_pth, 'wheel_r_scaler.jbl'))
        self.roll_scaler = joblib.load(osp.join(scalers_pth, 'roll_scaler.jbl'))
        self.pitch_scaler = joblib.load(osp.join(scalers_pth, 'pitch_scaler.jbl'))
        self.yaw_scaler = joblib.load(osp.join(scalers_pth, 'yaw_scaler.jbl'))
        self.beta_l_scaler = joblib.load(osp.join(scalers_pth, 'beta_l_scaler.jbl'))
        self.beta_r_scaler = joblib.load(osp.join(scalers_pth, 'beta_r_scaler.jbl'))
        self.alpha_scaler = joblib.load(osp.join(scalers_pth, 'alpha_scaler.jbl'))
        self.onnx_model = ort.InferenceSession(model_pth)
        
    def predict(self, x):
        # pre-process the input with scalers
        x_ = np.zeros((1,5), np.float32)
        x_[0][0] = wheel_l_scaler.transform(x[0].reshape(-1,1)).item()
        x_[0][1] = wheel_r_scaler.transform(x[1].reshape(-1,1)).item()
        x_[0][2] = roll_scaler.transform(x[2].reshape(-1,1)).item()
        x_[0][3] = pitch_scaler.transform(x[3].reshape(-1,1)).item()
        x_[0][4] = yaw_scaler.transform(x[4].reshape(-1,1)).item()
        
        # run the model inference
        y = onnx_model.run(None, {'modelInput':x_})[0]
        
        # post-process the model output with scalers
        y_ = np.zeros((3), np.float32)
        y_[0] = beta_l_scaler.inverse_transform(y[0][0].reshape(-1,1)).item()
        y_[1] = beta_r_scaler.inverse_transform(y[0][1].reshape(-1,1)).item()
        y_[2] = alpha_scaler.inverse_transform(y[0][2].reshape(-1,1)).item()
        if self.output == 'beta_l':
            return y_[0]
        if self.output == 'beta_r':
            return y_[1]
        if self.output=='alpha':
            return y_[2]
        return y_

alpha_model = SlipNN(output='alpha')
beta_l_model = SlipNN(output='beta_l')
beta_r_model = SlipNN(output='beta_r')
input = np.array([5., 8., 0., 0.0, 0.])
print(f" friction_coeff: {0.4}")
print(f" alpha {alpha_model.predict(input)}, Beta_l {beta_l_model.predict(input)}, Beta_r {beta_r_model.predict(input)}")



