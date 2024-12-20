#!/usr/bin/env python
# coding: utf-8

# In[10]:


import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import os.path as osp
import joblib

import torch
import torch.nn as nn
import lightning as L

from torch.utils.data import Dataset, DataLoader
from lightning.pytorch.callbacks import EarlyStopping, ModelCheckpoint
from lightning.pytorch.loggers import TensorBoardLogger
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score


# In[11]:


files_path = osp.join('data2')


# In[12]:


files_list = [f for f in os.listdir(files_path) if (f.endswith('.csv') and ('ramp' in f))]
len(files_list)


# In[13]:


v_thresh = 0.01


# In[14]:


df = pd.DataFrame()

for f in files_list:
    tmp_df = pd.read_csv(osp.join(files_path, f), header=0)
    v = (tmp_df['wheel_l'].values + tmp_df['wheel_r'].values) / 2. * 0.0856
    df = pd.concat([df, tmp_df[v > v_thresh]], ignore_index=True)


# In[15]:


df


# In[16]:


data = df[['wheel_l','wheel_r', 'roll', 'pitch', 'yaw','beta_l','beta_r','alpha']].values




split_size=0.3
rnd_state=13


# In[19]:


train_ix, valid_ix = train_test_split(np.arange(len(df)), test_size=split_size, random_state=rnd_state)
valid_ix, test_ix = train_test_split(valid_ix, test_size=split_size, random_state=rnd_state)

print(f"train: {len(train_ix)}, validation: {len(valid_ix)}, test: {len(test_ix)}")


# In[20]:


wheel_l_scaler = StandardScaler(copy=False)
wheel_l_scaler.fit(data[train_ix, 0].reshape(-1, 1))
wheel_l_scaler.transform(data[..., 0].reshape(-1, 1))

wheel_l_scaler.copy = True
joblib.dump(wheel_l_scaler, osp.join('scalers', 'wheel_l_scaler.jbl'))


# In[21]:


wheel_r_scaler = StandardScaler(copy=False)
wheel_r_scaler.fit(data[train_ix, 1].reshape(-1, 1))
wheel_r_scaler.transform(data[..., 1].reshape(-1, 1))

wheel_r_scaler.copy = True
joblib.dump(wheel_r_scaler, osp.join('scalers', 'wheel_r_scaler.jbl'))


# In[22]:


roll_scaler = StandardScaler(copy=False)
roll_scaler.fit(data[train_ix, 2].reshape(-1, 1))
roll_scaler.transform(data[..., 2].reshape(-1, 1))

roll_scaler.copy = True
joblib.dump(roll_scaler, osp.join('scalers', 'roll_scaler.jbl'))


# In[23]:


pitch_scaler = StandardScaler(copy=False)
pitch_scaler.fit(data[train_ix, 3].reshape(-1, 1))
pitch_scaler.transform(data[..., 3].reshape(-1, 1))

pitch_scaler.copy = True
joblib.dump(pitch_scaler, osp.join('scalers', 'pitch_scaler.jbl'))


# In[24]:


yaw_scaler = StandardScaler(copy=False)
yaw_scaler.fit(data[train_ix, 4].reshape(-1, 1))
yaw_scaler.transform(data[..., 4].reshape(-1, 1))

yaw_scaler.copy = True
joblib.dump(yaw_scaler, osp.join('scalers', 'yaw_scaler.jbl'))


# In[25]:


beta_l_scaler = StandardScaler(copy=False)
beta_l_scaler.fit(data[train_ix, 5].reshape(-1, 1))
beta_l_scaler.transform(data[..., 5].reshape(-1, 1))

beta_l_scaler.copy = True
joblib.dump(beta_l_scaler, osp.join('scalers', 'beta_l_scaler.jbl'))


# In[26]:


beta_r_scaler = StandardScaler(copy=False)
beta_r_scaler.fit(data[train_ix, 6].reshape(-1, 1))
beta_r_scaler.transform(data[..., 6].reshape(-1, 1))

beta_r_scaler.copy = True
joblib.dump(beta_r_scaler, osp.join('scalers', 'beta_r_scaler.jbl'))


# In[27]:


alpha_scaler = StandardScaler(copy=False)
alpha_scaler.fit(data[train_ix, 7].reshape(-1, 1))
alpha_scaler.transform(data[..., 7].reshape(-1, 1))

alpha_scaler.copy = True
joblib.dump(alpha_scaler, osp.join('scalers', 'alpha_scaler.jbl'))


# In[28]:


class SlipDataset(Dataset):
    def __init__(self, data):
        self.data = torch.tensor(data).to(torch.float32)

    def __len__(self):
        return self.data.shape[0]
    
    def __getitem__(self, ix):
        return self.data[ix]


# In[31]:




train_dts = SlipDataset(data[train_ix])
valid_dts = SlipDataset(data[valid_ix])
test_dts = SlipDataset(data[test_ix])


# In[30]:


train_dl = DataLoader(train_dts, batch_size=512, num_workers=3)
valid_dl = DataLoader(valid_dts, batch_size=512, num_workers=3)
test_dl = DataLoader(test_dts, batch_size=512,num_workers=3)


# In[21]:


class SlipNN(L.LightningModule):
    def __init__(self, input_dim=4, output_dim=3, layer_mul=2, lr=1e-4):
        super().__init__()
        self.save_hyperparameters()
        self.lr = lr
        self.input_dim = input_dim
        self.output_dim = output_dim

        self.model = nn.Sequential(
            nn.Linear(input_dim, 64 * layer_mul),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(64 * layer_mul, 128 * layer_mul),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(128 * layer_mul, 64 * layer_mul),
            nn.ReLU(),
            nn.Linear(64 * layer_mul, output_dim)
        )

    def forward(self, x):
        y = self.model(x)
        return y

    def training_step(self, batch, batch_ix):
        x = batch[..., :self.input_dim]
        y = batch[..., -self.output_dim:]

        out = self(x)

        loss = nn.functional.mse_loss(out, y)
        self.log('train_loss', loss.item(), on_step=False, on_epoch=True)

        return loss

    def validation_step(self, batch, batch_ix):
        x = batch[..., :self.input_dim]
        y = batch[..., -self.output_dim:]

        out = self(x)

        loss = nn.functional.mse_loss(out, y)
        self.log('valid_loss', loss.item(), on_step=False, on_epoch=True)

        return loss

    def test_step(self, batch, batch_ix):
        x = batch[..., :self.input_dim]
        y = batch[..., -self.output_dim:]

        out = self(x)

        loss = nn.functional.mse_loss(out, y)
        self.log('test_loss', loss.item(), on_step=False, on_epoch=True)

        return loss

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(
            self.parameters(), lr=self.lr, weight_decay=1e-6)
        return optimizer


# In[22]:


model = SlipNN(input_dim=5, output_dim=3, layer_mul=2)
model


# In[23]:


trainer = L.Trainer(accelerator='cpu', callbacks=[EarlyStopping(monitor='valid_loss', min_delta=1e-5, patience=5, mode='min'),
                                                  ModelCheckpoint(dirpath='checkpoints', save_top_k=1, monitor='valid_loss')],
                    log_every_n_steps=5,
                    logger=TensorBoardLogger("tb_logs", name="ValueFN"))


# In[24]:


trainer.fit(model=model, train_dataloaders=train_dl, val_dataloaders=valid_dl)


# In[25]:


trainer.test(model=model, dataloaders=test_dl)


# In[27]:


true_y = np.array([])
predicted = np.array([])

torch.set_grad_enabled(False)
model.eval()

with torch.no_grad():
    for batch in test_dl:
        x = batch[..., :5]
        y = batch[..., -3:]
        out = model(x)

        true_y = np.concatenate((true_y,y.flatten()))
        predicted = np.concatenate((predicted,out.flatten()))


# In[28]:


true_y = true_y.reshape(-1,3)
predicted = predicted.reshape(-1,3)


# In[29]:


plt.figure(figsize=(5,5))
plt.scatter(true_y[...,0], predicted[...,0])
plt.plot([true_y[..., 0].min(), true_y[..., 0].max()], [true_y[..., 0].min(), true_y[..., 0].max()], color="black", linestyle='--')
plt.show()


# In[30]:


plt.figure(figsize=(5,5))
plt.scatter(true_y[...,1], predicted[...,1])
plt.plot([true_y[..., 1].min(), true_y[..., 1].max()], [true_y[..., 1].min(), true_y[..., 1].max()], color="black", linestyle='--')
plt.show()


# In[31]:


plt.figure(figsize=(5,5))
plt.scatter(true_y[...,2], predicted[...,2])
plt.plot([true_y[..., 2].min(), true_y[..., 2].max()], [true_y[..., 2].min(), true_y[..., 2].max()], color="black", linestyle='--')
plt.show()


# In[33]:


print(f'R2 metric test beta_l: {r2_score(true_y[...,0], predicted[...,0])}')
print(f'R2 metric test beta_r: {r2_score(true_y[...,1], predicted[...,1])}')
print(f'R2 metric test alpha: {r2_score(true_y[...,2], predicted[...,2])}')


# In[34]:


torch.__version__


# In[43]:


torch_input = torch.randn(1, 5)
onnx_program = torch.onnx.dynamo_export(model, torch_input)


# In[44]:


onnx_program.save("slippage_model.onnx")


# In[86]:


torch.onnx.export(model,torch_input,"slippage_model_onnx.onnx",input_names=["modelInput"],output_names=["modelOutput"],)


# In[45]:


wheel_l_scaler.scale_, wheel_l_scaler.mean_


# In[46]:


wheel_r_scaler.scale_, wheel_r_scaler.mean_


# In[47]:


roll_scaler.scale_, roll_scaler.mean_


# In[48]:


pitch_scaler.scale_, pitch_scaler.mean_


# In[49]:


yaw_scaler.scale_, yaw_scaler.mean_


# In[50]:


beta_l_scaler.scale_, beta_l_scaler.mean_


# In[51]:


beta_r_scaler.scale_, beta_r_scaler.mean_


# In[52]:


alpha_scaler.scale_, alpha_scaler.mean_


# In[73]:


# SCALING
x = 0.5


# In[74]:


wheel_l_scaler.copy = True


# In[75]:


x_scaled = x
x_scaled -= wheel_l_scaler.mean_
x_scaled /= wheel_l_scaler.scale_


# In[76]:


wheel_l_scaler.transform([[x]]), x_scaled


# In[77]:


# INVERSE SCALING
x_inv_scale = x_scaled.copy()
x_inv_scale *= wheel_l_scaler.scale_
x_inv_scale += wheel_l_scaler.mean_


# In[78]:


x_inv_scale, wheel_l_scaler.inverse_transform([x_scaled])


# In[87]:


import onnxruntime as ort


# In[88]:


onnx_model = ort.InferenceSession('slippage_model_onnx.onnx')


# In[105]:


x = data[0][:5][None].astype(np.float32)
x


# In[106]:


onnx_model.run(None, {'modelInput':x})


# In[115]:


model(torch.tensor([data[0][:5]]).to(torch.float32))

