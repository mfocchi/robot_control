# %%
import pandas as pd
import numpy as np
import catboost as cb

from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score
import os
os.environ["MPLBACKEND"] = "TkAgg" #do it out side it export MPLBACKEND=TkAgg
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt #do as last import
import pandas as pd
from termcolor import colored

friction_coeff = 0.4 #file.split("_")[4]

list_file = [x for x in os.listdir('data3d') if (x.endswith('.csv') and 'ramp' in x and str(friction_coeff) in x )]
df_vpos = pd.DataFrame()

for file in list_file:
    print("reading...",file)
    tmp_df = pd.read_csv(os.path.join('data3d/',file),header=1, names=['time', 'wheel_l','wheel_r','roll', 'pitch', 'yaw', 'beta_l','beta_r','alpha'])
    v = (tmp_df['wheel_l'].values + tmp_df['wheel_r'].values) / 2. * 0.0856
    idx_filter = v > 0.01
    df_vpos = pd.concat([df_vpos, tmp_df[idx_filter]], ignore_index=True)

x = df_vpos[['wheel_l','wheel_r', 'roll', 'pitch', 'yaw']].values
y = df_vpos[['beta_l','beta_r','alpha']].values
# wheel_l = df_vpos.wheel_l.values
# wheel_r = df_vpos.wheel_r.values
# roll = df_vpos.roll.values
# pitch = df_vpos.pitch.values
# beta_l = df_vpos.beta_l.values
# beta_r = df_vpos.beta_r.values
# alpha = df_vpos.alpha.values

# %% spit dataset in train and valid+test set (10%)
x_train, x_valid, y_train, y_valid = train_test_split(x, y, random_state=13, test_size=0.2)

# further split validation 60% + test 40%
x_valid, x_test, y_valid, y_test = train_test_split(x_valid, y_valid, random_state=13, test_size=0.4)


# # %% create model of regressor Beta_l
print(colored("Training beta_l","red"))
model_beta_l = cb.CatBoostRegressor(max_depth=15)#learning_rate=1e-2, max_depth=15)
model_beta_l.fit(x_train, y_train[..., 0].reshape(-1, 1), verbose=True,eval_set=(x_valid, y_valid[..., 0].reshape(-1, 1)), use_best_model=True)
preds_train_beta_l = model_beta_l.predict(x_train)
preds_beta_l = model_beta_l.predict(x_test)
print(f'R2 metric train beta_l: {r2_score(y_train[...,0], preds_train_beta_l)}')
print(f'R2 metric test beta_l: {r2_score(y_test[...,0], preds_beta_l)}')
model_name_beta_l = 'model_beta_l'+str(friction_coeff)+'.cb'
model_beta_l.save_model(model_name_beta_l)

# # %% create model of regressor Beta_r
print(colored("Training beta_r","red"))
model_beta_r = cb.CatBoostRegressor(max_depth=15)#learning_rate=1e-2, max_depth=15)
model_beta_r.fit(x_train, y_train[..., 1].reshape(-1, 1), verbose=True,
                 eval_set=(x_valid, y_valid[..., 1].reshape(-1, 1)), use_best_model=True)
preds_train_beta_r = model_beta_r.predict(x_train)
preds_beta_r = model_beta_r.predict(x_test)
print(f'R2 metric train beta_r: {r2_score(y_train[...,1], preds_train_beta_r)}')
print(f'R2 metric test beta_r: {r2_score(y_test[...,1], preds_beta_r)}')
model_name_beta_r = 'model_beta_r'+str(friction_coeff)+'.cb'
model_beta_r.save_model(model_name_beta_r)

# regressor for Alpha
print(colored("Training Alpha","red"))
model_alpha = cb.CatBoostRegressor(max_depth=15)#iterations=10000)
model_alpha.fit(x_train, y_train[..., 2].reshape(-1, 1), verbose=True,
                eval_set=(x_valid, y_valid[..., 2].reshape(-1, 1)), use_best_model=True)
preds_train_alpha = model_alpha.predict(x_train)
preds_alpha = model_alpha.predict(x_test)
print(f'R2 metric train alpha: {r2_score(y_train[...,2], preds_train_alpha)}')
print(f'R2 metric test alpha: {r2_score(y_test[...,2], preds_alpha)}')
# %% save the model python
model_name_alpha = 'model_alpha'+str(friction_coeff)+'.cb'
model_alpha.save_model(model_name_alpha)

#################################################
# 2D plots of quality of results
fig, ax = plt.subplots(1, 3, figsize=(20, 7))
ax[0].set_ylabel("Predicted")
ax[0].set_xlabel("True")
ax[0].scatter(y_test[..., 0], preds_beta_l,  color="blue", label="test", alpha=0.5)
ax[0].scatter(y_train[..., 0], preds_train_beta_l,   color="red", label="train", alpha=0.5)
ax[0].plot([y[..., 0].min(), y[..., 0].max()], [y[..., 0].min(), y[..., 0].max()], color="black")
ax[0].set_title('beta_l')
ax[0].legend()

ax[1].set_ylabel("Predicted")
ax[1].set_xlabel("True")
ax[1].scatter(y_test[..., 1], preds_beta_r, color="blue", alpha=0.5)
ax[1].scatter(y_train[..., 1], preds_train_beta_r, color="red", alpha=0.5)
ax[1].plot([y[..., 1].min(), y[..., 1].max()], [y[..., 1].min(), y[..., 1].max()], color="black")
ax[1].set_title('beta_r')

ax[2].set_ylabel("Predicted")
ax[1].set_xlabel("True")
ax[2].scatter(y_test[..., 2], preds_alpha, color="blue", alpha=0.5)
ax[2].scatter(y_train[..., 2], preds_train_alpha, color="red", alpha=0.5)
ax[2].plot([y[..., 2].min(), y[..., 2].max()], [y[..., 2].min(), y[..., 2].max()], color="black")
ax[2].set_title('alpha')
plt.show()
#
# ##########################################
# #### 3D plots (NON SENSE)
# from mpl_toolkits import mplot3d
# fig = plt.figure(figsize=(10, 10))
# ax = plt.axes(projection='3d')
# ax.scatter(x[..., 0], x[..., 1], y[..., 0], label='groundtruth',  color='blue')
# ax.scatter(x_train[..., 0], x_train[..., 1], preds_train_beta_l.reshape(-1, 1), label='predicted train',color='red')
# ax.scatter(x_test[..., 0], x_test[..., 1], preds_beta_l.reshape(-1, 1),label='predicted test', color='green')
# ax.set_xlabel('wheel_l')
# ax.set_ylabel('wheel_r')
# ax.set_zlabel("beta_l")
# ax.legend()
# plt.show()
#
# fig = plt.figure(figsize=(10, 10))
# ax = plt.axes(projection='3d')
# ax.scatter(x[..., 0], x[..., 1], y[..., 1], label='groundtruth', color='blue')
# ax.scatter(x_train[..., 0], x_train[..., 1], preds_train_beta_r.reshape(-1,1), label='predicted train', color='red')
# ax.scatter(x_test[..., 0], x_test[..., 1], preds_beta_r.reshape(-1,1),label='predicted test', color='green')
# ax.set_xlabel('wheel_l')
# ax.set_ylabel('wheel_r')
# ax.set_zlabel("beta_r")
# ax.legend()
# plt.show()
#
# fig = plt.figure(figsize=(10, 10))
# ax = plt.axes(projection='3d')
# ax.scatter(x[..., 0], x[..., 1], y[..., 2],label='groundtruth', color='blue')
# ax.scatter(x_train[..., 0], x_train[..., 1], preds_train_alpha.reshape(-1,1), label='predicted train',color='red')
# ax.scatter(x_test[..., 0], x_test[..., 1], preds_alpha.reshape(-1,1), label='predicted test', color='green')
# ax.set_xlabel('wheel_l')
# ax.set_ylabel('wheel_r')
# ax.set_zlabel("alpha")
# ax.legend()
# plt.show()
#


# # To test
# ### Load Model
import numpy as np
import catboost as cb
model_beta_l = cb.CatBoostRegressor()
model_beta_l.load_model(model_name_beta_l)
beta_l = model_beta_l.predict(np.array([-3.4,3.4,0.,0.1,0]))

model_beta_r = cb.CatBoostRegressor()
model_beta_r.load_model(model_name_beta_r)
beta_r =  model_beta_r.predict(np.array([-3.4,3.4, 0.,0.1, 0]))

model_alpha = cb.CatBoostRegressor()
model_alpha.load_model(model_name_alpha)
alpha = model_alpha.predict(np.array([-3.4,3.4, 0.,0.1,0]))

print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")
