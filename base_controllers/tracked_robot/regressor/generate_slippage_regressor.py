# %%
import pandas as pd
import numpy as np
import catboost as cb
from scipy.interpolate import RBFInterpolator
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score
import os
from termcolor import colored

os.environ["MPLBACKEND"] = "TkAgg" #do it out side it exports MPLBACKEND=TkAgg
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt #do as last import

#old way
sim = True
if sim:
    # only data long_v>0
    #data = 'data2d/csv/ident_wheels_sim_0.1_long_v_positive_WLmax_10.csv'
    # Using this with 0.4
    data = 'data2d/csv/ident_wheels_sim_0.4_long_v_positive_WLmax_18.csv'
    friction_coeff = data.split("_")[3]
else:
    data = 'data2d/csv/ident_wheels_real_indoor.csv'

print(colored("remember to run in interactive mode otherwise the plots do now show up!","red"))
df = pd.read_csv(data,header=None, names=['wheel_l','wheel_r','beta_l','beta_r','alpha'])
x = df[['wheel_l','wheel_r']].values
y = df[['beta_l','beta_r','alpha']].values

# new way
# friction_coeff = '0.1'
#
# if friction_coeff == '0.1':
#     WLmax = 10
# elif friction_coeff == '0.4':
#     WLmax = 18
# else:
#     print("wrong friction coeff")
#
# list_file = [x for x in os.listdir('data2d') if (x.endswith('.csv') )]
#
# # Initialize logs
# des_wheel_l_log = []
# des_wheel_r_log = []
# beta_l_log = []
# beta_r_log = []
# alpha_log = []
#
# for file in list_file:
#     print("reading...",file)
#     tmp_df = pd.read_csv(os.path.join('data2d/',file),header=1, names=['time', 'wheel_l','wheel_r','roll', 'pitch', 'yaw', 'beta_l','beta_r','alpha'])
#
#     des_wheel_R = tmp_df['wheel_r']
#     des_wheel_L = tmp_df['wheel_l']
#     alpha = tmp_df['alpha']
#     beta_l = tmp_df['beta_l']
#     beta_r = tmp_df['beta_r']
#
#     # Get switch indices where des_wheel_R changes
#     switch_idx = np.where(np.diff(des_wheel_R) != 0)[0] + 1  # +1 to match MATLAB's diff behavior
#     switch_idx = np.concatenate(([0], switch_idx))  # prepend 0 (equivalent to 1 in MATLAB)
#
#     number_of_samples_per_batch = np.diff(switch_idx)
#     margin = round(number_of_samples_per_batch[0] * 0.2)
#
#     number_of_samples = len(switch_idx) - 1
#
#     # Preallocate "steady" arrays as lists of np arrays
#     beta_l_steady = [None] * number_of_samples
#     beta_r_steady = [None] * number_of_samples
#     alpha_steady = [None] * number_of_samples
#     des_wheel_L_steady = [None] * number_of_samples
#     des_wheel_R_steady = [None] * number_of_samples
#
#     for idx in range(number_of_samples):
#         start = switch_idx[idx] + margin
#         end = switch_idx[idx + 1] - margin
#
#         beta_l_steady[idx] = beta_l[start:end]
#         beta_r_steady[idx] = beta_r[start:end]
#         alpha_steady[idx] = alpha[start:end]
#         des_wheel_L_steady[idx] = des_wheel_L[start:end]
#         des_wheel_R_steady[idx] = des_wheel_R[start:end]
#
#     beta_l_vec = []
#     beta_r_vec = []
#     avg_alpha_vec = []
#     des_wheel_L_vec = []
#     des_wheel_R_vec = []
#
#     for i in range(number_of_samples):
#         beta_l_vec.append(np.mean(beta_l_steady[i]))
#         beta_r_vec.append(np.mean(beta_r_steady[i]))
#         avg_alpha_vec.append(np.mean(alpha_steady[i]))
#         des_wheel_L_vec.append(np.mean(des_wheel_L_steady[i]))
#         des_wheel_R_vec.append(np.mean(des_wheel_R_steady[i]))
#
#     # Append to logs
#     des_wheel_l_log.extend(des_wheel_L_vec)
#     des_wheel_r_log.extend(des_wheel_R_vec)
#     beta_l_log.extend(beta_l_vec)
#     beta_r_log.extend(beta_r_vec)
#     alpha_log.extend(avg_alpha_vec)
#
# # Convert logs to arrays
# wheel_l = np.array(des_wheel_l_log)
# wheel_r = np.array(des_wheel_r_log)
# beta_l = np.array(beta_l_log)      # assuming you meant beta_l_log here
# beta_r = np.array(beta_r_log)
# alpha = np.array(alpha_log)    # assuming you meant avg_alpha_log here
#
# idx_filter = (np.abs(wheel_l) <= WLmax) &  (np.abs(wheel_r) <= WLmax)
# x = np.column_stack((wheel_l[idx_filter], wheel_r[idx_filter]))
# y = np.column_stack((beta_l[idx_filter], beta_r[idx_filter], alpha[idx_filter]))
# df = pd.DataFrame(np.hstack([x, y]), columns=['wheel_l', 'wheel_r', 'beta_l', 'beta_r', 'alpha'])
#
# #save to be used in PAPER
# df.to_csv('data2d/csv/ident_wheels_sim_'+friction_coeff+'_WLmax_'+str(WLmax)+'.csv', index=False, header=False)
#
# # consider only sample with positive velocity
# v = (wheel_l + wheel_r) / 2.0 * 0.0856
# # Apply filter
# idx_filter = (v > 0.02)  & (np.abs(wheel_l) <= WLmax) &  (np.abs(wheel_r) <= WLmax)
# # Create input/output arrays
# x = np.column_stack((wheel_l[idx_filter], wheel_r[idx_filter]))
# y = np.column_stack((beta_l[idx_filter], beta_r[idx_filter], alpha[idx_filter]))
#
# #save to be used directly for PAPER
# df = pd.DataFrame(np.hstack([x, y]), columns=['wheel_l', 'wheel_r', 'beta_l', 'beta_r', 'alpha'])
# df.to_csv('data2d/csv/ident_wheels_sim_'+friction_coeff+'_long_v_positive_WLmax_'+str(WLmax)+'.csv', index=False, header=False)


#########################################

# #upsampling
# #Fit an interpolator for each output dimension
interpolator_beta_l = RBFInterpolator(x, y[:, 0], smoothing=0.1)
interpolator_beta_r = RBFInterpolator(x, y[:, 1], smoothing=0.1)
interpolator_alpha  = RBFInterpolator(x, y[:, 2], smoothing=0.1)
#
# Create a dense grid of inputs
wl_dense = np.linspace(-10, 10, 200)
wr_dense = np.linspace(-10, 10, 200)
wl_mesh, wr_mesh = np.meshgrid(wl_dense, wr_dense)
x_dense = np.stack([wl_mesh.ravel(), wr_mesh.ravel()], axis=1)

# Interpolate new outputs
beta_l_dense = interpolator_beta_l(x_dense)
beta_r_dense = interpolator_beta_r(x_dense)
alpha_dense  = interpolator_alpha(x_dense)
y_dense = np.stack([beta_l_dense, beta_r_dense, alpha_dense], axis=1)
data_dense = np.hstack([x_dense, y_dense])
#
# # Create DataFrame with proper column names
df_dense = pd.DataFrame(data_dense, columns=['wheel_l', 'wheel_r', 'beta_l', 'beta_r', 'alpha'])

v = (df_dense.wheel_l + df_dense.wheel_r) / 2 * 0.0856
idx_filter = v > 0.01
v_pos_data = df_dense[idx_filter]

x = v_pos_data[['wheel_l','wheel_r']].values
y = v_pos_data[['beta_l','beta_r','alpha']].values

# %%compute input correlation
df_dense.corr()

# %% plot histogram to see if input distribution is well behaved, to see if it is neeeded a scaling
fig, ax = plt.subplots(1, 5, figsize=(20, 4))
ax[0].hist(x[..., 0])
ax[0].set_title('wheel_l')
ax[1].hist(x[..., 1])
ax[1].set_title('wheel_r')
# %% plot histogram to see if output distribution is well behaved, to see if it is neeeded a scaling
ax[2].hist(y[..., 0])
ax[2].set_title('beta_l')
ax[3].hist(y[..., 1])
ax[3].set_title('beta_r')
ax[4].hist(y[..., 2])
ax[4].set_title('alpha')
plt.show()

# %% spit dataset in train and valid+test set (10%)
x_train, x_valid, y_train, y_valid = train_test_split(
    x, y, random_state=13, test_size=0.2)

# further split validation 60% + test 40%
x_valid, x_test, y_valid, y_test = train_test_split(
    x_valid, y_valid, random_state=13, test_size=0.4)

len(x_train), len(x_valid), len(x_test)
len(y_train), len(y_valid), len(y_test)

# %% [markdown]
# # %% create model of regressor Beta_l
model_beta_l = cb.CatBoostRegressor()
# %% train the model
model_beta_l.fit(x_train, y_train[..., 0].reshape(-1, 1), verbose=100,
                 eval_set=(x_valid, y_valid[..., 0].reshape(-1, 1)), use_best_model=True)
preds_train_beta_l = model_beta_l.predict(x_train)
# %%
preds_beta_l = model_beta_l.predict(x_test)

# %%
print(f'R2 metric train beta_l: {r2_score(y_train[...,0], preds_train_beta_l)}')
print(f'R2 metric test beta_l: {r2_score(y_test[...,0], preds_beta_l)}')


# %% save the model python
model_name_beta_l = 'model_beta_l'+friction_coeff+'.cb'
model_beta_l.save_model(model_name_beta_l)
# model_name_beta_l = 'model_beta_l'+friction_coeff+'.cbm'
# model_beta_l.save_model(model_name_beta_l,format="cbm")
# # Save model to ONNX-ML format
# model_beta_l.save_model(
#     'model_beta_l'+friction_coeff+'.onnx',
#     format="onnx",
#     export_parameters={
#         'onnx_domain': 'ai.catboost',
#         'onnx_model_version': 1,
#         'onnx_doc_string': 'test model for Regression',
#         'onnx_graph_name': 'CatBoostModel_for_Regression'
#     }
# )

# %% [markdown]
# # %% create model of regressor Beta_r
model_beta_r = cb.CatBoostRegressor()
model_beta_r.fit(x_train, y_train[..., 1].reshape(-1, 1), verbose=100,
                 eval_set=(x_valid, y_valid[..., 1].reshape(-1, 1)), use_best_model=True)
preds_train_beta_r = model_beta_r.predict(x_train)
preds_beta_r = model_beta_r.predict(x_test)

print(f'R2 metric train beta_r: {r2_score(y_train[...,1], preds_train_beta_r)}')
print(f'R2 metric test beta_r: {r2_score(y_test[...,1], preds_beta_r)}')

# %% save the model python
model_name_beta_r = 'model_beta_r'+friction_coeff+'.cb'
model_beta_r.save_model(model_name_beta_r)
# %% save the model cpp binary
# model_name_beta_r = 'model_beta_r'+friction_coeff+'.cbm'
# model_beta_r.save_model(model_name_beta_r,format="cbm")
# # Save model to ONNX-ML format
# model_beta_r.save_model(
#     'model_beta_r'+friction_coeff+'.onnx',
#     format="onnx",
#     export_parameters={
#         'onnx_domain': 'ai.catboost',
#         'onnx_model_version': 1,
#         'onnx_doc_string': 'test model for Regression',
#         'onnx_graph_name': 'CatBoostModel_for_Regression'
#     }
# )

# regressor for Alpha (cannot
model_alpha = cb.CatBoostRegressor()

model_alpha.fit(x_train, y_train[..., 2].reshape(-1, 1), verbose=100,
                eval_set=(x_valid, y_valid[..., 2].reshape(-1, 1)), use_best_model=True)

preds_train_alpha = model_alpha.predict(x_train)
preds_alpha = model_alpha.predict(x_test)

# %%
print(f'R2 metric train alpha: {r2_score(y_train[...,2], preds_train_alpha)}')
print(f'R2 metric test alpha: {r2_score(y_test[...,2], preds_alpha)}')

# %%
# %% save the model python
model_name_alpha = 'model_alpha'+friction_coeff+'.cb'
model_alpha.save_model(model_name_alpha)
# %% save the model cpp binary
# model_name_alpha = 'model_alpha'+friction_coeff+'.cbm'
# model_alpha.save_model(model_name_alpha,format="cbm")
# Save model to ONNX-ML format
# model_alpha.save_model(
#     'model_alpha.onnx',
#     format="onnx",
#     export_parameters={
#         'onnx_domain': 'ai.catboost',
#         'onnx_model_version': 1,
#         'onnx_doc_string': 'test model for Regression',
#         'onnx_graph_name': 'CatBoostModel_for_Regression'
#     }
# )


# 2D plots of quality of results
fig, ax = plt.subplots(1, 3, figsize=(20, 7))

ax[0].set_ylabel("Predicted")
ax[0].set_xlabel("True")
ax[0].scatter(y_test[..., 0], preds_beta_l,
              color="blue", label="test", alpha=0.5)
ax[0].scatter(y_train[..., 0], preds_train_beta_l,
              color="red", label="train", alpha=0.5)
ax[0].plot([y[..., 0].min(), y[..., 0].max()], [
           y[..., 0].min(), y[..., 0].max()], color="black")
ax[0].set_title('beta_l')
ax[0].legend()

ax[1].set_ylabel("Predicted")
ax[1].set_xlabel("True")
ax[1].scatter(y_test[..., 1], preds_beta_r, color="blue", alpha=0.5)
ax[1].scatter(y_train[..., 1], preds_train_beta_r, color="red", alpha=0.5)
ax[1].plot([y[..., 1].min(), y[..., 1].max()], [
           y[..., 1].min(), y[..., 1].max()], color="black")
ax[1].set_title('beta_r')

ax[2].set_ylabel("Predicted")
ax[1].set_xlabel("True")
ax[2].scatter(y_test[..., 2], preds_alpha, color="blue", alpha=0.5)
ax[2].scatter(y_train[..., 2], preds_train_alpha, color="red", alpha=0.5)
ax[2].plot([y[..., 2].min(), y[..., 2].max()], [
           y[..., 2].min(), y[..., 2].max()], color="black")
ax[2].set_title('alpha')
plt.show()


#### 3D plots
from mpl_toolkits import mplot3d
fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')
ax.scatter(x[..., 0], x[..., 1], y[..., 0], label='groundtruth',  color='blue')
ax.scatter(x_train[..., 0], x_train[..., 1], preds_train_beta_l.reshape(-1, 1), label='predicted train',color='red')
ax.scatter(x_test[..., 0], x_test[..., 1], preds_beta_l.reshape(-1, 1),label='predicted test', color='green')
ax.set_xlabel('wheel_l')
ax.set_ylabel('wheel_r')
ax.set_zlabel("beta_l")
ax.legend()
plt.show()

fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')
ax.scatter(x[..., 0], x[..., 1], y[..., 1], label='groundtruth', color='blue')
ax.scatter(x_train[..., 0], x_train[..., 1], preds_train_beta_r.reshape(-1,1), label='predicted train', color='red')
ax.scatter(x_test[..., 0], x_test[..., 1], preds_beta_r.reshape(-1,1),label='predicted test', color='green')
ax.set_xlabel('wheel_l')
ax.set_ylabel('wheel_r')
ax.set_zlabel("beta_r")
ax.legend()
plt.show()

fig = plt.figure(figsize=(10, 10))
ax = plt.axes(projection='3d')
ax.scatter(x[..., 0], x[..., 1], y[..., 2],label='groundtruth', color='blue')
ax.scatter(x_train[..., 0], x_train[..., 1], preds_train_alpha.reshape(-1,1), label='predicted train',color='red')
ax.scatter(x_test[..., 0], x_test[..., 1], preds_alpha.reshape(-1,1), label='predicted test', color='green')
ax.set_xlabel('wheel_l')
ax.set_ylabel('wheel_r')
ax.set_zlabel("alpha")
ax.legend()
plt.show()



# To test
# ### Load Model
import numpy as np
import catboost as cb
model_beta_l = cb.CatBoostRegressor()
model_beta_l.load_model(model_name_beta_l)
beta_l = model_beta_l.predict(np.array([-3.4,3.4]))

model_beta_r = cb.CatBoostRegressor()
model_beta_r.load_model(model_name_beta_r)
beta_r =  model_beta_r.predict(np.array([-3.4,3.4]))

model_alpha = cb.CatBoostRegressor()
model_alpha.load_model(model_name_alpha)
alpha = model_alpha.predict(np.array([-3.4,3.4]))
print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")

#test check directly with interpolators for comparison
beta_l = interpolator_beta_l(np.array([[-3.4,3.4]]))
beta_r = interpolator_beta_r(np.array([[-3.4,3.4]]))
alpha = interpolator_alpha(np.array([[-3.4,3.4]]))
print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")
