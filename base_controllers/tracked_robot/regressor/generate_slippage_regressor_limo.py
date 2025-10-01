
from scipy.interpolate import RBFInterpolator
import os
os.environ["MPLBACKEND"] = "TkAgg" #do it out side it exports MPLBACKEND=TkAgg
import matplotlib
matplotlib.use('TkAgg')
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
import glob
import sys

pattern = "model_limo*.cb"
# Search in the current directory
matching_files = glob.glob(pattern)
if matching_files:
    response = input(colored(f"Some model file already exists. Overwrite? [y/N]: ","red")).strip().lower()
    if response != 'y':
        print("Aborting script.")
        sys.exit(0)
# Continue with script if file does not exist or user agrees to overwrite
print("Proceeding...")

list_file = [x for x in os.listdir('limo/') if (x.endswith('.csv') )]
df_vpos = pd.DataFrame()

# Initialize logs
des_wheel_l_log = []
des_wheel_r_log = []
beta_l_log = []
beta_r_log = []
alpha_log = []

samples_to_discard = 50

for file in list_file:
    print("reading...",file)
    tmp_df = pd.read_csv(os.path.join('limo/',file),header=1, names=['time', 'wheel_l','wheel_r','des_wheel_l','des_wheel_r','roll', 'pitch', 'yaw', 'beta_l','beta_r','alpha'])


    des_wheel_L = tmp_df['des_wheel_l']
    des_wheel_R = tmp_df['des_wheel_r']
    alpha = tmp_df['alpha']
    beta_l = tmp_df['beta_l']
    beta_r = tmp_df['beta_r']

    # Append to logs
    des_wheel_l_log.extend(des_wheel_L[samples_to_discard:])
    des_wheel_r_log.extend(des_wheel_R[samples_to_discard:])
    beta_l_log.extend(beta_l[samples_to_discard:])
    beta_r_log.extend(beta_r[samples_to_discard:])
    alpha_log.extend(alpha[samples_to_discard:])

    plt.figure()
    plt.subplot(4,1,1)
    plt.title(file)
    plt.ylabel("wheel")
    plt.plot(des_wheel_L[samples_to_discard:])
    plt.plot(des_wheel_R[samples_to_discard:])
    plt.subplot(4,1,2)
    plt.plot(beta_l[samples_to_discard:])
    plt.ylabel("beta_l")
    plt.subplot(4,1,3)
    plt.plot(beta_r[samples_to_discard:])
    plt.ylabel("beta_r")
    plt.subplot(4,1,4)
    plt.plot(alpha[samples_to_discard:])
    plt.ylabel("alpha")
    plt.show()


# Convert logs to arrays
wheel_l = np.array(des_wheel_l_log)
wheel_r = np.array(des_wheel_r_log)
beta_l = np.array(beta_l_log)      # assuming you meant beta_l_log here
beta_r = np.array(beta_r_log)
alpha = np.array(alpha_log)    # assuming you meant avg_alpha_log here

x = np.column_stack((wheel_l, wheel_r))
y = np.column_stack((beta_l, beta_r, alpha))
df = pd.DataFrame(np.hstack([x, y]), columns=['des_wheel_l', 'des_wheel_r', 'beta_l', 'beta_r', 'alpha'])
#
# #upsampling
#Fit an interpolator for each output dimension
interpolator_beta_l = RBFInterpolator(x, y[:, 0], smoothing=0.1)
interpolator_beta_r = RBFInterpolator(x, y[:, 1], smoothing=0.1)
interpolator_alpha  = RBFInterpolator(x, y[:, 2], smoothing=0.1)
#
# Create a dense grid of inputs
wl_dense = np.linspace(0, 18, 100)
wr_dense = np.linspace(0, 18, 100)
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
df_dense = pd.DataFrame(data_dense, columns=['des_wheel_l', 'des_wheel_r', 'beta_l', 'beta_r', 'alpha'])

x = df_dense[['des_wheel_l','des_wheel_r']].values
y = df_dense[['beta_l','beta_r','alpha']].values

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
model_beta_l = cb.CatBoostRegressor(learning_rate=1e-2, max_depth=5)
# %% train the model
model_beta_l.fit(x_train, y_train[..., 0].reshape(-1, 1), verbose=100,  eval_set=(x_valid, y_valid[..., 0].reshape(-1, 1)), use_best_model=True)
preds_train_beta_l = model_beta_l.predict(x_train)
preds_beta_l = model_beta_l.predict(x_test)
print(f'R2 metric train beta_l: {r2_score(y_train[...,0], preds_train_beta_l)}')
print(f'R2 metric test beta_l: {r2_score(y_test[...,0], preds_beta_l)}')
# %% save the model python
model_name_beta_l = 'model_limo_beta_l.cb'
model_beta_l.save_model(model_name_beta_l)



model_beta_r = cb.CatBoostRegressor(learning_rate=1e-2, max_depth=5)
model_beta_r.fit(x_train, y_train[..., 1].reshape(-1, 1), verbose=100,     eval_set=(x_valid, y_valid[..., 1].reshape(-1, 1)), use_best_model=True)
preds_train_beta_r = model_beta_r.predict(x_train)
preds_beta_r = model_beta_r.predict(x_test)
print(f'R2 metric train beta_r: {r2_score(y_train[...,1], preds_train_beta_r)}')
print(f'R2 metric test beta_r: {r2_score(y_test[...,1], preds_beta_r)}')
# %% save the model python
model_name_beta_r = 'model_limo_beta_r.cb'
model_beta_r.save_model(model_name_beta_r)


model_alpha = cb.CatBoostRegressor(learning_rate=1e-2, max_depth=5)
model_alpha.fit(x_train, y_train[..., 2].reshape(-1, 1), verbose=100,        eval_set=(x_valid, y_valid[..., 2].reshape(-1, 1)), use_best_model=True)
preds_train_alpha = model_alpha.predict(x_train)
preds_alpha = model_alpha.predict(x_test)
print(f'R2 metric train alpha: {r2_score(y_train[...,2], preds_train_alpha)}')
print(f'R2 metric test alpha: {r2_score(y_test[...,2], preds_alpha)}')
model_name_alpha = 'model_limo_alpha.cb'
model_alpha.save_model(model_name_alpha)

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
model_beta_r = cb.CatBoostRegressor()
model_beta_r.load_model(model_name_beta_r)
model_alpha = cb.CatBoostRegressor()
model_alpha.load_model(model_name_alpha)


print("test left")
beta_l = model_beta_l.predict(np.array([-3.4,3.4]))
beta_r =  model_beta_r.predict(np.array([-3.4,3.4]))
alpha = model_alpha.predict(np.array([-3.4,3.4]))
print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")

#test check directly with interpolators for comparison turn left
# beta_l = interpolator_beta_l(np.array([[-3.4,3.4]]))
# beta_r = interpolator_beta_r(np.array([[-3.4,3.4]]))
# alpha = interpolator_alpha(np.array([[-3.4,3.4]]))
# print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")

print("test right")
beta_l = model_beta_l.predict(np.array([3.4,-3.4]))
beta_r =  model_beta_r.predict(np.array([3.4,-3.4]))
alpha = model_alpha.predict(np.array([3.4,-3.4]))
print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")

#test check directly with interpolators for comparison turn left
# beta_l = interpolator_beta_l(np.array([[3.4,-3.4]]))
# beta_r = interpolator_beta_r(np.array([[3.4,-3.4]]))
# alpha = interpolator_alpha(np.array([[3.4,-3.4]]))
# print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")