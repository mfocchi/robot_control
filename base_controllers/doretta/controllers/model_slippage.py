# %%
import pandas as pd
import numpy as np
import catboost as cb
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split

# %%
from sklearn.metrics import r2_score

# %% [markdown]
# ## Load data
# v omega
# df = pd.read_csv('data_v_omega.csv',header=None, names=['v','omega','beta_l','beta_r','alpha'])
# x = df[['v','omega']].values
# y = df[['beta_l','beta_r','alpha']].values

#wheel_l wheel_r
data = 'ident_wheels_sim.csv'
#data = 'ident_wheels_real.csv'
sim = True

df = pd.read_csv(data,header=None, names=['wheel_l','wheel_r','beta_l','beta_r','alpha'])
x = df[['wheel_l','wheel_r']].values
y = df[['beta_l','beta_r','alpha']].values

# %% plot hist to see if it is neeeded a scaling
plt.figure()
plt.hist(x[:,0])
plt.figure()
plt.hist(x[:,1])
plt.show()

# %%
plt.figure()
plt.hist(y[...,0])
plt.figure()
plt.hist(y[...,1])
plt.figure()
plt.hist(y[...,2])
plt.show()

# %% [markdown]
# Split data into train test, and test set (10%), with random seed = 13

# %% spit dataset in train and test set (10%)
x_train, x_test, y_train, y_test = train_test_split(x, y, random_state=13, test_size=0.1)

# %%
len(x_train),len(x_test)
len(y_train),len(y_test)

# %% create model of regressor
model = cb.CatBoostRegressor(loss_function='MultiRMSE',iterations=200)
# %% train the model
model.fit(x_train,y_train)

# %% inference from xtrain
y_preds_train = model.predict(x_train)

# %%inference from xtest
y_preds_test = model.predict(x_test)

# %%
print(f'R2 metric train: {r2_score(y_train, y_preds_train)}')
print(f'R2 metric test: {r2_score(y_test, y_preds_test)}')


if sim:
    # %% save the model python
    model_name = 'slippage_regressor_wheels.cb'
    model.save_model(model_name)
else:
    # %% save the model cpp binary
    model_name = 'slippage_regressor_wheels.cbm'
    model.save_model(model_name,format="cbm")

# %%
plt.figure(figsize=(7,7))
plt.title("value function beta_l with train data")
plt.xlabel("True")
plt.ylabel("Predicted")
plt.scatter(y_train[:,0],y_preds_train[:,0], color="blue", alpha=0.5)
plt.plot([y_train[:,0].min(),y_train[:,0].max()],[y_train[:,0].min(),y_train[:,0].max()], color="black")
plt.show()

# %%
plt.figure(figsize=(7,7))
plt.title("value function beta_l with TEST data")
plt.xlabel("True")
plt.ylabel("Predicted")
plt.scatter(y_test[:,0],y_preds_test[:,0], color="blue", alpha=0.5)
plt.plot([y_test[:,0].min(),y_test[:,0].max()],[y_test[:,0].min(),y_test[:,0].max()], color="black")
plt.show()
# %%
plt.figure(figsize=(7,7))
plt.title("value function beta_r with TEST data")
plt.xlabel("True")
plt.ylabel("Predicted")
plt.scatter(y_test[:,1],y_preds_test[:,1], color="red", alpha=0.5)
plt.plot([y_test[:,1].min(),y_test[:,1].max()],[y_test[:,1].min(),y_test[:,1].max()], color="black")
plt.show()
# %%
plt.figure(figsize=(7,7))
plt.title("value function alpha with TEST data")
plt.xlabel("True")
plt.ylabel("Predicted")
plt.scatter(y_test[:,2],y_preds_test[:,2], color="blue", alpha=0.5)
plt.plot([y_test[:,2].min(),y_test[:,2].max()],[y_test[:,2].min(),y_test[:,2].max()], color="black")
plt.show()

# To test
# ### Load Model
import numpy as np
import catboost as cb
model = cb.CatBoostRegressor()
model.load_model(model_name)
model.predict(np.array([0.5, 0.3]))

