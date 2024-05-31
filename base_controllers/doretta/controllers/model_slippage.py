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

# %%
df = pd.read_csv('data.csv',header=None, names=['v','omega','beta_l','beta_r','alpha'])

# %%
x = df[['v','omega']].values

# %%
y = df[['beta_l','beta_r','alpha']].values

# %%
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

# %%
x_train, x_test, y_train, y_test = train_test_split(x, y, random_state=13, test_size=0.1)

# %%
len(x_train),len(x_test)

# %%
len(y_train),len(y_test)

# %%
model = cb.CatBoostRegressor(loss_function='MultiRMSE',iterations=1000)

# %%
model.fit(x_train,y_train)

# %%
y_preds_train = model.predict(x_train)

# %%
y_preds_test = model.predict(x_test)

# %%
print(f'R2 metric train: {r2_score(y_train, y_preds_train)}')
print(f'R2 metric test: {r2_score(y_test, y_preds_test)}')

# %% save the model
model.save_model('slippage_regressor.cb')

# %%
plt.figure(figsize=(7,7))
plt.title("Value function Train")
plt.xlabel("True")
plt.ylabel("Predicted")
plt.scatter(y_train[...,0],y_preds_train[...,0], color="blue", alpha=0.5)
plt.plot([0,y_train[...,0].max()],[0,y_train[...,0].max()], color="black")
plt.show()

# %%
plt.figure(figsize=(7,7))
plt.title("Value function Test")
plt.xlabel("True")
plt.ylabel("Predicted")
plt.scatter(y_test[...,1],y_preds_test[...,1], color="red", alpha=0.5)
plt.plot([0,y_train[...,1].max()],[0,y_train[...,1].max()], color="black")
plt.show()


# %% [markdown]
# ### Load Model

# %%
import numpy as np
import catboost as cb

# %%
model = cb.CatBoostRegressor()

# %%
model.load_model('slippage_regressor.cb')

# %%
model.predict(np.array([0.1,0.3]))


