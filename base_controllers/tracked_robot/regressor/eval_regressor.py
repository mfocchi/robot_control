# %%
import pandas as pd
import numpy as np
import catboost as cb


import os
os.environ["MPLBACKEND"] = "TkAgg" #do it out side it export MPLBACKEND=TkAgg
import matplotlib
matplotlib.use('TkAgg')


friction_coeff = str(0.4)
model_beta_l = cb.CatBoostRegressor()
model_name_beta_l = 'model_beta_l'+friction_coeff+'.cb'
model_beta_l.load_model(model_name_beta_l)

model_beta_r = cb.CatBoostRegressor()
model_name_beta_r = 'model_beta_r'+friction_coeff+'.cb'
model_beta_r.load_model(model_name_beta_r)

model_alpha = cb.CatBoostRegressor()
model_name_alpha  = 'model_alpha'+friction_coeff+'.cb'
model_alpha.load_model(model_name_alpha)

if len(model_beta_l.feature_names_)==2:
    input = np.array([5., 8.])
elif len(model_beta_l.feature_names_)==4:
    input = np.array([5., 8., 0.,0.])
else:
    input = np.array([5., 8., 0., 0.0, 0.])
beta_l = model_beta_l.predict(input)
beta_r =  model_beta_r.predict(input)
alpha = model_alpha.predict(input)

print(f" friction_coeff: {friction_coeff}")
print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")

#MATLAB
# import matlab.engine
# eng = matlab.engine.start_matlab()
# model_alpha = eng.load('training.mat')['alpha_model_18']
# model_beta_l = eng.load('training.mat')['beta_l_model_18']
# model_beta_r = eng.load('training.mat')['beta_r_model_18']
# #model_fast = eng.loadLearnerForCoder('alpha_model_forcodegen')
# alpha = eng.feval(model_alpha['predictFcn'], omegas)
# beta_l = eng.feval(model_beta_l['predictFcn'], omegas)
# beta_r = eng.feval(model_beta_r['predictFcn'], omegas)
# print(f" alpha {alpha}, Beta_l {beta_l}, Beta_r {beta_r}")


# import time
# for i in range(100):
#     t = time.time()
#     alpha = eng.feval(model_alpha['predictFcn'], omegas)
#     beta_l = eng.feval(model_beta_r['predictFcn'], omegas)
#     beta_r = eng.feval(model_beta_l['predictFcn'], omegas)
#     #this is faster but cannot split models
#     #eng.eval_slippage(matlab.double(omega_l), matlab.double(omega_r))
#     print(time.time() - t)
