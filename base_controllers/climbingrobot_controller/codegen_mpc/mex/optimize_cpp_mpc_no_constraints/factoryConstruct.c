/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct.c
 *
 * Code generation for function 'factoryConstruct'
 *
 */

/* Include files */
#include "factoryConstruct.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void factoryConstruct(int32_T nVarMax, int32_T mConstrMax, const emxArray_real_T
                      *x0, d_struct_T *obj)
{
  int32_T i;
  obj->nVarMax = nVarMax;
  obj->mNonlinIneq = 0;
  obj->mNonlinEq = 0;
  obj->mIneq = 0;
  obj->mEq = 0;
  obj->iNonIneq0 = 1;
  obj->iNonEq0 = 1;
  obj->sqpFval = 0.0;
  obj->sqpFval_old = 0.0;
  i = obj->xstarsqp->size[0] * obj->xstarsqp->size[1];
  obj->xstarsqp->size[0] = 1;
  obj->xstarsqp->size[1] = x0->size[1];
  emxEnsureCapacity_real_T(obj->xstarsqp, i);
  i = obj->xstarsqp_old->size[0] * obj->xstarsqp_old->size[1];
  obj->xstarsqp_old->size[0] = 1;
  obj->xstarsqp_old->size[1] = x0->size[1];
  emxEnsureCapacity_real_T(obj->xstarsqp_old, i);
  i = obj->grad->size[0];
  obj->grad->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->grad, i);
  i = obj->grad_old->size[0];
  obj->grad_old->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->grad_old, i);
  obj->FunctionEvaluations = 0;
  obj->sqpIterations = 0;
  obj->sqpExitFlag = 0;
  i = obj->lambdasqp->size[0];
  obj->lambdasqp->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->lambdasqp, i);
  for (i = 0; i < mConstrMax; i++) {
    obj->lambdasqp->data[i] = 0.0;
  }

  i = obj->lambdasqp_old->size[0];
  obj->lambdasqp_old->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->lambdasqp_old, i);
  obj->steplength = 1.0;
  i = obj->delta_x->size[0];
  obj->delta_x->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->delta_x, i);
  for (i = 0; i < nVarMax; i++) {
    obj->delta_x->data[i] = 0.0;
  }

  i = obj->socDirection->size[0];
  obj->socDirection->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->socDirection, i);
  i = obj->lambda_old->size[0];
  obj->lambda_old->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->lambda_old, i);
  i = obj->workingset_old->size[0];
  obj->workingset_old->size[0] = mConstrMax;
  emxEnsureCapacity_int32_T(obj->workingset_old, i);
  i = obj->gradLag->size[0];
  obj->gradLag->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->gradLag, i);
  i = obj->delta_gradLag->size[0];
  obj->delta_gradLag->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->delta_gradLag, i);
  i = obj->xstar->size[0];
  obj->xstar->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->xstar, i);
  obj->fstar = 0.0;
  obj->firstorderopt = 0.0;
  i = obj->lambda->size[0];
  obj->lambda->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->lambda, i);
  for (i = 0; i < mConstrMax; i++) {
    obj->lambda->data[i] = 0.0;
  }

  obj->state = 0;
  obj->maxConstr = 0.0;
  obj->iterations = 0;
  i = obj->searchDir->size[0];
  obj->searchDir->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->searchDir, i);
}

/* End of code generation (factoryConstruct.c) */
