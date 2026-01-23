/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct1.c
 *
 * Code generation for function 'factoryConstruct1'
 *
 */

/* Include files */
#include "factoryConstruct1.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void b_factoryConstruct(
    const real_T objfun_workspace_p0[3], const param *objfun_workspace_params,
    const real_T nonlin_workspace_p0[3], const real_T nonlin_workspace_pf[3],
    real_T nonlin_workspace_Fleg_max, real_T nonlin_workspace_mu,
    const param *nonlin_workspace_params, int32_T nVar, int32_T mCineq,
    const emxArray_real_T *lb, const emxArray_real_T *ub, k_struct_T *obj)
{
  const real_T *lb_data;
  const real_T *ub_data;
  int32_T idx;
  boolean_T b;
  ub_data = ub->data;
  lb_data = lb->data;
  obj->objfun.workspace.params = *objfun_workspace_params;
  obj->objfun.workspace.p0[0] = objfun_workspace_p0[0];
  obj->nonlin.workspace.p0[0] = nonlin_workspace_p0[0];
  obj->nonlin.workspace.pf[0] = nonlin_workspace_pf[0];
  obj->objfun.workspace.p0[1] = objfun_workspace_p0[1];
  obj->nonlin.workspace.p0[1] = nonlin_workspace_p0[1];
  obj->nonlin.workspace.pf[1] = nonlin_workspace_pf[1];
  obj->objfun.workspace.p0[2] = objfun_workspace_p0[2];
  obj->nonlin.workspace.p0[2] = nonlin_workspace_p0[2];
  obj->nonlin.workspace.pf[2] = nonlin_workspace_pf[2];
  obj->nonlin.workspace.Fleg_max = nonlin_workspace_Fleg_max;
  obj->nonlin.workspace.mu = nonlin_workspace_mu;
  obj->nonlin.workspace.params = *nonlin_workspace_params;
  obj->f_1 = 0.0;
  idx = obj->cIneq_1->size[0];
  obj->cIneq_1->size[0] = mCineq;
  emxEnsureCapacity_real_T(obj->cIneq_1, idx);
  idx = obj->cIneq_2->size[0];
  obj->cIneq_2->size[0] = mCineq;
  emxEnsureCapacity_real_T(obj->cIneq_2, idx);
  obj->nVar = nVar;
  obj->mIneq = mCineq;
  obj->numEvals = 0;
  idx = obj->hasLB->size[0];
  obj->hasLB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasLB, idx);
  idx = obj->hasUB->size[0];
  obj->hasUB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasUB, idx);
  b = false;
  idx = 0;
  while ((!b) && (idx + 1 <= nVar)) {
    obj->hasLB->data[idx] = ((!muDoubleScalarIsInf(lb_data[idx])) &&
                             (!muDoubleScalarIsNaN(lb_data[idx])));
    obj->hasUB->data[idx] = ((!muDoubleScalarIsInf(ub_data[idx])) &&
                             (!muDoubleScalarIsNaN(ub_data[idx])));
    if (obj->hasLB->data[idx] || obj->hasUB->data[idx]) {
      b = true;
    }
    idx++;
  }
  while (idx + 1 <= nVar) {
    obj->hasLB->data[idx] = ((!muDoubleScalarIsInf(lb_data[idx])) &&
                             (!muDoubleScalarIsNaN(lb_data[idx])));
    obj->hasUB->data[idx] = ((!muDoubleScalarIsInf(ub_data[idx])) &&
                             (!muDoubleScalarIsNaN(ub_data[idx])));
    idx++;
  }
  obj->hasBounds = b;
}

/* End of code generation (factoryConstruct1.c) */
