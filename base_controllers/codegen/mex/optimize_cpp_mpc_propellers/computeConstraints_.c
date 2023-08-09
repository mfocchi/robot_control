/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeConstraints_.c
 *
 * Code generation for function 'computeConstraints_'
 *
 */

/* Include files */
#include "computeConstraints_.h"
#include "optimize_cpp_mpc_propellers.h"
#include "optimize_cpp_mpc_propellers_data.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
int32_T computeConstraints_(const emxArray_real_T
  *c_obj_nonlcon_tunableEnvironmen, const emxArray_real_T
  *d_obj_nonlcon_tunableEnvironmen, int64_T e_obj_nonlcon_tunableEnvironmen,
  int32_T obj_mCineq, const emxArray_real_T *x, emxArray_real_T *Cineq_workspace,
  int32_T ineq0)
{
  emxArray_real_T *varargout_1;
  int32_T i;
  int32_T idx_current;
  int32_T idx_end;
  int32_T status;
  boolean_T allFinite;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&varargout_1, 2, true);
  anon(c_obj_nonlcon_tunableEnvironmen, d_obj_nonlcon_tunableEnvironmen,
       e_obj_nonlcon_tunableEnvironmen, x, varargout_1);
  if (ineq0 > (ineq0 + obj_mCineq) - 1) {
    idx_current = 1;
  } else {
    idx_current = ineq0;
  }

  idx_end = varargout_1->size[1];
  for (i = 0; i < idx_end; i++) {
    Cineq_workspace->data[(idx_current + i) - 1] = varargout_1->data[i];
  }

  emxFree_real_T(&varargout_1);
  status = 1;
  allFinite = true;
  idx_current = ineq0;
  idx_end = (ineq0 + obj_mCineq) - 1;
  while (allFinite && (idx_current <= idx_end)) {
    allFinite = ((!muDoubleScalarIsInf(Cineq_workspace->data[idx_current - 1])) &&
                 (!muDoubleScalarIsNaN(Cineq_workspace->data[idx_current - 1])));
    idx_current++;
  }

  if (!allFinite) {
    idx_current -= 2;
    if (muDoubleScalarIsNaN(Cineq_workspace->data[idx_current])) {
      status = -3;
    } else if (Cineq_workspace->data[idx_current] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return status;
}

/* End of code generation (computeConstraints_.c) */
