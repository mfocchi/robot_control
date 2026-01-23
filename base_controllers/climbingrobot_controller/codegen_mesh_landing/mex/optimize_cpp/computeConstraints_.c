/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeConstraints_.c
 *
 * Code generation for function 'computeConstraints_'
 *
 */

/* Include files */
#include "computeConstraints_.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
int32_T computeConstraints_(int32_T c_obj_next_next_next_next_next_,
                            const real_T d_obj_next_next_next_next_next_[3],
                            const real_T e_obj_next_next_next_next_next_[3],
                            real_T f_obj_next_next_next_next_next_,
                            real_T g_obj_next_next_next_next_next_,
                            const param *h_obj_next_next_next_next_next_,
                            const emxArray_real_T *x,
                            emxArray_real_T *Cineq_workspace, int32_T ineq0)
{
  emxArray_int32_T *y;
  emxArray_real_T *r;
  real_T *Cineq_workspace_data;
  real_T *r1;
  int32_T idx_end;
  int32_T status;
  int32_T yk;
  int32_T *y_data;
  boolean_T allFinite;
  Cineq_workspace_data = Cineq_workspace->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_int32_T(&y, 2);
  idx_end = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = c_obj_next_next_next_next_next_;
  emxEnsureCapacity_int32_T(y, idx_end);
  y_data = y->data;
  y_data[0] = 0;
  yk = 0;
  for (idx_end = 2; idx_end <= c_obj_next_next_next_next_next_; idx_end++) {
    yk++;
    y_data[idx_end - 1] = yk;
  }
  emxInit_real_T(&r, 2);
  optimize_cpp_anonFcn2(
      d_obj_next_next_next_next_next_, e_obj_next_next_next_next_next_,
      f_obj_next_next_next_next_next_, g_obj_next_next_next_next_next_,
      h_obj_next_next_next_next_next_, x, r);
  r1 = r->data;
  yk = r->size[1];
  for (idx_end = 0; idx_end < yk; idx_end++) {
    Cineq_workspace_data[(y_data[idx_end] + ineq0) - 1] = r1[idx_end];
  }
  emxFree_real_T(&r);
  emxFree_int32_T(&y);
  status = 1;
  allFinite = true;
  yk = ineq0;
  idx_end = (ineq0 + c_obj_next_next_next_next_next_) - 1;
  while (allFinite && (yk <= idx_end)) {
    real_T allFinite_tmp;
    allFinite_tmp = Cineq_workspace_data[yk - 1];
    allFinite = ((!muDoubleScalarIsInf(allFinite_tmp)) &&
                 (!muDoubleScalarIsNaN(allFinite_tmp)));
    yk++;
  }
  if (!allFinite) {
    yk -= 2;
    if (muDoubleScalarIsNaN(Cineq_workspace_data[yk])) {
      status = -3;
    } else if (Cineq_workspace_data[yk] < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return status;
}

/* End of code generation (computeConstraints_.c) */
