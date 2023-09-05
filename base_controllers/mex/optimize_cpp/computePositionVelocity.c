/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computePositionVelocity.c
 *
 * Code generation for function 'computePositionVelocity'
 *
 */

/* Include files */
#include "computePositionVelocity.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void computePositionVelocity(real_T params_b, const emxArray_real_T *psi, const
  emxArray_real_T *l1, const emxArray_real_T *l2, const emxArray_real_T *psid,
  const emxArray_real_T *l1d, const emxArray_real_T *l2d, emxArray_real_T *p,
  emxArray_real_T *pd)
{
  emxArray_real_T *pdx;
  emxArray_real_T *pdy;
  emxArray_real_T *pdz;
  emxArray_real_T *px;
  emxArray_real_T *py;
  emxArray_real_T *pz;
  real_T a_tmp;
  real_T a_tmp_tmp;
  real_T b_a_tmp;
  real_T c_a_tmp;
  real_T d;
  real_T d1;
  real_T d2;
  real_T d3;
  real_T d4;
  real_T d5;
  real_T d6;
  real_T d7;
  real_T d8;
  real_T n_pz_l1;
  real_T px_l1_tmp;
  real_T px_tmp;
  real_T py2b;
  real_T pz_tmp;
  int32_T i;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&px, 2, true);
  i = px->size[0] * px->size[1];
  px->size[0] = 1;
  px->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(px, i);
  loop_ub = psi->size[1];
  for (i = 0; i < loop_ub; i++) {
    px->data[i] = 0.0;
  }

  emxInit_real_T(&py, 2, true);
  i = py->size[0] * py->size[1];
  py->size[0] = 1;
  py->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(py, i);
  loop_ub = psi->size[1];
  for (i = 0; i < loop_ub; i++) {
    py->data[i] = 0.0;
  }

  emxInit_real_T(&pz, 2, true);
  i = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  pz->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pz, i);
  loop_ub = psi->size[1];
  for (i = 0; i < loop_ub; i++) {
    pz->data[i] = 0.0;
  }

  emxInit_real_T(&pdx, 2, true);
  i = pdx->size[0] * pdx->size[1];
  pdx->size[0] = 1;
  pdx->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pdx, i);
  loop_ub = psi->size[1];
  for (i = 0; i < loop_ub; i++) {
    pdx->data[i] = 0.0;
  }

  emxInit_real_T(&pdy, 2, true);
  i = pdy->size[0] * pdy->size[1];
  pdy->size[0] = 1;
  pdy->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pdy, i);
  loop_ub = psi->size[1];
  for (i = 0; i < loop_ub; i++) {
    pdy->data[i] = 0.0;
  }

  emxInit_real_T(&pdz, 2, true);
  i = pdz->size[0] * pdz->size[1];
  pdz->size[0] = 1;
  pdz->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pdz, i);
  loop_ub = psi->size[1];
  for (i = 0; i < loop_ub; i++) {
    pdz->data[i] = 0.0;
  }

  i = psi->size[1];
  for (loop_ub = 0; loop_ub < i; loop_ub++) {
    d = psi->data[loop_ub];
    px_tmp = muDoubleScalarSin(d);
    d1 = l1->data[loop_ub];
    d2 = l2->data[loop_ub];
    a_tmp = params_b * params_b;
    b_a_tmp = d1 * d1;
    a_tmp_tmp = d2 * d2;
    c_a_tmp = (a_tmp + b_a_tmp) - a_tmp_tmp;
    d3 = 4.0 * a_tmp * b_a_tmp;
    d4 = muDoubleScalarSqrt(1.0 - c_a_tmp * c_a_tmp / d3);
    d5 = d1 * px_tmp * d4;
    d6 = c_a_tmp / (2.0 * params_b);
    pz_tmp = muDoubleScalarCos(d);
    d = -d1 * pz_tmp * d4;
    px_l1_tmp = d5 / d1;
    n_pz_l1 = -d / d1;
    py2b = d6 * 2.0 * params_b;
    d4 = l1d->data[loop_ub];
    d7 = psid->data[loop_ub];
    d8 = l2d->data[loop_ub];
    a_tmp = ((d4 * a_tmp - d4 * b_a_tmp) + 2.0 * d8 * d1 * d2) - d4 * a_tmp_tmp;
    c_a_tmp = d3 * (px_l1_tmp / px_tmp);
    pdx->data[loop_ub] = (d4 * px_l1_tmp + d1 * n_pz_l1 * d7) + py2b * px_tmp *
      a_tmp / c_a_tmp;
    pdy->data[loop_ub] = (d1 * d4 - d2 * d8) / params_b;
    pdz->data[loop_ub] = (d1 * d7 * px_l1_tmp - d4 * n_pz_l1) - py2b * pz_tmp *
      a_tmp / c_a_tmp;
    px->data[loop_ub] = d5;
    py->data[loop_ub] = d6;
    pz->data[loop_ub] = d;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  i = pd->size[0] * pd->size[1];
  pd->size[0] = 3;
  pd->size[1] = pdx->size[1];
  emxEnsureCapacity_real_T(pd, i);
  loop_ub = pdx->size[1];
  for (i = 0; i < loop_ub; i++) {
    pd->data[3 * i] = pdx->data[i];
  }

  emxFree_real_T(&pdx);
  loop_ub = pdy->size[1];
  for (i = 0; i < loop_ub; i++) {
    pd->data[3 * i + 1] = pdy->data[i];
  }

  emxFree_real_T(&pdy);
  loop_ub = pdz->size[1];
  for (i = 0; i < loop_ub; i++) {
    pd->data[3 * i + 2] = pdz->data[i];
  }

  emxFree_real_T(&pdz);
  i = p->size[0] * p->size[1];
  p->size[0] = 3;
  p->size[1] = px->size[1];
  emxEnsureCapacity_real_T(p, i);
  loop_ub = px->size[1];
  for (i = 0; i < loop_ub; i++) {
    p->data[3 * i] = px->data[i];
  }

  emxFree_real_T(&px);
  loop_ub = py->size[1];
  for (i = 0; i < loop_ub; i++) {
    p->data[3 * i + 1] = py->data[i];
  }

  emxFree_real_T(&py);
  loop_ub = pz->size[1];
  for (i = 0; i < loop_ub; i++) {
    p->data[3 * i + 2] = pz->data[i];
  }

  emxFree_real_T(&pz);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (computePositionVelocity.c) */
