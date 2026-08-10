/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
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
void b_computePositionVelocity(real_T params_b, const emxArray_real_T *psi,
                               const emxArray_real_T *l1,
                               const emxArray_real_T *l2,
                               const emxArray_real_T *psid,
                               const emxArray_real_T *l1d,
                               const emxArray_real_T *l2d, emxArray_real_T *p,
                               emxArray_real_T *pd)
{
  emxArray_real_T *pdx;
  emxArray_real_T *pdy;
  emxArray_real_T *pdz;
  emxArray_real_T *px;
  emxArray_real_T *py;
  emxArray_real_T *pz;
  const real_T *l1_data;
  const real_T *l1d_data;
  const real_T *l2_data;
  const real_T *l2d_data;
  const real_T *psi_data;
  const real_T *psid_data;
  real_T *p_data;
  real_T *pd_data;
  real_T *pdx_data;
  real_T *pdy_data;
  real_T *pdz_data;
  real_T *px_data;
  real_T *py_data;
  real_T *pz_data;
  int32_T i;
  int32_T loop_ub;
  l2d_data = l2d->data;
  l1d_data = l1d->data;
  psid_data = psid->data;
  l2_data = l2->data;
  l1_data = l1->data;
  psi_data = psi->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&px, 2);
  i = px->size[0] * px->size[1];
  px->size[0] = 1;
  px->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(px, i);
  px_data = px->data;
  loop_ub = psi->size[1];
  emxInit_real_T(&py, 2);
  i = py->size[0] * py->size[1];
  py->size[0] = 1;
  py->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(py, i);
  py_data = py->data;
  emxInit_real_T(&pz, 2);
  i = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  pz->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pz, i);
  pz_data = pz->data;
  emxInit_real_T(&pdx, 2);
  i = pdx->size[0] * pdx->size[1];
  pdx->size[0] = 1;
  pdx->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pdx, i);
  pdx_data = pdx->data;
  emxInit_real_T(&pdy, 2);
  i = pdy->size[0] * pdy->size[1];
  pdy->size[0] = 1;
  pdy->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pdy, i);
  pdy_data = pdy->data;
  emxInit_real_T(&pdz, 2);
  i = pdz->size[0] * pdz->size[1];
  pdz->size[0] = 1;
  pdz->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pdz, i);
  pdz_data = pdz->data;
  for (i = 0; i < loop_ub; i++) {
    px_data[i] = 0.0;
    py_data[i] = 0.0;
    pz_data[i] = 0.0;
    pdx_data[i] = 0.0;
    pdy_data[i] = 0.0;
    pdz_data[i] = 0.0;
  }
  i = psi->size[1];
  for (loop_ub = 0; loop_ub < i; loop_ub++) {
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
    d = psi_data[loop_ub];
    px_tmp = muDoubleScalarSin(d);
    d1 = l1_data[loop_ub];
    d2 = l2_data[loop_ub];
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
    d4 = l1d_data[loop_ub];
    d7 = psid_data[loop_ub];
    d8 = l2d_data[loop_ub];
    a_tmp = ((d4 * a_tmp - d4 * b_a_tmp) + 2.0 * d8 * d1 * d2) - d4 * a_tmp_tmp;
    c_a_tmp = d3 * (px_l1_tmp / px_tmp);
    pdx_data[loop_ub] =
        (d4 * px_l1_tmp + d1 * n_pz_l1 * d7) + py2b * px_tmp * a_tmp / c_a_tmp;
    pdy_data[loop_ub] = (d1 * d4 - d2 * d8) / params_b;
    pdz_data[loop_ub] =
        (d1 * d7 * px_l1_tmp - d4 * n_pz_l1) - py2b * pz_tmp * a_tmp / c_a_tmp;
    px_data[loop_ub] = d5;
    py_data[loop_ub] = d6;
    pz_data[loop_ub] = d;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  i = pd->size[0] * pd->size[1];
  pd->size[0] = 3;
  pd->size[1] = pdx->size[1];
  emxEnsureCapacity_real_T(pd, i);
  pd_data = pd->data;
  loop_ub = pdx->size[1];
  i = p->size[0] * p->size[1];
  p->size[0] = 3;
  p->size[1] = px->size[1];
  emxEnsureCapacity_real_T(p, i);
  p_data = p->data;
  for (i = 0; i < loop_ub; i++) {
    int32_T i1;
    int32_T i2;
    pd_data[3 * i] = pdx_data[i];
    i1 = 3 * i + 1;
    pd_data[i1] = pdy_data[i];
    i2 = 3 * i + 2;
    pd_data[i2] = pdz_data[i];
    p_data[3 * i] = px_data[i];
    p_data[i1] = py_data[i];
    p_data[i2] = pz_data[i];
  }
  emxFree_real_T(&pdz);
  emxFree_real_T(&pdy);
  emxFree_real_T(&pdx);
  emxFree_real_T(&pz);
  emxFree_real_T(&py);
  emxFree_real_T(&px);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

void computePositionVelocity(real_T params_b, const emxArray_real_T *psi,
                             const emxArray_real_T *l1,
                             const emxArray_real_T *l2, emxArray_real_T *p)
{
  emxArray_real_T *px;
  emxArray_real_T *py;
  emxArray_real_T *pz;
  const real_T *l1_data;
  const real_T *l2_data;
  const real_T *psi_data;
  real_T *p_data;
  real_T *px_data;
  real_T *py_data;
  real_T *pz_data;
  int32_T i;
  int32_T loop_ub;
  l2_data = l2->data;
  l1_data = l1->data;
  psi_data = psi->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&px, 2);
  i = px->size[0] * px->size[1];
  px->size[0] = 1;
  px->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(px, i);
  px_data = px->data;
  loop_ub = psi->size[1];
  emxInit_real_T(&py, 2);
  i = py->size[0] * py->size[1];
  py->size[0] = 1;
  py->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(py, i);
  py_data = py->data;
  emxInit_real_T(&pz, 2);
  i = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  pz->size[1] = psi->size[1];
  emxEnsureCapacity_real_T(pz, i);
  pz_data = pz->data;
  for (i = 0; i < loop_ub; i++) {
    px_data[i] = 0.0;
    py_data[i] = 0.0;
    pz_data[i] = 0.0;
  }
  i = psi->size[1];
  for (loop_ub = 0; loop_ub < i; loop_ub++) {
    real_T a_tmp;
    real_T a_tmp_tmp;
    real_T b_a_tmp_tmp;
    real_T d;
    real_T d1;
    d = l1_data[loop_ub];
    d1 = l2_data[loop_ub];
    a_tmp_tmp = params_b * params_b;
    b_a_tmp_tmp = d * d;
    a_tmp = (a_tmp_tmp + b_a_tmp_tmp) - d1 * d1;
    d1 = psi_data[loop_ub];
    a_tmp_tmp = muDoubleScalarSqrt(1.0 - a_tmp * a_tmp /
                                             (4.0 * a_tmp_tmp * b_a_tmp_tmp));
    px_data[loop_ub] = d * muDoubleScalarSin(d1) * a_tmp_tmp;
    py_data[loop_ub] = a_tmp / (2.0 * params_b);
    pz_data[loop_ub] = -d * muDoubleScalarCos(d1) * a_tmp_tmp;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  i = p->size[0] * p->size[1];
  p->size[0] = 3;
  p->size[1] = px->size[1];
  emxEnsureCapacity_real_T(p, i);
  p_data = p->data;
  loop_ub = px->size[1];
  for (i = 0; i < loop_ub; i++) {
    p_data[3 * i] = px_data[i];
    p_data[3 * i + 1] = py_data[i];
    p_data[3 * i + 2] = pz_data[i];
  }
  emxFree_real_T(&pz);
  emxFree_real_T(&py);
  emxFree_real_T(&px);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (computePositionVelocity.c) */
