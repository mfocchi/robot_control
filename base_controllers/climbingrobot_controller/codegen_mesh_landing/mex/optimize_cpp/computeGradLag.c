/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGradLag.c
 *
 * Code generation for function 'computeGradLag'
 *
 */

/* Include files */
#include "computeGradLag.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeGradLag(emxArray_real_T *workspace, int32_T ldA, int32_T nVar,
                    const emxArray_real_T *grad, int32_T mIneq,
                    const emxArray_real_T *AineqTrans,
                    const emxArray_int32_T *finiteFixed, int32_T mFixed,
                    const emxArray_int32_T *finiteLB, int32_T mLB,
                    const emxArray_int32_T *finiteUB, int32_T mUB,
                    const emxArray_real_T *lambda)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  const real_T *AineqTrans_data;
  const real_T *grad_data;
  const real_T *lambda_data;
  real_T alpha1;
  real_T beta1;
  real_T *workspace_data;
  const int32_T *finiteFixed_data;
  const int32_T *finiteLB_data;
  const int32_T *finiteUB_data;
  int32_T iL0;
  int32_T idx;
  char_T TRANSA;
  lambda_data = lambda->data;
  finiteUB_data = finiteUB->data;
  finiteLB_data = finiteLB->data;
  finiteFixed_data = finiteFixed->data;
  AineqTrans_data = AineqTrans->data;
  grad_data = grad->data;
  workspace_data = workspace->data;
  for (iL0 = 0; iL0 < nVar; iL0++) {
    workspace_data[iL0] = grad_data[iL0];
  }
  for (idx = 0; idx < mFixed; idx++) {
    workspace_data[finiteFixed_data[idx] - 1] += lambda_data[idx];
  }
  if (mIneq >= 1) {
    alpha1 = 1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)nVar;
    n_t = (ptrdiff_t)mIneq;
    lda_t = (ptrdiff_t)ldA;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, (real_T *)&AineqTrans_data[0], &lda_t,
          (real_T *)&lambda_data[mFixed], &incx_t, &beta1, &workspace_data[0],
          &incy_t);
  }
  iL0 = mFixed + mIneq;
  for (idx = 0; idx < mLB; idx++) {
    workspace_data[finiteLB_data[idx] - 1] -= lambda_data[iL0 + idx];
  }
  if (mLB - 1 >= 0) {
    iL0 += mLB;
  }
  for (idx = 0; idx < mUB; idx++) {
    workspace_data[finiteUB_data[idx] - 1] += lambda_data[iL0 + idx];
  }
}

/* End of code generation (computeGradLag.c) */
