/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeGradLag.c
 *
 * Code generation for function 'computeGradLag'
 *
 */

/* Include files */
#include "computeGradLag.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeGradLag(emxArray_real_T *workspace, int32_T ldA, int32_T nVar, const
                    emxArray_real_T *grad, int32_T mIneq, const emxArray_real_T *
                    AineqTrans, const emxArray_int32_T *finiteFixed, int32_T
                    mFixed, const emxArray_int32_T *finiteLB, int32_T mLB, const
                    emxArray_int32_T *finiteUB, int32_T mUB, const
                    emxArray_real_T *lambda)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  int32_T iL0;
  int32_T idx;
  char_T TRANSA;
  for (iL0 = 0; iL0 < nVar; iL0++) {
    workspace->data[iL0] = grad->data[iL0];
  }

  for (idx = 0; idx < mFixed; idx++) {
    workspace->data[finiteFixed->data[idx] - 1] += lambda->data[idx];
  }

  if ((nVar >= 1) && (mIneq >= 1)) {
    alpha1 = 1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)nVar;
    n_t = (ptrdiff_t)mIneq;
    lda_t = (ptrdiff_t)ldA;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &AineqTrans->data[0], &lda_t,
          &lambda->data[mFixed], &incx_t, &beta1, &workspace->data[0], &incy_t);
  }

  iL0 = mFixed + mIneq;
  for (idx = 0; idx < mLB; idx++) {
    workspace->data[finiteLB->data[idx] - 1] -= lambda->data[iL0];
    iL0++;
  }

  for (idx = 0; idx < mUB; idx++) {
    workspace->data[finiteUB->data[idx] - 1] += lambda->data[iL0];
    iL0++;
  }
}

/* End of code generation (computeGradLag.c) */
