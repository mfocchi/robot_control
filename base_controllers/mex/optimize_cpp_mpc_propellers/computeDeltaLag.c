/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeDeltaLag.c
 *
 * Code generation for function 'computeDeltaLag'
 *
 */

/* Include files */
#include "computeDeltaLag.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeDeltaLag(int32_T nVar, int32_T ldJ, int32_T mNonlinIneq,
                     emxArray_real_T *workspace, const emxArray_real_T *grad,
                     const emxArray_real_T *JacIneqTrans, int32_T ineqJ0, const
                     emxArray_real_T *grad_old, const emxArray_real_T
                     *JacIneqTrans_old, const emxArray_real_T *lambda, int32_T
                     ineqL0)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T a;
  real_T beta1;
  int32_T i;
  char_T TRANSA;
  for (i = 0; i < nVar; i++) {
    workspace->data[i] = grad->data[i];
  }

  if (nVar >= 1) {
    a = -1.0;
    n_t = (ptrdiff_t)nVar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    daxpy(&n_t, &a, &grad_old->data[0], &incx_t, &workspace->data[0], &incy_t);
  }

  if ((mNonlinIneq > 0) && (nVar >= 1)) {
    a = 1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)nVar;
    n_t = (ptrdiff_t)mNonlinIneq;
    lda_t = (ptrdiff_t)ldJ;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &a, &JacIneqTrans->data[ldJ * (ineqJ0 - 1)],
          &lda_t, &lambda->data[ineqL0 - 1], &incx_t, &beta1, &workspace->data[0],
          &incy_t);
    a = -1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)nVar;
    n_t = (ptrdiff_t)mNonlinIneq;
    lda_t = (ptrdiff_t)ldJ;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &a, &JacIneqTrans_old->data[0], &lda_t,
          &lambda->data[ineqL0 - 1], &incx_t, &beta1, &workspace->data[0],
          &incy_t);
  }
}

/* End of code generation (computeDeltaLag.c) */
