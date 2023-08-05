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
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeDeltaLag(int32_T nVar, emxArray_real_T *workspace, const
                     emxArray_real_T *grad, const emxArray_real_T *grad_old)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  real_T a;
  int32_T i;
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
}

/* End of code generation (computeDeltaLag.c) */
