/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xasum.c
 *
 * Code generation for function 'xasum'
 *
 */

/* Include files */
#include "xasum.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T xasum(int32_T n, const emxArray_real_T *x, int32_T ix0)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  real_T y;
  if (n < 1) {
    y = 0.0;
  } else {
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)1;
    y = dasum(&n_t, &x->data[ix0 - 1], &incx_t);
  }

  return y;
}

/* End of code generation (xasum.c) */
