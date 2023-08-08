/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * ixamax.c
 *
 * Code generation for function 'ixamax'
 *
 */

/* Include files */
#include "ixamax.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
int32_T ixamax(int32_T n, const emxArray_real_T *x)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  int32_T idxmax;
  if (n < 1) {
    idxmax = 0;
  } else {
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)1;
    n_t = idamax(&n_t, &x->data[0], &incx_t);
    idxmax = (int32_T)n_t;
  }

  return idxmax;
}

/* End of code generation (ixamax.c) */
