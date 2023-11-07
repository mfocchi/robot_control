/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xpotrf.c
 *
 * Code generation for function 'xpotrf'
 *
 */

/* Include files */
#include "xpotrf.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
int32_T xpotrf(int32_T n, emxArray_real_T *A, int32_T lda)
{
  ptrdiff_t info_t;
  int32_T A_idx_0;
  int32_T A_idx_1;
  int32_T i;
  int32_T i1;
  info_t = LAPACKE_dpotrf_work(102, 'U', (ptrdiff_t)n, &A->data[0], (ptrdiff_t)
    lda);
  if ((int32_T)info_t < 0) {
    A_idx_0 = A->size[0];
    A_idx_1 = A->size[1];
    i = A->size[0] * A->size[1];
    A->size[0] = A_idx_0;
    A->size[1] = A_idx_1;
    emxEnsureCapacity_real_T(A, i);
    for (i = 0; i < A_idx_1; i++) {
      for (i1 = 0; i1 < A_idx_0; i1++) {
        A->data[i1 + A->size[0] * i] = rtNaN;
      }
    }
  }

  return (int32_T)info_t;
}

/* End of code generation (xpotrf.c) */
