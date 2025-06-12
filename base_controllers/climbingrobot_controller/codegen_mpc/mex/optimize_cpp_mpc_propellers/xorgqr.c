/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xorgqr.c
 *
 * Code generation for function 'xorgqr'
 *
 */

/* Include files */
#include "xorgqr.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void xorgqr(int32_T m, int32_T n, int32_T k, emxArray_real_T *A, int32_T lda,
            const emxArray_real_T *tau)
{
  ptrdiff_t info_t;
  int32_T A_idx_0;
  int32_T A_idx_1;
  int32_T i;
  int32_T i1;
  info_t = LAPACKE_dorgqr(102, (ptrdiff_t)m, (ptrdiff_t)n, (ptrdiff_t)k,
    &A->data[0], (ptrdiff_t)lda, &tau->data[0]);
  if ((int32_T)info_t != 0) {
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
}

/* End of code generation (xorgqr.c) */
