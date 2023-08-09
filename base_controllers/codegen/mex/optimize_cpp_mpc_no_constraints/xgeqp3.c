/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xgeqp3.c
 *
 * Code generation for function 'xgeqp3'
 *
 */

/* Include files */
#include "xgeqp3.h"
#include "optimize_cpp_mpc_no_constraints_data.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void xgeqp3(emxArray_real_T *A, int32_T m, int32_T n, emxArray_int32_T *jpvt,
            emxArray_real_T *tau)
{
  ptrdiff_t info_t;
  emxArray_ptrdiff_t *jpvt_t;
  int32_T i;
  int32_T ma;
  int32_T minmana;
  int32_T na;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  ma = A->size[0];
  na = A->size[1];
  minmana = muIntScalarMin_sint32(ma, na);
  i = tau->size[0];
  tau->size[0] = minmana;
  emxEnsureCapacity_real_T(tau, i);
  emxInit_ptrdiff_t(&jpvt_t, 1, true);
  if ((m < 1) || (n < 1)) {
    i = tau->size[0];
    tau->size[0] = minmana;
    emxEnsureCapacity_real_T(tau, i);
    for (i = 0; i < minmana; i++) {
      tau->data[i] = 0.0;
    }

    for (ma = 0; ma < n; ma++) {
      jpvt->data[ma] = ma + 1;
    }
  } else {
    i = jpvt_t->size[0];
    jpvt_t->size[0] = jpvt->size[0];
    emxEnsureCapacity_ptrdiff_t(jpvt_t, i);
    na = jpvt->size[0];
    for (i = 0; i < na; i++) {
      jpvt_t->data[i] = (ptrdiff_t)jpvt->data[i];
    }

    info_t = LAPACKE_dgeqp3(102, (ptrdiff_t)m, (ptrdiff_t)n, &A->data[0],
      (ptrdiff_t)A->size[0], &jpvt_t->data[0], &tau->data[0]);
    if ((int32_T)info_t != 0) {
      for (na = 0; na < n; na++) {
        for (i = 0; i < m; i++) {
          A->data[na * ma + i] = rtNaN;
        }
      }

      na = muIntScalarMin_sint32(m, n) - 1;
      for (ma = 0; ma <= na; ma++) {
        tau->data[ma] = rtNaN;
      }

      i = na + 2;
      for (ma = i; ma <= minmana; ma++) {
        tau->data[ma - 1] = 0.0;
      }

      for (ma = 0; ma < n; ma++) {
        jpvt->data[ma] = ma + 1;
      }
    } else {
      for (ma = 0; ma < n; ma++) {
        jpvt->data[ma] = (int32_T)jpvt_t->data[ma];
      }
    }
  }

  emxFree_ptrdiff_t(&jpvt_t);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (xgeqp3.c) */
