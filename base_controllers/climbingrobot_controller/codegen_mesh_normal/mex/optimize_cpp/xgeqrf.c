/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgeqrf.c
 *
 * Code generation for function 'xgeqrf'
 *
 */

/* Include files */
#include "xgeqrf.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void xgeqrf(emxArray_real_T *A, int32_T m, int32_T n, emxArray_real_T *tau)
{
  real_T *A_data;
  real_T *tau_data;
  int32_T i;
  int32_T ma;
  int32_T minmana;
  int32_T na;
  A_data = A->data;
  ma = A->size[0];
  na = A->size[1];
  minmana = muIntScalarMin_sint32(ma, na);
  na = tau->size[0];
  tau->size[0] = minmana;
  emxEnsureCapacity_real_T(tau, na);
  tau_data = tau->data;
  if (n == 0) {
    na = tau->size[0];
    tau->size[0] = minmana;
    emxEnsureCapacity_real_T(tau, na);
    tau_data = tau->data;
    for (na = 0; na < minmana; na++) {
      tau_data[na] = 0.0;
    }
  } else {
    ptrdiff_t info_t;
    info_t = LAPACKE_dgeqrf(102, (ptrdiff_t)m, (ptrdiff_t)n, &A_data[0],
                            (ptrdiff_t)A->size[0], &tau_data[0]);
    if ((int32_T)info_t != 0) {
      for (na = 0; na < n; na++) {
        for (i = 0; i < m; i++) {
          A_data[na * ma + i] = rtNaN;
        }
      }
      na = muIntScalarMin_sint32(m, n) - 1;
      for (i = 0; i <= na; i++) {
        tau_data[i] = rtNaN;
      }
      na += 2;
      for (i = na; i <= minmana; i++) {
        tau_data[i - 1] = 0.0;
      }
    }
  }
}

/* End of code generation (xgeqrf.c) */
