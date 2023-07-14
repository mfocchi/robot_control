/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgeqp3.c
 *
 * Code generation for function 'xgeqp3'
 *
 */

/* Include files */
#include "xgeqp3.h"
#include "optimize_cpp_mpc_data.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void xgeqp3(emxArray_real_T *A, int32_T m, int32_T n, emxArray_int32_T *jpvt,
            emxArray_real_T *tau)
{
  ptrdiff_t *jpvt_t_data;
  emxArray_ptrdiff_t *jpvt_t;
  real_T *A_data;
  real_T *tau_data;
  int32_T i;
  int32_T k;
  int32_T ma;
  int32_T minmana;
  int32_T na;
  int32_T *jpvt_data;
  jpvt_data = jpvt->data;
  A_data = A->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  ma = A->size[0];
  na = A->size[1];
  minmana = muIntScalarMin_sint32(ma, na);
  i = tau->size[0];
  tau->size[0] = minmana;
  emxEnsureCapacity_real_T(tau, i);
  tau_data = tau->data;
  emxInit_ptrdiff_t(&jpvt_t);
  if ((m < 1) || (n < 1)) {
    i = tau->size[0];
    tau->size[0] = minmana;
    emxEnsureCapacity_real_T(tau, i);
    tau_data = tau->data;
    for (i = 0; i < minmana; i++) {
      tau_data[i] = 0.0;
    }
    na = (n / 4) << 2;
    ma = na - 4;
    for (k = 0; k <= ma; k += 4) {
      _mm_storeu_si128(
          (__m128i *)&jpvt_data[k],
          _mm_add_epi32(_mm_add_epi32(_mm_set1_epi32(k),
                                      _mm_loadu_si128((const __m128i *)&iv[0])),
                        _mm_set1_epi32(1)));
    }
    for (k = na; k < n; k++) {
      jpvt_data[k] = k + 1;
    }
  } else {
    ptrdiff_t info_t;
    i = jpvt_t->size[0];
    jpvt_t->size[0] = jpvt->size[0];
    emxEnsureCapacity_ptrdiff_t(jpvt_t, i);
    jpvt_t_data = jpvt_t->data;
    na = jpvt->size[0];
    for (i = 0; i < na; i++) {
      jpvt_t_data[i] = (ptrdiff_t)jpvt_data[i];
    }
    info_t =
        LAPACKE_dgeqp3(102, (ptrdiff_t)m, (ptrdiff_t)n, &A_data[0],
                       (ptrdiff_t)A->size[0], &jpvt_t_data[0], &tau_data[0]);
    if ((int32_T)info_t != 0) {
      for (na = 0; na < n; na++) {
        for (i = 0; i < m; i++) {
          A_data[na * ma + i] = rtNaN;
        }
      }
      na = muIntScalarMin_sint32(m, n) - 1;
      for (k = 0; k <= na; k++) {
        tau_data[k] = rtNaN;
      }
      i = na + 2;
      for (k = i; k <= minmana; k++) {
        tau_data[k - 1] = 0.0;
      }
      na = (n / 4) << 2;
      ma = na - 4;
      for (k = 0; k <= ma; k += 4) {
        _mm_storeu_si128(
            (__m128i *)&jpvt_data[k],
            _mm_add_epi32(
                _mm_add_epi32(_mm_set1_epi32(k),
                              _mm_loadu_si128((const __m128i *)&iv[0])),
                _mm_set1_epi32(1)));
      }
      for (k = na; k < n; k++) {
        jpvt_data[k] = k + 1;
      }
    } else {
      for (k = 0; k < n; k++) {
        jpvt_data[k] = (int32_T)jpvt_t_data[k];
      }
    }
  }
  emxFree_ptrdiff_t(&jpvt_t);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (xgeqp3.c) */
