/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factorQR.c
 *
 * Code generation for function 'factorQR'
 *
 */

/* Include files */
#include "factorQR.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "xgeqrf.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void factorQR(c_struct_T *obj, const emxArray_real_T *A, int32_T mrows,
              int32_T ncols, int32_T ldA)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  const real_T *A_data;
  int32_T idx;
  int32_T scalarLB;
  boolean_T guard1;
  A_data = A->data;
  scalarLB = mrows * ncols;
  guard1 = false;
  if (scalarLB > 0) {
    for (idx = 0; idx < ncols; idx++) {
      n_t = (ptrdiff_t)mrows;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, (real_T *)&A_data[ldA * idx], &incx_t,
            &obj->QR->data[obj->ldq * idx], &incy_t);
    }
    guard1 = true;
  } else if (scalarLB == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    int32_T vectorUB;
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    scalarLB = (ncols / 4) << 2;
    vectorUB = scalarLB - 4;
    for (idx = 0; idx <= vectorUB; idx += 4) {
      _mm_storeu_si128(
          (__m128i *)&obj->jpvt->data[idx],
          _mm_add_epi32(_mm_add_epi32(_mm_set1_epi32(idx),
                                      _mm_loadu_si128((const __m128i *)&iv[0])),
                        _mm_set1_epi32(1)));
    }
    for (idx = scalarLB; idx < ncols; idx++) {
      obj->jpvt->data[idx] = idx + 1;
    }
    obj->minRowCol = muIntScalarMin_sint32(mrows, ncols);
    xgeqrf(obj->QR, mrows, ncols, obj->tau);
  }
}

/* End of code generation (factorQR.c) */
