/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fullColLDL2_.c
 *
 * Code generation for function 'fullColLDL2_'
 *
 */

/* Include files */
#include "fullColLDL2_.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void fullColLDL2_(d_struct_T *obj, int32_T LD_offset, int32_T NColsRemain)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  real_T alpha1;
  int32_T LDimSizeP1;
  int32_T ijA;
  int32_T j;
  int32_T jA;
  int32_T k;
  LDimSizeP1 = obj->ldm;
  for (k = 0; k < NColsRemain; k++) {
    real_T y;
    int32_T LD_diagOffset;
    int32_T subMatrixDim;
    LD_diagOffset = LD_offset + (LDimSizeP1 + 1) * k;
    alpha1 = -1.0 / obj->FMat->data[LD_diagOffset - 1];
    subMatrixDim = (NColsRemain - k) - 2;
    y = obj->workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj->FMat->data[LD_diagOffset + jA];
    }
    obj->workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = (LD_diagOffset + LDimSizeP1) - 1;
      for (j = 0; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          real_T temp;
          int32_T i;
          int32_T i1;
          int32_T scalarLB;
          int32_T vectorUB;
          temp = y * alpha1;
          i = jA + 2;
          i1 = subMatrixDim + jA;
          scalarLB = (((((i1 - jA) + 1) / 2) << 1) + jA) + 2;
          vectorUB = scalarLB - 2;
          for (ijA = i; ijA <= vectorUB; ijA += 2) {
            __m128d r;
            r = _mm_loadu_pd(&obj->FMat->data[ijA - 1]);
            _mm_storeu_pd(&obj->FMat->data[ijA - 1],
                          _mm_add_pd(r, _mm_set1_pd(y * temp)));
          }
          for (ijA = scalarLB; ijA <= i1 + 2; ijA++) {
            obj->FMat->data[ijA - 1] += y * temp;
          }
        }
        jA += obj->ldm;
      }
    }
    alpha1 = 1.0 / obj->FMat->data[LD_diagOffset - 1];
    if (subMatrixDim + 1 >= 1) {
      n_t = (ptrdiff_t)(subMatrixDim + 1);
      incx_t = (ptrdiff_t)1;
      dscal(&n_t, &alpha1, &obj->FMat->data[LD_diagOffset], &incx_t);
    }
  }
}

/* End of code generation (fullColLDL2_.c) */
