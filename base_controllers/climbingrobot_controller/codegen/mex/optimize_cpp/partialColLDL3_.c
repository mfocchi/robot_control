/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * partialColLDL3_.c
 *
 * Code generation for function 'partialColLDL3_'
 *
 */

/* Include files */
#include "partialColLDL3_.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void partialColLDL3_(d_struct_T *obj, int32_T LD_offset, int32_T NColsRemain)
{
  int32_T LD_diagOffset;
  int32_T LDimSizeP1;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T ia;
  int32_T iac;
  int32_T idx;
  int32_T ix;
  int32_T iy0;
  int32_T j;
  int32_T k;
  int32_T subRows;
  LDimSizeP1 = obj->ldm + 1;
  i = NColsRemain - 1;
  for (k = 0; k < 48; k++) {
    real_T y;
    subRows = (NColsRemain - k) - 1;
    LD_diagOffset = (LD_offset + LDimSizeP1 * k) - 1;
    for (idx = 0; idx <= subRows; idx++) {
      obj->workspace_ = obj->FMat->data[LD_diagOffset + idx];
    }
    for (idx = 0; idx <= i; idx++) {
      obj->workspace2_ = obj->workspace_;
    }
    idx = obj->ldm;
    y = obj->workspace2_;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      i1 = obj->ldm * (k - 1) + 1;
      for (iac = 1; idx < 0 ? iac >= i1 : iac <= i1; iac += idx) {
        i2 = (iac + NColsRemain) - 1;
        for (ia = iac; ia <= i2; ia++) {
          y += obj->workspace_ * -obj->FMat->data[ix - 1];
        }
        ix += obj->ldm;
      }
    }
    obj->workspace2_ = y;
    for (idx = 0; idx <= i; idx++) {
      obj->workspace_ = y;
    }
    for (idx = 0; idx <= subRows; idx++) {
      obj->FMat->data[LD_diagOffset + idx] = obj->workspace_;
    }
    for (idx = 0; idx < subRows; idx++) {
      i1 = (LD_diagOffset + idx) + 1;
      obj->FMat->data[i1] /= obj->FMat->data[LD_diagOffset];
    }
  }
  for (j = 48; j <= i; j += 48) {
    int32_T i3;
    int32_T m;
    int32_T subBlockSize;
    int32_T y_tmp;
    y_tmp = NColsRemain - j;
    subBlockSize = muIntScalarMin_sint32(48, y_tmp);
    i1 = j + subBlockSize;
    i2 = i1 - 1;
    for (k = j; k <= i2; k++) {
      m = i1 - k;
      iy0 = (LD_offset + LDimSizeP1 * k) - 1;
      for (idx = 0; idx < 48; idx++) {
        obj->workspace2_ =
            obj->FMat->data[((LD_offset + k) + idx * obj->ldm) - 1];
      }
      ix = k + 1;
      idx = obj->ldm;
      if (m != 0) {
        i3 = (k + obj->ldm * 47) + 1;
        for (iac = ix; idx < 0 ? iac >= i3 : iac <= i3; iac += idx) {
          subRows = (iac + m) - 1;
          for (ia = iac; ia <= subRows; ia++) {
            LD_diagOffset = (iy0 + ia) - iac;
            obj->FMat->data[LD_diagOffset] +=
                obj->workspace_ * -obj->workspace2_;
          }
        }
      }
    }
    if (i1 < NColsRemain) {
      m = y_tmp - subBlockSize;
      k = ((LD_offset + subBlockSize) + LDimSizeP1 * j) - 1;
      i1 = subBlockSize - 1;
      for (idx = 0; idx < 48; idx++) {
        ix = (LD_offset + j) + idx * obj->ldm;
        for (subRows = 0; subRows <= i1; subRows++) {
          obj->workspace2_ = obj->FMat->data[(ix + subRows) - 1];
        }
      }
      y_tmp = obj->ldm;
      if ((m != 0) && (subBlockSize != 0)) {
        ix = k + obj->ldm * (subBlockSize - 1);
        subRows = 0;
        for (iy0 = k; y_tmp < 0 ? iy0 >= ix : iy0 <= ix; iy0 += y_tmp) {
          subRows++;
          i1 = subRows + y_tmp * 47;
          for (iac = subRows; y_tmp < 0 ? iac >= i1 : iac <= i1; iac += y_tmp) {
            i2 = iy0 + 1;
            i3 = iy0 + m;
            LD_diagOffset = ((((i3 - iy0) / 2) << 1) + iy0) + 1;
            idx = LD_diagOffset - 2;
            for (ia = i2; ia <= idx; ia += 2) {
              __m128d r;
              r = _mm_loadu_pd(&obj->FMat->data[ia - 1]);
              _mm_storeu_pd(&obj->FMat->data[ia - 1],
                            _mm_add_pd(r, _mm_set1_pd(-obj->workspace2_ *
                                                      obj->workspace_)));
            }
            for (ia = LD_diagOffset; ia <= i3; ia++) {
              obj->FMat->data[ia - 1] += -obj->workspace2_ * obj->workspace_;
            }
          }
        }
      }
    }
  }
}

/* End of code generation (partialColLDL3_.c) */
