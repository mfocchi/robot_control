/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compressBounds.c
 *
 * Code generation for function 'compressBounds'
 *
 */

/* Include files */
#include "compressBounds.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
int32_T compressBounds(int32_T nVar, emxArray_int32_T *indexLB,
                       emxArray_int32_T *indexUB, emxArray_int32_T *indexFixed,
                       const emxArray_real_T *lb, const emxArray_real_T *ub,
                       int32_T *mUB, int32_T *mFixed)
{
  const real_T *lb_data;
  const real_T *ub_data;
  int32_T idx;
  int32_T mLB;
  int32_T *indexFixed_data;
  int32_T *indexLB_data;
  int32_T *indexUB_data;
  ub_data = ub->data;
  lb_data = lb->data;
  indexFixed_data = indexFixed->data;
  indexUB_data = indexUB->data;
  indexLB_data = indexLB->data;
  mLB = 0;
  *mUB = 0;
  *mFixed = 0;
  if (ub->size[1] != 0) {
    if (lb->size[1] != 0) {
      for (idx = 0; idx < nVar; idx++) {
        real_T d;
        boolean_T guard1;
        d = lb_data[idx];
        guard1 = false;
        if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
          if (muDoubleScalarAbs(d - ub_data[idx]) < 0.001) {
            (*mFixed)++;
            indexFixed_data[*mFixed - 1] = idx + 1;
          } else {
            mLB++;
            indexLB_data[mLB - 1] = idx + 1;
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          d = ub_data[idx];
          if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
            (*mUB)++;
            indexUB_data[*mUB - 1] = idx + 1;
          }
        }
      }
    } else {
      for (idx = 0; idx < nVar; idx++) {
        real_T d;
        d = ub_data[idx];
        if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
          (*mUB)++;
          indexUB_data[*mUB - 1] = idx + 1;
        }
      }
    }
  } else if (lb->size[1] != 0) {
    for (idx = 0; idx < nVar; idx++) {
      real_T d;
      d = lb_data[idx];
      if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
        mLB++;
        indexLB_data[mLB - 1] = idx + 1;
      }
    }
  }
  return mLB;
}

/* End of code generation (compressBounds.c) */
