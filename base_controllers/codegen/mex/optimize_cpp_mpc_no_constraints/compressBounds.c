/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * compressBounds.c
 *
 * Code generation for function 'compressBounds'
 *
 */

/* Include files */
#include "compressBounds.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void compressBounds(int32_T nVar, emxArray_int32_T *indexLB, emxArray_int32_T
                    *indexUB, emxArray_int32_T *indexFixed, const
                    emxArray_real_T *lb, const emxArray_real_T *ub, int32_T *mLB,
                    int32_T *mUB, int32_T *mFixed)
{
  real_T d;
  int32_T idx;
  boolean_T guard1 = false;
  *mLB = 0;
  *mUB = 0;
  *mFixed = 0;
  if (ub->size[1] != 0) {
    if (lb->size[1] != 0) {
      for (idx = 0; idx < nVar; idx++) {
        d = lb->data[idx];
        guard1 = false;
        if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
          if (muDoubleScalarAbs(d - ub->data[idx]) < 0.001) {
            (*mFixed)++;
            indexFixed->data[*mFixed - 1] = idx + 1;
          } else {
            (*mLB)++;
            indexLB->data[*mLB - 1] = idx + 1;
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          d = ub->data[idx];
          if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
            (*mUB)++;
            indexUB->data[*mUB - 1] = idx + 1;
          }
        }
      }
    } else {
      for (idx = 0; idx < nVar; idx++) {
        d = ub->data[idx];
        if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
          (*mUB)++;
          indexUB->data[*mUB - 1] = idx + 1;
        }
      }
    }
  } else {
    if (lb->size[1] != 0) {
      for (idx = 0; idx < nVar; idx++) {
        d = lb->data[idx];
        if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
          (*mLB)++;
          indexLB->data[*mLB - 1] = idx + 1;
        }
      }
    }
  }
}

/* End of code generation (compressBounds.c) */
