/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diff.c
 *
 * Code generation for function 'diff'
 *
 */

/* Include files */
#include "diff.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void diff(const emxArray_real_T *x, emxArray_real_T *y)
{
  const real_T *x_data;
  real_T *y_data;
  int32_T dimSize;
  int32_T m;
  x_data = x->data;
  dimSize = x->size[1];
  if (x->size[1] == 0) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    m = x->size[1] - 1;
    if (muIntScalarMin_sint32(m, 1) < 1) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else {
      m = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = x->size[1] - 1;
      emxEnsureCapacity_real_T(y, m);
      y_data = y->data;
      if (x->size[1] - 1 != 0) {
        real_T work_data;
        work_data = x_data[0];
        for (m = 2; m <= dimSize; m++) {
          real_T d;
          real_T tmp1;
          tmp1 = x_data[m - 1];
          d = tmp1;
          tmp1 -= work_data;
          work_data = d;
          y_data[m - 2] = tmp1;
        }
      }
    }
  }
}

/* End of code generation (diff.c) */
