/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * vecnorm.c
 *
 * Code generation for function 'vecnorm'
 *
 */

/* Include files */
#include "vecnorm.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void vecnorm(const emxArray_real_T *x, emxArray_real_T *y)
{
  const real_T *x_data;
  real_T *y_data;
  int32_T ix0;
  int32_T j;
  int32_T k;
  int32_T ncols;
  x_data = x->data;
  ncols = x->size[1];
  ix0 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, ix0);
  y_data = y->data;
  for (j = 0; j < ncols; j++) {
    real_T b_y;
    real_T scale;
    int32_T kend;
    ix0 = j * 3;
    b_y = 0.0;
    scale = 3.3121686421112381E-170;
    kend = ix0 + 3;
    for (k = ix0 + 1; k <= kend; k++) {
      real_T absxk;
      absxk = muDoubleScalarAbs(x_data[k - 1]);
      if (absxk > scale) {
        real_T t;
        t = scale / absxk;
        b_y = b_y * t * t + 1.0;
        scale = absxk;
      } else {
        real_T t;
        t = absxk / scale;
        b_y += t * t;
      }
    }
    y_data[j] = scale * muDoubleScalarSqrt(b_y);
  }
}

/* End of code generation (vecnorm.c) */
