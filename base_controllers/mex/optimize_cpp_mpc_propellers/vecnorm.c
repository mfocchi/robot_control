/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * vecnorm.c
 *
 * Code generation for function 'vecnorm'
 *
 */

/* Include files */
#include "vecnorm.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void vecnorm(const emxArray_real_T *x, emxArray_real_T *y)
{
  real_T absxk;
  real_T b_y;
  real_T scale;
  real_T t;
  int32_T ix0;
  int32_T j;
  int32_T k;
  int32_T kend;
  int32_T ncols;
  ncols = x->size[1];
  ix0 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, ix0);
  for (j = 0; j < ncols; j++) {
    ix0 = j * 3;
    b_y = 0.0;
    scale = 3.3121686421112381E-170;
    kend = ix0 + 3;
    for (k = ix0 + 1; k <= kend; k++) {
      absxk = muDoubleScalarAbs(x->data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        b_y = b_y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        b_y += t * t;
      }
    }

    y->data[j] = scale * muDoubleScalarSqrt(b_y);
  }
}

/* End of code generation (vecnorm.c) */
