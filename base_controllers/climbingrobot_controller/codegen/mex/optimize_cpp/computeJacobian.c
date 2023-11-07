/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeJacobian.c
 *
 * Code generation for function 'computeJacobian'
 *
 */

/* Include files */
#include "computeJacobian.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void computeJacobian(const real_T p[3], const real_T params_p_a1[3], const
                     real_T params_p_a2[3], real_T J[6])
{
  real_T J_tmp_idx_0;
  real_T J_tmp_idx_1;
  real_T J_tmp_idx_2;
  real_T absxk;
  real_T b_J_tmp_idx_0;
  real_T b_J_tmp_idx_1;
  real_T b_J_tmp_idx_2;
  real_T b_scale;
  real_T b_y;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  J_tmp_idx_2 = p[0] - params_p_a1[0];
  J_tmp_idx_0 = J_tmp_idx_2;
  b_J_tmp_idx_2 = p[0] - params_p_a2[0];
  b_J_tmp_idx_0 = b_J_tmp_idx_2;
  absxk = muDoubleScalarAbs(J_tmp_idx_2);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = muDoubleScalarAbs(b_J_tmp_idx_2);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  J_tmp_idx_2 = p[1] - params_p_a1[1];
  J_tmp_idx_1 = J_tmp_idx_2;
  b_J_tmp_idx_2 = p[1] - params_p_a2[1];
  b_J_tmp_idx_1 = b_J_tmp_idx_2;
  absxk = muDoubleScalarAbs(J_tmp_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = muDoubleScalarAbs(b_J_tmp_idx_2);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  J_tmp_idx_2 = p[2] - params_p_a1[2];
  b_J_tmp_idx_2 = p[2] - params_p_a2[2];
  absxk = muDoubleScalarAbs(J_tmp_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = muDoubleScalarAbs(b_J_tmp_idx_2);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  y = scale * muDoubleScalarSqrt(y);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  J[0] = J_tmp_idx_0 / y;
  J[3] = b_J_tmp_idx_0 / b_y;
  J[1] = J_tmp_idx_1 / y;
  J[4] = b_J_tmp_idx_1 / b_y;
  J[2] = J_tmp_idx_2 / y;
  J[5] = b_J_tmp_idx_2 / b_y;
}

/* End of code generation (computeJacobian.c) */
