/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eval_pos_vel_mpc.c
 *
 * Code generation for function 'eval_pos_vel_mpc'
 *
 */

/* Include files */
#include "eval_pos_vel_mpc.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void b_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                        int32_T in4, const emxArray_real_T *in5, int32_T in6,
                        int32_T in7)
{
  const real_T *in2_data;
  const real_T *in5_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in5_data = in5->data;
  in2_data = in2->data;
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  emxEnsureCapacity_real_T(in1, i);
  i = (in7 - in6) + 1;
  if (i == 1) {
    loop_ub = in4;
  } else {
    loop_ub = i;
  }
  stride_0_1 = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(in1, stride_0_1);
  in1_data = in1->data;
  stride_0_1 = (in4 != 1);
  stride_1_1 = (i != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] =
        in2_data[i * stride_0_1] + in5_data[(in6 + i * stride_1_1) - 1];
  }
}

/* End of code generation (eval_pos_vel_mpc.c) */
