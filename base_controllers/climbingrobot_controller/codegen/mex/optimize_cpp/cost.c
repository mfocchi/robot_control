/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cost.c
 *
 * Code generation for function 'cost'
 *
 */

/* Include files */
#include "cost.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void b_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                        int32_T in3, int32_T in4, const emxArray_real_T *in5)
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
  i = (in4 - in3) + 1;
  if (in5->size[1] == 1) {
    loop_ub = i;
  } else {
    loop_ub = in5->size[1];
  }
  stride_0_1 = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(in1, stride_0_1);
  in1_data = in1->data;
  stride_0_1 = (i != 1);
  stride_1_1 = (in5->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] =
        in2_data[in3 + i * stride_0_1] * in5_data[6 * (i * stride_1_1) + 5];
  }
}

void c_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                        int32_T in3, int32_T in4, const emxArray_real_T *in5)
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
  i = (in4 - in3) + 1;
  if (in5->size[1] == 1) {
    loop_ub = i;
  } else {
    loop_ub = in5->size[1];
  }
  stride_0_1 = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(in1, stride_0_1);
  in1_data = in1->data;
  stride_0_1 = (i != 1);
  stride_1_1 = (in5->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] =
        in2_data[in3 + i * stride_0_1] * in5_data[6 * (i * stride_1_1) + 4];
  }
}

/* End of code generation (cost.c) */
