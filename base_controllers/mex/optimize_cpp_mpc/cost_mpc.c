/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cost_mpc.c
 *
 * Code generation for function 'cost_mpc'
 *
 */

/* Include files */
#include "cost_mpc.h"
#include "optimize_cpp_mpc_data.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "vecnorm.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                      int32_T in4, const emxArray_real_T *in5)
{
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  const real_T *in5_data;
  real_T *b_in2_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in5_data = in5->data;
  in2_data = in2->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&b_in2, 2);
  i = b_in2->size[0] * b_in2->size[1];
  b_in2->size[0] = 3;
  if (in5->size[1] == 1) {
    loop_ub = in4;
  } else {
    loop_ub = in5->size[1];
  }
  b_in2->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_in2, i);
  b_in2_data = b_in2->data;
  stride_0_1 = (in4 != 1);
  stride_1_1 = (in5->size[1] != 1);
  aux_0_1 = -1;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in2_data[3 * (aux_0_1 + 1)]);
    r1 = _mm_loadu_pd(&in5_data[3 * aux_1_1]);
    _mm_storeu_pd(&b_in2_data[3 * i], _mm_sub_pd(r, r1));
    b_in2_data[3 * i + 2] =
        in2_data[3 * (aux_0_1 + 1) + 2] - in5_data[3 * aux_1_1 + 2];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  vecnorm(b_in2, in1);
  emxFree_real_T(&b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (cost_mpc.c) */
