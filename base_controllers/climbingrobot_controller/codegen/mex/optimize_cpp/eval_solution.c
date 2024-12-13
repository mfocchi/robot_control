/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eval_solution.c
 *
 * Code generation for function 'eval_solution'
 *
 */

/* Include files */
#include "eval_solution.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void binary_expand_op(struct0_T *in1, const emxArray_real_T *in3,
                      const emxArray_real_T *in4)
{
  emxArray_real_T *r;
  const real_T *in3_data;
  const real_T *in4_data;
  real_T *r1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  int32_T stride_2_1;
  in4_data = in4->data;
  in3_data = in3->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&r, 2);
  i = r->size[0] * r->size[1];
  r->size[0] = 1;
  if (in4->size[1] == 1) {
    if (in3->size[1] == 1) {
      loop_ub = in1->c->size[1];
    } else {
      loop_ub = in3->size[1];
    }
  } else {
    loop_ub = in4->size[1];
  }
  r->size[1] = loop_ub;
  emxEnsureCapacity_real_T(r, i);
  r1 = r->data;
  stride_0_1 = (in1->c->size[1] != 1);
  stride_1_1 = (in3->size[1] != 1);
  stride_2_1 = (in4->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    real_T b_varargin_1;
    real_T c_varargin_1;
    real_T varargin_1;
    varargin_1 = in1->c->data[i * stride_0_1];
    b_varargin_1 = in3_data[i * stride_1_1];
    c_varargin_1 = in4_data[i * stride_2_1];
    r1[i] = (varargin_1 * varargin_1 + b_varargin_1 * b_varargin_1) +
            c_varargin_1 * c_varargin_1;
  }
  i = in1->c->size[0] * in1->c->size[1];
  in1->c->size[0] = 1;
  in1->c->size[1] = r->size[1];
  emxEnsureCapacity_real_T(in1->c, i);
  loop_ub = r->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1->c->data[i] = r1[i];
  }
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (eval_solution.c) */
