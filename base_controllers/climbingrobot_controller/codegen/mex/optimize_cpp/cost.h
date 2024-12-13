/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cost.h
 *
 * Code generation for function 'cost'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                        int32_T in3, int32_T in4, const emxArray_real_T *in5);

void c_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                        int32_T in3, int32_T in4, const emxArray_real_T *in5);

/* End of code generation (cost.h) */
