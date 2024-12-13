/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mexutil.h
 *
 * Code generation for function 'optimize_cpp_mexutil'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
real_T b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *parentId);

void disp(const mxArray *m, emlrtMCInfo *location);

real_T emlrt_marshallIn(const mxArray *a__output_of_feval_,
                        const char_T *identifier);

real_T j_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *msgId);

/* End of code generation (optimize_cpp_mexutil.h) */
