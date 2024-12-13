/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGradLag.h
 *
 * Code generation for function 'computeGradLag'
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
void computeGradLag(emxArray_real_T *workspace, int32_T ldA, int32_T nVar,
                    const emxArray_real_T *grad, int32_T mIneq,
                    const emxArray_real_T *AineqTrans,
                    const emxArray_int32_T *finiteFixed, int32_T mFixed,
                    const emxArray_int32_T *finiteLB, int32_T mLB,
                    const emxArray_int32_T *finiteUB, int32_T mUB,
                    const emxArray_real_T *lambda);

/* End of code generation (computeGradLag.h) */
