/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * countsort.h
 *
 * Code generation for function 'countsort'
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
void countsort(emxArray_int32_T *x, int32_T xLen, emxArray_int32_T *workspace,
               int32_T xMin, int32_T xMax);

/* End of code generation (countsort.h) */
