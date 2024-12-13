/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * linearForm_.h
 *
 * Code generation for function 'linearForm_'
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
void linearForm_(boolean_T obj_hasLinear, int32_T obj_nvar,
                 emxArray_real_T *workspace, const emxArray_real_T *H,
                 const emxArray_real_T *f, const emxArray_real_T *x);

/* End of code generation (linearForm_.h) */
