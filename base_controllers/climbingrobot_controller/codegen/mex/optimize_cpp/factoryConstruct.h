/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct.h
 *
 * Code generation for function 'factoryConstruct'
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
void factoryConstruct(int32_T nVarMax, int32_T mConstrMax, int32_T mIneq,
                      const emxArray_real_T *x0, int32_T mNonlinIneq,
                      g_struct_T *obj);

/* End of code generation (factoryConstruct.h) */
