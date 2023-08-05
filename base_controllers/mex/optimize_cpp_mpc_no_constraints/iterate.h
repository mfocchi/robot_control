/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * iterate.h
 *
 * Code generation for function 'iterate'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void iterate(const emxArray_real_T *H, const emxArray_real_T *f, d_struct_T
             *solution, c_struct_T *memspace, j_struct_T *workingset, g_struct_T
             *qrmanager, h_struct_T *cholmanager, i_struct_T *objective, real_T
             options_StepTolerance, real_T options_ObjectiveLimit, int32_T
             runTimeOptions_MaxIterations);

/* End of code generation (iterate.h) */
