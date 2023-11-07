/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * driver1.h
 *
 * Code generation for function 'driver1'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_driver(const emxArray_real_T *H, const emxArray_real_T *f, d_struct_T
              *solution, c_struct_T *memspace, j_struct_T *workingset,
              g_struct_T *qrmanager, h_struct_T *cholmanager, i_struct_T
              *objective, b_struct_T *options, int32_T
              runTimeOptions_MaxIterations);

/* End of code generation (driver1.h) */
