/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeForwardDifferences.h
 *
 * Code generation for function 'computeForwardDifferences'
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
boolean_T computeForwardDifferences(f_struct_T *obj, real_T fCurrent,
  emxArray_real_T *xk, emxArray_real_T *gradf, const emxArray_real_T *lb, const
  emxArray_real_T *ub);

/* End of code generation (computeForwardDifferences.h) */
