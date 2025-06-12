/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * BFGSUpdate.h
 *
 * Code generation for function 'BFGSUpdate'
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
boolean_T BFGSUpdate(int32_T nvar, emxArray_real_T *Bk, const emxArray_real_T
                     *sk, emxArray_real_T *yk, emxArray_real_T *workspace);

/* End of code generation (BFGSUpdate.h) */
