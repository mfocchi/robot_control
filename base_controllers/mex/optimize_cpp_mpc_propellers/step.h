/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * step.h
 *
 * Code generation for function 'step'
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
boolean_T step(int32_T *STEP_TYPE, emxArray_real_T *Hessian, const
               emxArray_real_T *lb, const emxArray_real_T *ub, d_struct_T
               *TrialState, k_struct_T *MeritFunction, c_struct_T *memspace,
               j_struct_T *WorkingSet, g_struct_T *QRManager, h_struct_T
               *CholManager, i_struct_T *QPObjective, b_struct_T *qpoptions);

/* End of code generation (step.h) */
