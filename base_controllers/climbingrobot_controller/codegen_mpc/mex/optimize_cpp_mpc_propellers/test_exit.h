/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * test_exit.h
 *
 * Code generation for function 'test_exit'
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
void test_exit(struct_T *Flags, c_struct_T *memspace, k_struct_T *MeritFunction,
               const emxArray_real_T *fscales_cineq_constraint, j_struct_T
               *WorkingSet, d_struct_T *TrialState, g_struct_T *QRManager, const
               emxArray_real_T *lb, const emxArray_real_T *ub);

/* End of code generation (test_exit.h) */
