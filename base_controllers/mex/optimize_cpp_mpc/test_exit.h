/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * test_exit.h
 *
 * Code generation for function 'test_exit'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_internal_types.h"
#include "optimize_cpp_mpc_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_test_exit(b_struct_T *Flags, f_struct_T *memspace,
                 struct_T *MeritFunction, const h_struct_T *WorkingSet,
                 g_struct_T *TrialState, c_struct_T *QRManager,
                 const emxArray_real_T *lb, const emxArray_real_T *ub);

boolean_T test_exit(struct_T *MeritFunction, const h_struct_T *WorkingSet,
                    g_struct_T *TrialState, const emxArray_real_T *lb,
                    const emxArray_real_T *ub, boolean_T *Flags_fevalOK,
                    boolean_T *Flags_done, boolean_T *Flags_stepAccepted,
                    boolean_T *Flags_failedLineSearch, int32_T *Flags_stepType);

/* End of code generation (test_exit.h) */
