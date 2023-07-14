/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * step.h
 *
 * Code generation for function 'step'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
boolean_T step(int32_T *STEP_TYPE, emxArray_real_T *Hessian,
               const emxArray_real_T *lb, const emxArray_real_T *ub,
               g_struct_T *TrialState, struct_T *MeritFunction,
               f_struct_T *memspace, h_struct_T *WorkingSet,
               c_struct_T *QRManager, d_struct_T *CholManager,
               e_struct_T *QPObjective, l_struct_T *qpoptions);

/* End of code generation (step.h) */
