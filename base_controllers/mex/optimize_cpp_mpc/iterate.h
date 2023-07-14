/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * iterate.h
 *
 * Code generation for function 'iterate'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void iterate(const emxArray_real_T *H, const emxArray_real_T *f,
             g_struct_T *solution, f_struct_T *memspace, h_struct_T *workingset,
             c_struct_T *qrmanager, d_struct_T *cholmanager,
             e_struct_T *objective, const char_T options_SolverName[7],
             real_T options_StepTolerance, real_T options_ObjectiveLimit,
             int32_T runTimeOptions_MaxIterations);

/* End of code generation (iterate.h) */
