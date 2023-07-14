/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * relaxed.h
 *
 * Code generation for function 'relaxed'
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
void relaxed(const emxArray_real_T *Hessian, const emxArray_real_T *grad,
             g_struct_T *TrialState, struct_T *MeritFunction,
             f_struct_T *memspace, h_struct_T *WorkingSet,
             c_struct_T *QRManager, d_struct_T *CholManager,
             e_struct_T *QPObjective, k_struct_T *qpoptions);

/* End of code generation (relaxed.h) */
