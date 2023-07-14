/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * feasibleX0ForWorkingSet.h
 *
 * Code generation for function 'feasibleX0ForWorkingSet'
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
boolean_T feasibleX0ForWorkingSet(emxArray_real_T *workspace,
                                  emxArray_real_T *xCurrent,
                                  const h_struct_T *workingset,
                                  c_struct_T *qrmanager);

/* End of code generation (feasibleX0ForWorkingSet.h) */
