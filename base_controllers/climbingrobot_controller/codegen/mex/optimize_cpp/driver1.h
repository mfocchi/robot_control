/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * driver1.h
 *
 * Code generation for function 'driver1'
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
void b_driver(const emxArray_real_T *H, const emxArray_real_T *f,
              g_struct_T *solution, f_struct_T *memspace,
              h_struct_T *workingset, c_struct_T *qrmanager,
              d_struct_T *cholmanager, e_struct_T *objective,
              l_struct_T *options, int32_T runTimeOptions_MaxIterations);

/* End of code generation (driver1.h) */
