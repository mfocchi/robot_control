/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * wallSurfaceEval.h
 *
 * Code generation for function 'wallSurfaceEval'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
real_T wallSurfaceEval(real_T z_query, real_T y_query,
                       const real_T params_mesh_x[10000],
                       const real_T params_mesh_y[10000],
                       const real_T params_mesh_z[10000]);

/* End of code generation (wallSurfaceEval.h) */
