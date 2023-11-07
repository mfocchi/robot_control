/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * forwardKin.h
 *
 * Code generation for function 'forwardKin'
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
void b_forwardKin(const emlrtStack *sp, real_T params_b, real_T psi, real_T l1,
                  real_T l2, real_T psid, real_T l1d, real_T l2d, real_T *px,
                  real_T *py, real_T *pz, real_T *pdx, real_T *pdy, real_T *pdz);
void forwardKin(const emlrtStack *sp, real_T params_b, real_T psi, real_T l1,
                real_T l2, real_T *px, real_T *py, real_T *pz);

/* End of code generation (forwardKin.h) */
