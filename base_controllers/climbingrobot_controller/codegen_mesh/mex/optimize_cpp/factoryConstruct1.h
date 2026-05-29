/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct1.h
 *
 * Code generation for function 'factoryConstruct1'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_factoryConstruct(
    const real_T objfun_workspace_p0[3], const param *objfun_workspace_params,
    const real_T nonlin_workspace_p0[3], const real_T nonlin_workspace_pf[3],
    real_T nonlin_workspace_Fleg_max, real_T nonlin_workspace_mu,
    const param *nonlin_workspace_params, int32_T nVar, int32_T mCineq,
    const emxArray_real_T *lb, const emxArray_real_T *ub, k_struct_T *obj);

/* End of code generation (factoryConstruct1.h) */
