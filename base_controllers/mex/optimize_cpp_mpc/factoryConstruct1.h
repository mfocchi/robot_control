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
#include "optimize_cpp_mpc_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_factoryConstruct(const real_T objfun_workspace_actual_state[6],
                        const emxArray_real_T *objfun_workspace_ref_com,
                        const emxArray_real_T *objfun_workspace_Fr_l0,
                        const emxArray_real_T *objfun_workspace_Fr_r0,
                        int64_T objfun_workspace_mpc_N,
                        const param *objfun_workspace_params, int32_T nVar,
                        const emxArray_real_T *lb, const emxArray_real_T *ub,
                        j_struct_T *obj);

/* End of code generation (factoryConstruct1.h) */
