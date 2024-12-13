/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fmincon.h
 *
 * Code generation for function 'fmincon'
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
real_T fmincon(const real_T fun_workspace_p0[3],
               const param *fun_workspace_params, const emxArray_real_T *x0,
               const emxArray_real_T *lb, const emxArray_real_T *ub,
               const real_T nonlcon_workspace_p0[3],
               const real_T nonlcon_workspace_pf[3],
               real_T nonlcon_workspace_Fleg_max, real_T nonlcon_workspace_mu,
               const param *nonlcon_workspace_params, emxArray_real_T *x,
               char_T output_algorithm[3], real_T *exitflag,
               real_T *output_iterations, real_T *output_funcCount,
               real_T *output_constrviolation, real_T *output_stepsize,
               real_T *output_lssteplength, real_T *output_firstorderopt);

/* End of code generation (fmincon.h) */
