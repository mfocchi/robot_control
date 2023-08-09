/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mpc_propellers_initialize.c
 *
 * Code generation for function 'optimize_cpp_mpc_propellers_initialize'
 *
 */

/* Include files */
#include "optimize_cpp_mpc_propellers_initialize.h"
#include "_coder_optimize_cpp_mpc_propellers_mex.h"
#include "optimize_cpp_mpc_propellers_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void optimize_cpp_mpc_propellers_initialize(void)
{
  mex_InitInfAndNan();
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, 0);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLicenseCheckR2012b(emlrtRootTLSGlobal, "optimization_toolbox", 2);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp_mpc_propellers_initialize.c) */
