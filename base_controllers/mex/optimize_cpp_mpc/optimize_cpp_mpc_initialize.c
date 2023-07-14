/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mpc_initialize.c
 *
 * Code generation for function 'optimize_cpp_mpc_initialize'
 *
 */

/* Include files */
#include "optimize_cpp_mpc_initialize.h"
#include "_coder_optimize_cpp_mpc_mex.h"
#include "optimize_cpp_mpc_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void optimize_cpp_mpc_once(void);

/* Function Definitions */
static void optimize_cpp_mpc_once(void)
{
  mex_InitInfAndNan();
}

void optimize_cpp_mpc_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar =
      emlrtGetBreakCheckFlagAddressR2022b(emlrtRootTLSGlobal);
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, NULL);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLicenseCheckR2022a(emlrtRootTLSGlobal,
                          "EMLRT:runTime:MexFunctionNeedsLicense",
                          "optimization_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    optimize_cpp_mpc_once();
  }
}

/* End of code generation (optimize_cpp_mpc_initialize.c) */
