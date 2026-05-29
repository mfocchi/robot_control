/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_initialize.c
 *
 * Code generation for function 'optimize_cpp_initialize'
 *
 */

/* Include files */
#include "optimize_cpp_initialize.h"
#include "_coder_optimize_cpp_mex.h"
#include "optimize_cpp_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include <string.h>

/* Function Declarations */
static void optimize_cpp_once(void);

/* Function Definitions */
static void optimize_cpp_once(void)
{
  mex_InitInfAndNan();
  savedTime_not_empty_init();
}

void optimize_cpp_initialize(void)
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
    optimize_cpp_once();
  }
}

/* End of code generation (optimize_cpp_initialize.c) */
