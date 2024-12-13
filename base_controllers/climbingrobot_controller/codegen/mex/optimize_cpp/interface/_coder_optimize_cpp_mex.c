/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_optimize_cpp_mex.c
 *
 * Code generation for function '_coder_optimize_cpp_mex'
 *
 */

/* Include files */
#include "_coder_optimize_cpp_mex.h"
#include "_coder_optimize_cpp_api.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_initialize.h"
#include "optimize_cpp_terminate.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&optimize_cpp_atexit);
  /* Module initialization. */
  optimize_cpp_initialize();
  /* Dispatch the entry-point. */
  unsafe_optimize_cpp_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  optimize_cpp_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "UTF-8", true);
  return emlrtRootTLSGlobal;
}

void unsafe_optimize_cpp_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                     int32_T nrhs, const mxArray *prhs[6])
{
  const mxArray *outputs;
  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 6, 4, 12, "optimize_cpp");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "optimize_cpp");
  }
  /* Call the function. */
  optimize_cpp_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation (_coder_optimize_cpp_mex.c) */
