/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_optimize_cpp_mpc_mex.c
 *
 * Code generation for function '_coder_optimize_cpp_mpc_mex'
 *
 */

/* Include files */
#include "_coder_optimize_cpp_mpc_mex.h"
#include "_coder_optimize_cpp_mpc_api.h"
#include "optimize_cpp_mpc_data.h"
#include "optimize_cpp_mpc_initialize.h"
#include "optimize_cpp_mpc_terminate.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&optimize_cpp_mpc_atexit);
  /* Module initialization. */
  optimize_cpp_mpc_initialize();
  /* Dispatch the entry-point. */
  unsafe_optimize_cpp_mpc_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  optimize_cpp_mpc_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "UTF-8", true);
  return emlrtRootTLSGlobal;
}

void unsafe_optimize_cpp_mpc_mexFunction(int32_T nlhs, mxArray *plhs[3],
                                         int32_T nrhs, const mxArray *prhs[8])
{
  const mxArray *outputs[3];
  int32_T i;
  /* Check for proper number of arguments. */
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 8, 4, 16, "optimize_cpp_mpc");
  }
  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 16,
                        "optimize_cpp_mpc");
  }
  /* Call the function. */
  optimize_cpp_mpc_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_optimize_cpp_mpc_mex.c) */
