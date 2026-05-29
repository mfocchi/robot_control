/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fillOutputStruct.c
 *
 * Code generation for function 'fillOutputStruct'
 *
 */

/* Include files */
#include "fillOutputStruct.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T fillOutputStruct(int32_T nVar, int32_T TrialState_FunctionEvaluations,
                        int32_T TrialState_sqpIterations,
                        real_T TrialState_steplength,
                        const emxArray_real_T *TrialState_delta_x,
                        real_T c_MeritFunction_nlpPrimalFeasEr,
                        real_T MeritFunction_firstOrderOpt,
                        real_T *output_funcCount, char_T output_algorithm[3],
                        real_T *output_constrviolation, real_T *output_stepsize,
                        real_T *output_lssteplength,
                        real_T *output_firstorderopt)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  const real_T *TrialState_delta_x_data;
  real_T output_iterations;
  TrialState_delta_x_data = TrialState_delta_x->data;
  output_algorithm[0] = 's';
  output_algorithm[1] = 'q';
  output_algorithm[2] = 'p';
  output_iterations = TrialState_sqpIterations;
  *output_funcCount = TrialState_FunctionEvaluations;
  *output_constrviolation = c_MeritFunction_nlpPrimalFeasEr;
  if (nVar < 1) {
    *output_stepsize = 0.0;
  } else {
    n_t = (ptrdiff_t)nVar;
    incx_t = (ptrdiff_t)1;
    *output_stepsize =
        dnrm2(&n_t, (real_T *)&TrialState_delta_x_data[0], &incx_t);
  }
  *output_lssteplength = TrialState_steplength;
  *output_firstorderopt = MeritFunction_firstOrderOpt;
  return output_iterations;
}

/* End of code generation (fillOutputStruct.c) */
