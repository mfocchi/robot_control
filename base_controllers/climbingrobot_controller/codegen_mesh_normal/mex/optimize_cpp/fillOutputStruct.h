/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fillOutputStruct.h
 *
 * Code generation for function 'fillOutputStruct'
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
real_T fillOutputStruct(int32_T nVar, int32_T TrialState_FunctionEvaluations,
                        int32_T TrialState_sqpIterations,
                        real_T TrialState_steplength,
                        const emxArray_real_T *TrialState_delta_x,
                        real_T c_MeritFunction_nlpPrimalFeasEr,
                        real_T MeritFunction_firstOrderOpt,
                        real_T *output_funcCount, char_T output_algorithm[3],
                        real_T *output_constrviolation, real_T *output_stepsize,
                        real_T *output_lssteplength,
                        real_T *output_firstorderopt);

/* End of code generation (fillOutputStruct.h) */
