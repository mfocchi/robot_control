/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeDualFeasError.c
 *
 * Code generation for function 'computeDualFeasError'
 *
 */

/* Include files */
#include "computeDualFeasError.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
boolean_T computeDualFeasError(int32_T nVar, const emxArray_real_T *gradLag,
                               real_T *val)
{
  const real_T *gradLag_data;
  int32_T idx;
  boolean_T exitg1;
  boolean_T gradOK;
  gradLag_data = gradLag->data;
  gradOK = true;
  *val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= nVar - 1)) {
    gradOK = ((!muDoubleScalarIsInf(gradLag_data[idx])) &&
              (!muDoubleScalarIsNaN(gradLag_data[idx])));
    if (!gradOK) {
      exitg1 = true;
    } else {
      *val = muDoubleScalarMax(*val, muDoubleScalarAbs(gradLag_data[idx]));
      idx++;
    }
  }
  return gradOK;
}

/* End of code generation (computeDualFeasError.c) */
