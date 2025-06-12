/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * updatePenaltyParam.c
 *
 * Code generation for function 'updatePenaltyParam'
 *
 */

/* Include files */
#include "updatePenaltyParam.h"
#include "optimize_cpp_mpc_no_constraints_internal_types.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void updatePenaltyParam(k_struct_T *obj, real_T fval, int32_T sqpiter, real_T
  qpval, const emxArray_real_T *x, int32_T iReg0, int32_T nRegularized)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  real_T linearizedConstrViolPrev;
  real_T penaltyParamTrial;
  penaltyParamTrial = obj->penaltyParam;
  linearizedConstrViolPrev = obj->linearizedConstrViol;
  if (nRegularized < 1) {
    obj->linearizedConstrViol = 0.0;
  } else {
    n_t = (ptrdiff_t)nRegularized;
    incx_t = (ptrdiff_t)1;
    obj->linearizedConstrViol = dasum(&n_t, &x->data[iReg0 - 1], &incx_t);
  }

  linearizedConstrViolPrev -= obj->linearizedConstrViol;
  if ((linearizedConstrViolPrev > 2.2204460492503131E-16) && (qpval > 0.0)) {
    if (fval == 0.0) {
      penaltyParamTrial = 1.0;
    } else {
      penaltyParamTrial = 1.5;
    }

    penaltyParamTrial = penaltyParamTrial * qpval / linearizedConstrViolPrev;
  }

  if (penaltyParamTrial < obj->penaltyParam) {
    obj->phi = fval;
    if (obj->initFval - fval > (real_T)obj->nPenaltyDecreases * obj->threshold)
    {
      obj->nPenaltyDecreases++;
      if ((obj->nPenaltyDecreases << 1) > sqpiter) {
        obj->threshold *= 10.0;
      }

      obj->penaltyParam = muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
    } else {
      obj->phi = fval + obj->penaltyParam * 0.0;
    }
  } else {
    obj->penaltyParam = muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
    obj->phi = fval + obj->penaltyParam * 0.0;
  }

  obj->phiPrimePlus = muDoubleScalarMin(qpval - obj->penaltyParam * 0.0, 0.0);
}

/* End of code generation (updatePenaltyParam.c) */
