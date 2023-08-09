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
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void updatePenaltyParam(k_struct_T *obj, real_T fval, const emxArray_real_T
  *ineq_workspace, int32_T mIneq, int32_T sqpiter, real_T qpval, const
  emxArray_real_T *x, int32_T iReg0, int32_T nRegularized)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  real_T constrViolationIneq;
  real_T linearizedConstrViolPrev;
  real_T penaltyParamTrial;
  int32_T idx;
  penaltyParamTrial = obj->penaltyParam;
  constrViolationIneq = 0.0;
  for (idx = 0; idx < mIneq; idx++) {
    if (ineq_workspace->data[idx] > 0.0) {
      constrViolationIneq += ineq_workspace->data[idx];
    }
  }

  linearizedConstrViolPrev = obj->linearizedConstrViol;
  if (nRegularized < 1) {
    obj->linearizedConstrViol = 0.0;
  } else {
    n_t = (ptrdiff_t)nRegularized;
    incx_t = (ptrdiff_t)1;
    obj->linearizedConstrViol = dasum(&n_t, &x->data[iReg0 - 1], &incx_t);
  }

  linearizedConstrViolPrev = (constrViolationIneq + linearizedConstrViolPrev) -
    obj->linearizedConstrViol;
  if ((linearizedConstrViolPrev > 2.2204460492503131E-16) && (qpval > 0.0)) {
    if (fval == 0.0) {
      penaltyParamTrial = 1.0;
    } else {
      penaltyParamTrial = 1.5;
    }

    penaltyParamTrial = penaltyParamTrial * qpval / linearizedConstrViolPrev;
  }

  if (penaltyParamTrial < obj->penaltyParam) {
    obj->phi = fval + penaltyParamTrial * constrViolationIneq;
    if ((obj->initFval + penaltyParamTrial * obj->initConstrViolationIneq) -
        obj->phi > (real_T)obj->nPenaltyDecreases * obj->threshold) {
      obj->nPenaltyDecreases++;
      if ((obj->nPenaltyDecreases << 1) > sqpiter) {
        obj->threshold *= 10.0;
      }

      obj->penaltyParam = muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
    } else {
      obj->phi = fval + obj->penaltyParam * constrViolationIneq;
    }
  } else {
    obj->penaltyParam = muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
    obj->phi = fval + obj->penaltyParam * constrViolationIneq;
  }

  obj->phiPrimePlus = muDoubleScalarMin(qpval - obj->penaltyParam *
    constrViolationIneq, 0.0);
}

/* End of code generation (updatePenaltyParam.c) */
