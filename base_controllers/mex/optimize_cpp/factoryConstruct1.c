/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct1.c
 *
 * Code generation for function 'factoryConstruct1'
 *
 */

/* Include files */
#include "factoryConstruct1.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void b_factoryConstruct(const real_T objfun_tunableEnvironment_f1[3], const
  real_T objfun_tunableEnvironment_f2[3], const param
  *objfun_tunableEnvironment_f3, const real_T nonlin_tunableEnvironment_f1[3],
  const real_T nonlin_tunableEnvironment_f2[3], real_T
  nonlin_tunableEnvironment_f3, real_T nonlin_tunableEnvironment_f4, real_T
  nonlin_tunableEnvironment_f5, const param *nonlin_tunableEnvironment_f6,
  int32_T nVar, int32_T mCineq, const emxArray_real_T *lb, const emxArray_real_T
  *ub, e_struct_T *obj)
{
  int32_T idx;
  boolean_T b;
  obj->objfun.tunableEnvironment.f3 = *objfun_tunableEnvironment_f3;
  obj->objfun.tunableEnvironment.f1[0] = objfun_tunableEnvironment_f1[0];
  obj->objfun.tunableEnvironment.f2[0] = objfun_tunableEnvironment_f2[0];
  obj->nonlin.tunableEnvironment.f1[0] = nonlin_tunableEnvironment_f1[0];
  obj->nonlin.tunableEnvironment.f2[0] = nonlin_tunableEnvironment_f2[0];
  obj->objfun.tunableEnvironment.f1[1] = objfun_tunableEnvironment_f1[1];
  obj->objfun.tunableEnvironment.f2[1] = objfun_tunableEnvironment_f2[1];
  obj->nonlin.tunableEnvironment.f1[1] = nonlin_tunableEnvironment_f1[1];
  obj->nonlin.tunableEnvironment.f2[1] = nonlin_tunableEnvironment_f2[1];
  obj->objfun.tunableEnvironment.f1[2] = objfun_tunableEnvironment_f1[2];
  obj->objfun.tunableEnvironment.f2[2] = objfun_tunableEnvironment_f2[2];
  obj->nonlin.tunableEnvironment.f1[2] = nonlin_tunableEnvironment_f1[2];
  obj->nonlin.tunableEnvironment.f2[2] = nonlin_tunableEnvironment_f2[2];
  obj->nonlin.tunableEnvironment.f3 = nonlin_tunableEnvironment_f3;
  obj->nonlin.tunableEnvironment.f4 = nonlin_tunableEnvironment_f4;
  obj->nonlin.tunableEnvironment.f5 = nonlin_tunableEnvironment_f5;
  obj->nonlin.tunableEnvironment.f6 = *nonlin_tunableEnvironment_f6;
  obj->f_1 = 0.0;
  idx = obj->cIneq_1->size[0];
  obj->cIneq_1->size[0] = mCineq;
  emxEnsureCapacity_real_T(obj->cIneq_1, idx);
  obj->f_2 = 0.0;
  idx = obj->cIneq_2->size[0];
  obj->cIneq_2->size[0] = mCineq;
  emxEnsureCapacity_real_T(obj->cIneq_2, idx);
  obj->nVar = nVar;
  obj->mIneq = mCineq;
  obj->mEq = 0;
  obj->numEvals = 0;
  obj->SpecifyObjectiveGradient = false;
  obj->SpecifyConstraintGradient = false;
  idx = obj->hasLB->size[0];
  obj->hasLB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasLB, idx);
  idx = obj->hasUB->size[0];
  obj->hasUB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasUB, idx);
  obj->FiniteDifferenceType = 0;
  b = false;
  idx = 0;
  while ((!b) && (idx + 1 <= nVar)) {
    obj->hasLB->data[idx] = ((!muDoubleScalarIsInf(lb->data[idx])) &&
      (!muDoubleScalarIsNaN(lb->data[idx])));
    obj->hasUB->data[idx] = ((!muDoubleScalarIsInf(ub->data[idx])) &&
      (!muDoubleScalarIsNaN(ub->data[idx])));
    if (obj->hasLB->data[idx] || obj->hasUB->data[idx]) {
      b = true;
    }

    idx++;
  }

  while (idx + 1 <= nVar) {
    obj->hasLB->data[idx] = ((!muDoubleScalarIsInf(lb->data[idx])) &&
      (!muDoubleScalarIsNaN(lb->data[idx])));
    obj->hasUB->data[idx] = ((!muDoubleScalarIsInf(ub->data[idx])) &&
      (!muDoubleScalarIsNaN(ub->data[idx])));
    idx++;
  }

  obj->hasBounds = b;
}

/* End of code generation (factoryConstruct1.c) */
