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
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void b_factoryConstruct(const real_T objfun_tunableEnvironment_f1[6], real_T
  objfun_tunableEnvironment_f2, const emxArray_real_T
  *objfun_tunableEnvironment_f3, const emxArray_real_T
  *objfun_tunableEnvironment_f4, const emxArray_real_T
  *objfun_tunableEnvironment_f5, int64_T objfun_tunableEnvironment_f6, const
  param *objfun_tunableEnvironment_f7, int32_T nVar, const emxArray_real_T *lb,
  const emxArray_real_T *ub, f_struct_T *obj)
{
  int32_T b_i;
  int32_T i;
  boolean_T b;
  for (i = 0; i < 6; i++) {
    obj->objfun.tunableEnvironment.f1[i] = objfun_tunableEnvironment_f1[i];
  }

  obj->objfun.tunableEnvironment.f2 = objfun_tunableEnvironment_f2;
  b_i = obj->objfun.tunableEnvironment.f3->size[0] *
    obj->objfun.tunableEnvironment.f3->size[1];
  obj->objfun.tunableEnvironment.f3->size[0] = 3;
  obj->objfun.tunableEnvironment.f3->size[1] =
    objfun_tunableEnvironment_f3->size[1];
  emxEnsureCapacity_real_T(obj->objfun.tunableEnvironment.f3, b_i);
  i = objfun_tunableEnvironment_f3->size[0] * objfun_tunableEnvironment_f3->
    size[1];
  for (b_i = 0; b_i < i; b_i++) {
    obj->objfun.tunableEnvironment.f3->data[b_i] =
      objfun_tunableEnvironment_f3->data[b_i];
  }

  b_i = obj->objfun.tunableEnvironment.f4->size[0] *
    obj->objfun.tunableEnvironment.f4->size[1];
  obj->objfun.tunableEnvironment.f4->size[0] = 1;
  obj->objfun.tunableEnvironment.f4->size[1] =
    objfun_tunableEnvironment_f4->size[1];
  emxEnsureCapacity_real_T(obj->objfun.tunableEnvironment.f4, b_i);
  i = objfun_tunableEnvironment_f4->size[0] * objfun_tunableEnvironment_f4->
    size[1];
  for (b_i = 0; b_i < i; b_i++) {
    obj->objfun.tunableEnvironment.f4->data[b_i] =
      objfun_tunableEnvironment_f4->data[b_i];
  }

  b_i = obj->objfun.tunableEnvironment.f5->size[0] *
    obj->objfun.tunableEnvironment.f5->size[1];
  obj->objfun.tunableEnvironment.f5->size[0] = 1;
  obj->objfun.tunableEnvironment.f5->size[1] =
    objfun_tunableEnvironment_f5->size[1];
  emxEnsureCapacity_real_T(obj->objfun.tunableEnvironment.f5, b_i);
  i = objfun_tunableEnvironment_f5->size[0] * objfun_tunableEnvironment_f5->
    size[1];
  for (b_i = 0; b_i < i; b_i++) {
    obj->objfun.tunableEnvironment.f5->data[b_i] =
      objfun_tunableEnvironment_f5->data[b_i];
  }

  obj->objfun.tunableEnvironment.f6 = objfun_tunableEnvironment_f6;
  obj->objfun.tunableEnvironment.f7 = *objfun_tunableEnvironment_f7;
  obj->f_1 = 0.0;
  obj->f_2 = 0.0;
  obj->nVar = nVar;
  obj->mIneq = 0;
  obj->mEq = 0;
  obj->numEvals = 0;
  obj->SpecifyObjectiveGradient = false;
  obj->SpecifyConstraintGradient = false;
  b_i = obj->hasLB->size[0];
  obj->hasLB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasLB, b_i);
  b_i = obj->hasUB->size[0];
  obj->hasUB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasUB, b_i);
  obj->FiniteDifferenceType = 0;
  b = false;
  i = 0;
  switch ((uint32_T)(ub->size[1] == 0) << 1 | (lb->size[1] == 0)) {
   case 0U:
    while ((!b) && (i + 1 <= nVar)) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb->data[i])) &&
        (!muDoubleScalarIsNaN(lb->data[i])));
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub->data[i])) &&
        (!muDoubleScalarIsNaN(ub->data[i])));
      if (obj->hasLB->data[i] || obj->hasUB->data[i]) {
        b = true;
      }

      i++;
    }

    while (i + 1 <= nVar) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb->data[i])) &&
        (!muDoubleScalarIsNaN(lb->data[i])));
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub->data[i])) &&
        (!muDoubleScalarIsNaN(ub->data[i])));
      i++;
    }
    break;

   case 1U:
    while ((!b) && (i + 1 <= nVar)) {
      obj->hasLB->data[i] = false;
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub->data[i])) &&
        (!muDoubleScalarIsNaN(ub->data[i])));
      b = obj->hasUB->data[i];
      i++;
    }

    while (i + 1 <= nVar) {
      obj->hasLB->data[i] = false;
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub->data[i])) &&
        (!muDoubleScalarIsNaN(ub->data[i])));
      i++;
    }
    break;

   case 2U:
    while ((!b) && (i + 1 <= nVar)) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb->data[i])) &&
        (!muDoubleScalarIsNaN(lb->data[i])));
      obj->hasUB->data[i] = false;
      b = obj->hasLB->data[i];
      i++;
    }

    while (i + 1 <= nVar) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb->data[i])) &&
        (!muDoubleScalarIsNaN(lb->data[i])));
      obj->hasUB->data[i] = false;
      i++;
    }
    break;

   default:
    while (i + 1 <= nVar) {
      obj->hasLB->data[i] = false;
      obj->hasUB->data[i] = false;
      i++;
    }
    break;
  }

  obj->hasBounds = b;
}

/* End of code generation (factoryConstruct1.c) */
