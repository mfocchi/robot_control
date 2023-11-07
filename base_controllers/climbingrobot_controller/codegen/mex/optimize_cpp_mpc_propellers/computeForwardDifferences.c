/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeForwardDifferences.c
 *
 * Code generation for function 'computeForwardDifferences'
 *
 */

/* Include files */
#include "computeForwardDifferences.h"
#include "optimize_cpp_mpc_propellers.h"
#include "optimize_cpp_mpc_propellers_data.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
boolean_T computeForwardDifferences(f_struct_T *obj, real_T fCurrent, const
  emxArray_real_T *cIneqCurrent, int32_T ineq0, emxArray_real_T *xk,
  emxArray_real_T *gradf, emxArray_real_T *JacCineqTrans, int32_T CineqColStart,
  const emxArray_real_T *lb, const emxArray_real_T *ub)
{
  emxArray_real_T *r;
  emxArray_real_T *varargout_1;
  real_T deltaX;
  real_T lbDiff;
  real_T ubDiff;
  int32_T b_idx;
  int32_T i;
  int32_T idx;
  boolean_T evalOK;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T modifiedStep;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  evalOK = true;
  obj->numEvals = 0;
  idx = 0;
  emxInit_real_T(&r, 1, true);
  emxInit_real_T(&varargout_1, 2, true);
  exitg1 = false;
  while ((!exitg1) && (idx <= obj->nVar - 1)) {
    modifiedStep = false;
    deltaX = 1.4901161193847656E-8 * (1.0 - 2.0 * (real_T)(xk->data[idx] < 0.0))
      * muDoubleScalarMax(muDoubleScalarAbs(xk->data[idx]), 1.0);
    if (obj->hasLB->data[idx] || obj->hasUB->data[idx]) {
      if (obj->hasLB->data[idx] && obj->hasUB->data[idx]) {
        ubDiff = deltaX;
        if ((lb->data[idx] != ub->data[idx]) && (xk->data[idx] >= lb->data[idx])
            && (xk->data[idx] <= ub->data[idx]) && ((xk->data[idx] + deltaX >
              ub->data[idx]) || (xk->data[idx] + deltaX < lb->data[idx]))) {
          ubDiff = -deltaX;
          modifiedStep = true;
          lbDiff = xk->data[idx] + -deltaX;
          if ((lbDiff > ub->data[idx]) || (lbDiff < lb->data[idx])) {
            lbDiff = xk->data[idx] - lb->data[idx];
            ubDiff = ub->data[idx] - xk->data[idx];
            if (lbDiff <= ubDiff) {
              ubDiff = -lbDiff;
            }
          }
        }

        deltaX = ubDiff;
      } else if (obj->hasUB->data[idx]) {
        if ((xk->data[idx] <= ub->data[idx]) && (xk->data[idx] + deltaX >
             ub->data[idx])) {
          deltaX = -deltaX;
          modifiedStep = true;
        }
      } else {
        if ((xk->data[idx] >= lb->data[idx]) && (xk->data[idx] + deltaX <
             lb->data[idx])) {
          deltaX = -deltaX;
          modifiedStep = true;
        }
      }
    }

    i = r->size[0];
    r->size[0] = obj->cIneq_1->size[0];
    emxEnsureCapacity_real_T(r, i);
    b_idx = obj->cIneq_1->size[0];
    for (i = 0; i < b_idx; i++) {
      r->data[i] = obj->cIneq_1->data[i];
    }

    lbDiff = xk->data[idx];
    xk->data[idx] += deltaX;
    ubDiff = b_anon(obj->objfun.tunableEnvironment.f1,
                    obj->objfun.tunableEnvironment.f3,
                    obj->objfun.tunableEnvironment.f4,
                    obj->objfun.tunableEnvironment.f5,
                    obj->objfun.tunableEnvironment.f6,
                    obj->objfun.tunableEnvironment.f7.int_method,
                    obj->objfun.tunableEnvironment.f7.int_steps,
                    obj->objfun.tunableEnvironment.f7.b,
                    obj->objfun.tunableEnvironment.f7.p_a1,
                    obj->objfun.tunableEnvironment.f7.p_a2,
                    obj->objfun.tunableEnvironment.f7.g,
                    obj->objfun.tunableEnvironment.f7.m,
                    obj->objfun.tunableEnvironment.f7.w1,
                    obj->objfun.tunableEnvironment.f7.w2,
                    obj->objfun.tunableEnvironment.f7.mpc_dt, xk);
    evalOK = ((!muDoubleScalarIsInf(ubDiff)) && (!muDoubleScalarIsNaN(ubDiff)));
    if (evalOK) {
      anon(obj->nonlin.tunableEnvironment.f2, obj->nonlin.tunableEnvironment.f3,
           obj->nonlin.tunableEnvironment.f4, xk, varargout_1);
      i = r->size[0];
      r->size[0] = obj->cIneq_1->size[0];
      emxEnsureCapacity_real_T(r, i);
      b_idx = obj->cIneq_1->size[0];
      for (i = 0; i < b_idx; i++) {
        r->data[i] = varargout_1->data[i];
      }

      b_idx = 0;
      while (evalOK && (b_idx + 1 <= obj->mIneq)) {
        evalOK = ((!muDoubleScalarIsInf(varargout_1->data[b_idx])) &&
                  (!muDoubleScalarIsNaN(varargout_1->data[b_idx])));
        b_idx++;
      }

      if (evalOK) {
        xk->data[idx] = lbDiff;
      }
    }

    obj->f_1 = ubDiff;
    i = obj->cIneq_1->size[0];
    obj->cIneq_1->size[0] = r->size[0];
    emxEnsureCapacity_real_T(obj->cIneq_1, i);
    b_idx = r->size[0];
    for (i = 0; i < b_idx; i++) {
      obj->cIneq_1->data[i] = r->data[i];
    }

    obj->numEvals++;
    guard1 = false;
    if (!evalOK) {
      if (!modifiedStep) {
        deltaX = -deltaX;
        if (obj->hasLB->data[idx] && (xk->data[idx] + deltaX >= lb->data[idx]) &&
            obj->hasUB->data[idx] && (xk->data[idx] + deltaX <= ub->data[idx]))
        {
          modifiedStep = true;
        } else {
          modifiedStep = false;
        }

        if ((!obj->hasBounds) || modifiedStep) {
          i = obj->cIneq_1->size[0];
          obj->cIneq_1->size[0] = r->size[0];
          emxEnsureCapacity_real_T(obj->cIneq_1, i);
          b_idx = r->size[0];
          for (i = 0; i < b_idx; i++) {
            obj->cIneq_1->data[i] = r->data[i];
          }

          lbDiff = xk->data[idx];
          xk->data[idx] += deltaX;
          ubDiff = b_anon(obj->objfun.tunableEnvironment.f1,
                          obj->objfun.tunableEnvironment.f3,
                          obj->objfun.tunableEnvironment.f4,
                          obj->objfun.tunableEnvironment.f5,
                          obj->objfun.tunableEnvironment.f6,
                          obj->objfun.tunableEnvironment.f7.int_method,
                          obj->objfun.tunableEnvironment.f7.int_steps,
                          obj->objfun.tunableEnvironment.f7.b,
                          obj->objfun.tunableEnvironment.f7.p_a1,
                          obj->objfun.tunableEnvironment.f7.p_a2,
                          obj->objfun.tunableEnvironment.f7.g,
                          obj->objfun.tunableEnvironment.f7.m,
                          obj->objfun.tunableEnvironment.f7.w1,
                          obj->objfun.tunableEnvironment.f7.w2,
                          obj->objfun.tunableEnvironment.f7.mpc_dt, xk);
          evalOK = ((!muDoubleScalarIsInf(ubDiff)) && (!muDoubleScalarIsNaN
                     (ubDiff)));
          if (evalOK) {
            anon(obj->nonlin.tunableEnvironment.f2,
                 obj->nonlin.tunableEnvironment.f3,
                 obj->nonlin.tunableEnvironment.f4, xk, varargout_1);
            i = obj->cIneq_1->size[0];
            obj->cIneq_1->size[0] = r->size[0];
            emxEnsureCapacity_real_T(obj->cIneq_1, i);
            b_idx = r->size[0];
            for (i = 0; i < b_idx; i++) {
              obj->cIneq_1->data[i] = varargout_1->data[i];
            }

            b_idx = 0;
            while (evalOK && (b_idx + 1 <= obj->mIneq)) {
              evalOK = ((!muDoubleScalarIsInf(varargout_1->data[b_idx])) &&
                        (!muDoubleScalarIsNaN(varargout_1->data[b_idx])));
              b_idx++;
            }

            if (evalOK) {
              xk->data[idx] = lbDiff;
            }
          }

          obj->f_1 = ubDiff;
          obj->numEvals++;
        }
      }

      if (!evalOK) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      gradf->data[idx] = (obj->f_1 - fCurrent) / deltaX;
      i = obj->mIneq;
      for (b_idx = 0; b_idx < i; b_idx++) {
        JacCineqTrans->data[idx + JacCineqTrans->size[0] * ((CineqColStart +
          b_idx) - 1)] = (obj->cIneq_1->data[b_idx] - cIneqCurrent->data[(ineq0
          + b_idx) - 1]) / deltaX;
      }

      idx++;
    }
  }

  emxFree_real_T(&varargout_1);
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return evalOK;
}

/* End of code generation (computeForwardDifferences.c) */
