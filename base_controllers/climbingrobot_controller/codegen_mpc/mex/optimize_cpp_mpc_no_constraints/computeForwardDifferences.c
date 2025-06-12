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
#include "optimize_cpp_mpc_no_constraints.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
boolean_T computeForwardDifferences(f_struct_T *obj, real_T fCurrent,
  emxArray_real_T *xk, emxArray_real_T *gradf, const emxArray_real_T *lb, const
  emxArray_real_T *ub)
{
  real_T deltaX;
  real_T lbDiff;
  real_T ubDiff;
  int32_T idx;
  boolean_T evalOK;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T modifiedStep;
  evalOK = true;
  obj->numEvals = 0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= obj->nVar - 1)) {
    modifiedStep = false;
    deltaX = 1.4901161193847656E-8 * (1.0 - 2.0 * (real_T)(xk->data[idx] < 0.0))
      * muDoubleScalarMax(muDoubleScalarAbs(xk->data[idx]), 1.0);
    if (obj->hasLB->data[idx] || obj->hasUB->data[idx]) {
      if (obj->hasLB->data[idx] && obj->hasUB->data[idx]) {
        lbDiff = deltaX;
        if ((lb->data[idx] != ub->data[idx]) && (xk->data[idx] >= lb->data[idx])
            && (xk->data[idx] <= ub->data[idx]) && ((xk->data[idx] + deltaX >
              ub->data[idx]) || (xk->data[idx] + deltaX < lb->data[idx]))) {
          lbDiff = -deltaX;
          modifiedStep = true;
          ubDiff = xk->data[idx] + -deltaX;
          if ((ubDiff > ub->data[idx]) || (ubDiff < lb->data[idx])) {
            lbDiff = xk->data[idx] - lb->data[idx];
            ubDiff = ub->data[idx] - xk->data[idx];
            if (lbDiff <= ubDiff) {
              lbDiff = -lbDiff;
            } else {
              lbDiff = ubDiff;
            }
          }
        }

        deltaX = lbDiff;
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

    lbDiff = xk->data[idx];
    xk->data[idx] += deltaX;
    ubDiff = anon(obj->objfun.tunableEnvironment.f1,
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
      xk->data[idx] = lbDiff;
    }

    obj->f_1 = ubDiff;
    obj->numEvals++;
    guard1 = false;
    if (!evalOK) {
      if (!modifiedStep) {
        deltaX = -deltaX;
        if (obj->hasLB->data[idx]) {
          ubDiff = xk->data[idx] + deltaX;
          if ((ubDiff >= lb->data[idx]) && obj->hasUB->data[idx] && (ubDiff <=
               ub->data[idx])) {
            modifiedStep = true;
          } else {
            modifiedStep = false;
          }
        } else {
          modifiedStep = false;
        }

        if ((!obj->hasBounds) || modifiedStep) {
          lbDiff = xk->data[idx];
          xk->data[idx] += deltaX;
          ubDiff = anon(obj->objfun.tunableEnvironment.f1,
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
            xk->data[idx] = lbDiff;
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
      idx++;
    }
  }

  return evalOK;
}

/* End of code generation (computeForwardDifferences.c) */
