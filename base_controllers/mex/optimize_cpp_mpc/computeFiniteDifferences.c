/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeFiniteDifferences.c
 *
 * Code generation for function 'computeFiniteDifferences'
 *
 */

/* Include files */
#include "computeFiniteDifferences.h"
#include "optimize_cpp_mpc.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
boolean_T computeFiniteDifferences(j_struct_T *obj, real_T fCurrent,
                                   emxArray_real_T *xk, emxArray_real_T *gradf,
                                   const emxArray_real_T *lb,
                                   const emxArray_real_T *ub)
{
  const real_T *lb_data;
  const real_T *ub_data;
  real_T *gradf_data;
  real_T *xk_data;
  int32_T idx;
  boolean_T evalOK;
  boolean_T exitg1;
  ub_data = ub->data;
  lb_data = lb->data;
  gradf_data = gradf->data;
  xk_data = xk->data;
  evalOK = true;
  obj->numEvals = 0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= obj->nVar - 1)) {
    real_T deltaX;
    real_T lbDiff;
    real_T ubDiff;
    boolean_T guard1;
    boolean_T modifiedStep;
    modifiedStep = false;
    deltaX = 1.4901161193847656E-8 *
             (1.0 - 2.0 * (real_T)(xk_data[idx] < 0.0)) *
             muDoubleScalarMax(muDoubleScalarAbs(xk_data[idx]), 1.0);
    if (obj->hasLB->data[idx] || obj->hasUB->data[idx]) {
      if (obj->hasLB->data[idx] && obj->hasUB->data[idx]) {
        lbDiff = deltaX;
        if ((lb_data[idx] != ub_data[idx]) && (xk_data[idx] >= lb_data[idx]) &&
            (xk_data[idx] <= ub_data[idx])) {
          ubDiff = xk_data[idx] + deltaX;
          if ((ubDiff > ub_data[idx]) || (ubDiff < lb_data[idx])) {
            lbDiff = -deltaX;
            modifiedStep = true;
            ubDiff = xk_data[idx] - deltaX;
            if ((ubDiff > ub_data[idx]) || (ubDiff < lb_data[idx])) {
              lbDiff = xk_data[idx] - lb_data[idx];
              ubDiff = ub_data[idx] - xk_data[idx];
              if (lbDiff <= ubDiff) {
                lbDiff = -lbDiff;
              } else {
                lbDiff = ubDiff;
              }
            }
          }
        }
        deltaX = lbDiff;
      } else if (obj->hasUB->data[idx]) {
        if ((xk_data[idx] <= ub_data[idx]) &&
            (xk_data[idx] + deltaX > ub_data[idx])) {
          deltaX = -deltaX;
          modifiedStep = true;
        }
      } else if ((xk_data[idx] >= lb_data[idx]) &&
                 (xk_data[idx] + deltaX < lb_data[idx])) {
        deltaX = -deltaX;
        modifiedStep = true;
      }
    }
    lbDiff = xk_data[idx];
    xk_data[idx] += deltaX;
    ubDiff = optimize_cpp_mpc_anonFcn1(
        obj->objfun.workspace.actual_state, obj->objfun.workspace.ref_com,
        obj->objfun.workspace.Fr_l0, obj->objfun.workspace.Fr_r0,
        obj->objfun.workspace.mpc_N, obj->objfun.workspace.params.int_method,
        obj->objfun.workspace.params.int_steps, obj->objfun.workspace.params.b,
        obj->objfun.workspace.params.p_a1, obj->objfun.workspace.params.p_a2,
        obj->objfun.workspace.params.g, obj->objfun.workspace.params.m,
        obj->objfun.workspace.params.w1, obj->objfun.workspace.params.mpc_dt,
        xk);
    evalOK = ((!muDoubleScalarIsInf(ubDiff)) && (!muDoubleScalarIsNaN(ubDiff)));
    if (evalOK) {
      xk_data[idx] = lbDiff;
    }
    obj->f_1 = ubDiff;
    obj->numEvals++;
    guard1 = false;
    if (!evalOK) {
      if (!modifiedStep) {
        deltaX = -deltaX;
        if (obj->hasLB->data[idx]) {
          ubDiff = xk_data[idx] + deltaX;
          if ((ubDiff >= lb_data[idx]) && obj->hasUB->data[idx] &&
              (ubDiff <= ub_data[idx])) {
            modifiedStep = true;
          } else {
            modifiedStep = false;
          }
        } else {
          modifiedStep = false;
        }
        if ((!obj->hasBounds) || modifiedStep) {
          lbDiff = xk_data[idx];
          xk_data[idx] += deltaX;
          ubDiff = optimize_cpp_mpc_anonFcn1(
              obj->objfun.workspace.actual_state, obj->objfun.workspace.ref_com,
              obj->objfun.workspace.Fr_l0, obj->objfun.workspace.Fr_r0,
              obj->objfun.workspace.mpc_N,
              obj->objfun.workspace.params.int_method,
              obj->objfun.workspace.params.int_steps,
              obj->objfun.workspace.params.b, obj->objfun.workspace.params.p_a1,
              obj->objfun.workspace.params.p_a2, obj->objfun.workspace.params.g,
              obj->objfun.workspace.params.m, obj->objfun.workspace.params.w1,
              obj->objfun.workspace.params.mpc_dt, xk);
          evalOK = ((!muDoubleScalarIsInf(ubDiff)) &&
                    (!muDoubleScalarIsNaN(ubDiff)));
          if (evalOK) {
            xk_data[idx] = lbDiff;
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
      gradf_data[idx] = (obj->f_1 - fCurrent) / deltaX;
      idx++;
    }
  }
  return evalOK;
}

/* End of code generation (computeFiniteDifferences.c) */
