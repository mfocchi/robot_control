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
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
boolean_T computeFiniteDifferences(
    k_struct_T *obj, real_T fCurrent, const emxArray_real_T *cIneqCurrent,
    int32_T ineq0, emxArray_real_T *xk, emxArray_real_T *gradf,
    emxArray_real_T *JacCineqTrans, int32_T CineqColStart, int32_T ldJI,
    const emxArray_real_T *lb, const emxArray_real_T *ub)
{
  emxArray_real_T *r;
  emxArray_real_T *varargout_1;
  const real_T *cIneqCurrent_data;
  const real_T *lb_data;
  const real_T *ub_data;
  real_T *JacCineqTrans_data;
  real_T *gradf_data;
  real_T *r1;
  real_T *varargout_1_data;
  real_T *xk_data;
  int32_T b_idx;
  int32_T i;
  int32_T idx;
  boolean_T evalOK;
  boolean_T exitg1;
  ub_data = ub->data;
  lb_data = lb->data;
  JacCineqTrans_data = JacCineqTrans->data;
  gradf_data = gradf->data;
  xk_data = xk->data;
  cIneqCurrent_data = cIneqCurrent->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  evalOK = true;
  obj->numEvals = 0;
  idx = 0;
  emxInit_real_T(&r, 1);
  emxInit_real_T(&varargout_1, 2);
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
    i = r->size[0];
    r->size[0] = obj->cIneq_1->size[0];
    emxEnsureCapacity_real_T(r, i);
    r1 = r->data;
    b_idx = obj->cIneq_1->size[0];
    for (i = 0; i < b_idx; i++) {
      r1[i] = obj->cIneq_1->data[i];
    }
    lbDiff = xk_data[idx];
    xk_data[idx] += deltaX;
    ubDiff = optimize_cpp_anonFcn1(
        obj->objfun.workspace.p0, obj->objfun.workspace.params.m,
        obj->objfun.workspace.params.num_params,
        obj->objfun.workspace.params.int_method,
        obj->objfun.workspace.params.N_dyn,
        obj->objfun.workspace.params.int_steps, obj->objfun.workspace.params.b,
        obj->objfun.workspace.params.p_a1, obj->objfun.workspace.params.p_a2,
        obj->objfun.workspace.params.g, obj->objfun.workspace.params.w1,
        obj->objfun.workspace.params.w2, obj->objfun.workspace.params.T_th, xk);
    evalOK = ((!muDoubleScalarIsInf(ubDiff)) && (!muDoubleScalarIsNaN(ubDiff)));
    if (evalOK) {
      optimize_cpp_anonFcn2(obj->nonlin.workspace.p0, obj->nonlin.workspace.pf,
                            obj->nonlin.workspace.Fleg_max,
                            obj->nonlin.workspace.mu,
                            &obj->nonlin.workspace.params, xk, varargout_1);
      varargout_1_data = varargout_1->data;
      i = r->size[0];
      r->size[0] = obj->cIneq_1->size[0];
      emxEnsureCapacity_real_T(r, i);
      r1 = r->data;
      b_idx = obj->cIneq_1->size[0];
      for (i = 0; i < b_idx; i++) {
        r1[i] = varargout_1_data[i];
      }
      b_idx = 0;
      while (evalOK && (b_idx + 1 <= obj->mIneq)) {
        evalOK = ((!muDoubleScalarIsInf(varargout_1_data[b_idx])) &&
                  (!muDoubleScalarIsNaN(varargout_1_data[b_idx])));
        b_idx++;
      }
      if (evalOK) {
        xk_data[idx] = lbDiff;
      }
    }
    obj->f_1 = ubDiff;
    i = obj->cIneq_1->size[0];
    obj->cIneq_1->size[0] = r->size[0];
    emxEnsureCapacity_real_T(obj->cIneq_1, i);
    b_idx = r->size[0];
    for (i = 0; i < b_idx; i++) {
      obj->cIneq_1->data[i] = r1[i];
    }
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
          i = obj->cIneq_1->size[0];
          obj->cIneq_1->size[0] = r->size[0];
          emxEnsureCapacity_real_T(obj->cIneq_1, i);
          b_idx = r->size[0];
          for (i = 0; i < b_idx; i++) {
            obj->cIneq_1->data[i] = r1[i];
          }
          lbDiff = xk_data[idx];
          xk_data[idx] += deltaX;
          ubDiff = optimize_cpp_anonFcn1(
              obj->objfun.workspace.p0, obj->objfun.workspace.params.m,
              obj->objfun.workspace.params.num_params,
              obj->objfun.workspace.params.int_method,
              obj->objfun.workspace.params.N_dyn,
              obj->objfun.workspace.params.int_steps,
              obj->objfun.workspace.params.b, obj->objfun.workspace.params.p_a1,
              obj->objfun.workspace.params.p_a2, obj->objfun.workspace.params.g,
              obj->objfun.workspace.params.w1, obj->objfun.workspace.params.w2,
              obj->objfun.workspace.params.T_th, xk);
          evalOK = ((!muDoubleScalarIsInf(ubDiff)) &&
                    (!muDoubleScalarIsNaN(ubDiff)));
          if (evalOK) {
            optimize_cpp_anonFcn2(
                obj->nonlin.workspace.p0, obj->nonlin.workspace.pf,
                obj->nonlin.workspace.Fleg_max, obj->nonlin.workspace.mu,
                &obj->nonlin.workspace.params, xk, varargout_1);
            varargout_1_data = varargout_1->data;
            i = obj->cIneq_1->size[0];
            obj->cIneq_1->size[0] = r->size[0];
            emxEnsureCapacity_real_T(obj->cIneq_1, i);
            b_idx = r->size[0];
            for (i = 0; i < b_idx; i++) {
              obj->cIneq_1->data[i] = varargout_1_data[i];
            }
            b_idx = 0;
            while (evalOK && (b_idx + 1 <= obj->mIneq)) {
              evalOK = ((!muDoubleScalarIsInf(varargout_1_data[b_idx])) &&
                        (!muDoubleScalarIsNaN(varargout_1_data[b_idx])));
              b_idx++;
            }
            if (evalOK) {
              xk_data[idx] = lbDiff;
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
      gradf_data[idx] = (obj->f_1 - fCurrent) / deltaX;
      i = obj->mIneq;
      for (b_idx = 0; b_idx < i; b_idx++) {
        JacCineqTrans_data[idx + ldJI * ((CineqColStart + b_idx) - 1)] =
            (obj->cIneq_1->data[b_idx] -
             cIneqCurrent_data[(ineq0 + b_idx) - 1]) /
            deltaX;
      }
      idx++;
    }
  }
  emxFree_real_T(&varargout_1);
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return evalOK;
}

/* End of code generation (computeFiniteDifferences.c) */
