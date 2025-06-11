/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fmincon.c
 *
 * Code generation for function 'fmincon'
 *
 */

/* Include files */
#include "fmincon.h"
#include "computeConstraints_.h"
#include "computeFiniteDifferences.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "factoryConstruct2.h"
#include "fillOutputStruct.h"
#include "loadProblem.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "xcopy.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
real_T fmincon(optimize_cppStackData *SD, const real_T fun_workspace_p0[3],
               const param *fun_workspace_params, const emxArray_real_T *x0,
               const emxArray_real_T *lb, const emxArray_real_T *ub,
               const real_T nonlcon_workspace_p0[3],
               const real_T nonlcon_workspace_pf[3],
               real_T nonlcon_workspace_Fleg_max, real_T nonlcon_workspace_mu,
               const param *nonlcon_workspace_params, emxArray_real_T *x,
               real_T *exitflag, real_T *output_iterations,
               real_T *output_funcCount, char_T output_algorithm[3],
               real_T *output_constrviolation, real_T *output_stepsize,
               real_T *output_lssteplength, real_T *output_firstorderopt)
{
  c_struct_T QPObjective;
  d_struct_T memspace;
  e_struct_T obj;
  emxArray_real_T *Hessian;
  emxArray_real_T *fscales_cineq_constraint;
  emxArray_real_T *r;
  f_struct_T b_obj;
  g_struct_T TrialState;
  h_struct_T WorkingSet;
  struct_T MeritFunction;
  const real_T *lb_data;
  const real_T *ub_data;
  const real_T *x0_data;
  real_T fval;
  real_T normResid;
  real_T *Hessian_data;
  int32_T i;
  int32_T idxFillStart;
  int32_T loop_ub;
  int32_T mConstrMax;
  int32_T mFixed;
  int32_T mLB;
  int32_T mNonlinIneq;
  int32_T mUB;
  int32_T maxDims;
  int32_T nVar;
  int32_T nVarMax;
  ub_data = ub->data;
  lb_data = lb->data;
  x0_data = x0->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  nVar = x0->size[1] - 1;
  emxInit_real_T(&r, 2);
  optimize_cpp_anonFcn2(nonlcon_workspace_p0, nonlcon_workspace_pf,
                        nonlcon_workspace_Fleg_max, nonlcon_workspace_mu,
                        nonlcon_workspace_params, x0, r);
  idxFillStart = r->size[1];
  mNonlinIneq = r->size[1] - 1;
  mConstrMax = (((r->size[1] + lb->size[1]) + ub->size[1]) + r->size[1]) + 1;
  nVarMax = (x0->size[1] + r->size[1]) + 1;
  maxDims = muIntScalarMax_sint32(nVarMax, mConstrMax);
  emxInit_real_T(&Hessian, 2);
  i = Hessian->size[0] * Hessian->size[1];
  Hessian->size[0] = x0->size[1];
  Hessian->size[1] = x0->size[1];
  emxEnsureCapacity_real_T(Hessian, i);
  Hessian_data = Hessian->data;
  loop_ub = x0->size[1] * x0->size[1];
  for (i = 0; i < loop_ub; i++) {
    Hessian_data[i] = 0.0;
  }
  for (loop_ub = 0; loop_ub <= nVar; loop_ub++) {
    Hessian_data[loop_ub + Hessian->size[0] * loop_ub] = 1.0;
  }
  emxInitStruct_struct_T(&TrialState);
  factoryConstruct(nVarMax, mConstrMax, r->size[1], x0, r->size[1],
                   &TrialState);
  xcopy(x0->size[1], x0, TrialState.xstarsqp);
  SD->f0.FcnEvaluator.next.next.next.next.next.value = r->size[1];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .params = *fun_workspace_params;
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .p0[0] = fun_workspace_p0[0];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace.p0[0] =
      nonlcon_workspace_p0[0];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pf[0] =
      nonlcon_workspace_pf[0];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .p0[1] = fun_workspace_p0[1];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace.p0[1] =
      nonlcon_workspace_p0[1];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pf[1] =
      nonlcon_workspace_pf[1];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .p0[2] = fun_workspace_p0[2];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace.p0[2] =
      nonlcon_workspace_p0[2];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pf[2] =
      nonlcon_workspace_pf[2];
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace
      .Fleg_max = nonlcon_workspace_Fleg_max;
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace.mu =
      nonlcon_workspace_mu;
  SD->f0.FcnEvaluator.next.next.next.next.next.next.next.value.workspace
      .params = *nonlcon_workspace_params;
  emxInitStruct_struct_T1(&SD->f0.FiniteDifferences);
  b_factoryConstruct(fun_workspace_p0, fun_workspace_params,
                     nonlcon_workspace_p0, nonlcon_workspace_pf,
                     nonlcon_workspace_Fleg_max, nonlcon_workspace_mu,
                     nonlcon_workspace_params, x0->size[1], r->size[1], lb, ub,
                     &SD->f0.FiniteDifferences);
  emxInitStruct_struct_T2(&QPObjective);
  i = QPObjective.grad->size[0];
  QPObjective.grad->size[0] = nVarMax;
  emxEnsureCapacity_real_T(QPObjective.grad, i);
  i = QPObjective.Hx->size[0];
  QPObjective.Hx->size[0] = nVarMax - 1;
  emxEnsureCapacity_real_T(QPObjective.Hx, i);
  QPObjective.maxVar = nVarMax;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.nvar = x0->size[1];
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  emxInitStruct_struct_T3(&memspace);
  i = memspace.workspace_double->size[0] * memspace.workspace_double->size[1];
  memspace.workspace_double->size[0] = maxDims;
  memspace.workspace_double->size[1] = muIntScalarMax_sint32(nVarMax, 2);
  emxEnsureCapacity_real_T(memspace.workspace_double, i);
  i = memspace.workspace_int->size[0];
  memspace.workspace_int->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_int, i);
  i = memspace.workspace_sort->size[0];
  memspace.workspace_sort->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_sort, i);
  emxInit_real_T(&fscales_cineq_constraint, 1);
  i = fscales_cineq_constraint->size[0];
  fscales_cineq_constraint->size[0] = r->size[1];
  emxEnsureCapacity_real_T(fscales_cineq_constraint, i);
  Hessian_data = fscales_cineq_constraint->data;
  for (i = 0; i < idxFillStart; i++) {
    Hessian_data[i] = 1.0;
  }
  emxInitStruct_struct_T4(&WorkingSet);
  c_factoryConstruct(r->size[1], x0->size[1], nVarMax, mConstrMax, &WorkingSet);
  mLB = -1;
  mUB = -1;
  mFixed = -1;
  for (loop_ub = 0; loop_ub <= nVar; loop_ub++) {
    boolean_T guard1;
    normResid = lb_data[loop_ub];
    guard1 = false;
    if ((!muDoubleScalarIsInf(normResid)) &&
        (!muDoubleScalarIsNaN(normResid))) {
      if (muDoubleScalarAbs(normResid - ub_data[loop_ub]) < 0.001) {
        mFixed++;
        WorkingSet.indexFixed->data[mFixed] = loop_ub + 1;
      } else {
        mLB++;
        WorkingSet.indexLB->data[mLB] = loop_ub + 1;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      normResid = ub_data[loop_ub];
      if ((!muDoubleScalarIsInf(normResid)) &&
          (!muDoubleScalarIsNaN(normResid))) {
        mUB++;
        WorkingSet.indexUB->data[mUB] = loop_ub + 1;
      }
    }
  }
  loadProblem(&WorkingSet, r->size[1], mLB + 1, mUB + 1, mFixed + 1,
              mConstrMax);
  for (loop_ub = 0; loop_ub <= mLB; loop_ub++) {
    TrialState.xstarsqp->data[WorkingSet.indexLB->data[loop_ub] - 1] =
        muDoubleScalarMax(
            TrialState.xstarsqp->data[WorkingSet.indexLB->data[loop_ub] - 1],
            lb_data[WorkingSet.indexLB->data[loop_ub] - 1]);
  }
  for (loop_ub = 0; loop_ub <= mUB; loop_ub++) {
    TrialState.xstarsqp->data[WorkingSet.indexUB->data[loop_ub] - 1] =
        muDoubleScalarMin(
            TrialState.xstarsqp->data[WorkingSet.indexUB->data[loop_ub] - 1],
            ub_data[WorkingSet.indexUB->data[loop_ub] - 1]);
  }
  for (loop_ub = 0; loop_ub <= mFixed; loop_ub++) {
    TrialState.xstarsqp->data[WorkingSet.indexFixed->data[loop_ub] - 1] =
        ub_data[WorkingSet.indexFixed->data[loop_ub] - 1];
  }
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = TrialState.xstarsqp->size[1];
  emxEnsureCapacity_real_T(x, i);
  Hessian_data = x->data;
  loop_ub = TrialState.xstarsqp->size[1];
  for (i = 0; i < loop_ub; i++) {
    Hessian_data[i] = TrialState.xstarsqp->data[i];
  }
  fval = optimize_cpp_anonFcn1(
      fun_workspace_p0, fun_workspace_params->m,
      fun_workspace_params->num_params, fun_workspace_params->int_method,
      fun_workspace_params->N_dyn, fun_workspace_params->int_steps,
      fun_workspace_params->b, fun_workspace_params->p_a1,
      fun_workspace_params->p_a2, fun_workspace_params->g,
      fun_workspace_params->w1, fun_workspace_params->w2,
      fun_workspace_params->T_th, x);
  if ((!muDoubleScalarIsInf(fval)) && (!muDoubleScalarIsNaN(fval))) {
    i = TrialState.xstarsqp->size[0] * TrialState.xstarsqp->size[1];
    TrialState.xstarsqp->size[0] = 1;
    TrialState.xstarsqp->size[1] = x->size[1];
    emxEnsureCapacity_real_T(TrialState.xstarsqp, i);
    loop_ub = x->size[1];
    for (i = 0; i < loop_ub; i++) {
      TrialState.xstarsqp->data[i] = Hessian_data[i];
    }
    computeConstraints_(r->size[1],
                        SD->f0.FcnEvaluator.next.next.next.next.next.next.next
                            .value.workspace.p0,
                        SD->f0.FcnEvaluator.next.next.next.next.next.next.next
                            .value.workspace.pf,
                        nonlcon_workspace_Fleg_max, nonlcon_workspace_mu,
                        nonlcon_workspace_params, TrialState.xstarsqp,
                        TrialState.cIneq, TrialState.iNonIneq0);
  }
  TrialState.sqpFval = fval;
  computeFiniteDifferences(&SD->f0.FiniteDifferences, fval, TrialState.cIneq,
                           TrialState.iNonIneq0, TrialState.xstarsqp,
                           TrialState.grad, WorkingSet.Aineq,
                           TrialState.iNonIneq0, WorkingSet.ldA, lb, ub);
  TrialState.FunctionEvaluations = SD->f0.FiniteDifferences.numEvals + 1;
  idxFillStart = (r->size[1] / 2) << 1;
  emxFree_real_T(&r);
  nVarMax = idxFillStart - 2;
  for (loop_ub = 0; loop_ub <= nVarMax; loop_ub += 2) {
    __m128d r1;
    r1 = _mm_loadu_pd(&TrialState.cIneq->data[loop_ub]);
    _mm_storeu_pd(&WorkingSet.bineq->data[loop_ub],
                  _mm_mul_pd(r1, _mm_set1_pd(-1.0)));
  }
  for (loop_ub = idxFillStart; loop_ub <= mNonlinIneq; loop_ub++) {
    WorkingSet.bineq->data[loop_ub] = -TrialState.cIneq->data[loop_ub];
  }
  for (loop_ub = 0; loop_ub <= mLB; loop_ub++) {
    WorkingSet.lb->data[WorkingSet.indexLB->data[loop_ub] - 1] =
        -lb_data[WorkingSet.indexLB->data[loop_ub] - 1] +
        x0_data[WorkingSet.indexLB->data[loop_ub] - 1];
  }
  for (loop_ub = 0; loop_ub <= mUB; loop_ub++) {
    WorkingSet.ub->data[WorkingSet.indexUB->data[loop_ub] - 1] =
        ub_data[WorkingSet.indexUB->data[loop_ub] - 1] -
        x0_data[WorkingSet.indexUB->data[loop_ub] - 1];
  }
  for (loop_ub = 0; loop_ub <= mFixed; loop_ub++) {
    normResid = ub_data[WorkingSet.indexFixed->data[loop_ub] - 1] -
                x0_data[WorkingSet.indexFixed->data[loop_ub] - 1];
    WorkingSet.ub->data[WorkingSet.indexFixed->data[loop_ub] - 1] = normResid;
    WorkingSet.bwset->data[loop_ub] = normResid;
  }
  setProblemType(&WorkingSet, 3);
  idxFillStart = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  for (loop_ub = idxFillStart; loop_ub <= i; loop_ub++) {
    WorkingSet.isActiveConstr->data[loop_ub - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  idxFillStart = WorkingSet.sizes[0];
  for (mLB = 0; mLB < idxFillStart; mLB++) {
    WorkingSet.Wid->data[mLB] = 1;
    WorkingSet.Wlocalidx->data[mLB] = mLB + 1;
    WorkingSet.isActiveConstr->data[mLB] = true;
    nVarMax = WorkingSet.ldA * mLB;
    i = WorkingSet.indexFixed->data[mLB];
    for (mUB = 0; mUB <= i - 2; mUB++) {
      WorkingSet.ATwset->data[mUB + nVarMax] = 0.0;
    }
    WorkingSet.ATwset->data[(WorkingSet.indexFixed->data[mLB] + nVarMax) - 1] =
        1.0;
    i = WorkingSet.indexFixed->data[mLB] + 1;
    loop_ub = WorkingSet.nVar;
    for (mUB = i; mUB <= loop_ub; mUB++) {
      WorkingSet.ATwset->data[(mUB + nVarMax) - 1] = 0.0;
    }
    WorkingSet.bwset->data[mLB] =
        WorkingSet.ub->data[WorkingSet.indexFixed->data[mLB] - 1];
  }
  MeritFunction.initFval = fval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initConstrViolationEq = 0.0;
  normResid = 0.0;
  for (loop_ub = 0; loop_ub <= mNonlinIneq; loop_ub++) {
    if (TrialState.cIneq->data[loop_ub] > 0.0) {
      normResid += TrialState.cIneq->data[loop_ub];
    }
  }
  MeritFunction.initConstrViolationIneq = normResid;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  emxInitStruct_struct_T5(&obj);
  obj.ldq = maxDims;
  i = obj.QR->size[0] * obj.QR->size[1];
  obj.QR->size[0] = maxDims;
  obj.QR->size[1] = maxDims;
  emxEnsureCapacity_real_T(obj.QR, i);
  i = obj.Q->size[0] * obj.Q->size[1];
  obj.Q->size[0] = maxDims;
  obj.Q->size[1] = maxDims;
  emxEnsureCapacity_real_T(obj.Q, i);
  loop_ub = maxDims * maxDims;
  for (i = 0; i < loop_ub; i++) {
    obj.Q->data[i] = 0.0;
  }
  i = obj.jpvt->size[0];
  obj.jpvt->size[0] = maxDims;
  emxEnsureCapacity_int32_T(obj.jpvt, i);
  for (i = 0; i < maxDims; i++) {
    obj.jpvt->data[i] = 0;
  }
  obj.mrows = 0;
  obj.ncols = 0;
  i = obj.tau->size[0];
  obj.tau->size[0] = muIntScalarMin_sint32(maxDims, maxDims);
  emxEnsureCapacity_real_T(obj.tau, i);
  obj.minRowCol = 0;
  obj.usedPivoting = false;
  emxInitStruct_struct_T6(&b_obj);
  i = b_obj.FMat->size[0] * b_obj.FMat->size[1];
  b_obj.FMat->size[0] = maxDims;
  b_obj.FMat->size[1] = maxDims;
  emxEnsureCapacity_real_T(b_obj.FMat, i);
  b_obj.ldm = maxDims;
  b_obj.ndims = 0;
  b_obj.info = 0;
  b_obj.scaleFactor = 0.0;
  b_obj.ConvexCheck = true;
  b_obj.regTol_ = rtInf;
  b_obj.workspace_ = rtInf;
  b_obj.workspace2_ = rtInf;
  driver(Hessian, lb, ub, &TrialState, &MeritFunction, &SD->f0.FcnEvaluator,
         &SD->f0.FiniteDifferences, &memspace, &WorkingSet, &obj, &b_obj,
         &QPObjective, fscales_cineq_constraint);
  emxFreeStruct_struct_T6(&b_obj);
  emxFreeStruct_struct_T5(&obj);
  emxFree_real_T(&Hessian);
  emxFreeStruct_struct_T4(&WorkingSet);
  emxFree_real_T(&fscales_cineq_constraint);
  emxFreeStruct_struct_T3(&memspace);
  emxFreeStruct_struct_T2(&QPObjective);
  emxFreeStruct_struct_T1(&SD->f0.FiniteDifferences);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = TrialState.xstarsqp->size[1];
  emxEnsureCapacity_real_T(x, i);
  Hessian_data = x->data;
  loop_ub = TrialState.xstarsqp->size[1];
  for (i = 0; i < loop_ub; i++) {
    Hessian_data[i] = TrialState.xstarsqp->data[i];
  }
  *output_iterations = fillOutputStruct(
      x0->size[1], TrialState.FunctionEvaluations, TrialState.sqpIterations,
      TrialState.steplength, TrialState.delta_x,
      MeritFunction.nlpPrimalFeasError, MeritFunction.firstOrderOpt,
      output_funcCount, output_algorithm, output_constrviolation,
      output_stepsize, output_lssteplength, output_firstorderopt);
  emxFreeStruct_struct_T(&TrialState);
  fval = TrialState.sqpFval;
  *exitflag = TrialState.sqpExitFlag;
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return fval;
}

/* End of code generation (fmincon.c) */
