/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
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
#include "computeForwardDifferences.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "factoryConstruct2.h"
#include "factoryConstruct3.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "xcopy.h"
#include "xnrm2.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void fmincon(const real_T fun_tunableEnvironment_f1[3], const real_T
             fun_tunableEnvironment_f2[3], const param
             *fun_tunableEnvironment_f3, const emxArray_real_T *x0, const
             emxArray_real_T *lb, const emxArray_real_T *ub, const real_T
             nonlcon_tunableEnvironment_f1[3], const real_T
             nonlcon_tunableEnvironment_f2[3], real_T
             nonlcon_tunableEnvironment_f3, real_T nonlcon_tunableEnvironment_f4,
             real_T nonlcon_tunableEnvironment_f5, const param
             *nonlcon_tunableEnvironment_f6, emxArray_real_T *x, real_T *fval,
             real_T *exitflag, real_T *output_iterations, real_T
             *output_funcCount, char_T output_algorithm[3], real_T
             *output_constrviolation, real_T *output_stepsize, real_T
             *output_lssteplength, real_T *output_firstorderopt)
{
  c_struct_T memspace;
  d_struct_T TrialState;
  e_struct_T FiniteDifferences;
  emxArray_int32_T *indexFixed;
  emxArray_int32_T *indexLB;
  emxArray_int32_T *indexUB;
  emxArray_real_T *Hessian;
  emxArray_real_T *fscales_cineq_constraint;
  emxArray_real_T *r;
  f_struct_T QRManager;
  g_struct_T FcnEvaluator;
  h_struct_T CholManager;
  i_struct_T QPObjective;
  j_struct_T WorkingSet;
  k_struct_T MeritFunction;
  real_T normResid;
  int32_T Cineq_size_idx_1;
  int32_T i;
  int32_T idxFillStart;
  int32_T mConstrMax;
  int32_T mLB;
  int32_T mNonlinIneq;
  int32_T mUB;
  int32_T maxDims;
  int32_T nVar;
  int32_T nVarMax;
  boolean_T guard1 = false;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&Hessian, 2, true);
  emxInit_real_T(&r, 2, true);
  output_algorithm[0] = 's';
  output_algorithm[1] = 'q';
  output_algorithm[2] = 'p';
  anon(nonlcon_tunableEnvironment_f1, nonlcon_tunableEnvironment_f2,
       nonlcon_tunableEnvironment_f3, nonlcon_tunableEnvironment_f5,
       nonlcon_tunableEnvironment_f6, x0, r);
  Cineq_size_idx_1 = r->size[1];
  mNonlinIneq = r->size[1] - 1;
  mLB = lb->size[1];
  mUB = ub->size[1];
  nVar = x0->size[1] - 1;
  mConstrMax = (((r->size[1] + lb->size[1]) + ub->size[1]) + r->size[1]) + 1;
  nVarMax = (x0->size[1] + r->size[1]) + 1;
  maxDims = muIntScalarMax_sint32(nVarMax, mConstrMax);
  i = Hessian->size[0] * Hessian->size[1];
  Hessian->size[0] = x0->size[1];
  Hessian->size[1] = x0->size[1];
  emxEnsureCapacity_real_T(Hessian, i);
  idxFillStart = x0->size[1] * x0->size[1];
  for (i = 0; i < idxFillStart; i++) {
    Hessian->data[i] = 0.0;
  }

  for (idxFillStart = 0; idxFillStart <= nVar; idxFillStart++) {
    Hessian->data[idxFillStart + Hessian->size[0] * idxFillStart] = 1.0;
  }

  emxInitStruct_struct_T(&TrialState, true);
  emxInitStruct_struct_T1(&FiniteDifferences, true);
  emxInitStruct_struct_T2(&QRManager, true);
  factoryConstruct(nVarMax, mConstrMax, r->size[1], x0, r->size[1], &TrialState);
  xcopy(x0->size[1], x0, TrialState.xstarsqp);
  FcnEvaluator.nVar = x0->size[1];
  FcnEvaluator.mCineq = r->size[1];
  FcnEvaluator.objfun.tunableEnvironment.f3 = *fun_tunableEnvironment_f3;
  FcnEvaluator.objfun.tunableEnvironment.f1[0] = fun_tunableEnvironment_f1[0];
  FcnEvaluator.objfun.tunableEnvironment.f2[0] = fun_tunableEnvironment_f2[0];
  FcnEvaluator.nonlcon.tunableEnvironment.f1[0] = nonlcon_tunableEnvironment_f1
    [0];
  FcnEvaluator.nonlcon.tunableEnvironment.f2[0] = nonlcon_tunableEnvironment_f2
    [0];
  FcnEvaluator.objfun.tunableEnvironment.f1[1] = fun_tunableEnvironment_f1[1];
  FcnEvaluator.objfun.tunableEnvironment.f2[1] = fun_tunableEnvironment_f2[1];
  FcnEvaluator.nonlcon.tunableEnvironment.f1[1] = nonlcon_tunableEnvironment_f1
    [1];
  FcnEvaluator.nonlcon.tunableEnvironment.f2[1] = nonlcon_tunableEnvironment_f2
    [1];
  FcnEvaluator.objfun.tunableEnvironment.f1[2] = fun_tunableEnvironment_f1[2];
  FcnEvaluator.objfun.tunableEnvironment.f2[2] = fun_tunableEnvironment_f2[2];
  FcnEvaluator.nonlcon.tunableEnvironment.f1[2] = nonlcon_tunableEnvironment_f1
    [2];
  FcnEvaluator.nonlcon.tunableEnvironment.f2[2] = nonlcon_tunableEnvironment_f2
    [2];
  FcnEvaluator.nonlcon.tunableEnvironment.f3 = nonlcon_tunableEnvironment_f3;
  FcnEvaluator.nonlcon.tunableEnvironment.f4 = nonlcon_tunableEnvironment_f4;
  FcnEvaluator.nonlcon.tunableEnvironment.f5 = nonlcon_tunableEnvironment_f5;
  FcnEvaluator.nonlcon.tunableEnvironment.f6 = *nonlcon_tunableEnvironment_f6;
  FcnEvaluator.mCeq = 0;
  FcnEvaluator.NonFiniteSupport = true;
  FcnEvaluator.SpecifyObjectiveGradient = false;
  FcnEvaluator.SpecifyConstraintGradient = false;
  FcnEvaluator.ScaleProblem = false;
  b_factoryConstruct(fun_tunableEnvironment_f1, fun_tunableEnvironment_f2,
                     fun_tunableEnvironment_f3, nonlcon_tunableEnvironment_f1,
                     nonlcon_tunableEnvironment_f2,
                     nonlcon_tunableEnvironment_f3,
                     nonlcon_tunableEnvironment_f4,
                     nonlcon_tunableEnvironment_f5,
                     nonlcon_tunableEnvironment_f6, x0->size[1], r->size[1], lb,
                     ub, &FiniteDifferences);
  QRManager.ldq = maxDims;
  i = QRManager.QR->size[0] * QRManager.QR->size[1];
  QRManager.QR->size[0] = maxDims;
  QRManager.QR->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.QR, i);
  i = QRManager.Q->size[0] * QRManager.Q->size[1];
  QRManager.Q->size[0] = maxDims;
  QRManager.Q->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.Q, i);
  idxFillStart = maxDims * maxDims;
  for (i = 0; i < idxFillStart; i++) {
    QRManager.Q->data[i] = 0.0;
  }

  i = QRManager.jpvt->size[0];
  QRManager.jpvt->size[0] = maxDims;
  emxEnsureCapacity_int32_T(QRManager.jpvt, i);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt->data[i] = 0;
  }

  emxInitStruct_struct_T3(&CholManager, true);
  emxInitStruct_struct_T4(&QPObjective, true);
  emxInitStruct_struct_T5(&memspace, true);
  emxInit_real_T(&fscales_cineq_constraint, 1, true);
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  i = QRManager.tau->size[0];
  QRManager.tau->size[0] = muIntScalarMin_sint32(maxDims, maxDims);
  emxEnsureCapacity_real_T(QRManager.tau, i);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  i = CholManager.FMat->size[0] * CholManager.FMat->size[1];
  CholManager.FMat->size[0] = maxDims;
  CholManager.FMat->size[1] = maxDims;
  emxEnsureCapacity_real_T(CholManager.FMat, i);
  CholManager.ldm = maxDims;
  CholManager.ndims = 0;
  CholManager.info = 0;
  c_factoryConstruct(nVarMax, &QPObjective);
  QPObjective.nvar = x0->size[1];
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  i = memspace.workspace_double->size[0] * memspace.workspace_double->size[1];
  memspace.workspace_double->size[0] = maxDims;
  if (2 < nVarMax) {
    memspace.workspace_double->size[1] = nVarMax;
  } else {
    memspace.workspace_double->size[1] = 2;
  }

  emxEnsureCapacity_real_T(memspace.workspace_double, i);
  i = memspace.workspace_int->size[0];
  memspace.workspace_int->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_int, i);
  i = memspace.workspace_sort->size[0];
  memspace.workspace_sort->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_sort, i);
  i = fscales_cineq_constraint->size[0];
  fscales_cineq_constraint->size[0] = r->size[1];
  emxEnsureCapacity_real_T(fscales_cineq_constraint, i);
  for (i = 0; i < Cineq_size_idx_1; i++) {
    fscales_cineq_constraint->data[i] = 1.0;
  }

  emxInit_int32_T(&indexLB, 1, true);
  emxInit_int32_T(&indexUB, 1, true);
  emxInit_int32_T(&indexFixed, 1, true);
  i = indexLB->size[0];
  indexLB->size[0] = lb->size[1];
  emxEnsureCapacity_int32_T(indexLB, i);
  i = indexUB->size[0];
  indexUB->size[0] = ub->size[1];
  emxEnsureCapacity_int32_T(indexUB, i);
  i = indexFixed->size[0];
  indexFixed->size[0] = muIntScalarMin_sint32(mLB, mUB);
  emxEnsureCapacity_int32_T(indexFixed, i);
  mLB = -1;
  mUB = -1;
  Cineq_size_idx_1 = -1;
  for (maxDims = 0; maxDims <= nVar; maxDims++) {
    normResid = lb->data[maxDims];
    guard1 = false;
    if ((!muDoubleScalarIsInf(normResid)) && (!muDoubleScalarIsNaN(normResid)))
    {
      if (muDoubleScalarAbs(normResid - ub->data[maxDims]) < 0.001) {
        Cineq_size_idx_1++;
        indexFixed->data[Cineq_size_idx_1] = maxDims + 1;
      } else {
        mLB++;
        indexLB->data[mLB] = maxDims + 1;
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      normResid = ub->data[maxDims];
      if ((!muDoubleScalarIsInf(normResid)) && (!muDoubleScalarIsNaN(normResid)))
      {
        mUB++;
        indexUB->data[mUB] = maxDims + 1;
      }
    }
  }

  emxInitStruct_struct_T6(&WorkingSet, true);
  d_factoryConstruct(r->size[1], mLB + 1, indexLB, mUB + 1, indexUB,
                     Cineq_size_idx_1 + 1, indexFixed, x0->size[1], nVarMax,
                     mConstrMax, &WorkingSet);
  emxFree_int32_T(&indexFixed);
  emxFree_int32_T(&indexUB);
  emxFree_int32_T(&indexLB);
  for (maxDims = 0; maxDims <= mLB; maxDims++) {
    TrialState.xstarsqp->data[WorkingSet.indexLB->data[maxDims] - 1] =
      muDoubleScalarMax(TrialState.xstarsqp->data[WorkingSet.indexLB->
                        data[maxDims] - 1], lb->data[WorkingSet.indexLB->
                        data[maxDims] - 1]);
  }

  for (maxDims = 0; maxDims <= mUB; maxDims++) {
    TrialState.xstarsqp->data[WorkingSet.indexUB->data[maxDims] - 1] =
      muDoubleScalarMin(TrialState.xstarsqp->data[WorkingSet.indexUB->
                        data[maxDims] - 1], ub->data[WorkingSet.indexUB->
                        data[maxDims] - 1]);
  }

  for (maxDims = 0; maxDims <= Cineq_size_idx_1; maxDims++) {
    TrialState.xstarsqp->data[WorkingSet.indexFixed->data[maxDims] - 1] =
      ub->data[WorkingSet.indexFixed->data[maxDims] - 1];
  }

  *fval = c_anon(fun_tunableEnvironment_f1, fun_tunableEnvironment_f3->m,
                 fun_tunableEnvironment_f3->num_params,
                 fun_tunableEnvironment_f3->int_method,
                 fun_tunableEnvironment_f3->N_dyn,
                 fun_tunableEnvironment_f3->int_steps,
                 fun_tunableEnvironment_f3->b, fun_tunableEnvironment_f3->p_a1,
                 fun_tunableEnvironment_f3->p_a2, fun_tunableEnvironment_f3->g,
                 fun_tunableEnvironment_f3->w4, fun_tunableEnvironment_f3->T_th,
                 TrialState.xstarsqp);
  idxFillStart = 1;
  if (muDoubleScalarIsInf(*fval) || muDoubleScalarIsNaN(*fval)) {
    if (muDoubleScalarIsNaN(*fval)) {
      idxFillStart = -6;
    } else if (*fval < 0.0) {
      idxFillStart = -4;
    } else {
      idxFillStart = -5;
    }
  }

  TrialState.sqpFval = *fval;
  if (idxFillStart == 1) {
    computeConstraints_(FcnEvaluator.nonlcon.tunableEnvironment.f1,
                        FcnEvaluator.nonlcon.tunableEnvironment.f2,
                        nonlcon_tunableEnvironment_f3,
                        nonlcon_tunableEnvironment_f5,
                        nonlcon_tunableEnvironment_f6, r->size[1],
                        TrialState.xstarsqp, TrialState.cIneq,
                        TrialState.iNonIneq0);
  }

  emxFree_real_T(&r);
  computeForwardDifferences(&FiniteDifferences, *fval, TrialState.cIneq,
    TrialState.iNonIneq0, TrialState.xstarsqp, TrialState.grad, WorkingSet.Aineq,
    TrialState.iNonIneq0, lb, ub);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  for (maxDims = 0; maxDims <= mNonlinIneq; maxDims++) {
    WorkingSet.bineq->data[maxDims] = -TrialState.cIneq->data[maxDims];
  }

  for (maxDims = 0; maxDims <= mLB; maxDims++) {
    WorkingSet.lb->data[WorkingSet.indexLB->data[maxDims] - 1] = -lb->
      data[WorkingSet.indexLB->data[maxDims] - 1] + x0->data
      [WorkingSet.indexLB->data[maxDims] - 1];
  }

  for (maxDims = 0; maxDims <= mUB; maxDims++) {
    WorkingSet.ub->data[WorkingSet.indexUB->data[maxDims] - 1] = ub->
      data[WorkingSet.indexUB->data[maxDims] - 1] - x0->data
      [WorkingSet.indexUB->data[maxDims] - 1];
  }

  for (maxDims = 0; maxDims <= Cineq_size_idx_1; maxDims++) {
    normResid = ub->data[WorkingSet.indexFixed->data[maxDims] - 1] - x0->
      data[WorkingSet.indexFixed->data[maxDims] - 1];
    WorkingSet.ub->data[WorkingSet.indexFixed->data[maxDims] - 1] = normResid;
    WorkingSet.bwset->data[maxDims] = normResid;
  }

  setProblemType(&WorkingSet, 3);
  idxFillStart = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  for (maxDims = idxFillStart; maxDims <= i; maxDims++) {
    WorkingSet.isActiveConstr->data[maxDims - 1] = false;
  }

  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  idxFillStart = WorkingSet.sizes[0];
  for (maxDims = 0; maxDims < idxFillStart; maxDims++) {
    WorkingSet.Wid->data[maxDims] = 1;
    WorkingSet.Wlocalidx->data[maxDims] = maxDims + 1;
    WorkingSet.isActiveConstr->data[maxDims] = true;
    i = WorkingSet.indexFixed->data[maxDims];
    for (nVar = 0; nVar <= i - 2; nVar++) {
      WorkingSet.ATwset->data[nVar + WorkingSet.ATwset->size[0] * maxDims] = 0.0;
    }

    WorkingSet.ATwset->data[(WorkingSet.indexFixed->data[maxDims] +
      WorkingSet.ATwset->size[0] * maxDims) - 1] = 1.0;
    i = WorkingSet.indexFixed->data[maxDims] + 1;
    Cineq_size_idx_1 = WorkingSet.nVar;
    for (nVar = i; nVar <= Cineq_size_idx_1; nVar++) {
      WorkingSet.ATwset->data[(nVar + WorkingSet.ATwset->size[0] * maxDims) - 1]
        = 0.0;
    }

    WorkingSet.bwset->data[maxDims] = WorkingSet.ub->data
      [WorkingSet.indexFixed->data[maxDims] - 1];
  }

  MeritFunction.initFval = *fval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initConstrViolationEq = 0.0;
  normResid = 0.0;
  for (maxDims = 0; maxDims <= mNonlinIneq; maxDims++) {
    if (TrialState.cIneq->data[maxDims] > 0.0) {
      normResid += TrialState.cIneq->data[maxDims];
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
  driver(Hessian, lb, ub, &TrialState, &MeritFunction, &FcnEvaluator,
         &FiniteDifferences, &memspace, &WorkingSet, &QRManager, &CholManager,
         &QPObjective, fscales_cineq_constraint);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = TrialState.xstarsqp->size[1];
  emxEnsureCapacity_real_T(x, i);
  idxFillStart = TrialState.xstarsqp->size[0] * TrialState.xstarsqp->size[1];
  emxFree_real_T(&Hessian);
  emxFreeStruct_struct_T6(&WorkingSet);
  emxFree_real_T(&fscales_cineq_constraint);
  emxFreeStruct_struct_T5(&memspace);
  emxFreeStruct_struct_T4(&QPObjective);
  emxFreeStruct_struct_T3(&CholManager);
  emxFreeStruct_struct_T2(&QRManager);
  emxFreeStruct_struct_T1(&FiniteDifferences);
  for (i = 0; i < idxFillStart; i++) {
    x->data[i] = TrialState.xstarsqp->data[i];
  }

  *output_stepsize = xnrm2(x0->size[1], TrialState.delta_x);
  *fval = TrialState.sqpFval;
  *exitflag = TrialState.sqpExitFlag;
  *output_iterations = TrialState.sqpIterations;
  *output_funcCount = TrialState.FunctionEvaluations;
  *output_constrviolation = MeritFunction.nlpPrimalFeasError;
  *output_lssteplength = TrialState.steplength;
  *output_firstorderopt = MeritFunction.firstOrderOpt;
  emxFreeStruct_struct_T(&TrialState);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (fmincon.c) */
