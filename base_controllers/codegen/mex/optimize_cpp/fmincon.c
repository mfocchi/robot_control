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
#include "initActiveSet.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T fmincon(const real_T fun_workspace_p0[3],
               const param *fun_workspace_params, const emxArray_real_T *x0,
               const emxArray_real_T *lb, const emxArray_real_T *ub,
               const real_T nonlcon_workspace_p0[3],
               const real_T nonlcon_workspace_pf[3],
               real_T nonlcon_workspace_Fleg_max, real_T nonlcon_workspace_mu,
               const param *nonlcon_workspace_params, emxArray_real_T *x,
               char_T output_algorithm[3], real_T *exitflag,
               real_T *output_iterations, real_T *output_funcCount,
               real_T *output_constrviolation, real_T *output_stepsize,
               real_T *output_lssteplength, real_T *output_firstorderopt)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  c_struct_T QRManager;
  d_struct_T CholManager;
  e_struct_T QPObjective;
  emxArray_real_T *Hessian;
  emxArray_real_T *fscales_cineq_constraint;
  emxArray_real_T *r;
  f_struct_T memspace;
  g_struct_T TrialState;
  h_struct_T WorkingSet;
  i_coder_internal_stickyStruct FcnEvaluator;
  k_struct_T FiniteDifferences;
  struct_T MeritFunction;
  const real_T *lb_data;
  const real_T *ub_data;
  const real_T *x0_data;
  real_T fval;
  real_T normResid;
  real_T *Hessian_data;
  int32_T Cineq_size_idx_1;
  int32_T i;
  int32_T loop_ub;
  int32_T mConstrMax;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  int32_T maxDims;
  int32_T nVar;
  int32_T nVarMax;
  boolean_T b;
  ub_data = ub->data;
  lb_data = lb->data;
  x0_data = x0->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  output_algorithm[0] = 's';
  output_algorithm[1] = 'q';
  output_algorithm[2] = 'p';
  nVar = x0->size[1] - 1;
  emxInit_real_T(&r, 2);
  optimize_cpp_anonFcn2(nonlcon_workspace_p0, nonlcon_workspace_pf,
                        nonlcon_workspace_Fleg_max, nonlcon_workspace_mu,
                        nonlcon_workspace_params, x0, r);
  Cineq_size_idx_1 = r->size[1];
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
  n_t = (ptrdiff_t)x0->size[1];
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dcopy(&n_t, (real_T *)&x0_data[0], &incx_t, &TrialState.xstarsqp->data[0],
        &incy_t);
  FcnEvaluator.next.next.next.next.next.value = r->size[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.params =
      *fun_workspace_params;
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.p0[0] =
      fun_workspace_p0[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.p0[0] =
      nonlcon_workspace_p0[0];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pf[0] =
      nonlcon_workspace_pf[0];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.p0[1] =
      fun_workspace_p0[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.p0[1] =
      nonlcon_workspace_p0[1];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pf[1] =
      nonlcon_workspace_pf[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.p0[2] =
      fun_workspace_p0[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.p0[2] =
      nonlcon_workspace_p0[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pf[2] =
      nonlcon_workspace_pf[2];
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.Fleg_max =
      nonlcon_workspace_Fleg_max;
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.mu =
      nonlcon_workspace_mu;
  FcnEvaluator.next.next.next.next.next.next.next.value.workspace.params =
      *nonlcon_workspace_params;
  emxInitStruct_struct_T1(&FiniteDifferences);
  b_factoryConstruct(fun_workspace_p0, fun_workspace_params,
                     nonlcon_workspace_p0, nonlcon_workspace_pf,
                     nonlcon_workspace_Fleg_max, nonlcon_workspace_mu,
                     nonlcon_workspace_params, x0->size[1], r->size[1], lb, ub,
                     &FiniteDifferences);
  emxInitStruct_struct_T2(&QRManager);
  QRManager.ldq = maxDims;
  i = QRManager.QR->size[0] * QRManager.QR->size[1];
  QRManager.QR->size[0] = maxDims;
  QRManager.QR->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.QR, i);
  i = QRManager.Q->size[0] * QRManager.Q->size[1];
  QRManager.Q->size[0] = maxDims;
  QRManager.Q->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.Q, i);
  loop_ub = maxDims * maxDims;
  for (i = 0; i < loop_ub; i++) {
    QRManager.Q->data[i] = 0.0;
  }
  i = QRManager.jpvt->size[0];
  QRManager.jpvt->size[0] = maxDims;
  emxEnsureCapacity_int32_T(QRManager.jpvt, i);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt->data[i] = 0;
  }
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  i = QRManager.tau->size[0];
  QRManager.tau->size[0] = muIntScalarMin_sint32(maxDims, maxDims);
  emxEnsureCapacity_real_T(QRManager.tau, i);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  emxInitStruct_struct_T3(&CholManager);
  i = CholManager.FMat->size[0] * CholManager.FMat->size[1];
  CholManager.FMat->size[0] = maxDims;
  CholManager.FMat->size[1] = maxDims;
  emxEnsureCapacity_real_T(CholManager.FMat, i);
  CholManager.ldm = maxDims;
  CholManager.ndims = 0;
  CholManager.info = 0;
  CholManager.scaleFactor = 0.0;
  CholManager.ConvexCheck = true;
  CholManager.regTol_ = rtInf;
  CholManager.workspace_ = rtInf;
  CholManager.workspace2_ = rtInf;
  emxInitStruct_struct_T4(&QPObjective);
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
  emxInitStruct_struct_T5(&memspace);
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
  for (i = 0; i < Cineq_size_idx_1; i++) {
    Hessian_data[i] = 1.0;
  }
  emxInitStruct_struct_T6(&WorkingSet);
  c_factoryConstruct(r->size[1], x0->size[1], nVarMax, mConstrMax, &WorkingSet);
  nVar = x0->size[1];
  mLB = 0;
  mUB = 0;
  mFixed = 0;
  for (loop_ub = 0; loop_ub < nVar; loop_ub++) {
    boolean_T guard1;
    normResid = lb_data[loop_ub];
    guard1 = false;
    if ((!muDoubleScalarIsInf(normResid)) &&
        (!muDoubleScalarIsNaN(normResid))) {
      if (muDoubleScalarAbs(normResid - ub_data[loop_ub]) < 0.001) {
        mFixed++;
        WorkingSet.indexFixed->data[mFixed - 1] = loop_ub + 1;
      } else {
        mLB++;
        WorkingSet.indexLB->data[mLB - 1] = loop_ub + 1;
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
        WorkingSet.indexUB->data[mUB - 1] = loop_ub + 1;
      }
    }
  }
  WorkingSet.mConstrMax = mConstrMax;
  maxDims = r->size[1] + mLB;
  nVarMax = (maxDims + mUB) + mFixed;
  WorkingSet.mConstr = nVarMax;
  WorkingSet.mConstrOrig = nVarMax;
  WorkingSet.sizes[0] = mFixed;
  WorkingSet.sizes[1] = 0;
  WorkingSet.sizes[2] = r->size[1];
  WorkingSet.sizes[3] = mLB;
  WorkingSet.sizes[4] = mUB;
  WorkingSet.sizesPhaseOne[0] = mFixed;
  WorkingSet.sizesPhaseOne[1] = 0;
  WorkingSet.sizesPhaseOne[2] = r->size[1];
  WorkingSet.sizesPhaseOne[3] = mLB + 1;
  WorkingSet.sizesPhaseOne[4] = mUB;
  WorkingSet.sizesRegularized[0] = mFixed;
  WorkingSet.sizesRegularized[1] = 0;
  WorkingSet.sizesRegularized[2] = r->size[1];
  WorkingSet.sizesRegularized[3] = maxDims;
  WorkingSet.sizesRegularized[4] = mUB;
  WorkingSet.sizesRegPhaseOne[0] = mFixed;
  WorkingSet.sizesRegPhaseOne[1] = 0;
  WorkingSet.sizesRegPhaseOne[2] = r->size[1];
  WorkingSet.sizesRegPhaseOne[3] = maxDims + 1;
  WorkingSet.sizesRegPhaseOne[4] = mUB;
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[3] = r->size[1];
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.sizesNormal[loop_ub] = WorkingSet.sizes[loop_ub];
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdx[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
    WorkingSet.isActiveIdxNormal[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[3] = r->size[1];
  WorkingSet.isActiveIdxRegPhaseOne[4] = mLB + 1;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxPhaseOne[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[3] = r->size[1];
  WorkingSet.isActiveIdxRegPhaseOne[4] = maxDims;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxRegularized[i] = WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[3] = r->size[1];
  WorkingSet.isActiveIdxRegPhaseOne[4] = maxDims + 1;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (loop_ub = 0; loop_ub < 5; loop_ub++) {
    WorkingSet.isActiveIdxRegPhaseOne[loop_ub + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[loop_ub];
  }
  for (loop_ub = 0; loop_ub < mLB; loop_ub++) {
    TrialState.xstarsqp->data[WorkingSet.indexLB->data[loop_ub] - 1] =
        muDoubleScalarMax(
            TrialState.xstarsqp->data[WorkingSet.indexLB->data[loop_ub] - 1],
            lb_data[WorkingSet.indexLB->data[loop_ub] - 1]);
  }
  for (loop_ub = 0; loop_ub < mUB; loop_ub++) {
    TrialState.xstarsqp->data[WorkingSet.indexUB->data[loop_ub] - 1] =
        muDoubleScalarMin(
            TrialState.xstarsqp->data[WorkingSet.indexUB->data[loop_ub] - 1],
            ub_data[WorkingSet.indexUB->data[loop_ub] - 1]);
  }
  for (loop_ub = 0; loop_ub < mFixed; loop_ub++) {
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
      fun_workspace_params->w4, fun_workspace_params->T_th,
      TrialState.xstarsqp);
  nVarMax = 1;
  b = muDoubleScalarIsNaN(fval);
  if (muDoubleScalarIsInf(fval) || b) {
    if (b) {
      nVarMax = -3;
    } else if (fval < 0.0) {
      nVarMax = -1;
    } else {
      nVarMax = -2;
    }
  }
  TrialState.sqpFval = fval;
  if (nVarMax == 1) {
    computeConstraints_(
        r->size[1],
        FcnEvaluator.next.next.next.next.next.next.next.value.workspace.p0,
        FcnEvaluator.next.next.next.next.next.next.next.value.workspace.pf,
        nonlcon_workspace_Fleg_max, nonlcon_workspace_mu,
        nonlcon_workspace_params, x, TrialState.cIneq, TrialState.iNonIneq0);
  }
  i = TrialState.xstarsqp->size[0] * TrialState.xstarsqp->size[1];
  TrialState.xstarsqp->size[0] = 1;
  TrialState.xstarsqp->size[1] = x->size[1];
  emxEnsureCapacity_real_T(TrialState.xstarsqp, i);
  loop_ub = x->size[1];
  for (i = 0; i < loop_ub; i++) {
    TrialState.xstarsqp->data[i] = Hessian_data[i];
  }
  computeFiniteDifferences(&FiniteDifferences, fval, TrialState.cIneq,
                           TrialState.iNonIneq0, TrialState.xstarsqp,
                           TrialState.grad, WorkingSet.Aineq,
                           TrialState.iNonIneq0, WorkingSet.ldA, lb, ub);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  nVarMax = (r->size[1] / 2) << 1;
  emxFree_real_T(&r);
  maxDims = nVarMax - 2;
  for (loop_ub = 0; loop_ub <= maxDims; loop_ub += 2) {
    __m128d r1;
    r1 = _mm_loadu_pd(&TrialState.cIneq->data[loop_ub]);
    _mm_storeu_pd(&WorkingSet.bineq->data[loop_ub],
                  _mm_mul_pd(r1, _mm_set1_pd(-1.0)));
  }
  for (loop_ub = nVarMax; loop_ub < Cineq_size_idx_1; loop_ub++) {
    WorkingSet.bineq->data[loop_ub] = -TrialState.cIneq->data[loop_ub];
  }
  for (loop_ub = 0; loop_ub < mLB; loop_ub++) {
    WorkingSet.lb->data[WorkingSet.indexLB->data[loop_ub] - 1] =
        -lb_data[WorkingSet.indexLB->data[loop_ub] - 1] +
        x0_data[WorkingSet.indexLB->data[loop_ub] - 1];
  }
  for (loop_ub = 0; loop_ub < mUB; loop_ub++) {
    WorkingSet.ub->data[WorkingSet.indexUB->data[loop_ub] - 1] =
        ub_data[WorkingSet.indexUB->data[loop_ub] - 1] -
        x0_data[WorkingSet.indexUB->data[loop_ub] - 1];
  }
  for (loop_ub = 0; loop_ub < mFixed; loop_ub++) {
    normResid = ub_data[WorkingSet.indexFixed->data[loop_ub] - 1] -
                x0_data[WorkingSet.indexFixed->data[loop_ub] - 1];
    WorkingSet.ub->data[WorkingSet.indexFixed->data[loop_ub] - 1] = normResid;
    WorkingSet.bwset->data[loop_ub] = normResid;
  }
  initActiveSet(&WorkingSet);
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initFval = fval;
  MeritFunction.initConstrViolationEq = 0.0;
  normResid = 0.0;
  for (loop_ub = 0; loop_ub < Cineq_size_idx_1; loop_ub++) {
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
  driver(Hessian, lb, ub, &TrialState, &MeritFunction, &FcnEvaluator,
         &FiniteDifferences, &memspace, &WorkingSet, &QRManager, &CholManager,
         &QPObjective, fscales_cineq_constraint);
  emxFree_real_T(&Hessian);
  emxFreeStruct_struct_T6(&WorkingSet);
  emxFree_real_T(&fscales_cineq_constraint);
  emxFreeStruct_struct_T5(&memspace);
  emxFreeStruct_struct_T4(&QPObjective);
  emxFreeStruct_struct_T3(&CholManager);
  emxFreeStruct_struct_T2(&QRManager);
  emxFreeStruct_struct_T1(&FiniteDifferences);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = TrialState.xstarsqp->size[1];
  emxEnsureCapacity_real_T(x, i);
  Hessian_data = x->data;
  loop_ub = TrialState.xstarsqp->size[1];
  for (i = 0; i < loop_ub; i++) {
    Hessian_data[i] = TrialState.xstarsqp->data[i];
  }
  if (x0->size[1] < 1) {
    *output_stepsize = 0.0;
  } else {
    n_t = (ptrdiff_t)x0->size[1];
    incx_t = (ptrdiff_t)1;
    *output_stepsize = dnrm2(&n_t, &TrialState.delta_x->data[0], &incx_t);
  }
  emxFreeStruct_struct_T(&TrialState);
  fval = TrialState.sqpFval;
  *exitflag = TrialState.sqpExitFlag;
  *output_iterations = TrialState.sqpIterations;
  *output_funcCount = TrialState.FunctionEvaluations;
  *output_constrviolation = MeritFunction.nlpPrimalFeasError;
  *output_lssteplength = TrialState.steplength;
  *output_firstorderopt = MeritFunction.firstOrderOpt;
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return fval;
}

/* End of code generation (fmincon.c) */
