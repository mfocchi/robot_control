/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mpc.c
 *
 * Code generation for function 'optimize_cpp_mpc'
 *
 */

/* Include files */
#include "optimize_cpp_mpc.h"
#include "compressBounds.h"
#include "computeFiniteDifferences.h"
#include "cost_mpc.h"
#include "driver.h"
#include "eval_pos_vel_mpc.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "factoryConstruct2.h"
#include "integrate_dynamics.h"
#include "optimize_cpp_mpc_data.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_internal_types.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sumMatrixIncludeNaN.h"
#include "vecnorm.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = {
    11,         /* lineNo */
    9,          /* colNo */
    "cost_mpc", /* fName */
    "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/"
    "optimal_control_2ropes/mpc/cost_mpc.m" /* pName */
};

static emlrtMCInfo emlrtMCI = {
    10,         /* lineNo */
    9,          /* colNo */
    "cost_mpc", /* fName */
    "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/"
    "optimal_control_2ropes/mpc/cost_mpc.m" /* pName */
};

static emlrtMCInfo b_emlrtMCI = {
    9,                  /* lineNo */
    9,                  /* colNo */
    "eval_pos_vel_mpc", /* fName */
    "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/"
    "optimal_control_2ropes/eval_pos_vel_mpc.m" /* pName */
};

static emlrtMCInfo c_emlrtMCI = {
    33,                   /* lineNo */
    13,                   /* colNo */
    "integrate_dynamics", /* fName */
    "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/"
    "optimal_control_2ropes/integrate_dynamics.m" /* pName */
};

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);

static void disp(const mxArray *m, emlrtMCInfo *location);

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *m;
  const mxArray *y;
  const real_T *u_data;
  real_T *pData;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T i;
  u_data = u->data;
  y = NULL;
  b_iv[0] = 3;
  b_iv[1] = u->size[1];
  m = emlrtCreateNumericArray(2, &b_iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u->size[1]; b_i++) {
    pData[i] = u_data[3 * b_i];
    pData[i + 1] = u_data[3 * b_i + 1];
    pData[i + 2] = u_data[3 * b_i + 2];
    i += 3;
  }
  emlrtAssign(&y, m);
  return y;
}

static void disp(const mxArray *m, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = m;
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, NULL, 1, &pArray, "disp", true,
                        location);
}

void optimize_cpp_mpc(const real_T actual_state[6], real_T actual_t,
                      const emxArray_real_T *ref_com,
                      const emxArray_real_T *Fr_l0,
                      const emxArray_real_T *Fr_r0, real_T Fr_max,
                      int64_T mpc_N, const param *params, emxArray_real_T *x,
                      real_T *EXITFLAG, real_T *final_cost)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  c_struct_T QRManager;
  d_struct_T CholManager;
  e_struct_T QPObjective;
  emxArray_real_T *Hessian;
  emxArray_real_T *lb;
  emxArray_real_T *ub;
  emxArray_real_T *x0;
  f_struct_T memspace;
  g_struct_T TrialState;
  h_struct_T WorkingSet;
  i_coder_internal_stickyStruct FcnEvaluator;
  j_struct_T FiniteDifferences;
  struct_T MeritFunction;
  const real_T *Fr_l0_data;
  const real_T *Fr_r0_data;
  const real_T *ref_com_data;
  real_T fval;
  real_T *Hessian_data;
  real_T *lb_data;
  real_T *ub_data;
  real_T *x0_data;
  int32_T b_i;
  int32_T i;
  int32_T mConstrMax;
  int32_T mFixed;
  int32_T maxDims;
  int32_T nVar;
  int32_T nVarMax;
  boolean_T hasLB;
  boolean_T hasUB;
  (void)actual_t;
  Fr_r0_data = Fr_r0->data;
  Fr_l0_data = Fr_l0->data;
  ref_com_data = ref_com->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&x0, 2);
  i = x0->size[0] * x0->size[1];
  x0->size[0] = 1;
  maxDims = (int32_T)mpc_N + (int32_T)mpc_N;
  x0->size[1] = maxDims;
  emxEnsureCapacity_real_T(x0, i);
  x0_data = x0->data;
  nVarMax = (int32_T)mpc_N;
  for (i = 0; i < nVarMax; i++) {
    x0_data[i] = 0.0;
  }
  for (i = 0; i < nVarMax; i++) {
    x0_data[i + (int32_T)mpc_N] = 0.0;
  }
  /* opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r */
  emxInit_real_T(&lb, 2);
  i = lb->size[0] * lb->size[1];
  lb->size[0] = 1;
  lb->size[1] = maxDims;
  emxEnsureCapacity_real_T(lb, i);
  lb_data = lb->data;
  for (i = 0; i < nVarMax; i++) {
    lb_data[i] = -Fr_max;
  }
  for (i = 0; i < nVarMax; i++) {
    lb_data[i + (int32_T)mpc_N] = -Fr_max;
  }
  emxInit_real_T(&ub, 2);
  i = ub->size[0] * ub->size[1];
  ub->size[0] = 1;
  ub->size[1] = maxDims;
  emxEnsureCapacity_real_T(ub, i);
  ub_data = ub->data;
  for (i = 0; i < nVarMax; i++) {
    ub_data[i] = Fr_max;
  }
  for (i = 0; i < nVarMax; i++) {
    ub_data[i + (int32_T)mpc_N] = Fr_max;
  }
  /*  % does not always satisfy bounds */
  /* optim (comment this for sanity check test)  */
  nVar = x0->size[1] - 1;
  mConstrMax = (lb->size[1] + ub->size[1]) + 1;
  nVarMax = x0->size[1] + 1;
  maxDims = muIntScalarMax_sint32(nVarMax, mConstrMax);
  emxInit_real_T(&Hessian, 2);
  i = Hessian->size[0] * Hessian->size[1];
  Hessian->size[0] = x0->size[1];
  Hessian->size[1] = x0->size[1];
  emxEnsureCapacity_real_T(Hessian, i);
  Hessian_data = Hessian->data;
  nVarMax = x0->size[1] * x0->size[1];
  for (i = 0; i < nVarMax; i++) {
    Hessian_data[i] = 0.0;
  }
  if (x0->size[1] > 0) {
    for (nVarMax = 0; nVarMax <= nVar; nVarMax++) {
      Hessian_data[nVarMax + Hessian->size[0] * nVarMax] = 1.0;
    }
  }
  emxInitStruct_struct_T(&TrialState);
  factoryConstruct(x0->size[1] + 1, mConstrMax, x0, &TrialState);
  if (x0->size[1] >= 1) {
    n_t = (ptrdiff_t)x0->size[1];
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &x0_data[0], &incx_t, &TrialState.xstarsqp->data[0], &incy_t);
  }
  c_emxInitStruct_coder_internal_(&FcnEvaluator);
  for (b_i = 0; b_i < 6; b_i++) {
    FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
        .actual_state[b_i] = actual_state[b_i];
  }
  i = FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
          .ref_com->size[0] *
      FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
          .ref_com->size[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.ref_com
      ->size[0] = 3;
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.ref_com
      ->size[1] = ref_com->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.next.next.next.next.next.next.next.next
                               .value.workspace.ref_com,
                           i);
  nVarMax = 3 * ref_com->size[1];
  for (i = 0; i < nVarMax; i++) {
    FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
        .ref_com->data[i] = ref_com_data[i];
  }
  i = FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
          .Fr_l0->size[0] *
      FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
          .Fr_l0->size[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.Fr_l0
      ->size[0] = 1;
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.Fr_l0
      ->size[1] = Fr_l0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.next.next.next.next.next.next.next.next
                               .value.workspace.Fr_l0,
                           i);
  nVarMax = Fr_l0->size[1];
  for (i = 0; i < nVarMax; i++) {
    FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.Fr_l0
        ->data[i] = Fr_l0_data[i];
  }
  i = FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
          .Fr_r0->size[0] *
      FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
          .Fr_r0->size[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.Fr_r0
      ->size[0] = 1;
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.Fr_r0
      ->size[1] = Fr_r0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.next.next.next.next.next.next.next.next
                               .value.workspace.Fr_r0,
                           i);
  nVarMax = Fr_r0->size[1];
  for (i = 0; i < nVarMax; i++) {
    FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.Fr_r0
        ->data[i] = Fr_r0_data[i];
  }
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.mpc_N =
      mpc_N;
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.params =
      *params;
  emxInitStruct_struct_T2(&FiniteDifferences);
  b_factoryConstruct(actual_state, ref_com, Fr_l0, Fr_r0, mpc_N, params,
                     x0->size[1], lb, ub, &FiniteDifferences);
  emxInitStruct_struct_T3(&QRManager);
  QRManager.ldq = maxDims;
  i = QRManager.QR->size[0] * QRManager.QR->size[1];
  QRManager.QR->size[0] = maxDims;
  QRManager.QR->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.QR, i);
  i = QRManager.Q->size[0] * QRManager.Q->size[1];
  QRManager.Q->size[0] = maxDims;
  QRManager.Q->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.Q, i);
  nVarMax = maxDims * maxDims;
  for (i = 0; i < nVarMax; i++) {
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
  emxInitStruct_struct_T4(&CholManager);
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
  emxInitStruct_struct_T5(&QPObjective);
  i = QPObjective.grad->size[0];
  QPObjective.grad->size[0] = x0->size[1] + 1;
  emxEnsureCapacity_real_T(QPObjective.grad, i);
  i = QPObjective.Hx->size[0];
  QPObjective.Hx->size[0] = x0->size[1];
  emxEnsureCapacity_real_T(QPObjective.Hx, i);
  QPObjective.maxVar = x0->size[1] + 1;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.nvar = x0->size[1];
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  emxInitStruct_struct_T6(&memspace);
  i = memspace.workspace_double->size[0] * memspace.workspace_double->size[1];
  memspace.workspace_double->size[0] = maxDims;
  nVar = x0->size[1] + 1;
  memspace.workspace_double->size[1] = muIntScalarMax_sint32(nVar, 2);
  emxEnsureCapacity_real_T(memspace.workspace_double, i);
  i = memspace.workspace_int->size[0];
  memspace.workspace_int->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_int, i);
  i = memspace.workspace_sort->size[0];
  memspace.workspace_sort->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_sort, i);
  emxInitStruct_struct_T7(&WorkingSet);
  c_factoryConstruct(x0->size[1], x0->size[1] + 1, mConstrMax, &WorkingSet);
  nVar = compressBounds(x0->size[1], WorkingSet.indexLB, WorkingSet.indexUB,
                        WorkingSet.indexFixed, lb, ub, &maxDims, &mFixed);
  emxFree_real_T(&x0);
  WorkingSet.mConstrMax = mConstrMax;
  nVarMax = (nVar + maxDims) + mFixed;
  WorkingSet.mConstr = nVarMax;
  WorkingSet.mConstrOrig = nVarMax;
  WorkingSet.sizes[0] = mFixed;
  WorkingSet.sizes[1] = 0;
  WorkingSet.sizes[2] = 0;
  WorkingSet.sizes[3] = nVar;
  WorkingSet.sizes[4] = maxDims;
  WorkingSet.sizesPhaseOne[0] = mFixed;
  WorkingSet.sizesPhaseOne[1] = 0;
  WorkingSet.sizesPhaseOne[2] = 0;
  WorkingSet.sizesPhaseOne[3] = nVar + 1;
  WorkingSet.sizesPhaseOne[4] = maxDims;
  WorkingSet.isActiveIdx[0] = 1;
  WorkingSet.isActiveIdx[1] = mFixed;
  WorkingSet.isActiveIdx[2] = 0;
  WorkingSet.isActiveIdx[3] = 0;
  WorkingSet.isActiveIdx[4] = nVar;
  WorkingSet.isActiveIdx[5] = maxDims;
  for (nVarMax = 0; nVarMax < 5; nVarMax++) {
    i = WorkingSet.sizes[nVarMax];
    WorkingSet.sizesNormal[nVarMax] = i;
    WorkingSet.sizesRegularized[nVarMax] = i;
    WorkingSet.sizesRegPhaseOne[nVarMax] = WorkingSet.sizesPhaseOne[nVarMax];
    WorkingSet.isActiveIdx[nVarMax + 1] += WorkingSet.isActiveIdx[nVarMax];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxNormal[i] = WorkingSet.isActiveIdx[i];
  }
  WorkingSet.isActiveIdxPhaseOne[0] = 1;
  WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxPhaseOne[2] = 0;
  WorkingSet.isActiveIdxPhaseOne[3] = 0;
  WorkingSet.isActiveIdxPhaseOne[4] = nVar + 1;
  WorkingSet.isActiveIdxPhaseOne[5] = maxDims;
  for (nVarMax = 0; nVarMax < 5; nVarMax++) {
    WorkingSet.isActiveIdxPhaseOne[nVarMax + 1] +=
        WorkingSet.isActiveIdxPhaseOne[nVarMax];
  }
  for (i = 0; i < 6; i++) {
    WorkingSet.isActiveIdxRegularized[i] = WorkingSet.isActiveIdx[i];
    WorkingSet.isActiveIdxRegPhaseOne[i] = WorkingSet.isActiveIdxPhaseOne[i];
  }
  if (lb->size[1] != 0) {
    for (mConstrMax = 0; mConstrMax < nVar; mConstrMax++) {
      TrialState.xstarsqp
          ->data[WorkingSet.indexLB->data[mConstrMax] - 1] = muDoubleScalarMax(
          TrialState.xstarsqp->data[WorkingSet.indexLB->data[mConstrMax] - 1],
          lb_data[WorkingSet.indexLB->data[mConstrMax] - 1]);
    }
  }
  if (ub->size[1] != 0) {
    for (mConstrMax = 0; mConstrMax < maxDims; mConstrMax++) {
      TrialState.xstarsqp
          ->data[WorkingSet.indexUB->data[mConstrMax] - 1] = muDoubleScalarMin(
          TrialState.xstarsqp->data[WorkingSet.indexUB->data[mConstrMax] - 1],
          ub_data[WorkingSet.indexUB->data[mConstrMax] - 1]);
    }
    for (mConstrMax = 0; mConstrMax < mFixed; mConstrMax++) {
      TrialState.xstarsqp->data[WorkingSet.indexFixed->data[mConstrMax] - 1] =
          ub_data[WorkingSet.indexFixed->data[mConstrMax] - 1];
    }
  }
  fval = optimize_cpp_mpc_anonFcn1(
      actual_state, ref_com, Fr_l0, Fr_r0, mpc_N, params->int_method,
      params->int_steps, params->b, params->p_a1, params->p_a2, params->g,
      params->m, params->w1, params->mpc_dt, TrialState.xstarsqp);
  TrialState.sqpFval = fval;
  computeFiniteDifferences(&FiniteDifferences, fval, TrialState.xstarsqp,
                           TrialState.grad, lb, ub);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  hasLB = (lb->size[1] != 0);
  hasUB = (ub->size[1] != 0);
  if (hasLB) {
    for (mConstrMax = 0; mConstrMax < nVar; mConstrMax++) {
      WorkingSet.lb->data[WorkingSet.indexLB->data[mConstrMax] - 1] =
          -lb_data[WorkingSet.indexLB->data[mConstrMax] - 1];
    }
  }
  if (hasUB) {
    for (mConstrMax = 0; mConstrMax < maxDims; mConstrMax++) {
      WorkingSet.ub->data[WorkingSet.indexUB->data[mConstrMax] - 1] =
          ub_data[WorkingSet.indexUB->data[mConstrMax] - 1];
    }
  }
  if (hasLB && hasUB) {
    for (mConstrMax = 0; mConstrMax < mFixed; mConstrMax++) {
      real_T d;
      d = ub_data[WorkingSet.indexFixed->data[mConstrMax] - 1];
      WorkingSet.ub->data[WorkingSet.indexFixed->data[mConstrMax] - 1] = d;
      WorkingSet.bwset->data[mConstrMax] = d;
    }
  }
  setProblemType(&WorkingSet, 3);
  nVarMax = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  for (mConstrMax = nVarMax; mConstrMax <= i; mConstrMax++) {
    WorkingSet.isActiveConstr->data[mConstrMax - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  nVarMax = WorkingSet.sizes[0];
  for (mFixed = 0; mFixed < nVarMax; mFixed++) {
    WorkingSet.Wid->data[mFixed] = 1;
    WorkingSet.Wlocalidx->data[mFixed] = mFixed + 1;
    WorkingSet.isActiveConstr->data[mFixed] = true;
    nVar = WorkingSet.ldA * mFixed;
    i = WorkingSet.indexFixed->data[mFixed];
    for (b_i = 0; b_i <= i - 2; b_i++) {
      WorkingSet.ATwset->data[b_i + nVar] = 0.0;
    }
    WorkingSet.ATwset->data[(WorkingSet.indexFixed->data[mFixed] + nVar) - 1] =
        1.0;
    i = WorkingSet.indexFixed->data[mFixed] + 1;
    maxDims = WorkingSet.nVar;
    for (b_i = i; b_i <= maxDims; b_i++) {
      WorkingSet.ATwset->data[(b_i + nVar) - 1] = 0.0;
    }
    WorkingSet.bwset->data[mFixed] =
        WorkingSet.ub->data[WorkingSet.indexFixed->data[mFixed] - 1];
  }
  MeritFunction.initFval = fval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initConstrViolationEq = 0.0;
  MeritFunction.initConstrViolationIneq = 0.0;
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
         &QPObjective);
  emxFree_real_T(&Hessian);
  emxFreeStruct_struct_T7(&WorkingSet);
  emxFreeStruct_struct_T6(&memspace);
  emxFreeStruct_struct_T5(&QPObjective);
  emxFreeStruct_struct_T4(&CholManager);
  emxFreeStruct_struct_T3(&QRManager);
  emxFreeStruct_struct_T2(&FiniteDifferences);
  k_emxFreeStruct_coder_internal_(&FcnEvaluator);
  emxFree_real_T(&ub);
  emxFree_real_T(&lb);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = TrialState.xstarsqp->size[1];
  emxEnsureCapacity_real_T(x, i);
  x0_data = x->data;
  nVarMax = TrialState.xstarsqp->size[1];
  for (i = 0; i < nVarMax; i++) {
    x0_data[i] = TrialState.xstarsqp->data[i];
  }
  emxFreeStruct_struct_T(&TrialState);
  /* ,  @(x) constraints_mpc(x, actual_com, ref_com, Fr_l0, Fr_r0 ) , options);
   */
  *EXITFLAG = TrialState.sqpExitFlag;
  *final_cost = TrialState.sqpFval;
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

real_T optimize_cpp_mpc_anonFcn1(
    const real_T actual_state[6], const emxArray_real_T *ref_com,
    const emxArray_real_T *Fr_l0, const emxArray_real_T *Fr_r0, int64_T mpc_N,
    const char_T params_int_method[3], real_T params_int_steps, real_T params_b,
    const real_T params_p_a1[3], const real_T params_p_a2[3], real_T params_g,
    real_T params_m, real_T params_w1, real_T params_mpc_dt,
    const emxArray_real_T *x)
{
  static const int32_T b_iv[2] = {1, 72};
  static const int32_T iv1[2] = {1, 66};
  static const int32_T iv2[2] = {1, 15};
  static const int32_T iv3[2] = {1, 15};
  static const char_T u[72] = {
      'c', 'o', 's', 't', '_', 'm', 'p', 'c', ':', 'w', 'r', 'o', 'n', 'g', ' ',
      'r', 'e', 'f', '_', 'c', 'o', 'm', ' ', 'i', 'n', 'p', 'u', 't', ' ', 'l',
      'e', 'n', 'g', 't', 'h', ':', ' ', 'r', 'e', 'f', '_', 'c', 'o', 'm', ' ',
      's', 'h', 'o', 'u', 'l', 'd', ' ', 'b', 'e', ' ', 'l', 'o', 'n', 'g', 'e',
      'r', ' ', 't', 'h', 'a', 'n', ' ', 'm', 'p', 'c', '_', 'N'};
  static const char_T b_u[66] = {
      'e', 'v', 'a', 'l', '_', 'p', 'o', 's', '_', 'm', 'p', 'c', ':', 'w',
      'r', 'o', 'n', 'g', ' ', 'i', 'n', 'p', 'u', 't', ' ', 'l', 'e', 'n',
      'g', 't', 'h', ':', ' ', 'i', 'n', 'p', 'u', 't', ' ', 's', 'h', 'o',
      'u', 'l', 'd', ' ', 'b', 'e', ' ', 'l', 'o', 'n', 'g', 'e', 'r', ' ',
      't', 'h', 'a', 'n', ' ', 'm', 'p', 'c', '_', 'N'};
  static const char_T c_u[15] = {'U', 'n', 'k', 'n', 'o', 'w', 'n', ' ',
                                 'm', 'e', 't', 'h', 'o', 'd', '.'};
  static const char_T b[3] = {'r', 'k', '4'};
  emxArray_real_T b_px;
  emxArray_real_T *b_p;
  emxArray_real_T *b_ref_com;
  emxArray_real_T *px;
  emxArray_real_T *py;
  emxArray_real_T *pz;
  emxArray_real_T *states_rough;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *m;
  const mxArray *y;
  real_T _1[6];
  real_T a__1[6];
  real_T dv[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  const real_T *Fr_l0_data;
  const real_T *Fr_r0_data;
  const real_T *ref_com_data;
  const real_T *x_data;
  real_T a_tmp;
  real_T varargout_1;
  real_T *p_data;
  real_T *px_data;
  real_T *py_data;
  real_T *pz_data;
  real_T *states_rough_data;
  int32_T b_Fr_l0;
  int32_T c_Fr_l0;
  int32_T d_Fr_l0;
  int32_T e_Fr_l0;
  int32_T i;
  int32_T ib;
  int32_T nfb;
  boolean_T p;
  x_data = x->data;
  Fr_r0_data = Fr_r0->data;
  Fr_l0_data = Fr_l0->data;
  ref_com_data = ref_com->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /*  init for cpp */
  varargout_1 = 0.0;
  p = false;
  if (mpc_N >= 4503599627370496L) {
    p = true;
  } else if (mpc_N > -4503599627370496L) {
    if (ref_com->size[1] == 0) {
      i = 0;
    } else if (ref_com->size[1] < 3) {
      i = 3;
    } else {
      i = ref_com->size[1];
    }
    p = ((real_T)i < mpc_N);
  }
  if (p) {
    y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 72, m, &u[0]);
    emlrtAssign(&y, m);
    disp(y, &emlrtMCI);
    emlrtDisplayR2012b(b_emlrt_marshallOut(ref_com), "ref_com", &emlrtRTEI,
                       emlrtRootTLSGlobal);
  } else {
    __m128d r;
    int64_T b_i;
    real_T dt_step;
    int32_T b_loop_ub;
    int32_T c_loop_ub;
    int32_T inb;
    int32_T loop_ub;
    int32_T nleft;
    boolean_T guard1;
    if (mpc_N < 1L) {
      i = 0;
    } else {
      i = (int32_T)mpc_N;
    }
    b_i = mpc_N << 1;
    if (mpc_N + 1L > b_i) {
      nleft = 0;
      ib = 0;
    } else {
      nleft = (int32_T)(mpc_N + 1L) - 1;
      ib = (int32_T)b_i;
    }
    if (mpc_N < 1L) {
      loop_ub = 0;
    } else {
      loop_ub = (int32_T)mpc_N;
    }
    /* init values for cpp */
    emxInit_real_T(&b_p, 2);
    nfb = b_p->size[0] * b_p->size[1];
    b_p->size[0] = 3;
    b_p->size[1] = (int32_T)mpc_N;
    emxEnsureCapacity_real_T(b_p, nfb);
    p_data = b_p->data;
    b_loop_ub = 3 * (int32_T)mpc_N;
    for (nfb = 0; nfb < b_loop_ub; nfb++) {
      p_data[nfb] = 0.0;
    }
    p = false;
    if (mpc_N >= 4503599627370496L) {
      p = true;
    } else if (mpc_N > -4503599627370496L) {
      p = ((real_T)Fr_l0->size[1] < mpc_N);
    }
    emxInit_real_T(&states_rough, 2);
    emxInit_real_T(&px, 2);
    emxInit_real_T(&py, 2);
    emxInit_real_T(&pz, 2);
    guard1 = false;
    if (p) {
      guard1 = true;
    } else {
      p = false;
      if (mpc_N >= 4503599627370496L) {
        p = true;
      } else if (mpc_N > -4503599627370496L) {
        p = ((real_T)Fr_r0->size[1] < mpc_N);
      }
      if (p) {
        guard1 = true;
      } else {
        real_T b_k_1_tmp;
        real_T k_1_tmp;
        /*  check vectors are row and extract first mpc_N elements */
        if (mpc_N < 1L) {
          b_loop_ub = 0;
          c_loop_ub = 0;
        } else {
          b_loop_ub = (int32_T)mpc_N;
          c_loop_ub = (int32_T)mpc_N;
        }
        /*  single shooting */
        if (b_loop_ub == i) {
          i = px->size[0] * px->size[1];
          px->size[0] = 1;
          px->size[1] = b_loop_ub;
          emxEnsureCapacity_real_T(px, i);
          px_data = px->data;
          nfb = (b_loop_ub / 2) << 1;
          inb = nfb - 2;
          for (i = 0; i <= inb; i += 2) {
            _mm_storeu_pd(&px_data[i], _mm_add_pd(_mm_loadu_pd(&Fr_l0_data[i]),
                                                  _mm_loadu_pd(&x_data[i])));
          }
          for (i = nfb; i < b_loop_ub; i++) {
            px_data[i] = Fr_l0_data[i] + x_data[i];
          }
        } else {
          b_binary_expand_op(px, Fr_l0, b_loop_ub, x, 1, i);
          px_data = px->data;
        }
        if (c_loop_ub == ib - nleft) {
          i = py->size[0] * py->size[1];
          py->size[0] = 1;
          py->size[1] = c_loop_ub;
          emxEnsureCapacity_real_T(py, i);
          py_data = py->data;
          nfb = (c_loop_ub / 2) << 1;
          inb = nfb - 2;
          for (i = 0; i <= inb; i += 2) {
            _mm_storeu_pd(&py_data[i],
                          _mm_add_pd(_mm_loadu_pd(&Fr_r0_data[i]),
                                     _mm_loadu_pd(&x_data[nleft + i])));
          }
          for (i = nfb; i < c_loop_ub; i++) {
            py_data[i] = Fr_r0_data[i] + x_data[nleft + i];
          }
        } else {
          b_binary_expand_op(py, Fr_r0, c_loop_ub, x, nleft + 1, ib);
          py_data = py->data;
        }
        /* init */
        i = states_rough->size[0] * states_rough->size[1];
        states_rough->size[0] = 6;
        states_rough->size[1] = (int32_T)mpc_N;
        emxEnsureCapacity_real_T(states_rough, i);
        states_rough_data = states_rough->data;
        b_loop_ub = 6 * (int32_T)mpc_N;
        for (i = 0; i < b_loop_ub; i++) {
          states_rough_data[i] = 0.0;
        }
        if (params_int_steps == 0.0) {
          /* verify is a column vector */
          i = states_rough->size[0] * states_rough->size[1];
          states_rough->size[0] = 6;
          states_rough->size[1] = 1;
          emxEnsureCapacity_real_T(states_rough, i);
          states_rough_data = states_rough->data;
          for (nfb = 0; nfb < 6; nfb++) {
            a__1[nfb] = actual_state[nfb];
            states_rough_data[nfb] = actual_state[nfb];
          }
          if (memcmp((char_T *)&params_int_method[0], (char_T *)&b[0], 3) ==
              0) {
            /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
             */
            /*  we have  time invariant dynamics so t wont count */
            if (mpc_N - 1L >= 1L) {
              a_tmp = 0.5 * params_mpc_dt;
            }
            b_i = 1L;
            while (b_i <= mpc_N - 1L) {
              __m128d r1;
              __m128d r2;
              __m128d r3;
              __m128d r4;
              __m128d r5;
              __m128d r6;
              __m128d r7;
              k_1_tmp = px_data[(int32_T)b_i - 1];
              b_k_1_tmp = py_data[(int32_T)b_i - 1];
              integrate_dynamics_anonFcn1(params_b, params_p_a1, params_p_a2,
                                          params_g, params_m, a__1, k_1_tmp,
                                          b_k_1_tmp, k_1);
              r = _mm_loadu_pd(&k_1[0]);
              r1 = _mm_loadu_pd(&a__1[0]);
              r2 = _mm_set1_pd(a_tmp);
              _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
              r = _mm_loadu_pd(&k_1[2]);
              r1 = _mm_loadu_pd(&a__1[2]);
              _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
              r = _mm_loadu_pd(&k_1[4]);
              r1 = _mm_loadu_pd(&a__1[4]);
              _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
              integrate_dynamics_anonFcn1(params_b, params_p_a1, params_p_a2,
                                          params_g, params_m, _1, k_1_tmp,
                                          b_k_1_tmp, k_2);
              r = _mm_loadu_pd(&k_2[0]);
              r1 = _mm_loadu_pd(&a__1[0]);
              r2 = _mm_set1_pd(a_tmp);
              _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
              r = _mm_loadu_pd(&k_2[2]);
              r1 = _mm_loadu_pd(&a__1[2]);
              _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
              r = _mm_loadu_pd(&k_2[4]);
              r1 = _mm_loadu_pd(&a__1[4]);
              _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
              integrate_dynamics_anonFcn1(params_b, params_p_a1, params_p_a2,
                                          params_g, params_m, _1, k_1_tmp,
                                          b_k_1_tmp, k_3);
              r = _mm_loadu_pd(&k_3[0]);
              r1 = _mm_loadu_pd(&a__1[0]);
              r2 = _mm_set1_pd(params_mpc_dt);
              _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
              r = _mm_loadu_pd(&k_3[2]);
              r1 = _mm_loadu_pd(&a__1[2]);
              _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
              r = _mm_loadu_pd(&k_3[4]);
              r1 = _mm_loadu_pd(&a__1[4]);
              _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
              integrate_dynamics_anonFcn1(params_b, params_p_a1, params_p_a2,
                                          params_g, params_m, _1, k_1_tmp,
                                          b_k_1_tmp, dv);
              i = states_rough->size[1];
              nleft = states_rough->size[0] * states_rough->size[1];
              states_rough->size[0] = 6;
              states_rough->size[1]++;
              emxEnsureCapacity_real_T(states_rough, nleft);
              states_rough_data = states_rough->data;
              r = _mm_loadu_pd(&k_2[0]);
              r1 = _mm_loadu_pd(&k_1[0]);
              r3 = _mm_loadu_pd(&k_3[0]);
              r4 = _mm_loadu_pd(&dv[0]);
              r5 = _mm_loadu_pd(&a__1[0]);
              r6 = _mm_set1_pd(2.0);
              r7 = _mm_set1_pd(0.16666666666666666);
              r = _mm_add_pd(
                  r5,
                  _mm_mul_pd(
                      _mm_mul_pd(
                          r7, _mm_add_pd(
                                  _mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                             _mm_mul_pd(r6, r3)),
                                  r4)),
                      r2));
              _mm_storeu_pd(&a__1[0], r);
              _mm_storeu_pd(&states_rough_data[6 * i], r);
              r = _mm_loadu_pd(&k_2[2]);
              r1 = _mm_loadu_pd(&k_1[2]);
              r3 = _mm_loadu_pd(&k_3[2]);
              r4 = _mm_loadu_pd(&dv[2]);
              r5 = _mm_loadu_pd(&a__1[2]);
              r = _mm_add_pd(
                  r5,
                  _mm_mul_pd(
                      _mm_mul_pd(
                          r7, _mm_add_pd(
                                  _mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                             _mm_mul_pd(r6, r3)),
                                  r4)),
                      r2));
              _mm_storeu_pd(&a__1[2], r);
              _mm_storeu_pd(&states_rough_data[6 * i + 2], r);
              r = _mm_loadu_pd(&k_2[4]);
              r1 = _mm_loadu_pd(&k_1[4]);
              r3 = _mm_loadu_pd(&k_3[4]);
              r4 = _mm_loadu_pd(&dv[4]);
              r5 = _mm_loadu_pd(&a__1[4]);
              r = _mm_add_pd(
                  r5,
                  _mm_mul_pd(
                      _mm_mul_pd(
                          r7, _mm_add_pd(
                                  _mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                             _mm_mul_pd(r6, r3)),
                                  r4)),
                      r2));
              _mm_storeu_pd(&a__1[4], r);
              _mm_storeu_pd(&states_rough_data[6 * i + 4], r);
              b_i++;
              if (*emlrtBreakCheckR2012bFlagVar != 0) {
                emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
              }
            }
          } else {
            d_y = NULL;
            m = emlrtCreateCharArray(2, &iv3[0]);
            emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &c_u[0]);
            emlrtAssign(&d_y, m);
            disp(d_y, &c_emlrtMCI);
          }
        } else {
          dt_step = params_mpc_dt / (params_int_steps - 1.0);
          b_i = 0L;
          while (b_i + 1L <= mpc_N) {
            if (b_i + 1L >= 2L) {
              for (i = 0; i < 6; i++) {
                dv[i] = states_rough_data[i + 6 * ((int32_T)b_i - 1)];
              }
              /* verify is a column vector */
              if (memcmp((char_T *)&params_int_method[0], (char_T *)&b[0], 3) ==
                  0) {
                /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
                 */
                /*  we have  time invariant dynamics so t wont count */
                i = (int32_T)(params_int_steps - 1.0);
                if ((int32_T)(params_int_steps - 1.0) - 1 >= 0) {
                  a_tmp = 0.5 * dt_step;
                }
                for (nfb = 0; nfb < i; nfb++) {
                  __m128d r1;
                  __m128d r2;
                  __m128d r3;
                  __m128d r4;
                  __m128d r5;
                  __m128d r6;
                  __m128d r7;
                  k_1_tmp = px_data[(int32_T)b_i - 1];
                  b_k_1_tmp = py_data[(int32_T)b_i - 1];
                  integrate_dynamics_anonFcn1(params_b, params_p_a1,
                                              params_p_a2, params_g, params_m,
                                              dv, k_1_tmp, b_k_1_tmp, k_1);
                  r = _mm_loadu_pd(&k_1[0]);
                  r1 = _mm_loadu_pd(&dv[0]);
                  r2 = _mm_set1_pd(a_tmp);
                  _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
                  r = _mm_loadu_pd(&k_1[2]);
                  r1 = _mm_loadu_pd(&dv[2]);
                  _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
                  r = _mm_loadu_pd(&k_1[4]);
                  r1 = _mm_loadu_pd(&dv[4]);
                  _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
                  integrate_dynamics_anonFcn1(params_b, params_p_a1,
                                              params_p_a2, params_g, params_m,
                                              _1, k_1_tmp, b_k_1_tmp, k_2);
                  r = _mm_loadu_pd(&k_2[0]);
                  r1 = _mm_loadu_pd(&dv[0]);
                  r2 = _mm_set1_pd(a_tmp);
                  _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
                  r = _mm_loadu_pd(&k_2[2]);
                  r1 = _mm_loadu_pd(&dv[2]);
                  _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
                  r = _mm_loadu_pd(&k_2[4]);
                  r1 = _mm_loadu_pd(&dv[4]);
                  _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
                  integrate_dynamics_anonFcn1(params_b, params_p_a1,
                                              params_p_a2, params_g, params_m,
                                              _1, k_1_tmp, b_k_1_tmp, k_3);
                  r = _mm_loadu_pd(&k_3[0]);
                  r1 = _mm_loadu_pd(&dv[0]);
                  r2 = _mm_set1_pd(dt_step);
                  _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
                  r = _mm_loadu_pd(&k_3[2]);
                  r1 = _mm_loadu_pd(&dv[2]);
                  _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
                  r = _mm_loadu_pd(&k_3[4]);
                  r1 = _mm_loadu_pd(&dv[4]);
                  _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
                  integrate_dynamics_anonFcn1(params_b, params_p_a1,
                                              params_p_a2, params_g, params_m,
                                              _1, k_1_tmp, b_k_1_tmp, a__1);
                  r = _mm_loadu_pd(&k_2[0]);
                  r1 = _mm_loadu_pd(&k_1[0]);
                  r3 = _mm_loadu_pd(&k_3[0]);
                  r4 = _mm_loadu_pd(&a__1[0]);
                  r5 = _mm_loadu_pd(&dv[0]);
                  r6 = _mm_set1_pd(2.0);
                  r7 = _mm_set1_pd(0.16666666666666666);
                  _mm_storeu_pd(
                      &dv[0],
                      _mm_add_pd(
                          r5,
                          _mm_mul_pd(
                              _mm_mul_pd(
                                  r7, _mm_add_pd(
                                          _mm_add_pd(
                                              _mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                          r4)),
                              r2)));
                  r = _mm_loadu_pd(&k_2[2]);
                  r1 = _mm_loadu_pd(&k_1[2]);
                  r3 = _mm_loadu_pd(&k_3[2]);
                  r4 = _mm_loadu_pd(&a__1[2]);
                  r5 = _mm_loadu_pd(&dv[2]);
                  _mm_storeu_pd(
                      &dv[2],
                      _mm_add_pd(
                          r5,
                          _mm_mul_pd(
                              _mm_mul_pd(
                                  r7, _mm_add_pd(
                                          _mm_add_pd(
                                              _mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                          r4)),
                              r2)));
                  r = _mm_loadu_pd(&k_2[4]);
                  r1 = _mm_loadu_pd(&k_1[4]);
                  r3 = _mm_loadu_pd(&k_3[4]);
                  r4 = _mm_loadu_pd(&a__1[4]);
                  r5 = _mm_loadu_pd(&dv[4]);
                  _mm_storeu_pd(
                      &dv[4],
                      _mm_add_pd(
                          r5,
                          _mm_mul_pd(
                              _mm_mul_pd(
                                  r7, _mm_add_pd(
                                          _mm_add_pd(
                                              _mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                          r4)),
                              r2)));
                  if (*emlrtBreakCheckR2012bFlagVar != 0) {
                    emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
                  }
                }
              } else {
                c_y = NULL;
                m = emlrtCreateCharArray(2, &iv2[0]);
                emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &c_u[0]);
                emlrtAssign(&c_y, m);
                disp(c_y, &c_emlrtMCI);
              }
              for (i = 0; i < 6; i++) {
                states_rough_data[i + 6 * ((int32_T)(b_i + 1L) - 1)] = dv[i];
              }
              /*  keep Fr constant            */
            } else {
              for (i = 0; i < 6; i++) {
                states_rough_data[i + 6 * ((int32_T)(b_i + 1L) - 1)] =
                    actual_state[i];
              }
            }
            b_i++;
            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
            }
          }
        }
        i = px->size[0] * px->size[1];
        px->size[0] = 1;
        px->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(px, i);
        px_data = px->data;
        b_loop_ub = states_rough->size[1];
        i = py->size[0] * py->size[1];
        py->size[0] = 1;
        py->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(py, i);
        py_data = py->data;
        i = pz->size[0] * pz->size[1];
        pz->size[0] = 1;
        pz->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(pz, i);
        pz_data = pz->data;
        for (i = 0; i < b_loop_ub; i++) {
          px_data[i] = 0.0;
          py_data[i] = 0.0;
          pz_data[i] = 0.0;
        }
        i = states_rough->size[1];
        for (nfb = 0; nfb < i; nfb++) {
          real_T d;
          real_T d1;
          d = states_rough_data[6 * nfb];
          d1 = states_rough_data[6 * nfb + 1];
          dt_step = states_rough_data[6 * nfb + 2];
          k_1_tmp = params_b * params_b;
          b_k_1_tmp = d1 * d1;
          a_tmp = (k_1_tmp + b_k_1_tmp) - dt_step * dt_step;
          dt_step = muDoubleScalarSqrt(1.0 - a_tmp * a_tmp /
                                                 (4.0 * k_1_tmp * b_k_1_tmp));
          px_data[nfb] = d1 * muDoubleScalarSin(d) * dt_step;
          py_data[nfb] = a_tmp / (2.0 * params_b);
          pz_data[nfb] = -d1 * muDoubleScalarCos(d) * dt_step;
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
          }
        }
        i = b_p->size[0] * b_p->size[1];
        b_p->size[0] = 3;
        b_p->size[1] = px->size[1];
        emxEnsureCapacity_real_T(b_p, i);
        p_data = b_p->data;
        b_loop_ub = px->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          p_data[3 * i] = px_data[i];
          p_data[3 * i + 1] = py_data[i];
          p_data[3 * i + 2] = pz_data[i];
        }
      }
    }
    if (guard1) {
      b_y = NULL;
      m = emlrtCreateCharArray(2, &iv1[0]);
      emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 66, m, &b_u[0]);
      emlrtAssign(&b_y, m);
      disp(b_y, &b_emlrtMCI);
    }
    emxFree_real_T(&pz);
    emxFree_real_T(&py);
    emxFree_real_T(&states_rough);
    /* p has mpc_N +1 elements  */
    /*  track */
    /*  smoothnes: minimize jerky control action */
    if (loop_ub == b_p->size[1]) {
      emxInit_real_T(&b_ref_com, 2);
      i = b_ref_com->size[0] * b_ref_com->size[1];
      b_ref_com->size[0] = 3;
      b_ref_com->size[1] = loop_ub;
      emxEnsureCapacity_real_T(b_ref_com, i);
      states_rough_data = b_ref_com->data;
      for (i = 0; i < loop_ub; i++) {
        r = _mm_loadu_pd(&p_data[3 * i]);
        _mm_storeu_pd(&states_rough_data[3 * i],
                      _mm_sub_pd(_mm_loadu_pd(&ref_com_data[3 * i]), r));
        nleft = 3 * i + 2;
        states_rough_data[nleft] = ref_com_data[nleft] - p_data[nleft];
      }
      vecnorm(b_ref_com, px);
      emxFree_real_T(&b_ref_com);
    } else {
      binary_expand_op(px, ref_com, loop_ub, b_p);
    }
    emxFree_real_T(&b_p);
    i = px->size[0] * px->size[1];
    px->size[0] = 1;
    emxEnsureCapacity_real_T(px, i);
    px_data = px->data;
    loop_ub = px->size[1] - 1;
    for (i = 0; i <= loop_ub; i++) {
      dt_step = px_data[i];
      px_data[i] = dt_step * dt_step;
    }
    if (px->size[1] == 0) {
      dt_step = 0.0;
    } else if (px->size[1] < 4096) {
      c_loop_ub = px->size[1];
      b_px = *px;
      b_Fr_l0 = c_loop_ub;
      b_px.size = &b_Fr_l0;
      b_px.numDimensions = 1;
      dt_step = sumColumnB(&b_px, px->size[1]);
    } else {
      nfb = (int32_T)((uint32_T)px->size[1] >> 12);
      inb = nfb << 12;
      nleft = px->size[1] - inb;
      c_loop_ub = px->size[1];
      b_px = *px;
      c_Fr_l0 = c_loop_ub;
      b_px.size = &c_Fr_l0;
      b_px.numDimensions = 1;
      dt_step = sumColumnB4(&b_px, 1);
      if (nfb >= 2) {
        c_loop_ub = px->size[1];
      }
      for (ib = 2; ib <= nfb; ib++) {
        b_px = *px;
        d_Fr_l0 = c_loop_ub;
        b_px.size = &d_Fr_l0;
        b_px.numDimensions = 1;
        dt_step += sumColumnB4(&b_px, ((ib - 1) << 12) + 1);
      }
      if (nleft > 0) {
        c_loop_ub = px->size[1];
        b_px = *px;
        e_Fr_l0 = c_loop_ub;
        b_px.size = &e_Fr_l0;
        b_px.numDimensions = 1;
        dt_step += b_sumColumnB(&b_px, nleft, inb + 1);
      }
    }
    emxFree_real_T(&px);
    varargout_1 = params_w1 * dt_step;
    /*  + w2 *smooth ; */
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

/* End of code generation (optimize_cpp_mpc.c) */
