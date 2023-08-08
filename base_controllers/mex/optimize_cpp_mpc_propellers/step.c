/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * step.c
 *
 * Code generation for function 'step'
 *
 */

/* Include files */
#include "step.h"
#include "normal.h"
#include "optimize_cpp_mpc_propellers_data.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "soc.h"
#include "xcopy.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
boolean_T step(int32_T *STEP_TYPE, emxArray_real_T *Hessian, const
               emxArray_real_T *lb, const emxArray_real_T *ub, d_struct_T
               *TrialState, k_struct_T *MeritFunction, c_struct_T *memspace,
               j_struct_T *WorkingSet, g_struct_T *QRManager, h_struct_T
               *CholManager, i_struct_T *QPObjective, b_struct_T *qpoptions)
{
  emxArray_real_T *r;
  real_T nrmDirInf;
  real_T nrmGradInf;
  int32_T exitg1;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idx_col;
  int32_T k;
  int32_T mUB;
  int32_T nVar;
  boolean_T checkBoundViolation;
  boolean_T guard1 = false;
  boolean_T stepSuccess;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  stepSuccess = true;
  checkBoundViolation = true;
  nVar = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    xcopy(WorkingSet->nVar, TrialState->xstarsqp, TrialState->xstar);
  } else {
    xcopy(WorkingSet->nVar, TrialState->xstar, TrialState->searchDir);
  }

  emxInit_real_T(&r, 1, true);
  do {
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
     case 1:
      normal(Hessian, TrialState->grad, TrialState, MeritFunction, memspace,
             WorkingSet, QRManager, CholManager, QPObjective, qpoptions);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        xcopy(nVar, TrialState->xstar, TrialState->delta_x);
        guard1 = true;
      }
      break;

     case 2:
      idxStartIneq = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
      idxEndIneq = WorkingSet->nActiveConstr;
      for (mUB = idxStartIneq; mUB <= idxEndIneq; mUB++) {
        WorkingSet->isActiveConstr->data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid->data[mUB - 1] - 1] + WorkingSet->Wlocalidx->data[mUB
          - 1]) - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = WorkingSet->nWConstr[0] + WorkingSet->
        nWConstr[1];
      idx_col = r->size[0];
      r->size[0] = TrialState->xstar->size[0];
      emxEnsureCapacity_real_T(r, idx_col);
      idxStartIneq = TrialState->xstar->size[0];
      for (idx_col = 0; idx_col < idxStartIneq; idx_col++) {
        r->data[idx_col] = TrialState->xstar->data[idx_col];
      }

      idxEndIneq = WorkingSet->sizes[3] - 1;
      mUB = WorkingSet->sizes[4] - 1;
      if (lb->size[1] != 0) {
        if (ub->size[1] == 0) {
          for (idxStartIneq = 0; idxStartIneq <= idxEndIneq; idxStartIneq++) {
            nrmGradInf = WorkingSet->lb->data[WorkingSet->indexLB->
              data[idxStartIneq] - 1];
            if (-r->data[WorkingSet->indexLB->data[idxStartIneq] - 1] >
                nrmGradInf) {
              r->data[WorkingSet->indexLB->data[idxStartIneq] - 1] = -nrmGradInf
                + muDoubleScalarAbs(nrmGradInf);
            }
          }
        } else {
          for (idxStartIneq = 0; idxStartIneq <= idxEndIneq; idxStartIneq++) {
            nrmGradInf = WorkingSet->lb->data[WorkingSet->indexLB->
              data[idxStartIneq] - 1];
            if (-r->data[WorkingSet->indexLB->data[idxStartIneq] - 1] >
                nrmGradInf) {
              if (muDoubleScalarIsInf(ub->data[WorkingSet->indexLB->
                                      data[idxStartIneq] - 1])) {
                r->data[WorkingSet->indexLB->data[idxStartIneq] - 1] =
                  -nrmGradInf + muDoubleScalarAbs(nrmGradInf);
              } else {
                r->data[WorkingSet->indexLB->data[idxStartIneq] - 1] =
                  (WorkingSet->ub->data[WorkingSet->indexLB->data[idxStartIneq]
                   - 1] - nrmGradInf) / 2.0;
              }
            }
          }
        }
      }

      if (ub->size[1] != 0) {
        if (lb->size[1] == 0) {
          for (idxStartIneq = 0; idxStartIneq <= mUB; idxStartIneq++) {
            nrmGradInf = WorkingSet->ub->data[WorkingSet->indexUB->
              data[idxStartIneq] - 1];
            if (r->data[WorkingSet->indexUB->data[idxStartIneq] - 1] >
                nrmGradInf) {
              r->data[WorkingSet->indexUB->data[idxStartIneq] - 1] = nrmGradInf
                - muDoubleScalarAbs(nrmGradInf);
            }
          }
        } else {
          for (idxStartIneq = 0; idxStartIneq <= mUB; idxStartIneq++) {
            nrmGradInf = WorkingSet->ub->data[WorkingSet->indexUB->
              data[idxStartIneq] - 1];
            if (r->data[WorkingSet->indexUB->data[idxStartIneq] - 1] >
                nrmGradInf) {
              if (muDoubleScalarIsInf(lb->data[WorkingSet->indexUB->
                                      data[idxStartIneq] - 1])) {
                r->data[WorkingSet->indexUB->data[idxStartIneq] - 1] =
                  nrmGradInf - muDoubleScalarAbs(nrmGradInf);
              } else {
                r->data[WorkingSet->indexUB->data[idxStartIneq] - 1] =
                  (nrmGradInf - WorkingSet->lb->data[WorkingSet->indexUB->
                   data[idxStartIneq] - 1]) / 2.0;
              }
            }
          }
        }
      }

      idx_col = TrialState->xstar->size[0];
      TrialState->xstar->size[0] = r->size[0];
      emxEnsureCapacity_real_T(TrialState->xstar, idx_col);
      idxStartIneq = r->size[0];
      for (idx_col = 0; idx_col < idxStartIneq; idx_col++) {
        TrialState->xstar->data[idx_col] = r->data[idx_col];
      }

      relaxed(Hessian, TrialState->grad, TrialState, MeritFunction, memspace,
              WorkingSet, QRManager, CholManager, QPObjective, qpoptions);
      xcopy(nVar, TrialState->xstar, TrialState->delta_x);
      guard1 = true;
      break;

     default:
      idx_col = r->size[0];
      r->size[0] = TrialState->grad->size[0];
      emxEnsureCapacity_real_T(r, idx_col);
      idxStartIneq = TrialState->grad->size[0];
      for (idx_col = 0; idx_col < idxStartIneq; idx_col++) {
        r->data[idx_col] = TrialState->grad->data[idx_col];
      }

      stepSuccess = soc(Hessian, r, TrialState, memspace, WorkingSet, QRManager,
                        CholManager, QPObjective, qpoptions);
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        for (idxStartIneq = 0; idxStartIneq < nVar; idxStartIneq++) {
          TrialState->delta_x->data[idxStartIneq] = TrialState->xstar->
            data[idxStartIneq] + TrialState->socDirection->data[idxStartIneq];
        }
      }

      guard1 = true;
      break;
    }

    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        mUB = Hessian->size[0] - 1;
        nrmGradInf = 0.0;
        nrmDirInf = 1.0;
        for (idxStartIneq = 0; idxStartIneq <= mUB; idxStartIneq++) {
          nrmGradInf = muDoubleScalarMax(nrmGradInf, muDoubleScalarAbs
            (TrialState->grad->data[idxStartIneq]));
          nrmDirInf = muDoubleScalarMax(nrmDirInf, muDoubleScalarAbs
            (TrialState->xstar->data[idxStartIneq]));
        }

        nrmGradInf = muDoubleScalarMax(2.2204460492503131E-16, nrmGradInf /
          nrmDirInf);
        for (idx_col = 0; idx_col <= mUB; idx_col++) {
          idxStartIneq = (mUB + 1) * idx_col;
          for (k = 0; k < idx_col; k++) {
            Hessian->data[idxStartIneq + k] = 0.0;
          }

          Hessian->data[idx_col + Hessian->size[0] * idx_col] = nrmGradInf;
          idxStartIneq += idx_col;
          idxEndIneq = mUB - idx_col;
          for (k = 0; k < idxEndIneq; k++) {
            Hessian->data[(idxStartIneq + k) + 1] = 0.0;
          }
        }
      }
    }
  } while (exitg1 == 0);

  if (checkBoundViolation) {
    idxEndIneq = WorkingSet->sizes[3];
    mUB = WorkingSet->sizes[4];
    idx_col = r->size[0];
    r->size[0] = TrialState->delta_x->size[0];
    emxEnsureCapacity_real_T(r, idx_col);
    idxStartIneq = TrialState->delta_x->size[0];
    for (idx_col = 0; idx_col < idxStartIneq; idx_col++) {
      r->data[idx_col] = TrialState->delta_x->data[idx_col];
    }

    if (lb->size[1] != 0) {
      for (idxStartIneq = 0; idxStartIneq < idxEndIneq; idxStartIneq++) {
        nrmGradInf = r->data[WorkingSet->indexLB->data[idxStartIneq] - 1];
        nrmDirInf = (TrialState->xstarsqp->data[WorkingSet->indexLB->
                     data[idxStartIneq] - 1] + nrmGradInf) - lb->data
          [WorkingSet->indexLB->data[idxStartIneq] - 1];
        if (nrmDirInf < 0.0) {
          r->data[WorkingSet->indexLB->data[idxStartIneq] - 1] = nrmGradInf -
            nrmDirInf;
          TrialState->xstar->data[WorkingSet->indexLB->data[idxStartIneq] - 1] -=
            nrmDirInf;
        }
      }
    }

    if (ub->size[1] != 0) {
      for (idxStartIneq = 0; idxStartIneq < mUB; idxStartIneq++) {
        nrmGradInf = r->data[WorkingSet->indexUB->data[idxStartIneq] - 1];
        nrmDirInf = (ub->data[WorkingSet->indexUB->data[idxStartIneq] - 1] -
                     TrialState->xstarsqp->data[WorkingSet->indexUB->
                     data[idxStartIneq] - 1]) - nrmGradInf;
        if (nrmDirInf < 0.0) {
          r->data[WorkingSet->indexUB->data[idxStartIneq] - 1] = nrmGradInf +
            nrmDirInf;
          TrialState->xstar->data[WorkingSet->indexUB->data[idxStartIneq] - 1] +=
            nrmDirInf;
        }
      }
    }

    idx_col = TrialState->delta_x->size[0];
    TrialState->delta_x->size[0] = r->size[0];
    emxEnsureCapacity_real_T(TrialState->delta_x, idx_col);
    idxStartIneq = r->size[0];
    for (idx_col = 0; idx_col < idxStartIneq; idx_col++) {
      TrialState->delta_x->data[idx_col] = r->data[idx_col];
    }
  }

  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return stepSuccess;
}

/* End of code generation (step.c) */
