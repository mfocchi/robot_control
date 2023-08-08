/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * PresolveWorkingSet.c
 *
 * Code generation for function 'PresolveWorkingSet'
 *
 */

/* Include files */
#include "PresolveWorkingSet.h"
#include "ComputeNumDependentEq_.h"
#include "IndexOfDependentEq_.h"
#include "countsort.h"
#include "factorQRE.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void PresolveWorkingSet(d_struct_T *solution, c_struct_T *memspace, j_struct_T
  *workingset, g_struct_T *qrmanager)
{
  real_T tol;
  int32_T b_idx;
  int32_T i;
  int32_T idx;
  int32_T idxEndIneq;
  int32_T idx_col;
  int32_T idx_row;
  int32_T nVar;
  boolean_T guard1 = false;
  boolean_T okWorkingSet;
  solution->state = 82;
  nVar = workingset->nVar;
  idxEndIneq = workingset->nWConstr[1] + workingset->nWConstr[0];
  idx_col = 0;
  if (idxEndIneq > 0) {
    for (idx_row = 0; idx_row < idxEndIneq; idx_row++) {
      for (idx_col = 0; idx_col < nVar; idx_col++) {
        qrmanager->QR->data[idx_row + qrmanager->QR->size[0] * idx_col] =
          workingset->ATwset->data[idx_col + workingset->ATwset->size[0] *
          idx_row];
      }
    }

    idx_col = ComputeNumDependentEq_(qrmanager, workingset->bwset, idxEndIneq,
      workingset->nVar);
    if (idx_col > 0) {
      IndexOfDependentEq_(memspace->workspace_int, workingset->nWConstr[0],
                          idx_col, qrmanager, workingset->ATwset,
                          workingset->nVar, idxEndIneq);
      countsort(memspace->workspace_int, idx_col, memspace->workspace_sort, 1,
                idxEndIneq);
      for (idx = idx_col; idx >= 1; idx--) {
        i = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (i != 0) {
          idx_row = memspace->workspace_int->data[idx - 1];
          if (idx_row <= i) {
            if ((workingset->nActiveConstr == i) || (idx_row == i)) {
              workingset->mEqRemoved++;
              removeConstr(workingset, memspace->workspace_int->data[idx - 1]);
            } else {
              workingset->mEqRemoved++;
              nVar = workingset->Wid->data[idx_row - 1] - 1;
              idxEndIneq = workingset->Wlocalidx->data[idx_row - 1];
              workingset->isActiveConstr->data[(workingset->isActiveIdx[nVar] +
                idxEndIneq) - 2] = false;
              workingset->Wid->data[idx_row - 1] = workingset->Wid->data[i - 1];
              workingset->Wlocalidx->data[idx_row - 1] = workingset->
                Wlocalidx->data[i - 1];
              idxEndIneq = workingset->nVar;
              for (b_idx = 0; b_idx < idxEndIneq; b_idx++) {
                workingset->ATwset->data[b_idx + workingset->ATwset->size[0] *
                  (idx_row - 1)] = workingset->ATwset->data[b_idx +
                  workingset->ATwset->size[0] * (i - 1)];
              }

              workingset->bwset->data[idx_row - 1] = workingset->bwset->data[i -
                1];
              workingset->Wid->data[i - 1] = workingset->Wid->data
                [workingset->nActiveConstr - 1];
              workingset->Wlocalidx->data[i - 1] = workingset->Wlocalidx->
                data[workingset->nActiveConstr - 1];
              idx_row = workingset->nVar;
              for (b_idx = 0; b_idx < idx_row; b_idx++) {
                workingset->ATwset->data[b_idx + workingset->ATwset->size[0] *
                  (i - 1)] = workingset->ATwset->data[b_idx + workingset->
                  ATwset->size[0] * (workingset->nActiveConstr - 1)];
              }

              workingset->bwset->data[i - 1] = workingset->bwset->
                data[workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[nVar]--;
            }
          }
        }
      }
    }
  }

  if (idx_col != -1) {
    nVar = workingset->nActiveConstr;
    i = workingset->nWConstr[1] + workingset->nWConstr[0];
    if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
        workingset->nWConstr[4] > 0) {
      tol = 100.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
      for (idx = 0; idx < i; idx++) {
        qrmanager->jpvt->data[idx] = 1;
      }

      idx_row = i + 1;
      for (idx = idx_row; idx <= nVar; idx++) {
        qrmanager->jpvt->data[idx - 1] = 0;
      }

      factorQRE(qrmanager, workingset->ATwset, workingset->nVar,
                workingset->nActiveConstr);
      nVar = 0;
      for (idx = workingset->nActiveConstr; idx > workingset->nVar; idx--) {
        nVar++;
        memspace->workspace_int->data[nVar - 1] = qrmanager->jpvt->data[idx - 1];
      }

      if (idx <= workingset->nVar) {
        while ((idx > i) && (muDoubleScalarAbs(qrmanager->QR->data[(idx +
                  qrmanager->QR->size[0] * (idx - 1)) - 1]) < tol)) {
          nVar++;
          memspace->workspace_int->data[nVar - 1] = qrmanager->jpvt->data[idx -
            1];
          idx--;
        }
      }

      countsort(memspace->workspace_int, nVar, memspace->workspace_sort, i + 1,
                workingset->nActiveConstr);
      for (idx = nVar; idx >= 1; idx--) {
        removeConstr(workingset, memspace->workspace_int->data[idx - 1]);
      }
    }

    okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      nVar = workingset->nActiveConstr;
      i = workingset->nWConstr[1] + workingset->nWConstr[0];
      if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
          workingset->nWConstr[4] > 0) {
        tol = 1000.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
        for (idx = 0; idx < i; idx++) {
          qrmanager->jpvt->data[idx] = 1;
        }

        idx_row = i + 1;
        for (idx = idx_row; idx <= nVar; idx++) {
          qrmanager->jpvt->data[idx - 1] = 0;
        }

        factorQRE(qrmanager, workingset->ATwset, workingset->nVar,
                  workingset->nActiveConstr);
        nVar = 0;
        for (idx = workingset->nActiveConstr; idx > workingset->nVar; idx--) {
          nVar++;
          memspace->workspace_int->data[nVar - 1] = qrmanager->jpvt->data[idx -
            1];
        }

        if (idx <= workingset->nVar) {
          while ((idx > i) && (muDoubleScalarAbs(qrmanager->QR->data[(idx +
                    qrmanager->QR->size[0] * (idx - 1)) - 1]) < tol)) {
            nVar++;
            memspace->workspace_int->data[nVar - 1] = qrmanager->jpvt->data[idx
              - 1];
            idx--;
          }
        }

        countsort(memspace->workspace_int, nVar, memspace->workspace_sort, i + 1,
                  workingset->nActiveConstr);
        for (idx = nVar; idx >= 1; idx--) {
          removeConstr(workingset, memspace->workspace_int->data[idx - 1]);
        }
      }

      okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
        solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = maxConstraintViolation(workingset, solution->xstar);
      if (tol > 0.001) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    nVar = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (idx_row = nVar; idx_row <= idxEndIneq; idx_row++) {
      workingset->isActiveConstr->data[(workingset->isActiveIdx[workingset->
        Wid->data[idx_row - 1] - 1] + workingset->Wlocalidx->data[idx_row - 1])
        - 2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

/* End of code generation (PresolveWorkingSet.c) */
