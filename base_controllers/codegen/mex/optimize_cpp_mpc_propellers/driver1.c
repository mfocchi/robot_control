/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * driver1.c
 *
 * Code generation for function 'driver1'
 *
 */

/* Include files */
#include "driver1.h"
#include "PresolveWorkingSet.h"
#include "computeFval.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "xcopy.h"
#include <string.h>

/* Function Definitions */
void b_driver(const emxArray_real_T *H, const emxArray_real_T *f, d_struct_T
              *solution, c_struct_T *memspace, j_struct_T *workingset,
              g_struct_T *qrmanager, h_struct_T *cholmanager, i_struct_T
              *objective, b_struct_T *options, int32_T
              runTimeOptions_MaxIterations)
{
  real_T maxConstr_new;
  int32_T PHASEONE;
  int32_T PROBTYPE_ORIG;
  int32_T b_nVar;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idx_global;
  int32_T nVar;
  int32_T nVarP1;
  boolean_T exitg1;
  boolean_T guard1 = false;
  solution->iterations = 0;
  nVar = workingset->nVar;
  guard1 = false;
  if (workingset->probType == 3) {
    PHASEONE = workingset->sizes[0];
    for (idxStartIneq = 0; idxStartIneq < PHASEONE; idxStartIneq++) {
      solution->xstar->data[workingset->indexFixed->data[idxStartIneq] - 1] =
        workingset->ub->data[workingset->indexFixed->data[idxStartIneq] - 1];
    }

    PHASEONE = workingset->sizes[3];
    for (idxStartIneq = 0; idxStartIneq < PHASEONE; idxStartIneq++) {
      if (workingset->isActiveConstr->data[(workingset->isActiveIdx[3] +
           idxStartIneq) - 1]) {
        solution->xstar->data[workingset->indexLB->data[idxStartIneq] - 1] =
          -workingset->lb->data[workingset->indexLB->data[idxStartIneq] - 1];
      }
    }

    PHASEONE = workingset->sizes[4];
    for (idxStartIneq = 0; idxStartIneq < PHASEONE; idxStartIneq++) {
      if (workingset->isActiveConstr->data[(workingset->isActiveIdx[4] +
           idxStartIneq) - 1]) {
        solution->xstar->data[workingset->indexUB->data[idxStartIneq] - 1] =
          workingset->ub->data[workingset->indexUB->data[idxStartIneq] - 1];
      }
    }

    PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution->state >= 0) {
      guard1 = true;
    }
  } else {
    solution->state = 82;
    guard1 = true;
  }

  if (guard1) {
    solution->iterations = 0;
    solution->maxConstr = maxConstraintViolation(workingset, solution->xstar);
    if (solution->maxConstr > 0.001) {
      PROBTYPE_ORIG = workingset->probType;
      b_nVar = workingset->nVar;
      nVarP1 = workingset->nVar;
      solution->xstar->data[workingset->nVar] = solution->maxConstr + 1.0;
      if (workingset->probType == 3) {
        PHASEONE = 1;
      } else {
        PHASEONE = 4;
      }

      idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
      idxEndIneq = workingset->nActiveConstr;
      for (idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
        workingset->isActiveConstr->data[(workingset->isActiveIdx
          [workingset->Wid->data[idx_global - 1] - 1] + workingset->
          Wlocalidx->data[idx_global - 1]) - 2] = false;
      }

      workingset->nWConstr[2] = 0;
      workingset->nWConstr[3] = 0;
      workingset->nWConstr[4] = 0;
      workingset->nActiveConstr = workingset->nWConstr[0] + workingset->
        nWConstr[1];
      setProblemType(workingset, PHASEONE);
      objective->prev_objtype = objective->objtype;
      objective->prev_nvar = objective->nvar;
      objective->prev_hasLinear = objective->hasLinear;
      objective->objtype = 5;
      objective->nvar = nVarP1 + 1;
      objective->gammaScalar = 1.0;
      objective->hasLinear = true;
      solution->fstar = computeFval(objective, memspace->workspace_double, H, f,
        solution->xstar);
      solution->state = 5;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, 1.4901161193847657E-10, 0.001,
              runTimeOptions_MaxIterations);
      if (workingset->isActiveConstr->data[(workingset->isActiveIdx[3] +
           workingset->sizes[3]) - 2]) {
        idxStartIneq = workingset->sizes[0];
        exitg1 = false;
        while ((!exitg1) && (idxStartIneq + 1 <= workingset->nActiveConstr)) {
          if ((workingset->Wid->data[idxStartIneq] == 4) &&
              (workingset->Wlocalidx->data[idxStartIneq] == workingset->sizes[3]))
          {
            removeConstr(workingset, idxStartIneq + 1);
            exitg1 = true;
          } else {
            idxStartIneq++;
          }
        }
      }

      PHASEONE = workingset->nActiveConstr;
      idxStartIneq = workingset->sizes[0];
      while ((PHASEONE > idxStartIneq) && (PHASEONE > b_nVar)) {
        removeConstr(workingset, PHASEONE);
        PHASEONE--;
      }

      solution->maxConstr = solution->xstar->data[nVarP1];
      setProblemType(workingset, PROBTYPE_ORIG);
      objective->objtype = objective->prev_objtype;
      objective->nvar = objective->prev_nvar;
      objective->hasLinear = objective->prev_hasLinear;
      options->ObjectiveLimit = rtMinusInf;
      options->StepTolerance = 1.0E-6;
      if (solution->state != 0) {
        solution->maxConstr = maxConstraintViolation(workingset, solution->xstar);
        if (solution->maxConstr > 0.001) {
          PHASEONE = workingset->mConstrMax;
          for (idxStartIneq = 0; idxStartIneq < PHASEONE; idxStartIneq++) {
            solution->lambda->data[idxStartIneq] = 0.0;
          }

          solution->fstar = computeFval(objective, memspace->workspace_double, H,
            f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            xcopy(nVar, solution->xstar, solution->searchDir);
            PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = maxConstraintViolation(workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              xcopy(nVar, solution->searchDir, solution->xstar);
            }
          }

          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options->StepTolerance, options->ObjectiveLimit,
                  runTimeOptions_MaxIterations);
        }
      }
    } else {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options->StepTolerance, options->ObjectiveLimit,
              runTimeOptions_MaxIterations);
    }
  }
}

/* End of code generation (driver1.c) */
