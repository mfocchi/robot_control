/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * iterate.c
 *
 * Code generation for function 'iterate'
 *
 */

/* Include files */
#include "iterate.h"
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "compute_lambda.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "squareQ_appendCol.h"
#include "xaxpy.h"
#include "xcopy.h"
#include "xgemm.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "xorgqr.h"
#include "xpotrf.h"
#include "xrot.h"
#include "xscal.h"
#include "blas.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void iterate(const emxArray_real_T *H, const emxArray_real_T *f, d_struct_T
             *solution, c_struct_T *memspace, j_struct_T *workingset, g_struct_T
             *qrmanager, h_struct_T *cholmanager, i_struct_T *objective, real_T
             options_StepTolerance, real_T options_ObjectiveLimit, int32_T
             runTimeOptions_MaxIterations)
{
  real_T c;
  real_T d;
  real_T normDelta;
  real_T s;
  int32_T TYPE;
  int32_T activeSetChangeID;
  int32_T b_i;
  int32_T endIdx;
  int32_T exitg1;
  int32_T globalActiveConstrIdx;
  int32_T i;
  int32_T iQR0;
  int32_T idx;
  int32_T idx_row;
  int32_T k;
  int32_T nVar;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T nonDegenerateWset;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double, f,
    solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  iQR0 = workingset->mConstrMax;
  for (k = 0; k < iQR0; k++) {
    solution->lambda->data[k] = 0.0;
  }

  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          squareQ_appendCol(qrmanager, workingset->ATwset, workingset->ldA *
                            (workingset->nActiveConstr - 1) + 1);
          break;

         case -1:
          i = globalActiveConstrIdx;
          if (qrmanager->usedPivoting) {
            i = 1;
            while ((i <= qrmanager->ncols) && (qrmanager->jpvt->data[i - 1] !=
                    globalActiveConstrIdx)) {
              i++;
            }
          }

          if (i >= qrmanager->ncols) {
            qrmanager->ncols--;
          } else {
            qrmanager->jpvt->data[i - 1] = qrmanager->jpvt->data
              [qrmanager->ncols - 1];
            b_i = qrmanager->minRowCol;
            for (k = 0; k < b_i; k++) {
              qrmanager->QR->data[k + qrmanager->QR->size[0] * (i - 1)] =
                qrmanager->QR->data[k + qrmanager->QR->size[0] *
                (qrmanager->ncols - 1)];
            }

            qrmanager->ncols--;
            qrmanager->minRowCol = muIntScalarMin_sint32(qrmanager->mrows,
              qrmanager->ncols);
            if (i < qrmanager->mrows) {
              iQR0 = qrmanager->mrows - 1;
              endIdx = muIntScalarMin_sint32(iQR0, qrmanager->ncols);
              for (k = endIdx; k >= i; k--) {
                d = qrmanager->QR->data[(k + qrmanager->QR->size[0] * (i - 1)) -
                  1];
                normDelta = qrmanager->QR->data[k + qrmanager->QR->size[0] * (i
                  - 1)];
                c = 0.0;
                s = 0.0;
                drotg(&d, &normDelta, &c, &s);
                qrmanager->QR->data[(k + qrmanager->QR->size[0] * (i - 1)) - 1] =
                  d;
                qrmanager->QR->data[k + qrmanager->QR->size[0] * (i - 1)] =
                  normDelta;
                qrmanager->QR->data[k + qrmanager->QR->size[0] * (k - 1)] = 0.0;
                iQR0 = k + qrmanager->ldq * i;
                b_xrot(qrmanager->ncols - i, qrmanager->QR, iQR0, qrmanager->ldq,
                       iQR0 + 1, qrmanager->ldq, c, s);
                iQR0 = qrmanager->ldq * (k - 1) + 1;
                xrot(qrmanager->mrows, qrmanager->Q, iQR0, qrmanager->ldq + iQR0,
                     c, s);
              }

              b_i = i + 1;
              for (k = b_i; k <= endIdx; k++) {
                d = qrmanager->QR->data[(k + qrmanager->QR->size[0] * (k - 1)) -
                  1];
                normDelta = qrmanager->QR->data[k + qrmanager->QR->size[0] * (k
                  - 1)];
                c = 0.0;
                s = 0.0;
                drotg(&d, &normDelta, &c, &s);
                qrmanager->QR->data[(k + qrmanager->QR->size[0] * (k - 1)) - 1] =
                  d;
                qrmanager->QR->data[k + qrmanager->QR->size[0] * (k - 1)] =
                  normDelta;
                iQR0 = k * (qrmanager->ldq + 1);
                b_xrot(qrmanager->ncols - k, qrmanager->QR, iQR0, qrmanager->ldq,
                       iQR0 + 1, qrmanager->ldq, c, s);
                iQR0 = qrmanager->ldq * (k - 1) + 1;
                xrot(qrmanager->mrows, qrmanager->Q, iQR0, qrmanager->ldq + iQR0,
                     c, s);
              }
            }
          }
          break;

         default:
          factorQR(qrmanager, workingset->ATwset, nVar,
                   workingset->nActiveConstr);
          b_i = qrmanager->minRowCol;
          for (idx = 0; idx < b_i; idx++) {
            iQR0 = (qrmanager->ldq * idx + idx) + 2;
            b_xcopy((qrmanager->mrows - idx) - 1, qrmanager->QR, iQR0,
                    qrmanager->Q, iQR0);
          }

          xorgqr(qrmanager->mrows, qrmanager->mrows, qrmanager->minRowCol,
                 qrmanager->Q, qrmanager->ldq, qrmanager->tau);
          break;
        }

        iQR0 = qrmanager->mrows - 1;
        i = qrmanager->mrows - qrmanager->ncols;
        if (i <= 0) {
          for (idx = 0; idx <= iQR0; idx++) {
            solution->searchDir->data[idx] = 0.0;
          }
        } else {
          for (idx = 0; idx <= iQR0; idx++) {
            solution->searchDir->data[idx] = -objective->grad->data[idx];
          }

          if (qrmanager->ncols <= 0) {
            switch (objective->objtype) {
             case 5:
              break;

             case 3:
              cholmanager->ndims = qrmanager->mrows;
              if ((H->size[0] != 0) && (H->size[1] != 0)) {
                for (idx = 0; idx <= iQR0; idx++) {
                  b_xcopy(iQR0 + 1, H, (iQR0 + 1) * idx + 1, cholmanager->FMat,
                          cholmanager->ldm * idx + 1);
                }
              }

              cholmanager->info = xpotrf(qrmanager->mrows, cholmanager->FMat,
                cholmanager->ldm);
              if (cholmanager->info != 0) {
                solution->state = -6;
              } else {
                solve(cholmanager, solution->searchDir);
              }
              break;

             default:
              iQR0 = objective->nvar;
              cholmanager->ndims = objective->nvar;
              if ((H->size[0] != 0) && (H->size[1] != 0)) {
                for (idx = 0; idx < iQR0; idx++) {
                  b_xcopy(iQR0, H, iQR0 * idx + 1, cholmanager->FMat,
                          cholmanager->ldm * idx + 1);
                }
              }

              cholmanager->info = xpotrf(objective->nvar, cholmanager->FMat,
                cholmanager->ldm);
              if (cholmanager->info != 0) {
                solution->state = -6;
              } else {
                solve(cholmanager, solution->searchDir);
                xscal(qrmanager->mrows - objective->nvar, 1.0 / objective->beta,
                      solution->searchDir, objective->nvar + 1);
              }
              break;
            }
          } else {
            k = qrmanager->ldq * qrmanager->ncols + 1;
            switch (objective->objtype) {
             case 5:
              for (idx = 0; idx < i; idx++) {
                memspace->workspace_double->data[idx] = -qrmanager->Q->data[iQR0
                  + qrmanager->Q->size[0] * (qrmanager->ncols + idx)];
              }

              e_xgemv(qrmanager->mrows, i, qrmanager->Q, k, qrmanager->ldq,
                      memspace->workspace_double, solution->searchDir);
              break;

             default:
              switch (objective->objtype) {
               case 3:
                endIdx = memspace->workspace_double->size[0];
                c_xgemm(qrmanager->mrows, i, qrmanager->mrows, H,
                        qrmanager->mrows, qrmanager->Q, k, qrmanager->ldq,
                        memspace->workspace_double, memspace->
                        workspace_double->size[0]);
                d_xgemm(i, i, qrmanager->mrows, qrmanager->Q, k, qrmanager->ldq,
                        memspace->workspace_double, endIdx, cholmanager->FMat,
                        cholmanager->ldm);
                break;

               default:
                iQR0 = qrmanager->mrows;
                endIdx = memspace->workspace_double->size[0];
                c_xgemm(objective->nvar, i, objective->nvar, H, objective->nvar,
                        qrmanager->Q, k, qrmanager->ldq,
                        memspace->workspace_double, memspace->
                        workspace_double->size[0]);
                for (idx = 0; idx < i; idx++) {
                  b_i = objective->nvar + 1;
                  for (idx_row = b_i; idx_row <= iQR0; idx_row++) {
                    memspace->workspace_double->data[(idx_row +
                      memspace->workspace_double->size[0] * idx) - 1] =
                      objective->beta * qrmanager->Q->data[(idx_row +
                      qrmanager->Q->size[0] * (idx + qrmanager->ncols)) - 1];
                  }
                }

                d_xgemm(i, i, qrmanager->mrows, qrmanager->Q, k, qrmanager->ldq,
                        memspace->workspace_double, endIdx, cholmanager->FMat,
                        cholmanager->ldm);
                break;
              }

              cholmanager->ndims = i;
              cholmanager->info = xpotrf(i, cholmanager->FMat, cholmanager->ldm);
              if (cholmanager->info != 0) {
                solution->state = -6;
              } else {
                f_xgemv(qrmanager->mrows, i, qrmanager->Q, k, qrmanager->ldq,
                        objective->grad, memspace->workspace_double);
                b_solve(cholmanager, memspace->workspace_double);
                e_xgemv(qrmanager->mrows, i, qrmanager->Q, k, qrmanager->ldq,
                        memspace->workspace_double, solution->searchDir);
              }
              break;
            }
          }
        }

        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = xnrm2(nVar, solution->searchDir);
          if ((normDelta < options_StepTolerance) || (workingset->nActiveConstr >=
               nVar)) {
            guard2 = true;
          } else {
            feasibleratiotest(solution->xstar, solution->searchDir,
                              workingset->nVar, workingset->lb, workingset->ub,
                              workingset->indexLB, workingset->indexUB,
                              workingset->sizes, workingset->isActiveIdx,
                              workingset->isActiveConstr, workingset->nWConstr,
                              TYPE == 5, &normDelta, &updateFval, &b_i, &iQR0);
            if (updateFval) {
              switch (b_i) {
               case 3:
                workingset->nWConstr[2]++;
                workingset->isActiveConstr->data[(workingset->isActiveIdx[2] +
                  iQR0) - 2] = true;
                workingset->nActiveConstr++;
                workingset->Wid->data[workingset->nActiveConstr - 1] = 3;
                workingset->Wlocalidx->data[workingset->nActiveConstr - 1] =
                  iQR0;

                /* A check that is always false is detected at compile-time. Eliminating code that follows. */
                break;

               case 4:
                addBoundToActiveSetMatrix_(workingset, 4, iQR0);
                break;

               default:
                addBoundToActiveSetMatrix_(workingset, 5, iQR0);
                break;
              }

              activeSetChangeID = 1;
            } else {
              if (objective->objtype == 5) {
                if (xnrm2(objective->nvar, solution->searchDir) > 100.0 *
                    (real_T)objective->nvar * 1.4901161193847656E-8) {
                  solution->state = 3;
                } else {
                  solution->state = 4;
                }
              }

              subProblemChanged = false;
              if (workingset->nActiveConstr == 0) {
                solution->state = 1;
              }
            }

            xaxpy(nVar, normDelta, solution->searchDir, solution->xstar);
            computeGrad_StoreHx(objective, H, f, solution->xstar);
            updateFval = true;
            guard1 = true;
          }
        }
      } else {
        for (k = 0; k < nVar; k++) {
          solution->searchDir->data[k] = 0.0;
        }

        guard2 = true;
      }

      if (guard2) {
        compute_lambda(memspace->workspace_double, solution, objective,
                       qrmanager);
        if (solution->state != -7) {
          iQR0 = 0;
          normDelta = 0.0;
          b_i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          endIdx = workingset->nActiveConstr;
          for (idx = b_i; idx <= endIdx; idx++) {
            d = solution->lambda->data[idx - 1];
            if (d < normDelta) {
              normDelta = d;
              iQR0 = idx;
            }
          }

          if (iQR0 == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iQR0;
            subProblemChanged = true;
            removeConstr(workingset, iQR0);
            solution->lambda->data[iQR0 - 1] = 0.0;
          }
        } else {
          iQR0 = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda->data[iQR0 - 1] = 0.0;
        }

        updateFval = false;
        guard1 = true;
      }

      if (guard1) {
        solution->iterations++;
        iQR0 = objective->nvar;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = b_maxConstraintViolation(workingset,
            solution->xstar);
          if (solution->maxConstr > 0.001) {
            xcopy(objective->nvar, solution->xstar, solution->searchDir);
            nonDegenerateWset = feasibleX0ForWorkingSet
              (memspace->workspace_double, solution->searchDir, workingset,
               qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }

            activeSetChangeID = 0;
            normDelta = b_maxConstraintViolation(workingset, solution->searchDir);
            if (normDelta < solution->maxConstr) {
              for (idx = 0; idx < iQR0; idx++) {
                solution->xstar->data[idx] = solution->searchDir->data[idx];
              }

              solution->maxConstr = normDelta;
            }
          }
        }

        if ((options_ObjectiveLimit > rtMinusInf) && updateFval) {
          solution->fstar = computeFval_ReuseHx(objective,
            memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) && ((solution->state !=
                0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = computeFval_ReuseHx(objective,
          memspace->workspace_double, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* End of code generation (iterate.c) */
