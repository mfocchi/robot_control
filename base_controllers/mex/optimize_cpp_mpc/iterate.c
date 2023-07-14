/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
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
#include "computeQ_.h"
#include "compute_deltax.h"
#include "compute_lambda.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void iterate(const emxArray_real_T *H, const emxArray_real_T *f,
             g_struct_T *solution, f_struct_T *memspace, h_struct_T *workingset,
             c_struct_T *qrmanager, d_struct_T *cholmanager,
             e_struct_T *objective, const char_T options_SolverName[7],
             real_T options_StepTolerance, real_T options_ObjectiveLimit,
             int32_T runTimeOptions_MaxIterations)
{
  static const char_T b[7] = {'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T c;
  real_T minLambda;
  real_T s;
  real_T temp;
  int32_T TYPE;
  int32_T activeSetChangeID;
  int32_T b_nVar;
  int32_T globalActiveConstrIdx;
  int32_T i;
  int32_T idx;
  int32_T k;
  int32_T nVar;
  char_T TRANSA;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double,
                                        f, solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }
  b_nVar = workingset->mConstrMax;
  for (k = 0; k < b_nVar; k++) {
    solution->lambda->data[k] = 0.0;
  }
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      int32_T Qk0;
      boolean_T guard1;
      boolean_T guard2;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1: {
          b_nVar = qrmanager->ncols + 1;
          qrmanager->minRowCol =
              muIntScalarMin_sint32(qrmanager->mrows, b_nVar);
          if (qrmanager->mrows >= 1) {
            temp = 1.0;
            minLambda = 0.0;
            TRANSA = 'T';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)qrmanager->mrows;
            lda_t = (ptrdiff_t)qrmanager->ldq;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dgemv(&TRANSA, &m_t, &n_t, &temp, &qrmanager->Q->data[0], &lda_t,
                  &workingset->ATwset->data[workingset->ldA *
                                            (workingset->nActiveConstr - 1)],
                  &incx_t, &minLambda,
                  &qrmanager->QR->data[qrmanager->ldq * qrmanager->ncols],
                  &incy_t);
          }
          qrmanager->ncols = b_nVar;
          i = qrmanager->ncols - 1;
          qrmanager->jpvt->data[i] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            b_nVar = idx + qrmanager->ldq * i;
            temp = qrmanager->QR->data[b_nVar];
            minLambda = qrmanager->QR->data[b_nVar + 1];
            c = 0.0;
            s = 0.0;
            drotg(&temp, &minLambda, &c, &s);
            qrmanager->QR->data[b_nVar] = temp;
            qrmanager->QR->data[b_nVar + 1] = minLambda;
            Qk0 = qrmanager->ldq * idx;
            b_nVar = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              int32_T iy;
              iy = qrmanager->ldq + Qk0;
              for (k = 0; k < b_nVar; k++) {
                int32_T b_temp_tmp;
                int32_T temp_tmp;
                temp_tmp = iy + k;
                b_temp_tmp = Qk0 + k;
                temp = c * qrmanager->Q->data[b_temp_tmp] +
                       s * qrmanager->Q->data[temp_tmp];
                qrmanager->Q->data[temp_tmp] =
                    c * qrmanager->Q->data[temp_tmp] -
                    s * qrmanager->Q->data[b_temp_tmp];
                qrmanager->Q->data[b_temp_tmp] = temp;
              }
            }
          }
        } break;
        case -1:
          deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;
        default:
          factorQR(qrmanager, workingset->ATwset, nVar,
                   workingset->nActiveConstr, workingset->ldA);
          computeQ_(qrmanager, qrmanager->mrows);
          break;
        }
        compute_deltax(
            H, solution, memspace, qrmanager, cholmanager, objective,
            memcmp((char_T *)&options_SolverName[0], (char_T *)&b[0], 7) == 0);
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          if (nVar < 1) {
            temp = 0.0;
          } else {
            n_t = (ptrdiff_t)nVar;
            incx_t = (ptrdiff_t)1;
            temp = dnrm2(&n_t, &solution->searchDir->data[0], &incx_t);
          }
          if ((temp < options_StepTolerance) ||
              (workingset->nActiveConstr >= nVar)) {
            guard2 = true;
          } else {
            minLambda = feasibleratiotest(
                solution->xstar, solution->searchDir, workingset->nVar,
                workingset->lb, workingset->ub, workingset->indexLB,
                workingset->indexUB, workingset->sizes, workingset->isActiveIdx,
                workingset->isActiveConstr, workingset->nWConstr, TYPE == 5,
                &updateFval, &i, &b_nVar);
            if (updateFval) {
              switch (i) {
              case 3:
                workingset->nWConstr[2]++;
                workingset->isActiveConstr
                    ->data[(workingset->isActiveIdx[2] + b_nVar) - 2] = true;
                workingset->nActiveConstr++;
                i = workingset->nActiveConstr - 1;
                workingset->Wid->data[i] = 3;
                workingset->Wlocalidx->data[i] = b_nVar;
                /* A check that is always false is detected at compile-time.
                 * Eliminating code that follows. */
                break;
              case 4:
                addBoundToActiveSetMatrix_(workingset, 4, b_nVar);
                break;
              default:
                addBoundToActiveSetMatrix_(workingset, 5, b_nVar);
                break;
              }
              activeSetChangeID = 1;
            } else {
              if (objective->objtype == 5) {
                if (objective->nvar < 1) {
                  temp = 0.0;
                } else {
                  n_t = (ptrdiff_t)objective->nvar;
                  incx_t = (ptrdiff_t)1;
                  temp = dnrm2(&n_t, &solution->searchDir->data[0], &incx_t);
                }
                if (temp >
                    100.0 * (real_T)objective->nvar * 1.4901161193847656E-8) {
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
            if (nVar >= 1) {
              n_t = (ptrdiff_t)nVar;
              incx_t = (ptrdiff_t)1;
              incy_t = (ptrdiff_t)1;
              daxpy(&n_t, &minLambda, &solution->searchDir->data[0], &incx_t,
                    &solution->xstar->data[0], &incy_t);
            }
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
        if ((solution->state != -7) || (workingset->nActiveConstr > nVar)) {
          Qk0 = 0;
          minLambda = 0.0;
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          b_nVar = workingset->nActiveConstr;
          for (idx = i; idx <= b_nVar; idx++) {
            temp = solution->lambda->data[idx - 1];
            if (temp < minLambda) {
              minLambda = temp;
              Qk0 = idx;
            }
          }
          if (Qk0 == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = Qk0;
            subProblemChanged = true;
            removeConstr(workingset, Qk0);
            solution->lambda->data[Qk0 - 1] = 0.0;
          }
        } else {
          Qk0 = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda->data[Qk0 - 1] = 0.0;
        }
        updateFval = false;
        guard1 = true;
      }
      if (guard1) {
        solution->iterations++;
        b_nVar = objective->nvar;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }
        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr =
              maxConstraintViolation(workingset, solution->xstar);
          temp = solution->maxConstr;
          if (objective->objtype == 5) {
            temp = solution->maxConstr -
                   solution->xstar->data[objective->nvar - 1];
          }
          if (temp > 0.001) {
            boolean_T nonDegenerateWset;
            if (objective->nvar >= 1) {
              n_t = (ptrdiff_t)objective->nvar;
              incx_t = (ptrdiff_t)1;
              incy_t = (ptrdiff_t)1;
              dcopy(&n_t, &solution->xstar->data[0], &incx_t,
                    &solution->searchDir->data[0], &incy_t);
            }
            nonDegenerateWset = feasibleX0ForWorkingSet(
                memspace->workspace_double, solution->searchDir, workingset,
                qrmanager);
            if ((!nonDegenerateWset) && (solution->state != 0)) {
              solution->state = -2;
            }
            activeSetChangeID = 0;
            temp = maxConstraintViolation(workingset, solution->searchDir);
            if (temp < solution->maxConstr) {
              for (idx = 0; idx < b_nVar; idx++) {
                solution->xstar->data[idx] = solution->searchDir->data[idx];
              }
              solution->maxConstr = temp;
            }
          }
        }
        if (updateFval && (options_ObjectiveLimit > rtMinusInf)) {
          solution->fstar = computeFval_ReuseHx(
              objective, memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) &&
              ((solution->state != 0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = computeFval_ReuseHx(
            objective, memspace->workspace_double, f, solution->xstar);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* End of code generation (iterate.c) */
