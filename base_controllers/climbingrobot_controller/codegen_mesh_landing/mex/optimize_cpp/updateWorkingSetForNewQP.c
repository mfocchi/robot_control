/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * updateWorkingSetForNewQP.c
 *
 * Code generation for function 'updateWorkingSetForNewQP'
 *
 */

/* Include files */
#include "updateWorkingSetForNewQP.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void updateWorkingSetForNewQP(const emxArray_real_T *xk, h_struct_T *WorkingSet,
                              int32_T mIneq, int32_T mNonlinIneq,
                              const emxArray_real_T *cIneq, int32_T mLB,
                              const emxArray_real_T *lb, int32_T mUB,
                              const emxArray_real_T *ub, int32_T mFixed)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  const real_T *cIneq_data;
  const real_T *lb_data;
  const real_T *ub_data;
  const real_T *xk_data;
  int32_T idx;
  int32_T ineqStart;
  int32_T vectorUB;
  ub_data = ub->data;
  lb_data = lb->data;
  cIneq_data = cIneq->data;
  xk_data = xk->data;
  ineqStart = (mIneq / 2) << 1;
  vectorUB = ineqStart - 2;
  for (idx = 0; idx <= vectorUB; idx += 2) {
    _mm_storeu_pd(
        &WorkingSet->bineq->data[idx],
        _mm_mul_pd(_mm_loadu_pd(&cIneq_data[idx]), _mm_set1_pd(-1.0)));
  }
  for (idx = ineqStart; idx < mIneq; idx++) {
    WorkingSet->bineq->data[idx] = -cIneq_data[idx];
  }
  for (idx = 0; idx < mLB; idx++) {
    WorkingSet->lb->data[WorkingSet->indexLB->data[idx] - 1] =
        -lb_data[WorkingSet->indexLB->data[idx] - 1] +
        xk_data[WorkingSet->indexLB->data[idx] - 1];
  }
  for (idx = 0; idx < mUB; idx++) {
    WorkingSet->ub->data[WorkingSet->indexUB->data[idx] - 1] =
        ub_data[WorkingSet->indexUB->data[idx] - 1] -
        xk_data[WorkingSet->indexUB->data[idx] - 1];
  }
  for (idx = 0; idx < mFixed; idx++) {
    real_T d;
    d = ub_data[WorkingSet->indexFixed->data[idx] - 1] -
        xk_data[WorkingSet->indexFixed->data[idx] - 1];
    WorkingSet->ub->data[WorkingSet->indexFixed->data[idx] - 1] = d;
    WorkingSet->bwset->data[idx] = d;
  }
  if (WorkingSet->nActiveConstr > mFixed) {
    int32_T i;
    i = mFixed + 1;
    ineqStart = muIntScalarMax_sint32(i, 1);
    vectorUB = WorkingSet->nActiveConstr;
    for (idx = ineqStart; idx <= vectorUB; idx++) {
      switch (WorkingSet->Wid->data[idx - 1]) {
      case 4:
        WorkingSet->bwset->data[idx - 1] =
            WorkingSet->lb
                ->data[WorkingSet->indexLB
                           ->data[WorkingSet->Wlocalidx->data[idx - 1] - 1] -
                       1];
        break;
      case 5:
        WorkingSet->bwset->data[idx - 1] =
            WorkingSet->ub
                ->data[WorkingSet->indexUB
                           ->data[WorkingSet->Wlocalidx->data[idx - 1] - 1] -
                       1];
        break;
      default:
        i = WorkingSet->Wlocalidx->data[idx - 1];
        WorkingSet->bwset->data[idx - 1] = WorkingSet->bineq->data[i - 1];
        if (i >= mNonlinIneq) {
          n_t = (ptrdiff_t)WorkingSet->nVar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t,
                &WorkingSet->Aineq
                     ->data[WorkingSet->ldA *
                            (WorkingSet->Wlocalidx->data[idx - 1] - 1)],
                &incx_t, &WorkingSet->ATwset->data[WorkingSet->ldA * (idx - 1)],
                &incy_t);
        }
        break;
      }
    }
  }
}

/* End of code generation (updateWorkingSetForNewQP.c) */
