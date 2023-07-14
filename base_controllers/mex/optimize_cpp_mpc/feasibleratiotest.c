/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * feasibleratiotest.c
 *
 * Code generation for function 'feasibleratiotest'
 *
 */

/* Include files */
#include "feasibleratiotest.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T feasibleratiotest(
    const emxArray_real_T *solution_xstar,
    const emxArray_real_T *solution_searchDir, int32_T workingset_nVar,
    const emxArray_real_T *workingset_lb, const emxArray_real_T *workingset_ub,
    const emxArray_int32_T *workingset_indexLB,
    const emxArray_int32_T *workingset_indexUB,
    const int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6],
    const emxArray_boolean_T *workingset_isActiveConstr,
    const int32_T workingset_nWConstr[5], boolean_T isPhaseOne,
    boolean_T *newBlocking, int32_T *constrType, int32_T *constrIdx)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  const real_T *solution_searchDir_data;
  const real_T *solution_xstar_data;
  const real_T *workingset_lb_data;
  const real_T *workingset_ub_data;
  real_T alpha;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T phaseOneCorrectionX;
  real_T pk_corrected;
  real_T ratio;
  const int32_T *workingset_indexLB_data;
  const int32_T *workingset_indexUB_data;
  int32_T idx;
  int32_T totalUB;
  const boolean_T *workingset_isActiveConstr_data;
  workingset_isActiveConstr_data = workingset_isActiveConstr->data;
  workingset_indexUB_data = workingset_indexUB->data;
  workingset_indexLB_data = workingset_indexLB->data;
  workingset_ub_data = workingset_ub->data;
  workingset_lb_data = workingset_lb->data;
  solution_searchDir_data = solution_searchDir->data;
  solution_xstar_data = solution_xstar->data;
  totalUB = workingset_sizes[4];
  alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  if (workingset_nVar < 1) {
    pk_corrected = 0.0;
  } else {
    n_t = (ptrdiff_t)workingset_nVar;
    incx_t = (ptrdiff_t)1;
    pk_corrected = dnrm2(&n_t, (real_T *)&solution_searchDir_data[0], &incx_t);
  }
  denomTol = 2.2204460492503131E-13 * pk_corrected;
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    int32_T i;
    phaseOneCorrectionX =
        (real_T)isPhaseOne * solution_xstar_data[workingset_nVar - 1];
    phaseOneCorrectionP =
        (real_T)isPhaseOne * solution_searchDir_data[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      pk_corrected =
          -solution_searchDir_data[workingset_indexLB_data[idx] - 1] -
          phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr_data[(workingset_isActiveIdx[3] + idx) -
                                           1])) {
        ratio = (-solution_xstar_data[workingset_indexLB_data[idx] - 1] -
                 workingset_lb_data[workingset_indexLB_data[idx] - 1]) -
                phaseOneCorrectionX;
        pk_corrected =
            muDoubleScalarMin(muDoubleScalarAbs(ratio), 0.001 - ratio) /
            pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
    i = workingset_indexLB_data[workingset_sizes[3] - 1] - 1;
    pk_corrected = -solution_searchDir_data[i];
    if ((pk_corrected > denomTol) &&
        (!workingset_isActiveConstr_data
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar_data[i] - workingset_lb_data[i];
      pk_corrected =
          muDoubleScalarMin(muDoubleScalarAbs(ratio), 0.001 - ratio) /
          pk_corrected;
      if (pk_corrected < alpha) {
        alpha = pk_corrected;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX =
        (real_T)isPhaseOne * solution_xstar_data[workingset_nVar - 1];
    phaseOneCorrectionP =
        (real_T)isPhaseOne * solution_searchDir_data[workingset_nVar - 1];
    for (idx = 0; idx < totalUB; idx++) {
      pk_corrected = solution_searchDir_data[workingset_indexUB_data[idx] - 1] -
                     phaseOneCorrectionP;
      if ((pk_corrected > denomTol) &&
          (!workingset_isActiveConstr_data[(workingset_isActiveIdx[4] + idx) -
                                           1])) {
        ratio = (solution_xstar_data[workingset_indexUB_data[idx] - 1] -
                 workingset_ub_data[workingset_indexUB_data[idx] - 1]) -
                phaseOneCorrectionX;
        pk_corrected =
            muDoubleScalarMin(muDoubleScalarAbs(ratio), 0.001 - ratio) /
            pk_corrected;
        if (pk_corrected < alpha) {
          alpha = pk_corrected;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }
  if (!isPhaseOne) {
    if ((*newBlocking) && (alpha > 1.0)) {
      *newBlocking = false;
    }
    alpha = muDoubleScalarMin(alpha, 1.0);
  }
  return alpha;
}

/* End of code generation (feasibleratiotest.c) */
