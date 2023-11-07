/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * initActiveSet.c
 *
 * Code generation for function 'initActiveSet'
 *
 */

/* Include files */
#include "initActiveSet.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ae_emlrtRSI = { 1,  /* lineNo */
  "initActiveSet",                     /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/initActiveSet.p"/* pathName */
};

static emlrtBCInfo id_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  1,                                   /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "initActiveSet",                     /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/initActiveSet.p",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void initActiveSet(const emlrtStack *sp, j_struct_T *obj)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T a;
  int32_T b;
  int32_T b_i;
  int32_T i;
  int32_T idx;
  int32_T idxFillStart;
  int32_T idx_local;
  boolean_T overflow;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ae_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  setProblemType(&st, obj, 3);
  idxFillStart = obj->isActiveIdx[2];
  b = obj->mConstrMax;
  st.site = &ae_emlrtRSI;
  if ((obj->isActiveIdx[2] <= obj->mConstrMax) && (obj->mConstrMax > 2147483646))
  {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = idxFillStart; idx <= b; idx++) {
    i = obj->isActiveConstr->size[0];
    if ((idx < 1) || (idx > i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, i, &id_emlrtBCI, sp);
    }

    obj->isActiveConstr->data[idx - 1] = false;
  }

  obj->nWConstr[0] = obj->sizes[0];
  obj->nWConstr[1] = 0;
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0];
  idxFillStart = obj->sizes[0];
  st.site = &ae_emlrtRSI;
  if ((1 <= obj->sizes[0]) && (obj->sizes[0] > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx_local = 0; idx_local < idxFillStart; idx_local++) {
    i = obj->Wid->size[0];
    if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
    }

    obj->Wid->data[idx_local] = 1;
    i = obj->Wlocalidx->size[0];
    if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
    }

    obj->Wlocalidx->data[idx_local] = idx_local + 1;
    i = obj->isActiveConstr->size[0];
    if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
    }

    obj->isActiveConstr->data[idx_local] = true;
    i = obj->indexFixed->size[0];
    if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
    }

    idx = obj->indexFixed->data[idx_local];
    st.site = &ae_emlrtRSI;
    if (1 > idx - 1) {
      overflow = false;
    } else {
      overflow = (obj->indexFixed->data[idx_local] - 1 > 2147483646);
    }

    if (overflow) {
      b_st.site = &t_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (b_i = 0; b_i <= idx - 2; b_i++) {
      i = obj->ATwset->size[0];
      if ((b_i + 1 < 1) || (b_i + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, i, &id_emlrtBCI, sp);
      }

      i = obj->ATwset->size[1];
      if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
      }

      obj->ATwset->data[b_i + obj->ATwset->size[0] * idx_local] = 0.0;
    }

    i = obj->ATwset->size[0];
    if ((idx < 1) || (idx > i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, i, &id_emlrtBCI, sp);
    }

    i = obj->ATwset->size[1];
    if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
    }

    obj->ATwset->data[(idx + obj->ATwset->size[0] * idx_local) - 1] = 1.0;
    a = idx + 1;
    b = obj->nVar;
    st.site = &ae_emlrtRSI;
    if ((idx + 1 <= obj->nVar) && (obj->nVar > 2147483646)) {
      b_st.site = &t_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (b_i = a; b_i <= b; b_i++) {
      i = obj->ATwset->size[0];
      if ((b_i < 1) || (b_i > i)) {
        emlrtDynamicBoundsCheckR2012b(b_i, 1, i, &id_emlrtBCI, sp);
      }

      i = obj->ATwset->size[1];
      if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
      }

      obj->ATwset->data[(b_i + obj->ATwset->size[0] * idx_local) - 1] = 0.0;
    }

    i = obj->ub->size[0];
    if (idx > i) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, i, &id_emlrtBCI, sp);
    }

    i = obj->bwset->size[0];
    if ((idx_local + 1 < 1) || (idx_local + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local + 1, 1, i, &id_emlrtBCI, sp);
    }

    obj->bwset->data[idx_local] = obj->ub->data[idx - 1];
  }

  st.site = &ae_emlrtRSI;
}

/* End of code generation (initActiveSet.c) */
