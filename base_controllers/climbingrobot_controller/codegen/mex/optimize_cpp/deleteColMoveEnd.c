/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * deleteColMoveEnd.c
 *
 * Code generation for function 'deleteColMoveEnd'
 *
 */

/* Include files */
#include "deleteColMoveEnd.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "xrot.h"
#include "blas.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo lh_emlrtRSI = { 1,  /* lineNo */
  "deleteColMoveEnd",                  /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+QRManager/deleteColMoveEnd.p"/* pathName */
};

static emlrtBCInfo hf_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  1,                                   /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "deleteColMoveEnd",                  /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+QRManager/deleteColMoveEnd.p",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void deleteColMoveEnd(const emlrtStack *sp, f_struct_T *obj, int32_T idx)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T c;
  real_T d;
  real_T d1;
  real_T s;
  int32_T a;
  int32_T b_i;
  int32_T endIdx;
  int32_T i;
  int32_T k;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (obj->usedPivoting) {
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i <= obj->ncols)) {
      b_i = obj->jpvt->size[0];
      if ((i < 1) || (i > b_i)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &hf_emlrtBCI, sp);
      }

      if (obj->jpvt->data[i - 1] != idx) {
        i++;
      } else {
        exitg1 = true;
      }
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    b_i = obj->jpvt->size[0];
    if ((obj->ncols < 1) || (obj->ncols > b_i)) {
      emlrtDynamicBoundsCheckR2012b(obj->ncols, 1, b_i, &hf_emlrtBCI, sp);
    }

    b_i = obj->jpvt->size[0];
    if ((idx < 1) || (idx > b_i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &hf_emlrtBCI, sp);
    }

    obj->jpvt->data[idx - 1] = obj->jpvt->data[obj->ncols - 1];
    i = obj->minRowCol;
    st.site = &lh_emlrtRSI;
    if ((1 <= obj->minRowCol) && (obj->minRowCol > 2147483646)) {
      b_st.site = &t_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (k = 0; k < i; k++) {
      b_i = obj->QR->size[0];
      if ((k + 1 < 1) || (k + 1 > b_i)) {
        emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_i, &hf_emlrtBCI, sp);
      }

      b_i = obj->QR->size[1];
      if ((obj->ncols < 1) || (obj->ncols > b_i)) {
        emlrtDynamicBoundsCheckR2012b(obj->ncols, 1, b_i, &hf_emlrtBCI, sp);
      }

      b_i = obj->QR->size[0];
      if ((k + 1 < 1) || (k + 1 > b_i)) {
        emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_i, &hf_emlrtBCI, sp);
      }

      b_i = obj->QR->size[1];
      if (idx > b_i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &hf_emlrtBCI, sp);
      }

      obj->QR->data[k + obj->QR->size[0] * (idx - 1)] = obj->QR->data[k +
        obj->QR->size[0] * (obj->ncols - 1)];
    }

    obj->ncols--;
    obj->minRowCol = muIntScalarMin_sint32(obj->mrows, obj->ncols);
    if (idx < obj->mrows) {
      i = obj->mrows - 1;
      endIdx = muIntScalarMin_sint32(i, obj->ncols);
      for (k = endIdx; k >= idx; k--) {
        st.site = &lh_emlrtRSI;
        b_i = obj->QR->size[0];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, &st);
        }

        b_i = obj->QR->size[1];
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &hf_emlrtBCI, &st);
        }

        d = obj->QR->data[(k + obj->QR->size[0] * (idx - 1)) - 1];
        b_i = obj->QR->size[0];
        if ((k + 1 < 1) || (k + 1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_i, &hf_emlrtBCI, &st);
        }

        b_i = obj->QR->size[1];
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &hf_emlrtBCI, &st);
        }

        d1 = obj->QR->data[k + obj->QR->size[0] * (idx - 1)];
        c = 0.0;
        s = 0.0;
        drotg(&d, &d1, &c, &s);
        b_i = obj->QR->size[0];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, sp);
        }

        b_i = obj->QR->size[1];
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &hf_emlrtBCI, sp);
        }

        obj->QR->data[(k + obj->QR->size[0] * (idx - 1)) - 1] = d;
        b_i = obj->QR->size[0];
        if ((k + 1 < 1) || (k + 1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_i, &hf_emlrtBCI, sp);
        }

        b_i = obj->QR->size[1];
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &hf_emlrtBCI, sp);
        }

        obj->QR->data[k + obj->QR->size[0] * (idx - 1)] = d1;
        b_i = obj->QR->size[0];
        if ((k + 1 < 1) || (k + 1 > b_i)) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_i, &hf_emlrtBCI, sp);
        }

        b_i = obj->QR->size[1];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, sp);
        }

        obj->QR->data[k + obj->QR->size[0] * (k - 1)] = 0.0;
        i = k + obj->ldq * idx;
        st.site = &lh_emlrtRSI;
        b_xrot(obj->ncols - idx, obj->QR, i, obj->ldq, i + 1, obj->ldq, c, s);
        i = obj->ldq * (k - 1) + 1;
        st.site = &lh_emlrtRSI;
        xrot(obj->mrows, obj->Q, i, obj->ldq + i, c, s);
      }

      a = idx + 1;
      st.site = &lh_emlrtRSI;
      if ((idx + 1 <= endIdx) && (endIdx > 2147483646)) {
        b_st.site = &t_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (k = a; k <= endIdx; k++) {
        st.site = &lh_emlrtRSI;
        b_i = obj->QR->size[0];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, &st);
        }

        b_i = obj->QR->size[1];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, &st);
        }

        d = obj->QR->data[(k + obj->QR->size[0] * (k - 1)) - 1];
        b_i = obj->QR->size[0];
        if (k + 1 > b_i) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_i, &hf_emlrtBCI, &st);
        }

        b_i = obj->QR->size[1];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, &st);
        }

        d1 = obj->QR->data[k + obj->QR->size[0] * (k - 1)];
        c = 0.0;
        s = 0.0;
        drotg(&d, &d1, &c, &s);
        b_i = obj->QR->size[0];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, sp);
        }

        b_i = obj->QR->size[1];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, sp);
        }

        obj->QR->data[(k + obj->QR->size[0] * (k - 1)) - 1] = d;
        b_i = obj->QR->size[0];
        if (k + 1 > b_i) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, b_i, &hf_emlrtBCI, sp);
        }

        b_i = obj->QR->size[1];
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &hf_emlrtBCI, sp);
        }

        obj->QR->data[k + obj->QR->size[0] * (k - 1)] = d1;
        i = k * (obj->ldq + 1);
        st.site = &lh_emlrtRSI;
        b_xrot(obj->ncols - k, obj->QR, i, obj->ldq, i + 1, obj->ldq, c, s);
        i = obj->ldq * (k - 1) + 1;
        st.site = &lh_emlrtRSI;
        xrot(obj->mrows, obj->Q, i, obj->ldq + i, c, s);
      }
    }
  }
}

/* End of code generation (deleteColMoveEnd.c) */
