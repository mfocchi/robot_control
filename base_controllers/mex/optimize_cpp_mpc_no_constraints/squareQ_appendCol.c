/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * squareQ_appendCol.c
 *
 * Code generation for function 'squareQ_appendCol'
 *
 */

/* Include files */
#include "squareQ_appendCol.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xrot.h"
#include "blas.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void squareQ_appendCol(g_struct_T *obj, const emxArray_real_T *vec, int32_T iv0)
{
  real_T c;
  real_T d;
  real_T d1;
  real_T s;
  int32_T Qk0;
  int32_T idx;
  idx = obj->ncols + 1;
  obj->minRowCol = muIntScalarMin_sint32(obj->mrows, idx);
  d_xgemv(obj->mrows, obj->mrows, obj->Q, obj->ldq, vec, iv0, obj->QR, obj->ldq *
          obj->ncols + 1);
  obj->ncols++;
  obj->jpvt->data[obj->ncols - 1] = obj->ncols;
  for (idx = obj->mrows - 2; idx + 2 > obj->ncols; idx--) {
    d = obj->QR->data[idx + obj->QR->size[0] * (obj->ncols - 1)];
    d1 = obj->QR->data[(idx + obj->QR->size[0] * (obj->ncols - 1)) + 1];
    c = 0.0;
    s = 0.0;
    drotg(&d, &d1, &c, &s);
    obj->QR->data[idx + obj->QR->size[0] * (obj->ncols - 1)] = d;
    obj->QR->data[(idx + obj->QR->size[0] * (obj->ncols - 1)) + 1] = d1;
    Qk0 = obj->ldq * idx + 1;
    xrot(obj->mrows, obj->Q, Qk0, obj->ldq + Qk0, c, s);
  }
}

/* End of code generation (squareQ_appendCol.c) */
