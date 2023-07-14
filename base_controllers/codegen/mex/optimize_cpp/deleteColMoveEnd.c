/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * deleteColMoveEnd.c
 *
 * Code generation for function 'deleteColMoveEnd'
 *
 */

/* Include files */
#include "deleteColMoveEnd.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void deleteColMoveEnd(c_struct_T *obj, int32_T idx)
{
  real_T c;
  real_T d;
  real_T s;
  real_T temp;
  int32_T b_k;
  int32_T i;
  int32_T k;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt->data[i - 1] != idx)) {
      i++;
    }
    idx = i;
  }
  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    int32_T b_i;
    b_i = obj->ncols - 1;
    obj->jpvt->data[idx - 1] = obj->jpvt->data[b_i];
    i = obj->minRowCol;
    for (k = 0; k < i; k++) {
      obj->QR->data[k + obj->ldq * (idx - 1)] =
          obj->QR->data[k + obj->ldq * b_i];
    }
    obj->ncols = b_i;
    obj->minRowCol = muIntScalarMin_sint32(obj->mrows, obj->ncols);
    if (idx < obj->mrows) {
      int32_T endIdx;
      int32_T idxRotGCol;
      int32_T ix;
      int32_T n;
      int32_T temp_tmp;
      i = obj->mrows - 1;
      endIdx = muIntScalarMin_sint32(i, obj->ncols);
      k = endIdx;
      idxRotGCol = obj->ldq * (idx - 1);
      while (k >= idx) {
        b_i = k + idxRotGCol;
        temp = obj->QR->data[b_i - 1];
        d = obj->QR->data[b_i];
        c = 0.0;
        s = 0.0;
        drotg(&temp, &d, &c, &s);
        obj->QR->data[b_i - 1] = temp;
        obj->QR->data[b_i] = d;
        b_i = obj->ldq * (k - 1);
        obj->QR->data[k + b_i] = 0.0;
        i = k + obj->ldq * idx;
        n = obj->ncols - idx;
        if (n >= 1) {
          ix = i - 1;
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * obj->QR->data[ix] + s * obj->QR->data[i];
            obj->QR->data[i] = c * obj->QR->data[i] - s * obj->QR->data[ix];
            obj->QR->data[ix] = temp;
            i += obj->ldq;
            ix += obj->ldq;
          }
        }
        i = obj->ldq + b_i;
        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          ix = i + b_k;
          temp_tmp = b_i + b_k;
          temp = c * obj->Q->data[temp_tmp] + s * obj->Q->data[ix];
          obj->Q->data[ix] = c * obj->Q->data[ix] - s * obj->Q->data[temp_tmp];
          obj->Q->data[temp_tmp] = temp;
        }
        k--;
      }
      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        idxRotGCol = obj->ldq * (k - 1);
        i = k + idxRotGCol;
        temp = obj->QR->data[i - 1];
        d = obj->QR->data[i];
        c = 0.0;
        s = 0.0;
        drotg(&temp, &d, &c, &s);
        obj->QR->data[i - 1] = temp;
        obj->QR->data[i] = d;
        i = k * (obj->ldq + 1);
        n = obj->ncols - k;
        if (n >= 1) {
          ix = i - 1;
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * obj->QR->data[ix] + s * obj->QR->data[i];
            obj->QR->data[i] = c * obj->QR->data[i] - s * obj->QR->data[ix];
            obj->QR->data[ix] = temp;
            i += obj->ldq;
            ix += obj->ldq;
          }
        }
        i = obj->ldq + idxRotGCol;
        n = obj->mrows;
        for (b_k = 0; b_k < n; b_k++) {
          ix = i + b_k;
          temp_tmp = idxRotGCol + b_k;
          temp = c * obj->Q->data[temp_tmp] + s * obj->Q->data[ix];
          obj->Q->data[ix] = c * obj->Q->data[ix] - s * obj->Q->data[temp_tmp];
          obj->Q->data[temp_tmp] = temp;
        }
      }
    }
  }
}

/* End of code generation (deleteColMoveEnd.c) */
