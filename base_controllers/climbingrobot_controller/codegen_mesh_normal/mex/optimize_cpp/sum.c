/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sum.c
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "sum.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include <string.h>

/* Function Definitions */
real_T sum(const emxArray_real_T *x)
{
  emxArray_real_T c_x;
  real_T y;
  int32_T d_x;
  int32_T e_x;
  int32_T f_x;
  int32_T g_x;
  int32_T ib;
  if (x->size[1] == 0) {
    y = 0.0;
  } else if (x->size[1] < 4096) {
    int32_T b_x;
    b_x = x->size[1];
    c_x = *x;
    d_x = b_x;
    c_x.size = &d_x;
    c_x.numDimensions = 1;
    y = sumColumnB(&c_x, x->size[1]);
  } else {
    int32_T b_x;
    int32_T inb;
    int32_T nfb;
    int32_T nleft;
    nfb = (int32_T)((uint32_T)x->size[1] >> 12);
    inb = nfb << 12;
    nleft = x->size[1] - inb;
    b_x = x->size[1];
    c_x = *x;
    e_x = b_x;
    c_x.size = &e_x;
    c_x.numDimensions = 1;
    y = sumColumnB4(&c_x, 1);
    if (nfb >= 2) {
      b_x = x->size[1];
    }
    for (ib = 2; ib <= nfb; ib++) {
      c_x = *x;
      f_x = b_x;
      c_x.size = &f_x;
      c_x.numDimensions = 1;
      y += sumColumnB4(&c_x, ((ib - 1) << 12) + 1);
    }
    if (nleft > 0) {
      b_x = x->size[1];
      c_x = *x;
      g_x = b_x;
      c_x.size = &g_x;
      c_x.numDimensions = 1;
      y += b_sumColumnB(&c_x, nleft, inb + 1);
    }
  }
  return y;
}

/* End of code generation (sum.c) */
