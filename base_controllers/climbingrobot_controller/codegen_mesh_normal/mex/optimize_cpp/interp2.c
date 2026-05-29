/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * interp2.c
 *
 * Code generation for function 'interp2'
 *
 */

/* Include files */
#include "interp2.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
real_T interp2(const real_T varargin_1[10000], const real_T varargin_2[10000],
               const real_T varargin_3[10000], real_T varargin_4,
               real_T varargin_5)
{
  real_T b_v[100];
  real_T v[100];
  real_T Vq;
  int32_T low_ip1;
  memcpy(&b_v[0], &varargin_2[0], 100U * sizeof(real_T));
  for (low_ip1 = 0; low_ip1 < 100; low_ip1++) {
    v[low_ip1] = varargin_1[100 * low_ip1];
  }
  if ((varargin_4 >= v[0]) && (varargin_4 <= v[99]) && (varargin_5 >= b_v[0]) &&
      (varargin_5 <= b_v[99])) {
    real_T qx1;
    real_T ry;
    int32_T b_low_i;
    int32_T high_i;
    int32_T low_i;
    int32_T mid_i;
    low_i = 0;
    low_ip1 = 2;
    high_i = 100;
    while (high_i > low_ip1) {
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (varargin_4 >= v[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }
    b_low_i = 1;
    low_ip1 = 2;
    high_i = 100;
    while (high_i > low_ip1) {
      mid_i = (b_low_i + high_i) >> 1;
      if (varargin_5 >= b_v[mid_i - 1]) {
        b_low_i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }
    if (varargin_4 == v[low_i]) {
      low_ip1 = b_low_i + 100 * low_i;
      qx1 = varargin_3[low_ip1 - 1];
      Vq = varargin_3[low_ip1];
    } else {
      ry = v[low_i + 1];
      if (varargin_4 == ry) {
        low_ip1 = b_low_i + 100 * (low_i + 1);
        qx1 = varargin_3[low_ip1 - 1];
        Vq = varargin_3[low_ip1];
      } else {
        real_T rx;
        rx = (varargin_4 - v[low_i]) / (ry - v[low_i]);
        high_i = b_low_i + 100 * low_i;
        ry = varargin_3[high_i - 1];
        low_ip1 = b_low_i + 100 * (low_i + 1);
        Vq = varargin_3[low_ip1 - 1];
        if (ry == Vq) {
          qx1 = ry;
        } else {
          qx1 = (1.0 - rx) * ry + rx * Vq;
        }
        Vq = varargin_3[high_i];
        ry = varargin_3[low_ip1];
        if (!(Vq == ry)) {
          Vq = (1.0 - rx) * Vq + rx * ry;
        }
      }
    }
    ry = b_v[b_low_i - 1];
    if ((varargin_5 == ry) || (qx1 == Vq)) {
      Vq = qx1;
    } else if (!(varargin_5 == b_v[b_low_i])) {
      ry = (varargin_5 - ry) / (b_v[b_low_i] - ry);
      Vq = (1.0 - ry) * qx1 + ry * Vq;
    }
  } else {
    Vq = 0.0;
  }
  return Vq;
}

/* End of code generation (interp2.c) */
