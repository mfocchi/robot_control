/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * wallSurfaceEval.c
 *
 * Code generation for function 'wallSurfaceEval'
 *
 */

/* Include files */
#include "wallSurfaceEval.h"
#include "all.h"
#include "optimize_cpp_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtMCInfo c_emlrtMCI = {
    27,                                                            /* lineNo */
    5,                                                             /* colNo */
    "error",                                                       /* fName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/lib/matlab/lang/error.m" /* pName */
};

/* Function Declarations */
static void b_error(const mxArray *m, emlrtMCInfo *location);

/* Function Definitions */
static void b_error(const mxArray *m, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = m;
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, NULL, 1, &pArray, "error", true,
                        location);
}

real_T wallSurfaceEval(real_T z_query, real_T y_query,
                       const real_T params_mesh_x[10000],
                       const real_T params_mesh_y[10000],
                       const real_T params_mesh_z[10000])
{
  static const int32_T b_iv[2] = {1, 80};
  static const int32_T iv1[2] = {1, 304};
  static const char_T b_varargin_1[304] = {
      'p', 'a', 'r', 'a', 'm', 's', '.', 'c', 'o', 's', 't', '_', 'x', ' ', 'a',
      'p', 'p', 'e', 'a', 'r', 's', ' ', 't', 'o', ' ', 'b', 'e', ' ', 'i', 'd',
      'e', 'n', 't', 'i', 'c', 'a', 'l', ' ', 't', 'o', ' ', 'o', 'n', 'e', ' ',
      'o', 'f', ' ', 't', 'h', 'e', ' ', 'm', 'e', 's', 'h', ' ', 'm', 'a', 't',
      'r', 'i', 'c', 'e', 's', ' ', '(', 'm', 'e', 's', 'h', '_', 'z', ' ', 'o',
      'r', ' ', 'm', 'e', 's', 'h', '_', 'y', ')', '.', ' ', 'p', 'a', 'r', 'a',
      'm', 's', '.', 'c', 'o', 's', 't', '_', 'x', ' ', 'm', 'u', 's', 't', ' ',
      'b', 'e', ' ', 't', 'h', 'e', ' ', 'm', 'a', 't', 'r', 'i', 'x', ' ', 'o',
      'f', ' ', 'c', 'o', 's', 't', ' ', 'v', 'a', 'l', 'u', 'e', 's', ' ', 'e',
      'v', 'a', 'l', 'u', 'a', 't', 'e', 'd', ' ', 'o', 'n', ' ', 't', 'h', 'e',
      ' ', 'g', 'r', 'i', 'd', ',', ' ', 'i', '.', 'e', '.', ' ', 'c', 'o', 's',
      't', '_', 'x', ' ', '=', ' ', 'f', '(', 'm', 'e', 's', 'h', '_', 'z', ',',
      ' ', 'm', 'e', 's', 'h', '_', 'y', ')', '.', ' ', 'P', 'l', 'e', 'a', 's',
      'e', ' ', 'c', 'o', 'm', 'p', 'u', 't', 'e', ' ', 't', 'h', 'e', ' ', 'c',
      'o', 's', 't', ' ', 'm', 'a', 't', 'r', 'i', 'x', ' ', 'b', 'e', 'f', 'o',
      'r', 'e', ' ', 'c', 'a', 'l', 'l', 'i', 'n', 'g', ' ', 't', 'h', 'i', 's',
      ' ', 'f', 'u', 'n', 'c', 't', 'i', 'o', 'n', '.', ' ', 'E', 'x', 'a', 'm',
      'p', 'l', 'e', ':', ' ', 'p', 'a', 'r', 'a', 'm', 's', '.', 'c', 'o', 's',
      't', '_', 'x', ' ', '=', ' ', 'y', 'o', 'u', 'r', 'C', 'o', 's', 't', 'F',
      'u', 'n', '(', 'm', 'e', 's', 'h', '_', 'z', ',', ' ', 'm', 'e', 's', 'h',
      '_', 'y', ')', ';'};
  static const char_T varargin_1[80] = {
      'm', 'e', 's', 'h', 'g', 'r', 'i', 'd', ' ', 's', 't', 'r', 'u', 'c',
      't', 'u', 'r', 'e', ' ', 'u', 'n', 'e', 'x', 'p', 'e', 'c', 't', 'e',
      'd', ':', ' ', 'm', 'e', 's', 'h', '_', 'z', ' ', 'r', 'o', 'w', 's',
      ' ', 'o', 'r', ' ', 'm', 'e', 's', 'h', '_', 'y', ' ', 'c', 'o', 'l',
      'u', 'm', 'n', 's', ' ', 'a', 'r', 'e', ' ', 'n', 'o', 't', ' ', 'c',
      'o', 'n', 's', 'i', 's', 't', 'e', 'n', 't', '.'};
  static const int8_T vbbidx[4] = {1, 2, 101, 102};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  real_T V[10000];
  real_T dv[2];
  real_T qp[2];
  real_T xmax[2];
  real_T xmin[2];
  real_T r;
  real_T val;
  int32_T high_i;
  int32_T ibmat;
  int32_T idx;
  int32_T low_i;
  boolean_T y[10000];
  boolean_T x[100];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T is_y_cols_identical;
  boolean_T is_z_rows_identical;
  /*  --- Robust griddedInterpolant builder (meshgrid inputs) */
  /*  Basic checks */
  /*  Recover compact vectors from meshgrid */
  for (idx = 0; idx < 100; idx++) {
    ibmat = idx * 100;
    for (high_i = 0; high_i < 100; high_i++) {
      V[ibmat + high_i] = params_mesh_z[100 * idx];
    }
  }
  for (high_i = 0; high_i < 10000; high_i++) {
    r = params_mesh_z[high_i] - V[high_i];
    V[high_i] = r;
    y[high_i] = (muDoubleScalarAbs(r) < 1.0E-12);
  }
  all(y, x);
  is_z_rows_identical = true;
  high_i = 0;
  exitg1 = false;
  while ((!exitg1) && (high_i < 100)) {
    if (!x[high_i]) {
      is_z_rows_identical = false;
      exitg1 = true;
    } else {
      high_i++;
    }
  }
  for (idx = 0; idx < 100; idx++) {
    ibmat = idx * 100;
    memcpy(&V[ibmat], &params_mesh_y[0], 100U * sizeof(real_T));
  }
  for (high_i = 0; high_i < 10000; high_i++) {
    r = params_mesh_y[high_i] - V[high_i];
    V[high_i] = r;
    y[high_i] = (muDoubleScalarAbs(r) < 1.0E-12);
  }
  all(y, x);
  is_y_cols_identical = true;
  high_i = 0;
  exitg1 = false;
  while ((!exitg1) && (high_i < 100)) {
    if (!x[high_i]) {
      is_y_cols_identical = false;
      exitg1 = true;
    } else {
      high_i++;
    }
  }
  if ((!is_z_rows_identical) || (!is_y_cols_identical)) {
    b_y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 80, m, &varargin_1[0]);
    emlrtAssign(&b_y, m);
    b_error(b_y, &c_emlrtMCI);
  }
  /*  1 x nz */
  /*  ny x 1 */
  /*  Detect common user mistake: cost_x equals one of the mesh matrices */
  for (high_i = 0; high_i < 10000; high_i++) {
    V[high_i] =
        muDoubleScalarAbs(params_mesh_x[high_i] - params_mesh_z[high_i]);
  }
  is_z_rows_identical = true;
  high_i = 0;
  exitg1 = false;
  while ((!exitg1) && (high_i < 10000)) {
    if (!(V[high_i] < 1.0E-12)) {
      is_z_rows_identical = false;
      exitg1 = true;
    } else {
      high_i++;
    }
  }
  guard1 = false;
  if (is_z_rows_identical) {
    guard1 = true;
  } else {
    for (high_i = 0; high_i < 10000; high_i++) {
      V[high_i] =
          muDoubleScalarAbs(params_mesh_x[high_i] - params_mesh_y[high_i]);
    }
    is_z_rows_identical = true;
    high_i = 0;
    exitg1 = false;
    while ((!exitg1) && (high_i < 10000)) {
      if (!(V[high_i] < 1.0E-12)) {
        is_z_rows_identical = false;
        exitg1 = true;
      } else {
        high_i++;
      }
    }
    if (is_z_rows_identical) {
      guard1 = true;
    }
  }
  if (guard1) {
    c_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 304, m, &b_varargin_1[0]);
    emlrtAssign(&c_y, m);
    b_error(c_y, &c_emlrtMCI);
  }
  /*  Ensure V has orientation [numel(y_vec), numel(z_vec)] */
  /*  Build griddedInterpolant with linear extrapolation */
  /*  In theory I should use this that extrapolates the mesh out of domain */
  /*  but works worse for some reason */
  /* better implemetation that extrapolates last value out of domain */
  /*  this extrapolates out of the domain, the gradient is not defined  */
  /* wallSurfaceFcn = @(z_query, y_query) interp2(params.mesh_z, params.mesh_y,
   * params.mesh_x, z_query, y_query, 'linear', 0); */
  dv[0] = y_query;
  dv[1] = z_query;
  xmin[0] = params_mesh_y[0];
  xmax[0] = params_mesh_y[99];
  xmin[1] = params_mesh_z[0];
  xmax[1] = params_mesh_z[9900];
  memcpy(&V[0], &params_mesh_x[0], 10000U * sizeof(real_T));
  is_z_rows_identical = true;
  for (ibmat = 0; ibmat < 2; ibmat++) {
    r = dv[ibmat];
    qp[ibmat] = r;
    if ((!is_z_rows_identical) || (!(r >= xmin[ibmat])) ||
        (!(r <= xmax[ibmat]))) {
      is_z_rows_identical = false;
    }
  }
  if (is_z_rows_identical) {
    real_T vbox[4];
    real_T xbox[4];
    int32_T b_low_i;
    low_i = 1;
    ibmat = 2;
    high_i = 100;
    while (high_i > ibmat) {
      idx = (low_i + high_i) >> 1;
      if (qp[0] >= params_mesh_y[idx - 1]) {
        low_i = idx;
        ibmat = idx + 1;
      } else {
        high_i = idx;
      }
    }
    xbox[0] = params_mesh_y[low_i - 1];
    xbox[1] = params_mesh_y[low_i];
    b_low_i = 1;
    ibmat = 2;
    high_i = 100;
    while (high_i > ibmat) {
      idx = (b_low_i + high_i) >> 1;
      if (qp[1] >= params_mesh_z[100 * (idx - 1)]) {
        b_low_i = idx;
        ibmat = idx + 1;
      } else {
        high_i = idx;
      }
    }
    high_i = (b_low_i - 1) * 100;
    ibmat = low_i + high_i;
    xbox[2] = params_mesh_z[high_i];
    xbox[3] = params_mesh_z[100 * b_low_i];
    vbox[0] = V[ibmat - 1];
    vbox[1] = V[ibmat];
    vbox[2] = V[ibmat + 99];
    vbox[3] = V[ibmat + 100];
    for (low_i = 0; low_i < 2; low_i++) {
      real_T d;
      idx = (1 << (1 - low_i)) - 1;
      ibmat = low_i << 1;
      r = xbox[ibmat];
      d = qp[low_i];
      if (d == r) {
        for (high_i = 0; high_i <= idx; high_i++) {
          vbox[high_i] = vbox[((high_i + 1) << 1) - 2];
        }
      } else {
        real_T d1;
        d1 = xbox[ibmat + 1];
        if (d == d1) {
          for (high_i = 0; high_i <= idx; high_i++) {
            vbox[high_i] = vbox[((high_i + 1) << 1) - 1];
          }
        } else {
          r = (d - r) / (d1 - r);
          for (high_i = 0; high_i <= idx; high_i++) {
            ibmat = ((high_i + 1) << 1) - 1;
            vbox[high_i] = (1.0 - r) * vbox[ibmat - 1] + r * vbox[ibmat];
          }
        }
      }
    }
    val = vbox[0];
  } else {
    int32_T b_low_i;
    low_i = 1;
    ibmat = 2;
    high_i = 100;
    while (high_i > ibmat) {
      idx = (low_i + high_i) >> 1;
      if (qp[0] >= params_mesh_y[idx - 1]) {
        low_i = idx;
        ibmat = idx + 1;
      } else {
        high_i = idx;
      }
    }
    b_low_i = 1;
    ibmat = 2;
    high_i = 100;
    while (high_i > ibmat) {
      idx = (b_low_i + high_i) >> 1;
      if (qp[1] >= params_mesh_z[100 * (idx - 1)]) {
        b_low_i = idx;
        ibmat = idx + 1;
      } else {
        high_i = idx;
      }
    }
    ibmat = 100 * (b_low_i - 1);
    idx = 0;
    if (muDoubleScalarAbs(qp[0] - params_mesh_y[low_i]) <=
        muDoubleScalarAbs(qp[0] - params_mesh_y[low_i - 1])) {
      idx = 1;
    }
    if (muDoubleScalarAbs(qp[1] - params_mesh_z[100 * b_low_i]) <=
        muDoubleScalarAbs(qp[1] - params_mesh_z[ibmat])) {
      idx += 2;
    }
    val = V[((vbbidx[idx] + low_i) + ibmat) - 2];
  }
  return val;
}

/* End of code generation (wallSurfaceEval.c) */
