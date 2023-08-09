/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * forwardKin.c
 *
 * Code generation for function 'forwardKin'
 *
 */

/* Include files */
#include "forwardKin.h"
#include "optimize_cpp_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo sb_emlrtRSI = { 3,  /* lineNo */
  "forwardKin",                        /* fcnName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/forwardKin.m"/* pathName */
};

static emlrtRSInfo tb_emlrtRSI = { 4,  /* lineNo */
  "forwardKin",                        /* fcnName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/forwardKin.m"/* pathName */
};

static emlrtRSInfo ub_emlrtRSI = { 5,  /* lineNo */
  "forwardKin",                        /* fcnName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/forwardKin.m"/* pathName */
};

static emlrtRSInfo yb_emlrtRSI = { 41, /* lineNo */
  "applyBinaryScalarFunction",         /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/eml/+coder/+internal/applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo ac_emlrtRSI = { 139,/* lineNo */
  "scalar_float_power",                /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/ops/power.m"/* pathName */
};

static emlrtRSInfo hd_emlrtRSI = { 15, /* lineNo */
  "forwardKin",                        /* fcnName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/forwardKin.m"/* pathName */
};

static emlrtRSInfo id_emlrtRSI = { 17, /* lineNo */
  "forwardKin",                        /* fcnName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/forwardKin.m"/* pathName */
};

static emlrtRTEInfo o_emlrtRTEI = { 82,/* lineNo */
  5,                                   /* colNo */
  "fltpower",                          /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/ops/power.m"/* pName */
};

/* Function Definitions */
void b_forwardKin(const emlrtStack *sp, real_T params_b, real_T psi, real_T l1,
                  real_T l2, real_T psid, real_T l1d, real_T l2d, real_T *px,
                  real_T *py, real_T *pz, real_T *pdx, real_T *pdy, real_T *pdz)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T a_tmp;
  real_T a_tmp_tmp;
  real_T b_a_tmp;
  real_T c_a_tmp;
  real_T d_a_tmp;
  real_T e_a_tmp;
  real_T n_pz_l1;
  real_T px_l1_tmp;
  real_T px_tmp;
  real_T py2b;
  real_T pz_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  px_tmp = muDoubleScalarSin(psi);
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  a_tmp = params_b * params_b;
  b_a_tmp = l1 * l1;
  a_tmp_tmp = l2 * l2;
  c_a_tmp = (a_tmp + b_a_tmp) - a_tmp_tmp;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  d_a_tmp = 4.0 * a_tmp * b_a_tmp;
  e_a_tmp = 1.0 - c_a_tmp * c_a_tmp / d_a_tmp;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  e_st.site = &yb_emlrtRSI;
  f_st.site = &ac_emlrtRSI;
  if (e_a_tmp < 0.0) {
    emlrtErrorWithMessageIdR2018a(&f_st, &b_emlrtRTEI,
      "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError", 3, 4,
      4, "sqrt");
  }

  if (e_a_tmp < 0.0) {
    emlrtErrorWithMessageIdR2018a(&c_st, &o_emlrtRTEI,
      "Coder:toolbox:power_domainError", "Coder:toolbox:power_domainError", 0);
  }

  *px = l1 * px_tmp * muDoubleScalarSqrt(e_a_tmp);
  st.site = &tb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &tb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &tb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  *py = c_a_tmp / (2.0 * params_b);
  pz_tmp = muDoubleScalarCos(psi);
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  e_st.site = &yb_emlrtRSI;
  f_st.site = &ac_emlrtRSI;
  if (e_a_tmp < 0.0) {
    emlrtErrorWithMessageIdR2018a(&f_st, &b_emlrtRTEI,
      "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError", 3, 4,
      4, "sqrt");
  }

  if (e_a_tmp < 0.0) {
    emlrtErrorWithMessageIdR2018a(&c_st, &o_emlrtRTEI,
      "Coder:toolbox:power_domainError", "Coder:toolbox:power_domainError", 0);
  }

  *pz = -l1 * pz_tmp * muDoubleScalarSqrt(e_a_tmp);
  px_l1_tmp = *px / l1;
  n_pz_l1 = -*pz / l1;
  py2b = *py * 2.0 * params_b;
  st.site = &hd_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &hd_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &hd_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &hd_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &hd_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  e_a_tmp = ((l1d * a_tmp - l1d * b_a_tmp) + 2.0 * l2d * l1 * l2) - l1d *
    a_tmp_tmp;
  c_a_tmp = d_a_tmp * (px_l1_tmp / px_tmp);
  *pdx = (l1d * px_l1_tmp + l1 * n_pz_l1 * psid) + py2b * px_tmp * e_a_tmp /
    c_a_tmp;
  *pdy = (l1 * l1d - l2 * l2d) / params_b;
  st.site = &id_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &id_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &id_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &id_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &id_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  *pdz = (l1 * psid * px_l1_tmp - l1d * n_pz_l1) - py2b * pz_tmp * e_a_tmp /
    c_a_tmp;
}

void forwardKin(const emlrtStack *sp, real_T params_b, real_T psi, real_T l1,
                real_T l2, real_T *px, real_T *py, real_T *pz)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T a_tmp;
  real_T b_a_tmp;
  real_T c_a_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &sb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  a_tmp = params_b * params_b;
  b_a_tmp = l1 * l1;
  c_a_tmp = (a_tmp + b_a_tmp) - l2 * l2;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &sb_emlrtRSI;
  a_tmp = 1.0 - c_a_tmp * c_a_tmp / (4.0 * a_tmp * b_a_tmp);
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  e_st.site = &yb_emlrtRSI;
  f_st.site = &ac_emlrtRSI;
  if (a_tmp < 0.0) {
    emlrtErrorWithMessageIdR2018a(&f_st, &b_emlrtRTEI,
      "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError", 3, 4,
      4, "sqrt");
  }

  *px = l1 * muDoubleScalarSin(psi) * muDoubleScalarSqrt(a_tmp);
  st.site = &tb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &tb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &tb_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  *py = c_a_tmp / (2.0 * params_b);
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  st.site = &ub_emlrtRSI;
  b_st.site = &vb_emlrtRSI;
  c_st.site = &wb_emlrtRSI;
  d_st.site = &xb_emlrtRSI;
  e_st.site = &yb_emlrtRSI;
  f_st.site = &ac_emlrtRSI;
  if (a_tmp < 0.0) {
    emlrtErrorWithMessageIdR2018a(&f_st, &b_emlrtRTEI,
      "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError", 3, 4,
      4, "sqrt");
  }

  *pz = -l1 * muDoubleScalarCos(psi) * muDoubleScalarSqrt(a_tmp);
}

/* End of code generation (forwardKin.c) */
