/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * _coder_optimize_cpp_api.c
 *
 * Code generation for function '_coder_optimize_cpp_api'
 *
 */

/* Include files */
#include "_coder_optimize_cpp_api.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_mexutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*c_emlrt_marshallIn(const mxArray *p0, const char_T *identifier))
  [3];
static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[3];
static void e_emlrt_marshallIn(const mxArray *params, const char_T *identifier,
  param *y);
static const mxArray *emlrt_marshallOut(const struct0_T *u);
static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, param *y);
static boolean_T g_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId);
static void h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T y[3]);
static void i_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, char_T y[3]);
static real_T (*k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[3];
static boolean_T l_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId);
static void m_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T ret[3]);
static void n_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, char_T ret[3]);

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T iv[2];
  int32_T b_i;
  int32_T i;
  y = NULL;
  iv[0] = u->size[0];
  iv[1] = u->size[1];
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u->size[1]; b_i++) {
    pData[i] = u->data[b_i];
    i++;
  }

  emlrtAssign(&y, m);
  return y;
}

static real_T (*c_emlrt_marshallIn(const mxArray *p0, const char_T *identifier))
  [3]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(p0), &thisId);
  emlrtDestroyArray(&p0);
  return y;
}
  static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *
  parentId))[3]
{
  real_T (*y)[3];
  y = k_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void e_emlrt_marshallIn(const mxArray *params, const char_T *identifier,
  param *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(emlrtAlias(params), &thisId, y);
  emlrtDestroyArray(&params);
}

static const mxArray *emlrt_marshallOut(const struct0_T *u)
{
  static const int32_T iv[1] = { 3 };

  static const int32_T iv2[1] = { 3 };

  static const char_T *sv[30] = { "path_length", "initial_error",
    "final_error_real", "Fleg", "Fr_l", "Fr_r", "p", "psi", "l1", "l2", "psid",
    "l1d", "l2d", "time", "Tf", "achieved_target", "Etot", "Ekin", "Ekin0x",
    "Ekin0y", "Ekin0z", "Ekin0", "intEkin", "U0", "Uf", "Ekinfx", "Ekinfy",
    "Ekinfz", "Ekinf", "T_th" };

  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *j_y;
  const mxArray *k_y;
  const mxArray *l_y;
  const mxArray *m;
  const mxArray *m_y;
  const mxArray *n_y;
  const mxArray *o_y;
  const mxArray *p_y;
  const mxArray *q_y;
  const mxArray *r_y;
  const mxArray *s_y;
  const mxArray *t_y;
  const mxArray *u_y;
  const mxArray *y;
  real_T *pData;
  int32_T iv1[2];
  int32_T b_i;
  int32_T i;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 30, sv));
  b_y = NULL;
  m = emlrtCreateDoubleScalar(u->path_length);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "path_length", b_y, 0);
  c_y = NULL;
  m = emlrtCreateDoubleScalar(u->initial_error);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "initial_error", c_y, 1);
  d_y = NULL;
  m = emlrtCreateDoubleScalar(u->final_error_real);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "final_error_real", d_y, 2);
  e_y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->Fleg[0];
  pData[1] = u->Fleg[1];
  pData[2] = u->Fleg[2];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "Fleg", e_y, 3);
  emlrtSetFieldR2017b(y, 0, "Fr_l", b_emlrt_marshallOut(u->Fr_l), 4);
  emlrtSetFieldR2017b(y, 0, "Fr_r", b_emlrt_marshallOut(u->Fr_r), 5);
  f_y = NULL;
  iv1[0] = u->p->size[0];
  iv1[1] = u->p->size[1];
  m = emlrtCreateNumericArray(2, &iv1[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u->p->size[1]; b_i++) {
    pData[i] = u->p->data[3 * b_i];
    i++;
    pData[i] = u->p->data[3 * b_i + 1];
    i++;
    pData[i] = u->p->data[3 * b_i + 2];
    i++;
  }

  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "p", f_y, 6);
  emlrtSetFieldR2017b(y, 0, "psi", b_emlrt_marshallOut(u->psi), 7);
  emlrtSetFieldR2017b(y, 0, "l1", b_emlrt_marshallOut(u->l1), 8);
  emlrtSetFieldR2017b(y, 0, "l2", b_emlrt_marshallOut(u->l2), 9);
  emlrtSetFieldR2017b(y, 0, "psid", b_emlrt_marshallOut(u->psid), 10);
  emlrtSetFieldR2017b(y, 0, "l1d", b_emlrt_marshallOut(u->l1d), 11);
  emlrtSetFieldR2017b(y, 0, "l2d", b_emlrt_marshallOut(u->l2d), 12);
  emlrtSetFieldR2017b(y, 0, "time", b_emlrt_marshallOut(u->time), 13);
  g_y = NULL;
  m = emlrtCreateDoubleScalar(u->Tf);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "Tf", g_y, 14);
  h_y = NULL;
  m = emlrtCreateNumericArray(1, &iv2[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->achieved_target[0];
  pData[1] = u->achieved_target[1];
  pData[2] = u->achieved_target[2];
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0, "achieved_target", h_y, 15);
  i_y = NULL;
  m = emlrtCreateDoubleScalar(u->Etot);
  emlrtAssign(&i_y, m);
  emlrtSetFieldR2017b(y, 0, "Etot", i_y, 16);
  emlrtSetFieldR2017b(y, 0, "Ekin", b_emlrt_marshallOut(u->Ekin), 17);
  j_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0x);
  emlrtAssign(&j_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0x", j_y, 18);
  k_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0y);
  emlrtAssign(&k_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0y", k_y, 19);
  l_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0z);
  emlrtAssign(&l_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0z", l_y, 20);
  m_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0);
  emlrtAssign(&m_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0", m_y, 21);
  n_y = NULL;
  m = emlrtCreateDoubleScalar(u->intEkin);
  emlrtAssign(&n_y, m);
  emlrtSetFieldR2017b(y, 0, "intEkin", n_y, 22);
  o_y = NULL;
  m = emlrtCreateDoubleScalar(u->U0);
  emlrtAssign(&o_y, m);
  emlrtSetFieldR2017b(y, 0, "U0", o_y, 23);
  p_y = NULL;
  m = emlrtCreateDoubleScalar(u->Uf);
  emlrtAssign(&p_y, m);
  emlrtSetFieldR2017b(y, 0, "Uf", p_y, 24);
  q_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinfx);
  emlrtAssign(&q_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinfx", q_y, 25);
  r_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinfy);
  emlrtAssign(&r_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinfy", r_y, 26);
  s_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinfz);
  emlrtAssign(&s_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinfz", s_y, 27);
  t_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinf);
  emlrtAssign(&t_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinf", t_y, 28);
  u_y = NULL;
  m = emlrtCreateDoubleScalar(u->T_th);
  emlrtAssign(&u_y, m);
  emlrtSetFieldR2017b(y, 0, "T_th", u_y, 29);
  return y;
}

static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, param *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[21] = { "jump_clearance", "m",
    "obstacle_avoidance", "obstacle_location", "num_params", "int_method",
    "N_dyn", "FRICTION_CONE", "int_steps", "contact_normal", "b", "p_a1", "p_a2",
    "g", "w1", "w2", "w3", "w4", "w5", "w6", "T_th" };

  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(emlrtRootTLSGlobal, parentId, u, 21, fieldNames, 0U,
    &dims);
  thisId.fIdentifier = "jump_clearance";
  y->jump_clearance = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 0, "jump_clearance")), &thisId);
  thisId.fIdentifier = "m";
  y->m = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
    0, 1, "m")), &thisId);
  thisId.fIdentifier = "obstacle_avoidance";
  y->obstacle_avoidance = g_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 2, "obstacle_avoidance")), &thisId);
  thisId.fIdentifier = "obstacle_location";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 3,
    "obstacle_location")), &thisId, y->obstacle_location);
  thisId.fIdentifier = "num_params";
  y->num_params = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 4, "num_params")), &thisId);
  thisId.fIdentifier = "int_method";
  i_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 5,
    "int_method")), &thisId, y->int_method);
  thisId.fIdentifier = "N_dyn";
  y->N_dyn = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 6, "N_dyn")), &thisId);
  thisId.fIdentifier = "FRICTION_CONE";
  y->FRICTION_CONE = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 7, "FRICTION_CONE")), &thisId);
  thisId.fIdentifier = "int_steps";
  y->int_steps = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 8, "int_steps")), &thisId);
  thisId.fIdentifier = "contact_normal";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 9,
    "contact_normal")), &thisId, y->contact_normal);
  thisId.fIdentifier = "b";
  y->b = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
    0, 10, "b")), &thisId);
  thisId.fIdentifier = "p_a1";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 11,
    "p_a1")), &thisId, y->p_a1);
  thisId.fIdentifier = "p_a2";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 12,
    "p_a2")), &thisId, y->p_a2);
  thisId.fIdentifier = "g";
  y->g = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
    0, 13, "g")), &thisId);
  thisId.fIdentifier = "w1";
  y->w1 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 14, "w1")), &thisId);
  thisId.fIdentifier = "w2";
  y->w2 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 15, "w2")), &thisId);
  thisId.fIdentifier = "w3";
  y->w3 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 16, "w3")), &thisId);
  thisId.fIdentifier = "w4";
  y->w4 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 17, "w4")), &thisId);
  thisId.fIdentifier = "w5";
  y->w5 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 18, "w5")), &thisId);
  thisId.fIdentifier = "w6";
  y->w6 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 19, "w6")), &thisId);
  thisId.fIdentifier = "T_th";
  y->T_th = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 20, "T_th")), &thisId);
  emlrtDestroyArray(&u);
}

static boolean_T g_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId)
{
  boolean_T y;
  y = l_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T y[3])
{
  m_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, char_T y[3])
{
  n_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T (*k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[3]
{
  static const int32_T dims[2] = { 1, 3 };

  real_T (*ret)[3];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims);
  ret = (real_T (*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static boolean_T l_emlrt_marshallIn(const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  boolean_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "logical", false, 0U,
    &dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void m_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T ret[3])
{
  static const int32_T dims[1] = { 3 };

  real_T (*r)[3];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
    dims);
  r = (real_T (*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static void n_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, char_T ret[3])
{
  static const int32_T dims[2] = { 1, 3 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "char", false, 2U,
    dims);
  emlrtImportCharArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 3);
  emlrtDestroyArray(&src);
}

void optimize_cpp_api(const mxArray * const prhs[6], const mxArray *plhs[1])
{
  param params;
  struct0_T solution;
  real_T (*p0)[3];
  real_T (*pf)[3];
  real_T Fleg_max;
  real_T Fr_max;
  real_T mu;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInitStruct_struct0_T(&solution, true);

  /* Marshall function inputs */
  p0 = c_emlrt_marshallIn(emlrtAlias(prhs[0]), "p0");
  pf = c_emlrt_marshallIn(emlrtAlias(prhs[1]), "pf");
  Fleg_max = emlrt_marshallIn(emlrtAliasP(prhs[2]), "Fleg_max");
  Fr_max = emlrt_marshallIn(emlrtAliasP(prhs[3]), "Fr_max");
  mu = emlrt_marshallIn(emlrtAliasP(prhs[4]), "mu");
  e_emlrt_marshallIn(emlrtAliasP(prhs[5]), "params", &params);

  /* Invoke the target function */
  optimize_cpp(*p0, *pf, Fleg_max, Fr_max, mu, &params, &solution);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(&solution);
  emxFreeStruct_struct0_T(&solution);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (_coder_optimize_cpp_api.c) */
