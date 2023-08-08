/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * _coder_optimize_cpp_mpc_propellers_api.c
 *
 * Code generation for function '_coder_optimize_cpp_mpc_propellers_api'
 *
 */

/* Include files */
#include "_coder_optimize_cpp_mpc_propellers_api.h"
#include "optimize_cpp_mpc_propellers.h"
#include "optimize_cpp_mpc_propellers_data.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static real_T b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId);
static real_T (*c_emlrt_marshallIn(const mxArray *actual_state, const char_T
  *identifier))[6];
static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[6];
static void e_emlrt_marshallIn(const mxArray *ref_com, const char_T *identifier,
  emxArray_real_T *y);
static real_T emlrt_marshallIn(const mxArray *a__output_of_feval_, const char_T *
  identifier);
static const mxArray *emlrt_marshallOut(const real_T u);
static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, emxArray_real_T *y);
static void g_emlrt_marshallIn(const mxArray *Fr_l0, const char_T *identifier,
  emxArray_real_T *y);
static void h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, emxArray_real_T *y);
static int64_T i_emlrt_marshallIn(const mxArray *mpc_N, const char_T *identifier);
static int64_T j_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId);
static void k_emlrt_marshallIn(const mxArray *params, const char_T *identifier,
  param *y);
static void l_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, param *y);
static void m_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, char_T y[3]);
static void n_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T y[3]);
static real_T o_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId);
static real_T (*p_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[6];
static void q_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, emxArray_real_T *ret);
static void r_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, emxArray_real_T *ret);
static int64_T s_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId);
static void t_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, char_T ret[3]);
static void u_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T ret[3]);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId)
{
  real_T y;
  y = o_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*c_emlrt_marshallIn(const mxArray *actual_state, const char_T
  *identifier))[6]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[6];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(actual_state), &thisId);
  emlrtDestroyArray(&actual_state);
  return y;
}
  static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u)
{
  static const int32_T iv[2] = { 0, 0 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u->data[0]);
  emlrtSetDimensions((mxArray *)m, u->size, 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[6]
{
  real_T (*y)[6];
  y = p_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static void e_emlrt_marshallIn(const mxArray *ref_com, const char_T
  *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(emlrtAlias(ref_com), &thisId, y);
  emlrtDestroyArray(&ref_com);
}

static real_T emlrt_marshallIn(const mxArray *a__output_of_feval_, const char_T *
  identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(a__output_of_feval_), &thisId);
  emlrtDestroyArray(&a__output_of_feval_);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, emxArray_real_T *y)
{
  q_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const mxArray *Fr_l0, const char_T *identifier,
  emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  h_emlrt_marshallIn(emlrtAlias(Fr_l0), &thisId, y);
  emlrtDestroyArray(&Fr_l0);
}

static void h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, emxArray_real_T *y)
{
  r_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static int64_T i_emlrt_marshallIn(const mxArray *mpc_N, const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  int64_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(emlrtAlias(mpc_N), &thisId);
  emlrtDestroyArray(&mpc_N);
  return y;
}

static int64_T j_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId)
{
  int64_T y;
  y = s_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void k_emlrt_marshallIn(const mxArray *params, const char_T *identifier,
  param *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  l_emlrt_marshallIn(emlrtAlias(params), &thisId, y);
  emlrtDestroyArray(&params);
}

static void l_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, param *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[11] = { "int_method", "int_steps",
    "contact_normal", "b", "p_a1", "p_a2", "g", "m", "w1", "w2", "mpc_dt" };

  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(emlrtRootTLSGlobal, parentId, u, 11, fieldNames, 0U,
    &dims);
  thisId.fIdentifier = "int_method";
  m_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 0,
    "int_method")), &thisId, y->int_method);
  thisId.fIdentifier = "int_steps";
  y->int_steps = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 1, "int_steps")), &thisId);
  thisId.fIdentifier = "contact_normal";
  n_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 2,
    "contact_normal")), &thisId, y->contact_normal);
  thisId.fIdentifier = "b";
  y->b = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
    0, 3, "b")), &thisId);
  thisId.fIdentifier = "p_a1";
  n_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 4,
    "p_a1")), &thisId, y->p_a1);
  thisId.fIdentifier = "p_a2";
  n_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 5,
    "p_a2")), &thisId, y->p_a2);
  thisId.fIdentifier = "g";
  y->g = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
    0, 6, "g")), &thisId);
  thisId.fIdentifier = "m";
  y->m = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
    0, 7, "m")), &thisId);
  thisId.fIdentifier = "w1";
  y->w1 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 8, "w1")), &thisId);
  thisId.fIdentifier = "w2";
  y->w2 = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal,
    u, 0, 9, "w2")), &thisId);
  thisId.fIdentifier = "mpc_dt";
  y->mpc_dt = b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b
    (emlrtRootTLSGlobal, u, 0, 10, "mpc_dt")), &thisId);
  emlrtDestroyArray(&u);
}

static void m_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, char_T y[3])
{
  t_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void n_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T y[3])
{
  u_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T o_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 0U,
    &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*p_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[6]
{
  static const int32_T dims[1] = { 6 };

  real_T (*ret)[6];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
    dims);
  ret = (real_T (*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static void q_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, emxArray_real_T *ret)
{
  static const int32_T dims[2] = { 3, -1 };

  int32_T iv[2];
  int32_T i;
  const boolean_T bv[2] = { false, true };

  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims, &bv[0], iv);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(ret, i);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static void r_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, emxArray_real_T *ret)
{
  static const int32_T dims[2] = { 1, -1 };

  int32_T iv[2];
  int32_T i;
  const boolean_T bv[2] = { false, true };

  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims, &bv[0], iv);
  ret->allocatedSize = iv[0] * iv[1];
  i = ret->size[0] * ret->size[1];
  ret->size[0] = iv[0];
  ret->size[1] = iv[1];
  emxEnsureCapacity_real_T(ret, i);
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static int64_T s_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId)
{
  static const int32_T dims = 0;
  int64_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "int64", false, 0U,
    &dims);
  ret = *(int64_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void t_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, char_T ret[3])
{
  static const int32_T dims[2] = { 1, 3 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "char", false, 2U,
    dims);
  emlrtImportCharArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 3);
  emlrtDestroyArray(&src);
}

static void u_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
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

void optimize_cpp_mpc_propellers_api(const mxArray * const prhs[8], int32_T nlhs,
  const mxArray *plhs[3])
{
  emxArray_real_T *Fr_l0;
  emxArray_real_T *Fr_r0;
  emxArray_real_T *ref_com;
  emxArray_real_T *x;
  param params;
  int64_T mpc_N;
  real_T (*actual_state)[6];
  real_T EXITFLAG;
  real_T Fr_max;
  real_T actual_t;
  real_T final_cost;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&ref_com, 2, true);
  emxInit_real_T(&Fr_l0, 2, true);
  emxInit_real_T(&Fr_r0, 2, true);
  emxInit_real_T(&x, 2, true);

  /* Marshall function inputs */
  actual_state = c_emlrt_marshallIn(emlrtAlias(prhs[0]), "actual_state");
  actual_t = emlrt_marshallIn(emlrtAliasP(prhs[1]), "actual_t");
  ref_com->canFreeData = false;
  e_emlrt_marshallIn(emlrtAlias(prhs[2]), "ref_com", ref_com);
  Fr_l0->canFreeData = false;
  g_emlrt_marshallIn(emlrtAlias(prhs[3]), "Fr_l0", Fr_l0);
  Fr_r0->canFreeData = false;
  g_emlrt_marshallIn(emlrtAlias(prhs[4]), "Fr_r0", Fr_r0);
  Fr_max = emlrt_marshallIn(emlrtAliasP(prhs[5]), "Fr_max");
  mpc_N = i_emlrt_marshallIn(emlrtAliasP(prhs[6]), "mpc_N");
  k_emlrt_marshallIn(emlrtAliasP(prhs[7]), "params", &params);

  /* Invoke the target function */
  optimize_cpp_mpc_propellers(*actual_state, actual_t, ref_com, Fr_l0, Fr_r0,
    Fr_max, mpc_N, &params, x, &EXITFLAG, &final_cost);

  /* Marshall function outputs */
  x->canFreeData = false;
  plhs[0] = c_emlrt_marshallOut(x);
  emxFree_real_T(&x);
  emxFree_real_T(&Fr_r0);
  emxFree_real_T(&Fr_l0);
  emxFree_real_T(&ref_com);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(EXITFLAG);
  }

  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(final_cost);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (_coder_optimize_cpp_mpc_propellers_api.c) */
