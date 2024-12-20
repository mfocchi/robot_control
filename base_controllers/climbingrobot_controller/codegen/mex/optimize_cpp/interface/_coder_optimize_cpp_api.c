/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
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

static real_T (*c_emlrt_marshallIn(const mxArray *p0,
                                   const char_T *identifier))[3];

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);

static real_T (*d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3];

static void e_emlrt_marshallIn(const mxArray *params, const char_T *identifier,
                               param *y);

static const mxArray *emlrt_marshallOut(const struct0_T *u);

static void f_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId, param *y);

static boolean_T g_emlrt_marshallIn(const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void h_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static void i_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[3]);

static real_T (*k_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static boolean_T l_emlrt_marshallIn(const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static void m_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void n_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[3]);

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *m;
  const mxArray *y;
  const real_T *u_data;
  real_T *pData;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T i;
  u_data = u->data;
  y = NULL;
  b_iv[0] = 1;
  b_iv[1] = u->size[1];
  m = emlrtCreateNumericArray(2, &b_iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u->size[1]; b_i++) {
    pData[i] = u_data[b_i];
    i++;
  }
  emlrtAssign(&y, m);
  return y;
}

static real_T (*c_emlrt_marshallIn(const mxArray *p0,
                                   const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(p0), &thisId);
  emlrtDestroyArray(&p0);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *m;
  const mxArray *y;
  const real_T *u_data;
  real_T *pData;
  int32_T b_iv[2];
  int32_T b_i;
  int32_T i;
  u_data = u->data;
  y = NULL;
  b_iv[0] = 3;
  b_iv[1] = u->size[1];
  m = emlrtCreateNumericArray(2, &b_iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u->size[1]; b_i++) {
    pData[i] = u_data[3 * b_i];
    pData[i + 1] = u_data[3 * b_i + 1];
    pData[i + 2] = u_data[3 * b_i + 2];
    i += 3;
  }
  emlrtAssign(&y, m);
  return y;
}

static real_T (*d_emlrt_marshallIn(const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
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
  static const int32_T b_iv[2] = {1, 3};
  static const int32_T i = 3;
  static const int32_T i1 = 3;
  static const char_T *sv[47] = {"path_length",
                                 "initial_error",
                                 "final_error_real",
                                 "Fleg",
                                 "Fr_l",
                                 "Fr_r",
                                 "p",
                                 "psi",
                                 "l1",
                                 "l2",
                                 "psid",
                                 "l1d",
                                 "l2d",
                                 "time",
                                 "Fr_l_fine",
                                 "Fr_r_fine",
                                 "p_fine",
                                 "psi_fine",
                                 "l1_fine",
                                 "l2_fine",
                                 "psid_fine",
                                 "l1d_fine",
                                 "l2d_fine",
                                 "time_fine",
                                 "Tf",
                                 "achieved_target",
                                 "Etot",
                                 "Ekin",
                                 "Ekin0x",
                                 "Ekin0y",
                                 "Ekin0z",
                                 "Ekin0",
                                 "intEkin",
                                 "U0",
                                 "Uf",
                                 "Ekinfx",
                                 "Ekinfy",
                                 "Ekinfz",
                                 "Ekinf",
                                 "T_th",
                                 "cost",
                                 "problem_solved",
                                 "optim_output",
                                 "c",
                                 "num_constr",
                                 "solution_constr",
                                 "constr_tolerance"};
  static const char_T *sv3[9] = {"p",   "psi",  "l1",
                                 "l2",  "psid", "l1d",
                                 "l2d", "time", "final_error_discrete"};
  static const char_T *sv1[7] = {"iterations",      "funcCount", "algorithm",
                                 "constrviolation", "stepsize",  "lssteplength",
                                 "firstorderopt"};
  static const char_T *sv2[5] = {
      "wall_constraints", "retraction_force_constraints", "force_constraints",
      "initial_final_constraints", "via_point"};
  const mxArray *ab_y;
  const mxArray *b_y;
  const mxArray *bb_y;
  const mxArray *c_y;
  const mxArray *cb_y;
  const mxArray *d_y;
  const mxArray *db_y;
  const mxArray *e_y;
  const mxArray *eb_y;
  const mxArray *f_y;
  const mxArray *fb_y;
  const mxArray *g_y;
  const mxArray *gb_y;
  const mxArray *h_y;
  const mxArray *hb_y;
  const mxArray *i_y;
  const mxArray *ib_y;
  const mxArray *j_y;
  const mxArray *jb_y;
  const mxArray *k_y;
  const mxArray *kb_y;
  const mxArray *l_y;
  const mxArray *lb_y;
  const mxArray *m;
  const mxArray *m_y;
  const mxArray *mb_y;
  const mxArray *n_y;
  const mxArray *nb_y;
  const mxArray *o_y;
  const mxArray *p_y;
  const mxArray *q_y;
  const mxArray *r_y;
  const mxArray *s_y;
  const mxArray *t_y;
  const mxArray *u_y;
  const mxArray *v_y;
  const mxArray *w_y;
  const mxArray *x_y;
  const mxArray *y;
  const mxArray *y_y;
  real_T *pData;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 47, (const char_T **)&sv[0]));
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
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->Fleg[0];
  pData[1] = u->Fleg[1];
  pData[2] = u->Fleg[2];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "Fleg", e_y, 3);
  emlrtSetFieldR2017b(y, 0, "Fr_l", b_emlrt_marshallOut(u->Fr_l), 4);
  emlrtSetFieldR2017b(y, 0, "Fr_r", b_emlrt_marshallOut(u->Fr_r), 5);
  emlrtSetFieldR2017b(y, 0, "p", c_emlrt_marshallOut(u->p), 6);
  emlrtSetFieldR2017b(y, 0, "psi", b_emlrt_marshallOut(u->psi), 7);
  emlrtSetFieldR2017b(y, 0, "l1", b_emlrt_marshallOut(u->l1), 8);
  emlrtSetFieldR2017b(y, 0, "l2", b_emlrt_marshallOut(u->l2), 9);
  emlrtSetFieldR2017b(y, 0, "psid", b_emlrt_marshallOut(u->psid), 10);
  emlrtSetFieldR2017b(y, 0, "l1d", b_emlrt_marshallOut(u->l1d), 11);
  emlrtSetFieldR2017b(y, 0, "l2d", b_emlrt_marshallOut(u->l2d), 12);
  emlrtSetFieldR2017b(y, 0, "time", b_emlrt_marshallOut(u->time), 13);
  emlrtSetFieldR2017b(y, 0, "Fr_l_fine", b_emlrt_marshallOut(u->Fr_l_fine), 14);
  emlrtSetFieldR2017b(y, 0, "Fr_r_fine", b_emlrt_marshallOut(u->Fr_r_fine), 15);
  emlrtSetFieldR2017b(y, 0, "p_fine", c_emlrt_marshallOut(u->p_fine), 16);
  emlrtSetFieldR2017b(y, 0, "psi_fine", b_emlrt_marshallOut(u->psi_fine), 17);
  emlrtSetFieldR2017b(y, 0, "l1_fine", b_emlrt_marshallOut(u->l1_fine), 18);
  emlrtSetFieldR2017b(y, 0, "l2_fine", b_emlrt_marshallOut(u->l2_fine), 19);
  emlrtSetFieldR2017b(y, 0, "psid_fine", b_emlrt_marshallOut(u->psid_fine), 20);
  emlrtSetFieldR2017b(y, 0, "l1d_fine", b_emlrt_marshallOut(u->l1d_fine), 21);
  emlrtSetFieldR2017b(y, 0, "l2d_fine", b_emlrt_marshallOut(u->l2d_fine), 22);
  emlrtSetFieldR2017b(y, 0, "time_fine", b_emlrt_marshallOut(u->time_fine), 23);
  f_y = NULL;
  m = emlrtCreateDoubleScalar(u->Tf);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "Tf", f_y, 24);
  g_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->achieved_target[0];
  pData[1] = u->achieved_target[1];
  pData[2] = u->achieved_target[2];
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "achieved_target", g_y, 25);
  h_y = NULL;
  m = emlrtCreateDoubleScalar(u->Etot);
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0, "Etot", h_y, 26);
  emlrtSetFieldR2017b(y, 0, "Ekin", b_emlrt_marshallOut(u->Ekin), 27);
  i_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0x);
  emlrtAssign(&i_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0x", i_y, 28);
  j_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0y);
  emlrtAssign(&j_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0y", j_y, 29);
  k_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0z);
  emlrtAssign(&k_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0z", k_y, 30);
  l_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekin0);
  emlrtAssign(&l_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekin0", l_y, 31);
  m_y = NULL;
  m = emlrtCreateDoubleScalar(u->intEkin);
  emlrtAssign(&m_y, m);
  emlrtSetFieldR2017b(y, 0, "intEkin", m_y, 32);
  n_y = NULL;
  m = emlrtCreateDoubleScalar(u->U0);
  emlrtAssign(&n_y, m);
  emlrtSetFieldR2017b(y, 0, "U0", n_y, 33);
  o_y = NULL;
  m = emlrtCreateDoubleScalar(u->Uf);
  emlrtAssign(&o_y, m);
  emlrtSetFieldR2017b(y, 0, "Uf", o_y, 34);
  p_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinfx);
  emlrtAssign(&p_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinfx", p_y, 35);
  q_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinfy);
  emlrtAssign(&q_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinfy", q_y, 36);
  r_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinfz);
  emlrtAssign(&r_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinfz", r_y, 37);
  s_y = NULL;
  m = emlrtCreateDoubleScalar(u->Ekinf);
  emlrtAssign(&s_y, m);
  emlrtSetFieldR2017b(y, 0, "Ekinf", s_y, 38);
  t_y = NULL;
  m = emlrtCreateDoubleScalar(u->T_th);
  emlrtAssign(&t_y, m);
  emlrtSetFieldR2017b(y, 0, "T_th", t_y, 39);
  u_y = NULL;
  m = emlrtCreateDoubleScalar(u->cost);
  emlrtAssign(&u_y, m);
  emlrtSetFieldR2017b(y, 0, "cost", u_y, 40);
  v_y = NULL;
  m = emlrtCreateDoubleScalar(u->problem_solved);
  emlrtAssign(&v_y, m);
  emlrtSetFieldR2017b(y, 0, "problem_solved", v_y, 41);
  w_y = NULL;
  emlrtAssign(&w_y, emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&sv1[0]));
  x_y = NULL;
  m = emlrtCreateDoubleScalar(u->optim_output.iterations);
  emlrtAssign(&x_y, m);
  emlrtSetFieldR2017b(w_y, 0, "iterations", x_y, 0);
  y_y = NULL;
  m = emlrtCreateDoubleScalar(u->optim_output.funcCount);
  emlrtAssign(&y_y, m);
  emlrtSetFieldR2017b(w_y, 0, "funcCount", y_y, 1);
  ab_y = NULL;
  m = emlrtCreateCharArray(2, &b_iv[0]);
  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 3, m,
                           &u->optim_output.algorithm[0]);
  emlrtAssign(&ab_y, m);
  emlrtSetFieldR2017b(w_y, 0, "algorithm", ab_y, 2);
  bb_y = NULL;
  m = emlrtCreateDoubleScalar(u->optim_output.constrviolation);
  emlrtAssign(&bb_y, m);
  emlrtSetFieldR2017b(w_y, 0, "constrviolation", bb_y, 3);
  cb_y = NULL;
  m = emlrtCreateDoubleScalar(u->optim_output.stepsize);
  emlrtAssign(&cb_y, m);
  emlrtSetFieldR2017b(w_y, 0, "stepsize", cb_y, 4);
  db_y = NULL;
  m = emlrtCreateDoubleScalar(u->optim_output.lssteplength);
  emlrtAssign(&db_y, m);
  emlrtSetFieldR2017b(w_y, 0, "lssteplength", db_y, 5);
  eb_y = NULL;
  m = emlrtCreateDoubleScalar(u->optim_output.firstorderopt);
  emlrtAssign(&eb_y, m);
  emlrtSetFieldR2017b(w_y, 0, "firstorderopt", eb_y, 6);
  emlrtSetFieldR2017b(y, 0, "optim_output", w_y, 42);
  emlrtSetFieldR2017b(y, 0, "c", b_emlrt_marshallOut(u->c), 43);
  fb_y = NULL;
  emlrtAssign(&fb_y,
              emlrtCreateStructMatrix(1, 1, 5, (const char_T **)&sv2[0]));
  gb_y = NULL;
  m = emlrtCreateDoubleScalar(u->num_constr.wall_constraints);
  emlrtAssign(&gb_y, m);
  emlrtSetFieldR2017b(fb_y, 0, "wall_constraints", gb_y, 0);
  hb_y = NULL;
  m = emlrtCreateDoubleScalar(u->num_constr.retraction_force_constraints);
  emlrtAssign(&hb_y, m);
  emlrtSetFieldR2017b(fb_y, 0, "retraction_force_constraints", hb_y, 1);
  ib_y = NULL;
  m = emlrtCreateDoubleScalar(u->num_constr.force_constraints);
  emlrtAssign(&ib_y, m);
  emlrtSetFieldR2017b(fb_y, 0, "force_constraints", ib_y, 2);
  jb_y = NULL;
  m = emlrtCreateDoubleScalar(u->num_constr.initial_final_constraints);
  emlrtAssign(&jb_y, m);
  emlrtSetFieldR2017b(fb_y, 0, "initial_final_constraints", jb_y, 3);
  kb_y = NULL;
  m = emlrtCreateDoubleScalar(u->num_constr.via_point);
  emlrtAssign(&kb_y, m);
  emlrtSetFieldR2017b(fb_y, 0, "via_point", kb_y, 4);
  emlrtSetFieldR2017b(y, 0, "num_constr", fb_y, 44);
  lb_y = NULL;
  emlrtAssign(&lb_y,
              emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&sv3[0]));
  emlrtSetFieldR2017b(lb_y, 0, "p", c_emlrt_marshallOut(u->solution_constr.p),
                      0);
  emlrtSetFieldR2017b(lb_y, 0, "psi",
                      b_emlrt_marshallOut(u->solution_constr.psi), 1);
  emlrtSetFieldR2017b(lb_y, 0, "l1", b_emlrt_marshallOut(u->solution_constr.l1),
                      2);
  emlrtSetFieldR2017b(lb_y, 0, "l2", b_emlrt_marshallOut(u->solution_constr.l2),
                      3);
  emlrtSetFieldR2017b(lb_y, 0, "psid",
                      b_emlrt_marshallOut(u->solution_constr.psid), 4);
  emlrtSetFieldR2017b(lb_y, 0, "l1d",
                      b_emlrt_marshallOut(u->solution_constr.l1d), 5);
  emlrtSetFieldR2017b(lb_y, 0, "l2d",
                      b_emlrt_marshallOut(u->solution_constr.l2d), 6);
  emlrtSetFieldR2017b(lb_y, 0, "time",
                      b_emlrt_marshallOut(u->solution_constr.time), 7);
  mb_y = NULL;
  m = emlrtCreateDoubleScalar(u->solution_constr.final_error_discrete);
  emlrtAssign(&mb_y, m);
  emlrtSetFieldR2017b(lb_y, 0, "final_error_discrete", mb_y, 8);
  emlrtSetFieldR2017b(y, 0, "solution_constr", lb_y, 45);
  nb_y = NULL;
  m = emlrtCreateDoubleScalar(u->constr_tolerance);
  emlrtAssign(&nb_y, m);
  emlrtSetFieldR2017b(y, 0, "constr_tolerance", nb_y, 46);
  return y;
}

static void f_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId, param *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[22] = {"jump_clearance",
                                         "m",
                                         "obstacle_avoidance",
                                         "obstacle_location",
                                         "obstacle_size",
                                         "num_params",
                                         "int_method",
                                         "N_dyn",
                                         "FRICTION_CONE",
                                         "int_steps",
                                         "contact_normal",
                                         "b",
                                         "p_a1",
                                         "p_a2",
                                         "g",
                                         "w1",
                                         "w2",
                                         "w3",
                                         "w4",
                                         "w5",
                                         "w6",
                                         "T_th"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(emlrtRootTLSGlobal, parentId, u, 22,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "jump_clearance";
  y->jump_clearance =
      b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(
                             emlrtRootTLSGlobal, u, 0, 0, "jump_clearance")),
                         &thisId);
  thisId.fIdentifier = "m";
  y->m = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 1, "m")),
      &thisId);
  thisId.fIdentifier = "obstacle_avoidance";
  y->obstacle_avoidance = g_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 2,
                                     "obstacle_avoidance")),
      &thisId);
  thisId.fIdentifier = "obstacle_location";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 3,
                                                    "obstacle_location")),
                     &thisId, y->obstacle_location);
  thisId.fIdentifier = "obstacle_size";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 4,
                                                    "obstacle_size")),
                     &thisId, y->obstacle_size);
  thisId.fIdentifier = "num_params";
  y->num_params =
      b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
                                                        0, 5, "num_params")),
                         &thisId);
  thisId.fIdentifier = "int_method";
  i_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 6,
                                                    "int_method")),
                     &thisId, y->int_method);
  thisId.fIdentifier = "N_dyn";
  y->N_dyn = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 7, "N_dyn")),
      &thisId);
  thisId.fIdentifier = "FRICTION_CONE";
  y->FRICTION_CONE =
      b_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u,
                                                        0, 8, "FRICTION_CONE")),
                         &thisId);
  thisId.fIdentifier = "int_steps";
  y->int_steps = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 9, "int_steps")),
      &thisId);
  thisId.fIdentifier = "contact_normal";
  h_emlrt_marshallIn(emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0,
                                                    10, "contact_normal")),
                     &thisId, y->contact_normal);
  thisId.fIdentifier = "b";
  y->b = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 11, "b")),
      &thisId);
  thisId.fIdentifier = "p_a1";
  h_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 12, "p_a1")),
      &thisId, y->p_a1);
  thisId.fIdentifier = "p_a2";
  h_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 13, "p_a2")),
      &thisId, y->p_a2);
  thisId.fIdentifier = "g";
  y->g = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 14, "g")),
      &thisId);
  thisId.fIdentifier = "w1";
  y->w1 = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 15, "w1")),
      &thisId);
  thisId.fIdentifier = "w2";
  y->w2 = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 16, "w2")),
      &thisId);
  thisId.fIdentifier = "w3";
  y->w3 = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 17, "w3")),
      &thisId);
  thisId.fIdentifier = "w4";
  y->w4 = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 18, "w4")),
      &thisId);
  thisId.fIdentifier = "w5";
  y->w5 = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 19, "w5")),
      &thisId);
  thisId.fIdentifier = "w6";
  y->w6 = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 20, "w6")),
      &thisId);
  thisId.fIdentifier = "T_th";
  y->T_th = b_emlrt_marshallIn(
      emlrtAlias(emlrtGetFieldR2017b(emlrtRootTLSGlobal, u, 0, 21, "T_th")),
      &thisId);
  emlrtDestroyArray(&u);
}

static boolean_T g_emlrt_marshallIn(const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = l_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void h_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  m_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[3])
{
  n_emlrt_marshallIn(emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T (*k_emlrt_marshallIn(const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims[2] = {1, 3};
  real_T(*ret)[3];
  int32_T b_iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &b_iv[0]);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static boolean_T l_emlrt_marshallIn(const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  boolean_T ret;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void m_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims = 3;
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static void n_emlrt_marshallIn(const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[3])
{
  static const int32_T dims[2] = {1, 3};
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b(emlrtRootTLSGlobal, src, &ret[0], 3);
  emlrtDestroyArray(&src);
}

void optimize_cpp_api(const mxArray *const prhs[7], const mxArray **plhs)
{
  param params;
  struct0_T solution;
  real_T(*p0)[3];
  real_T(*pf)[3];
  real_T Fleg_max;
  real_T Fr_max;
  real_T Fr_min;
  real_T mu;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /* Marshall function inputs */
  p0 = c_emlrt_marshallIn(emlrtAlias(prhs[0]), "p0");
  pf = c_emlrt_marshallIn(emlrtAlias(prhs[1]), "pf");
  Fleg_max = emlrt_marshallIn(emlrtAliasP(prhs[2]), "Fleg_max");
  Fr_max = emlrt_marshallIn(emlrtAliasP(prhs[3]), "Fr_max");
  Fr_min = emlrt_marshallIn(emlrtAliasP(prhs[4]), "Fr_min");
  mu = emlrt_marshallIn(emlrtAliasP(prhs[5]), "mu");
  e_emlrt_marshallIn(emlrtAliasP(prhs[6]), "params", &params);
  /* Invoke the target function */
  emxInitStruct_struct0_T(&solution);
  optimize_cpp(*p0, *pf, Fleg_max, Fr_max, Fr_min, mu, &params, &solution);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(&solution);
  emxFreeStruct_struct0_T(&solution);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (_coder_optimize_cpp_api.c) */
