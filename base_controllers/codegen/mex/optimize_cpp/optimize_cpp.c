/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp.c
 *
 * Code generation for function 'optimize_cpp'
 *
 */

/* Include files */
#include "optimize_cpp.h"
#include "computePositionVelocity.h"
#include "computeRollout.h"
#include "diff.h"
#include "eml_i64dplus.h"
#include "eval_solution.h"
#include "fmincon.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "tic.h"
#include "toc.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void optimize_cpp(const real_T p0[3], const real_T pf[3], real_T Fleg_max,
                  real_T Fr_max, real_T mu, const param *params,
                  boolean_T *problem_solved, struct0_T *solution)
{
  emxArray_real_T *b_Fleg_max;
  emxArray_real_T *b_states;
  emxArray_real_T *b_x;
  emxArray_real_T *c_Fleg_max;
  emxArray_real_T *c_states;
  emxArray_real_T *d_Fleg_max;
  emxArray_real_T *d_states;
  emxArray_real_T *pd;
  emxArray_real_T *states;
  int64_T i;
  int64_T i1;
  real_T this_workspace_pf[3];
  real_T x[3];
  real_T EXITFLAG;
  real_T absxk;
  real_T b_expl_temp;
  real_T b_scale;
  real_T c_expl_temp;
  real_T l1_tmp;
  real_T l2_tmp;
  real_T scale;
  real_T t;
  real_T *Fleg_max_data;
  real_T *b_Fleg_max_data;
  real_T *b_states_data;
  real_T *c_Fleg_max_data;
  real_T *c_states_data;
  real_T *d_states_data;
  real_T *states_data;
  real_T *x_data;
  int32_T k;
  int32_T loop_ub;
  int32_T nx;
  int32_T vectorUB;
  char_T expl_temp[3];
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /* make it column vector */
  /* for eval solution */
  /*  needs to be fixed for code generation */
  /* compute initial state from jump param */
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p0[0] - params->p_a1[0]);
  if (absxk > 3.3121686421112381E-170) {
    l1_tmp = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    l1_tmp = t * t;
  }
  absxk = muDoubleScalarAbs(p0[0] - params->p_a2[0]);
  if (absxk > 3.3121686421112381E-170) {
    l2_tmp = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    l2_tmp = t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params->p_a1[1]);
  if (absxk > scale) {
    t = scale / absxk;
    l1_tmp = l1_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    l1_tmp += t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params->p_a2[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    l2_tmp = l2_tmp * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    l2_tmp += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params->p_a1[2]);
  if (absxk > scale) {
    t = scale / absxk;
    l1_tmp = l1_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    l1_tmp += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params->p_a2[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    l2_tmp = l2_tmp * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    l2_tmp += t * t;
  }
  l1_tmp = scale * muDoubleScalarSqrt(l1_tmp);
  l2_tmp = b_scale * muDoubleScalarSqrt(l2_tmp);
  /* pendulum period */
  /*  half period TODO replace with linearized x0(2) = l10 */
  /* opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r */
  /*  % does not always satisfy bounds */
  tic();
  x[0] = p0[0];
  this_workspace_pf[0] = pf[0];
  x[1] = p0[1];
  this_workspace_pf[1] = pf[1];
  x[2] = p0[2];
  this_workspace_pf[2] = pf[2];
  emxInit_real_T(&b_Fleg_max, 2);
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  vectorUB = ((int32_T)params->N_dyn + (int32_T)params->N_dyn) + 4;
  b_Fleg_max->size[1] = vectorUB;
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  Fleg_max_data[0] = Fleg_max;
  Fleg_max_data[1] = Fleg_max;
  Fleg_max_data[2] = Fleg_max;
  Fleg_max_data[3] =
      6.2831853071795862 * muDoubleScalarSqrt(l1_tmp / params->g) / 4.0;
  nx = (int32_T)params->N_dyn;
  for (k = 0; k < nx; k++) {
    Fleg_max_data[k + 4] = 0.0;
  }
  for (k = 0; k < nx; k++) {
    Fleg_max_data[(k + (int32_T)params->N_dyn) + 4] = 0.0;
  }
  emxInit_real_T(&c_Fleg_max, 2);
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = vectorUB;
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  b_Fleg_max_data[0] = -Fleg_max;
  b_Fleg_max_data[1] = -Fleg_max;
  b_Fleg_max_data[2] = -Fleg_max;
  b_Fleg_max_data[3] = 0.01;
  for (k = 0; k < nx; k++) {
    b_Fleg_max_data[k + 4] = -Fr_max;
  }
  for (k = 0; k < nx; k++) {
    b_Fleg_max_data[(k + (int32_T)params->N_dyn) + 4] = -Fr_max;
  }
  emxInit_real_T(&d_Fleg_max, 2);
  k = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = vectorUB;
  emxEnsureCapacity_real_T(d_Fleg_max, k);
  c_Fleg_max_data = d_Fleg_max->data;
  c_Fleg_max_data[0] = Fleg_max;
  c_Fleg_max_data[1] = Fleg_max;
  c_Fleg_max_data[2] = Fleg_max;
  c_Fleg_max_data[3] = rtInf;
  for (k = 0; k < nx; k++) {
    c_Fleg_max_data[k + 4] = 0.0;
  }
  for (k = 0; k < nx; k++) {
    c_Fleg_max_data[(k + (int32_T)params->N_dyn) + 4] = 0.0;
  }
  emxInit_real_T(&b_x, 2);
  fmincon(p0, params, b_Fleg_max, c_Fleg_max, d_Fleg_max, x, this_workspace_pf,
          Fleg_max, mu, params, b_x, expl_temp, &EXITFLAG, &b_expl_temp, &scale,
          &b_scale, &absxk, &t, &c_expl_temp);
  x_data = b_x->data;
  toc();
  /* eval trajectory */
  solution->Fleg[0] = x_data[0];
  solution->Fleg[1] = x_data[1];
  solution->Fleg[2] = x_data[2];
  i = plus(params->num_params, params->N_dyn);
  if (params->num_params + 1L > i) {
    k = 0;
    vectorUB = 0;
  } else {
    k = (int32_T)(params->num_params + 1L) - 1;
    vectorUB = (int32_T)i;
  }
  nx = solution->Fr_l->size[0] * solution->Fr_l->size[1];
  solution->Fr_l->size[0] = 1;
  loop_ub = vectorUB - k;
  solution->Fr_l->size[1] = loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_l, nx);
  for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
    solution->Fr_l->data[vectorUB] = x_data[k + vectorUB];
  }
  i1 = plus(params->num_params, 2.0 * params->N_dyn);
  if (i + 1L > i1) {
    k = 0;
    vectorUB = 0;
  } else {
    k = (int32_T)(i + 1L) - 1;
    vectorUB = (int32_T)i1;
  }
  nx = solution->Fr_r->size[0] * solution->Fr_r->size[1];
  solution->Fr_r->size[0] = 1;
  loop_ub = vectorUB - k;
  solution->Fr_r->size[1] = loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_r, nx);
  for (vectorUB = 0; vectorUB < loop_ub; vectorUB++) {
    solution->Fr_r->data[vectorUB] = x_data[k + vectorUB];
  }
  real_T dv[6];
  /*  single shooting */
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = l1_tmp;
  dv[2] = l2_tmp;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  emxInit_real_T(&states, 2);
  computeRollout(dv, x_data[3] / (params->N_dyn - 1.0), params->N_dyn,
                 solution->Fr_l, solution->Fr_r, solution->Fleg,
                 params->int_method, params->int_steps, params->m, params->b,
                 params->p_a1, params->p_a2, params->g, params->T_th, states,
                 solution->time);
  states_data = states->data;
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  loop_ub = states->size[1];
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  k = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_Fleg_max, k);
  c_Fleg_max_data = d_Fleg_max->data;
  emxInit_real_T(&b_states, 2);
  k = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, k);
  b_states_data = b_states->data;
  emxInit_real_T(&c_states, 2);
  k = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, k);
  c_states_data = c_states->data;
  emxInit_real_T(&d_states, 2);
  k = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, k);
  d_states_data = d_states->data;
  for (k = 0; k < loop_ub; k++) {
    Fleg_max_data[k] = states_data[6 * k];
    b_Fleg_max_data[k] = states_data[6 * k + 1];
    c_Fleg_max_data[k] = states_data[6 * k + 2];
    b_states_data[k] = states_data[6 * k + 3];
    c_states_data[k] = states_data[6 * k + 4];
    d_states_data[k] = states_data[6 * k + 5];
  }
  emxInit_real_T(&pd, 2);
  computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
                          b_states, c_states, d_states, solution->p, pd);
  c_states_data = pd->data;
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  /*  init struct foc C++ code generation */
  /* compute path */
  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  loop_ub = solution->p->size[1];
  for (k = 0; k < loop_ub; k++) {
    Fleg_max_data[k] = solution->p->data[3 * k];
  }
  diff(b_Fleg_max, c_Fleg_max);
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  loop_ub = solution->p->size[1];
  for (k = 0; k < loop_ub; k++) {
    Fleg_max_data[k] = solution->p->data[3 * k + 1];
  }
  diff(b_Fleg_max, d_Fleg_max);
  c_Fleg_max_data = d_Fleg_max->data;
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  loop_ub = solution->p->size[1];
  for (k = 0; k < loop_ub; k++) {
    Fleg_max_data[k] = solution->p->data[3 * k + 2];
  }
  diff(b_Fleg_max, b_states);
  b_states_data = b_states->data;
  emxFree_real_T(&b_Fleg_max);
  if (c_Fleg_max->size[1] == 1) {
    nx = d_Fleg_max->size[1];
  } else {
    nx = c_Fleg_max->size[1];
  }
  if ((c_Fleg_max->size[1] == d_Fleg_max->size[1]) &&
      (nx == b_states->size[1])) {
    loop_ub = c_Fleg_max->size[1] - 1;
    k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
    c_Fleg_max->size[0] = 1;
    emxEnsureCapacity_real_T(c_Fleg_max, k);
    b_Fleg_max_data = c_Fleg_max->data;
    for (k = 0; k <= loop_ub; k++) {
      l2_tmp = b_Fleg_max_data[k];
      b_expl_temp = c_Fleg_max_data[k];
      scale = b_states_data[k];
      b_Fleg_max_data[k] =
          (l2_tmp * l2_tmp + b_expl_temp * b_expl_temp) + scale * scale;
    }
  } else {
    binary_expand_op(c_Fleg_max, d_Fleg_max, b_states);
    b_Fleg_max_data = c_Fleg_max->data;
  }
  emxFree_real_T(&b_states);
  emxFree_real_T(&d_Fleg_max);
  nx = c_Fleg_max->size[1];
  loop_ub = (c_Fleg_max->size[1] / 2) << 1;
  vectorUB = loop_ub - 2;
  for (k = 0; k <= vectorUB; k += 2) {
    __m128d r;
    r = _mm_loadu_pd(&b_Fleg_max_data[k]);
    _mm_storeu_pd(&b_Fleg_max_data[k], _mm_sqrt_pd(r));
  }
  for (k = loop_ub; k < nx; k++) {
    b_Fleg_max_data[k] = muDoubleScalarSqrt(b_Fleg_max_data[k]);
  }
  /*  check length is always l */
  /*      a = vecnorm(p) */
  /*      a -  ones(1,length(a))*l */
  k = solution->Ekin->size[0] * solution->Ekin->size[1];
  solution->Ekin->size[0] = 1;
  solution->Ekin->size[1] = solution->time->size[1];
  emxEnsureCapacity_real_T(solution->Ekin, k);
  loop_ub = solution->time->size[1];
  for (k = 0; k < loop_ub; k++) {
    solution->Ekin->data[k] = 0.0;
  }
  c_expl_temp = 0.0;
  /*  kinetic energy at the beginning */
  l1_tmp = params->m / 2.0;
  k = solution->time->size[1];
  for (loop_ub = 0; loop_ub < k; loop_ub++) {
    l2_tmp = c_states_data[3 * loop_ub];
    b_expl_temp = l1_tmp * l2_tmp * l2_tmp;
    l2_tmp = c_states_data[3 * loop_ub + 1];
    b_expl_temp += l1_tmp * l2_tmp * l2_tmp;
    l2_tmp = c_states_data[3 * loop_ub + 2];
    b_expl_temp += l1_tmp * l2_tmp * l2_tmp;
    solution->Ekin->data[loop_ub] = b_expl_temp;
    c_expl_temp += b_expl_temp * 0.001;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  solution->path_length = sum(c_Fleg_max);
  emxFree_real_T(&c_Fleg_max);
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(solution->p->data[0] - p0[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_expl_temp = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_expl_temp = t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[3 * (solution->p->size[1] - 1)] -
                            pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    l2_tmp = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    l2_tmp = t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[1] - p0[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_expl_temp = b_expl_temp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_expl_temp += t * t;
  }
  absxk = muDoubleScalarAbs(
      solution->p->data[3 * (solution->p->size[1] - 1) + 1] - pf[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    l2_tmp = l2_tmp * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    l2_tmp += t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[2] - p0[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_expl_temp = b_expl_temp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_expl_temp += t * t;
  }
  absxk = muDoubleScalarAbs(
      solution->p->data[3 * (solution->p->size[1] - 1) + 2] - pf[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    l2_tmp = l2_tmp * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    l2_tmp += t * t;
  }
  solution->initial_error = scale * muDoubleScalarSqrt(b_expl_temp);
  solution->final_error_real = b_scale * muDoubleScalarSqrt(l2_tmp);
  k = solution->psi->size[0] * solution->psi->size[1];
  solution->psi->size[0] = 1;
  solution->psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psi, k);
  loop_ub = states->size[1];
  k = solution->l1->size[0] * solution->l1->size[1];
  solution->l1->size[0] = 1;
  solution->l1->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1, k);
  k = solution->l2->size[0] * solution->l2->size[1];
  solution->l2->size[0] = 1;
  solution->l2->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2, k);
  k = solution->psid->size[0] * solution->psid->size[1];
  solution->psid->size[0] = 1;
  solution->psid->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psid, k);
  k = solution->l1d->size[0] * solution->l1d->size[1];
  solution->l1d->size[0] = 1;
  solution->l1d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1d, k);
  k = solution->l2d->size[0] * solution->l2d->size[1];
  solution->l2d->size[0] = 1;
  solution->l2d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2d, k);
  for (k = 0; k < loop_ub; k++) {
    solution->psi->data[k] = states_data[6 * k];
    solution->l1->data[k] = states_data[6 * k + 1];
    solution->l2->data[k] = states_data[6 * k + 2];
    solution->psid->data[k] = states_data[6 * k + 3];
    solution->l1d->data[k] = states_data[6 * k + 4];
    solution->l2d->data[k] = states_data[6 * k + 5];
  }
  emxFree_real_T(&states);
  solution->Tf = x_data[3];
  emxFree_real_T(&b_x);
  solution->Etot = 0.0;
  b_scale = l1_tmp * c_states_data[0] * c_states_data[0];
  solution->Ekin0x = b_scale;
  absxk = l1_tmp * c_states_data[1] * c_states_data[1];
  solution->Ekin0y = absxk;
  t = l1_tmp * c_states_data[2] * c_states_data[2];
  solution->Ekin0z = t;
  solution->intEkin = c_expl_temp;
  solution->U0 = 0.0;
  solution->Uf = 0.0;
  l2_tmp = c_states_data[3 * (pd->size[1] - 1)];
  l2_tmp *= l1_tmp * l2_tmp;
  solution->Ekinfx = l2_tmp;
  b_expl_temp = c_states_data[3 * (pd->size[1] - 1) + 1];
  b_expl_temp *= l1_tmp * b_expl_temp;
  solution->Ekinfy = b_expl_temp;
  scale = c_states_data[3 * (pd->size[1] - 1) + 2];
  emxFree_real_T(&pd);
  scale *= l1_tmp * scale;
  solution->Ekinfz = scale;
  solution->achieved_target[0] =
      solution->p->data[3 * (solution->p->size[1] - 1)];
  solution->achieved_target[1] =
      solution->p->data[3 * (solution->p->size[1] - 1) + 1];
  solution->achieved_target[2] =
      solution->p->data[3 * (solution->p->size[1] - 1) + 2];
  solution->Ekin0 = (b_scale + absxk) + t;
  solution->Ekinf = (l2_tmp + b_expl_temp) + scale;
  solution->T_th = params->T_th;
  if ((EXITFLAG == 1.0) || (EXITFLAG == 2.0)) {
    *problem_solved = true;
  } else {
    *problem_solved = false;
  }
  /* save('test_matlab2.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf');
   */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

real_T optimize_cpp_anonFcn1(const real_T p0[3], real_T params_m,
                             int64_T params_num_params,
                             const char_T params_int_method[3],
                             real_T params_N_dyn, real_T params_int_steps,
                             real_T params_b, const real_T params_p_a1[3],
                             const real_T params_p_a2[3], real_T params_g,
                             real_T params_w4, real_T params_T_th,
                             const emxArray_real_T *x)
{
  emxArray_real_T *Fr_l;
  emxArray_real_T *Fr_r;
  emxArray_real_T *b_states;
  emxArray_real_T *b_t;
  emxArray_real_T *c_states;
  emxArray_real_T *d_states;
  emxArray_real_T *e_states;
  emxArray_real_T *f_states;
  emxArray_real_T *p;
  emxArray_real_T *pd;
  emxArray_real_T *states;
  int64_T i;
  int64_T i4;
  real_T dv[6];
  real_T b_x[3];
  const real_T *x_data;
  real_T absxk;
  real_T b_scale;
  real_T b_y;
  real_T scale;
  real_T t;
  real_T varargout_1;
  real_T y;
  real_T *Fr_l_data;
  real_T *b_states_data;
  real_T *c_states_data;
  real_T *d_states_data;
  real_T *e_states_data;
  real_T *states_data;
  real_T *t_data;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  i = plus(params_num_params, params_N_dyn);
  if (params_num_params + 1L > i) {
    i1 = 0;
    i2 = 0;
  } else {
    i1 = (int32_T)(params_num_params + 1L) - 1;
    i2 = (int32_T)i;
  }
  emxInit_real_T(&Fr_l, 2);
  i3 = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  loop_ub = i2 - i1;
  Fr_l->size[1] = loop_ub;
  emxEnsureCapacity_real_T(Fr_l, i3);
  Fr_l_data = Fr_l->data;
  for (i2 = 0; i2 < loop_ub; i2++) {
    Fr_l_data[i2] = x_data[i1 + i2];
  }
  i4 = plus(params_num_params, 2.0 * params_N_dyn);
  if (i + 1L > i4) {
    i1 = 0;
    i2 = 0;
  } else {
    i1 = (int32_T)(i + 1L) - 1;
    i2 = (int32_T)i4;
  }
  emxInit_real_T(&Fr_r, 2);
  i3 = Fr_r->size[0] * Fr_r->size[1];
  Fr_r->size[0] = 1;
  loop_ub = i2 - i1;
  Fr_r->size[1] = loop_ub;
  emxEnsureCapacity_real_T(Fr_r, i3);
  Fr_l_data = Fr_r->data;
  for (i2 = 0; i2 < loop_ub; i2++) {
    Fr_l_data[i2] = x_data[i1 + i2];
  }
  /*  check they are column vectors */
  /*  variable intergration step */
  /*  single shooting */
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p0[0] - params_p_a1[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[0] - params_p_a2[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params_p_a1[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params_p_a2[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params_p_a1[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params_p_a2[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = y;
  dv[2] = b_y;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  b_x[0] = x_data[0];
  b_x[1] = x_data[1];
  b_x[2] = x_data[2];
  emxInit_real_T(&states, 2);
  emxInit_real_T(&b_t, 2);
  computeRollout(dv, x_data[3] / (params_N_dyn - 1.0), params_N_dyn, Fr_l, Fr_r,
                 b_x, params_int_method, params_int_steps, params_m, params_b,
                 params_p_a1, params_p_a2, params_g, params_T_th, states, b_t);
  Fr_l_data = states->data;
  i1 = b_t->size[0] * b_t->size[1];
  b_t->size[0] = 1;
  b_t->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_t, i1);
  t_data = b_t->data;
  loop_ub = states->size[1];
  emxInit_real_T(&b_states, 2);
  i1 = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, i1);
  states_data = b_states->data;
  emxInit_real_T(&c_states, 2);
  i1 = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, i1);
  b_states_data = c_states->data;
  emxInit_real_T(&d_states, 2);
  i1 = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, i1);
  c_states_data = d_states->data;
  emxInit_real_T(&e_states, 2);
  i1 = e_states->size[0] * e_states->size[1];
  e_states->size[0] = 1;
  e_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(e_states, i1);
  d_states_data = e_states->data;
  emxInit_real_T(&f_states, 2);
  i1 = f_states->size[0] * f_states->size[1];
  f_states->size[0] = 1;
  f_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(f_states, i1);
  e_states_data = f_states->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    t_data[i1] = Fr_l_data[6 * i1];
    states_data[i1] = Fr_l_data[6 * i1 + 1];
    b_states_data[i1] = Fr_l_data[6 * i1 + 2];
    c_states_data[i1] = Fr_l_data[6 * i1 + 3];
    d_states_data[i1] = Fr_l_data[6 * i1 + 4];
    e_states_data[i1] = Fr_l_data[6 * i1 + 5];
  }
  emxFree_real_T(&states);
  emxInit_real_T(&p, 2);
  emxInit_real_T(&pd, 2);
  computePositionVelocity(params_b, b_t, b_states, c_states, d_states, e_states,
                          f_states, p, pd);
  emxFree_real_T(&f_states);
  emxFree_real_T(&e_states);
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  emxFree_real_T(&pd);
  emxFree_real_T(&p);
  /*  be careful there are only N values in this vector the path migh be */
  /*  underestimated! */
  /*      deltax = diff(p(1,:));  % diff(X); */
  /*      deltay = diff(p(2,:));   % diff(Y); */
  /*      deltaz = diff(p(3,:));    % diff(Z); */
  /*      path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2)); */
  /* minimize the final kin energy at contact */
  /* Ekinfcost=  m/2*contact_normal'*pd(:,end)'*pd(:,end)*contact_normal; */
  /*  minimize hoist work / energy consumption */
  /* assume the motor is not regenreating */
  /*  smoothnes: minimize jerky control action */
  /* fprintf("hoist_work %f\n ",hoist_work)     */
  /* fprintf("smooth %f\n ", smooth) */
  /* fprintf("tempo %f\n ", w6*Tf) */
  diff(Fr_r, b_states);
  emxFree_real_T(&Fr_r);
  diff(Fr_l, b_t);
  emxFree_real_T(&Fr_l);
  varargout_1 = params_w4 * (sum(b_states) + sum(b_t));
  emxFree_real_T(&b_states);
  emxFree_real_T(&b_t);
  /*  + w6*Tf;% */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

void optimize_cpp_anonFcn2(const real_T p0[3], const real_T pf[3],
                           real_T Fleg_max, real_T mu, const param *params,
                           const emxArray_real_T *x,
                           emxArray_real_T *varargout_1)
{
  emxArray_real_T *p;
  emxArray_real_T *px;
  emxArray_real_T *py;
  emxArray_real_T *pz;
  emxArray_real_T *states;
  int64_T i;
  int64_T i3;
  real_T dv[6];
  real_T Fleg[3];
  const real_T *x_data;
  real_T absxk;
  real_T b_params;
  real_T b_scale;
  real_T b_y;
  real_T c_idx_0;
  real_T c_idx_1;
  real_T c_idx_2;
  real_T scale;
  real_T t;
  real_T y;
  real_T *p_data;
  real_T *px_data;
  real_T *py_data;
  real_T *pz_data;
  real_T *states_data;
  int32_T b_i;
  int32_T i1;
  int32_T i2;
  int32_T i4;
  int32_T i5;
  int32_T loop_ub;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /*  ineq are <= 0 */
  Fleg[0] = x_data[0];
  Fleg[1] = x_data[1];
  Fleg[2] = x_data[2];
  i = plus(params->num_params, params->N_dyn);
  if (params->num_params + 1L > i) {
    i1 = 0;
    i2 = 0;
  } else {
    i1 = (int32_T)(params->num_params + 1L) - 1;
    i2 = (int32_T)i;
  }
  i3 = plus(params->num_params, 2.0 * params->N_dyn);
  if (i + 1L > i3) {
    i4 = 0;
    b_i = 0;
  } else {
    i4 = (int32_T)(i + 1L) - 1;
    b_i = (int32_T)i3;
  }
  /*  check they are column vectors */
  /*  size not known */
  varargout_1->size[0] = 1;
  varargout_1->size[1] = 0;
  /*  number of constraints */
  /*  already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes
   */
  /*  variable intergration step */
  /*  single shooting */
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p0[0] - params->p_a1[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[0] - params->p_a2[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params->p_a1[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params->p_a2[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params->p_a1[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params->p_a2[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = y;
  dv[2] = b_y;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  emxInit_real_T(&py, 2);
  i5 = py->size[0] * py->size[1];
  py->size[0] = 1;
  loop_ub = i2 - i1;
  py->size[1] = loop_ub;
  emxEnsureCapacity_real_T(py, i5);
  py_data = py->data;
  for (i2 = 0; i2 < loop_ub; i2++) {
    py_data[i2] = x_data[i1 + i2];
  }
  emxInit_real_T(&pz, 2);
  i1 = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  loop_ub = b_i - i4;
  pz->size[1] = loop_ub;
  emxEnsureCapacity_real_T(pz, i1);
  pz_data = pz->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    pz_data[i1] = x_data[i4 + i1];
  }
  emxInit_real_T(&states, 2);
  emxInit_real_T(&px, 2);
  computeRollout(dv, x_data[3] / (params->N_dyn - 1.0), params->N_dyn, py, pz,
                 Fleg, params->int_method, params->int_steps, params->m,
                 params->b, params->p_a1, params->p_a2, params->g, params->T_th,
                 states, px);
  states_data = states->data;
  i1 = px->size[0] * px->size[1];
  px->size[0] = 1;
  px->size[1] = states->size[1];
  emxEnsureCapacity_real_T(px, i1);
  px_data = px->data;
  loop_ub = states->size[1];
  i1 = py->size[0] * py->size[1];
  py->size[0] = 1;
  py->size[1] = states->size[1];
  emxEnsureCapacity_real_T(py, i1);
  py_data = py->data;
  i1 = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  pz->size[1] = states->size[1];
  emxEnsureCapacity_real_T(pz, i1);
  pz_data = pz->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    px_data[i1] = 0.0;
    py_data[i1] = 0.0;
    pz_data[i1] = 0.0;
  }
  i1 = states->size[1];
  for (b_i = 0; b_i < i1; b_i++) {
    absxk = states_data[6 * b_i];
    t = states_data[6 * b_i + 1];
    b_y = states_data[6 * b_i + 2];
    b_scale = params->b * params->b;
    scale = t * t;
    b_y = (b_scale + scale) - b_y * b_y;
    b_scale = muDoubleScalarSqrt(1.0 - b_y * b_y / (4.0 * b_scale * scale));
    px_data[b_i] = t * muDoubleScalarSin(absxk) * b_scale;
    py_data[b_i] = b_y / (2.0 * params->b);
    pz_data[b_i] = -t * muDoubleScalarCos(absxk) * b_scale;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  emxFree_real_T(&states);
  emxInit_real_T(&p, 2);
  i1 = p->size[0] * p->size[1];
  p->size[0] = 3;
  p->size[1] = px->size[1];
  emxEnsureCapacity_real_T(p, i1);
  p_data = p->data;
  loop_ub = px->size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    p_data[3 * i1] = px_data[i1];
    p_data[3 * i1 + 1] = py_data[i1];
    p_data[3 * i1 + 2] = pz_data[i1];
  }
  emxFree_real_T(&pz);
  emxFree_real_T(&py);
  emxFree_real_T(&px);
  /* only position */
  /*  I assume px py pz  are row vectors */
  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0  */
  if (params->obstacle_avoidance) {
    /* px > sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2); */
    /* -px + sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2)<0 */
    i1 = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i1; b_i++) {
      b_scale = p_data[3 * b_i + 2] - -4.5;
      b_y = p_data[3 * b_i + 1] - 3.0;
      /* %%add ineq only if inside sphere */
      i2 = varargout_1->size[1];
      i4 = varargout_1->size[0] * varargout_1->size[1];
      varargout_1->size[0] = 1;
      varargout_1->size[1]++;
      emxEnsureCapacity_real_T(varargout_1, i4);
      states_data = varargout_1->data;
      states_data[i2] =
          -p_data[3 * b_i] +
          muDoubleScalarSqrt((2.25 - 3.0 * (b_scale * b_scale)) - b_y * b_y);
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    i1 = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i1; b_i++) {
      i2 = varargout_1->size[1];
      i4 = varargout_1->size[0] * varargout_1->size[1];
      varargout_1->size[1]++;
      emxEnsureCapacity_real_T(varargout_1, i4);
      states_data = varargout_1->data;
      states_data[i2] = -p_data[3 * b_i];
      /* ineq = [ineq -psi(i) ];  */
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  }
  /*  % % debug */
  /*  disp('after wall') */
  /*  length(ineq) */
  /*  2- N_dyn constraints on retraction force   -Fr_max < Fr < 0  */
  /*  unilaterality */
  /*  debug */
  /*  disp('after Fr') */
  /*  length(ineq) */
  /*  constraints on impulse force */
  b_scale = params->contact_normal[1] * 0.0 - params->contact_normal[2];
  b_y = params->contact_normal[2] * 0.0 - params->contact_normal[0] * 0.0;
  scale = params->contact_normal[0] - params->contact_normal[1] * 0.0;
  c_idx_0 = params->contact_normal[1] - params->contact_normal[2] * 0.0;
  c_idx_1 = params->contact_normal[2] * 0.0 - params->contact_normal[0];
  c_idx_2 = params->contact_normal[0] * 0.0 - params->contact_normal[1] * 0.0;
  /*  compute components */
  b_params = (params->contact_normal[0] * Fleg[0] +
              params->contact_normal[1] * Fleg[1]) +
             params->contact_normal[2] * Fleg[2];
  b_y =
      ((b_y * params->contact_normal[2] - params->contact_normal[1] * scale) *
           Fleg[0] +
       (params->contact_normal[0] * scale -
        b_scale * params->contact_normal[2]) *
           Fleg[1]) +
      (b_scale * params->contact_normal[1] - params->contact_normal[0] * b_y) *
          Fleg[2];
  /* 3 ------------------------------ Fleg constraints */
  /*  unilateral */
  i1 = varargout_1->size[1];
  i2 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i2);
  states_data = varargout_1->data;
  states_data[i1] = -b_params;
  /* (Fun >fmin )  */
  /* max force  */
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(Fleg[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(Fleg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  b_scale = ((c_idx_1 * params->contact_normal[2] -
              params->contact_normal[1] * c_idx_2) *
                 Fleg[0] +
             (params->contact_normal[0] * c_idx_2 -
              c_idx_0 * params->contact_normal[2]) *
                 Fleg[1]) +
            (c_idx_0 * params->contact_normal[1] -
             params->contact_normal[0] * c_idx_1) *
                Fleg[2];
  absxk = muDoubleScalarAbs(Fleg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  i1 = varargout_1->size[1];
  i2 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i2);
  states_data = varargout_1->data;
  states_data[i1] = y - Fleg_max;
  /* (Fun < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    i1 = varargout_1->size[1];
    i2 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = 1;
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i2);
    states_data = varargout_1->data;
    states_data[i1] =
        muDoubleScalarSqrt(b_y * b_y + b_scale * b_scale) - mu * b_params;
    /* friction constraints */
  }
  /*   */
  /*  % debug */
  /*  disp('after Fu') */
  /*  length(ineq) */
  /*  final point  variable slack   */
  /* ineq= [ineq norm(p_f - pf) - x(num_params+N+N_dyn+1)]; */
  /*  4- initial final point  fixed slack  */
  /* *norm(p0 - pf);  */
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p_data[3 * (p->size[1] - 1)] - pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(p_data[3 * (p->size[1] - 1) + 1] - pf[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p_data[3 * (p->size[1] - 1) + 2] - pf[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  i1 = varargout_1->size[1];
  i2 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i2);
  states_data = varargout_1->data;
  states_data[i1] = y - 0.02;
  /* 5 - jump clearance */
  if (!params->obstacle_avoidance) {
    i1 = varargout_1->size[1];
    i2 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i2);
    states_data = varargout_1->data;
    states_data[i1] = -p_data[3 * ((int32_T)(params->N_dyn / 2.0) - 1)] +
                      params->jump_clearance;
  }
  emxFree_real_T(&p);
  /*  if any(isinf(ineq)) */
  /*      disp('Infn in constraint') */
  /*      find(isinf(ineq))  */
  /*      isinf(ineq) */
  /*  end */
  /*  if any(isnan(ineq)) */
  /*      disp('Nan in constraint') */
  /*      find(isnan(ineq)) */
  /*      isnan(ineq) */
  /*  end */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp.c) */
