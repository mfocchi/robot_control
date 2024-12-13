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
#include "cost.h"
#include "diff.h"
#include "eval_solution.h"
#include "fmincon.h"
#include "integrate_dynamics.h"
#include "norm.h"
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
                  struct0_T *solution)
{
  __m128d r;
  emxArray_real_T *b_Fleg_max;
  emxArray_real_T *b_states;
  emxArray_real_T *c_Fleg_max;
  emxArray_real_T *c_states;
  emxArray_real_T *d_Fleg_max;
  emxArray_real_T *d_states;
  emxArray_real_T *pd_fine;
  emxArray_real_T *states;
  emxArray_real_T *states_rough;
  emxArray_real_T *x;
  real_T state0[6];
  real_T Fleg[3];
  real_T b_p0[3];
  real_T this_workspace_pf[3];
  real_T a_y;
  real_T a_z;
  real_T arg;
  real_T d;
  real_T d1;
  real_T d2;
  real_T l1_tmp;
  real_T n_samples;
  real_T solution_tmp;
  real_T state0_tmp;
  real_T t_;
  real_T this_workspace_pf_tmp;
  real_T y_tmp;
  real_T *Fleg_max_data;
  real_T *b_Fleg_max_data;
  real_T *b_states_data;
  real_T *c_Fleg_max_data;
  real_T *c_states_data;
  real_T *d_states_data;
  real_T *states_data;
  real_T *states_rough_data;
  real_T *x_data;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T k;
  int32_T loop_ub;
  int32_T loop_ub_tmp;
  int32_T nx;
  int32_T vectorUB;
  uint32_T rough_count;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /* make sure is column vector */
  /* for eval solution */
  /*  needs to be fixed for code generation */
  /*  only to evaluate solution */
  /* compute initial state from jump param */
  _mm_storeu_pd(&b_p0[0], _mm_sub_pd(_mm_loadu_pd(&p0[0]),
                                     _mm_loadu_pd(&params->p_a1[0])));
  arg = p0[2] - params->p_a1[2];
  b_p0[2] = arg;
  l1_tmp = b_norm(b_p0);
  /* pendulum period */
  /*  half period TODO replace with linearized x0(2) = l10 */
  /* opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r */
  /*  % does not always satisfy bounds */
  tic();
  b_p0[0] = p0[0];
  this_workspace_pf[0] = pf[0];
  b_p0[1] = p0[1];
  this_workspace_pf[1] = pf[1];
  b_p0[2] = p0[2];
  this_workspace_pf[2] = pf[2];
  emxInit_real_T(&b_Fleg_max, 2);
  i = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  k = ((int32_T)params->N_dyn + (int32_T)params->N_dyn) + 4;
  b_Fleg_max->size[1] = k;
  emxEnsureCapacity_real_T(b_Fleg_max, i);
  Fleg_max_data = b_Fleg_max->data;
  Fleg_max_data[0] = Fleg_max;
  Fleg_max_data[1] = Fleg_max;
  Fleg_max_data[2] = Fleg_max;
  Fleg_max_data[3] =
      6.2831853071795862 * muDoubleScalarSqrt(l1_tmp / params->g) / 4.0;
  loop_ub_tmp = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub_tmp; i++) {
    Fleg_max_data[i + 4] = 0.0;
  }
  for (i = 0; i < loop_ub_tmp; i++) {
    Fleg_max_data[(i + (int32_T)params->N_dyn) + 4] = 0.0;
  }
  emxInit_real_T(&c_Fleg_max, 2);
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = k;
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  b_Fleg_max_data = c_Fleg_max->data;
  b_Fleg_max_data[0] = -Fleg_max;
  b_Fleg_max_data[1] = -Fleg_max;
  b_Fleg_max_data[2] = -Fleg_max;
  b_Fleg_max_data[3] = 0.01;
  for (i = 0; i < loop_ub_tmp; i++) {
    b_Fleg_max_data[i + 4] = -Fr_max;
  }
  for (i = 0; i < loop_ub_tmp; i++) {
    b_Fleg_max_data[(i + (int32_T)params->N_dyn) + 4] = -Fr_max;
  }
  emxInit_real_T(&d_Fleg_max, 2);
  i = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = k;
  emxEnsureCapacity_real_T(d_Fleg_max, i);
  c_Fleg_max_data = d_Fleg_max->data;
  c_Fleg_max_data[0] = Fleg_max;
  c_Fleg_max_data[1] = Fleg_max;
  c_Fleg_max_data[2] = Fleg_max;
  c_Fleg_max_data[3] = rtInf;
  for (i = 0; i < loop_ub_tmp; i++) {
    c_Fleg_max_data[i + 4] = 0.0;
  }
  for (i = 0; i < loop_ub_tmp; i++) {
    c_Fleg_max_data[(i + (int32_T)params->N_dyn) + 4] = 0.0;
  }
  emxInit_real_T(&x, 2);
  solution->cost = fmincon(
      p0, params, b_Fleg_max, c_Fleg_max, d_Fleg_max, b_p0, this_workspace_pf,
      Fleg_max, mu, params, x, solution->optim_output.algorithm,
      &solution->problem_solved, &solution->optim_output.iterations,
      &solution->optim_output.funcCount,
      &solution->optim_output.constrviolation, &solution->optim_output.stepsize,
      &solution->optim_output.lssteplength,
      &solution->optim_output.firstorderopt);
  x_data = x->data;
  toc();
  /* eval trajectory */
  Fleg[0] = x_data[0];
  Fleg[1] = x_data[1];
  Fleg[2] = x_data[2];
  d = params->num_params + params->N_dyn;
  if (params->num_params + 1.0 > d) {
    i = 0;
    k = 0;
  } else {
    i = (int32_T)(params->num_params + 1.0) - 1;
    k = (int32_T)d;
  }
  d1 = params->num_params + 2.0 * params->N_dyn;
  if (d + 1.0 > d1) {
    i1 = 0;
    loop_ub = 0;
  } else {
    i1 = (int32_T)(d + 1.0) - 1;
    loop_ub = (int32_T)d1;
  }
  /*  resample inputs  */
  n_samples = muDoubleScalarFloor(x_data[3] / 0.001);
  b_i = solution->Fr_l_fine->size[0] * solution->Fr_l_fine->size[1];
  solution->Fr_l_fine->size[0] = 1;
  nx = (int32_T)n_samples;
  solution->Fr_l_fine->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(solution->Fr_l_fine, b_i);
  b_i = solution->Fr_r_fine->size[0] * solution->Fr_r_fine->size[1];
  solution->Fr_r_fine->size[0] = 1;
  solution->Fr_r_fine->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(solution->Fr_r_fine, b_i);
  rough_count = 1U;
  t_ = 0.0;
  for (b_i = 0; b_i < nx; b_i++) {
    t_ += 0.001;
    if (t_ >= n_samples * 0.001 / (params->N_dyn - 1.0)) {
      rough_count++;
      t_ = 0.0;
    }
    solution->Fr_l_fine->data[b_i] = x_data[(i + (int32_T)rough_count) - 1];
    solution->Fr_r_fine->data[b_i] = x_data[(i1 + (int32_T)rough_count) - 1];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /*  single shooting */
  r = _mm_loadu_pd(&p0[0]);
  _mm_storeu_pd(&b_p0[0], _mm_sub_pd(r, _mm_loadu_pd(&params->p_a1[0])));
  _mm_storeu_pd(&this_workspace_pf[0],
                _mm_sub_pd(r, _mm_loadu_pd(&params->p_a2[0])));
  b_p0[2] = arg;
  this_workspace_pf_tmp = p0[2] - params->p_a2[2];
  this_workspace_pf[2] = this_workspace_pf_tmp;
  state0_tmp = muDoubleScalarAtan2(p0[0], -p0[2]);
  state0[0] = state0_tmp;
  state0[1] = b_norm(b_p0);
  state0[2] = b_norm(this_workspace_pf);
  state0[3] = 0.0;
  state0[4] = 0.0;
  state0[5] = 0.0;
  /*  course integration */
  b_i = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_loop_ub = k - i;
  b_Fleg_max->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(b_Fleg_max, b_i);
  Fleg_max_data = b_Fleg_max->data;
  for (k = 0; k < b_loop_ub; k++) {
    Fleg_max_data[k] = x_data[i + k];
  }
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  loop_ub -= i1;
  c_Fleg_max->size[1] = loop_ub;
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  for (k = 0; k < loop_ub; k++) {
    b_Fleg_max_data[k] = x_data[i1 + k];
  }
  d2 = x_data[3] / (params->N_dyn - 1.0);
  emxInit_real_T(&states, 2);
  computeRollout(state0, d2, params->N_dyn, b_Fleg_max, c_Fleg_max, Fleg,
                 params->int_method, params->int_steps, params->m, params->b,
                 params->p_a1, params->p_a2, params->g, params->T_th, states,
                 solution->time);
  states_data = states->data;
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  nx = states->size[1];
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
  for (k = 0; k < nx; k++) {
    Fleg_max_data[k] = states_data[6 * k];
    b_Fleg_max_data[k] = states_data[6 * k + 1];
    c_Fleg_max_data[k] = states_data[6 * k + 2];
    b_states_data[k] = states_data[6 * k + 3];
    c_states_data[k] = states_data[6 * k + 4];
    d_states_data[k] = states_data[6 * k + 5];
  }
  b_computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
                            b_states, c_states, d_states,
                            solution->solution_constr.p, solution->p_fine);
  /*  fine integration  */
  /* init */
  emxInit_real_T(&states_rough, 2);
  integrate_dynamics(state0, 0.001, n_samples, solution->Fr_l_fine,
                     solution->Fr_r_fine, Fleg, params->int_method, params->m,
                     params->b, params->p_a1, params->p_a2, params->g,
                     params->T_th, states_rough, solution->time_fine);
  states_rough_data = states_rough->data;
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  nx = states_rough->size[1];
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  k = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(d_Fleg_max, k);
  c_Fleg_max_data = d_Fleg_max->data;
  k = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(b_states, k);
  b_states_data = b_states->data;
  k = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(c_states, k);
  c_states_data = c_states->data;
  k = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(d_states, k);
  d_states_data = d_states->data;
  for (k = 0; k < nx; k++) {
    Fleg_max_data[k] = states_rough_data[6 * k];
    b_Fleg_max_data[k] = states_rough_data[6 * k + 1];
    c_Fleg_max_data[k] = states_rough_data[6 * k + 2];
    b_states_data[k] = states_rough_data[6 * k + 3];
    c_states_data[k] = states_rough_data[6 * k + 4];
    d_states_data[k] = states_rough_data[6 * k + 5];
  }
  emxInit_real_T(&pd_fine, 2);
  b_computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
                            b_states, c_states, d_states, solution->p_fine,
                            pd_fine);
  b_states_data = pd_fine->data;
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  emxFree_real_T(&b_states);
  /*  init struct foc C++ code generation */
  /* compute path */
  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  nx = solution->solution_constr.p->size[1];
  for (k = 0; k < nx; k++) {
    b_Fleg_max_data[k] = solution->solution_constr.p->data[3 * k];
  }
  diff(c_Fleg_max, solution->c);
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  nx = solution->solution_constr.p->size[1];
  for (k = 0; k < nx; k++) {
    b_Fleg_max_data[k] = solution->solution_constr.p->data[3 * k + 1];
  }
  diff(c_Fleg_max, b_Fleg_max);
  Fleg_max_data = b_Fleg_max->data;
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  nx = solution->solution_constr.p->size[1];
  for (k = 0; k < nx; k++) {
    b_Fleg_max_data[k] = solution->solution_constr.p->data[3 * k + 2];
  }
  diff(c_Fleg_max, d_Fleg_max);
  c_Fleg_max_data = d_Fleg_max->data;
  if (solution->c->size[1] == 1) {
    nx = b_Fleg_max->size[1];
  } else {
    nx = solution->c->size[1];
  }
  if ((solution->c->size[1] == b_Fleg_max->size[1]) &&
      (nx == d_Fleg_max->size[1])) {
    nx = solution->c->size[1] - 1;
    k = solution->c->size[0] * solution->c->size[1];
    solution->c->size[0] = 1;
    emxEnsureCapacity_real_T(solution->c, k);
    for (k = 0; k <= nx; k++) {
      arg = solution->c->data[k];
      t_ = Fleg_max_data[k];
      a_y = c_Fleg_max_data[k];
      solution->c->data[k] = (arg * arg + t_ * t_) + a_y * a_y;
    }
  } else {
    binary_expand_op(solution, b_Fleg_max, d_Fleg_max);
  }
  nx = solution->c->size[1];
  b_i = (solution->c->size[1] / 2) << 1;
  vectorUB = b_i - 2;
  for (k = 0; k <= vectorUB; k += 2) {
    r = _mm_loadu_pd(&solution->c->data[k]);
    _mm_storeu_pd(&solution->c->data[k], _mm_sqrt_pd(r));
  }
  for (k = b_i; k < nx; k++) {
    solution->c->data[k] = muDoubleScalarSqrt(solution->c->data[k]);
  }
  /*  check length is always l */
  /*      a = vecnorm(p) */
  /*      a -  ones(1,length(a))*l */
  k = solution->Ekin->size[0] * solution->Ekin->size[1];
  solution->Ekin->size[0] = 1;
  solution->Ekin->size[1] = solution->time_fine->size[1];
  emxEnsureCapacity_real_T(solution->Ekin, k);
  nx = solution->time_fine->size[1];
  for (k = 0; k < nx; k++) {
    solution->Ekin->data[k] = 0.0;
  }
  a_y = 0.0;
  /*  kinetic energy at the beginning */
  y_tmp = params->m / 2.0;
  k = solution->time_fine->size[1];
  for (b_i = 0; b_i < k; b_i++) {
    arg = b_states_data[3 * b_i];
    t_ = y_tmp * arg * arg;
    arg = b_states_data[3 * b_i + 1];
    t_ += y_tmp * arg * arg;
    arg = b_states_data[3 * b_i + 2];
    t_ += y_tmp * arg * arg;
    solution->Ekin->data[b_i] = t_;
    a_y += t_ * 0.001;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  solution->path_length = sum(solution->c);
  r = _mm_loadu_pd(&solution->solution_constr.p->data[0]);
  _mm_storeu_pd(&b_p0[0], _mm_sub_pd(r, _mm_loadu_pd(&p0[0])));
  b_p0[2] = solution->solution_constr.p->data[2] - p0[2];
  solution->initial_error = b_norm(b_p0);
  nx = solution->solution_constr.p->size[1];
  r = _mm_loadu_pd(&solution->solution_constr.p
                        ->data[3 * (solution->solution_constr.p->size[1] - 1)]);
  _mm_storeu_pd(&b_p0[0], _mm_sub_pd(r, _mm_loadu_pd(&pf[0])));
  r = _mm_loadu_pd(&Fleg[0]);
  _mm_storeu_pd(&solution->Fleg[0], r);
  b_p0[2] = solution->solution_constr.p->data[3 * (nx - 1) + 2] - pf[2];
  solution->Fleg[2] = Fleg[2];
  solution->final_error_real = b_norm(b_p0);
  k = solution->Fr_l->size[0] * solution->Fr_l->size[1];
  solution->Fr_l->size[0] = 1;
  solution->Fr_l->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_l, k);
  for (k = 0; k < b_loop_ub; k++) {
    solution->Fr_l->data[k] = x_data[i + k];
  }
  i = solution->Fr_r->size[0] * solution->Fr_r->size[1];
  solution->Fr_r->size[0] = 1;
  solution->Fr_r->size[1] = loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_r, i);
  for (i = 0; i < loop_ub; i++) {
    solution->Fr_r->data[i] = x_data[i1 + i];
  }
  i = solution->p->size[0] * solution->p->size[1];
  solution->p->size[0] = 3;
  solution->p->size[1] = solution->solution_constr.p->size[1];
  emxEnsureCapacity_real_T(solution->p, i);
  b_loop_ub = 3 * solution->solution_constr.p->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    solution->p->data[i] = solution->solution_constr.p->data[i];
  }
  i = solution->psi->size[0] * solution->psi->size[1];
  solution->psi->size[0] = 1;
  solution->psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psi, i);
  b_loop_ub = states->size[1];
  i = solution->l1->size[0] * solution->l1->size[1];
  solution->l1->size[0] = 1;
  solution->l1->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1, i);
  i = solution->l2->size[0] * solution->l2->size[1];
  solution->l2->size[0] = 1;
  solution->l2->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2, i);
  i = solution->psid->size[0] * solution->psid->size[1];
  solution->psid->size[0] = 1;
  solution->psid->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psid, i);
  i = solution->l1d->size[0] * solution->l1d->size[1];
  solution->l1d->size[0] = 1;
  solution->l1d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1d, i);
  i = solution->l2d->size[0] * solution->l2d->size[1];
  solution->l2d->size[0] = 1;
  solution->l2d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2d, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->psi->data[i] = states_data[6 * i];
    solution->l1->data[i] = states_data[6 * i + 1];
    solution->l2->data[i] = states_data[6 * i + 2];
    solution->psid->data[i] = states_data[6 * i + 3];
    solution->l1d->data[i] = states_data[6 * i + 4];
    solution->l2d->data[i] = states_data[6 * i + 5];
  }
  i = solution->psi_fine->size[0] * solution->psi_fine->size[1];
  solution->psi_fine->size[0] = 1;
  solution->psi_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->psi_fine, i);
  b_loop_ub = states_rough->size[1];
  i = solution->l1_fine->size[0] * solution->l1_fine->size[1];
  solution->l1_fine->size[0] = 1;
  solution->l1_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l1_fine, i);
  i = solution->l2_fine->size[0] * solution->l2_fine->size[1];
  solution->l2_fine->size[0] = 1;
  solution->l2_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l2_fine, i);
  i = solution->psid_fine->size[0] * solution->psid_fine->size[1];
  solution->psid_fine->size[0] = 1;
  solution->psid_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->psid_fine, i);
  i = solution->l1d_fine->size[0] * solution->l1d_fine->size[1];
  solution->l1d_fine->size[0] = 1;
  solution->l1d_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l1d_fine, i);
  i = solution->l2d_fine->size[0] * solution->l2d_fine->size[1];
  solution->l2d_fine->size[0] = 1;
  solution->l2d_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->l2d_fine, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->psi_fine->data[i] = states_rough_data[6 * i];
    solution->l1_fine->data[i] = states_rough_data[6 * i + 1];
    solution->l2_fine->data[i] = states_rough_data[6 * i + 2];
    solution->psid_fine->data[i] = states_rough_data[6 * i + 3];
    solution->l1d_fine->data[i] = states_rough_data[6 * i + 4];
    solution->l2d_fine->data[i] = states_rough_data[6 * i + 5];
  }
  emxFree_real_T(&states_rough);
  solution->Tf = x_data[3];
  solution->Etot = 0.0;
  a_z = y_tmp * b_states_data[0] * b_states_data[0];
  solution->Ekin0x = a_z;
  solution_tmp = y_tmp * b_states_data[1] * b_states_data[1];
  solution->Ekin0y = solution_tmp;
  n_samples = y_tmp * b_states_data[2] * b_states_data[2];
  solution->Ekin0z = n_samples;
  solution->intEkin = a_y;
  solution->U0 = 0.0;
  solution->Uf = 0.0;
  arg = b_states_data[3 * (pd_fine->size[1] - 1)];
  arg *= y_tmp * arg;
  solution->Ekinfx = arg;
  t_ = b_states_data[3 * (pd_fine->size[1] - 1) + 1];
  t_ *= y_tmp * t_;
  solution->Ekinfy = t_;
  a_y = b_states_data[3 * (pd_fine->size[1] - 1) + 2];
  emxFree_real_T(&pd_fine);
  a_y *= y_tmp * a_y;
  solution->Ekinfz = a_y;
  solution->achieved_target[0] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1)];
  solution->achieved_target[1] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1) + 1];
  solution->achieved_target[2] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1) + 2];
  solution->Ekin0 = (a_z + solution_tmp) + n_samples;
  solution->Ekinf = (arg + t_) + a_y;
  solution->T_th = params->T_th;
  /* (EXITFLAG == 1) || (EXITFLAG == 2); */
  /*  1 First-order optimality measure was less than
   * options.OptimalityTolerance, and maximum constraint violation was less than
   * options.ConstraintTolerance. */
  /*  0 Number of iterations exceeded options.MaxIterations or number of
   * function evaluations exceeded options.MaxFunctionEvaluations. */
  /*  -1 Stopped by an output function or plot function. */
  /*  -2 No feasible point was found. */
  /*  2 Change in x was less than options.StepTolerance (Termination tolerance
   * on x, a scalar, the default is 1e-10) and maximum constraint violation was
   * less than options.ConstraintTolerance. */
  /*  evaluate constraint violation  */
  /*  ineq are <= 0 */
  Fleg[0] = x_data[0];
  Fleg[1] = x_data[1];
  Fleg[2] = x_data[2];
  if (params->num_params + 1.0 > d) {
    i = 0;
    k = 0;
  } else {
    i = (int32_T)(params->num_params + 1.0) - 1;
    k = (int32_T)d;
  }
  if (d + 1.0 > d1) {
    i1 = 0;
    loop_ub = 0;
  } else {
    i1 = (int32_T)(d + 1.0) - 1;
    loop_ub = (int32_T)d1;
  }
  /*  check they are column vectors */
  /*  size not known */
  solution->c->size[0] = 1;
  solution->c->size[1] = 0;
  /*  number of constraints */
  /*  already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes
   */
  if (params->FRICTION_CONE != 0.0) {
    nx = 3;
  } else {
    nx = 2;
    /* unilateral and actuation */
  }
  vectorUB = !params->obstacle_avoidance;
  /*  variable intergration step */
  /*  single shooting */
  _mm_storeu_pd(&b_p0[0], _mm_sub_pd(_mm_loadu_pd(&p0[0]),
                                     _mm_loadu_pd(&params->p_a2[0])));
  b_p0[2] = this_workspace_pf_tmp;
  state0[0] = state0_tmp;
  state0[1] = l1_tmp;
  state0[2] = b_norm(b_p0);
  state0[3] = 0.0;
  state0[4] = 0.0;
  state0[5] = 0.0;
  b_i = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_loop_ub = k - i;
  b_Fleg_max->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(b_Fleg_max, b_i);
  Fleg_max_data = b_Fleg_max->data;
  for (k = 0; k < b_loop_ub; k++) {
    Fleg_max_data[k] = x_data[i + k];
  }
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  b_loop_ub = loop_ub - i1;
  c_Fleg_max->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  b_Fleg_max_data = c_Fleg_max->data;
  for (i = 0; i < b_loop_ub; i++) {
    b_Fleg_max_data[i] = x_data[i1 + i];
  }
  emxFree_real_T(&x);
  computeRollout(state0, d2, params->N_dyn, b_Fleg_max, c_Fleg_max, Fleg,
                 params->int_method, params->int_steps, params->m, params->b,
                 params->p_a1, params->p_a2, params->g, params->T_th, states,
                 solution->solution_constr.time);
  states_data = states->data;
  i = solution->solution_constr.psid->size[0] *
      solution->solution_constr.psid->size[1];
  solution->solution_constr.psid->size[0] = 1;
  solution->solution_constr.psid->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.psid, i);
  b_loop_ub = states->size[1];
  i = solution->solution_constr.l1d->size[0] *
      solution->solution_constr.l1d->size[1];
  solution->solution_constr.l1d->size[0] = 1;
  solution->solution_constr.l1d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l1d, i);
  i = solution->solution_constr.l2d->size[0] *
      solution->solution_constr.l2d->size[1];
  solution->solution_constr.l2d->size[0] = 1;
  solution->solution_constr.l2d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l2d, i);
  i = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, i);
  Fleg_max_data = b_Fleg_max->data;
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  b_Fleg_max_data = c_Fleg_max->data;
  i = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_Fleg_max, i);
  c_Fleg_max_data = d_Fleg_max->data;
  for (i = 0; i < b_loop_ub; i++) {
    solution->solution_constr.psid->data[i] = states_data[6 * i + 3];
    solution->solution_constr.l1d->data[i] = states_data[6 * i + 4];
    solution->solution_constr.l2d->data[i] = states_data[6 * i + 5];
    Fleg_max_data[i] = states_data[6 * i];
    b_Fleg_max_data[i] = states_data[6 * i + 1];
    c_Fleg_max_data[i] = states_data[6 * i + 2];
  }
  computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
                          solution->solution_constr.p);
  emxFree_real_T(&d_Fleg_max);
  emxFree_real_T(&c_Fleg_max);
  emxFree_real_T(&b_Fleg_max);
  /* only position */
  /*  I assume px py pz  are row vectors */
  /*  init struct foc C++ code generation */
  i = solution->solution_constr.psi->size[0] *
      solution->solution_constr.psi->size[1];
  solution->solution_constr.psi->size[0] = 1;
  solution->solution_constr.psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.psi, i);
  b_loop_ub = states->size[1];
  i = solution->solution_constr.l1->size[0] *
      solution->solution_constr.l1->size[1];
  solution->solution_constr.l1->size[0] = 1;
  solution->solution_constr.l1->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l1, i);
  i = solution->solution_constr.l2->size[0] *
      solution->solution_constr.l2->size[1];
  solution->solution_constr.l2->size[0] = 1;
  solution->solution_constr.l2->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->solution_constr.l2, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->solution_constr.psi->data[i] = states_data[6 * i];
    solution->solution_constr.l1->data[i] = states_data[6 * i + 1];
    solution->solution_constr.l2->data[i] = states_data[6 * i + 2];
  }
  emxFree_real_T(&states);
  r = _mm_loadu_pd(&solution->solution_constr.p
                        ->data[3 * (solution->solution_constr.p->size[1] - 1)]);
  _mm_storeu_pd(&b_p0[0], _mm_sub_pd(r, _mm_loadu_pd(&pf[0])));
  b_p0[2] = solution->solution_constr.p
                ->data[3 * (solution->solution_constr.p->size[1] - 1) + 2] -
            pf[2];
  solution->solution_constr.final_error_discrete = b_norm(b_p0);
  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0  */
  if (params->obstacle_avoidance) {
    /* [0; 3;-7.5]; */
    n_samples = params->obstacle_size[0] * params->obstacle_size[0];
    a_y = n_samples / (params->obstacle_size[1] * params->obstacle_size[1]);
    a_z = n_samples / (params->obstacle_size[2] * params->obstacle_size[2]);
    /* px > sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2); */
    /* -px + sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2)<0 */
    /*  better implementaiton than the one with with complex numbers for code */
    /*  generation, we just check if the argument of the sqrt becomes */
    /*  negative */
    for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
      arg = solution->solution_constr.p->data[3 * b_i + 2] -
            params->obstacle_location[2];
      t_ = solution->solution_constr.p->data[3 * b_i + 1] -
           params->obstacle_location[1];
      arg = (n_samples - a_z * (arg * arg)) - a_y * (t_ * t_);
      /* %%add ineq only if inside sphere */
      if (arg > 0.0) {
        i = solution->c->size[1];
        k = solution->c->size[0] * solution->c->size[1];
        solution->c->size[0] = 1;
        solution->c->size[1]++;
        emxEnsureCapacity_real_T(solution->c, k);
        solution->c->data[i] = ((-solution->solution_constr.p->data[3 * b_i] +
                                 params->obstacle_location[0]) +
                                muDoubleScalarSqrt(arg)) +
                               params->jump_clearance;
      } else {
        i = solution->c->size[1];
        k = solution->c->size[0] * solution->c->size[1];
        solution->c->size[0] = 1;
        solution->c->size[1]++;
        emxEnsureCapacity_real_T(solution->c, k);
        solution->c->data[i] = -solution->solution_constr.p->data[3 * b_i];
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
      i = solution->c->size[1];
      k = solution->c->size[0] * solution->c->size[1];
      solution->c->size[0] = 1;
      solution->c->size[1]++;
      emxEnsureCapacity_real_T(solution->c, k);
      solution->c->data[i] = -solution->solution_constr.p->data[3 * b_i];
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
  arg = params->contact_normal[1] * 0.0 - params->contact_normal[2];
  t_ = params->contact_normal[2] * 0.0 - params->contact_normal[0] * 0.0;
  n_samples = params->contact_normal[0] - params->contact_normal[1] * 0.0;
  a_y = params->contact_normal[1] - params->contact_normal[2] * 0.0;
  a_z = params->contact_normal[2] * 0.0 - params->contact_normal[0];
  solution_tmp =
      params->contact_normal[0] * 0.0 - params->contact_normal[1] * 0.0;
  /*  compute components */
  y_tmp = (params->contact_normal[0] * Fleg[0] +
           params->contact_normal[1] * Fleg[1]) +
          params->contact_normal[2] * Fleg[2];
  t_ = ((t_ * params->contact_normal[2] -
         params->contact_normal[1] * n_samples) *
            Fleg[0] +
        (params->contact_normal[0] * n_samples -
         arg * params->contact_normal[2]) *
            Fleg[1]) +
       (arg * params->contact_normal[1] - params->contact_normal[0] * t_) *
           Fleg[2];
  arg = ((a_z * params->contact_normal[2] -
          params->contact_normal[1] * solution_tmp) *
             Fleg[0] +
         (params->contact_normal[0] * solution_tmp -
          a_y * params->contact_normal[2]) *
             Fleg[1]) +
        (a_y * params->contact_normal[1] - params->contact_normal[0] * a_z) *
            Fleg[2];
  /* 3 ------------------------------ Fleg constraints */
  /*  unilateral */
  i = solution->c->size[1];
  k = solution->c->size[0] * solution->c->size[1];
  solution->c->size[1]++;
  emxEnsureCapacity_real_T(solution->c, k);
  solution->c->data[i] = -y_tmp;
  /* (Fun >fmin )  */
  /* max force  */
  i = solution->c->size[1];
  k = solution->c->size[0] * solution->c->size[1];
  solution->c->size[0] = 1;
  solution->c->size[1]++;
  emxEnsureCapacity_real_T(solution->c, k);
  solution->c->data[i] = b_norm(Fleg) - Fleg_max;
  /* (Fun < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    i = solution->c->size[1];
    k = solution->c->size[0] * solution->c->size[1];
    solution->c->size[0] = 1;
    solution->c->size[1]++;
    emxEnsureCapacity_real_T(solution->c, k);
    solution->c->data[i] = muDoubleScalarSqrt(t_ * t_ + arg * arg) - mu * y_tmp;
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
  r = _mm_loadu_pd(&solution->solution_constr.p
                        ->data[3 * (solution->solution_constr.p->size[1] - 1)]);
  _mm_storeu_pd(&b_p0[0], _mm_sub_pd(r, _mm_loadu_pd(&pf[0])));
  b_p0[2] = solution->solution_constr.p
                ->data[3 * (solution->solution_constr.p->size[1] - 1) + 2] -
            pf[2];
  i = solution->c->size[1];
  k = solution->c->size[0] * solution->c->size[1];
  solution->c->size[0] = 1;
  solution->c->size[1]++;
  emxEnsureCapacity_real_T(solution->c, k);
  solution->c->data[i] = b_norm(b_p0) - 0.02;
  /* 5 - jump clearance */
  if (vectorUB > 0) {
    i = solution->c->size[1];
    k = solution->c->size[0] * solution->c->size[1];
    solution->c->size[0] = 1;
    solution->c->size[1]++;
    emxEnsureCapacity_real_T(solution->c, k);
    solution->c->data[i] =
        -solution->solution_constr.p
             ->data[3 * ((int32_T)(params->N_dyn / 2.0) - 1)] +
        params->jump_clearance;
  }
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
  solution->num_constr.wall_constraints = params->N_dyn;
  solution->num_constr.retraction_force_constraints = 0.0;
  solution->num_constr.force_constraints = nx;
  solution->num_constr.initial_final_constraints = 1.0;
  solution->num_constr.via_point = vectorUB;
  solution->constr_tolerance = 0.001;
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

real_T optimize_cpp_anonFcn1(const real_T p0[3], real_T params_m,
                             real_T params_num_params,
                             const char_T params_int_method[3],
                             real_T params_N_dyn, real_T params_int_steps,
                             real_T params_b, const real_T params_p_a1[3],
                             const real_T params_p_a2[3], real_T params_g,
                             real_T params_w1, real_T params_w2,
                             real_T params_T_th, const emxArray_real_T *x)
{
  __m128d r;
  emxArray_real_T *Fr_l;
  emxArray_real_T *Fr_r;
  emxArray_real_T *b_states;
  emxArray_real_T *b_t;
  emxArray_real_T *c_states;
  emxArray_real_T *d_states;
  emxArray_real_T *e_states;
  emxArray_real_T *f_states;
  emxArray_real_T *g_states;
  emxArray_real_T *p;
  emxArray_real_T *pd;
  emxArray_real_T *states;
  real_T dv[6];
  real_T b_x[3];
  const real_T *x_data;
  real_T absxk;
  real_T b_scale;
  real_T b_y;
  real_T dt_dyn;
  real_T scale;
  real_T t;
  real_T varargout_1;
  real_T y;
  real_T *Fr_l_data;
  real_T *b_states_data;
  real_T *c_states_data;
  real_T *d_states_data;
  real_T *e_states_data;
  real_T *f_states_data;
  real_T *states_data;
  int32_T b_loop_ub;
  int32_T c_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  int32_T nx;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  dt_dyn = params_num_params + params_N_dyn;
  if (params_num_params + 1.0 > dt_dyn) {
    i = 0;
    nx = 0;
  } else {
    i = (int32_T)(params_num_params + 1.0) - 1;
    nx = (int32_T)dt_dyn;
  }
  emxInit_real_T(&Fr_l, 2);
  i1 = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  loop_ub = nx - i;
  Fr_l->size[1] = loop_ub;
  emxEnsureCapacity_real_T(Fr_l, i1);
  Fr_l_data = Fr_l->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    Fr_l_data[i1] = x_data[i + i1];
  }
  scale = params_num_params + 2.0 * params_N_dyn;
  if (dt_dyn + 1.0 > scale) {
    i1 = 0;
    i2 = 0;
  } else {
    i1 = (int32_T)(dt_dyn + 1.0) - 1;
    i2 = (int32_T)scale;
  }
  emxInit_real_T(&Fr_r, 2);
  i3 = Fr_r->size[0] * Fr_r->size[1];
  Fr_r->size[0] = 1;
  b_loop_ub = i2 - i1;
  Fr_r->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(Fr_r, i3);
  Fr_l_data = Fr_r->data;
  for (i3 = 0; i3 < b_loop_ub; i3++) {
    Fr_l_data[i3] = x_data[i1 + i3];
  }
  /*  check they are column vectors */
  /*  variable intergration step */
  dt_dyn = x_data[3] / (params_N_dyn - 1.0);
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
  computeRollout(dv, dt_dyn, params_N_dyn, Fr_l, Fr_r, b_x, params_int_method,
                 params_int_steps, params_m, params_b, params_p_a1, params_p_a2,
                 params_g, params_T_th, states, b_t);
  states_data = states->data;
  emxInit_real_T(&b_states, 2);
  i3 = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, i3);
  Fr_l_data = b_states->data;
  c_loop_ub = states->size[1];
  emxInit_real_T(&c_states, 2);
  i3 = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, i3);
  b_states_data = c_states->data;
  emxInit_real_T(&d_states, 2);
  i3 = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, i3);
  c_states_data = d_states->data;
  emxInit_real_T(&e_states, 2);
  i3 = e_states->size[0] * e_states->size[1];
  e_states->size[0] = 1;
  e_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(e_states, i3);
  d_states_data = e_states->data;
  emxInit_real_T(&f_states, 2);
  i3 = f_states->size[0] * f_states->size[1];
  f_states->size[0] = 1;
  f_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(f_states, i3);
  e_states_data = f_states->data;
  emxInit_real_T(&g_states, 2);
  i3 = g_states->size[0] * g_states->size[1];
  g_states->size[0] = 1;
  g_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(g_states, i3);
  f_states_data = g_states->data;
  for (i3 = 0; i3 < c_loop_ub; i3++) {
    Fr_l_data[i3] = states_data[6 * i3];
    b_states_data[i3] = states_data[6 * i3 + 1];
    c_states_data[i3] = states_data[6 * i3 + 2];
    d_states_data[i3] = states_data[6 * i3 + 3];
    e_states_data[i3] = states_data[6 * i3 + 4];
    f_states_data[i3] = states_data[6 * i3 + 5];
  }
  emxInit_real_T(&p, 2);
  emxInit_real_T(&pd, 2);
  b_computePositionVelocity(params_b, b_states, c_states, d_states, e_states,
                            f_states, g_states, p, pd);
  emxFree_real_T(&g_states);
  emxFree_real_T(&f_states);
  emxFree_real_T(&e_states);
  emxFree_real_T(&pd);
  emxFree_real_T(&p);
  /*  be careful there are only N values in this vector the path migh be */
  /*  underestimated! */
  /*      deltax = diff(p(1,:));  % diff(X); */
  /*      deltay = diff(p(2,:));   % diff(Y); */
  /*      deltaz = diff(p(3,:));    % diff(Z); */
  /*      path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2)); */
  /* minimize the final kin energy at contact */
  /*  minimize hoist work / energy consumption for the hoist work we integrathe
   * the power on a rough grid */
  if (loop_ub == states->size[1]) {
    nx = b_t->size[0] * b_t->size[1];
    b_t->size[0] = 1;
    b_t->size[1] = loop_ub;
    emxEnsureCapacity_real_T(b_t, nx);
    Fr_l_data = b_t->data;
    for (nx = 0; nx < loop_ub; nx++) {
      Fr_l_data[nx] = x_data[i + nx] * states_data[6 * nx + 4];
    }
  } else {
    c_binary_expand_op(b_t, x, i, nx - 1, states);
    Fr_l_data = b_t->data;
  }
  nx = b_t->size[1];
  i = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = b_t->size[1];
  emxEnsureCapacity_real_T(c_states, i);
  b_states_data = c_states->data;
  for (c_loop_ub = 0; c_loop_ub < nx; c_loop_ub++) {
    b_states_data[c_loop_ub] = muDoubleScalarAbs(Fr_l_data[c_loop_ub]);
  }
  if (b_loop_ub == states->size[1]) {
    i = b_t->size[0] * b_t->size[1];
    b_t->size[0] = 1;
    b_t->size[1] = b_loop_ub;
    emxEnsureCapacity_real_T(b_t, i);
    Fr_l_data = b_t->data;
    for (i = 0; i < b_loop_ub; i++) {
      Fr_l_data[i] = x_data[i1 + i] * states_data[6 * i + 5];
    }
  } else {
    b_binary_expand_op(b_t, x, i1, i2 - 1, states);
    Fr_l_data = b_t->data;
  }
  emxFree_real_T(&states);
  nx = b_t->size[1];
  i = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = b_t->size[1];
  emxEnsureCapacity_real_T(d_states, i);
  c_states_data = d_states->data;
  for (c_loop_ub = 0; c_loop_ub < nx; c_loop_ub++) {
    c_states_data[c_loop_ub] = muDoubleScalarAbs(Fr_l_data[c_loop_ub]);
  }
  /* assume the motor is not regenreating */
  /*  smoothnes: minimize jerky control action TODO this is wrong! it goes */
  /*  to -180 and stays there! with sum(abs(diff(Fr_r))) + */
  /*  sum(abs(diff(Fr_l))) but does not converge at all  */
  /*  this is nice but slower */
  /* fprintf("hoist_work %f\n ",hoist_work)     */
  /* fprintf("smooth %f\n ", smooth) */
  /* fprintf("tempo %f\n ", w6*Tf) */
  /* cost =  0.001 * params.w1 *Ekinfcost +   params.w4 *smooth ;% converge */
  /* super slowly */
  i = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = c_states->size[1];
  emxEnsureCapacity_real_T(b_states, i);
  Fr_l_data = b_states->data;
  loop_ub = c_states->size[1];
  nx = (c_states->size[1] / 2) << 1;
  c_loop_ub = nx - 2;
  for (i = 0; i <= c_loop_ub; i += 2) {
    r = _mm_loadu_pd(&b_states_data[i]);
    _mm_storeu_pd(&Fr_l_data[i], _mm_mul_pd(r, _mm_set1_pd(dt_dyn)));
  }
  for (i = nx; i < loop_ub; i++) {
    Fr_l_data[i] = b_states_data[i] * dt_dyn;
  }
  i = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = d_states->size[1];
  emxEnsureCapacity_real_T(c_states, i);
  b_states_data = c_states->data;
  loop_ub = d_states->size[1];
  nx = (d_states->size[1] / 2) << 1;
  c_loop_ub = nx - 2;
  for (i = 0; i <= c_loop_ub; i += 2) {
    r = _mm_loadu_pd(&c_states_data[i]);
    _mm_storeu_pd(&b_states_data[i], _mm_mul_pd(r, _mm_set1_pd(dt_dyn)));
  }
  for (i = nx; i < loop_ub; i++) {
    b_states_data[i] = c_states_data[i] * dt_dyn;
  }
  diff(Fr_r, d_states);
  emxFree_real_T(&Fr_r);
  diff(Fr_l, b_t);
  emxFree_real_T(&Fr_l);
  varargout_1 = params_w2 * (sum(b_states) + sum(c_states)) +
                params_w1 * (sum(d_states) + sum(b_t));
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  emxFree_real_T(&b_states);
  emxFree_real_T(&b_t);
  /*  72 iter */
  /*  cost =    params.w4 *smooth ;% 27 iter */
  /*  cost =    params.w4 *smooth_correct ;% 96 iter */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

void optimize_cpp_anonFcn2(const real_T p0[3], const real_T pf[3],
                           real_T Fleg_max, real_T mu, const param *params,
                           const emxArray_real_T *x,
                           emxArray_real_T *varargout_1)
{
  emxArray_real_T *b_t;
  emxArray_real_T *b_x;
  emxArray_real_T *c_x;
  emxArray_real_T *p;
  emxArray_real_T *states;
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
  real_T *b_x_data;
  real_T *c_x_data;
  real_T *p_data;
  real_T *t_data;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /*  ineq are <= 0 */
  Fleg[0] = x_data[0];
  Fleg[1] = x_data[1];
  Fleg[2] = x_data[2];
  scale = params->num_params + params->N_dyn;
  if (params->num_params + 1.0 > scale) {
    i = 0;
    i1 = 0;
  } else {
    i = (int32_T)(params->num_params + 1.0) - 1;
    i1 = (int32_T)scale;
  }
  y = params->num_params + 2.0 * params->N_dyn;
  if (scale + 1.0 > y) {
    i2 = 0;
    b_i = 0;
  } else {
    i2 = (int32_T)(scale + 1.0) - 1;
    b_i = (int32_T)y;
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
  b_scale = 3.3121686421112381E-170;
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p0[0] - params->p_a1[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[0] - params->p_a2[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params->p_a1[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params->p_a2[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params->p_a1[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params->p_a2[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  y = scale * muDoubleScalarSqrt(y);
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = b_y;
  dv[2] = y;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  emxInit_real_T(&b_x, 2);
  i3 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  loop_ub = i1 - i;
  b_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_x, i3);
  b_x_data = b_x->data;
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_x_data[i1] = x_data[i + i1];
  }
  emxInit_real_T(&c_x, 2);
  i = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  loop_ub = b_i - i2;
  c_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(c_x, i);
  c_x_data = c_x->data;
  for (i = 0; i < loop_ub; i++) {
    c_x_data[i] = x_data[i2 + i];
  }
  emxInit_real_T(&states, 2);
  emxInit_real_T(&b_t, 2);
  computeRollout(dv, x_data[3] / (params->N_dyn - 1.0), params->N_dyn, b_x, c_x,
                 Fleg, params->int_method, params->int_steps, params->m,
                 params->b, params->p_a1, params->p_a2, params->g, params->T_th,
                 states, b_t);
  p_data = states->data;
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_x, i);
  b_x_data = b_x->data;
  loop_ub = states->size[1];
  i = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_x, i);
  c_x_data = c_x->data;
  i = b_t->size[0] * b_t->size[1];
  b_t->size[0] = 1;
  b_t->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_t, i);
  t_data = b_t->data;
  for (i = 0; i < loop_ub; i++) {
    b_x_data[i] = p_data[6 * i];
    c_x_data[i] = p_data[6 * i + 1];
    t_data[i] = p_data[6 * i + 2];
  }
  emxFree_real_T(&states);
  emxInit_real_T(&p, 2);
  computePositionVelocity(params->b, b_x, c_x, b_t, p);
  p_data = p->data;
  emxFree_real_T(&c_x);
  emxFree_real_T(&b_x);
  emxFree_real_T(&b_t);
  /* only position */
  /*  I assume px py pz  are row vectors */
  /*  init struct foc C++ code generation */
  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0  */
  if (params->obstacle_avoidance) {
    /* [0; 3;-7.5]; */
    b_scale = params->obstacle_size[0] * params->obstacle_size[0];
    absxk = b_scale / (params->obstacle_size[1] * params->obstacle_size[1]);
    t = b_scale / (params->obstacle_size[2] * params->obstacle_size[2]);
    /* px > sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2); */
    /* -px + sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2)<0 */
    /*  better implementaiton than the one with with complex numbers for code */
    /*  generation, we just check if the argument of the sqrt becomes */
    /*  negative */
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      y = p_data[3 * b_i + 2] - params->obstacle_location[2];
      scale = p_data[3 * b_i + 1] - params->obstacle_location[1];
      scale = (b_scale - t * (y * y)) - absxk * (scale * scale);
      /* %%add ineq only if inside sphere */
      if (scale > 0.0) {
        i1 = varargout_1->size[1];
        i2 = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[0] = 1;
        varargout_1->size[1]++;
        emxEnsureCapacity_real_T(varargout_1, i2);
        t_data = varargout_1->data;
        t_data[i1] = ((-p_data[3 * b_i] + params->obstacle_location[0]) +
                      muDoubleScalarSqrt(scale)) +
                     params->jump_clearance;
      } else {
        i1 = varargout_1->size[1];
        i2 = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[1]++;
        emxEnsureCapacity_real_T(varargout_1, i2);
        t_data = varargout_1->data;
        t_data[i1] = -p_data[3 * b_i];
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      i1 = varargout_1->size[1];
      i2 = varargout_1->size[0] * varargout_1->size[1];
      varargout_1->size[1]++;
      emxEnsureCapacity_real_T(varargout_1, i2);
      t_data = varargout_1->data;
      t_data[i1] = -p_data[3 * b_i];
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
  scale = params->contact_normal[1] * 0.0 - params->contact_normal[2];
  y = params->contact_normal[2] * 0.0 - params->contact_normal[0] * 0.0;
  b_scale = params->contact_normal[0] - params->contact_normal[1] * 0.0;
  c_idx_0 = params->contact_normal[1] - params->contact_normal[2] * 0.0;
  c_idx_1 = params->contact_normal[2] * 0.0 - params->contact_normal[0];
  c_idx_2 = params->contact_normal[0] * 0.0 - params->contact_normal[1] * 0.0;
  /*  compute components */
  b_params = (params->contact_normal[0] * Fleg[0] +
              params->contact_normal[1] * Fleg[1]) +
             params->contact_normal[2] * Fleg[2];
  y = ((y * params->contact_normal[2] - params->contact_normal[1] * b_scale) *
           Fleg[0] +
       (params->contact_normal[0] * b_scale -
        scale * params->contact_normal[2]) *
           Fleg[1]) +
      (scale * params->contact_normal[1] - params->contact_normal[0] * y) *
          Fleg[2];
  /* 3 ------------------------------ Fleg constraints */
  /*  unilateral */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  t_data[i] = -b_params;
  /* (Fun >fmin )  */
  /* max force  */
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(Fleg[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }
  absxk = muDoubleScalarAbs(Fleg[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  scale = ((c_idx_1 * params->contact_normal[2] -
            params->contact_normal[1] * c_idx_2) *
               Fleg[0] +
           (params->contact_normal[0] * c_idx_2 -
            c_idx_0 * params->contact_normal[2]) *
               Fleg[1]) +
          (c_idx_0 * params->contact_normal[1] -
           params->contact_normal[0] * c_idx_1) *
              Fleg[2];
  absxk = muDoubleScalarAbs(Fleg[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  t_data[i] = b_y - Fleg_max;
  /* (Fun < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    i = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = 1;
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    t_data = varargout_1->data;
    t_data[i] = muDoubleScalarSqrt(y * y + scale * scale) - mu * b_params;
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
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p_data[3 * (p->size[1] - 1)] - pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }
  absxk = muDoubleScalarAbs(p_data[3 * (p->size[1] - 1) + 1] - pf[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  absxk = muDoubleScalarAbs(p_data[3 * (p->size[1] - 1) + 2] - pf[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  t_data[i] = b_y - 0.02;
  /* 5 - jump clearance */
  if (!params->obstacle_avoidance) {
    i = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    t_data = varargout_1->data;
    t_data[i] = -p_data[3 * ((int32_T)(params->N_dyn / 2.0) - 1)] +
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
