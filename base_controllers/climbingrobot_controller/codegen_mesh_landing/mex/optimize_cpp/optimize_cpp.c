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
#include "constraints.h"
#include "cost.h"
#include "diff.h"
#include "eval_solution.h"
#include "find.h"
#include "fmincon.h"
#include "integrate_dynamics.h"
#include "interp2.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_mexutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "tic.h"
#include "toc.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void optimize_cpp(optimize_cppStackData *SD, const real_T p0[3],
                  const real_T pf[3], real_T Fleg_max, real_T Fr_max,
                  real_T Fr_min, real_T mu, const param *params,
                  struct0_T *solution)
{
  __m128d r2;
  emxArray_boolean_T *c_solution;
  emxArray_int32_T *r;
  emxArray_real_T *b_Fleg_max;
  emxArray_real_T *b_solution;
  emxArray_real_T *b_states;
  emxArray_real_T *c_Fleg_max;
  emxArray_real_T *c_states;
  emxArray_real_T *d_Fleg_max;
  emxArray_real_T *pd_fine;
  emxArray_real_T *states;
  emxArray_real_T *states_rough;
  real_T state0[6];
  real_T this_workspace_pf[3];
  real_T x[3];
  real_T absxk;
  real_T b_scale;
  real_T hoist_work_fine;
  real_T intEkin;
  real_T l1_tmp;
  real_T l2_tmp;
  real_T n_samples;
  real_T scale;
  real_T t;
  real_T y;
  real_T y_tmp;
  real_T *Fleg_max_data;
  real_T *b_Fleg_max_data;
  real_T *b_states_data;
  real_T *c_Fleg_max_data;
  real_T *c_states_data;
  real_T *solution_data;
  real_T *states_data;
  real_T *states_rough_data;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T impulse_end_idx;
  int32_T impulse_end_idx_tmp;
  int32_T k;
  int32_T last;
  int32_T loop_ub;
  int32_T nx;
  int32_T *r1;
  uint32_T rough_count;
  boolean_T *b_solution_data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /* make sure is column vector */
  /* for eval solution */
  /*  needs to be fixed for code generation */
  /*  only to evaluate solution */
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
  last = (int32_T)params->N_dyn;
  for (i = 0; i < last; i++) {
    Fleg_max_data[i + 4] = 0.0;
  }
  for (i = 0; i < last; i++) {
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
  for (i = 0; i < last; i++) {
    b_Fleg_max_data[i + 4] = -Fr_max;
  }
  for (i = 0; i < last; i++) {
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
  for (i = 0; i < last; i++) {
    c_Fleg_max_data[i + 4] = -Fr_min;
  }
  for (i = 0; i < last; i++) {
    c_Fleg_max_data[(i + (int32_T)params->N_dyn) + 4] = -Fr_min;
  }
  solution->cost = fmincon(
      SD, p0, params, b_Fleg_max, c_Fleg_max, d_Fleg_max, x, this_workspace_pf,
      Fleg_max, mu, params, solution->x, &solution->problem_solved,
      &solution->optim_output.iterations, &solution->optim_output.funcCount,
      solution->optim_output.algorithm, &solution->optim_output.constrviolation,
      &solution->optim_output.stepsize, &solution->optim_output.lssteplength,
      &solution->optim_output.firstorderopt);
  toc();
  /* eval trajectory */
  solution->Fleg[0] = solution->x->data[0];
  solution->Fleg[1] = solution->x->data[1];
  solution->Fleg[2] = solution->x->data[2];
  scale = params->num_params + params->N_dyn;
  if (params->num_params + 1.0 > scale) {
    i = 0;
    k = 0;
  } else {
    i = (int32_T)(params->num_params + 1.0) - 1;
    k = (int32_T)scale;
  }
  n_samples = params->num_params + 2.0 * params->N_dyn;
  if (scale + 1.0 > n_samples) {
    i1 = 0;
    nx = 0;
  } else {
    i1 = (int32_T)(scale + 1.0) - 1;
    nx = (int32_T)n_samples;
  }
  /*  resample inputs  */
  n_samples = muDoubleScalarFloor(solution->x->data[3] / 0.001);
  last = solution->Fr_l_fine->size[0] * solution->Fr_l_fine->size[1];
  solution->Fr_l_fine->size[0] = 1;
  impulse_end_idx_tmp = (int32_T)n_samples;
  solution->Fr_l_fine->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(solution->Fr_l_fine, last);
  last = solution->Fr_r_fine->size[0] * solution->Fr_r_fine->size[1];
  solution->Fr_r_fine->size[0] = 1;
  solution->Fr_r_fine->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(solution->Fr_r_fine, last);
  rough_count = 1U;
  scale = 0.0;
  for (last = 0; last < impulse_end_idx_tmp; last++) {
    scale += 0.001;
    if (scale >= n_samples * 0.001 / (params->N_dyn - 1.0)) {
      rough_count++;
      scale = 0.0;
    }
    solution->Fr_l_fine->data[last] =
        solution->x->data[(i + (int32_T)rough_count) - 1];
    solution->Fr_r_fine->data[last] =
        solution->x->data[(i1 + (int32_T)rough_count) - 1];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /*  single shooting */
  state0[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  state0[1] = l1_tmp;
  state0[2] = l2_tmp;
  state0[3] = 0.0;
  state0[4] = 0.0;
  state0[5] = 0.0;
  /*  course integration */
  emxInit_real_T(&b_solution, 2);
  last = b_solution->size[0] * b_solution->size[1];
  b_solution->size[0] = 1;
  loop_ub = k - i;
  b_solution->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_solution, last);
  solution_data = b_solution->data;
  for (k = 0; k < loop_ub; k++) {
    solution_data[k] = solution->x->data[i + k];
  }
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_loop_ub = nx - i1;
  b_Fleg_max->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  for (k = 0; k < b_loop_ub; k++) {
    Fleg_max_data[k] = solution->x->data[i1 + k];
  }
  emxInit_real_T(&states, 2);
  computeRollout(state0, solution->x->data[3] / (params->N_dyn - 1.0),
                 params->N_dyn, b_solution, b_Fleg_max, solution->Fleg,
                 params->int_method, params->int_steps, params->m, params->b,
                 params->p_a1, params->p_a2, params->g, params->T_th, states,
                 solution->time);
  states_data = states->data;
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  last = states->size[1];
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
  k = b_solution->size[0] * b_solution->size[1];
  b_solution->size[0] = 1;
  b_solution->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_solution, k);
  solution_data = b_solution->data;
  emxInit_real_T(&c_states, 2);
  k = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, k);
  c_states_data = c_states->data;
  for (k = 0; k < last; k++) {
    Fleg_max_data[k] = states_data[6 * k];
    b_Fleg_max_data[k] = states_data[6 * k + 1];
    c_Fleg_max_data[k] = states_data[6 * k + 2];
    b_states_data[k] = states_data[6 * k + 3];
    solution_data[k] = states_data[6 * k + 4];
    c_states_data[k] = states_data[6 * k + 5];
  }
  b_computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
                            b_states, b_solution, c_states, solution->p,
                            solution->p_fine);
  /*  fine integration  */
  /* init */
  emxInit_real_T(&states_rough, 2);
  integrate_dynamics(
      state0, 0.001, n_samples, solution->Fr_l_fine, solution->Fr_r_fine,
      solution->Fleg, params->int_method, params->m, params->b, params->p_a1,
      params->p_a2, params->g, params->T_th, states_rough, solution->time_fine);
  states_rough_data = states_rough->data;
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  last = states_rough->size[1];
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
  k = b_solution->size[0] * b_solution->size[1];
  b_solution->size[0] = 1;
  b_solution->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(b_solution, k);
  solution_data = b_solution->data;
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
  for (k = 0; k < last; k++) {
    Fleg_max_data[k] = states_rough_data[6 * k];
    b_Fleg_max_data[k] = states_rough_data[6 * k + 1];
    c_Fleg_max_data[k] = states_rough_data[6 * k + 2];
    solution_data[k] = states_rough_data[6 * k + 3];
    b_states_data[k] = states_rough_data[6 * k + 4];
    c_states_data[k] = states_rough_data[6 * k + 5];
  }
  emxInit_real_T(&pd_fine, 2);
  b_computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
                            b_solution, b_states, c_states, solution->p_fine,
                            pd_fine);
  c_states_data = pd_fine->data;
  emxFree_real_T(&c_states);
  /*  init struct foc C++ code generation */
  /* compute path */
  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  /*  check length is always l */
  /*      a = vecnorm(p) */
  /*      a -  ones(1,length(a))*l */
  /*  kinetic energy at the beginning */
  y_tmp = params->m / 2.0;
  k = solution->Ekin->size[0] * solution->Ekin->size[1];
  solution->Ekin->size[0] = 1;
  solution->Ekin->size[1] = solution->time_fine->size[1];
  emxEnsureCapacity_real_T(solution->Ekin, k);
  last = solution->time_fine->size[1];
  for (k = 0; k < last; k++) {
    solution->Ekin->data[k] = 0.0;
  }
  intEkin = 0.0;
  k = solution->time_fine->size[1];
  for (last = 0; last < k; last++) {
    scale = c_states_data[3 * last];
    y = y_tmp * scale * scale;
    scale = c_states_data[3 * last + 1];
    y += y_tmp * scale * scale;
    scale = c_states_data[3 * last + 2];
    y += y_tmp * scale * scale;
    solution->Ekin->data[last] = y;
    intEkin += y * 0.001;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /* Once a structure is read or used in a way that forces code generation to
   * fix its definition, you cannot add new fields afterward. */
  /* energy computations */
  /*  this function is not used, is implemented just as a reference for python
   */
  /*  we compute energy consumption in python */
  /* Energy consumption */
  /* maps joule to Wh */
  emxInit_boolean_T(&c_solution, 2);
  k = c_solution->size[0] * c_solution->size[1];
  c_solution->size[0] = 1;
  c_solution->size[1] = solution->time->size[1];
  emxEnsureCapacity_boolean_T(c_solution, k);
  b_solution_data = c_solution->data;
  last = solution->time->size[1];
  for (k = 0; k < last; k++) {
    b_solution_data[k] = (solution->time->data[k] <= params->T_th);
  }
  emxInit_int32_T(&r, 2);
  eml_find(c_solution, r);
  r1 = r->data;
  emxFree_boolean_T(&c_solution);
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = r->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  b_Fleg_max_data = c_Fleg_max->data;
  last = r->size[1];
  for (k = 0; k < last; k++) {
    b_Fleg_max_data[k] = r1[k];
  }
  emxFree_int32_T(&r);
  last = c_Fleg_max->size[1];
  if (c_Fleg_max->size[1] <= 2) {
    if (c_Fleg_max->size[1] == 1) {
      impulse_end_idx = (int32_T)b_Fleg_max_data[0];
    } else {
      last = (int32_T)b_Fleg_max_data[c_Fleg_max->size[1] - 1];
      impulse_end_idx_tmp = (int32_T)b_Fleg_max_data[0];
      if (impulse_end_idx_tmp < last) {
        impulse_end_idx = last;
      } else {
        impulse_end_idx = impulse_end_idx_tmp;
      }
    }
  } else {
    impulse_end_idx = (int32_T)b_Fleg_max_data[0];
    for (k = 2; k <= last; k++) {
      scale = b_Fleg_max_data[k - 1];
      if (impulse_end_idx < (int32_T)scale) {
        impulse_end_idx = (int32_T)scale;
      }
    }
  }
  /*  the work is the kinetic energy at the end of the thrusting */
  /* params.m/2*pd(:,impulse_end_idx)'*pd(:,impulse_end_idx); */
  /* for the hoist work we integrathe the ppowet on a rough grid */
  k = solution->time->size[1];
  for (last = 0; last < k; last++) {
    /* assume the motor is not regenreating */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /*  more precise */
  n_samples = solution->time_fine->data[1] - solution->time_fine->data[0];
  hoist_work_fine = 0.0;
  k = solution->time_fine->size[1];
  for (last = 0; last < k; last++) {
    hoist_work_fine += (muDoubleScalarAbs(solution->Fr_l_fine->data[last] *
                                          states_rough_data[6 * last + 4]) +
                        muDoubleScalarAbs(solution->Fr_r_fine->data[last] *
                                          states_rough_data[6 * last + 5])) *
                       n_samples;
    /* assume the motor is not regenreating */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /*      impulse_workWh = J_TO_Wh*impulse_work */
  /*      hoist_workWh=J_TO_Wh*hoist_work */
  if (states_rough->size[1] == solution->Fr_l_fine->size[1]) {
    k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
    c_Fleg_max->size[0] = 1;
    c_Fleg_max->size[1] = states_rough->size[1];
    emxEnsureCapacity_real_T(c_Fleg_max, k);
    b_Fleg_max_data = c_Fleg_max->data;
    last = states_rough->size[1];
    for (k = 0; k < last; k++) {
      b_Fleg_max_data[k] =
          solution->Fr_l_fine->data[k] * states_rough_data[6 * k + 4];
    }
  } else {
    b_binary_expand_op(c_Fleg_max, states_rough, solution);
    b_Fleg_max_data = c_Fleg_max->data;
  }
  nx = c_Fleg_max->size[1];
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = c_Fleg_max->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  Fleg_max_data = b_Fleg_max->data;
  for (k = 0; k < nx; k++) {
    Fleg_max_data[k] = muDoubleScalarAbs(b_Fleg_max_data[k]);
  }
  k = b_solution->size[0] * b_solution->size[1];
  b_solution->size[0] = 1;
  b_solution->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(b_solution, k);
  solution_data = b_solution->data;
  last = solution->p->size[1];
  for (k = 0; k < last; k++) {
    solution_data[k] = solution->p->data[3 * k];
  }
  diff(b_solution, c_Fleg_max);
  k = b_solution->size[0] * b_solution->size[1];
  b_solution->size[0] = 1;
  b_solution->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(b_solution, k);
  solution_data = b_solution->data;
  last = solution->p->size[1];
  for (k = 0; k < last; k++) {
    solution_data[k] = solution->p->data[3 * k + 1];
  }
  diff(b_solution, d_Fleg_max);
  c_Fleg_max_data = d_Fleg_max->data;
  k = b_solution->size[0] * b_solution->size[1];
  b_solution->size[0] = 1;
  b_solution->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(b_solution, k);
  solution_data = b_solution->data;
  last = solution->p->size[1];
  for (k = 0; k < last; k++) {
    solution_data[k] = solution->p->data[3 * k + 2];
  }
  diff(b_solution, b_states);
  b_states_data = b_states->data;
  emxFree_real_T(&b_solution);
  if (c_Fleg_max->size[1] == 1) {
    nx = d_Fleg_max->size[1];
  } else {
    nx = c_Fleg_max->size[1];
  }
  if ((c_Fleg_max->size[1] == d_Fleg_max->size[1]) &&
      (nx == b_states->size[1])) {
    last = c_Fleg_max->size[1] - 1;
    k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
    c_Fleg_max->size[0] = 1;
    emxEnsureCapacity_real_T(c_Fleg_max, k);
    b_Fleg_max_data = c_Fleg_max->data;
    for (k = 0; k <= last; k++) {
      n_samples = b_Fleg_max_data[k];
      scale = c_Fleg_max_data[k];
      l1_tmp = b_states_data[k];
      b_Fleg_max_data[k] =
          (n_samples * n_samples + scale * scale) + l1_tmp * l1_tmp;
    }
  } else {
    binary_expand_op(c_Fleg_max, d_Fleg_max, b_states);
    b_Fleg_max_data = c_Fleg_max->data;
  }
  emxFree_real_T(&b_states);
  emxFree_real_T(&d_Fleg_max);
  nx = c_Fleg_max->size[1];
  impulse_end_idx_tmp = (c_Fleg_max->size[1] / 2) << 1;
  last = impulse_end_idx_tmp - 2;
  for (k = 0; k <= last; k += 2) {
    r2 = _mm_loadu_pd(&b_Fleg_max_data[k]);
    _mm_storeu_pd(&b_Fleg_max_data[k], _mm_sqrt_pd(r2));
  }
  for (k = impulse_end_idx_tmp; k < nx; k++) {
    b_Fleg_max_data[k] = muDoubleScalarSqrt(b_Fleg_max_data[k]);
  }
  solution->path_length = sum(c_Fleg_max);
  emxFree_real_T(&c_Fleg_max);
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(solution->p->data[0] - p0[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[3 * (solution->p->size[1] - 1)] -
                            pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    n_samples = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    n_samples = t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[1] - p0[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(
      solution->p->data[3 * (solution->p->size[1] - 1) + 1] - pf[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    n_samples = n_samples * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    n_samples += t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[2] - p0[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(
      solution->p->data[3 * (solution->p->size[1] - 1) + 2] - pf[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    n_samples = n_samples * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    n_samples += t * t;
  }
  solution->initial_error = scale * muDoubleScalarSqrt(y);
  solution->final_error_real = b_scale * muDoubleScalarSqrt(n_samples);
  k = solution->Fr_l->size[0] * solution->Fr_l->size[1];
  solution->Fr_l->size[0] = 1;
  solution->Fr_l->size[1] = loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_l, k);
  for (k = 0; k < loop_ub; k++) {
    solution->Fr_l->data[k] = solution->x->data[i + k];
  }
  i = solution->Fr_r->size[0] * solution->Fr_r->size[1];
  solution->Fr_r->size[0] = 1;
  solution->Fr_r->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_r, i);
  for (i = 0; i < b_loop_ub; i++) {
    solution->Fr_r->data[i] = solution->x->data[i1 + i];
  }
  i = solution->psi->size[0] * solution->psi->size[1];
  solution->psi->size[0] = 1;
  solution->psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psi, i);
  loop_ub = states->size[1];
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
  for (i = 0; i < loop_ub; i++) {
    solution->psi->data[i] = states_data[6 * i];
    solution->l1->data[i] = states_data[6 * i + 1];
    solution->l2->data[i] = states_data[6 * i + 2];
    solution->psid->data[i] = states_data[6 * i + 3];
    solution->l1d->data[i] = states_data[6 * i + 4];
    solution->l2d->data[i] = states_data[6 * i + 5];
  }
  emxFree_real_T(&states);
  i = solution->psi_fine->size[0] * solution->psi_fine->size[1];
  solution->psi_fine->size[0] = 1;
  solution->psi_fine->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(solution->psi_fine, i);
  loop_ub = states_rough->size[1];
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
  for (i = 0; i < loop_ub; i++) {
    solution->psi_fine->data[i] = states_rough_data[6 * i];
    solution->l1_fine->data[i] = states_rough_data[6 * i + 1];
    solution->l2_fine->data[i] = states_rough_data[6 * i + 2];
    solution->psid_fine->data[i] = states_rough_data[6 * i + 3];
    solution->l1d_fine->data[i] = states_rough_data[6 * i + 4];
    solution->l2d_fine->data[i] = states_rough_data[6 * i + 5];
  }
  emxFree_real_T(&states_rough);
  solution->Tf = solution->x->data[3];
  solution->Etot = 0.0;
  l2_tmp = y_tmp * c_states_data[0] * c_states_data[0];
  solution->Ekin0x = l2_tmp;
  y = y_tmp * c_states_data[1] * c_states_data[1];
  solution->Ekin0y = y;
  b_scale = y_tmp * c_states_data[2] * c_states_data[2];
  solution->Ekin0z = b_scale;
  solution->intEkin = intEkin;
  solution->U0 = 0.0;
  solution->Uf = 0.0;
  n_samples = c_states_data[3 * (pd_fine->size[1] - 1)];
  n_samples *= y_tmp * n_samples;
  solution->Ekinfx = n_samples;
  scale = c_states_data[3 * (pd_fine->size[1] - 1) + 1];
  scale *= y_tmp * scale;
  solution->Ekinfy = scale;
  l1_tmp = c_states_data[3 * (pd_fine->size[1] - 1) + 2];
  emxFree_real_T(&pd_fine);
  l1_tmp *= y_tmp * l1_tmp;
  solution->Ekinfz = l1_tmp;
  solution->achieved_target[0] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1)];
  solution->achieved_target[1] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1) + 1];
  solution->achieved_target[2] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1) + 2];
  solution->Ekin0 = (l2_tmp + y) + b_scale;
  solution->Ekinf = (n_samples + scale) + l1_tmp;
  solution->consumed_energy =
      solution->Ekin->data[impulse_end_idx - 1] + hoist_work_fine;
  i = solution->instantaneous_power->size[0] *
      solution->instantaneous_power->size[1];
  solution->instantaneous_power->size[0] = 1;
  solution->instantaneous_power->size[1] = b_Fleg_max->size[1];
  emxEnsureCapacity_real_T(solution->instantaneous_power, i);
  loop_ub = b_Fleg_max->size[1];
  impulse_end_idx_tmp = (b_Fleg_max->size[1] / 2) << 1;
  last = impulse_end_idx_tmp - 2;
  for (i = 0; i <= last; i += 2) {
    r2 = _mm_loadu_pd(&Fleg_max_data[i]);
    _mm_storeu_pd(&solution->instantaneous_power->data[i], _mm_add_pd(r2, r2));
  }
  for (i = impulse_end_idx_tmp; i < loop_ub; i++) {
    scale = Fleg_max_data[i];
    solution->instantaneous_power->data[i] = scale + scale;
  }
  emxFree_real_T(&b_Fleg_max);
  solution->average_power = hoist_work_fine / solution->x->data[3];
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
  constraints(solution->x, p0, pf, Fleg_max, mu, params, solution->c,
              &solution->num_constr, &solution->solution_constr);
  solution->constr_tolerance = 0.001;
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

real_T optimize_cpp_anonFcn1(const real_T p0[3], const param *params,
                             const emxArray_real_T *x)
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
  dt_dyn = params->num_params + params->N_dyn;
  if (params->num_params + 1.0 > dt_dyn) {
    i = 0;
    nx = 0;
  } else {
    i = (int32_T)(params->num_params + 1.0) - 1;
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
  scale = params->num_params + 2.0 * params->N_dyn;
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
  dt_dyn = x_data[3] / (params->N_dyn - 1.0);
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
  b_x[0] = x_data[0];
  b_x[1] = x_data[1];
  b_x[2] = x_data[2];
  emxInit_real_T(&states, 2);
  emxInit_real_T(&b_t, 2);
  computeRollout(dv, dt_dyn, params->N_dyn, Fr_l, Fr_r, b_x, params->int_method,
                 params->int_steps, params->m, params->b, params->p_a1,
                 params->p_a2, params->g, params->T_th, states, b_t);
  states_data = states->data;
  emxInit_real_T(&b_states, 2);
  i3 = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, i3);
  b_states_data = b_states->data;
  c_loop_ub = states->size[1];
  emxInit_real_T(&c_states, 2);
  i3 = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, i3);
  c_states_data = c_states->data;
  emxInit_real_T(&d_states, 2);
  i3 = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, i3);
  d_states_data = d_states->data;
  emxInit_real_T(&e_states, 2);
  i3 = e_states->size[0] * e_states->size[1];
  e_states->size[0] = 1;
  e_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(e_states, i3);
  Fr_l_data = e_states->data;
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
    b_states_data[i3] = states_data[6 * i3];
    c_states_data[i3] = states_data[6 * i3 + 1];
    d_states_data[i3] = states_data[6 * i3 + 2];
    Fr_l_data[i3] = states_data[6 * i3 + 3];
    e_states_data[i3] = states_data[6 * i3 + 4];
    f_states_data[i3] = states_data[6 * i3 + 5];
  }
  emxInit_real_T(&p, 2);
  emxInit_real_T(&pd, 2);
  b_computePositionVelocity(params->b, b_states, c_states, d_states, e_states,
                            f_states, g_states, p, pd);
  e_states_data = p->data;
  emxFree_real_T(&g_states);
  emxFree_real_T(&f_states);
  emxFree_real_T(&e_states);
  emxFree_real_T(&pd);
  /* minimize the final kin energy at contact */
  /*  minimize hoist work / energy consumption for the hoist work we integrathe
   * the power on a rough grid */
  /* assume the motor is not regenreating */
  /*  smoothnes: minimize jerky control action TODO this is wrong! it goes */
  /*  to -180 and stays there! with sum(abs(diff(Fr_r))) + */
  /*  sum(abs(diff(Fr_l))) but does not converge at all  */
  /*  this is nice but slower */
  /* fprintf("\n smooth %f\n ", params.w1*smooth) */
  /* fprintf(" hoist_work %f\n ",params.w2*hoist_work)   */
  /* fprintf(" landing_cost %f\n ", params.w3*landing_cost) */
  /* cost =  0.001 * params.w1 *Ekinfcost +   params.w4 *smooth ;% converge */
  /* super slowly */
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
    d_binary_expand_op(b_t, x, i, nx - 1, states);
    Fr_l_data = b_t->data;
  }
  nx = b_t->size[1];
  i = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = b_t->size[1];
  emxEnsureCapacity_real_T(c_states, i);
  c_states_data = c_states->data;
  for (c_loop_ub = 0; c_loop_ub < nx; c_loop_ub++) {
    c_states_data[c_loop_ub] = muDoubleScalarAbs(Fr_l_data[c_loop_ub]);
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
    c_binary_expand_op(b_t, x, i1, i2 - 1, states);
    Fr_l_data = b_t->data;
  }
  emxFree_real_T(&states);
  nx = b_t->size[1];
  i = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = b_t->size[1];
  emxEnsureCapacity_real_T(d_states, i);
  d_states_data = d_states->data;
  for (c_loop_ub = 0; c_loop_ub < nx; c_loop_ub++) {
    d_states_data[c_loop_ub] = muDoubleScalarAbs(Fr_l_data[c_loop_ub]);
  }
  i = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = c_states->size[1];
  emxEnsureCapacity_real_T(b_states, i);
  b_states_data = b_states->data;
  loop_ub = c_states->size[1];
  nx = (c_states->size[1] / 2) << 1;
  c_loop_ub = nx - 2;
  for (i = 0; i <= c_loop_ub; i += 2) {
    r = _mm_loadu_pd(&c_states_data[i]);
    _mm_storeu_pd(&b_states_data[i], _mm_mul_pd(r, _mm_set1_pd(dt_dyn)));
  }
  for (i = nx; i < loop_ub; i++) {
    b_states_data[i] = c_states_data[i] * dt_dyn;
  }
  i = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = d_states->size[1];
  emxEnsureCapacity_real_T(c_states, i);
  c_states_data = c_states->data;
  loop_ub = d_states->size[1];
  nx = (d_states->size[1] / 2) << 1;
  c_loop_ub = nx - 2;
  for (i = 0; i <= c_loop_ub; i += 2) {
    r = _mm_loadu_pd(&d_states_data[i]);
    _mm_storeu_pd(&c_states_data[i], _mm_mul_pd(r, _mm_set1_pd(dt_dyn)));
  }
  for (i = nx; i < loop_ub; i++) {
    c_states_data[i] = d_states_data[i] * dt_dyn;
  }
  diff(Fr_r, d_states);
  emxFree_real_T(&Fr_r);
  diff(Fr_l, b_t);
  emxFree_real_T(&Fr_l);
  varargout_1 =
      (params->w2 * (sum(b_states) + sum(c_states)) +
       params->w1 * (sum(d_states) + sum(b_t))) +
      params->w3 * interp2(params->cost_z, params->cost_y, params->cost_x,
                           e_states_data[3 * (p->size[1] - 1) + 2],
                           e_states_data[3 * (p->size[1] - 1) + 1]);
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  emxFree_real_T(&b_states);
  emxFree_real_T(&p);
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
  static const int32_T b_iv[2] = {1, 13};
  static const char_T b[4] = {'n', 'o', 'n', 'e'};
  static const char_T b_b[4] = {'m', 'e', 's', 'h'};
  emxArray_real_T *b_t;
  emxArray_real_T *b_x;
  emxArray_real_T *c_x;
  emxArray_real_T *p;
  emxArray_real_T *states;
  const mxArray *c_y;
  const mxArray *m;
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
  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0 */
  if (memcmp((char_T *)&params->obstacle_avoidance[0], (char_T *)&b[0], 4) ==
      0) {
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      i1 = varargout_1->size[1];
      i2 = varargout_1->size[0] * varargout_1->size[1];
      varargout_1->size[1]++;
      emxEnsureCapacity_real_T(varargout_1, i2);
      t_data = varargout_1->data;
      t_data[i1] = -p_data[3 * b_i];
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else if (memcmp((char_T *)&params->obstacle_avoidance[0], (char_T *)&b_b[0],
                    4) == 0) {
    /* p_x > wall_x + jump_clearance => p_x -wall_z- jump_clearance  >0 => -p_x
     * +wall_z + jump_clearance  <0 */
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      i1 = varargout_1->size[1];
      i2 = varargout_1->size[0] * varargout_1->size[1];
      varargout_1->size[0] = 1;
      varargout_1->size[1]++;
      emxEnsureCapacity_real_T(varargout_1, i2);
      t_data = varargout_1->data;
      t_data[i1] = -p_data[3 * b_i] +
                   interp2(params->mesh_z, params->mesh_y, params->mesh_x,
                           p_data[3 * b_i + 2], p_data[3 * b_i + 1]);
      /* fprintf('debug wall: %f %f \n',p(1,i),params.jump_clearance +wall_x);
       */
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    c_y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 13, m, &cv[0]);
    emlrtAssign(&c_y, m);
    disp(c_y, &emlrtMCI);
  }
  /*  % % debug */
  /*  disp('after wall') */
  /*  length(ineq) */
  /*  2- N_dyn constraints on retraction force   -Fr_max < Fr < 0 */
  /*  unilaterality */
  /*  debug */
  /*  disp('after Fr') */
  /*  length(ineq) */
  /*  constraints on impulse force */
  b_scale = params->contact_normal[1] * 0.0 - params->contact_normal[2];
  scale = params->contact_normal[2] * 0.0 - params->contact_normal[0] * 0.0;
  y = params->contact_normal[0] - params->contact_normal[1] * 0.0;
  c_idx_0 = params->contact_normal[1] - params->contact_normal[2] * 0.0;
  c_idx_1 = params->contact_normal[2] * 0.0 - params->contact_normal[0];
  c_idx_2 = params->contact_normal[0] * 0.0 - params->contact_normal[1] * 0.0;
  /*  compute components */
  b_params = (params->contact_normal[0] * Fleg[0] +
              params->contact_normal[1] * Fleg[1]) +
             params->contact_normal[2] * Fleg[2];
  y = ((scale * params->contact_normal[2] - params->contact_normal[1] * y) *
           Fleg[0] +
       (params->contact_normal[0] * y - b_scale * params->contact_normal[2]) *
           Fleg[1]) +
      (b_scale * params->contact_normal[1] -
       params->contact_normal[0] * scale) *
          Fleg[2];
  /* 3 ------------------------------ Fleg constraints */
  /*  unilateral */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  t_data[i] = -b_params;
  /* (Fun >fmin ) */
  /* max force */
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
  /* (|Fun| < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    /* |Fut| < mu*Fun */
    i = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = 1;
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    t_data = varargout_1->data;
    t_data[i] = muDoubleScalarSqrt(y * y + scale * scale) - mu * b_params;
    /* friction constraints */
  }
  /* 4- landing constraint, landing point inside the selected patch */
  /* constraint Y direction inside patch  bounds  */
  /* p_f(2) < ymax => p_f(2) -ymax < 0 */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  scale = p_data[3 * (p->size[1] - 1) + 1];
  y = params->patch_side / 2.0;
  t_data[i] = scale - (pf[1] + y);
  /* p_f(2) > ymin => -p_f(2) < -ymin => -p_f(2) + ymin<0 */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  t_data[i] = -scale + (pf[1] - y);
  /* constraint Z direction inside patch  bounds */
  /* p_f(3) < zmax => p_f(3) -zmax < 0 */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  b_scale = p_data[3 * (p->size[1] - 1) + 2];
  t_data[i] = b_scale - (pf[2] + y);
  /* p_f(3) > zmin => -p_f(3) < -zmin => -p_f(3) + zmin<0 */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  t_data[i] = -b_scale + (pf[2] - y);
  /* constraint the X (otherwise it finds something in the air!) */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  t_data = varargout_1->data;
  t_data[i] = muDoubleScalarAbs(p_data[3 * (p->size[1] - 1)] -
                                interp2(params->mesh_z, params->mesh_y,
                                        params->mesh_x, b_scale, scale)) -
              0.02;
  /* old way  */
  /*  ineq= [ineq (norm(p_f - pf) - fixed_slack)]; */
  /*  number_of_constr.final_constraints =  1; */
  /* 5 - jump clearance p_x > jump_clearance */
  if (memcmp((char_T *)&params->obstacle_avoidance[0], (char_T *)&b_b[0], 4) ==
      0) {
    i = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = 1;
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    t_data = varargout_1->data;
    b_i = (int32_T)(params->N_dyn / 2.0) - 1;
    t_data[i] = (-p_data[3 * b_i] + interp2(params->mesh_z, params->mesh_y,
                                            params->mesh_x, p_data[3 * b_i + 2],
                                            p_data[3 * b_i + 1])) +
                params->jump_clearance;
  }
  emxFree_real_T(&p);
  /*  if any(isinf(ineq)) */
  /*      disp('Infn in constraint') */
  /*      find(isinf(ineq)) */
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
