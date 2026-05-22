/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eval_solution.c
 *
 * Code generation for function 'eval_solution'
 *
 */

/* Include files */
#include "eval_solution.h"
#include "computePositionVelocity.h"
#include "computeRollout.h"
#include "diff.h"
#include "integrate_dynamics.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Declarations */
static void c_binary_expand_op(emxArray_real_T *in1,
                               const emxArray_real_T *in2);

static void d_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in3,
                               const emxArray_real_T *in4);

/* Function Definitions */
static void c_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2)
{
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  real_T *b_in2_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&b_in2, 2);
  i = b_in2->size[0] * b_in2->size[1];
  b_in2->size[0] = 1;
  if (in1->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in1->size[1];
  }
  b_in2->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_in2, i);
  b_in2_data = b_in2->data;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in1->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[i] =
        in1_data[i * stride_1_1] * in2_data[6 * (i * stride_0_1) + 4];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = b_in2->size[1];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = b_in2->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in2_data[i];
  }
  emxFree_real_T(&b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

static void d_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in3,
                               const emxArray_real_T *in4)
{
  emxArray_real_T *r;
  const real_T *in3_data;
  const real_T *in4_data;
  real_T *in1_data;
  real_T *r1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  int32_T stride_2_1;
  in4_data = in4->data;
  in3_data = in3->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&r, 2);
  i = r->size[0] * r->size[1];
  r->size[0] = 1;
  if (in4->size[1] == 1) {
    if (in3->size[1] == 1) {
      loop_ub = in1->size[1];
    } else {
      loop_ub = in3->size[1];
    }
  } else {
    loop_ub = in4->size[1];
  }
  r->size[1] = loop_ub;
  emxEnsureCapacity_real_T(r, i);
  r1 = r->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in3->size[1] != 1);
  stride_2_1 = (in4->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    real_T b_varargin_1;
    real_T c_varargin_1;
    real_T varargin_1;
    varargin_1 = in1_data[i * stride_0_1];
    b_varargin_1 = in3_data[i * stride_1_1];
    c_varargin_1 = in4_data[i * stride_2_1];
    r1[i] = (varargin_1 * varargin_1 + b_varargin_1 * b_varargin_1) +
            c_varargin_1 * c_varargin_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = r->size[1];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = r->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = r1[i];
  }
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

void eval_solution(const emxArray_real_T *x, const real_T p0[3],
                   const real_T pf[3], real_T params_m,
                   real_T params_num_params, const char_T params_int_method[3],
                   real_T params_N_dyn, real_T params_int_steps,
                   real_T params_b, const real_T params_p_a1[3],
                   const real_T params_p_a2[3], real_T params_g,
                   real_T params_T_th, c_struct_T *solution)
{
  __m128d r;
  emxArray_boolean_T *d_x;
  emxArray_int32_T *b_i;
  emxArray_real_T *Ekin;
  emxArray_real_T *Fr_l_fine;
  emxArray_real_T *b_states;
  emxArray_real_T *b_x;
  emxArray_real_T *c_states;
  emxArray_real_T *c_x;
  emxArray_real_T *d_states;
  emxArray_real_T *pd_fine;
  emxArray_real_T *states;
  emxArray_real_T *states_rough;
  real_T state0[6];
  const real_T *x_data;
  real_T absxk;
  real_T b_y;
  real_T n_samples;
  real_T scale;
  real_T t;
  real_T t_;
  real_T y;
  real_T y_tmp;
  real_T *Ekin_data;
  real_T *Fr_l_fine_data;
  real_T *b_states_data;
  real_T *b_x_data;
  real_T *c_states_data;
  real_T *c_x_data;
  real_T *d_states_data;
  real_T *states_data;
  real_T *states_rough_data;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T k;
  int32_T last;
  int32_T loop_ub;
  int32_T nx;
  int32_T *i_data;
  uint32_T rough_count;
  boolean_T exitg1;
  boolean_T *d_x_data;
  x_data = x->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  /* eval trajectory */
  solution->Fleg[0] = x_data[0];
  solution->Fleg[1] = x_data[1];
  solution->Fleg[2] = x_data[2];
  scale = params_num_params + params_N_dyn;
  if (params_num_params + 1.0 > scale) {
    i = 0;
    k = 0;
  } else {
    i = (int32_T)(params_num_params + 1.0) - 1;
    k = (int32_T)scale;
  }
  t_ = params_num_params + 2.0 * params_N_dyn;
  if (scale + 1.0 > t_) {
    i1 = 0;
    loop_ub = 0;
  } else {
    i1 = (int32_T)(scale + 1.0) - 1;
    loop_ub = (int32_T)t_;
  }
  /*  resample inputs  */
  n_samples = muDoubleScalarFloor(x_data[3] / 0.001);
  emxInit_real_T(&Fr_l_fine, 2);
  idx = Fr_l_fine->size[0] * Fr_l_fine->size[1];
  Fr_l_fine->size[0] = 1;
  last = (int32_T)n_samples;
  Fr_l_fine->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(Fr_l_fine, idx);
  Fr_l_fine_data = Fr_l_fine->data;
  idx = solution->Fr_r_fine->size[0] * solution->Fr_r_fine->size[1];
  solution->Fr_r_fine->size[0] = 1;
  solution->Fr_r_fine->size[1] = (int32_T)n_samples;
  emxEnsureCapacity_real_T(solution->Fr_r_fine, idx);
  rough_count = 1U;
  t_ = 0.0;
  for (nx = 0; nx < last; nx++) {
    t_ += 0.001;
    if (t_ >= n_samples * 0.001 / (params_N_dyn - 1.0)) {
      rough_count++;
      t_ = 0.0;
    }
    Fr_l_fine_data[nx] = x_data[(i + (int32_T)rough_count) - 1];
    solution->Fr_r_fine->data[nx] = x_data[(i1 + (int32_T)rough_count) - 1];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /*  single shooting */
  t_ = 3.3121686421112381E-170;
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p0[0] - params_p_a1[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    t_ = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[0] - params_p_a2[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params_p_a1[1]);
  if (absxk > t_) {
    t = t_ / absxk;
    y = y * t * t + 1.0;
    t_ = absxk;
  } else {
    t = absxk / t_;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[1] - params_p_a2[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params_p_a1[2]);
  if (absxk > t_) {
    t = t_ / absxk;
    y = y * t * t + 1.0;
    t_ = absxk;
  } else {
    t = absxk / t_;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(p0[2] - params_p_a2[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }
  y = t_ * muDoubleScalarSqrt(y);
  b_y = scale * muDoubleScalarSqrt(b_y);
  state0[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  state0[1] = y;
  state0[2] = b_y;
  state0[3] = 0.0;
  state0[4] = 0.0;
  state0[5] = 0.0;
  /*  course integration */
  emxInit_real_T(&b_x, 2);
  idx = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_loop_ub = k - i;
  b_x->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(b_x, idx);
  b_x_data = b_x->data;
  for (k = 0; k < b_loop_ub; k++) {
    b_x_data[k] = x_data[i + k];
  }
  emxInit_real_T(&c_x, 2);
  k = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  loop_ub -= i1;
  c_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(c_x, k);
  c_x_data = c_x->data;
  for (k = 0; k < loop_ub; k++) {
    c_x_data[k] = x_data[i1 + k];
  }
  emxInit_real_T(&states, 2);
  computeRollout(state0, x_data[3] / (params_N_dyn - 1.0), params_N_dyn, b_x,
                 c_x, solution->Fleg, params_int_method, params_int_steps,
                 params_m, params_b, params_p_a1, params_p_a2, params_g,
                 params_T_th, states, solution->time);
  states_data = states->data;
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_x, k);
  b_x_data = b_x->data;
  idx = states->size[1];
  k = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_x, k);
  c_x_data = c_x->data;
  emxInit_real_T(&Ekin, 2);
  k = Ekin->size[0] * Ekin->size[1];
  Ekin->size[0] = 1;
  Ekin->size[1] = states->size[1];
  emxEnsureCapacity_real_T(Ekin, k);
  Ekin_data = Ekin->data;
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
  for (k = 0; k < idx; k++) {
    b_x_data[k] = states_data[6 * k];
    c_x_data[k] = states_data[6 * k + 1];
    Ekin_data[k] = states_data[6 * k + 2];
    b_states_data[k] = states_data[6 * k + 3];
    c_states_data[k] = states_data[6 * k + 4];
    d_states_data[k] = states_data[6 * k + 5];
  }
  b_computePositionVelocity(params_b, b_x, c_x, Ekin, b_states, c_states,
                            d_states, solution->p, solution->p_fine);
  /*  fine integration  */
  /* init */
  emxInit_real_T(&states_rough, 2);
  integrate_dynamics(state0, 0.001, n_samples, Fr_l_fine, solution->Fr_r_fine,
                     solution->Fleg, params_int_method, params_m, params_b,
                     params_p_a1, params_p_a2, params_g, params_T_th,
                     states_rough, solution->time_fine);
  states_rough_data = states_rough->data;
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(b_x, k);
  b_x_data = b_x->data;
  idx = states_rough->size[1];
  k = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(c_x, k);
  c_x_data = c_x->data;
  k = Ekin->size[0] * Ekin->size[1];
  Ekin->size[0] = 1;
  Ekin->size[1] = states_rough->size[1];
  emxEnsureCapacity_real_T(Ekin, k);
  Ekin_data = Ekin->data;
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
  for (k = 0; k < idx; k++) {
    b_x_data[k] = states_rough_data[6 * k];
    c_x_data[k] = states_rough_data[6 * k + 1];
    Ekin_data[k] = states_rough_data[6 * k + 2];
    b_states_data[k] = states_rough_data[6 * k + 3];
    c_states_data[k] = states_rough_data[6 * k + 4];
    d_states_data[k] = states_rough_data[6 * k + 5];
  }
  emxInit_real_T(&pd_fine, 2);
  b_computePositionVelocity(params_b, b_x, c_x, Ekin, b_states, c_states,
                            d_states, solution->p_fine, pd_fine);
  c_states_data = pd_fine->data;
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  /*  init struct foc C++ code generation */
  /* compute path */
  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  k = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(c_x, k);
  c_x_data = c_x->data;
  idx = solution->p->size[1];
  for (k = 0; k < idx; k++) {
    c_x_data[k] = solution->p->data[3 * k];
  }
  diff(c_x, Ekin);
  k = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(c_x, k);
  c_x_data = c_x->data;
  idx = solution->p->size[1];
  for (k = 0; k < idx; k++) {
    c_x_data[k] = solution->p->data[3 * k + 1];
  }
  diff(c_x, b_states);
  b_states_data = b_states->data;
  k = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(c_x, k);
  c_x_data = c_x->data;
  idx = solution->p->size[1];
  for (k = 0; k < idx; k++) {
    c_x_data[k] = solution->p->data[3 * k + 2];
  }
  diff(c_x, b_x);
  b_x_data = b_x->data;
  emxFree_real_T(&c_x);
  if (Ekin->size[1] == 1) {
    idx = b_states->size[1];
  } else {
    idx = Ekin->size[1];
  }
  if ((Ekin->size[1] == b_states->size[1]) && (idx == b_x->size[1])) {
    idx = Ekin->size[1] - 1;
    k = Ekin->size[0] * Ekin->size[1];
    Ekin->size[0] = 1;
    emxEnsureCapacity_real_T(Ekin, k);
    Ekin_data = Ekin->data;
    for (k = 0; k <= idx; k++) {
      t_ = Ekin_data[k];
      absxk = b_states_data[k];
      scale = b_x_data[k];
      Ekin_data[k] = (t_ * t_ + absxk * absxk) + scale * scale;
    }
  } else {
    d_binary_expand_op(Ekin, b_states, b_x);
    Ekin_data = Ekin->data;
  }
  emxFree_real_T(&b_x);
  nx = Ekin->size[1];
  idx = (Ekin->size[1] / 2) << 1;
  last = idx - 2;
  for (k = 0; k <= last; k += 2) {
    r = _mm_loadu_pd(&Ekin_data[k]);
    _mm_storeu_pd(&Ekin_data[k], _mm_sqrt_pd(r));
  }
  for (k = idx; k < nx; k++) {
    Ekin_data[k] = muDoubleScalarSqrt(Ekin_data[k]);
  }
  solution->path_length = sum(Ekin);
  /*  check length is always l */
  /*      a = vecnorm(p) */
  /*      a -  ones(1,length(a))*l */
  t_ = 3.3121686421112381E-170;
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(solution->p->data[0] - p0[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    t_ = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[3 * (solution->p->size[1] - 1)] -
                            pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[1] - p0[1]);
  if (absxk > t_) {
    t = t_ / absxk;
    y = y * t * t + 1.0;
    t_ = absxk;
  } else {
    t = absxk / t_;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(
      solution->p->data[3 * (solution->p->size[1] - 1) + 1] - pf[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }
  absxk = muDoubleScalarAbs(solution->p->data[2] - p0[2]);
  if (absxk > t_) {
    t = t_ / absxk;
    y = y * t * t + 1.0;
    t_ = absxk;
  } else {
    t = absxk / t_;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(
      solution->p->data[3 * (solution->p->size[1] - 1) + 2] - pf[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }
  solution->initial_error = t_ * muDoubleScalarSqrt(y);
  solution->final_error_real = scale * muDoubleScalarSqrt(b_y);
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
  emxFree_real_T(&states);
  i = solution->Fr_l_fine->size[0] * solution->Fr_l_fine->size[1];
  solution->Fr_l_fine->size[0] = 1;
  solution->Fr_l_fine->size[1] = Fr_l_fine->size[1];
  emxEnsureCapacity_real_T(solution->Fr_l_fine, i);
  b_loop_ub = Fr_l_fine->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    solution->Fr_l_fine->data[i] = Fr_l_fine_data[i];
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
  solution->Tf = x_data[3];
  solution->Etot = 0.0;
  solution->U0 = 0.0;
  solution->Uf = 0.0;
  /*  kinetic energy at the beginning */
  t_ = params_m / 2.0 * c_states_data[0] * c_states_data[0];
  solution->Ekin0x = t_;
  scale = params_m / 2.0 * c_states_data[1] * c_states_data[1];
  solution->Ekin0y = scale;
  absxk = params_m / 2.0 * c_states_data[2] * c_states_data[2];
  solution->Ekin0z = absxk;
  y_tmp = params_m / 2.0;
  t = c_states_data[3 * (pd_fine->size[1] - 1)];
  t *= params_m / 2.0 * t;
  solution->Ekinfx = t;
  b_y = c_states_data[3 * (pd_fine->size[1] - 1) + 1];
  b_y *= params_m / 2.0 * b_y;
  solution->Ekinfy = b_y;
  n_samples = c_states_data[3 * (pd_fine->size[1] - 1) + 2];
  n_samples *= params_m / 2.0 * n_samples;
  solution->Ekinfz = n_samples;
  solution->achieved_target[0] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1)];
  solution->achieved_target[1] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1) + 1];
  solution->achieved_target[2] =
      solution->p_fine->data[3 * (solution->p_fine->size[1] - 1) + 2];
  solution->Ekin0 = (t_ + scale) + absxk;
  solution->Ekinf = (t + b_y) + n_samples;
  i = Ekin->size[0] * Ekin->size[1];
  Ekin->size[0] = 1;
  Ekin->size[1] = solution->time_fine->size[1];
  emxEnsureCapacity_real_T(Ekin, i);
  Ekin_data = Ekin->data;
  b_loop_ub = solution->time_fine->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    Ekin_data[i] = 0.0;
  }
  t_ = 0.0;
  i = solution->time_fine->size[1];
  for (nx = 0; nx < i; nx++) {
    scale = c_states_data[3 * nx];
    y = y_tmp * scale * scale;
    scale = c_states_data[3 * nx + 1];
    y += y_tmp * scale * scale;
    scale = c_states_data[3 * nx + 2];
    y += y_tmp * scale * scale;
    Ekin_data[nx] = y;
    t_ += y * 0.001;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  emxFree_real_T(&pd_fine);
  i = solution->Ekin->size[0] * solution->Ekin->size[1];
  solution->Ekin->size[0] = 1;
  solution->Ekin->size[1] = Ekin->size[1];
  emxEnsureCapacity_real_T(solution->Ekin, i);
  b_loop_ub = Ekin->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    solution->Ekin->data[i] = Ekin_data[i];
  }
  solution->intEkin = t_;
  /* Once a structure is read or used in a way that forces code generation to
   * fix its definition, you cannot add new fields afterward. */
  /* energy computations */
  /*  this function is not used, is implemented just as a reference for python
   */
  /*  we compute energy consumption in python */
  /* Energy consumption */
  /* maps joule to Wh */
  emxInit_boolean_T(&d_x, 2);
  i = d_x->size[0] * d_x->size[1];
  d_x->size[0] = 1;
  d_x->size[1] = solution->time->size[1];
  emxEnsureCapacity_boolean_T(d_x, i);
  d_x_data = d_x->data;
  b_loop_ub = solution->time->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    d_x_data[i] = (solution->time->data[i] <= params_T_th);
  }
  nx = d_x->size[1];
  idx = 0;
  emxInit_int32_T(&b_i, 2);
  i = b_i->size[0] * b_i->size[1];
  b_i->size[0] = 1;
  b_i->size[1] = d_x->size[1];
  emxEnsureCapacity_int32_T(b_i, i);
  i_data = b_i->data;
  last = 0;
  exitg1 = false;
  while ((!exitg1) && (last <= nx - 1)) {
    if (d_x_data[last]) {
      idx++;
      i_data[idx - 1] = last + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        last++;
      }
    } else {
      last++;
    }
  }
  if (d_x->size[1] == 1) {
    if (idx == 0) {
      b_i->size[0] = 1;
      b_i->size[1] = 0;
    }
  } else {
    i = b_i->size[0] * b_i->size[1];
    if (idx < 1) {
      b_i->size[1] = 0;
    } else {
      b_i->size[1] = idx;
    }
    emxEnsureCapacity_int32_T(b_i, i);
    i_data = b_i->data;
  }
  emxFree_boolean_T(&d_x);
  i = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = b_i->size[1];
  emxEnsureCapacity_real_T(b_states, i);
  b_states_data = b_states->data;
  b_loop_ub = b_i->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    b_states_data[i] = i_data[i];
  }
  emxFree_int32_T(&b_i);
  last = b_states->size[1];
  if (b_states->size[1] <= 2) {
    if (b_states->size[1] == 1) {
      idx = (int32_T)b_states_data[0];
    } else {
      last = (int32_T)b_states_data[b_states->size[1] - 1];
      idx = (int32_T)b_states_data[0];
      if (idx < last) {
        idx = last;
      }
    }
  } else {
    idx = (int32_T)b_states_data[0];
    for (k = 2; k <= last; k++) {
      i = (int32_T)b_states_data[k - 1];
      if (idx < i) {
        idx = i;
      }
    }
  }
  emxFree_real_T(&b_states);
  /*  the work is the kinetic energy at the end of the thrusting */
  /* params.m/2*pd(:,impulse_end_idx)'*pd(:,impulse_end_idx); */
  /* for the hoist work we integrathe the ppowet on a rough grid */
  i = solution->time->size[1];
  for (nx = 0; nx < i; nx++) {
    /* assume the motor is not regenreating */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /*  more precise */
  t_ = solution->time_fine->data[1] - solution->time_fine->data[0];
  absxk = 0.0;
  i = solution->time_fine->size[1];
  for (nx = 0; nx < i; nx++) {
    absxk +=
        (muDoubleScalarAbs(Fr_l_fine_data[nx] * states_rough_data[6 * nx + 4]) +
         muDoubleScalarAbs(solution->Fr_r_fine->data[nx] *
                           states_rough_data[6 * nx + 5])) *
        t_;
    /* assume the motor is not regenreating */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }
  /*      impulse_workWh = J_TO_Wh*impulse_work */
  /*      hoist_workWh=J_TO_Wh*hoist_work */
  solution->consumed_energy = Ekin_data[idx - 1] + absxk;
  if (states_rough->size[1] == Fr_l_fine->size[1]) {
    i = Fr_l_fine->size[0] * Fr_l_fine->size[1];
    Fr_l_fine->size[0] = 1;
    Fr_l_fine->size[1] = states_rough->size[1];
    emxEnsureCapacity_real_T(Fr_l_fine, i);
    Fr_l_fine_data = Fr_l_fine->data;
    b_loop_ub = states_rough->size[1];
    for (i = 0; i < b_loop_ub; i++) {
      Fr_l_fine_data[i] *= states_rough_data[6 * i + 4];
    }
  } else {
    c_binary_expand_op(Fr_l_fine, states_rough);
    Fr_l_fine_data = Fr_l_fine->data;
  }
  emxFree_real_T(&states_rough);
  nx = Fr_l_fine->size[1];
  i = Ekin->size[0] * Ekin->size[1];
  Ekin->size[0] = 1;
  Ekin->size[1] = Fr_l_fine->size[1];
  emxEnsureCapacity_real_T(Ekin, i);
  Ekin_data = Ekin->data;
  for (k = 0; k < nx; k++) {
    Ekin_data[k] = muDoubleScalarAbs(Fr_l_fine_data[k]);
  }
  emxFree_real_T(&Fr_l_fine);
  i = solution->instantaneous_power->size[0] *
      solution->instantaneous_power->size[1];
  solution->instantaneous_power->size[0] = 1;
  solution->instantaneous_power->size[1] = Ekin->size[1];
  emxEnsureCapacity_real_T(solution->instantaneous_power, i);
  b_loop_ub = Ekin->size[1];
  idx = (Ekin->size[1] / 2) << 1;
  last = idx - 2;
  for (i = 0; i <= last; i += 2) {
    r = _mm_loadu_pd(&Ekin_data[i]);
    _mm_storeu_pd(&solution->instantaneous_power->data[i], _mm_add_pd(r, r));
  }
  for (i = idx; i < b_loop_ub; i++) {
    scale = Ekin_data[i];
    solution->instantaneous_power->data[i] = scale + scale;
  }
  emxFree_real_T(&Ekin);
  solution->average_power = absxk / x_data[3];
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (eval_solution.c) */
