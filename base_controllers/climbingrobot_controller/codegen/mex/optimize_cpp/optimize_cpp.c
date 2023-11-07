/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
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
#include "fmincon.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void anon(const real_T p0[3], const real_T pf[3], real_T Fleg_max, real_T mu,
          const param *params, const emxArray_real_T *x, emxArray_real_T
          *varargout_1)
{
  emxArray_real_T *Fr_l;
  emxArray_real_T *p;
  emxArray_real_T *py;
  emxArray_real_T *pz;
  emxArray_real_T *states;
  real_T dv[6];
  real_T Fleg[3];
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
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&Fr_l, 2, true);

  /*  ineq are <= 0 */
  Fleg[0] = x->data[0];
  Fleg[1] = x->data[1];
  Fleg[2] = x->data[2];
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  loop_ub = (int32_T)muDoubleScalarFloor(params->N_dyn - 1.0);
  Fr_l->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(Fr_l, i);
  for (i = 0; i <= loop_ub; i++) {
    Fr_l->data[i] = x->data[(int32_T)(params->num_params + (real_T)(i + 1)) - 1];
  }

  y = (params->num_params + params->N_dyn) + 1.0;
  scale = params->num_params + 2.0 * params->N_dyn;
  if (y > scale) {
    i = 0;
    i1 = 0;
  } else {
    i = (int32_T)y - 1;
    i1 = (int32_T)scale;
  }

  /*  check they are column vectors */
  /*  size not known */
  varargout_1->size[0] = 1;
  varargout_1->size[1] = 0;

  /*  number of constraints */
  /*  already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes */
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

  emxInit_real_T(&pz, 2, true);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  y = scale * muDoubleScalarSqrt(y);
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = b_y;
  dv[2] = y;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  i2 = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  loop_ub = i1 - i;
  pz->size[1] = loop_ub;
  emxEnsureCapacity_real_T(pz, i2);
  for (i1 = 0; i1 < loop_ub; i1++) {
    pz->data[i1] = x->data[i + i1];
  }

  emxInit_real_T(&states, 2, true);
  emxInit_real_T(&py, 2, true);
  computeRollout(dv, x->data[3] / (params->N_dyn - 1.0), params->N_dyn, Fr_l, pz,
                 Fleg, params->int_method, params->int_steps, params->m,
                 params->b, params->p_a1, params->p_a2, params->g, params->T_th,
                 states, py);
  loop_ub = states->size[1];
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  Fr_l->size[1] = states->size[1];
  emxEnsureCapacity_real_T(Fr_l, i);
  for (i = 0; i < loop_ub; i++) {
    Fr_l->data[i] = 0.0;
  }

  loop_ub = states->size[1];
  i = py->size[0] * py->size[1];
  py->size[0] = 1;
  py->size[1] = states->size[1];
  emxEnsureCapacity_real_T(py, i);
  for (i = 0; i < loop_ub; i++) {
    py->data[i] = 0.0;
  }

  loop_ub = states->size[1];
  i = pz->size[0] * pz->size[1];
  pz->size[0] = 1;
  pz->size[1] = states->size[1];
  emxEnsureCapacity_real_T(pz, i);
  for (i = 0; i < loop_ub; i++) {
    pz->data[i] = 0.0;
  }

  i = states->size[1] - 1;
  for (b_i = 0; b_i <= i; b_i++) {
    absxk = states->data[6 * b_i];
    t = states->data[6 * b_i + 1];
    scale = states->data[6 * b_i + 2];
    y = params->b * params->b;
    b_scale = t * t;
    scale = (y + b_scale) - scale * scale;
    y = muDoubleScalarSqrt(1.0 - scale * scale / (4.0 * y * b_scale));
    Fr_l->data[b_i] = t * muDoubleScalarSin(absxk) * y;
    py->data[b_i] = scale / (2.0 * params->b);
    pz->data[b_i] = -t * muDoubleScalarCos(absxk) * y;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  emxFree_real_T(&states);
  emxInit_real_T(&p, 2, true);
  i = p->size[0] * p->size[1];
  p->size[0] = 3;
  p->size[1] = Fr_l->size[1];
  emxEnsureCapacity_real_T(p, i);
  loop_ub = Fr_l->size[1];
  for (i = 0; i < loop_ub; i++) {
    p->data[3 * i] = Fr_l->data[i];
  }

  emxFree_real_T(&Fr_l);
  loop_ub = py->size[1];
  for (i = 0; i < loop_ub; i++) {
    p->data[3 * i + 1] = py->data[i];
  }

  emxFree_real_T(&py);
  loop_ub = pz->size[1];
  for (i = 0; i < loop_ub; i++) {
    p->data[3 * i + 2] = pz->data[i];
  }

  /* only position */
  /*  I assume px py pz  are row vectors */
  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0  */
  if (params->obstacle_avoidance) {
    /* [0; 3;-7.5]; */
    /* px > sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2); */
    /* -px + sqrt(radius.^2 - a_z*(pz-center(3)).^2 -a_y*(py-center(2)).^2)<0 */
    /*  better implementaiton with complex numbers for code generation */
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      scale = p->data[3 * b_i + 2] - params->obstacle_location[2];
      y = p->data[3 * b_i + 1] - params->obstacle_location[1];
      scale = (2.25 - 3.0001760103259394 * (scale * scale)) - y * y;

      /* %%add ineq only if inside sphere */
      if (scale > 0.0) {
        i1 = pz->size[0] * pz->size[1];
        pz->size[0] = 1;
        pz->size[1] = varargout_1->size[1] + 1;
        emxEnsureCapacity_real_T(pz, i1);
        loop_ub = varargout_1->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          pz->data[i1] = varargout_1->data[i1];
        }

        pz->data[varargout_1->size[1]] = ((-p->data[3 * b_i] +
          params->obstacle_location[0]) + muDoubleScalarSqrt(scale)) +
          params->jump_clearance;
        i1 = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[0] = 1;
        varargout_1->size[1] = pz->size[1];
        emxEnsureCapacity_real_T(varargout_1, i1);
        loop_ub = pz->size[0] * pz->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          varargout_1->data[i1] = pz->data[i1];
        }
      } else {
        i1 = varargout_1->size[1];
        i2 = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[1]++;
        emxEnsureCapacity_real_T(varargout_1, i2);
        varargout_1->data[i1] = -p->data[3 * b_i];
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
      varargout_1->data[i1] = -p->data[3 * b_i];

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
  b_params = (params->contact_normal[0] * Fleg[0] + params->contact_normal[1] *
              Fleg[1]) + params->contact_normal[2] * Fleg[2];
  y = ((y * params->contact_normal[2] - b_scale * params->contact_normal[1]) *
       Fleg[0] + (b_scale * params->contact_normal[0] - scale *
                  params->contact_normal[2]) * Fleg[1]) + (scale *
    params->contact_normal[1] - y * params->contact_normal[0]) * Fleg[2];

  /* 3 ------------------------------ Fleg constraints */
  /*  unilateral */
  i = varargout_1->size[1];
  i1 = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i1);
  varargout_1->data[i] = -b_params;

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

  scale = ((c_idx_1 * params->contact_normal[2] - c_idx_2 *
            params->contact_normal[1]) * Fleg[0] + (c_idx_2 *
            params->contact_normal[0] - c_idx_0 * params->contact_normal[2]) *
           Fleg[1]) + (c_idx_0 * params->contact_normal[1] - c_idx_1 *
                       params->contact_normal[0]) * Fleg[2];
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
  varargout_1->data[i] = b_y - Fleg_max;

  /* (Fun < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    i = pz->size[0] * pz->size[1];
    pz->size[0] = 1;
    pz->size[1] = varargout_1->size[1] + 1;
    emxEnsureCapacity_real_T(pz, i);
    loop_ub = varargout_1->size[1];
    for (i = 0; i < loop_ub; i++) {
      pz->data[i] = varargout_1->data[i];
    }

    pz->data[varargout_1->size[1]] = muDoubleScalarSqrt(y * y + scale * scale) -
      mu * b_params;
    i = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = 1;
    varargout_1->size[1] = pz->size[1];
    emxEnsureCapacity_real_T(varargout_1, i);
    loop_ub = pz->size[0] * pz->size[1];
    for (i = 0; i < loop_ub; i++) {
      varargout_1->data[i] = pz->data[i];
    }

    /* friction constraints */
  }

  emxFree_real_T(&pz);

  /*   */
  /*  % debug */
  /*  disp('after Fu') */
  /*  length(ineq) */
  /*  final point  variable slack   */
  /* ineq= [ineq norm(p_f - pf) - x(num_params+N+N_dyn+1)]; */
  /*  4- initial final point  fixed slack  */
  /* *norm(p0 - pf);  */
  b_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1)] - pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1) + 1] - pf[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_y = b_y * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_y += t * t;
  }

  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1) + 2] - pf[2]);
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
  varargout_1->data[i] = b_y - 0.02;

  /* 5 - jump clearance */
  if (!params->obstacle_avoidance) {
    i = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    varargout_1->data[i] = -p->data[3 * ((int32_T)(params->N_dyn / 2.0) - 1)] +
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

real_T c_anon(const real_T p0[3], real_T params_m, real_T params_num_params,
              const char_T params_int_method[3], real_T params_N_dyn, real_T
              params_int_steps, real_T params_b, const real_T params_p_a1[3],
              const real_T params_p_a2[3], real_T params_g, real_T params_w4,
              real_T params_T_th, const emxArray_real_T *x)
{
  emxArray_real_T *Fr_l;
  emxArray_real_T *b_states;
  emxArray_real_T *b_t;
  emxArray_real_T *b_x;
  emxArray_real_T *c_states;
  emxArray_real_T *d_states;
  emxArray_real_T *e_states;
  emxArray_real_T *p;
  emxArray_real_T *pd;
  emxArray_real_T *states;
  real_T dv[6];
  real_T c_x[3];
  real_T absxk;
  real_T b_scale;
  real_T b_y;
  real_T scale;
  real_T t;
  real_T varargout_1;
  real_T y;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T vlen;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&Fr_l, 2, true);
  i = Fr_l->size[0] * Fr_l->size[1];
  Fr_l->size[0] = 1;
  loop_ub = (int32_T)muDoubleScalarFloor(params_N_dyn - 1.0);
  Fr_l->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(Fr_l, i);
  for (i = 0; i <= loop_ub; i++) {
    Fr_l->data[i] = x->data[(int32_T)(params_num_params + (real_T)(i + 1)) - 1];
  }

  scale = (params_num_params + params_N_dyn) + 1.0;
  b_scale = params_num_params + 2.0 * params_N_dyn;
  if (scale > b_scale) {
    i = 0;
    i1 = 0;
  } else {
    i = (int32_T)scale - 1;
    i1 = (int32_T)b_scale;
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

  emxInit_real_T(&b_x, 2, true);
  y = scale * muDoubleScalarSqrt(y);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = y;
  dv[2] = b_y;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  vlen = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  loop_ub = i1 - i;
  b_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_x, vlen);
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_x->data[i1] = x->data[i + i1];
  }

  emxInit_real_T(&states, 2, true);
  emxInit_real_T(&b_t, 2, true);
  c_x[0] = x->data[0];
  c_x[1] = x->data[1];
  c_x[2] = x->data[2];
  computeRollout(dv, x->data[3] / (params_N_dyn - 1.0), params_N_dyn, Fr_l, b_x,
                 c_x, params_int_method, params_int_steps, params_m, params_b,
                 params_p_a1, params_p_a2, params_g, params_T_th, states, b_t);
  vlen = states->size[1];
  i1 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_x, i1);
  for (i1 = 0; i1 < vlen; i1++) {
    b_x->data[i1] = states->data[6 * i1];
  }

  vlen = states->size[1];
  i1 = b_t->size[0] * b_t->size[1];
  b_t->size[0] = 1;
  b_t->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_t, i1);
  for (i1 = 0; i1 < vlen; i1++) {
    b_t->data[i1] = states->data[6 * i1 + 1];
  }

  emxInit_real_T(&b_states, 2, true);
  vlen = states->size[1];
  i1 = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, i1);
  for (i1 = 0; i1 < vlen; i1++) {
    b_states->data[i1] = states->data[6 * i1 + 2];
  }

  emxInit_real_T(&c_states, 2, true);
  vlen = states->size[1];
  i1 = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, i1);
  for (i1 = 0; i1 < vlen; i1++) {
    c_states->data[i1] = states->data[6 * i1 + 3];
  }

  emxInit_real_T(&d_states, 2, true);
  vlen = states->size[1];
  i1 = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, i1);
  for (i1 = 0; i1 < vlen; i1++) {
    d_states->data[i1] = states->data[6 * i1 + 4];
  }

  emxInit_real_T(&e_states, 2, true);
  vlen = states->size[1];
  i1 = e_states->size[0] * e_states->size[1];
  e_states->size[0] = 1;
  e_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(e_states, i1);
  for (i1 = 0; i1 < vlen; i1++) {
    e_states->data[i1] = states->data[6 * i1 + 5];
  }

  emxFree_real_T(&states);
  emxInit_real_T(&p, 2, true);
  emxInit_real_T(&pd, 2, true);
  computePositionVelocity(params_b, b_x, b_t, b_states, c_states, d_states,
    e_states, p, pd);

  /*  be careful there are only N values in this vector the path migh be */
  /*  underestimated! */
  /*      deltax = diff(p(1,:));  % diff(X); */
  /*      deltay = diff(p(2,:));   % diff(Y); */
  /*      deltaz = diff(p(3,:));    % diff(Z); */
  /*      path_length = sum(sqrt(deltax.^2 + deltay.^2 + deltaz.^2)); */
  /* minimize the final kin energy at contact */
  /*  minimize hoist work / energy consumption (doe */
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
  /*  cost =  params.w1 *hoist_work +   params.w4 *smooth ;% 72 iter */
  i1 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_x, i1);
  emxFree_real_T(&e_states);
  emxFree_real_T(&d_states);
  emxFree_real_T(&c_states);
  emxFree_real_T(&b_states);
  emxFree_real_T(&pd);
  emxFree_real_T(&p);
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_x->data[i1] = x->data[i + i1];
  }

  diff(b_x, b_t);
  vlen = b_t->size[1];
  emxFree_real_T(&b_x);
  if (b_t->size[1] == 0) {
    y = 0.0;
  } else {
    y = b_t->data[0];
    for (loop_ub = 2; loop_ub <= vlen; loop_ub++) {
      y += b_t->data[loop_ub - 1];
    }
  }

  diff(Fr_l, b_t);
  vlen = b_t->size[1];
  emxFree_real_T(&Fr_l);
  if (b_t->size[1] == 0) {
    b_y = 0.0;
  } else {
    b_y = b_t->data[0];
    for (loop_ub = 2; loop_ub <= vlen; loop_ub++) {
      b_y += b_t->data[loop_ub - 1];
    }
  }

  emxFree_real_T(&b_t);
  varargout_1 = params_w4 * (y + b_y);

  /*  27 iter */
  /*  cost =    params.w4 *smooth_correct ;% 96 iter */
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

void optimize_cpp(const real_T p0[3], const real_T pf[3], real_T Fleg_max,
                  real_T Fr_max, real_T mu, const param *params, struct0_T
                  *solution)
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
  real_T dv[6];
  real_T b_this_tunableEnvironment_f2[3];
  real_T this_tunableEnvironment_f1[3];
  real_T this_tunableEnvironment_f2[3];
  real_T x[3];
  real_T EXITFLAG;
  real_T absxk;
  real_T b_expl_temp;
  real_T b_scale;
  real_T expl_temp;
  real_T final_cost;
  real_T l1_tmp;
  real_T l2_tmp;
  real_T scale;
  real_T t;
  int32_T i;
  int32_T k;
  int32_T loop_ub;
  int32_T nx;
  char_T c_expl_temp[3];
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

  emxInit_real_T(&b_Fleg_max, 2, true);
  l1_tmp = scale * muDoubleScalarSqrt(l1_tmp);
  l2_tmp = b_scale * muDoubleScalarSqrt(l2_tmp);

  /* pendulum period */
  /*  half period TODO replace with linearized x0(2) = l10 */
  /* opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r */
  /*  % does not always satisfy bounds */
  tic();
  x[0] = p0[0];
  this_tunableEnvironment_f2[0] = pf[0];
  this_tunableEnvironment_f1[0] = p0[0];
  b_this_tunableEnvironment_f2[0] = pf[0];
  x[1] = p0[1];
  this_tunableEnvironment_f2[1] = pf[1];
  this_tunableEnvironment_f1[1] = p0[1];
  b_this_tunableEnvironment_f2[1] = pf[1];
  x[2] = p0[2];
  this_tunableEnvironment_f2[2] = pf[2];
  this_tunableEnvironment_f1[2] = p0[2];
  b_this_tunableEnvironment_f2[2] = pf[2];
  i = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  k = ((int32_T)params->N_dyn + (int32_T)params->N_dyn) + 4;
  b_Fleg_max->size[1] = k;
  emxEnsureCapacity_real_T(b_Fleg_max, i);
  b_Fleg_max->data[0] = Fleg_max;
  b_Fleg_max->data[1] = Fleg_max;
  b_Fleg_max->data[2] = Fleg_max;
  b_Fleg_max->data[3] = 6.2831853071795862 * muDoubleScalarSqrt(l1_tmp /
    params->g) / 4.0;
  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    b_Fleg_max->data[i + 4] = 0.0;
  }

  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    b_Fleg_max->data[(i + (int32_T)params->N_dyn) + 4] = 0.0;
  }

  emxInit_real_T(&c_Fleg_max, 2, true);
  i = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = k;
  emxEnsureCapacity_real_T(c_Fleg_max, i);
  c_Fleg_max->data[0] = -Fleg_max;
  c_Fleg_max->data[1] = -Fleg_max;
  c_Fleg_max->data[2] = -Fleg_max;
  c_Fleg_max->data[3] = 0.01;
  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    c_Fleg_max->data[i + 4] = -Fr_max;
  }

  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    c_Fleg_max->data[(i + (int32_T)params->N_dyn) + 4] = -Fr_max;
  }

  emxInit_real_T(&d_Fleg_max, 2, true);
  i = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = k;
  emxEnsureCapacity_real_T(d_Fleg_max, i);
  d_Fleg_max->data[0] = Fleg_max;
  d_Fleg_max->data[1] = Fleg_max;
  d_Fleg_max->data[2] = Fleg_max;
  d_Fleg_max->data[3] = rtInf;
  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    d_Fleg_max->data[i + 4] = 0.0;
  }

  loop_ub = (int32_T)params->N_dyn;
  for (i = 0; i < loop_ub; i++) {
    d_Fleg_max->data[(i + (int32_T)params->N_dyn) + 4] = 0.0;
  }

  emxInit_real_T(&b_x, 2, true);
  fmincon(x, this_tunableEnvironment_f2, params, b_Fleg_max, c_Fleg_max,
          d_Fleg_max, this_tunableEnvironment_f1, b_this_tunableEnvironment_f2,
          Fleg_max, Fr_max, mu, params, b_x, &final_cost, &EXITFLAG, &expl_temp,
          &b_expl_temp, c_expl_temp, &scale, &b_scale, &absxk, &t);
  toc();

  /* eval trajectory */
  solution->Fleg[0] = b_x->data[0];
  solution->Fleg[1] = b_x->data[1];
  solution->Fleg[2] = b_x->data[2];
  i = solution->Fr_l->size[0] * solution->Fr_l->size[1];
  solution->Fr_l->size[0] = 1;
  loop_ub = (int32_T)muDoubleScalarFloor(params->N_dyn - 1.0);
  solution->Fr_l->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(solution->Fr_l, i);
  for (i = 0; i <= loop_ub; i++) {
    solution->Fr_l->data[i] = b_x->data[(int32_T)(params->num_params + (real_T)
      (i + 1)) - 1];
  }

  scale = (params->num_params + params->N_dyn) + 1.0;
  final_cost = params->num_params + 2.0 * params->N_dyn;
  if (scale > final_cost) {
    i = 0;
    k = 0;
  } else {
    i = (int32_T)scale - 1;
    k = (int32_T)final_cost;
  }

  /*  single shooting */
  dv[0] = muDoubleScalarAtan2(p0[0], -p0[2]);
  dv[1] = l1_tmp;
  dv[2] = l2_tmp;
  dv[3] = 0.0;
  dv[4] = 0.0;
  dv[5] = 0.0;
  nx = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  loop_ub = k - i;
  b_Fleg_max->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_Fleg_max, nx);
  for (k = 0; k < loop_ub; k++) {
    b_Fleg_max->data[k] = b_x->data[i + k];
  }

  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->Fr_l->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  nx = solution->Fr_l->size[0] * solution->Fr_l->size[1] - 1;
  for (k = 0; k <= nx; k++) {
    c_Fleg_max->data[k] = solution->Fr_l->data[k];
  }

  emxInit_real_T(&states, 2, true);
  computeRollout(dv, b_x->data[3] / (params->N_dyn - 1.0), params->N_dyn,
                 c_Fleg_max, b_Fleg_max, solution->Fleg, params->int_method,
                 params->int_steps, params->m, params->b, params->p_a1,
                 params->p_a2, params->g, params->T_th, states, solution->time);
  nx = states->size[1];
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  for (k = 0; k < nx; k++) {
    b_Fleg_max->data[k] = states->data[6 * k];
  }

  nx = states->size[1];
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  for (k = 0; k < nx; k++) {
    c_Fleg_max->data[k] = states->data[6 * k + 1];
  }

  nx = states->size[1];
  k = d_Fleg_max->size[0] * d_Fleg_max->size[1];
  d_Fleg_max->size[0] = 1;
  d_Fleg_max->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_Fleg_max, k);
  for (k = 0; k < nx; k++) {
    d_Fleg_max->data[k] = states->data[6 * k + 2];
  }

  emxInit_real_T(&b_states, 2, true);
  nx = states->size[1];
  k = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, k);
  for (k = 0; k < nx; k++) {
    b_states->data[k] = states->data[6 * k + 3];
  }

  emxInit_real_T(&c_states, 2, true);
  nx = states->size[1];
  k = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_states, k);
  for (k = 0; k < nx; k++) {
    c_states->data[k] = states->data[6 * k + 4];
  }

  emxInit_real_T(&d_states, 2, true);
  nx = states->size[1];
  k = d_states->size[0] * d_states->size[1];
  d_states->size[0] = 1;
  d_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(d_states, k);
  for (k = 0; k < nx; k++) {
    d_states->data[k] = states->data[6 * k + 5];
  }

  emxInit_real_T(&pd, 2, true);
  computePositionVelocity(params->b, b_Fleg_max, c_Fleg_max, d_Fleg_max,
    b_states, c_states, d_states, solution->p, pd);

  /*  init struct foc C++ code generation */
  /* compute path */
  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  nx = solution->p->size[1];
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  emxFree_real_T(&d_states);
  for (k = 0; k < nx; k++) {
    c_Fleg_max->data[k] = solution->p->data[3 * k];
  }

  diff(c_Fleg_max, d_Fleg_max);
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  b_Fleg_max->size[1] = d_Fleg_max->size[1];
  emxEnsureCapacity_real_T(b_Fleg_max, k);
  nx = d_Fleg_max->size[1];
  for (k = 0; k < nx; k++) {
    scale = d_Fleg_max->data[k];
    b_Fleg_max->data[k] = scale * scale;
  }

  nx = solution->p->size[1];
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  for (k = 0; k < nx; k++) {
    c_Fleg_max->data[k] = solution->p->data[3 * k + 1];
  }

  diff(c_Fleg_max, d_Fleg_max);
  k = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = d_Fleg_max->size[1];
  emxEnsureCapacity_real_T(b_states, k);
  nx = d_Fleg_max->size[1];
  for (k = 0; k < nx; k++) {
    scale = d_Fleg_max->data[k];
    b_states->data[k] = scale * scale;
  }

  nx = solution->p->size[1];
  k = c_Fleg_max->size[0] * c_Fleg_max->size[1];
  c_Fleg_max->size[0] = 1;
  c_Fleg_max->size[1] = solution->p->size[1];
  emxEnsureCapacity_real_T(c_Fleg_max, k);
  for (k = 0; k < nx; k++) {
    c_Fleg_max->data[k] = solution->p->data[3 * k + 2];
  }

  diff(c_Fleg_max, d_Fleg_max);
  k = c_states->size[0] * c_states->size[1];
  c_states->size[0] = 1;
  c_states->size[1] = d_Fleg_max->size[1];
  emxEnsureCapacity_real_T(c_states, k);
  nx = d_Fleg_max->size[1];
  emxFree_real_T(&c_Fleg_max);
  for (k = 0; k < nx; k++) {
    scale = d_Fleg_max->data[k];
    c_states->data[k] = scale * scale;
  }

  emxFree_real_T(&d_Fleg_max);
  k = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  nx = b_Fleg_max->size[0] * b_Fleg_max->size[1];
  b_Fleg_max->size[0] = 1;
  emxEnsureCapacity_real_T(b_Fleg_max, nx);
  nx = k - 1;
  for (k = 0; k <= nx; k++) {
    b_Fleg_max->data[k] = (b_Fleg_max->data[k] + b_states->data[k]) +
      c_states->data[k];
  }

  emxFree_real_T(&c_states);
  emxFree_real_T(&b_states);
  nx = b_Fleg_max->size[1];
  for (k = 0; k < nx; k++) {
    b_Fleg_max->data[k] = muDoubleScalarSqrt(b_Fleg_max->data[k]);
  }

  /*  check length is always l */
  /*      a = vecnorm(p) */
  /*      a -  ones(1,length(a))*l */
  k = solution->Ekin->size[0] * solution->Ekin->size[1];
  solution->Ekin->size[0] = 1;
  solution->Ekin->size[1] = solution->time->size[1];
  emxEnsureCapacity_real_T(solution->Ekin, k);
  nx = solution->time->size[1];
  for (k = 0; k < nx; k++) {
    solution->Ekin->data[k] = 0.0;
  }

  EXITFLAG = 0.0;

  /*  kinetic energy at the beginning */
  expl_temp = params->m / 2.0;
  k = solution->time->size[1];
  for (nx = 0; nx < k; nx++) {
    scale = pd->data[3 * nx];
    b_expl_temp = expl_temp * scale * scale;
    scale = pd->data[3 * nx + 1];
    b_expl_temp += expl_temp * scale * scale;
    scale = pd->data[3 * nx + 2];
    b_expl_temp += expl_temp * scale * scale;
    solution->Ekin->data[nx] = b_expl_temp;
    EXITFLAG += b_expl_temp * 0.001;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  nx = b_Fleg_max->size[1];
  if (b_Fleg_max->size[1] == 0) {
    b_expl_temp = 0.0;
  } else {
    b_expl_temp = b_Fleg_max->data[0];
    for (k = 2; k <= nx; k++) {
      b_expl_temp += b_Fleg_max->data[k - 1];
    }
  }

  emxFree_real_T(&b_Fleg_max);
  solution->path_length = b_expl_temp;
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
    final_cost = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    final_cost = t * t;
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

  absxk = muDoubleScalarAbs(solution->p->data[3 * (solution->p->size[1] - 1) + 1]
    - pf[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    final_cost = final_cost * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    final_cost += t * t;
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

  absxk = muDoubleScalarAbs(solution->p->data[3 * (solution->p->size[1] - 1) + 2]
    - pf[2]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    final_cost = final_cost * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    final_cost += t * t;
  }

  solution->initial_error = scale * muDoubleScalarSqrt(b_expl_temp);
  solution->final_error_real = b_scale * muDoubleScalarSqrt(final_cost);
  k = solution->Fr_r->size[0] * solution->Fr_r->size[1];
  solution->Fr_r->size[0] = 1;
  solution->Fr_r->size[1] = loop_ub;
  emxEnsureCapacity_real_T(solution->Fr_r, k);
  for (k = 0; k < loop_ub; k++) {
    solution->Fr_r->data[k] = b_x->data[i + k];
  }

  loop_ub = states->size[1];
  i = solution->psi->size[0] * solution->psi->size[1];
  solution->psi->size[0] = 1;
  solution->psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psi, i);
  for (i = 0; i < loop_ub; i++) {
    solution->psi->data[i] = states->data[6 * i];
  }

  loop_ub = states->size[1];
  i = solution->l1->size[0] * solution->l1->size[1];
  solution->l1->size[0] = 1;
  solution->l1->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1, i);
  for (i = 0; i < loop_ub; i++) {
    solution->l1->data[i] = states->data[6 * i + 1];
  }

  loop_ub = states->size[1];
  i = solution->l2->size[0] * solution->l2->size[1];
  solution->l2->size[0] = 1;
  solution->l2->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2, i);
  for (i = 0; i < loop_ub; i++) {
    solution->l2->data[i] = states->data[6 * i + 2];
  }

  loop_ub = states->size[1];
  i = solution->psid->size[0] * solution->psid->size[1];
  solution->psid->size[0] = 1;
  solution->psid->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->psid, i);
  for (i = 0; i < loop_ub; i++) {
    solution->psid->data[i] = states->data[6 * i + 3];
  }

  loop_ub = states->size[1];
  i = solution->l1d->size[0] * solution->l1d->size[1];
  solution->l1d->size[0] = 1;
  solution->l1d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l1d, i);
  for (i = 0; i < loop_ub; i++) {
    solution->l1d->data[i] = states->data[6 * i + 4];
  }

  loop_ub = states->size[1];
  i = solution->l2d->size[0] * solution->l2d->size[1];
  solution->l2d->size[0] = 1;
  solution->l2d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution->l2d, i);
  for (i = 0; i < loop_ub; i++) {
    solution->l2d->data[i] = states->data[6 * i + 5];
  }

  emxFree_real_T(&states);
  solution->Tf = b_x->data[3];
  solution->Etot = 0.0;
  solution->Ekin0x = expl_temp * pd->data[0] * pd->data[0];
  solution->Ekin0y = expl_temp * pd->data[1] * pd->data[1];
  solution->Ekin0z = expl_temp * pd->data[2] * pd->data[2];
  solution->intEkin = EXITFLAG;
  solution->U0 = 0.0;
  solution->Uf = 0.0;
  solution->Ekinfx = expl_temp * pd->data[3 * (pd->size[1] - 1)] * pd->data[3 *
    (pd->size[1] - 1)];
  solution->Ekinfy = expl_temp * pd->data[3 * (pd->size[1] - 1) + 1] * pd->data
    [3 * (pd->size[1] - 1) + 1];
  solution->Ekinfz = expl_temp * pd->data[3 * (pd->size[1] - 1) + 2] * pd->data
    [3 * (pd->size[1] - 1) + 2];
  solution->achieved_target[0] = solution->p->data[3 * (solution->p->size[1] - 1)];
  scale = pd->data[0];
  b_expl_temp = expl_temp * scale * scale;
  solution->achieved_target[1] = solution->p->data[3 * (solution->p->size[1] - 1)
    + 1];
  scale = pd->data[1];
  b_expl_temp += expl_temp * scale * scale;
  solution->achieved_target[2] = solution->p->data[3 * (solution->p->size[1] - 1)
    + 2];
  scale = pd->data[2];
  b_expl_temp += expl_temp * scale * scale;
  solution->Ekin0 = b_expl_temp;
  solution->Ekinf = (expl_temp * pd->data[3 * (pd->size[1] - 1)] * pd->data[3 *
                     (pd->size[1] - 1)] + expl_temp * pd->data[3 * (pd->size[1]
    - 1) + 1] * pd->data[3 * (pd->size[1] - 1) + 1]) + expl_temp * pd->data[3 *
    (pd->size[1] - 1) + 2] * pd->data[3 * (pd->size[1] - 1) + 2];
  solution->T_th = params->T_th;

  /* save('test_matlab2.mat','solution','mu','Fleg_max', 'Fr_max', 'p0','pf'); */
  emxFree_real_T(&pd);
  emxFree_real_T(&b_x);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp.c) */
