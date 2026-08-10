/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * constraints.c
 *
 * Code generation for function 'constraints'
 *
 */

/* Include files */
#include "constraints.h"
#include "computePositionVelocity.h"
#include "computeRollout.h"
#include "interp2.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_mexutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void constraints(const emxArray_real_T *x, const real_T p0[3],
                 const real_T pf[3], real_T Fleg_max, real_T mu,
                 const param *params, emxArray_real_T *ineq,
                 struct2_T *number_of_constr, struct3_T *solution_constr)
{
  static const int32_T b_iv[2] = {1, 13};
  static const char_T b[4] = {'m', 'e', 's', 'h'};
  static const char_T b_b[4] = {'n', 'o', 'n', 'e'};
  emxArray_real_T *b_states;
  emxArray_real_T *b_x;
  emxArray_real_T *c_x;
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
  real_T *ineq_data;
  real_T *states_data;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  int32_T number_of_constr_tmp;
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
  ineq->size[0] = 1;
  ineq->size[1] = 0;
  /*  number of constraints */
  number_of_constr->wall_constraints = params->N_dyn;
  number_of_constr->retraction_force_constraints = 0.0;
  /*  already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes
   */
  if (params->FRICTION_CONE != 0.0) {
    number_of_constr->force_constraints = 3.0;
  } else {
    number_of_constr->force_constraints = 2.0;
    /* unilateral and actuation */
  }
  number_of_constr->final_constraints = 5.0;
  number_of_constr_tmp =
      memcmp((char_T *)&params->obstacle_avoidance[0], (char_T *)&b[0], 4);
  number_of_constr->via_point = (number_of_constr_tmp == 0);
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
  computeRollout(dv, x_data[3] / (params->N_dyn - 1.0), params->N_dyn, b_x, c_x,
                 Fleg, params->int_method, params->int_steps, params->m,
                 params->b, params->p_a1, params->p_a2, params->g, params->T_th,
                 states, solution_constr->time);
  ineq_data = states->data;
  i = solution_constr->psid->size[0] * solution_constr->psid->size[1];
  solution_constr->psid->size[0] = 1;
  solution_constr->psid->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution_constr->psid, i);
  loop_ub = states->size[1];
  i = solution_constr->l1d->size[0] * solution_constr->l1d->size[1];
  solution_constr->l1d->size[0] = 1;
  solution_constr->l1d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution_constr->l1d, i);
  i = solution_constr->l2d->size[0] * solution_constr->l2d->size[1];
  solution_constr->l2d->size[0] = 1;
  solution_constr->l2d->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution_constr->l2d, i);
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_x, i);
  b_x_data = b_x->data;
  i = c_x->size[0] * c_x->size[1];
  c_x->size[0] = 1;
  c_x->size[1] = states->size[1];
  emxEnsureCapacity_real_T(c_x, i);
  c_x_data = c_x->data;
  emxInit_real_T(&b_states, 2);
  i = b_states->size[0] * b_states->size[1];
  b_states->size[0] = 1;
  b_states->size[1] = states->size[1];
  emxEnsureCapacity_real_T(b_states, i);
  states_data = b_states->data;
  for (i = 0; i < loop_ub; i++) {
    solution_constr->psid->data[i] = ineq_data[6 * i + 3];
    solution_constr->l1d->data[i] = ineq_data[6 * i + 4];
    solution_constr->l2d->data[i] = ineq_data[6 * i + 5];
    b_x_data[i] = ineq_data[6 * i];
    c_x_data[i] = ineq_data[6 * i + 1];
    states_data[i] = ineq_data[6 * i + 2];
  }
  computePositionVelocity(params->b, b_x, c_x, b_states, solution_constr->p);
  emxFree_real_T(&b_states);
  emxFree_real_T(&c_x);
  emxFree_real_T(&b_x);
  /* only position */
  /*  I assume px py pz  are row vectors */
  /*  init struct foc C++ code generation */
  i = solution_constr->psi->size[0] * solution_constr->psi->size[1];
  solution_constr->psi->size[0] = 1;
  solution_constr->psi->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution_constr->psi, i);
  loop_ub = states->size[1];
  i = solution_constr->l1->size[0] * solution_constr->l1->size[1];
  solution_constr->l1->size[0] = 1;
  solution_constr->l1->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution_constr->l1, i);
  i = solution_constr->l2->size[0] * solution_constr->l2->size[1];
  solution_constr->l2->size[0] = 1;
  solution_constr->l2->size[1] = states->size[1];
  emxEnsureCapacity_real_T(solution_constr->l2, i);
  for (i = 0; i < loop_ub; i++) {
    solution_constr->psi->data[i] = ineq_data[6 * i];
    solution_constr->l1->data[i] = ineq_data[6 * i + 1];
    solution_constr->l2->data[i] = ineq_data[6 * i + 2];
  }
  emxFree_real_T(&states);
  /*  1 -N_dyn  constraint to do not enter the wall, p_x >=0 */
  if (memcmp((char_T *)&params->obstacle_avoidance[0], (char_T *)&b_b[0], 4) ==
      0) {
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      i1 = ineq->size[1];
      i2 = ineq->size[0] * ineq->size[1];
      ineq->size[1]++;
      emxEnsureCapacity_real_T(ineq, i2);
      ineq_data = ineq->data;
      ineq_data[i1] = -solution_constr->p->data[3 * b_i];
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else if (number_of_constr_tmp == 0) {
    /* p_x > wall_x + jump_clearance => p_x -wall_z- jump_clearance  >0 => -p_x
     * +wall_z + jump_clearance  <0 */
    i = (int32_T)params->N_dyn;
    for (b_i = 0; b_i < i; b_i++) {
      i1 = ineq->size[1];
      i2 = ineq->size[0] * ineq->size[1];
      ineq->size[0] = 1;
      ineq->size[1]++;
      emxEnsureCapacity_real_T(ineq, i2);
      ineq_data = ineq->data;
      ineq_data[i1] = -solution_constr->p->data[3 * b_i] +
                      interp2(params->mesh_z, params->mesh_y, params->mesh_x,
                              solution_constr->p->data[3 * b_i + 2],
                              solution_constr->p->data[3 * b_i + 1]);
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
  i = ineq->size[1];
  i1 = ineq->size[0] * ineq->size[1];
  ineq->size[1]++;
  emxEnsureCapacity_real_T(ineq, i1);
  ineq_data = ineq->data;
  ineq_data[i] = -b_params;
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
  i = ineq->size[1];
  i1 = ineq->size[0] * ineq->size[1];
  ineq->size[1]++;
  emxEnsureCapacity_real_T(ineq, i1);
  ineq_data = ineq->data;
  ineq_data[i] = b_y - Fleg_max;
  /* (|Fun| < fun max ) actuation */
  if (params->FRICTION_CONE != 0.0) {
    /* |Fut| < mu*Fun */
    i = ineq->size[1];
    i1 = ineq->size[0] * ineq->size[1];
    ineq->size[0] = 1;
    ineq->size[1]++;
    emxEnsureCapacity_real_T(ineq, i1);
    ineq_data = ineq->data;
    ineq_data[i] = muDoubleScalarSqrt(y * y + scale * scale) - mu * b_params;
    /* friction constraints */
  }
  /* 4- landing constraint, landing point inside the selected patch */
  /* constraint Y direction inside patch  bounds  */
  /* 4.1 p_f(y) < ymax => p_f(2) -ymax < 0 */
  i = ineq->size[1];
  i1 = ineq->size[0] * ineq->size[1];
  ineq->size[1]++;
  emxEnsureCapacity_real_T(ineq, i1);
  ineq_data = ineq->data;
  scale = solution_constr->p->data[3 * (solution_constr->p->size[1] - 1) + 1];
  y = params->patch_side / 2.0;
  ineq_data[i] = scale - (pf[1] + y);
  /* 4.2 p_f(y) > ymin => -p_f(2) < -ymin => -p_f(2) + ymin<0 */
  i = ineq->size[1];
  i1 = ineq->size[0] * ineq->size[1];
  ineq->size[1]++;
  emxEnsureCapacity_real_T(ineq, i1);
  ineq_data = ineq->data;
  ineq_data[i] = -scale + (pf[1] - y);
  /* constraint Z direction inside patch  bounds */
  /* 4.3 p_f(z) < zmax => p_f(3) -zmax < 0 */
  i = ineq->size[1];
  i1 = ineq->size[0] * ineq->size[1];
  ineq->size[1]++;
  emxEnsureCapacity_real_T(ineq, i1);
  ineq_data = ineq->data;
  b_scale = solution_constr->p->data[3 * (solution_constr->p->size[1] - 1) + 2];
  ineq_data[i] = b_scale - (pf[2] + y);
  /* 4.4 p_f(z) > zmin => -p_f(3) < -zmin => -p_f(3) + zmin<0 */
  i = ineq->size[1];
  i1 = ineq->size[0] * ineq->size[1];
  ineq->size[1]++;
  emxEnsureCapacity_real_T(ineq, i1);
  ineq_data = ineq->data;
  ineq_data[i] = -b_scale + (pf[2] - y);
  /* 4.5 constraint the X ||pf(x) -wall_x||<fixed_slack (otherwise it finds
   * something in the air!) */
  i = ineq->size[1];
  i1 = ineq->size[0] * ineq->size[1];
  ineq->size[0] = 1;
  ineq->size[1]++;
  emxEnsureCapacity_real_T(ineq, i1);
  ineq_data = ineq->data;
  ineq_data[i] =
      muDoubleScalarAbs(
          solution_constr->p->data[3 * (solution_constr->p->size[1] - 1)] -
          interp2(params->mesh_z, params->mesh_y, params->mesh_x, b_scale,
                  scale)) -
      0.02;
  /* old way  */
  /*  ineq= [ineq (norm(p_f - pf) - fixed_slack)]; */
  /*  number_of_constr.final_constraints =  1; */
  /* 5 - jump clearance p_x > jump_clearance */
  if (number_of_constr->via_point > 0.0) {
    i = ineq->size[1];
    i1 = ineq->size[0] * ineq->size[1];
    ineq->size[0] = 1;
    ineq->size[1]++;
    emxEnsureCapacity_real_T(ineq, i1);
    ineq_data = ineq->data;
    b_i = (int32_T)(params->N_dyn / 2.0) - 1;
    ineq_data[i] = (-solution_constr->p->data[3 * b_i] +
                    interp2(params->mesh_z, params->mesh_y, params->mesh_x,
                            solution_constr->p->data[3 * b_i + 2],
                            solution_constr->p->data[3 * b_i + 1])) +
                   params->jump_clearance;
  }
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

/* End of code generation (constraints.c) */
