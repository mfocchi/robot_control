/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * evalObjAndConstr.c
 *
 * Code generation for function 'evalObjAndConstr'
 *
 */

/* Include files */
#include "evalObjAndConstr.h"
#include "computeConstraints_.h"
#include "optimize_cpp.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
real_T evalObjAndConstr(int32_T c_obj_next_next_next_next_next_,
                        const real_T d_obj_next_next_next_next_next_[3],
                        const real_T e_obj_next_next_next_next_next_[3],
                        real_T f_obj_next_next_next_next_next_,
                        real_T g_obj_next_next_next_next_next_,
                        const param *h_obj_next_next_next_next_next_,
                        const j_struct_T *i_obj_next_next_next_next_next_,
                        const emxArray_real_T *x,
                        emxArray_real_T *Cineq_workspace, int32_T ineq0,
                        int32_T *status)
{
  real_T fval;
  boolean_T b;
  fval =
      optimize_cpp_anonFcn1(i_obj_next_next_next_next_next_->p0,
                            i_obj_next_next_next_next_next_->params.m,
                            i_obj_next_next_next_next_next_->params.num_params,
                            i_obj_next_next_next_next_next_->params.int_method,
                            i_obj_next_next_next_next_next_->params.N_dyn,
                            i_obj_next_next_next_next_next_->params.int_steps,
                            i_obj_next_next_next_next_next_->params.b,
                            i_obj_next_next_next_next_next_->params.p_a1,
                            i_obj_next_next_next_next_next_->params.p_a2,
                            i_obj_next_next_next_next_next_->params.g,
                            i_obj_next_next_next_next_next_->params.w1,
                            i_obj_next_next_next_next_next_->params.w2,
                            i_obj_next_next_next_next_next_->params.T_th, x);
  *status = 1;
  b = muDoubleScalarIsNaN(fval);
  if (muDoubleScalarIsInf(fval) || b) {
    if (b) {
      *status = -3;
    } else if (fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }
  if (*status == 1) {
    *status = computeConstraints_(
        c_obj_next_next_next_next_next_, d_obj_next_next_next_next_next_,
        e_obj_next_next_next_next_next_, f_obj_next_next_next_next_next_,
        g_obj_next_next_next_next_next_, h_obj_next_next_next_next_next_, x,
        Cineq_workspace, ineq0);
  }
  return fval;
}

/* End of code generation (evalObjAndConstr.c) */
