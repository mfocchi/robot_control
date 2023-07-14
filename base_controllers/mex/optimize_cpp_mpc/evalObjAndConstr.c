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
#include "optimize_cpp_mpc.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
real_T evalObjAndConstr(const i_struct_T *c_obj_next_next_next_next_next_,
                        const emxArray_real_T *x, int32_T *status)
{
  real_T fval;
  boolean_T b;
  fval = optimize_cpp_mpc_anonFcn1(
      c_obj_next_next_next_next_next_->actual_state,
      c_obj_next_next_next_next_next_->ref_com,
      c_obj_next_next_next_next_next_->Fr_l0,
      c_obj_next_next_next_next_next_->Fr_r0,
      c_obj_next_next_next_next_next_->mpc_N,
      c_obj_next_next_next_next_next_->params.int_method,
      c_obj_next_next_next_next_next_->params.int_steps,
      c_obj_next_next_next_next_next_->params.b,
      c_obj_next_next_next_next_next_->params.p_a1,
      c_obj_next_next_next_next_next_->params.p_a2,
      c_obj_next_next_next_next_next_->params.g,
      c_obj_next_next_next_next_next_->params.m,
      c_obj_next_next_next_next_next_->params.w1,
      c_obj_next_next_next_next_next_->params.mpc_dt, x);
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
    *status = 1;
  }
  return fval;
}

/* End of code generation (evalObjAndConstr.c) */
