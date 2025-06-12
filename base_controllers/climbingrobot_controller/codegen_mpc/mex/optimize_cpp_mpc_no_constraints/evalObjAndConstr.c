/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * evalObjAndConstr.c
 *
 * Code generation for function 'evalObjAndConstr'
 *
 */

/* Include files */
#include "evalObjAndConstr.h"
#include "optimize_cpp_mpc_no_constraints.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void evalObjAndConstr(const real_T c_obj_objfun_tunableEnvironment[6], const
                      emxArray_real_T *d_obj_objfun_tunableEnvironment, const
                      emxArray_real_T *e_obj_objfun_tunableEnvironment, const
                      emxArray_real_T *f_obj_objfun_tunableEnvironment, int64_T
                      g_obj_objfun_tunableEnvironment, const char_T
                      h_obj_objfun_tunableEnvironment[3], real_T
                      i_obj_objfun_tunableEnvironment, real_T
                      j_obj_objfun_tunableEnvironment, const real_T
                      k_obj_objfun_tunableEnvironment[3], const real_T
                      l_obj_objfun_tunableEnvironment[3], real_T
                      m_obj_objfun_tunableEnvironment, real_T
                      n_obj_objfun_tunableEnvironment, real_T
                      o_obj_objfun_tunableEnvironment, real_T
                      p_obj_objfun_tunableEnvironment, real_T
                      q_obj_objfun_tunableEnvironment, const emxArray_real_T *x,
                      real_T *fval, int32_T *status)
{
  *fval = anon(c_obj_objfun_tunableEnvironment, d_obj_objfun_tunableEnvironment,
               e_obj_objfun_tunableEnvironment, f_obj_objfun_tunableEnvironment,
               g_obj_objfun_tunableEnvironment, h_obj_objfun_tunableEnvironment,
               i_obj_objfun_tunableEnvironment, j_obj_objfun_tunableEnvironment,
               k_obj_objfun_tunableEnvironment, l_obj_objfun_tunableEnvironment,
               m_obj_objfun_tunableEnvironment, n_obj_objfun_tunableEnvironment,
               o_obj_objfun_tunableEnvironment, p_obj_objfun_tunableEnvironment,
               q_obj_objfun_tunableEnvironment, x);
  *status = 1;
  if (muDoubleScalarIsInf(*fval) || muDoubleScalarIsNaN(*fval)) {
    if (muDoubleScalarIsNaN(*fval)) {
      *status = -6;
    } else if (*fval < 0.0) {
      *status = -4;
    } else {
      *status = -5;
    }
  }

  if (*status == 1) {
    *status = 1;
  }
}

/* End of code generation (evalObjAndConstr.c) */
