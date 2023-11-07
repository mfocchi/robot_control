/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeObjective_.c
 *
 * Code generation for function 'computeObjective_'
 *
 */

/* Include files */
#include "computeObjective_.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo xc_emlrtRSI = { 1,  /* lineNo */
  "computeObjective_",                 /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+utils/+ObjNonlinEvaluator/computeObjective_.p"/* pathName */
};

/* Function Definitions */
void computeObjective_(const emlrtStack *sp, const real_T
  c_obj_objfun_tunableEnvironment[3], real_T d_obj_objfun_tunableEnvironment,
  real_T e_obj_objfun_tunableEnvironment, const char_T
  f_obj_objfun_tunableEnvironment[3], real_T g_obj_objfun_tunableEnvironment,
  real_T h_obj_objfun_tunableEnvironment, real_T i_obj_objfun_tunableEnvironment,
  const real_T j_obj_objfun_tunableEnvironment[3], const real_T
  k_obj_objfun_tunableEnvironment[3], real_T l_obj_objfun_tunableEnvironment,
  real_T m_obj_objfun_tunableEnvironment, real_T n_obj_objfun_tunableEnvironment,
  const emxArray_real_T *x, real_T *fval, int32_T *status)
{
  emlrtStack b_st;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &xc_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &q_emlrtRSI;
  *fval = c_anon(&b_st, c_obj_objfun_tunableEnvironment,
                 d_obj_objfun_tunableEnvironment,
                 e_obj_objfun_tunableEnvironment,
                 f_obj_objfun_tunableEnvironment,
                 g_obj_objfun_tunableEnvironment,
                 h_obj_objfun_tunableEnvironment,
                 i_obj_objfun_tunableEnvironment,
                 j_obj_objfun_tunableEnvironment,
                 k_obj_objfun_tunableEnvironment,
                 l_obj_objfun_tunableEnvironment,
                 m_obj_objfun_tunableEnvironment,
                 n_obj_objfun_tunableEnvironment, x);
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
}

/* End of code generation (computeObjective_.c) */
