/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * evalObjAndConstr.h
 *
 * Code generation for function 'evalObjAndConstr'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
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
                      real_T *fval, int32_T *status);

/* End of code generation (evalObjAndConstr.h) */
