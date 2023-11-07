/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * evalObjAndConstr.h
 *
 * Code generation for function 'evalObjAndConstr'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
real_T evalObjAndConstr(int32_T c_obj_next_next_next_next_next_,
                        const real_T d_obj_next_next_next_next_next_[3],
                        const real_T e_obj_next_next_next_next_next_[3],
                        real_T f_obj_next_next_next_next_next_,
                        real_T g_obj_next_next_next_next_next_,
                        const param *h_obj_next_next_next_next_next_,
                        const j_struct_T *i_obj_next_next_next_next_next_,
                        const emxArray_real_T *x,
                        emxArray_real_T *Cineq_workspace, int32_T ineq0,
                        int32_T *status);

/* End of code generation (evalObjAndConstr.h) */
