/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * constraints.h
 *
 * Code generation for function 'constraints'
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
void constraints(const emxArray_real_T *x, const real_T p0[3],
                 const real_T pf[3], real_T Fleg_max, real_T mu,
                 const param *params, emxArray_real_T *ineq,
                 struct2_T *number_of_constr, struct3_T *solution_constr);

/* End of code generation (constraints.h) */
