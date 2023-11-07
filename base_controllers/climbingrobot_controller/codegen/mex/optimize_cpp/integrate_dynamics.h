/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * integrate_dynamics.h
 *
 * Code generation for function 'integrate_dynamics'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_anon(real_T params_m, real_T params_b, const real_T params_p_a1[3], const
            real_T params_p_a2[3], real_T params_g, real_T params_T_th, real_T t,
            const real_T x[6], real_T u1, real_T u2, const real_T u3[3], real_T
            varargout_1[6]);

/* End of code generation (integrate_dynamics.h) */
