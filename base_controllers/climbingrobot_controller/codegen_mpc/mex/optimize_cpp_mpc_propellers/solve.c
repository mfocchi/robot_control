/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * solve.c
 *
 * Code generation for function 'solve'
 *
 */

/* Include files */
#include "solve.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "xtrsv.h"
#include <string.h>

/* Function Definitions */
void b_solve(const h_struct_T *obj, emxArray_real_T *rhs)
{
  c_xtrsv(obj->ndims, obj->FMat, obj->ldm, rhs);
  xtrsv(obj->ndims, obj->FMat, obj->ldm, rhs);
}

void solve(const h_struct_T *obj, emxArray_real_T *rhs)
{
  b_xtrsv(obj->ndims, obj->FMat, obj->ldm, rhs);
  xtrsv(obj->ndims, obj->FMat, obj->ldm, rhs);
}

/* End of code generation (solve.c) */
