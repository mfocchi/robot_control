/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeFval.c
 *
 * Code generation for function 'computeFval'
 *
 */

/* Include files */
#include "computeFval.h"
#include "linearForm_.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T computeFval(const i_struct_T *obj, emxArray_real_T *workspace, const
                   emxArray_real_T *H, const emxArray_real_T *f, const
                   emxArray_real_T *x)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  real_T val;
  int32_T i;
  int32_T i1;
  int32_T idx;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x->data[obj->nvar - 1];
    break;

   case 3:
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    if (obj->nvar < 1) {
      val = 0.0;
    } else {
      n_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      val = ddot(&n_t, &x->data[0], &incx_t, &workspace->data[0], &incy_t);
    }
    break;

   default:
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    i = obj->nvar + 1;
    i1 = obj->maxVar - 1;
    for (idx = i; idx <= i1; idx++) {
      workspace->data[idx - 1] = 0.5 * obj->beta * x->data[idx - 1] + obj->rho;
    }

    if (obj->maxVar - 1 < 1) {
      val = 0.0;
    } else {
      n_t = (ptrdiff_t)(obj->maxVar - 1);
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      val = ddot(&n_t, &x->data[0], &incx_t, &workspace->data[0], &incy_t);
    }
    break;
  }

  return val;
}

/* End of code generation (computeFval.c) */
