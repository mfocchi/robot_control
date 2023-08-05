/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeFval_ReuseHx.c
 *
 * Code generation for function 'computeFval_ReuseHx'
 *
 */

/* Include files */
#include "computeFval_ReuseHx.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "xdot.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T computeFval_ReuseHx(const i_struct_T *obj, emxArray_real_T *workspace,
  const emxArray_real_T *f, const emxArray_real_T *x)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  real_T val;
  int32_T i;
  int32_T idx;
  int32_T maxRegVar;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x->data[obj->nvar - 1];
    break;

   case 3:
    if (obj->hasLinear) {
      i = obj->nvar;
      for (idx = 0; idx < i; idx++) {
        workspace->data[idx] = 0.5 * obj->Hx->data[idx] + f->data[idx];
      }

      if (obj->nvar < 1) {
        val = 0.0;
      } else {
        n_t = (ptrdiff_t)obj->nvar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        val = ddot(&n_t, &x->data[0], &incx_t, &workspace->data[0], &incy_t);
      }
    } else {
      val = 0.5 * b_xdot(obj->nvar, x, obj->Hx);
    }
    break;

   default:
    maxRegVar = obj->maxVar - 1;
    if (obj->hasLinear) {
      i = obj->nvar;
      for (idx = 0; idx < i; idx++) {
        workspace->data[idx] = f->data[idx];
      }

      i = obj->maxVar - obj->nvar;
      for (idx = 0; idx <= i - 2; idx++) {
        workspace->data[obj->nvar + idx] = obj->rho;
      }

      for (idx = 0; idx < maxRegVar; idx++) {
        workspace->data[idx] += 0.5 * obj->Hx->data[idx];
      }

      if (obj->maxVar - 1 < 1) {
        val = 0.0;
      } else {
        n_t = (ptrdiff_t)(obj->maxVar - 1);
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        val = ddot(&n_t, &x->data[0], &incx_t, &workspace->data[0], &incy_t);
      }
    } else {
      val = 0.5 * b_xdot(obj->maxVar - 1, x, obj->Hx);
      i = obj->nvar + 1;
      for (idx = i; idx <= maxRegVar; idx++) {
        val += x->data[idx - 1] * obj->rho;
      }
    }
    break;
  }

  return val;
}

/* End of code generation (computeFval_ReuseHx.c) */
