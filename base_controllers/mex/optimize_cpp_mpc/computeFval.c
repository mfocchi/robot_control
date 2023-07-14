/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
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
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
real_T computeFval(const e_struct_T *obj, emxArray_real_T *workspace,
                   const emxArray_real_T *H, const emxArray_real_T *f,
                   const emxArray_real_T *x)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  const real_T *x_data;
  real_T val;
  real_T *workspace_data;
  int32_T idx;
  x_data = x->data;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x_data[obj->nvar - 1];
    break;
  case 3:
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    workspace_data = workspace->data;
    if (obj->nvar < 1) {
      val = 0.0;
    } else {
      n_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      val = ddot(&n_t, (real_T *)&x_data[0], &incx_t, &workspace_data[0],
                 &incy_t);
    }
    break;
  default: {
    int32_T i;
    int32_T i1;
    int32_T scalarLB;
    int32_T vectorUB;
    linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    workspace_data = workspace->data;
    i = obj->nvar + 1;
    i1 = obj->maxVar - 1;
    scalarLB = ((((i1 - i) + 1) / 2) << 1) + i;
    vectorUB = scalarLB - 2;
    for (idx = i; idx <= vectorUB; idx += 2) {
      _mm_storeu_pd(&workspace_data[idx - 1],
                    _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5 * obj->beta),
                                          _mm_loadu_pd(&x_data[idx - 1])),
                               _mm_set1_pd(obj->rho)));
    }
    for (idx = scalarLB; idx <= i1; idx++) {
      workspace_data[idx - 1] = 0.5 * obj->beta * x_data[idx - 1] + obj->rho;
    }
    if (i1 < 1) {
      val = 0.0;
    } else {
      n_t = (ptrdiff_t)(obj->maxVar - 1);
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      val = ddot(&n_t, (real_T *)&x_data[0], &incx_t, &workspace_data[0],
                 &incy_t);
    }
  } break;
  }
  return val;
}

/* End of code generation (computeFval.c) */
