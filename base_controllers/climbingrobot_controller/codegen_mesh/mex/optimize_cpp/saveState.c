/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * saveState.c
 *
 * Code generation for function 'saveState'
 *
 */

/* Include files */
#include "saveState.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void saveState(g_struct_T *obj)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  int32_T i;
  int32_T nVar;
  obj->sqpFval_old = obj->sqpFval;
  nVar = obj->xstarsqp->size[1];
  n_t = (ptrdiff_t)obj->xstarsqp->size[1];
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dcopy(&n_t, &obj->xstarsqp->data[0], &incx_t, &obj->xstarsqp_old->data[0],
        &incy_t);
  for (i = 0; i < nVar; i++) {
    obj->grad_old->data[i] = obj->grad->data[i];
  }
  if (obj->mIneq >= 1) {
    n_t = (ptrdiff_t)obj->mIneq;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &obj->cIneq->data[0], &incx_t, &obj->cIneq_old->data[0],
          &incy_t);
  }
}

/* End of code generation (saveState.c) */
