/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeQ_.c
 *
 * Code generation for function 'computeQ_'
 *
 */

/* Include files */
#include "computeQ_.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeQ_(c_struct_T *obj, int32_T nrows)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  int32_T i;
  int32_T iQR0;
  int32_T idx;
  int32_T n;
  i = obj->minRowCol;
  for (idx = 0; idx < i; idx++) {
    iQR0 = (obj->ldq * idx + idx) + 1;
    n = (obj->mrows - idx) - 1;
    if (n >= 1) {
      n_t = (ptrdiff_t)n;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &obj->QR->data[iQR0], &incx_t, &obj->Q->data[iQR0], &incy_t);
    }
  }
  n_t = LAPACKE_dorgqr(102, (ptrdiff_t)obj->mrows, (ptrdiff_t)nrows,
                       (ptrdiff_t)obj->minRowCol, &obj->Q->data[0],
                       (ptrdiff_t)obj->ldq, &obj->tau->data[0]);
  if ((int32_T)n_t != 0) {
    iQR0 = obj->Q->size[0];
    n = obj->Q->size[1];
    i = obj->Q->size[0] * obj->Q->size[1];
    obj->Q->size[0] = iQR0;
    obj->Q->size[1] = n;
    emxEnsureCapacity_real_T(obj->Q, i);
    iQR0 *= n;
    for (i = 0; i < iQR0; i++) {
      obj->Q->data[i] = rtNaN;
    }
  }
}

/* End of code generation (computeQ_.c) */
