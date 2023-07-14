/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factor.c
 *
 * Code generation for function 'factor'
 *
 */

/* Include files */
#include "factor.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void factor(d_struct_T *obj, const emxArray_real_T *A, int32_T ndims,
            int32_T ldA)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  const real_T *A_data;
  int32_T i;
  int32_T idx;
  A_data = A->data;
  obj->ndims = ndims;
  for (idx = 0; idx < ndims; idx++) {
    if (ndims >= 1) {
      n_t = (ptrdiff_t)ndims;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, (real_T *)&A_data[ldA * idx], &incx_t,
            &obj->FMat->data[obj->ldm * idx], &incy_t);
    }
  }
  n_t = LAPACKE_dpotrf_work(102, 'U', (ptrdiff_t)ndims, &obj->FMat->data[0],
                            (ptrdiff_t)obj->ldm);
  if ((int32_T)n_t < 0) {
    int32_T obj_idx_1;
    idx = obj->FMat->size[0];
    obj_idx_1 = obj->FMat->size[1];
    i = obj->FMat->size[0] * obj->FMat->size[1];
    obj->FMat->size[0] = idx;
    obj->FMat->size[1] = obj_idx_1;
    emxEnsureCapacity_real_T(obj->FMat, i);
    idx *= obj_idx_1;
    for (i = 0; i < idx; i++) {
      obj->FMat->data[i] = rtNaN;
    }
  }
  obj->info = (int32_T)n_t;
}

/* End of code generation (factor.c) */
