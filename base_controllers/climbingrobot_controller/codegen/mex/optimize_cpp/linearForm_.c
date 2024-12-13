/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * linearForm_.c
 *
 * Code generation for function 'linearForm_'
 *
 */

/* Include files */
#include "linearForm_.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void linearForm_(boolean_T obj_hasLinear, int32_T obj_nvar,
                 emxArray_real_T *workspace, const emxArray_real_T *H,
                 const emxArray_real_T *f, const emxArray_real_T *x)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  const real_T *H_data;
  const real_T *f_data;
  const real_T *x_data;
  real_T alpha1;
  real_T beta1;
  real_T *workspace_data;
  int32_T i;
  char_T TRANSA;
  x_data = x->data;
  f_data = f->data;
  H_data = H->data;
  workspace_data = workspace->data;
  beta1 = 0.0;
  if (obj_hasLinear) {
    for (i = 0; i < obj_nvar; i++) {
      workspace_data[i] = f_data[i];
    }
    beta1 = 1.0;
  }
  if (obj_nvar >= 1) {
    alpha1 = 0.5;
    TRANSA = 'N';
    m_t = (ptrdiff_t)obj_nvar;
    n_t = (ptrdiff_t)obj_nvar;
    lda_t = (ptrdiff_t)obj_nvar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &alpha1, (real_T *)&H_data[0], &lda_t,
          (real_T *)&x_data[0], &incx_t, &beta1, &workspace_data[0], &incy_t);
  }
}

/* End of code generation (linearForm_.c) */
