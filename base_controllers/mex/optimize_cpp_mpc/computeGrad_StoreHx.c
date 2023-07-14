/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGrad_StoreHx.c
 *
 * Code generation for function 'computeGrad_StoreHx'
 *
 */

/* Include files */
#include "computeGrad_StoreHx.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void computeGrad_StoreHx(e_struct_T *obj, const emxArray_real_T *H,
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
  int32_T iy;
  int32_T k;
  char_T TRANSA;
  x_data = x->data;
  f_data = f->data;
  H_data = H->data;
  switch (obj->objtype) {
  case 5: {
    int32_T i;
    i = obj->nvar;
    for (iy = 0; iy <= i - 2; iy++) {
      obj->grad->data[iy] = 0.0;
    }
    obj->grad->data[obj->nvar - 1] = obj->gammaScalar;
  } break;
  case 3: {
    int32_T i;
    if (obj->nvar >= 1) {
      alpha1 = 1.0;
      beta1 = 0.0;
      TRANSA = 'N';
      m_t = (ptrdiff_t)obj->nvar;
      n_t = (ptrdiff_t)obj->nvar;
      lda_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &alpha1, (real_T *)&H_data[0], &lda_t,
            (real_T *)&x_data[0], &incx_t, &beta1, &obj->Hx->data[0], &incy_t);
    }
    i = obj->nvar;
    for (iy = 0; iy < i; iy++) {
      obj->grad->data[iy] = obj->Hx->data[iy];
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      alpha1 = 1.0;
      n_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      daxpy(&n_t, &alpha1, (real_T *)&f_data[0], &incx_t, &obj->grad->data[0],
            &incy_t);
    }
  } break;
  default: {
    int32_T i;
    int32_T maxRegVar;
    int32_T scalarLB;
    int32_T vectorUB;
    maxRegVar = obj->maxVar - 1;
    if (obj->nvar >= 1) {
      alpha1 = 1.0;
      beta1 = 0.0;
      TRANSA = 'N';
      m_t = (ptrdiff_t)obj->nvar;
      n_t = (ptrdiff_t)obj->nvar;
      lda_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &alpha1, (real_T *)&H_data[0], &lda_t,
            (real_T *)&x_data[0], &incx_t, &beta1, &obj->Hx->data[0], &incy_t);
    }
    i = obj->nvar + 1;
    scalarLB = ((((maxRegVar - i) + 1) / 2) << 1) + i;
    vectorUB = scalarLB - 2;
    for (iy = i; iy <= vectorUB; iy += 2) {
      _mm_storeu_pd(
          &obj->Hx->data[iy - 1],
          _mm_mul_pd(_mm_set1_pd(obj->beta), _mm_loadu_pd(&x_data[iy - 1])));
    }
    for (iy = scalarLB; iy <= maxRegVar; iy++) {
      obj->Hx->data[iy - 1] = obj->beta * x_data[iy - 1];
    }
    for (iy = 0; iy < maxRegVar; iy++) {
      obj->grad->data[iy] = obj->Hx->data[iy];
    }
    if (obj->hasLinear && (obj->nvar >= 1)) {
      alpha1 = 1.0;
      n_t = (ptrdiff_t)obj->nvar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      daxpy(&n_t, &alpha1, (real_T *)&f_data[0], &incx_t, &obj->grad->data[0],
            &incy_t);
    }
    maxRegVar = (obj->maxVar - obj->nvar) - 1;
    if (maxRegVar >= 1) {
      iy = obj->nvar;
      i = maxRegVar - 1;
      scalarLB = (maxRegVar / 2) << 1;
      vectorUB = scalarLB - 2;
      for (k = 0; k <= vectorUB; k += 2) {
        __m128d r;
        maxRegVar = iy + k;
        r = _mm_loadu_pd(&obj->grad->data[maxRegVar]);
        _mm_storeu_pd(&obj->grad->data[maxRegVar],
                      _mm_add_pd(r, _mm_set1_pd(obj->rho)));
      }
      for (k = scalarLB; k <= i; k++) {
        maxRegVar = iy + k;
        obj->grad->data[maxRegVar] += obj->rho;
      }
    }
  } break;
  }
}

/* End of code generation (computeGrad_StoreHx.c) */
