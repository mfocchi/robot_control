/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeGrad_StoreHx.c
 *
 * Code generation for function 'computeGrad_StoreHx'
 *
 */

/* Include files */
#include "computeGrad_StoreHx.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xgemv.h"
#include <string.h>

/* Function Definitions */
void computeGrad_StoreHx(i_struct_T *obj, const emxArray_real_T *H, const
  emxArray_real_T *f, const emxArray_real_T *x)
{
  int32_T i;
  int32_T iy;
  int32_T maxRegVar;
  switch (obj->objtype) {
   case 5:
    i = obj->nvar;
    for (iy = 0; iy <= i - 2; iy++) {
      obj->grad->data[iy] = 0.0;
    }

    obj->grad->data[obj->nvar - 1] = obj->gammaScalar;
    break;

   case 3:
    c_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    i = obj->nvar;
    for (iy = 0; iy < i; iy++) {
      obj->grad->data[iy] = obj->Hx->data[iy];
    }

    if (obj->hasLinear) {
      xaxpy(obj->nvar, 1.0, f, obj->grad);
    }
    break;

   default:
    maxRegVar = obj->maxVar - 1;
    c_xgemv(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    i = obj->nvar + 1;
    for (iy = i; iy <= maxRegVar; iy++) {
      obj->Hx->data[iy - 1] = obj->beta * x->data[iy - 1];
    }

    for (iy = 0; iy < maxRegVar; iy++) {
      obj->grad->data[iy] = obj->Hx->data[iy];
    }

    if (obj->hasLinear) {
      xaxpy(obj->nvar, 1.0, f, obj->grad);
    }

    maxRegVar = (obj->maxVar - obj->nvar) - 1;
    if (maxRegVar >= 1) {
      iy = obj->nvar;
      i = maxRegVar - 1;
      for (maxRegVar = 0; maxRegVar <= i; maxRegVar++) {
        obj->grad->data[iy] += obj->rho;
        iy++;
      }
    }
    break;
  }
}

/* End of code generation (computeGrad_StoreHx.c) */
