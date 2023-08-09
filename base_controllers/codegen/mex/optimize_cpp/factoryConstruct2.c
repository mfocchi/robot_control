/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct2.c
 *
 * Code generation for function 'factoryConstruct2'
 *
 */

/* Include files */
#include "factoryConstruct2.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void c_factoryConstruct(int32_T MaxVars, i_struct_T *obj)
{
  int32_T i;
  i = obj->grad->size[0];
  obj->grad->size[0] = MaxVars;
  emxEnsureCapacity_real_T(obj->grad, i);
  i = obj->Hx->size[0];
  obj->Hx->size[0] = MaxVars - 1;
  emxEnsureCapacity_real_T(obj->Hx, i);
  obj->hasLinear = false;
  obj->nvar = 0;
  obj->maxVar = MaxVars;
  obj->beta = 0.0;
  obj->rho = 0.0;
  obj->objtype = 3;
  obj->prev_objtype = 3;
  obj->prev_nvar = 0;
  obj->prev_hasLinear = false;
  obj->gammaScalar = 0.0;
}

/* End of code generation (factoryConstruct2.c) */
