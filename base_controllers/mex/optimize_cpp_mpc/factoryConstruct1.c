/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct1.c
 *
 * Code generation for function 'factoryConstruct1'
 *
 */

/* Include files */
#include "factoryConstruct1.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void b_factoryConstruct(const real_T objfun_workspace_actual_state[6],
                        const emxArray_real_T *objfun_workspace_ref_com,
                        const emxArray_real_T *objfun_workspace_Fr_l0,
                        const emxArray_real_T *objfun_workspace_Fr_r0,
                        int64_T objfun_workspace_mpc_N,
                        const param *objfun_workspace_params, int32_T nVar,
                        const emxArray_real_T *lb, const emxArray_real_T *ub,
                        j_struct_T *obj)
{
  const real_T *lb_data;
  const real_T *objfun_workspace_Fr_l0_data;
  const real_T *objfun_workspace_Fr_r0_data;
  const real_T *objfun_workspace_ref_com_data;
  const real_T *ub_data;
  int32_T b_i;
  int32_T i;
  boolean_T b;
  ub_data = ub->data;
  lb_data = lb->data;
  objfun_workspace_Fr_r0_data = objfun_workspace_Fr_r0->data;
  objfun_workspace_Fr_l0_data = objfun_workspace_Fr_l0->data;
  objfun_workspace_ref_com_data = objfun_workspace_ref_com->data;
  for (i = 0; i < 6; i++) {
    obj->objfun.workspace.actual_state[i] = objfun_workspace_actual_state[i];
  }
  b_i = obj->objfun.workspace.ref_com->size[0] *
        obj->objfun.workspace.ref_com->size[1];
  obj->objfun.workspace.ref_com->size[0] = 3;
  obj->objfun.workspace.ref_com->size[1] = objfun_workspace_ref_com->size[1];
  emxEnsureCapacity_real_T(obj->objfun.workspace.ref_com, b_i);
  i = 3 * objfun_workspace_ref_com->size[1];
  for (b_i = 0; b_i < i; b_i++) {
    obj->objfun.workspace.ref_com->data[b_i] =
        objfun_workspace_ref_com_data[b_i];
  }
  b_i = obj->objfun.workspace.Fr_l0->size[0] *
        obj->objfun.workspace.Fr_l0->size[1];
  obj->objfun.workspace.Fr_l0->size[0] = 1;
  obj->objfun.workspace.Fr_l0->size[1] = objfun_workspace_Fr_l0->size[1];
  emxEnsureCapacity_real_T(obj->objfun.workspace.Fr_l0, b_i);
  i = objfun_workspace_Fr_l0->size[1];
  for (b_i = 0; b_i < i; b_i++) {
    obj->objfun.workspace.Fr_l0->data[b_i] = objfun_workspace_Fr_l0_data[b_i];
  }
  b_i = obj->objfun.workspace.Fr_r0->size[0] *
        obj->objfun.workspace.Fr_r0->size[1];
  obj->objfun.workspace.Fr_r0->size[0] = 1;
  obj->objfun.workspace.Fr_r0->size[1] = objfun_workspace_Fr_r0->size[1];
  emxEnsureCapacity_real_T(obj->objfun.workspace.Fr_r0, b_i);
  i = objfun_workspace_Fr_r0->size[1];
  for (b_i = 0; b_i < i; b_i++) {
    obj->objfun.workspace.Fr_r0->data[b_i] = objfun_workspace_Fr_r0_data[b_i];
  }
  obj->objfun.workspace.mpc_N = objfun_workspace_mpc_N;
  obj->objfun.workspace.params = *objfun_workspace_params;
  obj->f_1 = 0.0;
  obj->nVar = nVar;
  obj->numEvals = 0;
  b_i = obj->hasLB->size[0];
  obj->hasLB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasLB, b_i);
  b_i = obj->hasUB->size[0];
  obj->hasUB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasUB, b_i);
  b = false;
  i = 0;
  switch ((uint32_T)(ub->size[1] == 0) << 1 | (uint32_T)(lb->size[1] == 0)) {
  case 0U:
    while ((!b) && (i + 1 <= nVar)) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb_data[i])) &&
                             (!muDoubleScalarIsNaN(lb_data[i])));
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub_data[i])) &&
                             (!muDoubleScalarIsNaN(ub_data[i])));
      if (obj->hasLB->data[i] || obj->hasUB->data[i]) {
        b = true;
      }
      i++;
    }
    while (i + 1 <= nVar) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb_data[i])) &&
                             (!muDoubleScalarIsNaN(lb_data[i])));
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub_data[i])) &&
                             (!muDoubleScalarIsNaN(ub_data[i])));
      i++;
    }
    break;
  case 1U:
    while ((!b) && (i + 1 <= nVar)) {
      obj->hasLB->data[i] = false;
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub_data[i])) &&
                             (!muDoubleScalarIsNaN(ub_data[i])));
      b = obj->hasUB->data[i];
      i++;
    }
    while (i + 1 <= nVar) {
      obj->hasLB->data[i] = false;
      obj->hasUB->data[i] = ((!muDoubleScalarIsInf(ub_data[i])) &&
                             (!muDoubleScalarIsNaN(ub_data[i])));
      i++;
    }
    break;
  case 2U:
    while ((!b) && (i + 1 <= nVar)) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb_data[i])) &&
                             (!muDoubleScalarIsNaN(lb_data[i])));
      obj->hasUB->data[i] = false;
      b = obj->hasLB->data[i];
      i++;
    }
    while (i + 1 <= nVar) {
      obj->hasLB->data[i] = ((!muDoubleScalarIsInf(lb_data[i])) &&
                             (!muDoubleScalarIsNaN(lb_data[i])));
      obj->hasUB->data[i] = false;
      i++;
    }
    break;
  default:
    for (i = 0; i < nVar; i++) {
      obj->hasLB->data[i] = false;
      obj->hasUB->data[i] = false;
    }
    break;
  }
  obj->hasBounds = b;
}

/* End of code generation (factoryConstruct1.c) */
