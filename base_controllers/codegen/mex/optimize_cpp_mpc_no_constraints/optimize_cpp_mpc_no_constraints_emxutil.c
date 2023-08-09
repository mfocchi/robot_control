/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mpc_no_constraints_emxutil.c
 *
 * Code generation for function 'optimize_cpp_mpc_no_constraints_emxutil'
 *
 */

/* Include files */
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_data.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void c_emxFreeStruct_anonymous_funct(anonymous_function *pStruct)
{
  emxFreeStruct_cell_6(&pStruct->tunableEnvironment);
}

void c_emxInitStruct_anonymous_funct(anonymous_function *pStruct, boolean_T
  doPush)
{
  emxInitStruct_cell_6(&pStruct->tunableEnvironment, doPush);
}

void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_ptrdiff_t(emxArray_ptrdiff_t *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(ptrdiff_t));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(ptrdiff_t) * oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (ptrdiff_t *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }

    newData = emlrtCallocMex((uint32_T)i, sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxFreeStruct_cell_6(cell_6 *pStruct)
{
  emxFree_real_T(&pStruct->f3);
  emxFree_real_T(&pStruct->f4);
  emxFree_real_T(&pStruct->f5);
}

void emxFreeStruct_struct_T(d_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->xstarsqp);
  emxFree_real_T(&pStruct->xstarsqp_old);
  emxFree_real_T(&pStruct->grad);
  emxFree_real_T(&pStruct->grad_old);
  emxFree_real_T(&pStruct->lambdasqp);
  emxFree_real_T(&pStruct->lambdasqp_old);
  emxFree_real_T(&pStruct->delta_x);
  emxFree_real_T(&pStruct->socDirection);
  emxFree_real_T(&pStruct->lambda_old);
  emxFree_int32_T(&pStruct->workingset_old);
  emxFree_real_T(&pStruct->gradLag);
  emxFree_real_T(&pStruct->delta_gradLag);
  emxFree_real_T(&pStruct->xstar);
  emxFree_real_T(&pStruct->lambda);
  emxFree_real_T(&pStruct->searchDir);
}

void emxFreeStruct_struct_T1(e_struct_T *pStruct)
{
  c_emxFreeStruct_anonymous_funct(&pStruct->objfun);
}

void emxFreeStruct_struct_T2(f_struct_T *pStruct)
{
  c_emxFreeStruct_anonymous_funct(&pStruct->objfun);
  emxFree_boolean_T(&pStruct->hasLB);
  emxFree_boolean_T(&pStruct->hasUB);
}

void emxFreeStruct_struct_T3(g_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->QR);
  emxFree_real_T(&pStruct->Q);
  emxFree_int32_T(&pStruct->jpvt);
  emxFree_real_T(&pStruct->tau);
}

void emxFreeStruct_struct_T4(h_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->FMat);
}

void emxFreeStruct_struct_T5(i_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->grad);
  emxFree_real_T(&pStruct->Hx);
}

void emxFreeStruct_struct_T6(c_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->workspace_double);
  emxFree_int32_T(&pStruct->workspace_int);
  emxFree_int32_T(&pStruct->workspace_sort);
}

void emxFreeStruct_struct_T7(j_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->lb);
  emxFree_real_T(&pStruct->ub);
  emxFree_int32_T(&pStruct->indexLB);
  emxFree_int32_T(&pStruct->indexUB);
  emxFree_int32_T(&pStruct->indexFixed);
  emxFree_real_T(&pStruct->ATwset);
  emxFree_real_T(&pStruct->bwset);
  emxFree_real_T(&pStruct->maxConstrWorkspace);
  emxFree_boolean_T(&pStruct->isActiveConstr);
  emxFree_int32_T(&pStruct->Wid);
  emxFree_int32_T(&pStruct->Wlocalidx);
}

void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

void emxFree_ptrdiff_t(emxArray_ptrdiff_t **pEmxArray)
{
  if (*pEmxArray != (emxArray_ptrdiff_t *)NULL) {
    if (((*pEmxArray)->data != (ptrdiff_t *)NULL) && (*pEmxArray)->canFreeData)
    {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_ptrdiff_t *)NULL;
  }
}

void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }

    emlrtFreeMex((*pEmxArray)->size);
    emlrtFreeMex(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void emxInitStruct_cell_6(cell_6 *pStruct, boolean_T doPush)
{
  emxInit_real_T(&pStruct->f3, 2, doPush);
  emxInit_real_T(&pStruct->f4, 2, doPush);
  emxInit_real_T(&pStruct->f5, 2, doPush);
}

void emxInitStruct_struct_T(d_struct_T *pStruct, boolean_T doPush)
{
  emxInit_real_T(&pStruct->xstarsqp, 2, doPush);
  emxInit_real_T(&pStruct->xstarsqp_old, 2, doPush);
  emxInit_real_T(&pStruct->grad, 1, doPush);
  emxInit_real_T(&pStruct->grad_old, 1, doPush);
  emxInit_real_T(&pStruct->lambdasqp, 1, doPush);
  emxInit_real_T(&pStruct->lambdasqp_old, 1, doPush);
  emxInit_real_T(&pStruct->delta_x, 1, doPush);
  emxInit_real_T(&pStruct->socDirection, 1, doPush);
  emxInit_real_T(&pStruct->lambda_old, 1, doPush);
  emxInit_int32_T(&pStruct->workingset_old, 1, doPush);
  emxInit_real_T(&pStruct->gradLag, 1, doPush);
  emxInit_real_T(&pStruct->delta_gradLag, 1, doPush);
  emxInit_real_T(&pStruct->xstar, 1, doPush);
  emxInit_real_T(&pStruct->lambda, 1, doPush);
  emxInit_real_T(&pStruct->searchDir, 1, doPush);
}

void emxInitStruct_struct_T1(e_struct_T *pStruct, boolean_T doPush)
{
  c_emxInitStruct_anonymous_funct(&pStruct->objfun, doPush);
}

void emxInitStruct_struct_T2(f_struct_T *pStruct, boolean_T doPush)
{
  c_emxInitStruct_anonymous_funct(&pStruct->objfun, doPush);
  emxInit_boolean_T(&pStruct->hasLB, 1, doPush);
  emxInit_boolean_T(&pStruct->hasUB, 1, doPush);
}

void emxInitStruct_struct_T3(g_struct_T *pStruct, boolean_T doPush)
{
  emxInit_real_T(&pStruct->QR, 2, doPush);
  emxInit_real_T(&pStruct->Q, 2, doPush);
  emxInit_int32_T(&pStruct->jpvt, 1, doPush);
  emxInit_real_T(&pStruct->tau, 1, doPush);
}

void emxInitStruct_struct_T4(h_struct_T *pStruct, boolean_T doPush)
{
  emxInit_real_T(&pStruct->FMat, 2, doPush);
}

void emxInitStruct_struct_T5(i_struct_T *pStruct, boolean_T doPush)
{
  emxInit_real_T(&pStruct->grad, 1, doPush);
  emxInit_real_T(&pStruct->Hx, 1, doPush);
}

void emxInitStruct_struct_T6(c_struct_T *pStruct, boolean_T doPush)
{
  emxInit_real_T(&pStruct->workspace_double, 2, doPush);
  emxInit_int32_T(&pStruct->workspace_int, 1, doPush);
  emxInit_int32_T(&pStruct->workspace_sort, 1, doPush);
}

void emxInitStruct_struct_T7(j_struct_T *pStruct, boolean_T doPush)
{
  emxInit_real_T(&pStruct->lb, 1, doPush);
  emxInit_real_T(&pStruct->ub, 1, doPush);
  emxInit_int32_T(&pStruct->indexLB, 1, doPush);
  emxInit_int32_T(&pStruct->indexUB, 1, doPush);
  emxInit_int32_T(&pStruct->indexFixed, 1, doPush);
  emxInit_real_T(&pStruct->ATwset, 2, doPush);
  emxInit_real_T(&pStruct->bwset, 1, doPush);
  emxInit_real_T(&pStruct->maxConstrWorkspace, 1, doPush);
  emxInit_boolean_T(&pStruct->isActiveConstr, 1, doPush);
  emxInit_int32_T(&pStruct->Wid, 1, doPush);
  emxInit_int32_T(&pStruct->Wlocalidx, 1, doPush);
}

void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int32_T numDimensions,
  boolean_T doPush)
{
  emxArray_boolean_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_boolean_T *)emlrtMallocMex(sizeof(emxArray_boolean_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(emlrtRootTLSGlobal, (void *)pEmxArray,
      (void *)&emxFree_boolean_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions,
                     boolean_T doPush)
{
  emxArray_int32_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int32_T *)emlrtMallocMex(sizeof(emxArray_int32_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(emlrtRootTLSGlobal, (void *)pEmxArray,
      (void *)&emxFree_int32_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_ptrdiff_t(emxArray_ptrdiff_t **pEmxArray, int32_T numDimensions,
  boolean_T doPush)
{
  emxArray_ptrdiff_t *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_ptrdiff_t *)emlrtMallocMex(sizeof(emxArray_ptrdiff_t));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(emlrtRootTLSGlobal, (void *)pEmxArray,
      (void *)&emxFree_ptrdiff_t);
  }

  emxArray = *pEmxArray;
  emxArray->data = (ptrdiff_t *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions,
                    boolean_T doPush)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocMex(sizeof(emxArray_real_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2012b(emlrtRootTLSGlobal, (void *)pEmxArray,
      (void *)&emxFree_real_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/* End of code generation (optimize_cpp_mpc_no_constraints_emxutil.c) */
