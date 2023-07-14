/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_emxutil.c
 *
 * Code generation for function 'optimize_cpp_emxutil'
 *
 */

/* Include files */
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
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
      memcpy(newData, emxArray->data, sizeof(boolean_T) * (uint32_T)oldNumel);
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
      memcpy(newData, emxArray->data, sizeof(int32_T) * (uint32_T)oldNumel);
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
      memcpy(newData, emxArray->data, sizeof(ptrdiff_t) * (uint32_T)oldNumel);
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
      memcpy(newData, emxArray->data, sizeof(real_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        emlrtFreeMex(emxArray->data);
      }
    }
    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxFreeStruct_struct0_T(struct0_T *pStruct)
{
  emxFree_real_T(&pStruct->Fr_l);
  emxFree_real_T(&pStruct->Fr_r);
  emxFree_real_T(&pStruct->p);
  emxFree_real_T(&pStruct->psi);
  emxFree_real_T(&pStruct->l1);
  emxFree_real_T(&pStruct->l2);
  emxFree_real_T(&pStruct->psid);
  emxFree_real_T(&pStruct->l1d);
  emxFree_real_T(&pStruct->l2d);
  emxFree_real_T(&pStruct->time);
  emxFree_real_T(&pStruct->Ekin);
}

void emxFreeStruct_struct_T(g_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->xstarsqp);
  emxFree_real_T(&pStruct->xstarsqp_old);
  emxFree_real_T(&pStruct->cIneq);
  emxFree_real_T(&pStruct->cIneq_old);
  emxFree_real_T(&pStruct->grad);
  emxFree_real_T(&pStruct->grad_old);
  emxFree_real_T(&pStruct->lambdasqp);
  emxFree_real_T(&pStruct->lambdaStopTest);
  emxFree_real_T(&pStruct->lambdaStopTestPrev);
  emxFree_real_T(&pStruct->delta_x);
  emxFree_real_T(&pStruct->socDirection);
  emxFree_int32_T(&pStruct->workingset_old);
  emxFree_real_T(&pStruct->JacCineqTrans_old);
  emxFree_real_T(&pStruct->gradLag);
  emxFree_real_T(&pStruct->delta_gradLag);
  emxFree_real_T(&pStruct->xstar);
  emxFree_real_T(&pStruct->lambda);
  emxFree_real_T(&pStruct->searchDir);
}

void emxFreeStruct_struct_T1(k_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->cIneq_1);
  emxFree_real_T(&pStruct->cIneq_2);
  emxFree_boolean_T(&pStruct->hasLB);
  emxFree_boolean_T(&pStruct->hasUB);
}

void emxFreeStruct_struct_T2(c_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->QR);
  emxFree_real_T(&pStruct->Q);
  emxFree_int32_T(&pStruct->jpvt);
  emxFree_real_T(&pStruct->tau);
}

void emxFreeStruct_struct_T3(d_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->FMat);
}

void emxFreeStruct_struct_T4(e_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->grad);
  emxFree_real_T(&pStruct->Hx);
}

void emxFreeStruct_struct_T5(f_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->workspace_double);
  emxFree_int32_T(&pStruct->workspace_int);
  emxFree_int32_T(&pStruct->workspace_sort);
}

void emxFreeStruct_struct_T6(h_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->Aineq);
  emxFree_real_T(&pStruct->bineq);
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
    if (((*pEmxArray)->data != (boolean_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference(emlrtRootTLSGlobal, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
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
    emlrtRemoveHeapReference(emlrtRootTLSGlobal, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

void emxFree_ptrdiff_t(emxArray_ptrdiff_t **pEmxArray)
{
  if (*pEmxArray != (emxArray_ptrdiff_t *)NULL) {
    if (((*pEmxArray)->data != (ptrdiff_t *)NULL) &&
        (*pEmxArray)->canFreeData) {
      emlrtFreeMex((*pEmxArray)->data);
    }
    emlrtFreeMex((*pEmxArray)->size);
    emlrtRemoveHeapReference(emlrtRootTLSGlobal, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
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
    emlrtRemoveHeapReference(emlrtRootTLSGlobal, (void *)pEmxArray);
    emlrtFreeEmxArray(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void emxInitStruct_struct0_T(struct0_T *pStruct)
{
  emxInit_real_T(&pStruct->Fr_l, 2);
  emxInit_real_T(&pStruct->Fr_r, 2);
  emxInit_real_T(&pStruct->p, 2);
  emxInit_real_T(&pStruct->psi, 2);
  emxInit_real_T(&pStruct->l1, 2);
  emxInit_real_T(&pStruct->l2, 2);
  emxInit_real_T(&pStruct->psid, 2);
  emxInit_real_T(&pStruct->l1d, 2);
  emxInit_real_T(&pStruct->l2d, 2);
  emxInit_real_T(&pStruct->time, 2);
  emxInit_real_T(&pStruct->Ekin, 2);
}

void emxInitStruct_struct_T(g_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->xstarsqp, 2);
  emxInit_real_T(&pStruct->xstarsqp_old, 2);
  emxInit_real_T(&pStruct->cIneq, 1);
  emxInit_real_T(&pStruct->cIneq_old, 1);
  emxInit_real_T(&pStruct->grad, 1);
  emxInit_real_T(&pStruct->grad_old, 1);
  emxInit_real_T(&pStruct->lambdasqp, 1);
  emxInit_real_T(&pStruct->lambdaStopTest, 1);
  emxInit_real_T(&pStruct->lambdaStopTestPrev, 1);
  emxInit_real_T(&pStruct->delta_x, 1);
  emxInit_real_T(&pStruct->socDirection, 1);
  emxInit_int32_T(&pStruct->workingset_old, 1);
  emxInit_real_T(&pStruct->JacCineqTrans_old, 2);
  emxInit_real_T(&pStruct->gradLag, 1);
  emxInit_real_T(&pStruct->delta_gradLag, 1);
  emxInit_real_T(&pStruct->xstar, 1);
  emxInit_real_T(&pStruct->lambda, 1);
  emxInit_real_T(&pStruct->searchDir, 1);
}

void emxInitStruct_struct_T1(k_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->cIneq_1, 1);
  emxInit_real_T(&pStruct->cIneq_2, 1);
  emxInit_boolean_T(&pStruct->hasLB);
  emxInit_boolean_T(&pStruct->hasUB);
}

void emxInitStruct_struct_T2(c_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->QR, 2);
  emxInit_real_T(&pStruct->Q, 2);
  emxInit_int32_T(&pStruct->jpvt, 1);
  emxInit_real_T(&pStruct->tau, 1);
}

void emxInitStruct_struct_T3(d_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->FMat, 2);
}

void emxInitStruct_struct_T4(e_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->grad, 1);
  emxInit_real_T(&pStruct->Hx, 1);
}

void emxInitStruct_struct_T5(f_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->workspace_double, 2);
  emxInit_int32_T(&pStruct->workspace_int, 1);
  emxInit_int32_T(&pStruct->workspace_sort, 1);
}

void emxInitStruct_struct_T6(h_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->Aineq, 1);
  emxInit_real_T(&pStruct->bineq, 1);
  pStruct->Aeq.size[0] = 0;
  emxInit_real_T(&pStruct->lb, 1);
  emxInit_real_T(&pStruct->ub, 1);
  emxInit_int32_T(&pStruct->indexLB, 1);
  emxInit_int32_T(&pStruct->indexUB, 1);
  emxInit_int32_T(&pStruct->indexFixed, 1);
  emxInit_real_T(&pStruct->ATwset, 1);
  emxInit_real_T(&pStruct->bwset, 1);
  emxInit_real_T(&pStruct->maxConstrWorkspace, 1);
  emxInit_boolean_T(&pStruct->isActiveConstr);
  emxInit_int32_T(&pStruct->Wid, 1);
  emxInit_int32_T(&pStruct->Wlocalidx, 1);
}

void emxInit_boolean_T(emxArray_boolean_T **pEmxArray)
{
  emxArray_boolean_T *emxArray;
  *pEmxArray =
      (emxArray_boolean_T *)emlrtMallocEmxArray(sizeof(emxArray_boolean_T));
  emlrtPushHeapReferenceStackEmxArray(
      emlrtRootTLSGlobal, false, (void *)pEmxArray, (void *)&emxFree_boolean_T,
      NULL, NULL, NULL);
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = 1;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  emxArray->size[0] = 0;
}

void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions)
{
  emxArray_int32_T *emxArray;
  int32_T i;
  *pEmxArray =
      (emxArray_int32_T *)emlrtMallocEmxArray(sizeof(emxArray_int32_T));
  emlrtPushHeapReferenceStackEmxArray(
      emlrtRootTLSGlobal, false, (void *)pEmxArray, (void *)&emxFree_int32_T,
      NULL, NULL, NULL);
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size =
      (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_ptrdiff_t(emxArray_ptrdiff_t **pEmxArray)
{
  emxArray_ptrdiff_t *emxArray;
  *pEmxArray =
      (emxArray_ptrdiff_t *)emlrtMallocEmxArray(sizeof(emxArray_ptrdiff_t));
  emlrtPushHeapReferenceStackEmxArray(
      emlrtRootTLSGlobal, false, (void *)pEmxArray, (void *)&emxFree_ptrdiff_t,
      NULL, NULL, NULL);
  emxArray = *pEmxArray;
  emxArray->data = (ptrdiff_t *)NULL;
  emxArray->numDimensions = 1;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  emxArray->size[0] = 0;
}

void emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocEmxArray(sizeof(emxArray_real_T));
  emlrtPushHeapReferenceStackEmxArray(
      emlrtRootTLSGlobal, false, (void *)pEmxArray, (void *)&emxFree_real_T,
      NULL, NULL, NULL);
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size =
      (int32_T *)emlrtMallocMex(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/* End of code generation (optimize_cpp_emxutil.c) */
