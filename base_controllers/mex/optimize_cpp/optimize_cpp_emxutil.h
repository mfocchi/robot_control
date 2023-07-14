/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_emxutil.h
 *
 * Code generation for function 'optimize_cpp_emxutil'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int32_T oldNumel);
void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int32_T oldNumel);
void emxEnsureCapacity_ptrdiff_t(emxArray_ptrdiff_t *emxArray, int32_T oldNumel);
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel);
void emxFreeStruct_struct0_T(struct0_T *pStruct);
void emxFreeStruct_struct_T(d_struct_T *pStruct);
void emxFreeStruct_struct_T1(e_struct_T *pStruct);
void emxFreeStruct_struct_T2(f_struct_T *pStruct);
void emxFreeStruct_struct_T3(h_struct_T *pStruct);
void emxFreeStruct_struct_T4(i_struct_T *pStruct);
void emxFreeStruct_struct_T5(c_struct_T *pStruct);
void emxFreeStruct_struct_T6(j_struct_T *pStruct);
void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
void emxFree_int32_T(emxArray_int32_T **pEmxArray);
void emxFree_ptrdiff_t(emxArray_ptrdiff_t **pEmxArray);
void emxFree_real_T(emxArray_real_T **pEmxArray);
void emxInitStruct_struct0_T(struct0_T *pStruct, boolean_T doPush);
void emxInitStruct_struct_T(d_struct_T *pStruct, boolean_T doPush);
void emxInitStruct_struct_T1(e_struct_T *pStruct, boolean_T doPush);
void emxInitStruct_struct_T2(f_struct_T *pStruct, boolean_T doPush);
void emxInitStruct_struct_T3(h_struct_T *pStruct, boolean_T doPush);
void emxInitStruct_struct_T4(i_struct_T *pStruct, boolean_T doPush);
void emxInitStruct_struct_T5(c_struct_T *pStruct, boolean_T doPush);
void emxInitStruct_struct_T6(j_struct_T *pStruct, boolean_T doPush);
void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int32_T numDimensions,
  boolean_T doPush);
void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions,
                     boolean_T doPush);
void emxInit_ptrdiff_t(emxArray_ptrdiff_t **pEmxArray, int32_T numDimensions,
  boolean_T doPush);
void emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions,
                    boolean_T doPush);

/* End of code generation (optimize_cpp_emxutil.h) */
