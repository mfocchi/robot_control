/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factor.c
 *
 * Code generation for function 'factor'
 *
 */

/* Include files */
#include "factor.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xpotrf.h"
#include <string.h>

/* Function Definitions */
void factor(const emlrtStack *sp, h_struct_T *obj, const emxArray_real_T *A,
            int32_T ndims, int32_T ldA)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  obj->ndims = ndims;
  st.site = &nh_emlrtRSI;
  if ((1 <= ndims) && (ndims > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < ndims; idx++) {
    st.site = &nh_emlrtRSI;
    b_xcopy(ndims, A, ldA * idx + 1, obj->FMat, obj->ldm * idx + 1);
  }

  st.site = &nh_emlrtRSI;
  obj->info = xpotrf(&st, ndims, obj->FMat, obj->ldm);
}

/* End of code generation (factor.c) */
