/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * diff.c
 *
 * Code generation for function 'diff'
 *
 */

/* Include files */
#include "diff.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void diff(const emxArray_real_T *x, emxArray_real_T *y)
{
  real_T d;
  real_T tmp1;
  real_T work_data_idx_0;
  int32_T dimSize;
  int32_T ixLead;
  int32_T iyLead;
  int32_T m;
  dimSize = x->size[1];
  if (x->size[1] == 0) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    ixLead = x->size[1] - 1;
    if (muIntScalarMin_sint32(ixLead, 1) < 1) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else {
      ixLead = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = x->size[1] - 1;
      emxEnsureCapacity_real_T(y, ixLead);
      if (x->size[1] - 1 != 0) {
        ixLead = 1;
        iyLead = 0;
        work_data_idx_0 = x->data[0];
        for (m = 2; m <= dimSize; m++) {
          tmp1 = x->data[ixLead];
          d = tmp1;
          tmp1 -= work_data_idx_0;
          work_data_idx_0 = d;
          ixLead++;
          y->data[iyLead] = tmp1;
          iyLead++;
        }
      }
    }
  }
}

/* End of code generation (diff.c) */
