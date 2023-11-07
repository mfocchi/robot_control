/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * tic.c
 *
 * Code generation for function 'tic'
 *
 */

/* Include files */
#include "tic.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "emlrt.h"
#include <string.h>

/* Function Definitions */
void tic(void)
{
  emlrtTimespec t;
  emlrtClockGettimeMonotonic(&t);
  timeKeeper(t);
}

/* End of code generation (tic.c) */
