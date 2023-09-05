/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * timeKeeper.c
 *
 * Code generation for function 'timeKeeper'
 *
 */

/* Include files */
#include "timeKeeper.h"
#include "rt_nonfinite.h"
#include "emlrt.h"
#include <string.h>

/* Variable Definitions */
static emlrtTimespec savedTime;
static boolean_T savedTime_not_empty;

/* Function Definitions */
void b_timeKeeper(real_T *outTime_tv_sec, real_T *outTime_tv_nsec)
{
  *outTime_tv_sec = savedTime.tv_sec;
  *outTime_tv_nsec = savedTime.tv_nsec;
}

void savedTime_not_empty_init(void)
{
  savedTime_not_empty = false;
}

void timeKeeper(const emlrtTimespec newTime)
{
  if (!savedTime_not_empty) {
    emlrtClockGettimeMonotonic(&savedTime);
    savedTime_not_empty = true;
  }

  savedTime = newTime;
}

/* End of code generation (timeKeeper.c) */
