/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * all.c
 *
 * Code generation for function 'all'
 *
 */

/* Include files */
#include "all.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void all(const boolean_T x[10000], boolean_T y[100])
{
  int32_T i;
  int32_T i2;
  i2 = 1;
  for (i = 0; i < 100; i++) {
    int32_T a;
    int32_T ix;
    boolean_T exitg1;
    y[i] = true;
    a = i2 + 99;
    ix = i2;
    i2 += 100;
    exitg1 = false;
    while ((!exitg1) && (ix <= a)) {
      if (!x[ix - 1]) {
        y[i] = false;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }
}

/* End of code generation (all.c) */
