/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_types.h
 *
 * Code generation for function 'optimize_cpp'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include <stddef.h>

/* Type Definitions */
#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  real_T wall_constraints;
  real_T retraction_force_constraints;
  real_T force_constraints;
  real_T initial_final_constraints;
  real_T via_point;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef typedef_param
#define typedef_param
typedef struct {
  real_T jump_clearance;
  real_T m;
  boolean_T obstacle_avoidance;
  real_T obstacle_location[3];
  real_T obstacle_size[3];
  real_T num_params;
  char_T int_method[3];
  real_T N_dyn;
  real_T FRICTION_CONE;
  real_T int_steps;
  real_T contact_normal[3];
  real_T b;
  real_T p_a1[3];
  real_T p_a2[3];
  real_T g;
  real_T w1;
  real_T w2;
  real_T w3;
  real_T w4;
  real_T w5;
  real_T w6;
  real_T T_th;
} param;
#endif /* typedef_param */

#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  real_T iterations;
  real_T funcCount;
  char_T algorithm[3];
  real_T constrviolation;
  real_T stepsize;
  real_T lssteplength;
  real_T firstorderopt;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef typedef_struct3_T
#define typedef_struct3_T
typedef struct {
  emxArray_real_T *p;
  emxArray_real_T *psi;
  emxArray_real_T *l1;
  emxArray_real_T *l2;
  emxArray_real_T *psid;
  emxArray_real_T *l1d;
  emxArray_real_T *l2d;
  emxArray_real_T *time;
  real_T final_error_discrete;
} struct3_T;
#endif /* typedef_struct3_T */

#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  real_T path_length;
  real_T initial_error;
  real_T final_error_real;
  real_T Fleg[3];
  emxArray_real_T *Fr_l;
  emxArray_real_T *Fr_r;
  emxArray_real_T *p;
  emxArray_real_T *psi;
  emxArray_real_T *l1;
  emxArray_real_T *l2;
  emxArray_real_T *psid;
  emxArray_real_T *l1d;
  emxArray_real_T *l2d;
  emxArray_real_T *time;
  emxArray_real_T *Fr_l_fine;
  emxArray_real_T *Fr_r_fine;
  emxArray_real_T *p_fine;
  emxArray_real_T *psi_fine;
  emxArray_real_T *l1_fine;
  emxArray_real_T *l2_fine;
  emxArray_real_T *psid_fine;
  emxArray_real_T *l1d_fine;
  emxArray_real_T *l2d_fine;
  emxArray_real_T *time_fine;
  real_T Tf;
  real_T achieved_target[3];
  real_T Etot;
  emxArray_real_T *Ekin;
  real_T Ekin0x;
  real_T Ekin0y;
  real_T Ekin0z;
  real_T Ekin0;
  real_T intEkin;
  real_T U0;
  real_T Uf;
  real_T Ekinfx;
  real_T Ekinfy;
  real_T Ekinfz;
  real_T Ekinf;
  real_T T_th;
  real_T cost;
  real_T problem_solved;
  struct1_T optim_output;
  emxArray_real_T *c;
  struct2_T num_constr;
  struct3_T solution_constr;
  real_T constr_tolerance;
} struct0_T;
#endif /* typedef_struct0_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  int32_T ldq;
  emxArray_real_T *QR;
  emxArray_real_T *Q;
  emxArray_int32_T *jpvt;
  int32_T mrows;
  int32_T ncols;
  emxArray_real_T *tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  emxArray_real_T *FMat;
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_;
  real_T workspace2_;
} d_struct_T;
#endif /* typedef_d_struct_T */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T
typedef struct {
  emxArray_real_T *grad;
  emxArray_real_T *Hx;
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T
typedef struct {
  emxArray_real_T *workspace_double;
  emxArray_int32_T *workspace_int;
  emxArray_int32_T *workspace_sort;
} f_struct_T;
#endif /* typedef_f_struct_T */

#ifndef struct_emxArray_ptrdiff_t
#define struct_emxArray_ptrdiff_t
struct emxArray_ptrdiff_t {
  ptrdiff_t *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_ptrdiff_t */
#ifndef typedef_emxArray_ptrdiff_t
#define typedef_emxArray_ptrdiff_t
typedef struct emxArray_ptrdiff_t emxArray_ptrdiff_t;
#endif /* typedef_emxArray_ptrdiff_t */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  int32_T nVarMax;
  int32_T mNonlinIneq;
  int32_T mNonlinEq;
  int32_T mIneq;
  int32_T mEq;
  int32_T iNonIneq0;
  int32_T iNonEq0;
  real_T sqpFval;
  real_T sqpFval_old;
  emxArray_real_T *xstarsqp;
  emxArray_real_T *xstarsqp_old;
  emxArray_real_T *cIneq;
  emxArray_real_T *cIneq_old;
  emxArray_real_T *grad;
  emxArray_real_T *grad_old;
  int32_T FunctionEvaluations;
  int32_T sqpIterations;
  int32_T sqpExitFlag;
  emxArray_real_T *lambdasqp;
  emxArray_real_T *lambdaStopTest;
  emxArray_real_T *lambdaStopTestPrev;
  real_T steplength;
  emxArray_real_T *delta_x;
  emxArray_real_T *socDirection;
  emxArray_int32_T *workingset_old;
  emxArray_real_T *JacCineqTrans_old;
  emxArray_real_T *gradLag;
  emxArray_real_T *delta_gradLag;
  emxArray_real_T *xstar;
  real_T fstar;
  real_T firstorderopt;
  emxArray_real_T *lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  emxArray_real_T *searchDir;
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef struct_emxArray_real_T_0
#define struct_emxArray_real_T_0
struct emxArray_real_T_0 {
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_0 */
#ifndef typedef_emxArray_real_T_0
#define typedef_emxArray_real_T_0
typedef struct emxArray_real_T_0 emxArray_real_T_0;
#endif /* typedef_emxArray_real_T_0 */

#ifndef typedef_h_struct_T
#define typedef_h_struct_T
typedef struct {
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  emxArray_real_T *Aineq;
  emxArray_real_T *bineq;
  emxArray_real_T_0 Aeq;
  emxArray_real_T *lb;
  emxArray_real_T *ub;
  emxArray_int32_T *indexLB;
  emxArray_int32_T *indexUB;
  emxArray_int32_T *indexFixed;
  int32_T mEqRemoved;
  emxArray_real_T *ATwset;
  emxArray_real_T *bwset;
  int32_T nActiveConstr;
  emxArray_real_T *maxConstrWorkspace;
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  emxArray_boolean_T *isActiveConstr;
  emxArray_int32_T *Wid;
  emxArray_int32_T *Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
} h_struct_T;
#endif /* typedef_h_struct_T */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T
typedef struct {
  real_T p0[3];
  real_T pf[3];
  real_T Fleg_max;
  real_T mu;
  param params;
} i_struct_T;
#endif /* typedef_i_struct_T */

#ifndef typedef_anonymous_function
#define typedef_anonymous_function
typedef struct {
  i_struct_T workspace;
} anonymous_function;
#endif /* typedef_anonymous_function */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T
typedef struct {
  real_T p0[3];
  param params;
} j_struct_T;
#endif /* typedef_j_struct_T */

#ifndef typedef_b_anonymous_function
#define typedef_b_anonymous_function
typedef struct {
  j_struct_T workspace;
} b_anonymous_function;
#endif /* typedef_b_anonymous_function */

#ifndef typedef_k_struct_T
#define typedef_k_struct_T
typedef struct {
  b_anonymous_function objfun;
  anonymous_function nonlin;
  real_T f_1;
  emxArray_real_T *cIneq_1;
  emxArray_real_T *cIneq_2;
  int32_T nVar;
  int32_T mIneq;
  int32_T numEvals;
  emxArray_boolean_T *hasLB;
  emxArray_boolean_T *hasUB;
  boolean_T hasBounds;
} k_struct_T;
#endif /* typedef_k_struct_T */

/* End of code generation (optimize_cpp_types.h) */
