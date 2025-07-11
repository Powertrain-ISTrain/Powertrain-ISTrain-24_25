#ifndef __c3_project_part1_h__
#define __c3_project_part1_h__

/* Type Definitions */
#ifndef typedef_SFc3_project_part1InstanceStruct
#define typedef_SFc3_project_part1InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_project_part1;
  void *c3_fEmlrtCtx;
  real_T *c3_dx;
  real_T *c3_theta;
  real_T (*c3_lin_map)[10];
} SFc3_project_part1InstanceStruct;

#endif                                 /*typedef_SFc3_project_part1InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_project_part1_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_project_part1_get_check_sum(mxArray *plhs[]);
extern void c3_project_part1_method_dispatcher(SimStruct *S, int_T method, void *
  data);

#endif
