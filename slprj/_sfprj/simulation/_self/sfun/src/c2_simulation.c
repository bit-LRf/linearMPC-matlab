/* Include files */

#include "simulation_sfun.h"
#include "c2_simulation.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

/* Forward Declarations */

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static void *c2_fcnDataPtrs[5];
static char_T *c2_dataNames[5];
static uint32_T c2_ssIds[5];
static uint32_T c2_statuses[5];
static void *c2_outMexFcns[5];
static void *c2_inMexFcns[5];
static emlrtRSInfo c2_emlrtRSI = { 3,  /* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#simulation:15"                     /* pathName */
};

static emlrtRSInfo c2_b_emlrtRSI = { 158,/* lineNo */
  "rostime",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2022b\\toolbox\\ros\\mlroscpp\\rostime.m"/* pathName */
};

/* Function Declarations */
static void initialize_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance);
static void initialize_params_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance);
static void mdl_start_c2_simulation(SFc2_simulationInstanceStruct *chartInstance);
static void mdl_terminate_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance);
static void mdl_setup_runtime_resources_c2_simulation
  (SFc2_simulationInstanceStruct *chartInstance);
static void mdl_cleanup_runtime_resources_c2_simulation
  (SFc2_simulationInstanceStruct *chartInstance);
static void enable_c2_simulation(SFc2_simulationInstanceStruct *chartInstance);
static void disable_c2_simulation(SFc2_simulationInstanceStruct *chartInstance);
static void sf_gateway_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance);
static void ext_mode_exec_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance);
static void c2_update_jit_animation_c2_simulation(SFc2_simulationInstanceStruct *
  chartInstance);
static void c2_do_animation_call_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_simulation(SFc2_simulationInstanceStruct *
  chartInstance);
static void set_sim_state_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void initSimStructsc2_simulation(SFc2_simulationInstanceStruct
  *chartInstance);
static c2_ros_TimeStruct_T c2_ros_TimeStruct(SFc2_simulationInstanceStruct
  *chartInstance);
static uint32_T c2_emlrt_marshallIn(SFc2_simulationInstanceStruct *chartInstance,
  const mxArray *c2_b_Nsec, const char_T *c2_identifier);
static uint32_T c2_b_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static uint8_T c2_c_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_simulation, const char_T
  *c2_identifier);
static uint8_T c2_d_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_slStringInitializeDynamicBuffers(SFc2_simulationInstanceStruct
  *chartInstance);
static const mxArray *c2_sf_marshallOut_ros_TimeStruct_T(void *chartInstanceVoid,
  void *c2_inData);
static c2_ros_TimeStruct_T c2_e_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn_ros_TimeStruct_T(void *chartInstanceVoid, const
  mxArray *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_sf_marshallOut_real_T(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_f_emlrt_marshallIn(SFc2_simulationInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn_real_T(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_sf_marshallOut_uint32_T(void *chartInstanceVoid, void
  *c2_inData);
static void c2_sf_marshallIn_uint32_T(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_chart_data_browse_helper(SFc2_simulationInstanceStruct
  *chartInstance, int32_T c2_ssIdNumber, const mxArray **c2_mxData, uint8_T
  *c2_isValueTooBig);
static void init_dsm_address_info(SFc2_simulationInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc2_simulationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance)
{
  emlrtStack c2_st = { NULL,           /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  c2_st.tls = chartInstance->c2_fEmlrtCtx;
  emlrtLicenseCheckR2022a(&c2_st, "EMLRT:runTime:MexFunctionNeedsLicense",
    "ros_toolbox", 2);
  sfListenerReportChartEnableDisable(chartInstance->c2_RuntimeVar, &_sfTime_, 13);
  sim_mode_is_external(chartInstance->S);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_simulation = 0U;
}

static void initialize_params_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void mdl_start_c2_simulation(SFc2_simulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_terminate_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void mdl_setup_runtime_resources_c2_simulation
  (SFc2_simulationInstanceStruct *chartInstance)
{
  static const int32_T c2_postfixPredicateTree[2] = { 0, -1 };

  static const int32_T c2_condTxtEndIdx[1] = { 315 };

  static const int32_T c2_condTxtStartIdx[1] = { 294 };

  static const uint32_T c2_decisionTxtEndIdx = 0U;
  static const uint32_T c2_decisionTxtStartIdx = 0U;
  setDebuggerFlag(chartInstance->S, true);
  setDataBrowseFcn(chartInstance->S, (void *)&c2_chart_data_browse_helper);
  chartInstance->c2_RuntimeVar = sfListenerInitializeUsingSimStruct
    (chartInstance->S);
  sfListenerInitializeRuntimeVars(chartInstance->c2_RuntimeVar,
    &chartInstance->c2_IsDebuggerActive,
    &chartInstance->c2_IsSequenceViewerPresent, 1, 0,
    &chartInstance->c2_mlFcnLineNumber, &chartInstance->c2_IsHeatMapPresent, 0);
  chartInstance->c2_SequenceViewerOptimization = 1;
  sfListenerInitializeIsStmtByStmtModeVar(chartInstance->c2_RuntimeVar,
    &chartInstance->c2_IsStmtByStmtMode);
  sfListenerInitializeRuntimeFcnVarsJitOff(chartInstance->c2_RuntimeVar,
    &chartInstance->c2_numFcnVars, c2_dataNames, c2_ssIds, c2_fcnDataPtrs,
    c2_outMexFcns, c2_inMexFcns, c2_statuses);
  sim_mode_is_external(chartInstance->S);
  covrtCreateStateflowInstanceData(chartInstance->c2_covrtInstance, 1U, 0U, 1U,
    4U);
  covrtChartInitFcn(chartInstance->c2_covrtInstance, 0U, false, false, false);
  covrtStateInitFcn(chartInstance->c2_covrtInstance, 0U, 0U, false, false, false,
                    0U, &c2_decisionTxtStartIdx, &c2_decisionTxtEndIdx);
  covrtTransInitFcn(chartInstance->c2_covrtInstance, 0U, 0, NULL, NULL, 0U, NULL);
  covrtEmlInitFcn(chartInstance->c2_covrtInstance, "", 4U, 0U, 1U, 0U, 0U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 0U,
                     "eML_blk_kernel", 0, -1, 107);
  covrtEmlInitFcn(chartInstance->c2_covrtInstance,
                  "D:/workspace/stady/matlab/MPC/linearMPC-matlab/msgdef/ros_TimeStruct.m",
                  14U, 0U, 1U, 0U, 1U, 0U, 0U, 0U, 0U, 0U, 1U, 1U);
  covrtEmlFcnInitFcn(chartInstance->c2_covrtInstance, 14U, 0U, 0U,
                     "ros_TimeStruct", 0, -1, 362);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 14U, 0U, 0U, 290, 315, -1,
                    358, false);
  covrtEmlMCDCInitFcn(chartInstance->c2_covrtInstance, 14U, 0U, 0U, 293, 315, 1U,
                      0U, c2_condTxtStartIdx, c2_condTxtEndIdx, 2U,
                      c2_postfixPredicateTree, false);
}

static void mdl_cleanup_runtime_resources_c2_simulation
  (SFc2_simulationInstanceStruct *chartInstance)
{
  sfListenerTerminate(chartInstance->c2_RuntimeVar);
  covrtDeleteStateflowInstanceData(chartInstance->c2_covrtInstance);
}

static void enable_c2_simulation(SFc2_simulationInstanceStruct *chartInstance)
{
  sfListenerReportChartEnableDisable(chartInstance->c2_RuntimeVar, &_sfTime_, 13);
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_simulation(SFc2_simulationInstanceStruct *chartInstance)
{
  sfListenerReportChartEnableDisable(chartInstance->c2_RuntimeVar, &_sfTime_, 14);
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void sf_gateway_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance)
{
  c2_ros_TimeStruct_T c2_msgStruct;
  c2_ros_TimeStruct_T c2_time;
  real_T c2_nargin = 0.0;
  real_T c2_nargout = 2.0;
  uint32_T c2_b_Nsec;
  uint32_T c2_b_Sec;
  chartInstance->c2_JITTransitionAnimation[0] = 0U;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_sfEvent = CALL_EVENT;
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportStartingSection(chartInstance->c2_RuntimeVar, 0, 1);
  }

  chartInstance->c2_numFcnVars = 5U;
  c2_fcnDataPtrs[0] = (void *)&c2_b_Sec;
  c2_fcnDataPtrs[1] = (void *)&c2_b_Nsec;
  c2_fcnDataPtrs[2] = (void *)&c2_time;
  c2_fcnDataPtrs[3] = (void *)&c2_nargin;
  c2_fcnDataPtrs[4] = (void *)&c2_nargout;
  c2_statuses[0] = 0U;
  c2_statuses[1] = 0U;
  c2_statuses[2] = 0U;
  c2_statuses[3] = 0U;
  c2_statuses[4] = 0U;
  c2_inMexFcns[0] = (void *)&c2_sf_marshallIn_uint32_T;
  c2_inMexFcns[1] = (void *)&c2_sf_marshallIn_uint32_T;
  c2_inMexFcns[2] = (void *)&c2_sf_marshallIn_ros_TimeStruct_T;
  c2_inMexFcns[3] = (void *)&c2_sf_marshallIn_real_T;
  c2_inMexFcns[4] = (void *)&c2_sf_marshallIn_real_T;
  c2_outMexFcns[0] = (void *)&c2_sf_marshallOut_uint32_T;
  c2_outMexFcns[1] = (void *)&c2_sf_marshallOut_uint32_T;
  c2_outMexFcns[2] = (void *)&c2_sf_marshallOut_ros_TimeStruct_T;
  c2_outMexFcns[3] = (void *)&c2_sf_marshallOut_real_T;
  c2_outMexFcns[4] = (void *)&c2_sf_marshallOut_real_T;
  c2_ssIds[0] = 0U;
  c2_ssIds[1] = 0U;
  c2_ssIds[2] = 0U;
  c2_ssIds[3] = 0U;
  c2_ssIds[4] = 0U;
  c2_dataNames[0] = (char_T *)"Sec";
  c2_dataNames[1] = (char_T *)"Nsec";
  c2_dataNames[2] = (char_T *)"time";
  c2_dataNames[3] = (char_T *)"nargin";
  c2_dataNames[4] = (char_T *)"nargout";
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportStartingSection(chartInstance->c2_RuntimeVar, 0, 4);
  }

  covrtEmlFcnEval(chartInstance->c2_covrtInstance, 4U, 0, 0);
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 3);
  }

  c2_ros_TimeStruct(chartInstance);
  time2struct(&c2_msgStruct, false);
  c2_time = c2_msgStruct;
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 5);
  }

  c2_b_Sec = c2_time.Sec;
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 6);
  }

  c2_b_Nsec = c2_time.Nsec;
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, -6);
  }

  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportEndingSection(chartInstance->c2_RuntimeVar, 0, 4);
  }

  *chartInstance->c2_Sec = c2_b_Sec;
  *chartInstance->c2_Nsec = c2_b_Nsec;
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportEndingSection(chartInstance->c2_RuntimeVar, 0, 1);
  }

  c2_do_animation_call_c2_simulation(chartInstance);
  covrtSigUpdateFcn(chartInstance->c2_covrtInstance, 0U, (real_T)
                    *chartInstance->c2_Sec);
  covrtSigUpdateFcn(chartInstance->c2_covrtInstance, 1U, (real_T)
                    *chartInstance->c2_Nsec);
}

static void ext_mode_exec_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_update_jit_animation_c2_simulation(SFc2_simulationInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void c2_do_animation_call_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static const mxArray *get_sim_state_c2_simulation(SFc2_simulationInstanceStruct *
  chartInstance)
{
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_d_y = NULL;
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(3, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", chartInstance->c2_Nsec, 7, 0U, 0U,
    0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", chartInstance->c2_Sec, 7, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y",
    &chartInstance->c2_is_active_c2_simulation, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_simulation(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *chartInstance->c2_Nsec = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 0)), "Nsec");
  *chartInstance->c2_Sec = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 1)), "Sec");
  chartInstance->c2_is_active_c2_simulation = c2_c_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
     "is_active_c2_simulation");
  sf_mex_destroy(&c2_u);
  sf_mex_destroy(&c2_st);
}

static void initSimStructsc2_simulation(SFc2_simulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static c2_ros_TimeStruct_T c2_ros_TimeStruct(SFc2_simulationInstanceStruct
  *chartInstance)
{
  static c2_ros_TimeStruct_T c2_r = { 0U,/* Sec */
    0U                                 /* Nsec */
  };

  c2_ros_TimeStruct_T c2_msg;
  real_T c2_nargin = 0.0;
  real_T c2_nargout = 1.0;
  chartInstance->c2_numFcnVars = 3U;
  c2_fcnDataPtrs[0] = (void *)&c2_msg;
  c2_fcnDataPtrs[1] = (void *)&c2_nargin;
  c2_fcnDataPtrs[2] = (void *)&c2_nargout;
  c2_statuses[0] = 0U;
  c2_statuses[1] = 0U;
  c2_statuses[2] = 0U;
  c2_inMexFcns[0] = (void *)&c2_sf_marshallIn_ros_TimeStruct_T;
  c2_inMexFcns[1] = (void *)&c2_sf_marshallIn_real_T;
  c2_inMexFcns[2] = (void *)&c2_sf_marshallIn_real_T;
  c2_outMexFcns[0] = (void *)&c2_sf_marshallOut_ros_TimeStruct_T;
  c2_outMexFcns[1] = (void *)&c2_sf_marshallOut_real_T;
  c2_outMexFcns[2] = (void *)&c2_sf_marshallOut_real_T;
  c2_ssIds[0] = 0U;
  c2_ssIds[1] = 0U;
  c2_ssIds[2] = 0U;
  c2_dataNames[0] = (char_T *)"msg";
  c2_dataNames[1] = (char_T *)"nargin";
  c2_dataNames[2] = (char_T *)"nargout";
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportScriptName(chartInstance->c2_RuntimeVar,
      "D:/workspace/stady/matlab/MPC/linearMPC-matlab/msgdef/ros_TimeStruct.m",
      "", "ros_TimeStruct");
  }

  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportStartingSection(chartInstance->c2_RuntimeVar, 0, 4);
  }

  covrtEmlFcnEval(chartInstance->c2_covrtInstance, 14U, 0, 0);
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 3);
  }

  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 4);
  }

  c2_msg = c2_r;
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 7);
  }

  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 8);
  }

  covrtEmlCondEval(chartInstance->c2_covrtInstance, 14U, 0, 0, false);
  covrtEmlMcdcEval(chartInstance->c2_covrtInstance, 14U, 0, 0, true);
  covrtEmlIfEval(chartInstance->c2_covrtInstance, 14U, 0, 0, true);
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, 9);
  }

  //(&c2_msg);
  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportLineNumber(chartInstance->c2_RuntimeVar, 0, -9);
  }

  if ((chartInstance->c2_IsDebuggerActive == 1) ||
      (chartInstance->c2_IsSequenceViewerPresent == 1) ||
      (chartInstance->c2_IsHeatMapPresent == 1)) {
    sfListenerReportEndingSection(chartInstance->c2_RuntimeVar, 0, 4);
  }

  return c2_msg;
}

const mxArray *sf_c2_simulation_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  const char_T *c2_data[4] = {
    "789cc553dd4ec23018ed0c1a6f40ae7c026e252626267a67100331182224622891ae2b3069d7a52d024fa1173e880fe08b78eb8be8c6567e9a34108d782ef6f5"
    "f4a4df39fdb601a75a7300003990a0b597d46ccaf369dd01ab3075c75235764166e59cd69fd38a79a0c8442524408ccc4f7a9cf9010a54731a122088e4f48978",
    "33a5e753d2f419692c939b98b1ab25694e62295e9706040f1b2306c4402e12d265329fc78be5be990de7716c9947ded0dbd556b953604851e40ace55012acea9"
    "cb27507009198d9e380c63a2a29b1499ce17fe325fcee0663ead47c60fb3812a31c26ab1dffda1bf099bbf86f67bfba19fee5f5de3a7f576b973790ec75c0c65",
    "88308152216f0a93f7036bf512a47e4090885647e926937d8ff4e0eaa08accc8dfb5e43bd830bfedffca82fd59fd7afd70b6e9d7fb7cf7b7e9a7f15f7e134bbf"
    "4dbfbf438b5fded0a7c37ee5d4bdbe476337387bbc38b9bdc358561639ea6b7cd6e50016fed7fdbf01a88570c0",
    "" };

  c2_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c2_data[0], 1648U, &c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static uint32_T c2_emlrt_marshallIn(SFc2_simulationInstanceStruct *chartInstance,
  const mxArray *c2_b_Nsec, const char_T *c2_identifier)
{
  emlrtMsgIdentifier c2_thisId;
  uint32_T c2_y;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Nsec), &c2_thisId);
  sf_mex_destroy(&c2_b_Nsec);
  return c2_y;
}

static uint32_T c2_b_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint32_T c2_b_u;
  uint32_T c2_y;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_b_u, 1, 7, 0U, 0, 0U, 0);
  c2_y = c2_b_u;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static uint8_T c2_c_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_simulation, const char_T
  *c2_identifier)
{
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_y;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_simulation), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_simulation);
  return c2_y;
}

static uint8_T c2_d_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_b_u;
  uint8_T c2_y;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_b_u, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_b_u;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_slStringInitializeDynamicBuffers(SFc2_simulationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c2_sf_marshallOut_ros_TimeStruct_T(void *chartInstanceVoid,
  void *c2_inData)
{
  static const char_T *c2_sv[2] = { "Sec", "Nsec" };

  SFc2_simulationInstanceStruct *chartInstance;
  c2_ros_TimeStruct_T c2_u;
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_mxArrayOutData;
  const mxArray *c2_y = NULL;
  uint32_T c2_b_u;
  uint32_T c2_c_u;
  chartInstance = (SFc2_simulationInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(c2_ros_TimeStruct_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createstruct("structure", 2, c2_sv, 2, 1, 1),
                false);
  c2_b_u = c2_u.Sec;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c2_y, 0, "Sec", c2_b_y, 0);
  c2_c_u = c2_u.Nsec;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_setfieldbynum(c2_y, 0, "Nsec", c2_c_y, 1);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static c2_ros_TimeStruct_T c2_e_emlrt_marshallIn(SFc2_simulationInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  static const char_T *c2_fieldNames[2] = { "Sec", "Nsec" };

  c2_ros_TimeStruct_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fParent = c2_parentId;
  c2_thisId.bParentIsCell = false;
  sf_mex_check_struct(c2_parentId, c2_u, 2, c2_fieldNames, 0U, NULL);
  c2_thisId.fIdentifier = "Sec";
  c2_y.Sec = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Sec", "Sec", 0)), &c2_thisId);
  c2_thisId.fIdentifier = "Nsec";
  c2_y.Nsec = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c2_u, "Nsec", "Nsec", 0)), &c2_thisId);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn_ros_TimeStruct_T(void *chartInstanceVoid, const
  mxArray *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  SFc2_simulationInstanceStruct *chartInstance;
  c2_ros_TimeStruct_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  const mxArray *c2_msg;
  const char_T *c2_identifier;
  chartInstance = (SFc2_simulationInstanceStruct *)chartInstanceVoid;
  c2_msg = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_msg), &c2_thisId);
  sf_mex_destroy(&c2_msg);
  *(c2_ros_TimeStruct_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_sf_marshallOut_real_T(void *chartInstanceVoid, void
  *c2_inData)
{
  SFc2_simulationInstanceStruct *chartInstance;
  const mxArray *c2_mxArrayOutData;
  const mxArray *c2_y = NULL;
  real_T c2_u;
  chartInstance = (SFc2_simulationInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_f_emlrt_marshallIn(SFc2_simulationInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_d;
  real_T c2_y;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn_real_T(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  SFc2_simulationInstanceStruct *chartInstance;
  emlrtMsgIdentifier c2_thisId;
  const mxArray *c2_nargin;
  real_T c2_y;
  const char_T *c2_identifier;
  chartInstance = (SFc2_simulationInstanceStruct *)chartInstanceVoid;
  c2_nargin = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargin), &c2_thisId);
  sf_mex_destroy(&c2_nargin);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_sf_marshallOut_uint32_T(void *chartInstanceVoid, void
  *c2_inData)
{
  SFc2_simulationInstanceStruct *chartInstance;
  const mxArray *c2_mxArrayOutData;
  const mxArray *c2_y = NULL;
  uint32_T c2_u;
  chartInstance = (SFc2_simulationInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(uint32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_sf_marshallIn_uint32_T(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  SFc2_simulationInstanceStruct *chartInstance;
  emlrtMsgIdentifier c2_thisId;
  const mxArray *c2_b_Nsec;
  uint32_T c2_y;
  const char_T *c2_identifier;
  chartInstance = (SFc2_simulationInstanceStruct *)chartInstanceVoid;
  c2_b_Nsec = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Nsec), &c2_thisId);
  sf_mex_destroy(&c2_b_Nsec);
  *(uint32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_chart_data_browse_helper(SFc2_simulationInstanceStruct
  *chartInstance, int32_T c2_ssIdNumber, const mxArray **c2_mxData, uint8_T
  *c2_isValueTooBig)
{
  uint32_T c2_u;
  uint32_T c2_u1;
  *c2_mxData = NULL;
  *c2_mxData = NULL;
  *c2_isValueTooBig = 0U;
  switch (c2_ssIdNumber) {
   case 5U:
    c2_u = *chartInstance->c2_Sec;
    sf_mex_assign(c2_mxData, sf_mex_create("mxData", &c2_u, 7, 0U, 0U, 0U, 0),
                  false);
    break;

   case 6U:
    c2_u1 = *chartInstance->c2_Nsec;
    sf_mex_assign(c2_mxData, sf_mex_create("mxData", &c2_u1, 7, 0U, 0U, 0U, 0),
                  false);
    break;
  }
}

static void init_dsm_address_info(SFc2_simulationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_simulationInstanceStruct
  *chartInstance)
{
  chartInstance->c2_covrtInstance = (CovrtStateflowInstance *)
    sfrtGetCovrtInstance(chartInstance->S);
  chartInstance->c2_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c2_Sec = (uint32_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_Nsec = (uint32_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c2_simulation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4153185595U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(307647906U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(83682115U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3337926547U);
}

mxArray *sf_c2_simulation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_simulation_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("ir_function_calls");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("//");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_simulation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = sf_mex_decode(
    "eNpjYPT0ZQACPiDewMTAwAakOYCYkQECWJH4TEjiIPUSjKSpd2BAqGfBop4fSb0AlJ+YkuKZl5x"
    "TmpIakFiSUQw2ZwEDfnsZ0ewtIGCvFZq9IL6zVUxAUX56UWKugltmTmpxjK9jiI+jU0yQkYGRUV"
    "JMSX5+TlJ+RUxRfnFMcn5KalFJYlF6aklMcVEywt4EuoYP2Jlgc14w0N7e3Byg15MLCuJLMnNT9"
    "TIGLJwB4qtEyw=="
    );
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_simulation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNpjYPT0ZQACPiDewMTAwAakOYCYiQECWKF8RiBmhtIQcRa4uAIQl1QWpILEi4uSPVOAdF5iLpi"
    "fWFrhmZeWDzbfggFhPhsW8xmRzOeEikPAB3vK9Es4gPQbIOlnwaKfBUm/AJDnV5yazAflD6z7RY"
    "hyPzOK+5kZglOTGajjfgUHyvRD7Pcg4H5xFPdD+JnF8YnJJZllqfHJRvHFmbmlOYklmfl5cH8BA"
    "H0fHeg="
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_simulation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "sgYD8mLXENIKknEzfboqzvB";
}

static void sf_opaque_initialize_c2_simulation(void *chartInstanceVar)
{
  initialize_params_c2_simulation((SFc2_simulationInstanceStruct*)
    chartInstanceVar);
  initialize_c2_simulation((SFc2_simulationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_simulation(void *chartInstanceVar)
{
  enable_c2_simulation((SFc2_simulationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_simulation(void *chartInstanceVar)
{
  disable_c2_simulation((SFc2_simulationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_simulation(void *chartInstanceVar)
{
  sf_gateway_c2_simulation((SFc2_simulationInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_simulation(SimStruct* S)
{
  return get_sim_state_c2_simulation((SFc2_simulationInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_simulation(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c2_simulation((SFc2_simulationInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_cleanup_runtime_resources_c2_simulation(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_simulationInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_simulation_optimization_info();
    }

    mdl_cleanup_runtime_resources_c2_simulation((SFc2_simulationInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c2_simulation(void *chartInstanceVar)
{
  mdl_start_c2_simulation((SFc2_simulationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_mdl_terminate_c2_simulation(void *chartInstanceVar)
{
  mdl_terminate_c2_simulation((SFc2_simulationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_simulation((SFc2_simulationInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_simulation(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  sf_warn_if_symbolic_dimension_param_changed(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_simulation((SFc2_simulationInstanceStruct*)
      sf_get_chart_instance_ptr(S));
    initSimStructsc2_simulation((SFc2_simulationInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c2_simulation_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [26] = {
    "eNrdWctv40QYd9pSsYIWDgg4gFiJPXBhtWKl1YIQtHltoz61TruvrLoT+3MyxJ5xZ8ZpUu2B/4A",
    "L4sb/sDeExB/CXjlx5IS4wTe2k7pptvEkpVvVUh6f7d/3fjmxCrVNC49lfP35sWUt4ueb+Jqzku",
    "ONlC5kXsn5BevrlH7+lmU53IUWMDvyPNqzzA4WBTtEkEBa5gcjAdwHyf1IUc5qzOP5sZR5IIA5y",
    "CDkQhnJlTSIfMo61Yg5WrJ80KZO227zyHeLyJC428zvv0puGKkdlFimAhxVBXBVW/Co1a76pHW2",
    "F4Q6LLXB6cgoMPaVBGVHoTZVbka+oqEPlR44NSYVQS/ICfbaiigoqZ6Zk7W90h6geRD6lLD8vm4",
    "TaUOI2aFgN3TxfTtS6L1cWKdNhCpCm3RBbtBOLJ0zyCWdSry7SRlRXFDiVwK/pLnltHfHRxs3sS",
    "R80xihvUUBpBNyypRhQdhV9HOFkaYPZWhGLUO5NhxEuhr2KByCMIuvV+JdEKQF28xM5zhGlV6cl",
    "MNayolVNIA9IlYdzF0JrlnfwKKTNsF0hDqyMcJC7OKarAvaxdww7XU1Xf5T9booSLJfToWN5Va6",
    "YJxXQ7lVh5WI70szbJ2HG9AFP5ZfJopMgU3kG4ClpG6dY3bobmPYsSJGsRJSbIkzl+bPyu4IKh5",
    "sWzikcsBpoMsAXHTzUPUho0l1FEnFgxK2nPLGRk55p7E1pkB4xIHcM0YQKgEVjvPKUK5LpS4kRK",
    "OXVGxlbg5JDU4FtaQXsfIhFx30sekwO/aVrgQzNLgtbMwK4iZXwezeI36UU+dAtrB+MD12JXZZM",
    "7mI1fUzFdghThtcPTmpD5vYZ5FB3hBLPfJX0douVf0ySEfQMG8lRdjQcehqL9X7IeyyDuOHrCp4",
    "YKeb1xl5BYBdgwhGWauII1z0q6h8Pq0FHNTj7m665Gg/E+WTps6Ne8BwGmpb9dZAHKyqCsMVGRW",
    "aBWvTI1ximKRS4aDuJ6M+mXt6f1+xjvf3hTH7+1Jmf383pZ0v9uNVlqQ9RvO5leHzdo7ngOHzw7",
    "zZ88NAnx+0HhxbT08dL/XH0eQBxpApnQYYHlz1u6nNHk2Gt50ltjQVVDOXhoS+pL/Hzc3G5ibam",
    "aTws0Rsz7MJ/rw14k9NP6k9rDy9kURTcK5uNBTnfpP3GoLLRuDjuxOGmtDby80gT9yWR+RoGhns",
    "xwYqETnqdNxG+QyOLJ+Bnd9PkF8bwWn6SeVp+auGbqIyxHnRwFp3+43E7sbmTqmBJYoliN8+T09",
    "iF3LBa5xU/GaQ0eNuRo93JuTPUnr+3x9fFmbBe3/8RmfBWzPqn8WvTYjD+yNx0HS/01q701x/RA",
    "6b7MvvVm/ff+A4ci3h91mGX2EMv2w+THP/tLhsnl6bO9tPc/itMKa/LY3IWRjBLaY+evmL9+LT5",
    "z+3yOOjn+r0179nkf/MsL8N6vajwXPOcKvqnlo88sT/g5H4a1q2HpXvBhsPK1u19Q6rHHlNfnDU",
    "Lcb8Xsydre/8iL6D89f1s5Xutpq/cGpupiWTKHn+H833xQn+uHYi3//6djb8hyuT+l0hkxOJvxa",
    "sLQnO8ivq9WL1fy+X/vMn9J+3bHCs89H/+sps+Onzlcp9ojdC2D+5eSyfQ38wxVkXjCtccXlXXc",
    "/ztM9krzbFzV2wPOs12fd/7zeX7f5p9/vLZsfo51nxt07F//La9btlth9+ktLfDH+vLLWp7475x",
    "SS9vAHEG3f1iuT3P4b+G+wXFe2/9I+tx7dXGfH7kiYP84PTO0L/PzG8JIDI8b9DvY55kqe+x+2D",
    "94rrU8+j/wD10Icw",
    ""
  };

  static char newstr [1817] = "";
  newstr[0] = '\0';
  for (i = 0; i < 26; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c2_simulation(SimStruct *S)
{
  const char* newstr = sf_c2_simulation_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(2959521753U));
  ssSetChecksum1(S,(1738374179U));
  ssSetChecksum2(S,(2524600929U));
  ssSetChecksum3(S,(4139346260U));
}

static void mdlRTW_c2_simulation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c2_simulation(SimStruct *S)
{
  SFc2_simulationInstanceStruct *chartInstance;
  chartInstance = (SFc2_simulationInstanceStruct *)utMalloc(sizeof
    (SFc2_simulationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc2_simulationInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_simulation;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_simulation;
  chartInstance->chartInfo.mdlStart = sf_opaque_mdl_start_c2_simulation;
  chartInstance->chartInfo.mdlTerminate = sf_opaque_mdl_terminate_c2_simulation;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c2_simulation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_simulation;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_simulation;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_simulation;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_simulation;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_simulation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_simulation;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_simulation;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->chartInfo.chartStateSetterFcn = NULL;
  chartInstance->chartInfo.chartStateGetterFcn = NULL;
  chartInstance->S = S;
  chartInstance->chartInfo.dispatchToExportedFcn = NULL;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0,
    chartInstance->c2_JITStateAnimation,
    chartInstance->c2_JITTransitionAnimation);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c2_simulation(chartInstance);
}

void c2_simulation_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c2_simulation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_simulation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_simulation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_simulation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
