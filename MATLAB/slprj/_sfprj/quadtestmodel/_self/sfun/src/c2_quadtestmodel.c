/* Include files */

#include <stddef.h>
#include "blas.h"
#include "quadtestmodel_sfun.h"
#include "c2_quadtestmodel.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "quadtestmodel_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[30] = { "Q", "Xdot", "Ts", "w", "a",
  "q", "Omega", "Xi", "DCM", "Fvqparts", "Fvq", "Fvba", "Fqq", "Fqbw", "F",
  "Gvwa", "Gqww", "G", "nargin", "nargout", "am", "wm", "agsamp", "t", "Xout",
  "Pout", "P", "X", "oldagsamp", "oldt" };

/* Function Declarations */
static void initialize_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance);
static void initialize_params_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct *
  chartInstance);
static void enable_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance);
static void disable_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance);
static void c2_update_debugger_state_c2_quadtestmodel
  (SFc2_quadtestmodelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_quadtestmodel
  (SFc2_quadtestmodelInstanceStruct *chartInstance);
static void set_sim_state_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance);
static void sf_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct *chartInstance);
static void c2_chartstep_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance);
static void initSimStructsc2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance);
static void registerMessagesc2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_oldt, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_oldagsamp, const char_T *c2_identifier);
static real_T c2_d_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_e_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_X, const char_T *c2_identifier, real_T
  c2_y[16]);
static void c2_f_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_P, const char_T *c2_identifier, real_T
  c2_y[256]);
static void c2_h_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[256]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_i_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_Pout, const char_T *c2_identifier, real_T
  c2_y[256]);
static void c2_j_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[256]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_k_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_Xout, const char_T *c2_identifier, real_T
  c2_y[16]);
static void c2_l_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_m_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_n_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[192]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_o_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[12]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_p_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_q_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16]);
static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_r_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[12]);
static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_s_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_t_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[53]);
static void c2_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance);
static real_T c2_norm(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T
                      c2_x[4]);
static void c2_eye(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T c2_I
                   [256]);
static void c2_b_eye(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T
                     c2_I[9]);
static void c2_c_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance);
static void c2_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T
  c2_A[256], real_T c2_B[256], real_T c2_C[256], real_T c2_b_C[256]);
static void c2_d_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance);
static void c2_b_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[144], real_T c2_C[192], real_T c2_b_C[192]);
static void c2_e_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance);
static void c2_c_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[192], real_T c2_C[256], real_T c2_b_C[256]);
static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_u_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_v_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_quadtestmodel, const char_T
  *c2_identifier);
static uint8_T c2_w_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[256], real_T c2_B[256], real_T c2_C[256]);
static void c2_e_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[144], real_T c2_C[192]);
static void c2_f_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[192], real_T c2_C[256]);
static void init_dsm_address_info(SFc2_quadtestmodelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_P_not_empty = FALSE;
  chartInstance->c2_X_not_empty = FALSE;
  chartInstance->c2_oldagsamp_not_empty = FALSE;
  chartInstance->c2_oldt_not_empty = FALSE;
  chartInstance->c2_is_active_c2_quadtestmodel = 0U;
}

static void initialize_params_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct *
  chartInstance)
{
}

static void enable_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_quadtestmodel
  (SFc2_quadtestmodelInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_quadtestmodel
  (SFc2_quadtestmodelInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[256];
  const mxArray *c2_b_y = NULL;
  int32_T c2_i1;
  real_T c2_b_u[16];
  const mxArray *c2_c_y = NULL;
  int32_T c2_i2;
  real_T c2_c_u[256];
  const mxArray *c2_d_y = NULL;
  int32_T c2_i3;
  real_T c2_d_u[16];
  const mxArray *c2_e_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  uint8_T c2_c_hoistedGlobal;
  uint8_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real_T (*c2_Xout)[16];
  real_T (*c2_Pout)[256];
  c2_Pout = (real_T (*)[256])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Xout = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(7), FALSE);
  for (c2_i0 = 0; c2_i0 < 256; c2_i0++) {
    c2_u[c2_i0] = (*c2_Pout)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 16, 16),
                FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  for (c2_i1 = 0; c2_i1 < 16; c2_i1++) {
    c2_b_u[c2_i1] = (*c2_Xout)[c2_i1];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 1, 16), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  for (c2_i2 = 0; c2_i2 < 256; c2_i2++) {
    c2_c_u[c2_i2] = chartInstance->c2_P[c2_i2];
  }

  c2_d_y = NULL;
  if (!chartInstance->c2_P_not_empty) {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 0, 0U, 1U, 0U, 2, 16, 16),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 2, c2_d_y);
  for (c2_i3 = 0; c2_i3 < 16; c2_i3++) {
    c2_d_u[c2_i3] = chartInstance->c2_X[c2_i3];
  }

  c2_e_y = NULL;
  if (!chartInstance->c2_X_not_empty) {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", c2_d_u, 0, 0U, 1U, 0U, 1, 16),
                  FALSE);
  }

  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_hoistedGlobal = chartInstance->c2_oldagsamp;
  c2_e_u = c2_hoistedGlobal;
  c2_f_y = NULL;
  if (!chartInstance->c2_oldagsamp_not_empty) {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_b_hoistedGlobal = chartInstance->c2_oldt;
  c2_f_u = c2_b_hoistedGlobal;
  c2_g_y = NULL;
  if (!chartInstance->c2_oldt_not_empty) {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  FALSE);
  } else {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_c_hoistedGlobal = chartInstance->c2_is_active_c2_quadtestmodel;
  c2_g_u = c2_c_hoistedGlobal;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 6, c2_h_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[256];
  int32_T c2_i4;
  real_T c2_dv1[16];
  int32_T c2_i5;
  real_T c2_dv2[256];
  int32_T c2_i6;
  real_T c2_dv3[16];
  int32_T c2_i7;
  real_T (*c2_Pout)[256];
  real_T (*c2_Xout)[16];
  c2_Pout = (real_T (*)[256])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_Xout = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                        "Pout", c2_dv0);
  for (c2_i4 = 0; c2_i4 < 256; c2_i4++) {
    (*c2_Pout)[c2_i4] = c2_dv0[c2_i4];
  }

  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                        "Xout", c2_dv1);
  for (c2_i5 = 0; c2_i5 < 16; c2_i5++) {
    (*c2_Xout)[c2_i5] = c2_dv1[c2_i5];
  }

  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)), "P",
                        c2_dv2);
  for (c2_i6 = 0; c2_i6 < 256; c2_i6++) {
    chartInstance->c2_P[c2_i6] = c2_dv2[c2_i6];
  }

  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 3)), "X",
                        c2_dv3);
  for (c2_i7 = 0; c2_i7 < 16; c2_i7++) {
    chartInstance->c2_X[c2_i7] = c2_dv3[c2_i7];
  }

  chartInstance->c2_oldagsamp = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 4)), "oldagsamp");
  chartInstance->c2_oldt = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 5)), "oldt");
  chartInstance->c2_is_active_c2_quadtestmodel = c2_v_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 6)),
     "is_active_c2_quadtestmodel");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_quadtestmodel(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
}

static void sf_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct *chartInstance)
{
  int32_T c2_i8;
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_i11;
  real_T *c2_agsamp;
  real_T *c2_t;
  real_T (*c2_Pout)[256];
  real_T (*c2_wm)[3];
  real_T (*c2_Xout)[16];
  real_T (*c2_am)[3];
  c2_Pout = (real_T (*)[256])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_agsamp = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_wm = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_Xout = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_am = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  for (c2_i8 = 0; c2_i8 < 3; c2_i8++) {
    _SFD_DATA_RANGE_CHECK((*c2_am)[c2_i8], 0U);
  }

  for (c2_i9 = 0; c2_i9 < 16; c2_i9++) {
    _SFD_DATA_RANGE_CHECK((*c2_Xout)[c2_i9], 1U);
  }

  for (c2_i10 = 0; c2_i10 < 3; c2_i10++) {
    _SFD_DATA_RANGE_CHECK((*c2_wm)[c2_i10], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_agsamp, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_t, 4U);
  for (c2_i11 = 0; c2_i11 < 256; c2_i11++) {
    _SFD_DATA_RANGE_CHECK((*c2_Pout)[c2_i11], 5U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_quadtestmodel(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_quadtestmodelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  int32_T c2_i12;
  real_T c2_am[3];
  int32_T c2_i13;
  real_T c2_wm[3];
  real_T c2_agsamp;
  real_T c2_t;
  uint32_T c2_debug_family_var_map[30];
  real_T c2_Q[144];
  real_T c2_Xdot[16];
  real_T c2_Ts;
  real_T c2_w[3];
  real_T c2_a[3];
  real_T c2_q[4];
  real_T c2_Omega[16];
  real_T c2_Xi[12];
  real_T c2_DCM[9];
  real_T c2_Fvqparts[4];
  real_T c2_Fvq[12];
  real_T c2_Fvba[9];
  real_T c2_Fqq[16];
  real_T c2_Fqbw[12];
  real_T c2_F[256];
  real_T c2_Gvwa[9];
  real_T c2_Gqww[12];
  real_T c2_G[192];
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 2.0;
  real_T c2_Xout[16];
  real_T c2_Pout[256];
  int32_T c2_i14;
  static real_T c2_dv4[16] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c2_i15;
  int32_T c2_i16;
  static real_T c2_b[144] = { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  real_T c2_b_b;
  real_T c2_y;
  real_T c2_b_a;
  real_T c2_c_b;
  real_T c2_b_y;
  real_T c2_d_b;
  real_T c2_c_y;
  real_T c2_c_a;
  real_T c2_e_b;
  real_T c2_d_y;
  real_T c2_f_b;
  real_T c2_e_y;
  real_T c2_d_a;
  real_T c2_g_b;
  real_T c2_f_y;
  real_T c2_h_b;
  real_T c2_g_y;
  real_T c2_e_a;
  real_T c2_i_b;
  real_T c2_h_y;
  real_T c2_j_b;
  real_T c2_i_y;
  real_T c2_f_a;
  real_T c2_k_b;
  real_T c2_j_y;
  real_T c2_l_b;
  real_T c2_k_y;
  real_T c2_g_a;
  real_T c2_m_b;
  real_T c2_l_y;
  real_T c2_n_b;
  real_T c2_m_y;
  real_T c2_h_a;
  real_T c2_o_b;
  real_T c2_n_y;
  real_T c2_p_b;
  real_T c2_o_y;
  real_T c2_i_a;
  real_T c2_q_b;
  real_T c2_p_y;
  real_T c2_r_b;
  real_T c2_q_y;
  real_T c2_j_a;
  real_T c2_s_b;
  real_T c2_r_y;
  real_T c2_t_b;
  real_T c2_s_y;
  real_T c2_k_a;
  real_T c2_u_b;
  real_T c2_t_y;
  real_T c2_v_b;
  real_T c2_u_y;
  real_T c2_l_a;
  real_T c2_w_b;
  real_T c2_v_y;
  real_T c2_x_b;
  real_T c2_w_y;
  real_T c2_m_a;
  real_T c2_y_b;
  real_T c2_x_y;
  real_T c2_ab_b;
  real_T c2_y_y;
  real_T c2_n_a;
  real_T c2_bb_b;
  real_T c2_ab_y;
  real_T c2_cb_b;
  real_T c2_bb_y;
  real_T c2_o_a;
  real_T c2_db_b;
  real_T c2_cb_y;
  real_T c2_eb_b;
  real_T c2_db_y;
  real_T c2_p_a;
  real_T c2_fb_b;
  real_T c2_eb_y;
  real_T c2_gb_b;
  real_T c2_fb_y;
  real_T c2_q_a;
  real_T c2_hb_b;
  real_T c2_gb_y;
  real_T c2_ib_b;
  real_T c2_hb_y;
  real_T c2_r_a;
  real_T c2_jb_b;
  real_T c2_ib_y;
  real_T c2_kb_b;
  real_T c2_jb_y;
  real_T c2_s_a;
  real_T c2_lb_b;
  real_T c2_kb_y;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  real_T c2_t_a[9];
  int32_T c2_i26;
  real_T c2_mb_b[3];
  int32_T c2_i27;
  real_T c2_lb_y[3];
  int32_T c2_i28;
  int32_T c2_i29;
  int32_T c2_i30;
  int32_T c2_i31;
  real_T c2_u_a[12];
  int32_T c2_i32;
  int32_T c2_i33;
  real_T c2_mb_y[4];
  int32_T c2_i34;
  int32_T c2_i35;
  int32_T c2_i36;
  int32_T c2_i37;
  int32_T c2_i38;
  int32_T c2_i39;
  real_T c2_c_hoistedGlobal[16];
  int32_T c2_i40;
  real_T c2_d_hoistedGlobal[16];
  int32_T c2_i41;
  int32_T c2_i42;
  real_T c2_e_hoistedGlobal[4];
  real_T c2_B;
  real_T c2_nb_y;
  real_T c2_ob_y;
  int32_T c2_i43;
  int32_T c2_i44;
  int32_T c2_i45;
  int32_T c2_i46;
  int32_T c2_i47;
  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  int32_T c2_i51;
  int32_T c2_i52;
  int32_T c2_i53;
  int32_T c2_i54;
  int32_T c2_i55;
  real_T c2_b_Fvqparts[12];
  int32_T c2_i56;
  int32_T c2_i57;
  int32_T c2_i58;
  int32_T c2_i59;
  int32_T c2_i60;
  int32_T c2_i61;
  int32_T c2_i62;
  int32_T c2_i63;
  int32_T c2_i64;
  real_T c2_f_hoistedGlobal[256];
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_i67;
  real_T c2_dv5[256];
  int32_T c2_i68;
  int32_T c2_i69;
  int32_T c2_i70;
  int32_T c2_i71;
  int32_T c2_i72;
  int32_T c2_i73;
  int32_T c2_i74;
  int32_T c2_i75;
  int32_T c2_i76;
  int32_T c2_i77;
  int32_T c2_i78;
  int32_T c2_i79;
  int32_T c2_i80;
  int32_T c2_i81;
  int32_T c2_i82;
  int32_T c2_i83;
  int32_T c2_i84;
  int32_T c2_i85;
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  int32_T c2_i89;
  int32_T c2_i90;
  int32_T c2_i91;
  int32_T c2_i92;
  int32_T c2_i93;
  int32_T c2_i94;
  int32_T c2_i95;
  int32_T c2_i96;
  int32_T c2_i97;
  int32_T c2_i98;
  int32_T c2_i99;
  int32_T c2_i100;
  int32_T c2_i101;
  int32_T c2_i102;
  int32_T c2_i103;
  int32_T c2_i104;
  int32_T c2_i105;
  int32_T c2_i106;
  int32_T c2_i107;
  int32_T c2_i108;
  int32_T c2_i109;
  int32_T c2_i110;
  int32_T c2_i111;
  int32_T c2_i112;
  int32_T c2_i113;
  real_T c2_dv6[9];
  int32_T c2_i114;
  int32_T c2_i115;
  int32_T c2_i116;
  real_T c2_dv7[192];
  int32_T c2_i117;
  int32_T c2_i118;
  int32_T c2_i119;
  int32_T c2_i120;
  int32_T c2_i121;
  int32_T c2_i122;
  int32_T c2_i123;
  int32_T c2_i124;
  int32_T c2_i125;
  int32_T c2_i126;
  int32_T c2_i127;
  int32_T c2_i128;
  int32_T c2_i129;
  int32_T c2_i130;
  int32_T c2_i131;
  int32_T c2_i132;
  int32_T c2_i133;
  int32_T c2_i134;
  int32_T c2_i135;
  int32_T c2_i136;
  int32_T c2_i137;
  int32_T c2_i138;
  int32_T c2_i139;
  int32_T c2_i140;
  int32_T c2_i141;
  int32_T c2_i142;
  int32_T c2_i143;
  int32_T c2_i144;
  int32_T c2_i145;
  int32_T c2_i146;
  int32_T c2_i147;
  int32_T c2_i148;
  int32_T c2_i149;
  int32_T c2_i150;
  int32_T c2_i151;
  int32_T c2_i152;
  int32_T c2_i153;
  int32_T c2_i154;
  int32_T c2_i155;
  real_T c2_v_a[256];
  int32_T c2_i156;
  real_T c2_pb_y[256];
  int32_T c2_i157;
  real_T c2_w_a[256];
  int32_T c2_i158;
  real_T c2_g_hoistedGlobal[256];
  int32_T c2_i159;
  int32_T c2_i160;
  int32_T c2_i161;
  int32_T c2_i162;
  int32_T c2_i163;
  int32_T c2_i164;
  real_T c2_qb_y[256];
  int32_T c2_i165;
  real_T c2_h_hoistedGlobal[256];
  int32_T c2_i166;
  real_T c2_x_a[192];
  int32_T c2_i167;
  real_T c2_rb_y[192];
  int32_T c2_i168;
  real_T c2_y_a[192];
  int32_T c2_i169;
  real_T c2_nb_b[144];
  int32_T c2_i170;
  int32_T c2_i171;
  int32_T c2_i172;
  int32_T c2_i173;
  real_T c2_ob_b[192];
  int32_T c2_i174;
  int32_T c2_i175;
  real_T c2_sb_y[192];
  int32_T c2_i176;
  real_T c2_pb_b[192];
  int32_T c2_i177;
  int32_T c2_i178;
  int32_T c2_i179;
  int32_T c2_i180;
  int32_T c2_i181;
  real_T *c2_b_agsamp;
  real_T *c2_b_t;
  real_T (*c2_b_Xout)[16];
  real_T (*c2_b_Pout)[256];
  real_T (*c2_b_wm)[3];
  real_T (*c2_b_am)[3];
  c2_b_Pout = (real_T (*)[256])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_agsamp = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_wm = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_Xout = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_am = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_agsamp;
  c2_b_hoistedGlobal = *c2_b_t;
  for (c2_i12 = 0; c2_i12 < 3; c2_i12++) {
    c2_am[c2_i12] = (*c2_b_am)[c2_i12];
  }

  for (c2_i13 = 0; c2_i13 < 3; c2_i13++) {
    c2_wm[c2_i13] = (*c2_b_wm)[c2_i13];
  }

  c2_agsamp = c2_hoistedGlobal;
  c2_t = c2_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 30U, 30U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_Q, 0U, c2_o_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Xdot, 1U, c2_f_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Ts, 2U, c2_g_sf_marshallOut,
    c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_w, 3U, c2_h_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_a, 4U, c2_h_sf_marshallOut,
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q, 5U, c2_n_sf_marshallOut,
    c2_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Omega, 6U, c2_l_sf_marshallOut,
    c2_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Xi, 7U, c2_j_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_DCM, 8U, c2_k_sf_marshallOut,
    c2_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Fvqparts, 9U, c2_n_sf_marshallOut,
    c2_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Fvq, 10U, c2_m_sf_marshallOut,
    c2_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Fvba, 11U, c2_k_sf_marshallOut,
    c2_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Fqq, 12U, c2_l_sf_marshallOut,
    c2_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Fqbw, 13U, c2_j_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_F, 14U, c2_e_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Gvwa, 15U, c2_k_sf_marshallOut,
    c2_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Gqww, 16U, c2_j_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_G, 17U, c2_i_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 18U, c2_g_sf_marshallOut,
    c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 19U, c2_g_sf_marshallOut,
    c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_am, 20U, c2_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_wm, 21U, c2_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_agsamp, 22U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_t, 23U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Xout, 24U, c2_f_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Pout, 25U, c2_e_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_P, 26U,
    c2_d_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(chartInstance->c2_X, 27U,
    c2_c_sf_marshallOut, c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c2_oldagsamp, 28U,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c2_oldt, 29U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c2_oldt_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
    chartInstance->c2_oldt = 0.0;
    chartInstance->c2_oldt_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  if (CV_EML_IF(0, 1, 1, !chartInstance->c2_oldagsamp_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
    chartInstance->c2_oldagsamp = 0.0;
    chartInstance->c2_oldagsamp_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  if (CV_EML_IF(0, 1, 2, !chartInstance->c2_X_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
    for (c2_i14 = 0; c2_i14 < 16; c2_i14++) {
      chartInstance->c2_X[c2_i14] = c2_dv4[c2_i14];
    }

    chartInstance->c2_X_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  if (CV_EML_IF(0, 1, 3, !chartInstance->c2_P_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
    for (c2_i15 = 0; c2_i15 < 256; c2_i15++) {
      chartInstance->c2_P[c2_i15] = 0.0;
    }

    chartInstance->c2_P_not_empty = TRUE;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
  for (c2_i16 = 0; c2_i16 < 144; c2_i16++) {
    c2_Q[c2_i16] = c2_b[c2_i16];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
  for (c2_i17 = 0; c2_i17 < 16; c2_i17++) {
    c2_Xdot[c2_i17] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 24);
  if (CV_EML_IF(0, 1, 4, c2_agsamp != chartInstance->c2_oldagsamp)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
    chartInstance->c2_oldagsamp = c2_agsamp;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
    c2_Ts = c2_t - chartInstance->c2_oldt;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
    chartInstance->c2_oldt = c2_t;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
    for (c2_i18 = 0; c2_i18 < 3; c2_i18++) {
      c2_w[c2_i18] = c2_wm[c2_i18] - chartInstance->c2_X[c2_i18 + 10];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
    for (c2_i19 = 0; c2_i19 < 3; c2_i19++) {
      c2_a[c2_i19] = c2_am[c2_i19] - chartInstance->c2_X[c2_i19 + 13];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
    for (c2_i20 = 0; c2_i20 < 4; c2_i20++) {
      c2_q[c2_i20] = chartInstance->c2_X[c2_i20 + 6];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
    c2_Omega[0] = 0.0;
    c2_Omega[4] = -c2_w[0];
    c2_Omega[8] = -c2_w[1];
    c2_Omega[12] = -c2_w[2];
    c2_Omega[1] = c2_w[0];
    c2_Omega[5] = 0.0;
    c2_Omega[9] = c2_w[2];
    c2_Omega[13] = -c2_w[1];
    c2_Omega[2] = c2_w[1];
    c2_Omega[6] = -c2_w[2];
    c2_Omega[10] = 0.0;
    c2_Omega[14] = c2_w[0];
    c2_Omega[3] = c2_w[2];
    c2_Omega[7] = c2_w[1];
    c2_Omega[11] = -c2_w[0];
    c2_Omega[15] = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
    c2_Xi[0] = -c2_q[1];
    c2_Xi[4] = -c2_q[2];
    c2_Xi[8] = -c2_q[3];
    c2_Xi[1] = c2_q[0];
    c2_Xi[5] = -c2_q[3];
    c2_Xi[9] = c2_q[2];
    c2_Xi[2] = c2_q[3];
    c2_Xi[6] = c2_q[0];
    c2_Xi[10] = -c2_q[1];
    c2_Xi[3] = -c2_q[2];
    c2_Xi[7] = c2_q[1];
    c2_Xi[11] = c2_q[0];
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 48);
    c2_b_b = c2_q[2];
    c2_y = 2.0 * c2_b_b;
    c2_b_a = c2_y;
    c2_c_b = c2_q[2];
    c2_b_y = c2_b_a * c2_c_b;
    c2_d_b = c2_q[3];
    c2_c_y = 2.0 * c2_d_b;
    c2_c_a = c2_c_y;
    c2_e_b = c2_q[3];
    c2_d_y = c2_c_a * c2_e_b;
    c2_f_b = c2_q[1];
    c2_e_y = 2.0 * c2_f_b;
    c2_d_a = c2_e_y;
    c2_g_b = c2_q[2];
    c2_f_y = c2_d_a * c2_g_b;
    c2_h_b = c2_q[3];
    c2_g_y = 2.0 * c2_h_b;
    c2_e_a = c2_g_y;
    c2_i_b = c2_q[0];
    c2_h_y = c2_e_a * c2_i_b;
    c2_j_b = c2_q[1];
    c2_i_y = 2.0 * c2_j_b;
    c2_f_a = c2_i_y;
    c2_k_b = c2_q[3];
    c2_j_y = c2_f_a * c2_k_b;
    c2_l_b = c2_q[2];
    c2_k_y = 2.0 * c2_l_b;
    c2_g_a = c2_k_y;
    c2_m_b = c2_q[0];
    c2_l_y = c2_g_a * c2_m_b;
    c2_n_b = c2_q[1];
    c2_m_y = 2.0 * c2_n_b;
    c2_h_a = c2_m_y;
    c2_o_b = c2_q[2];
    c2_n_y = c2_h_a * c2_o_b;
    c2_p_b = c2_q[3];
    c2_o_y = 2.0 * c2_p_b;
    c2_i_a = c2_o_y;
    c2_q_b = c2_q[0];
    c2_p_y = c2_i_a * c2_q_b;
    c2_r_b = c2_q[1];
    c2_q_y = 2.0 * c2_r_b;
    c2_j_a = c2_q_y;
    c2_s_b = c2_q[1];
    c2_r_y = c2_j_a * c2_s_b;
    c2_t_b = c2_q[3];
    c2_s_y = 2.0 * c2_t_b;
    c2_k_a = c2_s_y;
    c2_u_b = c2_q[3];
    c2_t_y = c2_k_a * c2_u_b;
    c2_v_b = c2_q[2];
    c2_u_y = 2.0 * c2_v_b;
    c2_l_a = c2_u_y;
    c2_w_b = c2_q[3];
    c2_v_y = c2_l_a * c2_w_b;
    c2_x_b = c2_q[1];
    c2_w_y = 2.0 * c2_x_b;
    c2_m_a = c2_w_y;
    c2_y_b = c2_q[0];
    c2_x_y = c2_m_a * c2_y_b;
    c2_ab_b = c2_q[1];
    c2_y_y = 2.0 * c2_ab_b;
    c2_n_a = c2_y_y;
    c2_bb_b = c2_q[3];
    c2_ab_y = c2_n_a * c2_bb_b;
    c2_cb_b = c2_q[2];
    c2_bb_y = 2.0 * c2_cb_b;
    c2_o_a = c2_bb_y;
    c2_db_b = c2_q[0];
    c2_cb_y = c2_o_a * c2_db_b;
    c2_eb_b = c2_q[2];
    c2_db_y = 2.0 * c2_eb_b;
    c2_p_a = c2_db_y;
    c2_fb_b = c2_q[3];
    c2_eb_y = c2_p_a * c2_fb_b;
    c2_gb_b = c2_q[1];
    c2_fb_y = 2.0 * c2_gb_b;
    c2_q_a = c2_fb_y;
    c2_hb_b = c2_q[0];
    c2_gb_y = c2_q_a * c2_hb_b;
    c2_ib_b = c2_q[1];
    c2_hb_y = 2.0 * c2_ib_b;
    c2_r_a = c2_hb_y;
    c2_jb_b = c2_q[1];
    c2_ib_y = c2_r_a * c2_jb_b;
    c2_kb_b = c2_q[2];
    c2_jb_y = 2.0 * c2_kb_b;
    c2_s_a = c2_jb_y;
    c2_lb_b = c2_q[2];
    c2_kb_y = c2_s_a * c2_lb_b;
    c2_DCM[0] = (1.0 - c2_b_y) - c2_d_y;
    c2_DCM[3] = c2_f_y + c2_h_y;
    c2_DCM[6] = c2_j_y - c2_l_y;
    c2_DCM[1] = c2_n_y - c2_p_y;
    c2_DCM[4] = (1.0 - c2_r_y) - c2_t_y;
    c2_DCM[7] = c2_v_y + c2_x_y;
    c2_DCM[2] = c2_ab_y + c2_cb_y;
    c2_DCM[5] = c2_eb_y - c2_gb_y;
    c2_DCM[8] = (1.0 - c2_ib_y) - c2_kb_y;
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 56);
    for (c2_i21 = 0; c2_i21 < 3; c2_i21++) {
      c2_Xdot[c2_i21] = chartInstance->c2_X[c2_i21 + 3];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 59);
    c2_i22 = 0;
    for (c2_i23 = 0; c2_i23 < 3; c2_i23++) {
      c2_i24 = 0;
      for (c2_i25 = 0; c2_i25 < 3; c2_i25++) {
        c2_t_a[c2_i25 + c2_i22] = c2_DCM[c2_i24 + c2_i23];
        c2_i24 += 3;
      }

      c2_i22 += 3;
    }

    for (c2_i26 = 0; c2_i26 < 3; c2_i26++) {
      c2_mb_b[c2_i26] = c2_a[c2_i26];
    }

    c2_eml_scalar_eg(chartInstance);
    c2_eml_scalar_eg(chartInstance);
    for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
      c2_lb_y[c2_i27] = 0.0;
      c2_i28 = 0;
      for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
        c2_lb_y[c2_i27] += c2_t_a[c2_i28 + c2_i27] * c2_mb_b[c2_i29];
        c2_i28 += 3;
      }
    }

    for (c2_i30 = 0; c2_i30 < 3; c2_i30++) {
      c2_Xdot[c2_i30 + 3] = c2_lb_y[c2_i30];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 60);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
    for (c2_i31 = 0; c2_i31 < 12; c2_i31++) {
      c2_u_a[c2_i31] = 0.5 * c2_Xi[c2_i31];
    }

    for (c2_i32 = 0; c2_i32 < 3; c2_i32++) {
      c2_mb_b[c2_i32] = c2_w[c2_i32];
    }

    c2_b_eml_scalar_eg(chartInstance);
    c2_b_eml_scalar_eg(chartInstance);
    for (c2_i33 = 0; c2_i33 < 4; c2_i33++) {
      c2_mb_y[c2_i33] = 0.0;
      c2_i34 = 0;
      for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
        c2_mb_y[c2_i33] += c2_u_a[c2_i34 + c2_i33] * c2_mb_b[c2_i35];
        c2_i34 += 4;
      }
    }

    for (c2_i36 = 0; c2_i36 < 4; c2_i36++) {
      c2_Xdot[c2_i36 + 6] = c2_mb_y[c2_i36];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 66);
    for (c2_i37 = 0; c2_i37 < 6; c2_i37++) {
      c2_Xdot[c2_i37 + 10] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 68);
    for (c2_i38 = 0; c2_i38 < 16; c2_i38++) {
      chartInstance->c2_X[c2_i38] += c2_Xdot[c2_i38] * c2_Ts;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 69);
    for (c2_i39 = 0; c2_i39 < 16; c2_i39++) {
      c2_c_hoistedGlobal[c2_i39] = chartInstance->c2_X[c2_i39];
    }

    for (c2_i40 = 0; c2_i40 < 16; c2_i40++) {
      c2_d_hoistedGlobal[c2_i40] = chartInstance->c2_X[c2_i40];
    }

    for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
      c2_mb_y[c2_i41] = c2_c_hoistedGlobal[c2_i41 + 6];
    }

    for (c2_i42 = 0; c2_i42 < 4; c2_i42++) {
      c2_e_hoistedGlobal[c2_i42] = c2_d_hoistedGlobal[c2_i42 + 6];
    }

    c2_B = c2_norm(chartInstance, c2_e_hoistedGlobal);
    c2_nb_y = c2_B;
    c2_ob_y = c2_nb_y;
    for (c2_i43 = 0; c2_i43 < 4; c2_i43++) {
      c2_mb_y[c2_i43] /= c2_ob_y;
    }

    for (c2_i44 = 0; c2_i44 < 4; c2_i44++) {
      chartInstance->c2_X[c2_i44 + 6] = c2_mb_y[c2_i44];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 77);
    for (c2_i45 = 0; c2_i45 < 12; c2_i45++) {
      c2_u_a[c2_i45] = c2_Xi[c2_i45];
    }

    for (c2_i46 = 0; c2_i46 < 3; c2_i46++) {
      c2_mb_b[c2_i46] = c2_a[c2_i46];
    }

    c2_b_eml_scalar_eg(chartInstance);
    c2_b_eml_scalar_eg(chartInstance);
    for (c2_i47 = 0; c2_i47 < 4; c2_i47++) {
      c2_Fvqparts[c2_i47] = 0.0;
    }

    for (c2_i48 = 0; c2_i48 < 4; c2_i48++) {
      c2_Fvqparts[c2_i48] = 0.0;
    }

    for (c2_i49 = 0; c2_i49 < 4; c2_i49++) {
      c2_mb_y[c2_i49] = c2_Fvqparts[c2_i49];
    }

    for (c2_i50 = 0; c2_i50 < 4; c2_i50++) {
      c2_Fvqparts[c2_i50] = c2_mb_y[c2_i50];
    }

    for (c2_i51 = 0; c2_i51 < 4; c2_i51++) {
      c2_mb_y[c2_i51] = c2_Fvqparts[c2_i51];
    }

    for (c2_i52 = 0; c2_i52 < 4; c2_i52++) {
      c2_Fvqparts[c2_i52] = c2_mb_y[c2_i52];
    }

    for (c2_i53 = 0; c2_i53 < 4; c2_i53++) {
      c2_Fvqparts[c2_i53] = 0.0;
      c2_i54 = 0;
      for (c2_i55 = 0; c2_i55 < 3; c2_i55++) {
        c2_Fvqparts[c2_i53] += c2_u_a[c2_i54 + c2_i53] * c2_mb_b[c2_i55];
        c2_i54 += 4;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 78);
    c2_b_Fvqparts[0] = c2_Fvqparts[1];
    c2_b_Fvqparts[3] = -c2_Fvqparts[0];
    c2_b_Fvqparts[6] = c2_Fvqparts[3];
    c2_b_Fvqparts[9] = -c2_Fvqparts[2];
    c2_b_Fvqparts[1] = c2_Fvqparts[2];
    c2_b_Fvqparts[4] = -c2_Fvqparts[3];
    c2_b_Fvqparts[7] = -c2_Fvqparts[0];
    c2_b_Fvqparts[10] = c2_Fvqparts[1];
    c2_b_Fvqparts[2] = c2_Fvqparts[3];
    c2_b_Fvqparts[5] = c2_Fvqparts[2];
    c2_b_Fvqparts[8] = -c2_Fvqparts[1];
    c2_b_Fvqparts[11] = -c2_Fvqparts[0];
    c2_i56 = 0;
    for (c2_i57 = 0; c2_i57 < 4; c2_i57++) {
      for (c2_i58 = 0; c2_i58 < 3; c2_i58++) {
        c2_Fvq[c2_i58 + c2_i56] = 2.0 * c2_b_Fvqparts[c2_i58 + c2_i56];
      }

      c2_i56 += 3;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 82);
    c2_i59 = 0;
    for (c2_i60 = 0; c2_i60 < 3; c2_i60++) {
      c2_i61 = 0;
      for (c2_i62 = 0; c2_i62 < 3; c2_i62++) {
        c2_Fvba[c2_i62 + c2_i59] = -c2_DCM[c2_i61 + c2_i60];
        c2_i61 += 3;
      }

      c2_i59 += 3;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 83);
    for (c2_i63 = 0; c2_i63 < 16; c2_i63++) {
      c2_Fqq[c2_i63] = 0.5 * c2_Omega[c2_i63];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 84);
    for (c2_i64 = 0; c2_i64 < 12; c2_i64++) {
      c2_Fqbw[c2_i64] = -0.5 * c2_Xi[c2_i64];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 86);
    c2_eye(chartInstance, c2_f_hoistedGlobal);
    c2_b_eye(chartInstance, c2_t_a);
    c2_i65 = 0;
    for (c2_i66 = 0; c2_i66 < 3; c2_i66++) {
      for (c2_i67 = 0; c2_i67 < 3; c2_i67++) {
        c2_dv5[c2_i67 + c2_i65] = 0.0;
      }

      c2_i65 += 16;
    }

    c2_i68 = 0;
    c2_i69 = 0;
    for (c2_i70 = 0; c2_i70 < 3; c2_i70++) {
      for (c2_i71 = 0; c2_i71 < 3; c2_i71++) {
        c2_dv5[(c2_i71 + c2_i68) + 48] = c2_t_a[c2_i71 + c2_i69];
      }

      c2_i68 += 16;
      c2_i69 += 3;
    }

    c2_i72 = 0;
    for (c2_i73 = 0; c2_i73 < 10; c2_i73++) {
      for (c2_i74 = 0; c2_i74 < 3; c2_i74++) {
        c2_dv5[(c2_i74 + c2_i72) + 96] = 0.0;
      }

      c2_i72 += 16;
    }

    c2_i75 = 0;
    for (c2_i76 = 0; c2_i76 < 6; c2_i76++) {
      for (c2_i77 = 0; c2_i77 < 3; c2_i77++) {
        c2_dv5[(c2_i77 + c2_i75) + 3] = 0.0;
      }

      c2_i75 += 16;
    }

    c2_i78 = 0;
    c2_i79 = 0;
    for (c2_i80 = 0; c2_i80 < 4; c2_i80++) {
      for (c2_i81 = 0; c2_i81 < 3; c2_i81++) {
        c2_dv5[(c2_i81 + c2_i78) + 99] = c2_Fvq[c2_i81 + c2_i79];
      }

      c2_i78 += 16;
      c2_i79 += 3;
    }

    c2_i82 = 0;
    for (c2_i83 = 0; c2_i83 < 3; c2_i83++) {
      for (c2_i84 = 0; c2_i84 < 3; c2_i84++) {
        c2_dv5[(c2_i84 + c2_i82) + 163] = 0.0;
      }

      c2_i82 += 16;
    }

    c2_i85 = 0;
    c2_i86 = 0;
    for (c2_i87 = 0; c2_i87 < 3; c2_i87++) {
      for (c2_i88 = 0; c2_i88 < 3; c2_i88++) {
        c2_dv5[(c2_i88 + c2_i85) + 211] = c2_Fvba[c2_i88 + c2_i86];
      }

      c2_i85 += 16;
      c2_i86 += 3;
    }

    c2_i89 = 0;
    for (c2_i90 = 0; c2_i90 < 6; c2_i90++) {
      for (c2_i91 = 0; c2_i91 < 4; c2_i91++) {
        c2_dv5[(c2_i91 + c2_i89) + 6] = 0.0;
      }

      c2_i89 += 16;
    }

    c2_i92 = 0;
    c2_i93 = 0;
    for (c2_i94 = 0; c2_i94 < 4; c2_i94++) {
      for (c2_i95 = 0; c2_i95 < 4; c2_i95++) {
        c2_dv5[(c2_i95 + c2_i92) + 102] = c2_Fqq[c2_i95 + c2_i93];
      }

      c2_i92 += 16;
      c2_i93 += 4;
    }

    c2_i96 = 0;
    c2_i97 = 0;
    for (c2_i98 = 0; c2_i98 < 3; c2_i98++) {
      for (c2_i99 = 0; c2_i99 < 4; c2_i99++) {
        c2_dv5[(c2_i99 + c2_i96) + 166] = c2_Fqbw[c2_i99 + c2_i97];
      }

      c2_i96 += 16;
      c2_i97 += 4;
    }

    c2_i100 = 0;
    for (c2_i101 = 0; c2_i101 < 3; c2_i101++) {
      for (c2_i102 = 0; c2_i102 < 4; c2_i102++) {
        c2_dv5[(c2_i102 + c2_i100) + 214] = 0.0;
      }

      c2_i100 += 16;
    }

    c2_i103 = 0;
    for (c2_i104 = 0; c2_i104 < 16; c2_i104++) {
      for (c2_i105 = 0; c2_i105 < 6; c2_i105++) {
        c2_dv5[(c2_i105 + c2_i103) + 10] = 0.0;
      }

      c2_i103 += 16;
    }

    c2_i106 = 0;
    for (c2_i107 = 0; c2_i107 < 16; c2_i107++) {
      for (c2_i108 = 0; c2_i108 < 16; c2_i108++) {
        c2_F[c2_i108 + c2_i106] = c2_f_hoistedGlobal[c2_i108 + c2_i106] + c2_Ts *
          c2_dv5[c2_i108 + c2_i106];
      }

      c2_i106 += 16;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 91);
    c2_i109 = 0;
    for (c2_i110 = 0; c2_i110 < 3; c2_i110++) {
      c2_i111 = 0;
      for (c2_i112 = 0; c2_i112 < 3; c2_i112++) {
        c2_Gvwa[c2_i112 + c2_i109] = c2_DCM[c2_i111 + c2_i110];
        c2_i111 += 3;
      }

      c2_i109 += 3;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 92);
    for (c2_i113 = 0; c2_i113 < 12; c2_i113++) {
      c2_Gqww[c2_i113] = 0.5 * c2_Xi[c2_i113];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 94);
    c2_b_eye(chartInstance, c2_t_a);
    c2_b_eye(chartInstance, c2_dv6);
    c2_i114 = 0;
    for (c2_i115 = 0; c2_i115 < 12; c2_i115++) {
      for (c2_i116 = 0; c2_i116 < 3; c2_i116++) {
        c2_dv7[c2_i116 + c2_i114] = 0.0;
      }

      c2_i114 += 16;
    }

    c2_i117 = 0;
    for (c2_i118 = 0; c2_i118 < 3; c2_i118++) {
      for (c2_i119 = 0; c2_i119 < 3; c2_i119++) {
        c2_dv7[(c2_i119 + c2_i117) + 3] = 0.0;
      }

      c2_i117 += 16;
    }

    c2_i120 = 0;
    c2_i121 = 0;
    for (c2_i122 = 0; c2_i122 < 3; c2_i122++) {
      for (c2_i123 = 0; c2_i123 < 3; c2_i123++) {
        c2_dv7[(c2_i123 + c2_i120) + 51] = c2_Gvwa[c2_i123 + c2_i121];
      }

      c2_i120 += 16;
      c2_i121 += 3;
    }

    c2_i124 = 0;
    for (c2_i125 = 0; c2_i125 < 6; c2_i125++) {
      for (c2_i126 = 0; c2_i126 < 3; c2_i126++) {
        c2_dv7[(c2_i126 + c2_i124) + 99] = 0.0;
      }

      c2_i124 += 16;
    }

    c2_i127 = 0;
    c2_i128 = 0;
    for (c2_i129 = 0; c2_i129 < 3; c2_i129++) {
      for (c2_i130 = 0; c2_i130 < 4; c2_i130++) {
        c2_dv7[(c2_i130 + c2_i127) + 6] = c2_Gqww[c2_i130 + c2_i128];
      }

      c2_i127 += 16;
      c2_i128 += 4;
    }

    c2_i131 = 0;
    for (c2_i132 = 0; c2_i132 < 9; c2_i132++) {
      for (c2_i133 = 0; c2_i133 < 4; c2_i133++) {
        c2_dv7[(c2_i133 + c2_i131) + 54] = 0.0;
      }

      c2_i131 += 16;
    }

    c2_i134 = 0;
    for (c2_i135 = 0; c2_i135 < 6; c2_i135++) {
      for (c2_i136 = 0; c2_i136 < 3; c2_i136++) {
        c2_dv7[(c2_i136 + c2_i134) + 10] = 0.0;
      }

      c2_i134 += 16;
    }

    c2_i137 = 0;
    c2_i138 = 0;
    for (c2_i139 = 0; c2_i139 < 3; c2_i139++) {
      for (c2_i140 = 0; c2_i140 < 3; c2_i140++) {
        c2_dv7[(c2_i140 + c2_i137) + 106] = c2_t_a[c2_i140 + c2_i138];
      }

      c2_i137 += 16;
      c2_i138 += 3;
    }

    c2_i141 = 0;
    for (c2_i142 = 0; c2_i142 < 3; c2_i142++) {
      for (c2_i143 = 0; c2_i143 < 3; c2_i143++) {
        c2_dv7[(c2_i143 + c2_i141) + 154] = 0.0;
      }

      c2_i141 += 16;
    }

    c2_i144 = 0;
    for (c2_i145 = 0; c2_i145 < 9; c2_i145++) {
      for (c2_i146 = 0; c2_i146 < 3; c2_i146++) {
        c2_dv7[(c2_i146 + c2_i144) + 13] = 0.0;
      }

      c2_i144 += 16;
    }

    c2_i147 = 0;
    c2_i148 = 0;
    for (c2_i149 = 0; c2_i149 < 3; c2_i149++) {
      for (c2_i150 = 0; c2_i150 < 3; c2_i150++) {
        c2_dv7[(c2_i150 + c2_i147) + 157] = c2_dv6[c2_i150 + c2_i148];
      }

      c2_i147 += 16;
      c2_i148 += 3;
    }

    c2_i151 = 0;
    for (c2_i152 = 0; c2_i152 < 12; c2_i152++) {
      for (c2_i153 = 0; c2_i153 < 16; c2_i153++) {
        c2_G[c2_i153 + c2_i151] = c2_Ts * c2_dv7[c2_i153 + c2_i151];
      }

      c2_i151 += 16;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 100);
    for (c2_i154 = 0; c2_i154 < 256; c2_i154++) {
      c2_f_hoistedGlobal[c2_i154] = chartInstance->c2_P[c2_i154];
    }

    for (c2_i155 = 0; c2_i155 < 256; c2_i155++) {
      c2_v_a[c2_i155] = c2_F[c2_i155];
    }

    c2_c_eml_scalar_eg(chartInstance);
    c2_c_eml_scalar_eg(chartInstance);
    for (c2_i156 = 0; c2_i156 < 256; c2_i156++) {
      c2_pb_y[c2_i156] = 0.0;
    }

    for (c2_i157 = 0; c2_i157 < 256; c2_i157++) {
      c2_w_a[c2_i157] = c2_v_a[c2_i157];
    }

    for (c2_i158 = 0; c2_i158 < 256; c2_i158++) {
      c2_g_hoistedGlobal[c2_i158] = c2_f_hoistedGlobal[c2_i158];
    }

    c2_d_eml_xgemm(chartInstance, c2_w_a, c2_g_hoistedGlobal, c2_pb_y);
    c2_i159 = 0;
    for (c2_i160 = 0; c2_i160 < 16; c2_i160++) {
      c2_i161 = 0;
      for (c2_i162 = 0; c2_i162 < 16; c2_i162++) {
        c2_f_hoistedGlobal[c2_i162 + c2_i159] = c2_F[c2_i161 + c2_i160];
        c2_i161 += 16;
      }

      c2_i159 += 16;
    }

    c2_c_eml_scalar_eg(chartInstance);
    c2_c_eml_scalar_eg(chartInstance);
    for (c2_i163 = 0; c2_i163 < 256; c2_i163++) {
      c2_v_a[c2_i163] = 0.0;
    }

    for (c2_i164 = 0; c2_i164 < 256; c2_i164++) {
      c2_qb_y[c2_i164] = c2_pb_y[c2_i164];
    }

    for (c2_i165 = 0; c2_i165 < 256; c2_i165++) {
      c2_h_hoistedGlobal[c2_i165] = c2_f_hoistedGlobal[c2_i165];
    }

    c2_d_eml_xgemm(chartInstance, c2_qb_y, c2_h_hoistedGlobal, c2_v_a);
    for (c2_i166 = 0; c2_i166 < 192; c2_i166++) {
      c2_x_a[c2_i166] = c2_G[c2_i166];
    }

    c2_d_eml_scalar_eg(chartInstance);
    c2_d_eml_scalar_eg(chartInstance);
    for (c2_i167 = 0; c2_i167 < 192; c2_i167++) {
      c2_rb_y[c2_i167] = 0.0;
    }

    for (c2_i168 = 0; c2_i168 < 192; c2_i168++) {
      c2_y_a[c2_i168] = c2_x_a[c2_i168];
    }

    for (c2_i169 = 0; c2_i169 < 144; c2_i169++) {
      c2_nb_b[c2_i169] = c2_b[c2_i169];
    }

    c2_e_eml_xgemm(chartInstance, c2_y_a, c2_nb_b, c2_rb_y);
    c2_i170 = 0;
    for (c2_i171 = 0; c2_i171 < 16; c2_i171++) {
      c2_i172 = 0;
      for (c2_i173 = 0; c2_i173 < 12; c2_i173++) {
        c2_ob_b[c2_i173 + c2_i170] = c2_G[c2_i172 + c2_i171];
        c2_i172 += 16;
      }

      c2_i170 += 12;
    }

    c2_e_eml_scalar_eg(chartInstance);
    c2_e_eml_scalar_eg(chartInstance);
    for (c2_i174 = 0; c2_i174 < 256; c2_i174++) {
      c2_pb_y[c2_i174] = 0.0;
    }

    for (c2_i175 = 0; c2_i175 < 192; c2_i175++) {
      c2_sb_y[c2_i175] = c2_rb_y[c2_i175];
    }

    for (c2_i176 = 0; c2_i176 < 192; c2_i176++) {
      c2_pb_b[c2_i176] = c2_ob_b[c2_i176];
    }

    c2_f_eml_xgemm(chartInstance, c2_sb_y, c2_pb_b, c2_pb_y);
    for (c2_i177 = 0; c2_i177 < 256; c2_i177++) {
      chartInstance->c2_P[c2_i177] = c2_v_a[c2_i177] + c2_pb_y[c2_i177];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 104);
  for (c2_i178 = 0; c2_i178 < 16; c2_i178++) {
    c2_Xout[c2_i178] = chartInstance->c2_X[c2_i178];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 105);
  for (c2_i179 = 0; c2_i179 < 256; c2_i179++) {
    c2_Pout[c2_i179] = chartInstance->c2_P[c2_i179];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -105);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i180 = 0; c2_i180 < 16; c2_i180++) {
    (*c2_b_Xout)[c2_i180] = c2_Xout[c2_i180];
  }

  for (c2_i181 = 0; c2_i181 < 256; c2_i181++) {
    (*c2_b_Pout)[c2_i181] = c2_Pout[c2_i181];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
}

static void registerMessagesc2_quadtestmodel(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_oldt_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_oldt, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_oldt), &c2_thisId);
  sf_mex_destroy(&c2_b_oldt);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_oldt_not_empty = FALSE;
  } else {
    chartInstance->c2_oldt_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d0;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_oldt;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_b_oldt = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_oldt), &c2_thisId);
  sf_mex_destroy(&c2_b_oldt);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  if (!chartInstance->c2_oldagsamp_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_oldagsamp, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_oldagsamp),
    &c2_thisId);
  sf_mex_destroy(&c2_b_oldagsamp);
  return c2_y;
}

static real_T c2_d_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d1;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_oldagsamp_not_empty = FALSE;
  } else {
    chartInstance->c2_oldagsamp_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d1, 1, 0, 0U, 0, 0U, 0);
    c2_y = c2_d1;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_oldagsamp;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_b_oldagsamp = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_oldagsamp),
    &c2_thisId);
  sf_mex_destroy(&c2_b_oldagsamp);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i182;
  real_T c2_b_inData[16];
  int32_T c2_i183;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i182 = 0; c2_i182 < 16; c2_i182++) {
    c2_b_inData[c2_i182] = (*(real_T (*)[16])c2_inData)[c2_i182];
  }

  for (c2_i183 = 0; c2_i183 < 16; c2_i183++) {
    c2_u[c2_i183] = c2_b_inData[c2_i183];
  }

  c2_y = NULL;
  if (!chartInstance->c2_X_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 16), FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_X, const char_T *c2_identifier, real_T
  c2_y[16])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_X), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_X);
}

static void c2_f_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16])
{
  real_T c2_dv8[16];
  int32_T c2_i184;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_X_not_empty = FALSE;
  } else {
    chartInstance->c2_X_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv8, 1, 0, 0U, 1, 0U, 1, 16);
    for (c2_i184 = 0; c2_i184 < 16; c2_i184++) {
      c2_y[c2_i184] = c2_dv8[c2_i184];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_X;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i185;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_b_X = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_X), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_X);
  for (c2_i185 = 0; c2_i185 < 16; c2_i185++) {
    (*(real_T (*)[16])c2_outData)[c2_i185] = c2_y[c2_i185];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i186;
  int32_T c2_i187;
  int32_T c2_i188;
  real_T c2_b_inData[256];
  int32_T c2_i189;
  int32_T c2_i190;
  int32_T c2_i191;
  real_T c2_u[256];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i186 = 0;
  for (c2_i187 = 0; c2_i187 < 16; c2_i187++) {
    for (c2_i188 = 0; c2_i188 < 16; c2_i188++) {
      c2_b_inData[c2_i188 + c2_i186] = (*(real_T (*)[256])c2_inData)[c2_i188 +
        c2_i186];
    }

    c2_i186 += 16;
  }

  c2_i189 = 0;
  for (c2_i190 = 0; c2_i190 < 16; c2_i190++) {
    for (c2_i191 = 0; c2_i191 < 16; c2_i191++) {
      c2_u[c2_i191 + c2_i189] = c2_b_inData[c2_i191 + c2_i189];
    }

    c2_i189 += 16;
  }

  c2_y = NULL;
  if (!chartInstance->c2_P_not_empty) {
    sf_mex_assign(&c2_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), FALSE);
  } else {
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 16, 16),
                  FALSE);
  }

  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_P, const char_T *c2_identifier, real_T
  c2_y[256])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_P), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_P);
}

static void c2_h_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[256])
{
  real_T c2_dv9[256];
  int32_T c2_i192;
  if (mxIsEmpty(c2_u)) {
    chartInstance->c2_P_not_empty = FALSE;
  } else {
    chartInstance->c2_P_not_empty = TRUE;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv9, 1, 0, 0U, 1, 0U, 2, 16,
                  16);
    for (c2_i192 = 0; c2_i192 < 256; c2_i192++) {
      c2_y[c2_i192] = c2_dv9[c2_i192];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_P;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[256];
  int32_T c2_i193;
  int32_T c2_i194;
  int32_T c2_i195;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_b_P = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_P), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_P);
  c2_i193 = 0;
  for (c2_i194 = 0; c2_i194 < 16; c2_i194++) {
    for (c2_i195 = 0; c2_i195 < 16; c2_i195++) {
      (*(real_T (*)[256])c2_outData)[c2_i195 + c2_i193] = c2_y[c2_i195 + c2_i193];
    }

    c2_i193 += 16;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i196;
  int32_T c2_i197;
  int32_T c2_i198;
  real_T c2_b_inData[256];
  int32_T c2_i199;
  int32_T c2_i200;
  int32_T c2_i201;
  real_T c2_u[256];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i196 = 0;
  for (c2_i197 = 0; c2_i197 < 16; c2_i197++) {
    for (c2_i198 = 0; c2_i198 < 16; c2_i198++) {
      c2_b_inData[c2_i198 + c2_i196] = (*(real_T (*)[256])c2_inData)[c2_i198 +
        c2_i196];
    }

    c2_i196 += 16;
  }

  c2_i199 = 0;
  for (c2_i200 = 0; c2_i200 < 16; c2_i200++) {
    for (c2_i201 = 0; c2_i201 < 16; c2_i201++) {
      c2_u[c2_i201 + c2_i199] = c2_b_inData[c2_i201 + c2_i199];
    }

    c2_i199 += 16;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 16, 16), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_Pout, const char_T *c2_identifier, real_T
  c2_y[256])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Pout), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Pout);
}

static void c2_j_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[256])
{
  real_T c2_dv10[256];
  int32_T c2_i202;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv10, 1, 0, 0U, 1, 0U, 2, 16,
                16);
  for (c2_i202 = 0; c2_i202 < 256; c2_i202++) {
    c2_y[c2_i202] = c2_dv10[c2_i202];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Pout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[256];
  int32_T c2_i203;
  int32_T c2_i204;
  int32_T c2_i205;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_Pout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Pout), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Pout);
  c2_i203 = 0;
  for (c2_i204 = 0; c2_i204 < 16; c2_i204++) {
    for (c2_i205 = 0; c2_i205 < 16; c2_i205++) {
      (*(real_T (*)[256])c2_outData)[c2_i205 + c2_i203] = c2_y[c2_i205 + c2_i203];
    }

    c2_i203 += 16;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i206;
  real_T c2_b_inData[16];
  int32_T c2_i207;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i206 = 0; c2_i206 < 16; c2_i206++) {
    c2_b_inData[c2_i206] = (*(real_T (*)[16])c2_inData)[c2_i206];
  }

  for (c2_i207 = 0; c2_i207 < 16; c2_i207++) {
    c2_u[c2_i207] = c2_b_inData[c2_i207];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 16), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_k_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_Xout, const char_T *c2_identifier, real_T
  c2_y[16])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Xout), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Xout);
}

static void c2_l_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16])
{
  real_T c2_dv11[16];
  int32_T c2_i208;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv11, 1, 0, 0U, 1, 0U, 1, 16);
  for (c2_i208 = 0; c2_i208 < 16; c2_i208++) {
    c2_y[c2_i208] = c2_dv11[c2_i208];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Xout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i209;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_Xout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Xout), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Xout);
  for (c2_i209 = 0; c2_i209 < 16; c2_i209++) {
    (*(real_T (*)[16])c2_outData)[c2_i209] = c2_y[c2_i209];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i210;
  real_T c2_b_inData[3];
  int32_T c2_i211;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i210 = 0; c2_i210 < 3; c2_i210++) {
    c2_b_inData[c2_i210] = (*(real_T (*)[3])c2_inData)[c2_i210];
  }

  for (c2_i211 = 0; c2_i211 < 3; c2_i211++) {
    c2_u[c2_i211] = c2_b_inData[c2_i211];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_m_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d2;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d2, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d2;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i212;
  int32_T c2_i213;
  int32_T c2_i214;
  real_T c2_b_inData[192];
  int32_T c2_i215;
  int32_T c2_i216;
  int32_T c2_i217;
  real_T c2_u[192];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i212 = 0;
  for (c2_i213 = 0; c2_i213 < 12; c2_i213++) {
    for (c2_i214 = 0; c2_i214 < 16; c2_i214++) {
      c2_b_inData[c2_i214 + c2_i212] = (*(real_T (*)[192])c2_inData)[c2_i214 +
        c2_i212];
    }

    c2_i212 += 16;
  }

  c2_i215 = 0;
  for (c2_i216 = 0; c2_i216 < 12; c2_i216++) {
    for (c2_i217 = 0; c2_i217 < 16; c2_i217++) {
      c2_u[c2_i217 + c2_i215] = c2_b_inData[c2_i217 + c2_i215];
    }

    c2_i215 += 16;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 16, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_n_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[192])
{
  real_T c2_dv12[192];
  int32_T c2_i218;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv12, 1, 0, 0U, 1, 0U, 2, 16,
                12);
  for (c2_i218 = 0; c2_i218 < 192; c2_i218++) {
    c2_y[c2_i218] = c2_dv12[c2_i218];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_G;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[192];
  int32_T c2_i219;
  int32_T c2_i220;
  int32_T c2_i221;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_G = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_G), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_G);
  c2_i219 = 0;
  for (c2_i220 = 0; c2_i220 < 12; c2_i220++) {
    for (c2_i221 = 0; c2_i221 < 16; c2_i221++) {
      (*(real_T (*)[192])c2_outData)[c2_i221 + c2_i219] = c2_y[c2_i221 + c2_i219];
    }

    c2_i219 += 16;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i222;
  int32_T c2_i223;
  int32_T c2_i224;
  real_T c2_b_inData[12];
  int32_T c2_i225;
  int32_T c2_i226;
  int32_T c2_i227;
  real_T c2_u[12];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i222 = 0;
  for (c2_i223 = 0; c2_i223 < 3; c2_i223++) {
    for (c2_i224 = 0; c2_i224 < 4; c2_i224++) {
      c2_b_inData[c2_i224 + c2_i222] = (*(real_T (*)[12])c2_inData)[c2_i224 +
        c2_i222];
    }

    c2_i222 += 4;
  }

  c2_i225 = 0;
  for (c2_i226 = 0; c2_i226 < 3; c2_i226++) {
    for (c2_i227 = 0; c2_i227 < 4; c2_i227++) {
      c2_u[c2_i227 + c2_i225] = c2_b_inData[c2_i227 + c2_i225];
    }

    c2_i225 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_o_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[12])
{
  real_T c2_dv13[12];
  int32_T c2_i228;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv13, 1, 0, 0U, 1, 0U, 2, 4, 3);
  for (c2_i228 = 0; c2_i228 < 12; c2_i228++) {
    c2_y[c2_i228] = c2_dv13[c2_i228];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Gqww;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[12];
  int32_T c2_i229;
  int32_T c2_i230;
  int32_T c2_i231;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_Gqww = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Gqww), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Gqww);
  c2_i229 = 0;
  for (c2_i230 = 0; c2_i230 < 3; c2_i230++) {
    for (c2_i231 = 0; c2_i231 < 4; c2_i231++) {
      (*(real_T (*)[12])c2_outData)[c2_i231 + c2_i229] = c2_y[c2_i231 + c2_i229];
    }

    c2_i229 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i232;
  int32_T c2_i233;
  int32_T c2_i234;
  real_T c2_b_inData[9];
  int32_T c2_i235;
  int32_T c2_i236;
  int32_T c2_i237;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i232 = 0;
  for (c2_i233 = 0; c2_i233 < 3; c2_i233++) {
    for (c2_i234 = 0; c2_i234 < 3; c2_i234++) {
      c2_b_inData[c2_i234 + c2_i232] = (*(real_T (*)[9])c2_inData)[c2_i234 +
        c2_i232];
    }

    c2_i232 += 3;
  }

  c2_i235 = 0;
  for (c2_i236 = 0; c2_i236 < 3; c2_i236++) {
    for (c2_i237 = 0; c2_i237 < 3; c2_i237++) {
      c2_u[c2_i237 + c2_i235] = c2_b_inData[c2_i237 + c2_i235];
    }

    c2_i235 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_p_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv14[9];
  int32_T c2_i238;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv14, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i238 = 0; c2_i238 < 9; c2_i238++) {
    c2_y[c2_i238] = c2_dv14[c2_i238];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Gvwa;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i239;
  int32_T c2_i240;
  int32_T c2_i241;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_Gvwa = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Gvwa), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Gvwa);
  c2_i239 = 0;
  for (c2_i240 = 0; c2_i240 < 3; c2_i240++) {
    for (c2_i241 = 0; c2_i241 < 3; c2_i241++) {
      (*(real_T (*)[9])c2_outData)[c2_i241 + c2_i239] = c2_y[c2_i241 + c2_i239];
    }

    c2_i239 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i242;
  int32_T c2_i243;
  int32_T c2_i244;
  real_T c2_b_inData[16];
  int32_T c2_i245;
  int32_T c2_i246;
  int32_T c2_i247;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i242 = 0;
  for (c2_i243 = 0; c2_i243 < 4; c2_i243++) {
    for (c2_i244 = 0; c2_i244 < 4; c2_i244++) {
      c2_b_inData[c2_i244 + c2_i242] = (*(real_T (*)[16])c2_inData)[c2_i244 +
        c2_i242];
    }

    c2_i242 += 4;
  }

  c2_i245 = 0;
  for (c2_i246 = 0; c2_i246 < 4; c2_i246++) {
    for (c2_i247 = 0; c2_i247 < 4; c2_i247++) {
      c2_u[c2_i247 + c2_i245] = c2_b_inData[c2_i247 + c2_i245];
    }

    c2_i245 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_q_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16])
{
  real_T c2_dv15[16];
  int32_T c2_i248;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv15, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c2_i248 = 0; c2_i248 < 16; c2_i248++) {
    c2_y[c2_i248] = c2_dv15[c2_i248];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Fqq;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i249;
  int32_T c2_i250;
  int32_T c2_i251;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_Fqq = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Fqq), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Fqq);
  c2_i249 = 0;
  for (c2_i250 = 0; c2_i250 < 4; c2_i250++) {
    for (c2_i251 = 0; c2_i251 < 4; c2_i251++) {
      (*(real_T (*)[16])c2_outData)[c2_i251 + c2_i249] = c2_y[c2_i251 + c2_i249];
    }

    c2_i249 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i252;
  int32_T c2_i253;
  int32_T c2_i254;
  real_T c2_b_inData[12];
  int32_T c2_i255;
  int32_T c2_i256;
  int32_T c2_i257;
  real_T c2_u[12];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i252 = 0;
  for (c2_i253 = 0; c2_i253 < 4; c2_i253++) {
    for (c2_i254 = 0; c2_i254 < 3; c2_i254++) {
      c2_b_inData[c2_i254 + c2_i252] = (*(real_T (*)[12])c2_inData)[c2_i254 +
        c2_i252];
    }

    c2_i252 += 3;
  }

  c2_i255 = 0;
  for (c2_i256 = 0; c2_i256 < 4; c2_i256++) {
    for (c2_i257 = 0; c2_i257 < 3; c2_i257++) {
      c2_u[c2_i257 + c2_i255] = c2_b_inData[c2_i257 + c2_i255];
    }

    c2_i255 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_r_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[12])
{
  real_T c2_dv16[12];
  int32_T c2_i258;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv16, 1, 0, 0U, 1, 0U, 2, 3, 4);
  for (c2_i258 = 0; c2_i258 < 12; c2_i258++) {
    c2_y[c2_i258] = c2_dv16[c2_i258];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Fvq;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[12];
  int32_T c2_i259;
  int32_T c2_i260;
  int32_T c2_i261;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_Fvq = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Fvq), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Fvq);
  c2_i259 = 0;
  for (c2_i260 = 0; c2_i260 < 4; c2_i260++) {
    for (c2_i261 = 0; c2_i261 < 3; c2_i261++) {
      (*(real_T (*)[12])c2_outData)[c2_i261 + c2_i259] = c2_y[c2_i261 + c2_i259];
    }

    c2_i259 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i262;
  real_T c2_b_inData[4];
  int32_T c2_i263;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i262 = 0; c2_i262 < 4; c2_i262++) {
    c2_b_inData[c2_i262] = (*(real_T (*)[4])c2_inData)[c2_i262];
  }

  for (c2_i263 = 0; c2_i263 < 4; c2_i263++) {
    c2_u[c2_i263] = c2_b_inData[c2_i263];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_s_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv17[4];
  int32_T c2_i264;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv17, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i264 = 0; c2_i264 < 4; c2_i264++) {
    c2_y[c2_i264] = c2_dv17[c2_i264];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Fvqparts;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i265;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_Fvqparts = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Fvqparts), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Fvqparts);
  for (c2_i265 = 0; c2_i265 < 4; c2_i265++) {
    (*(real_T (*)[4])c2_outData)[c2_i265] = c2_y[c2_i265];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_t_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv18[3];
  int32_T c2_i266;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv18, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i266 = 0; c2_i266 < 3; c2_i266++) {
    c2_y[c2_i266] = c2_dv18[c2_i266];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_a;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i267;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_a = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_a), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_a);
  for (c2_i267 = 0; c2_i267 < 3; c2_i267++) {
    (*(real_T (*)[3])c2_outData)[c2_i267] = c2_y[c2_i267];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i268;
  int32_T c2_i269;
  int32_T c2_i270;
  real_T c2_b_inData[144];
  int32_T c2_i271;
  int32_T c2_i272;
  int32_T c2_i273;
  real_T c2_u[144];
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i268 = 0;
  for (c2_i269 = 0; c2_i269 < 12; c2_i269++) {
    for (c2_i270 = 0; c2_i270 < 12; c2_i270++) {
      c2_b_inData[c2_i270 + c2_i268] = (*(real_T (*)[144])c2_inData)[c2_i270 +
        c2_i268];
    }

    c2_i268 += 12;
  }

  c2_i271 = 0;
  for (c2_i272 = 0; c2_i272 < 12; c2_i272++) {
    for (c2_i273 = 0; c2_i273 < 12; c2_i273++) {
      c2_u[c2_i273 + c2_i271] = c2_b_inData[c2_i273 + c2_i271];
    }

    c2_i271 += 12;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 12, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

const mxArray *sf_c2_quadtestmodel_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[53];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i274;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 53), FALSE);
  for (c2_i274 = 0; c2_i274 < 53; c2_i274++) {
    c2_r0 = &c2_info[c2_i274];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i274);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i274);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i274);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i274);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i274);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i274);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i274);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i274);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[53])
{
  c2_info[0].context = "";
  c2_info[0].name = "diag";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[0].fileTimeLo = 1286822286U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[1].name = "eml_index_class";
  c2_info[1].dominantType = "";
  c2_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[1].fileTimeLo = 1323174178U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[2].name = "eml_index_plus";
  c2_info[2].dominantType = "coder.internal.indexInt";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[2].fileTimeLo = 1286822378U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[3].name = "eml_index_class";
  c2_info[3].dominantType = "";
  c2_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[3].fileTimeLo = 1323174178U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[4].name = "eml_scalar_eg";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[4].fileTimeLo = 1286822396U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m";
  c2_info[5].name = "eml_int_forloop_overflow_check";
  c2_info[5].dominantType = "";
  c2_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[5].fileTimeLo = 1346513940U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[6].name = "intmax";
  c2_info[6].dominantType = "char";
  c2_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[6].fileTimeLo = 1311258916U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context = "";
  c2_info[7].name = "mtimes";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[7].fileTimeLo = 1289523292U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[8].name = "eml_index_class";
  c2_info[8].dominantType = "";
  c2_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[8].fileTimeLo = 1323174178U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[9].name = "eml_scalar_eg";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[9].fileTimeLo = 1286822396U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[10].name = "eml_xgemm";
  c2_info[10].dominantType = "char";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[10].fileTimeLo = 1299080372U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[11].name = "eml_blas_inline";
  c2_info[11].dominantType = "";
  c2_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[11].fileTimeLo = 1299080368U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[12].name = "mtimes";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[12].fileTimeLo = 1289523292U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[13].name = "eml_index_class";
  c2_info[13].dominantType = "";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[13].fileTimeLo = 1323174178U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[14].name = "eml_scalar_eg";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[14].fileTimeLo = 1286822396U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[15].name = "eml_refblas_xgemm";
  c2_info[15].dominantType = "char";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[15].fileTimeLo = 1299080374U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context = "";
  c2_info[16].name = "norm";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c2_info[16].fileTimeLo = 1336525694U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c2_info[17].name = "eml_index_class";
  c2_info[17].dominantType = "";
  c2_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[17].fileTimeLo = 1323174178U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c2_info[18].name = "eml_xnrm2";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[18].fileTimeLo = 1299080376U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[19].name = "eml_blas_inline";
  c2_info[19].dominantType = "";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[19].fileTimeLo = 1299080368U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[20].name = "eml_index_class";
  c2_info[20].dominantType = "";
  c2_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[20].fileTimeLo = 1323174178U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 0U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[21].name = "eml_refblas_xnrm2";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[21].fileTimeLo = 1299080384U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[22].name = "realmin";
  c2_info[22].dominantType = "char";
  c2_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[22].fileTimeLo = 1307654842U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[23].name = "eml_realmin";
  c2_info[23].dominantType = "char";
  c2_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[23].fileTimeLo = 1307654844U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[24].name = "eml_float_model";
  c2_info[24].dominantType = "char";
  c2_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[24].fileTimeLo = 1326731596U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[25].name = "eml_index_class";
  c2_info[25].dominantType = "";
  c2_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[25].fileTimeLo = 1323174178U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[26].name = "eml_index_minus";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[26].fileTimeLo = 1286822378U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[27].name = "eml_index_class";
  c2_info[27].dominantType = "";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[27].fileTimeLo = 1323174178U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[28].name = "eml_index_times";
  c2_info[28].dominantType = "coder.internal.indexInt";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[28].fileTimeLo = 1286822380U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[29].name = "eml_index_class";
  c2_info[29].dominantType = "";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[29].fileTimeLo = 1323174178U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[30].name = "eml_index_plus";
  c2_info[30].dominantType = "coder.internal.indexInt";
  c2_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[30].fileTimeLo = 1286822378U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[31].name = "eml_int_forloop_overflow_check";
  c2_info[31].dominantType = "";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[31].fileTimeLo = 1346513940U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[32].name = "abs";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[32].fileTimeLo = 1343833966U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[33].name = "eml_scalar_abs";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[33].fileTimeLo = 1286822312U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context = "";
  c2_info[34].name = "mrdivide";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[34].fileTimeLo = 1357955148U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 1319733566U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[35].name = "rdivide";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[35].fileTimeLo = 1346513988U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[36].name = "eml_scalexp_compatible";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[36].fileTimeLo = 1286822396U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[37].name = "eml_div";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[37].fileTimeLo = 1313351410U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context = "";
  c2_info[38].name = "eye";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m";
  c2_info[38].fileTimeLo = 1286822288U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[39].name = "eml_assert_valid_size_arg";
  c2_info[39].dominantType = "double";
  c2_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[39].fileTimeLo = 1286822294U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c2_info[40].name = "isinf";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c2_info[40].fileTimeLo = 1286822360U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c2_info[41].name = "mtimes";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[41].fileTimeLo = 1289523292U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[42].name = "eml_index_class";
  c2_info[42].dominantType = "";
  c2_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[42].fileTimeLo = 1323174178U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c2_info[43].name = "intmax";
  c2_info[43].dominantType = "char";
  c2_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[43].fileTimeLo = 1311258916U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[44].name = "eml_is_float_class";
  c2_info[44].dominantType = "char";
  c2_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[44].fileTimeLo = 1286822382U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[45].name = "min";
  c2_info[45].dominantType = "double";
  c2_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[45].fileTimeLo = 1311258918U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[46].name = "eml_min_or_max";
  c2_info[46].dominantType = "char";
  c2_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[46].fileTimeLo = 1334075090U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[47].name = "eml_scalar_eg";
  c2_info[47].dominantType = "double";
  c2_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[47].fileTimeLo = 1286822396U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[48].name = "eml_scalexp_alloc";
  c2_info[48].dominantType = "double";
  c2_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[48].fileTimeLo = 1352428460U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 0U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[49].name = "eml_index_class";
  c2_info[49].dominantType = "";
  c2_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[49].fileTimeLo = 1323174178U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[50].name = "eml_scalar_eg";
  c2_info[50].dominantType = "double";
  c2_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[50].fileTimeLo = 1286822396U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[51].name = "eml_index_class";
  c2_info[51].dominantType = "";
  c2_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[51].fileTimeLo = 1323174178U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c2_info[52].name = "eml_int_forloop_overflow_check";
  c2_info[52].dominantType = "";
  c2_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[52].fileTimeLo = 1346513940U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
}

static void c2_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance)
{
}

static void c2_b_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance)
{
}

static real_T c2_norm(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T
                      c2_x[4])
{
  real_T c2_y;
  real_T c2_scale;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_y = 0.0;
  c2_scale = 2.2250738585072014E-308;
  for (c2_k = 1; c2_k < 5; c2_k++) {
    c2_b_k = c2_k;
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 4, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_absxk = muDoubleScalarAbs(c2_c_x);
    if (c2_absxk > c2_scale) {
      c2_t = c2_scale / c2_absxk;
      c2_y = 1.0 + c2_y * c2_t * c2_t;
      c2_scale = c2_absxk;
    } else {
      c2_t = c2_absxk / c2_scale;
      c2_y += c2_t * c2_t;
    }
  }

  return c2_scale * muDoubleScalarSqrt(c2_y);
}

static void c2_eye(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T c2_I
                   [256])
{
  int32_T c2_i275;
  int32_T c2_i;
  int32_T c2_b_i;
  for (c2_i275 = 0; c2_i275 < 256; c2_i275++) {
    c2_I[c2_i275] = 0.0;
  }

  for (c2_i = 1; c2_i < 17; c2_i++) {
    c2_b_i = c2_i;
    c2_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_i), 1, 16, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 16, 2, 0) - 1) <<
           4)) - 1] = 1.0;
  }
}

static void c2_b_eye(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T
                     c2_I[9])
{
  int32_T c2_i276;
  int32_T c2_i;
  int32_T c2_b_i;
  for (c2_i276 = 0; c2_i276 < 9; c2_i276++) {
    c2_I[c2_i276] = 0.0;
  }

  for (c2_i = 1; c2_i < 4; c2_i++) {
    c2_b_i = c2_i;
    c2_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_i), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 3, 2, 0) - 1)) -
      1] = 1.0;
  }
}

static void c2_c_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance)
{
}

static void c2_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance, real_T
  c2_A[256], real_T c2_B[256], real_T c2_C[256], real_T c2_b_C[256])
{
  int32_T c2_i277;
  int32_T c2_i278;
  real_T c2_b_A[256];
  int32_T c2_i279;
  real_T c2_b_B[256];
  for (c2_i277 = 0; c2_i277 < 256; c2_i277++) {
    c2_b_C[c2_i277] = c2_C[c2_i277];
  }

  for (c2_i278 = 0; c2_i278 < 256; c2_i278++) {
    c2_b_A[c2_i278] = c2_A[c2_i278];
  }

  for (c2_i279 = 0; c2_i279 < 256; c2_i279++) {
    c2_b_B[c2_i279] = c2_B[c2_i279];
  }

  c2_d_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_d_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance)
{
}

static void c2_b_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[144], real_T c2_C[192], real_T c2_b_C[192])
{
  int32_T c2_i280;
  int32_T c2_i281;
  real_T c2_b_A[192];
  int32_T c2_i282;
  real_T c2_b_B[144];
  for (c2_i280 = 0; c2_i280 < 192; c2_i280++) {
    c2_b_C[c2_i280] = c2_C[c2_i280];
  }

  for (c2_i281 = 0; c2_i281 < 192; c2_i281++) {
    c2_b_A[c2_i281] = c2_A[c2_i281];
  }

  for (c2_i282 = 0; c2_i282 < 144; c2_i282++) {
    c2_b_B[c2_i282] = c2_B[c2_i282];
  }

  c2_e_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static void c2_e_eml_scalar_eg(SFc2_quadtestmodelInstanceStruct *chartInstance)
{
}

static void c2_c_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[192], real_T c2_C[256], real_T c2_b_C[256])
{
  int32_T c2_i283;
  int32_T c2_i284;
  real_T c2_b_A[192];
  int32_T c2_i285;
  real_T c2_b_B[192];
  for (c2_i283 = 0; c2_i283 < 256; c2_i283++) {
    c2_b_C[c2_i283] = c2_C[c2_i283];
  }

  for (c2_i284 = 0; c2_i284 < 192; c2_i284++) {
    c2_b_A[c2_i284] = c2_A[c2_i284];
  }

  for (c2_i285 = 0; c2_i285 < 192; c2_i285++) {
    c2_b_B[c2_i285] = c2_B[c2_i285];
  }

  c2_f_eml_xgemm(chartInstance, c2_b_A, c2_b_B, c2_b_C);
}

static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_u_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i286;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i286, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i286;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_u_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_v_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_quadtestmodel, const char_T
  *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_w_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_quadtestmodel), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_quadtestmodel);
  return c2_y;
}

static uint8_T c2_w_emlrt_marshallIn(SFc2_quadtestmodelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[256], real_T c2_B[256], real_T c2_C[256])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(16);
  c2_n_t = (ptrdiff_t)(16);
  c2_k_t = (ptrdiff_t)(16);
  c2_lda_t = (ptrdiff_t)(16);
  c2_ldb_t = (ptrdiff_t)(16);
  c2_ldc_t = (ptrdiff_t)(16);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void c2_e_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[144], real_T c2_C[192])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(16);
  c2_n_t = (ptrdiff_t)(12);
  c2_k_t = (ptrdiff_t)(12);
  c2_lda_t = (ptrdiff_t)(16);
  c2_ldb_t = (ptrdiff_t)(12);
  c2_ldc_t = (ptrdiff_t)(16);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void c2_f_eml_xgemm(SFc2_quadtestmodelInstanceStruct *chartInstance,
  real_T c2_A[192], real_T c2_B[192], real_T c2_C[256])
{
  real_T c2_alpha1;
  real_T c2_beta1;
  char_T c2_TRANSB;
  char_T c2_TRANSA;
  ptrdiff_t c2_m_t;
  ptrdiff_t c2_n_t;
  ptrdiff_t c2_k_t;
  ptrdiff_t c2_lda_t;
  ptrdiff_t c2_ldb_t;
  ptrdiff_t c2_ldc_t;
  double * c2_alpha1_t;
  double * c2_Aia0_t;
  double * c2_Bib0_t;
  double * c2_beta1_t;
  double * c2_Cic0_t;
  c2_alpha1 = 1.0;
  c2_beta1 = 0.0;
  c2_TRANSB = 'N';
  c2_TRANSA = 'N';
  c2_m_t = (ptrdiff_t)(16);
  c2_n_t = (ptrdiff_t)(16);
  c2_k_t = (ptrdiff_t)(12);
  c2_lda_t = (ptrdiff_t)(16);
  c2_ldb_t = (ptrdiff_t)(12);
  c2_ldc_t = (ptrdiff_t)(16);
  c2_alpha1_t = (double *)(&c2_alpha1);
  c2_Aia0_t = (double *)(&c2_A[0]);
  c2_Bib0_t = (double *)(&c2_B[0]);
  c2_beta1_t = (double *)(&c2_beta1);
  c2_Cic0_t = (double *)(&c2_C[0]);
  dgemm(&c2_TRANSA, &c2_TRANSB, &c2_m_t, &c2_n_t, &c2_k_t, c2_alpha1_t,
        c2_Aia0_t, &c2_lda_t, c2_Bib0_t, &c2_ldb_t, c2_beta1_t, c2_Cic0_t,
        &c2_ldc_t);
}

static void init_dsm_address_info(SFc2_quadtestmodelInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
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

void sf_c2_quadtestmodel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1270516659U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3422728784U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2286467794U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2483071276U);
}

mxArray *sf_c2_quadtestmodel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("3zSoZQhhqBXbDg2j2jdnwF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(16);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(16);
      pr[1] = (double)(16);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_quadtestmodel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c2_quadtestmodel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[9],T\"Pout\",},{M[1],M[5],T\"Xout\",},{M[4],M[0],T\"P\",S'l','i','p'{{M1x2[106 107],M[0],}}},{M[4],M[0],T\"X\",S'l','i','p'{{M1x2[108 109],M[0],}}},{M[4],M[0],T\"oldagsamp\",S'l','i','p'{{M1x2[110 119],M[0],}}},{M[4],M[0],T\"oldt\",S'l','i','p'{{M1x2[120 124],M[0],}}},{M[8],M[0],T\"is_active_c2_quadtestmodel\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_quadtestmodel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_quadtestmodelInstanceStruct *chartInstance;
    chartInstance = (SFc2_quadtestmodelInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _quadtestmodelMachineNumber_,
           2,
           1,
           1,
           6,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_quadtestmodelMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_quadtestmodelMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _quadtestmodelMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"am");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Xout");
          _SFD_SET_DATA_PROPS(2,1,1,0,"wm");
          _SFD_SET_DATA_PROPS(3,1,1,0,"agsamp");
          _SFD_SET_DATA_PROPS(4,1,1,0,"t");
          _SFD_SET_DATA_PROPS(5,2,0,1,"Pout");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,5,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",33,-1,2848);
        _SFD_CV_INIT_EML_IF(0,1,0,125,141,-1,159);
        _SFD_CV_INIT_EML_IF(0,1,1,160,181,-1,204);
        _SFD_CV_INIT_EML_IF(0,1,2,205,218,-1,284);
        _SFD_CV_INIT_EML_IF(0,1,3,285,298,-1,321);
        _SFD_CV_INIT_EML_IF(0,1,4,409,431,-1,2815);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_h_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 16;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_f_sf_marshallOut,(MexInFcnForType)
            c2_f_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_h_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_g_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_g_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 16;
          dimVector[1]= 16;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)
            c2_e_sf_marshallIn);
        }

        {
          real_T *c2_agsamp;
          real_T *c2_t;
          real_T (*c2_am)[3];
          real_T (*c2_Xout)[16];
          real_T (*c2_wm)[3];
          real_T (*c2_Pout)[256];
          c2_Pout = (real_T (*)[256])ssGetOutputPortSignal(chartInstance->S, 2);
          c2_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_agsamp = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_wm = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c2_Xout = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_am = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_am);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_Xout);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_wm);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_agsamp);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_t);
          _SFD_SET_DATA_VALUE_PTR(5U, *c2_Pout);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _quadtestmodelMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "gp4QVgl6IBev6dLRXoAzQC";
}

static void sf_opaque_initialize_c2_quadtestmodel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_quadtestmodelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*)
    chartInstanceVar);
  initialize_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_quadtestmodel(void *chartInstanceVar)
{
  enable_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_quadtestmodel(void *chartInstanceVar)
{
  disable_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_quadtestmodel(void *chartInstanceVar)
{
  sf_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_quadtestmodel(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_quadtestmodel
    ((SFc2_quadtestmodelInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_quadtestmodel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_quadtestmodel(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_quadtestmodel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_quadtestmodel(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_quadtestmodel(S);
}

static void sf_opaque_set_sim_state_c2_quadtestmodel(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c2_quadtestmodel(S, st);
}

static void sf_opaque_terminate_c2_quadtestmodel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_quadtestmodelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_quadtestmodel_optimization_info();
    }

    finalize_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_quadtestmodel(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_quadtestmodel((SFc2_quadtestmodelInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_quadtestmodel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_quadtestmodel_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1549325252U));
  ssSetChecksum1(S,(1251126574U));
  ssSetChecksum2(S,(2276402844U));
  ssSetChecksum3(S,(2737095998U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_quadtestmodel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_quadtestmodel(SimStruct *S)
{
  SFc2_quadtestmodelInstanceStruct *chartInstance;
  chartInstance = (SFc2_quadtestmodelInstanceStruct *)utMalloc(sizeof
    (SFc2_quadtestmodelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_quadtestmodelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_quadtestmodel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_quadtestmodel;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_quadtestmodel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_quadtestmodel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_quadtestmodel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_quadtestmodel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_quadtestmodel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_quadtestmodel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_quadtestmodel;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_quadtestmodel;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_quadtestmodel;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_quadtestmodel_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_quadtestmodel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_quadtestmodel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_quadtestmodel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_quadtestmodel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
