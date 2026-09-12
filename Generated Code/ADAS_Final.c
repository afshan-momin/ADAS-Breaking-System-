
#include "ADAS.h"
#include "rtwtypes.h"
#include <string.h>
#include "ADAS_private.h"

/* External inputs (root inport signals with default storage) */
ExtU_ADAS_T ADAS_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_ADAS_T ADAS_Y;

/* Real-time model */
static RT_MODEL_ADAS_T ADAS_M_;
RT_MODEL_ADAS_T *const ADAS_M = &ADAS_M_;

/* Model step function */
void ADAS_step(void)
{
  real_T rtb_Final_Fused_Dist;
  int32_T tmp;
  int32_T tmp_0;
  int32_T tmp_1;
  uint32_T tmp_2;

  /* Outputs for Atomic SubSystem: '<Root>/ADAS' */
  /* Switch: '<S1>/Final_Fused_Dist' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S6>/Constant'
   *  Gain: '<S1>/Gain'
   *  Gain: '<S1>/Gain1'
   *  Gain: '<S1>/Gain2'
   *  Gain: '<S1>/Gain3'
   *  Gain: '<S1>/Gain4'
   *  Inport: '<Root>/LDR_in'
   *  Inport: '<Root>/Lidar_dist'
   *  Logic: '<S1>/Logical Operator1'
   *  RelationalOperator: '<S3>/Compare'
   *  RelationalOperator: '<S6>/Compare'
   *  Sum: '<S1>/Fused_dist'
   *  Sum: '<S1>/Fused_dist1'
   *  Switch: '<S1>/Switch1'
   *  Switch: '<S1>/Switch3'
   */
  if (ADAS_U.LDR_in) {
    /* Switch: '<S1>/Switch' incorporates:
     *  Constant: '<S1>/Constant'
     *  Inport: '<Root>/Camera_dist'
     */
    if (ADAS_U.LDR_in) {
      tmp = ADAS_U.Camera_dist;
    } else {
      tmp = 0;
    }

    /* Switch: '<S1>/Switch1' incorporates:
     *  Constant: '<S1>/Constant2'
     *  Constant: '<S3>/Constant'
     *  Constant: '<S6>/Constant'
     *  Inport: '<Root>/Lidar_dist'
     *  Logic: '<S1>/Logical Operator1'
     *  RelationalOperator: '<S3>/Compare'
     *  RelationalOperator: '<S6>/Compare'
     */
    if ((ADAS_U.Lidar_dist > 0) && (ADAS_U.Lidar_dist <= 5)) {
      tmp_0 = ADAS_U.Lidar_dist;
    } else {
      tmp_0 = 0;
    }

    /* Switch: '<S1>/Switch3' incorporates:
     *  Constant: '<S10>/Constant'
     *  Constant: '<S11>/Constant'
     *  Constant: '<S1>/Constant1'
     *  Inport: '<Root>/Radar_dist'
     *  Logic: '<S1>/Logical Operator3'
     *  RelationalOperator: '<S10>/Compare'
     *  RelationalOperator: '<S11>/Compare'
     */
    if ((ADAS_U.Radar_dist > 0) && (ADAS_U.Radar_dist <= 5)) {
      tmp_1 = ADAS_U.Radar_dist;
    } else {
      tmp_1 = 0;
    }

    /* Gain: '<S1>/Gain' incorporates:
     *  Constant: '<S7>/Constant'
     *  RelationalOperator: '<S7>/Compare'
     *  Switch: '<S1>/Switch'
     */
    if (tmp <= 5) {
      tmp_2 = 3435973837U;
    } else {
      tmp_2 = 0U;
    }

    rtb_Final_Fused_Dist = ((real_T)tmp_2 * 1.1641532182693481E-10 + 0.4 *
      (real_T)tmp_0) + 0.2 * (real_T)tmp_1;
  } else {
    if ((ADAS_U.Lidar_dist > 0) && (ADAS_U.Lidar_dist <= 5)) {
      /* Switch: '<S1>/Switch1' incorporates:
       *  Inport: '<Root>/Lidar_dist'
       */
      tmp = ADAS_U.Lidar_dist;
    } else {
      /* Switch: '<S1>/Switch1' incorporates:
       *  Constant: '<S1>/Constant2'
       */
      tmp = 0;
    }

    /* Switch: '<S1>/Switch3' incorporates:
     *  Constant: '<S10>/Constant'
     *  Constant: '<S11>/Constant'
     *  Constant: '<S1>/Constant1'
     *  Inport: '<Root>/Radar_dist'
     *  Logic: '<S1>/Logical Operator3'
     *  RelationalOperator: '<S10>/Compare'
     *  RelationalOperator: '<S11>/Compare'
     */
    if ((ADAS_U.Radar_dist > 0) && (ADAS_U.Radar_dist <= 5)) {
      tmp_0 = ADAS_U.Radar_dist;
    } else {
      tmp_0 = 0;
    }

    rtb_Final_Fused_Dist = 0.6 * (real_T)tmp + 0.4 * (real_T)tmp_0;
  }

  /* End of Switch: '<S1>/Final_Fused_Dist' */

  /* Switch: '<S1>/Switch2' incorporates:
   *  Constant: '<S1>/Constant4'
   *  Constant: '<S8>/Constant'
   *  Constant: '<S9>/Constant'
   *  Inport: '<Root>/Radar_speed_ms'
   *  Logic: '<S1>/Logical Operator2'
   *  RelationalOperator: '<S8>/Compare'
   *  RelationalOperator: '<S9>/Compare'
   */
  if ((ADAS_U.Radar_speed_ms >= 13) && (ADAS_U.Radar_speed_ms <= 55)) {
    tmp = ADAS_U.Radar_speed_ms;
  } else {
    tmp = 0;
  }

  /* Product: '<S1>/Time' incorporates:
   *  Switch: '<S1>/Switch2'
   */
  rtb_Final_Fused_Dist /= (real_T)tmp;

  /* Switch: '<S1>/Final_Fused_Dist1' incorporates:
   *  Constant: '<S13>/Constant'
   *  Constant: '<S4>/Constant'
   *  Constant: '<S5>/Constant'
   *  Logic: '<S1>/Logical Operator'
   *  RelationalOperator: '<S13>/Compare'
   *  RelationalOperator: '<S4>/Compare'
   *  RelationalOperator: '<S5>/Compare'
   *  Switch: '<S1>/Final_Fused_Dist2'
   */
  if (rtb_Final_Fused_Dist < 0.169) {
    /* Outport: '<Root>/Output ' incorporates:
     *  Constant: '<S1>/Constant3'
     */
    ADAS_Y.Output = 1.0;
  } else if ((rtb_Final_Fused_Dist >= 0.169) && (rtb_Final_Fused_Dist <= 5.0)) {
    /* Switch: '<S1>/Final_Fused_Dist2' incorporates:
     *  Constant: '<S1>/Constant5'
     *  Outport: '<Root>/Output '
     */
    ADAS_Y.Output = 2.0;
  } else {
    /* Outport: '<Root>/Output ' incorporates:
     *  Constant: '<S1>/Constant6'
     *  Switch: '<S1>/Final_Fused_Dist2'
     */
    ADAS_Y.Output = 0.0;
  }

  /* End of Switch: '<S1>/Final_Fused_Dist1' */
  /* End of Outputs for SubSystem: '<Root>/ADAS' */

  /* Matfile logging */
  rt_UpdateTXYLogVars(ADAS_M->rtwLogInfo, (&ADAS_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.2s, 0.0s] */
    if ((rtmGetTFinal(ADAS_M)!=-1) &&
        !((rtmGetTFinal(ADAS_M)-ADAS_M->Timing.taskTime0) >
          ADAS_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(ADAS_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++ADAS_M->Timing.clockTick0)) {
    ++ADAS_M->Timing.clockTickH0;
  }

  ADAS_M->Timing.taskTime0 = ADAS_M->Timing.clockTick0 *
    ADAS_M->Timing.stepSize0 + ADAS_M->Timing.clockTickH0 *
    ADAS_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void ADAS_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)ADAS_M, 0,
                sizeof(RT_MODEL_ADAS_T));
  rtmSetTFinal(ADAS_M, 10.0);
  ADAS_M->Timing.stepSize0 = 0.2;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    ADAS_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(ADAS_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(ADAS_M->rtwLogInfo, (NULL));
    rtliSetLogT(ADAS_M->rtwLogInfo, "tout");
    rtliSetLogX(ADAS_M->rtwLogInfo, "");
    rtliSetLogXFinal(ADAS_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(ADAS_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(ADAS_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(ADAS_M->rtwLogInfo, 0);
    rtliSetLogDecimation(ADAS_M->rtwLogInfo, 1);
    rtliSetLogY(ADAS_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(ADAS_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(ADAS_M->rtwLogInfo, (NULL));
  }

  /* external inputs */
  (void)memset(&ADAS_U, 0, sizeof(ExtU_ADAS_T));

  /* external outputs */
  ADAS_Y.Output = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(ADAS_M->rtwLogInfo, 0.0, rtmGetTFinal(ADAS_M),
    ADAS_M->Timing.stepSize0, (&rtmGetErrorStatus(ADAS_M)));
}

/* Model terminate function */
void ADAS_terminate(void)
{
  /* (no terminate code required) */
}