/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
#include "crtp.h"
#include "motors.h"
#include "lqr.h"


#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static bool isInit;
static float z_ref;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

u_values_m u_base;
u_values_m u_equilibrium;
v_system_m v_sys;
v_system_m v_sys_pre;
w_control_m w_control;

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  static uint32_t tick_t = 0;
  static uint32_t dta_Tick = 0;
  z_ref = 843.0;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  u_base = initialize_lqr_u_variables();
  u_equilibrium = initialize_lqr_u_variables();
  v_sys = initialize_lqr_v_variables();
  v_sys_pre = initialize_lqr_v_variables();

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    getExtPosition(&state);
#ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorUpdate(&state, &sensorData, &control);
#else
    sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, &sensorData, tick);
#endif

    commanderGetSetpoint(&setpoint, &state);
    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    u_equilibrium = set_u (&u_base, &sensorData, &state, z_ref, &v_sys_pre);
    v_sys = set_dyn_model (&v_sys_pre , &sensorData, &state, &u_equilibrium);
    v_sys_pre = v_sys;

    w_control = calculate_w(&u_equilibrium);

    tick_t = xTaskGetTickCount();
    dta_Tick = tick_t - get_last_Tick();
    if ( dta_Tick < 1000)
    {
        /*motorsSetRatio(MOTOR_M1, w1_motor);
        motorsSetRatio(MOTOR_M2, w2_motor);
        motorsSetRatio(MOTOR_M3, w3_motor);
        motorsSetRatio(MOTOR_M4, w4_motor);*/
        motorsSetRatio(MOTOR_M1, 3000);
        motorsSetRatio(MOTOR_M2, 0);
        motorsSetRatio(MOTOR_M3, 0);
        motorsSetRatio(MOTOR_M4, 0);
    }
    else
    {
        motorsSetRatio(MOTOR_M1, 0);
        motorsSetRatio(MOTOR_M2, 0);
        motorsSetRatio(MOTOR_M3, 0);
        motorsSetRatio(MOTOR_M4, 0);
    }
    //stateController(&control, &setpoint, &sensorData, &state, tick);
    //powerDistribution(&control);

    tick++;
  }
}

LOG_GROUP_START(wcontrol)
LOG_ADD(LOG_FLOAT, w1, &w_control.w1)
LOG_ADD(LOG_FLOAT, w2, &w_control.w2)
LOG_ADD(LOG_FLOAT, w3, &w_control.w3)
LOG_ADD(LOG_FLOAT, w4, &w_control.w4)
LOG_GROUP_STOP(wcontrol)

LOG_GROUP_START(vsys)
LOG_ADD(LOG_FLOAT, z, &v_sys.z)
LOG_ADD(LOG_FLOAT, vz, &v_sys.vz)
LOG_ADD(LOG_FLOAT, yaw, &v_sys.yaw)
LOG_ADD(LOG_FLOAT, roll, &v_sys.roll)
LOG_ADD(LOG_FLOAT, pitch, &v_sys.pitch)
LOG_ADD(LOG_FLOAT, wx, &v_sys.wx)
LOG_ADD(LOG_FLOAT, wy, &v_sys.wy)
LOG_ADD(LOG_FLOAT, wz, &v_sys.wz)
LOG_GROUP_STOP(vsys)

LOG_GROUP_START(umatrix)
LOG_ADD(LOG_FLOAT, u1, &u_equilibrium.u_1)
LOG_ADD(LOG_FLOAT, u2, &u_equilibrium.u_2)
LOG_ADD(LOG_FLOAT, u3, &u_equilibrium.u_3)
LOG_ADD(LOG_FLOAT, u4, &u_equilibrium.u_4)
LOG_GROUP_STOP(umatrix)

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)
