/*
 * lqr.c
 *
 *  Created on: Mar 29, 2017
 *      Author: bitcraze
 */
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "sensors.h"
#include "math.h"
#include "lqr.h"

u_values_m set_u (const u_values_m *init, const sensorData_t *sensorData,
                  const state_t *state, float z_ref, v_system_m *vz_prev)
{
    u_values_m equi;
    equi.u_1 = - sensorData->baro.asl -((float)3.3434*vz_prev->z)+z_ref+init->u_1;
    equi.u_2 = -(float)5.7296* state->attitude.roll - (float)28.6479*sensorData->gyro.x;
    equi.u_3 = -(float)5.7296*-state->attitude.pitch - (float)28.6479*sensorData->gyro.y;
    equi.u_4 = -(float)5.7296* state->attitude.yaw - (float)28.6479*sensorData->gyro.z;

    return equi;
}

float update_z (float *past_z, float current_z)
{
    float vz = 0.0;
    vz = (current_z - *past_z)/RATE_MAIN_LOOP;
    *past_z = current_z;
    return vz;
}

v_system_m set_dyn_model (v_system_m *v_system_pre, const sensorData_t *sensorData, const state_t *state, const u_values_m *u_values)
{
    v_system_m v_system;
    float past_vz = v_system_pre->z;
    v_system.z=update_z(&past_vz, sensorData->baro.asl);
    v_system.vz=-GRAV+((1/MASS)*(float)cos(-state->attitude.pitch)*(float)cos(state->attitude.roll)*u_values->u_1);
    v_system.yaw=(((float)sin(state->attitude.roll)/(float)cos(-state->attitude.pitch))*sensorData->gyro.y)+
                  (((float)cos(state->attitude.roll)/(float)cos(-state->attitude.pitch))*sensorData->gyro.z);
    v_system.pitch=((float)cos(state->attitude.roll)*sensorData->gyro.y) -
                    ((float)sin(state->attitude.roll)*sensorData->gyro.z);
    v_system.roll =(sensorData->gyro.x)+
                    ((float)tan(-state->attitude.pitch)*(float)sin(state->attitude.roll)*sensorData->gyro.y)+
                    ((float)tan(-state->attitude.pitch)*(float)cos(state->attitude.roll)*sensorData->gyro.z);
    v_system.wx = (((I_y-I_z)/I_x)*sensorData->gyro.y*sensorData->gyro.z)+(u_values->u_2/I_x);
    v_system.wy = (((I_z-I_x)/I_y)*sensorData->gyro.x*sensorData->gyro.z)+(u_values->u_3/I_y);
    v_system.wz = (((I_x-I_y)/I_z)*sensorData->gyro.x*sensorData->gyro.y)+(u_values->u_4/I_z);

    return v_system;
}

w_control_m calculate_w (u_values_m *u_values)
{
    w_control_m w;
    float w1_sum;
    float w2_sum;
    float w3_sum;
    float w4_sum;
    w1_sum = ( cT*u_values->u_1) +
             (-((sqrtf(2)/2)*d*cT)*u_values->u_2) +
             (-((sqrtf(2)/2)*d*cT)*u_values->u_3) +
             (-cQ*u_values->u_4);

    w2_sum = ( cT*u_values->u_1) +
             (-((sqrtf(2)/2)*d*cT)*u_values->u_2) +
             ( ((sqrtf(2)/2)*d*cT)*u_values->u_3) +
             ( cQ*u_values->u_4);

    w3_sum = ( cT*u_values->u_1) +
             ( ((sqrtf(2)/2)*d*cT)*u_values->u_2) +
             ( ((sqrtf(2)/2)*d*cT)*u_values->u_3) +
             (-cQ*u_values->u_4);

    w4_sum = ( cT*u_values->u_1) +
             ( ((sqrtf(2)/2)*d*cT)*u_values->u_2) +
             (-((sqrtf(2)/2)*d*cT)*u_values->u_3) +
             ( cQ*u_values->u_4);

    w.w1 = 14000*sqrtf(w1_sum);
    w.w2 = 14000*sqrtf(w2_sum);
    w.w3 = 14000*sqrtf(w3_sum);
    w.w4 = 14000*sqrtf(w4_sum);

    if (ZERO > w.w1)
    {
        w.w1 = 0.0f;
    }

    if (ZERO > w.w2)
    {
        w.w2 = 0.0f;
    }

    if (ZERO > w.w3)
    {
        w.w3 = 0.0f;
    }

    if (ZERO > w.w4)
    {
        w.w4 = 0.0f;
    }

    return w;
}

w_motor_m calculate_motor (w_control_m *wc)
{
    w_motor_m wm;
    if (MAX_MOTOR > (uint16_t)wc->w1)
    {
        wm.w1 = (uint16_t)wc->w1;
    }
    else
    {
        wm.w1 = MAX_MOTOR;
    }

    if (MAX_MOTOR > (uint16_t)wc->w2)
    {
        wm.w2 = (uint16_t)wc->w2;
    }
    else
    {
        wm.w2 = MAX_MOTOR;
    }

    if (MAX_MOTOR > (uint16_t)wc->w3)
    {
        wm.w3 = (uint16_t)wc->w3;
    }
    else
    {
        wm.w3 = MAX_MOTOR;
    }

    if (MAX_MOTOR > (uint16_t)wc->w4)
    {
        wm.w4 = (uint16_t)wc->w4;
    }
    else
    {
        wm.w4 = MAX_MOTOR;
    }
    return wm;
}

u_values_m initialize_lqr_u_variables()
{
    u_values_m init;
    init.u_1 = MASS*GRAV;
    init.u_2 = 0.0f;
    init.u_3 = 0.0f;
    init.u_4 = 0.0f;
    return init;
}

v_system_m initialize_lqr_v_variables()
{
    v_system_m v_system;
    v_system.z = 842.0f;
    v_system.vz = 0.0f;
    v_system.pitch = 0.0f;
    v_system.roll = 0.0f;
    v_system.yaw = 0.0f;
    v_system.wx = 0.0f;
    v_system.wy = 0.0f;
    v_system.wz = 0.0f;
    return v_system;
}
