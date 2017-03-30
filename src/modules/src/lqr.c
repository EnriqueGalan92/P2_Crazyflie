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

bool set_u (const u_values_m *init, u_values_m *equi, v_system_m *sys, float z_ref)
{
    equi->u_1 = -sys->z -(3.3434f*sys->vz)+z_ref+init->u_1;
    equi->u_2 = -5.7296f*sys->roll - 28.6479f*sys->wx;
    equi->u_3 = -5.7296f*sys->pitch - 28.6479f*sys->wy;
    equi->u_4 = -5.7296f*sys->yaw - 28.6479f*sys->wz;

    return SUCCESS;
}

float update_z (float *past_z, float current_z)
{
    float vz = 0.0;
    vz = (current_z - *past_z)/RATE_MAIN_LOOP;
    *past_z = current_z;
    return vz;
}

bool set_dyn_model (v_system_m *v_system, const sensorData_t *sensorData, const state_t *state, const u_values_m *u_values)
{
    float past_vz = v_system->z;
    v_system->z=update_z(&past_vz, sensorData->baro.pressure);
    v_system->vz=-GRAV+(1/MASS)*((float)cos(state->attitude.pitch)*(float)cos(state->attitude.roll))*u_values->u_1;
    v_system->yaw=(((float)sin(state->attitude.roll)/(float)cos(state->attitude.pitch))*sensorData->gyro.y)+
                  (((float)cos(state->attitude.roll)/(float)cos(state->attitude.pitch))*sensorData->gyro.z);
    v_system->pitch=((float)cos(state->attitude.roll)*sensorData->gyro.y) -
                    ((float)sin(state->attitude.roll)*sensorData->gyro.z);
    v_system->roll =(sensorData->gyro.x)+
                    ((float)tan(state->attitude.pitch)*(float)sin(state->attitude.roll)*sensorData->gyro.y)+
                    ((float)tan(state->attitude.pitch)*(float)cos(state->attitude.roll)*sensorData->gyro.z);
    v_system->wx = (((I_y-I_z)/I_x)*sensorData->gyro.y*sensorData->gyro.z)+(u_values->u_2/I_x);
    v_system->wy = (((I_z-I_x)/I_y)*sensorData->gyro.x*sensorData->gyro.z)+(u_values->u_3/I_y);
    v_system->wz = (((I_x-I_y)/I_z)*sensorData->gyro.x*sensorData->gyro.y)+(u_values->u_4/I_z);

    return SUCCESS;
}

bool initialize_lqr_variables(u_values_m *init, v_system_m *v_system)
{
    init->u_1 = MASS*GRAV;
    init->u_2 = 0.0f;
    init->u_3 = 0.0f;
    init->u_4 = 0.0f;

    v_system->z = 0.0f;
    v_system->vz = 0.0f;
    v_system->pitch = 0.0f;
    v_system->roll = 0.0f;
    v_system->yaw = 0.0f;
    v_system->wx = 0.0f;
    v_system->wy = 0.0f;
    v_system->wz = 0.0f;

    return SUCCESS;
}





