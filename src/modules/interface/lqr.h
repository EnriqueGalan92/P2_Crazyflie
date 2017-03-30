/*
 * lqr.h
 *
 *  Created on: Mar 29, 2017
 *      Author: bitcraze
 */

#ifndef SRC_MODULES_INTERFACE_LQR_H_
#define SRC_MODULES_INTERFACE_LQR_H_

#include <stdbool.h>
#include <stdint.h>

#define D 0.046f
#define MASS 0.03352f
#define GRAV 9.80665f
#define ERROR 0
#define SUCCESS 1
#define I_x 0.000016571710f
#define I_y 0.000016655602f
#define I_z 0.000029261652f

typedef struct u_values
{
    float u_1;
    float u_2;
    float u_3;
    float u_4;
}u_values_m;

typedef struct v_system
{
    float z;
    float vz;
    float yaw;
    float pitch;
    float roll;
    float wx;
    float wy;
    float wz;
}v_system_m;

bool set_u (const u_values_m *init, u_values_m *equi, v_system_m *sys, float z_ref);
bool set_dyn_model (v_system_m *v_system, const sensorData_t *sensorData, const state_t *state, const u_values_m *u_values);
bool initialize_lqr_variables(u_values_m *init, v_system_m *v_system);
float update_z (float *past_z, float current_z);

#endif /* SRC_MODULES_INTERFACE_LQR_H_ */
