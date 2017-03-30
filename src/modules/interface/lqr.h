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
#define cQ 0.005964552f
#define cT 0.315f
#define d 0.046f

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

typedef struct w_control
{
    float w1;
    float w2;
    float w3;
    float w4;
}w_control_m;

u_values_m set_u (const u_values_m *init, const sensorData_t *sensorData,
                  const state_t *state, float z_ref, v_system_m *vz_prev);
v_system_m set_dyn_model (v_system_m *v_system, const sensorData_t *sensorData, const state_t *state, const u_values_m *u_values);
u_values_m initialize_lqr_u_variables();
v_system_m initialize_lqr_v_variables();
w_control_m calculate_w (u_values_m *u_values);
float update_z (float *past_z, float current_z);

#endif /* SRC_MODULES_INTERFACE_LQR_H_ */
