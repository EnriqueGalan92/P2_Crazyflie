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

struct u_values
{
    float u_1;
    float u_2;
    float u_3;
    float u_4;
};

struct v_system
{
    float z;
    float vz;
    float yaw;
    float pitch;
    float roll;
    float wx;
    float wy;
    float wz;
};

bool set_u (struct u_values *init, struct u_values *equi, struct v_system sys, float z_ref);
float update_z (float *past_z, float current_z);
#endif /* SRC_MODULES_INTERFACE_LQR_H_ */
