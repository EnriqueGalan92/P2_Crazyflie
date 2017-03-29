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

bool set_u (struct u_values *init, struct u_values *equi, struct v_system sys, float z_ref)
{
    init->u_1 = MASS*GRAV;
    init->u_2 = 0.0f;
    init->u_3 = 0.0f;
    init->u_4 = 0.0f;

    equi->u_1 = -sys.z -(3.3434f*sys.vz)+z_ref+init->u_1;
    equi->u_2 = -5.7296f*sys.roll - 28.6479f*sys.wx;
    equi->u_3 = -5.7296f*sys.pitch - 28.6479f*sys.wy;
    equi->u_4 = -5.7296f*sys.yaw - 28.6479f*sys.wz;

    return SUCCESS;
}

float update_z (float *past_z, float current_z)
{
    float vz = 0.0;
    vz = (current_z - *past_z)/RATE_MAIN_LOOP;
    *past_z = current_z;
    return vz;
}





