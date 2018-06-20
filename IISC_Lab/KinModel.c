/*
 * KinModel.c
 *
 *  Created on: 20-Jun-2018
 *      Author: kamal
 */

/*
 * V[0] ---->   Vx
 * V[1] ---->   Vy
 * V[2] ---->   W
 */
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>
#include "motors.h"
#include "KinModel.h"

volatile float V[3] = { 0, 0, 0 };
volatile float J[4][3] = { { 0, 10, 0.7274 }, { 10, 0, 0.7274 },
                        { 0, -10, 0.7274 }, { -10, 0, 0.7274 } };
volatile float w[4] = { 0, 0, 0, 0 };

void set_velocity(void)
{
    int i = 0, j = 0;
    float sum = 0;
    for (i = 0; i < 4; i++)
    {
        sum = 0;
        for (j = 0; j < 3; j++)
            sum += (J[i][j] * V[j]);
        w[i] = sum;
        w[i] = w[i] * 60 / (2 * M_PI); //rps to RpM
    }
    motors(w);
}
