/*
 * This file is part of Heliflight 3D.
 *
 * Heliflight 3D is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Heliflight 3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"

#define DEFAULT_SERVO_MIN     1000
#define DEFAULT_SERVO_MAX     2000
#define DEFAULT_SERVO_CENTER  1500
#define DEFAULT_SERVO_TRIM    0
#define DEFAULT_SERVO_RATE    1000
#define DEFAULT_SERVO_SPEED   0
#define DEFAULT_SERVO_UPDATE  50

#define SERVO_OVERRIDE_OFF    0
#define SERVO_OVERRIDE_MIN    PWM_SERVO_PULSE_MIN
#define SERVO_OVERRIDE_MAX    PWM_SERVO_PULSE_MAX

typedef struct servoParam_s {
    int16_t min;    // servo min
    int16_t max;    // servo max
    int16_t mid;    // servo midpoint
    int16_t trim;   // range [-250;+250] ; can be used to set the center trim of the servo
    int16_t rate;   // range [-1000;+1000] ; can be used to adjust a rate 0-2000%% and a direction
    int16_t speed;  // servo speed limit
} servoParam_t;

PG_DECLARE_ARRAY(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams);

typedef struct servoConfig_s {
    servoDevConfig_t dev;
} servoConfig_t;

PG_DECLARE(servoConfig_t, servoConfig);

void servoInit(void);
void servoUpdate(void);

float getServoOutput(uint8_t servo);
float getServoOverride(uint8_t servo);
float setServoOverride(uint8_t servo, float val);

