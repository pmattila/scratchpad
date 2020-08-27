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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/pwm_output.h"

#include "sensors/gyro.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"
#include "flight/mixer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


static FAST_RAM_ZERO_INIT float servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_RAM_ZERO_INIT float servoOverride[MAX_SUPPORTED_SERVOS];
static FAST_RAM_ZERO_INIT float servoSpeedLimit[MAX_SUPPORTED_SERVOS];


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoPwmRate = DEFAULT_SERVO_UPDATE;

    for (unsigned i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servoConfig->dev.ioTags[i] = timerioTagGetByUsage(TIM_USE_SERVO, i);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
                     .min  = DEFAULT_SERVO_MIN,
                     .max  = DEFAULT_SERVO_MAX,
                     .mid  = DEFAULT_SERVO_CENTER,
                     .rate = DEFAULT_SERVO_RATE,
                     .speed = DEFAULT_SERVO_SPEED,
        );
    }
}


float getServoOutput(uint8_t servo)
{
    return servoOutput[servo];
}

float getServoOverride(uint8_t servo)
{
    return servoOverride[servo];
}

float setServoOverride(uint8_t servo, float val)
{
    return servoOverride[servo] = val;
}

void servoInit(void)
{
    servoDevInit(&servoConfig()->dev);

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        if (servoParams(i)->speed)
            servoSpeedLimit[i] = 1e-3f * (servoParams(i)->max - servoParams(i)->min) * gyro.targetLooptime / servoParams(i)->speed;
        else
            servoSpeedLimit[i] = 0;
    }
}

static inline float limitSlewRate(float old, float new, float rate)
{
    float diff = new - old;

    if (diff > rate)
        return old + rate;
    else if (diff < -rate)
        return old - rate;

    return new;
}

void servoUpdate(void)
{
    float pwm;

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++)
    {
        if (!ARMING_FLAG(ARMED) && servoOverride[i] != SERVO_OVERRIDE_OFF)
            pwm = servoOverride[i];
        else
            pwm = servoParams(i)->mid + (mixerGetServoOutput(i) * servoParams(i)->rate / 2);

        if (servoSpeedLimit[i] > 0)
            pwm = limitSlewRate(servoOutput[i], pwm, servoSpeedLimit[i]);

        servoOutput[i] = constrainf(pwm, servoParams(i)->min, servoParams(i)->max);

        pwmWriteServo(i, servoOutput[i]);
    }
}

#endif
