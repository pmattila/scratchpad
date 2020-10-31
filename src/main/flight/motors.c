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

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"
#include "config/config.h"

#include "pg/motor.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/freq.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"

#include "io/motors.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/core.h"
#include "fc/rc.h"

#include "flight/motors.h"
#include "flight/servos.h"
#include "flight/mixer.h"
#include "flight/pid.h"


typedef enum {
    RPM_SRC_NONE = 0,
    RPM_SRC_DSHOT_TELEM,
    RPM_SRC_FREQ_SENSOR,
    RPM_SRC_ESC_SENSOR,
} rpmSource_e;


static FAST_RAM_ZERO_INIT uint8_t        motorCount;

static FAST_RAM_ZERO_INIT float          motorOutput[MAX_SUPPORTED_MOTORS];
static FAST_RAM_ZERO_INIT float          motorOverride[MAX_SUPPORTED_MOTORS];

static FAST_RAM_ZERO_INIT float          motorRpm[MAX_SUPPORTED_MOTORS];
static FAST_RAM_ZERO_INIT uint8_t        motorRpmDiv[MAX_SUPPORTED_MOTORS];
static FAST_RAM_ZERO_INIT uint8_t        motorRpmSource[MAX_SUPPORTED_MOTORS];
static FAST_RAM_ZERO_INIT biquadFilter_t motorRpmFilter[MAX_SUPPORTED_MOTORS];


uint8_t getMotorCount(void)
{
    return motorCount;
}

float getMotorOutput(uint8_t motor)
{
    return motorOutput[motor];
}

uint16_t getMotorOutputExt(uint8_t motor)
{
    return lrintf(motorOutput[motor] * 1000);
}

float getMotorOverride(uint8_t motor)
{
    return motorOverride[motor];
}

void setMotorOverride(uint8_t motor, float value)
{
    motorOverride[motor] = constrainf(value, 0, 1);
}

void setMotorOverrideExt(uint8_t motor, uint16_t value)
{
    motorOverride[motor] = constrain(value, 0, 1000) / 1000.0f;
}

void resetMotorOverride(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motorOverride[i] = 0;
}

bool areMotorsRunning(void)
{
    if (ARMING_FLAG(ARMED))
        return true;

    for (int i = 0; i < motorCount; i++)
        if (motorOutput[i] > 0)
            return true;

    return false;
}

void motorStop(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motorOutput[i] = 0;

    motorWriteAll(motorOutput);
    delay(50);
}

bool isRpmSourceActive(void)
{
    for (int i = 0; i < getMotorCount(); i++)
        if (motorRpmSource[i] == RPM_SRC_NONE)
            return false;
    return true;
}

int calcMotorRpm(uint8_t motor, int erpm)
{
    return 100 * erpm / motorRpmDiv[motor];
}

float calcMotorRpmf(uint8_t motor, int erpm)
{
    return 100.0f * erpm / motorRpmDiv[motor];
}

int getMotorRPM(uint8_t motor)
{
    return lrintf(motorRpm[motor]);
}

float getMotorRPMf(uint8_t motor)
{
    return motorRpm[motor];
}

int getMotorERPM(uint8_t motor)
{
    int erpm;
#ifdef USE_FREQ_SENSOR
    if (motorRpmSource[motor] == RPM_SRC_FREQ_SENSOR) {
        erpm = getFreqSensorRPM(motor);
    } else
#endif
#ifdef USE_DSHOT_TELEMETRY
    if (motorRpmSource[motor] == RPM_SRC_DSHOT_TELEM) {
        erpm = getDshotTelemetry(motor);
    } else
#endif
#ifdef USE_ESC_SENSOR
    if (motorRpmSource[motor] == RPM_SRC_ESC_SENSOR) {
        erpm = getEscSensorRPM(motor);
    } else
#endif
    {
        erpm = 0;
    }
    return erpm;
}

void rpmSourceInit(void)
{
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
#ifdef USE_FREQ_SENSOR
        if (featureIsEnabled(FEATURE_FREQ_SENSOR) && isFreqSensorPortInitialized(i)) {
            motorRpmSource[i] = RPM_SRC_FREQ_SENSOR;
        } else
#endif
#ifdef USE_DSHOT_TELEMETRY
        if (isMotorProtocolDshot() && motorConfig()->dev.useDshotTelemetry) {
            motorRpmSource[i] = RPM_SRC_DSHOT_TELEM;
        } else
#endif
#ifdef USE_ESC_SENSOR
        if (featureIsEnabled(FEATURE_ESC_SENSOR) && isEscSensorActive()) {
            motorRpmSource[i] = RPM_SRC_ESC_SENSOR;
        } else
#endif
        {
            motorRpmSource[i] = RPM_SRC_NONE;
        }

        motorRpmDiv[i] = constrain(motorConfig()->motorPoleCount[i] / 2, 1, 100);
        
        int freq = constrain(motorConfig()->motorRpmLpf[i], 1, 1000);
        biquadFilterInitLPF(&motorRpmFilter[i], freq, gyro.targetLooptime);
    }
}


void motorInit(void)
{
    motorCount = constrain(mixerGetActiveMotors(), 1, MAX_SUPPORTED_MOTORS);

    motorDevInit(&motorConfig()->dev, motorCount);
}

void motorUpdate(void)
{
    for (int i = 0; i < motorCount; i++) {
        if (ARMING_FLAG(ARMED))
            motorOutput[i] = constrainf(mixerGetMotorOutput(i), 0, 1);
        else
            motorOutput[i] = motorOverride[i];
    }
    
    motorWriteAll(motorOutput);

    for (int i = 0; i < motorCount; i++) {
        motorRpm[i] = biquadFilterApply(&motorRpmFilter[i], calcMotorRpmf(i,getMotorERPM(i)));
        DEBUG_SET(DEBUG_RPM_SOURCE, i, motorRpm[i]);
    }
}

