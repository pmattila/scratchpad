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

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/governor.h"

#include "rx/rx.h"

#include "io/gimbal.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"


PG_REGISTER_ARRAY(mixer_t, MIXER_RULE_COUNT, mixerRules, PG_HELI_MIXER, 0);


int mixerActiveServos = 0;
int mixerActiveMotors = 0;

static FAST_RAM_ZERO_INIT uint8_t mixerRuleCount;

static FAST_RAM_ZERO_INIT mixer_t mixer[MIXER_RULE_COUNT];

static FAST_RAM_ZERO_INIT float mixerInput[MIXER_INPUT_COUNT];
static FAST_RAM_ZERO_INIT float mixerOutput[MIXER_OUTPUT_COUNT];

static FAST_RAM_ZERO_INIT int16_t mixerOverride[MIXER_INPUT_COUNT];

static FAST_RAM_ZERO_INIT float cyclicTotal = 0;
static FAST_RAM_ZERO_INIT float cyclicLimit = 1;


void mixerInit(void)
{
    mixerRuleCount = 0;

    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        if (mixerRules(i)->oper == MIXER_OP_NUL)
            break;
        
        if (mixerRules(i)->output < MIXER_OUTPUT_MOTORS)
            mixerActiveServos = MAX(mixerActiveServos, i + 1);
        else
            mixerActiveMotors = MAX(mixerActiveMotors, i - MIXER_OUTPUT_MOTORS + 1);
            
        memcpy(&mixer[i], mixerRules(i), sizeof(mixer_t));
        
        mixerRuleCount++;
    }
}

void mixerInitProfile(void)
{
    cyclicLimit = currentPidProfile->pidSumLimit * PID_MIXER_SCALING;
}

void mixerUpdate(void)
{
    mixerInput[INPUT_STABILIZED_ROLL]  = constrainf(pidData[FD_ROLL].Sum,  -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) * PID_MIXER_SCALING;
    mixerInput[INPUT_STABILIZED_PITCH] = constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) * PID_MIXER_SCALING;
    mixerInput[INPUT_STABILIZED_YAW]   = constrainf(pidData[FD_YAW].Sum,   -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) * PID_MIXER_SCALING;

    mixerInput[INPUT_STABILIZED_COLLECTIVE]  = rcCommand[COLLECTIVE];
    
    mixerInput[INPUT_GOVERNOR_MAIN] = govOutput[0];
    mixerInput[INPUT_GOVERNOR_TAIL] = govOutput[1];

    mixerInput[INPUT_GIMBAL_PITCH]  = scaleRangef(attitude.values.pitch, -1800, 1800, -500, 500);
    mixerInput[INPUT_GIMBAL_ROLL]   = scaleRangef(attitude.values.roll,  -1800, 1800, -500, 500);

    mixerInput[INPUT_RC_ROLL]       = rcData[ROLL]       - rxConfig()->midrc;
    mixerInput[INPUT_RC_PITCH]      = rcData[PITCH]      - rxConfig()->midrc;
    mixerInput[INPUT_RC_YAW]        = rcData[YAW]        - rxConfig()->midrc;
    mixerInput[INPUT_RC_THROTTLE]   = rcData[THROTTLE]   - rxConfig()->midrc;
    mixerInput[INPUT_RC_COLLECTIVE] = rcData[COLLECTIVE] - rxConfig()->midrc;

    for (int i = 0; i < 8; i++)
        mixerInput[INPUT_RC_AUX1+i] = rcData[AUX1+i] - rxConfig()->midrc;

    // Current cyclic deflection and limit
    cyclicTotal = sqrt(mixerInput[INPUT_STABILIZED_ROLL] * mixerInput[INPUT_STABILIZED_ROLL] +
                       mixerInput[INPUT_STABILIZED_PITCH] * mixerInput[INPUT_STABILIZED_PITCH]);

    // Cyclic ring limit reached
    if (cyclicTotal > cyclicLimit) {
        mixerInput[INPUT_STABILIZED_ROLL]  *= cyclicLimit / cyclicTotal;
        mixerInput[INPUT_STABILIZED_PITCH] *= cyclicLimit / cyclicTotal;
    }

    // Input override
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
            if (mixerOverride[i] != MIXER_OVERRIDE_OFF)
                mixerInput[i] = mixerOverride[i];
        }
    }

    // Calculate mixer outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixerOutput[i] = 0;
    }
    
    for (int i = 0; i < mixerRuleCount; i++) {
        int src = mixer[i].input;
        int dst = mixer[i].output;
        float val = constrain(mixer[i].offset + mixerInput[src] * mixer[i].rate, mixer[i].min, mixer[i].max) / 1000.0f;
        
        switch (mixer[i].oper)
        {
            case MIXER_OP_SET:
                mixerOutput[dst] = val;
                break;
            case MIXER_OP_ADD:
                mixerOutput[dst] += val;
                break;
            case MIXER_OP_MUL:
                mixerOutput[dst] *= val;
                break;
        }
    }
}


float mixerGetInput(uint8_t i)
{
    return mixerInput[i];
}

float mixerGetServoOutput(uint8_t i)
{
    return mixerOutput[i];
}

float mixerGetMotorOutput(uint8_t i)
{
    return (mixerOutput[i + MIXER_OUTPUT_MOTORS] + 1) / 2;
}

float getCyclicDeflection(void)
{
    return MIN(cyclicTotal / cyclicLimit, 1.0f);
}

