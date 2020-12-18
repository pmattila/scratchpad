/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"


PG_REGISTER_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules, PG_GENERIC_MIXER_RULES, 0);

PG_REGISTER_ARRAY(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs, PG_GENERIC_MIXER_INPUTS, 0);


static FAST_RAM_ZERO_INIT mixerRule_t rules[MIXER_RULE_COUNT];

static FAST_RAM_ZERO_INIT float     mixInput[MIXER_INPUT_COUNT];
static FAST_RAM_ZERO_INIT float     mixOutput[MIXER_OUTPUT_COUNT];
static FAST_RAM_ZERO_INIT int16_t   mixOverride[MIXER_INPUT_COUNT];
static FAST_RAM_ZERO_INIT bool      mixSaturated[MIXER_INPUT_COUNT];

static FAST_RAM_ZERO_INIT uint32_t  flightModeMask;

static FAST_RAM_ZERO_INIT uint8_t   activeServos;
static FAST_RAM_ZERO_INIT uint8_t   activeMotors;

static FAST_RAM_ZERO_INIT float     cyclicTotal;
static FAST_RAM_ZERO_INIT float     cyclicLimit;


static inline float mixerConstrainf(uint8_t index, float val, float min, float max)
{
    if (val > max) {
        mixSaturated[index] = true;
        return max;
    }
    else if (val < min) {
        mixSaturated[index] = true;
        return min;
    }
    return val;
}


void mixerInit(void)
{
    activeServos = 0;
    activeMotors = 0;

    cyclicLimit = PIDSUM_LIMIT * MIXER_PID_SCALING;

    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        const mixerRule_t *rule = mixerRules(i);

        if (rule->oper) {
            rules[i].mode    = rule->mode;
            rules[i].oper    = constrain(rule->oper, 0, MIXER_OP_COUNT - 1);
            rules[i].input   = constrain(rule->input, 0, MIXER_INPUT_COUNT - 1);
            rules[i].output  = constrain(rule->output, 0, MIXER_OUTPUT_COUNT - 1);
            rules[i].offset  = constrain(rule->offset, MIXER_INPUT_MIN, MIXER_INPUT_MAX);
            rules[i].rate    = constrain(rule->rate, MIXER_RATE_MIN, MIXER_RATE_MAX);
            rules[i].min     = constrain(rule->min, MIXER_INPUT_MIN, MIXER_INPUT_MAX);
            rules[i].max     = constrain(rule->max, MIXER_INPUT_MIN, MIXER_INPUT_MAX);

            if (rules[i].output < MAX_SUPPORTED_SERVOS)
                activeServos = MAX(activeServos, rules[i].output + 1);
            else
                activeMotors = MAX(activeMotors, rules[i].output - MIXER_MOTOR_OFFSET + 1);
        }
    }

    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        mixOverride[i] = MIXER_OVERRIDE_OFF;
    }

    mixerInitProfile();
}

void mixerInitProfile(void)
{
    // Nothing
}

static void mixerUpdateInputs(void)
{
    mixInput[MIXER_IN_RC_COMMAND_ROLL]        = rcCommand[ROLL]       * MIXER_RC_SCALING;
    mixInput[MIXER_IN_RC_COMMAND_PITCH]       = rcCommand[PITCH]      * MIXER_RC_SCALING;
    mixInput[MIXER_IN_RC_COMMAND_YAW]         = rcCommand[YAW]        * MIXER_RC_SCALING;
    mixInput[MIXER_IN_RC_COMMAND_COLLECTIVE]  = rcCommand[COLLECTIVE] * MIXER_RC_SCALING;

    mixInput[MIXER_IN_RC_COMMAND_THROTTLE]    = (rcCommand[THROTTLE] - MIXER_THR_OFFSET) * MIXER_THR_SCALING;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++)
        mixInput[MIXER_IN_RC_CHANNEL_1 + i] = (rcData[i] - rxConfig()->midrc) * MIXER_RC_SCALING;

    mixInput[MIXER_IN_STABILIZED_ROLL]        = pidData[FD_ROLL].Sum  * MIXER_PID_SCALING;
    mixInput[MIXER_IN_STABILIZED_PITCH]       = pidData[FD_PITCH].Sum * MIXER_PID_SCALING;
    mixInput[MIXER_IN_STABILIZED_YAW]         = pidData[FD_YAW].Sum   * MIXER_PID_SCALING;

    // TODO
    mixInput[MIXER_IN_STABILIZED_COLLECTIVE]  = mixInput[MIXER_IN_RC_COMMAND_COLLECTIVE];

    mixInput[MIXER_IN_GOVERNOR_MAIN] = getGovernorOutput(GOV_MAIN);
    mixInput[MIXER_IN_GOVERNOR_TAIL] = getGovernorOutput(GOV_TAIL);

    // Cyclic deflection
    cyclicTotal = sqrtf(mixInput[MIXER_IN_STABILIZED_ROLL] * mixInput[MIXER_IN_STABILIZED_ROLL] +
                        mixInput[MIXER_IN_STABILIZED_PITCH] * mixInput[MIXER_IN_STABILIZED_PITCH]);

    // Cyclic ring limit reached
    if (cyclicTotal > cyclicLimit) {
        mixInput[MIXER_IN_STABILIZED_ROLL]  *= cyclicLimit / cyclicTotal;
        mixInput[MIXER_IN_STABILIZED_PITCH] *= cyclicLimit / cyclicTotal;
    }

    // Input override
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
            if (mixOverride[i] >= MIXER_OVERRIDE_MIN && mixOverride[i] <= MIXER_OVERRIDE_MAX)
                mixInput[i] = mixOverride[i] * 1e-3;
        }
    }

    // Input conditioning
    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        const mixerInput_t *input = mixerInputs(i);
        mixSaturated[i] = false;
        mixInput[i] = mixerConstrainf(i, input->offset + input->rate * mixInput[i], input->min, input->max) * 1e-3;
    }
}

void mixerUpdate(void)
{
    // Fetch input values
    mixerUpdateInputs();

    // Reset outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixOutput[i] = 0;
    }

    // Current flight mode bitmap
    flightModeMask = ((uint32_t)(~flightModeFlags)) << 16 | flightModeFlags;

    // Calculate mixer outputs
    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        if (rules[i].oper && ((rules[i].mode == 0) || (rules[i].mode & flightModeMask))) {
            uint8_t src = rules[i].input;
            uint8_t dst = rules[i].output;
            float   out = mixerConstrainf(src, rules[i].offset + rules[i].rate * mixInput[src], rules[i].min, rules[i].max) * 1e-3f;

            switch (rules[i].oper)
            {
                case MIXER_OP_SET:
                    mixOutput[dst] = out;
                    break;
                case MIXER_OP_ADD:
                    mixOutput[dst] += out;
                    break;
                case MIXER_OP_MUL:
                    mixOutput[dst] *= out;
                    break;
            }
        }
    }
}

void mixerSaturateOutput(uint8_t n)
{
    for (int i = MIXER_RULE_COUNT-1; i >= 0; i--) {
        if (rules[i].oper && ((rules[i].mode == 0) || (rules[i].mode & flightModeMask))) {
            if (rules[i].output == n) {
                mixSaturated[ rules[i].input ] = true;
                if (rules[i].oper == MIXER_OP_SET)
                    return;
            }
        }
    }
}

void mixerSaturateInput(uint8_t i)
{
    mixSaturated[i] = true;
}

bool mixerInputSaturated(uint8_t i)
{
    return mixSaturated[i];
}

float mixerGetInput(uint8_t i)
{
    return mixInput[i];
}

float mixerGetServoOutput(uint8_t i)
{
    return mixOutput[MIXER_SERVO_OFFSET + i];
}

float mixerGetMotorOutput(uint8_t i)
{
    return mixOutput[MIXER_MOTOR_OFFSET + i];
}

uint8_t mixerGetActiveServos(void)
{
    return activeServos;
}

uint8_t mixerGetActiveMotors(void)
{
    return activeMotors;
}

int16_t mixerGetOverride(uint8_t i)
{
    return mixOverride[i];
}

int16_t mixerSetOverride(uint8_t i, int16_t value)
{
    return mixOverride[i] = value;
}

float getCyclicDeflection(void)
{
    return MIN(cyclicTotal / cyclicLimit, 1.0f);
}

