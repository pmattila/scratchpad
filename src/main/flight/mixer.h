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

#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#include "flight/servos.h"
#include "flight/motors.h"
#include "flight/governor.h"
#include "flight/rpm_filter.h"


enum {
    MIXER_IN_NONE = 0,
    MIXER_IN_GOVERNOR_MAIN,
    MIXER_IN_GOVERNOR_TAIL,
    MIXER_IN_STABILIZED_ROLL,
    MIXER_IN_STABILIZED_PITCH,
    MIXER_IN_STABILIZED_YAW,
    MIXER_IN_STABILIZED_COLLECTIVE,
    MIXER_IN_RC_COMMAND_ROLL,
    MIXER_IN_RC_COMMAND_PITCH,
    MIXER_IN_RC_COMMAND_YAW,
    MIXER_IN_RC_COMMAND_COLLECTIVE,
    MIXER_IN_RC_COMMAND_THROTTLE,
    MIXER_IN_RC_CHANNEL_1,
    MIXER_IN_RC_CHANNEL_2,
    MIXER_IN_RC_CHANNEL_3,
    MIXER_IN_RC_CHANNEL_4,
    MIXER_IN_RC_CHANNEL_5,
    MIXER_IN_RC_CHANNEL_6,
    MIXER_IN_RC_CHANNEL_7,
    MIXER_IN_RC_CHANNEL_8,
    MIXER_IN_RC_CHANNEL_9,
    MIXER_IN_RC_CHANNEL_10,
    MIXER_IN_RC_CHANNEL_11,
    MIXER_IN_RC_CHANNEL_12,
    MIXER_IN_RC_CHANNEL_13,
    MIXER_IN_RC_CHANNEL_14,
    MIXER_IN_RC_CHANNEL_15,
    MIXER_IN_RC_CHANNEL_16,
    MIXER_IN_RC_CHANNEL_17,
    MIXER_IN_RC_CHANNEL_18,
    MIXER_IN_COUNT
};

enum {
    MIXER_OP_NUL = 0,
    MIXER_OP_SET,
    MIXER_OP_ADD,
    MIXER_OP_MUL,
    MIXER_OP_COUNT
};


#define MIXER_RULE_COUNT      32

#define MIXER_INPUT_COUNT     MIXER_IN_COUNT
#define MIXER_OUTPUT_COUNT    (MAX_SUPPORTED_SERVOS + MAX_SUPPORTED_MOTORS)
#define MIXER_OUTPUT_MOTORS   MAX_SUPPORTED_SERVOS

#define MIXER_RATE_MIN       -5000
#define MIXER_RATE_MAX        5000

#define MIXER_INPUT_MIN      -2000
#define MIXER_INPUT_MAX       2000

#define MIXER_OVERRIDE_MIN   -2000
#define MIXER_OVERRIDE_MAX    2000
#define MIXER_OVERRIDE_OFF    (MIXER_OVERRIDE_MAX + 1)

#define MIXER_RC_SCALING      (1.0f / 500)
#define MIXER_PID_SCALING     (1.0f / 500)
#define MIXER_THR_SCALING     (1.0f / (PWM_RANGE_MAX - PWM_RANGE_MIN))
#define MIXER_THR_OFFSET      PWM_RANGE_MIN

#define MIXER_CUSTOM          23


typedef struct
{
    int16_t   rate;             // rate multiplier
    int16_t   min;              // minimum value
    int16_t   max;              // maximum value
} mixerInput_t;

PG_DECLARE_ARRAY(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs);

typedef struct
{
    uint32_t  mode;              // active flight mode bitmap
    uint8_t   oper;              // rule operation
    uint8_t   input;             // input channel
    uint8_t   output;            // output channel
    int16_t   offset;            // output offset -2000..2000%%
    int16_t   rate;              // range [-2000;+2000] ; can be used to adjust rate 0-2000%% and direction
    int16_t   min;               // lower bound of rule range -1000..1000%%
    int16_t   max;               // lower bound of rule range -1000..1000%%
} mixerRule_t;

PG_DECLARE_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules);


void mixerInit(void);
void mixerInitProfile(void);

void mixerUpdate(void);

float mixerGetInput(uint8_t i);
float mixerGetServoOutput(uint8_t i);
float mixerGetMotorOutput(uint8_t i);

uint8_t mixerGetActiveServos(void);
uint8_t mixerGetActiveMotors(void);

int16_t mixerGetOverride(uint8_t i);
int16_t mixerSetOverride(uint8_t i, int16_t value);

void mixerSaturateOutput(uint8_t i);
void mixerSaturateInput(uint8_t i);

bool mixerInputSaturated(uint8_t i);

float getCyclicDeflection(void);

static inline float mixerGetThrottle(void) { return mixerGetInput(MIXER_IN_RC_COMMAND_THROTTLE); }

