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
    MIXER_IN_STABILIZED_ROLL,
    MIXER_IN_STABILIZED_PITCH,
    MIXER_IN_STABILIZED_YAW,
    MIXER_IN_STABILIZED_COLLECTIVE,
    MIXER_IN_GOVERNOR_MAIN,
    MIXER_IN_GOVERNOR_TAIL,
    MIXER_IN_RC_ROLL,
    MIXER_IN_RC_PITCH,
    MIXER_IN_RC_YAW,
    MIXER_IN_RC_THROTTLE,
    MIXER_IN_RC_COLLECTIVE,
    MIXER_IN_RC_AUX1,
    MIXER_IN_RC_AUX2,
    MIXER_IN_RC_AUX3,
    MIXER_IN_RC_AUX4,
    MIXER_IN_RC_AUX5,
    MIXER_IN_RC_AUX6,
    MIXER_IN_RC_AUX7,
    MIXER_IN_RC_AUX8,
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

#define MIXER_OVERRIDE_OFF    1001


typedef struct mixer_s
{
    uint8_t oper;              // rule operation
    uint8_t input;             // input channel
    uint8_t output;            // output channel
    int16_t offset;            // output offset -1000..1000%%
    int16_t rate;              // range [-1250;+1250] ; can be used to adjust rate 0-1250%% and direction
    int16_t min;               // lower bound of rule range -1000..1000%%
    int16_t max;               // lower bound of rule range -1000..1000%%
} mixer_t;

PG_DECLARE_ARRAY(mixer_t, MIXER_RULE_COUNT, mixerRules);


extern FAST_RAM_ZERO_INIT int mixerActiveServos;
extern FAST_RAM_ZERO_INIT int mixerActiveMotors;

extern FAST_RAM_ZERO_INIT int16_t mixerOverride[MIXER_INPUT_COUNT];


void mixerInit(void);
void mixerInitProfile(void);

void mixerUpdate(void);

float mixerGetInput(uint8_t i);
float mixerGetServoOutput(uint8_t i);
float mixerGetMotorOutput(uint8_t i);

float getCyclicDeflection(void);


#define mixerGetThrottle()      mixerGetInput(MIXER_IN_RC_THROTTLE)

#define mixerGetActiveServos()  mixerActiveServos
#define mixerGetActiveMotors()  mixerActiveMotors

