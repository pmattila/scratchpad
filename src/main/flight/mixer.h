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
    INPUT_SOURCE_NONE = 0,
    INPUT_STABILIZED_ROLL,
    INPUT_STABILIZED_PITCH,
    INPUT_STABILIZED_YAW,
    INPUT_STABILIZED_COLLECTIVE,
    INPUT_GOVERNOR_MAIN,
    INPUT_GOVERNOR_TAIL,
    INPUT_RC_ROLL,
    INPUT_RC_PITCH,
    INPUT_RC_YAW,
    INPUT_RC_THROTTLE,
    INPUT_RC_COLLECTIVE,
    INPUT_RC_AUX1,
    INPUT_RC_AUX2,
    INPUT_RC_AUX3,
    INPUT_RC_AUX4,
    INPUT_RC_AUX5,
    INPUT_RC_AUX6,
    INPUT_RC_AUX7,
    INPUT_RC_AUX8,
    INPUT_SOURCE_COUNT
};

enum {
    MIXER_OP_NUL = 0,
    MIXER_OP_SET,
    MIXER_OP_ADD,
    MIXER_OP_MUL,
    MIXER_OP_COUNT
};
   

#define MIXER_RULE_COUNT      32

#define MIXER_INPUT_COUNT     INPUT_SOURCE_COUNT
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


extern int mixerActiveServos;
extern int mixerActiveMotors;


void mixerInit(void);
void mixerInitProfile(void);

void mixerUpdate(void);

float mixerGetInput(uint8_t i);
float mixerGetServoOutput(uint8_t i);
float mixerGetMotorOutput(uint8_t i);

float getCyclicDeflection(void);


#define mixerGetThrottle()      mixerGetInput(INPUT_RC_THROTTLE)

#define mixerGetActiveServos()  mixerActiveServos
#define mixerGetActiveMotors()  mixerActiveMotors

