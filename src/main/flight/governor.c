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

#include "config/feature.h"
#include "config/config.h"

#include "fc/runtime_config.h"

#include "pg/motor.h"

#include "flight/motors.h"
#include "flight/governor.h"


PG_REGISTER_WITH_RESET_TEMPLATE(governorConfig_t, governorConfig, PG_GOVERNOR_CONFIG, 0);

PG_RESET_TEMPLATE(governorConfig_t, governorConfig,
    .gov_mode = GM_STANDARD,
    .gov_max_headspeed = 2000,
    .gov_spoolup_time = 10,
    .gov_bailout_time = 0,
    .gov_auto_timeout = 0,
    .gov_gear_ratio = 1000,
    .gov_p_gain = 0,
    .gov_i_gain = 0,
    .gov_cyclic_ff_gain = 0,
    .gov_collective_ff_gain = 0,
    .gov_collective_ff_impulse_gain = 0,
    .gov_tailmotor_assist_gain = 0,
    .gov_vbat_filter = 25,
    .gov_vbat_offset = 0,
    .gov_ff_exponent = 150,
    .gov_cs_filter = 1000,
    .gov_cf_filter = 1000,
    .gov_cg_filter = 1000,
    .gov_st_filter = 1000,
);


FAST_RAM_ZERO_INIT uint8_t govMode;
FAST_RAM_ZERO_INIT uint8_t govState;

FAST_RAM_ZERO_INIT float govHeadSpeed;
FAST_RAM_ZERO_INIT float govGearRatio;

FAST_RAM_ZERO_INIT float govOutput[MAX_SUPPORTED_MOTORS];


void governorInit(void)
{
    govMode = governorConfig()->gov_mode;
    
    govGearRatio = (float)governorConfig()->gov_gear_ratio / 1000.0;
    
    switch (govMode) {
        case GM_STANDARD:
            governorInitStandard();
            break;
        case GM_MODEL1:
        case GM_MODEL2:
        case GM_MODEL3:
            governorInitModels();
            break;
    }
}

void governorUpdate(void)
{
    // Other code looks to governor for headspeed, update it on every loop.
    govHeadSpeed = getMotorRPM(0) / govGearRatio;

    // If DISARMED or no motors, reset the governor state and variables, then exit.
    // This method does not preserve looptime, but it is **safe**.
    if (!ARMING_FLAG(ARMED) || getMotorCount() == 0) {
        govState = GS_THROTTLE_OFF;
        govOutput[GOV_MAIN] = 0;
        govOutput[GOV_TAIL] = 0;
        return;
    }

    // Select governor mode
    switch (govMode) {
        case GM_STANDARD:
            governorUpdateStandard();
            break;
        case GM_MODEL1:
            governorUpdateModel1();
            break;
        case GM_MODEL2:
            governorUpdateModel2();
            break;
        case GM_MODEL3:
            governorUpdateModel3();
            break;
    }
}

float getHeadSpeed(void)
{
    return govHeadSpeed;
}

uint8_t getGovernorMode(void)
{
    return govMode;
}

uint8_t getGovernorState(void)
{
    return govState;
}

float getGovernorOutput(uint8_t n)
{
    return govOutput[n];
}

bool isHeliSpooledUp(void)
{
    switch (govState)
    {
        case GS_THROTTLE_OFF:
        case GS_THROTTLE_IDLE:
        case GS_PASSTHROUGH_SPOOLING_UP:
        case GS_GOVERNOR_SPOOLING_UP:
            return false;
        case GS_PASSTHROUGH_ACTIVE:
        case GS_PASSTHROUGH_LOST_THROTTLE:
        case GS_PASSTHROUGH_LOST_HEADSPEED:
        case GS_GOVERNOR_ACTIVE:
        case GS_GOVERNOR_LOST_THROTTLE:
        case GS_GOVERNOR_LOST_HEADSPEED:
        case GS_AUTOROTATION_CLASSIC:
        case GS_AUTOROTATION_ASSIST:
        case GS_AUTOROTATION_BAILOUT:
            return true;
    }

    return false;
}

