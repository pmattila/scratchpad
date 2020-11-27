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

#include "platform.h"

#include "pg/pg.h"

enum {
    GOV_MAIN = 0,
    GOV_TAIL = 1,
};

typedef enum {
    GM_PASSTHROUGH,
    GM_STANDARD,
    GM_EPIF,
    GM_MEPI,
    GM_AEPI,
} govMode_e;

typedef enum {
    GS_THROTTLE_OFF,
    GS_THROTTLE_IDLE,
    GS_SPOOLING_UP,
    GS_RECOVERY,
    GS_ACTIVE,
    GS_LOST_THROTTLE,
    GS_LOST_HEADSPEED,
    GS_AUTOROTATION,
    GS_AUTOROTATION_BAILOUT,
} govState_e;

typedef struct governorConfig_s {
    uint8_t  gov_mode;
    uint16_t gov_max_headspeed;
    uint16_t gov_spoolup_time;
    uint16_t gov_recovery_time;
    uint16_t gov_autorotation_timeout;
    uint16_t gov_autorotation_bailout_time;
    uint16_t gov_autorotation_min_entry_time;
    uint16_t gov_lost_throttle_timeout;
    uint16_t gov_lost_headspeed_timeout;
    uint16_t gov_gear_ratio;
    uint16_t gov_p_gain;
    uint16_t gov_i_gain;
    uint16_t gov_cyclic_ff_gain;
    uint16_t gov_collective_ff_gain;
    uint16_t gov_collective_ff_impulse_gain;
    uint16_t gov_vbat_offset;
    uint16_t gov_ff_exponent;
    uint16_t gov_in_filter;
    uint16_t gov_cs_filter;
    uint16_t gov_cf_filter;
    uint16_t gov_cg_filter;
    uint16_t gov_st_filter;
    uint16_t gov_pt_filter;
    uint16_t gov_calibration[3];
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);


extern FAST_RAM_ZERO_INIT uint8_t govMode;
extern FAST_RAM_ZERO_INIT uint8_t govState;

extern FAST_RAM_ZERO_INIT float govOutput[MAX_SUPPORTED_MOTORS];


void governorInit();
void governorUpdate();

bool isHeliSpooledUp(void);

float getHeadSpeed(void);
float getGovernorOutput(uint8_t motor);

uint8_t getGovernorMode();
uint8_t getGovernorState();

