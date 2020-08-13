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


#define MAX_GOVERNOR_MOTORS 2

typedef struct governorConfig_s {
    uint16_t gov_max_headspeed;
    uint16_t gov_spoolup_time;
    uint16_t gov_gear_ratio;
    uint16_t gov_p_gain;
    uint16_t gov_i_gain;
    uint16_t gov_cyclic_ff_gain;
    uint16_t gov_collective_ff_gain;
    uint16_t gov_collective_ff_impulse_gain;
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);

extern float govOutput[];

void governorInit();
void governorUpdate();

float governorGetGearRatio(void);
float governorGetCollectivePulseFilterGain(void);

bool isHeliSpooledUp(void);

float getHeadSpeed(void);

