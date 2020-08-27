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


uint8_t getMotorCount(void);

float getMotorOutput(uint8_t motor);

float getMotorOverride(uint8_t motor);
uint16_t getMotorOutputExt(uint8_t motor);

void setMotorOverride(uint8_t motor, float value);
void setMotorOverrideExt(uint8_t motor, uint16_t value);

void resetMotorOverride(void);

bool areMotorsRunning(void);

bool isRpmSourceActive(void);

int getMotorRPM(uint8_t motor);
float getMotorRPMf(uint8_t motor);

int calcMotorRpm(uint8_t motor, int erpm);
float calcMotorRpmf(uint8_t motor, int erpm);

void rpmSourceInit(void);

void motorInit(void);
void motorStop(void);
void motorUpdate(void);

static inline void stopMotors(void) { motorStop(); }

static inline void initEscEndpoints(void) { }

static inline float getMotorMixRange(void) { return 1.0f; }

