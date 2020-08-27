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

#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"
#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"

#include "sensors/battery.h"

#include "flight/governor.h"
#include "flight/mixer.h"
#include "flight/pid.h"


PG_REGISTER_WITH_RESET_TEMPLATE(governorConfig_t, governorConfig, PG_GOVERNOR_CONFIG, 0);

PG_RESET_TEMPLATE(governorConfig_t, governorConfig,
    .gov_mode = GM_PASSTHROUGH,
    .gov_max_headspeed = 2000,
    .gov_spoolup_time = 10,
    .gov_recovery_time = 10,
    .gov_autorotation_timeout = 0,
    .gov_autorotation_bailout_time = 0,
    .gov_autorotation_min_entry_time = 10,
    .gov_lost_throttle_timeout = 30,
    .gov_lost_headspeed_timeout = 10,
    .gov_gear_ratio = 1000,
    .gov_p_gain = 0,
    .gov_i_gain = 0,
    .gov_cyclic_ff_gain = 0,
    .gov_collective_ff_gain = 0,
    .gov_collective_ff_impulse_gain = 0,
);


FAST_RAM_ZERO_INIT uint8_t govMode;
FAST_RAM_ZERO_INIT uint8_t govState;

FAST_RAM_ZERO_INIT float govGearRatio;

FAST_RAM_ZERO_INIT float govOutput[MAX_SUPPORTED_MOTORS];

static FAST_RAM_ZERO_INIT timeMs_t govStateEntryTime;

static FAST_RAM_ZERO_INIT float govHeadSpeed;
static FAST_RAM_ZERO_INIT float govMaxHeadspeed;

static FAST_RAM_ZERO_INIT bool  govAutoEnabled;
static FAST_RAM_ZERO_INIT long  govAutoTimeout;
static FAST_RAM_ZERO_INIT long  govAutoMinEntry;

static FAST_RAM_ZERO_INIT long  govLostThrottleTimeout;
static FAST_RAM_ZERO_INIT long  govLostHeadspeedTimeout;

static FAST_RAM_ZERO_INIT float govRampRate;
static FAST_RAM_ZERO_INIT float govBailoutRate;
static FAST_RAM_ZERO_INIT float govRecoveryRate;
static FAST_RAM_ZERO_INIT float govSetpointRate;

static FAST_RAM_ZERO_INIT float govKp;
static FAST_RAM_ZERO_INIT float govKi;
static FAST_RAM_ZERO_INIT float govCycKf;
static FAST_RAM_ZERO_INIT float govColKf;
static FAST_RAM_ZERO_INIT float govColPulseKf;

static FAST_RAM_ZERO_INIT float govBaseThrottle;
static FAST_RAM_ZERO_INIT float govFeedForward;
static FAST_RAM_ZERO_INIT float govPidSum;
static FAST_RAM_ZERO_INIT float govError;
static FAST_RAM_ZERO_INIT float govP;
static FAST_RAM_ZERO_INIT float govI;

static FAST_RAM_ZERO_INIT float govCyclicFF;
static FAST_RAM_ZERO_INIT float govCollectiveFF;
static FAST_RAM_ZERO_INIT float govCollectivePulseFF;

static FAST_RAM_ZERO_INIT float govSetpoint;
static FAST_RAM_ZERO_INIT float govSetpointLimited;


typedef float (*throttle_f)(void);


static inline void govChangeState(uint8_t futureState)
{
    govState = futureState;
    govStateEntryTime = millis();
}

static inline long govStateTime(void)
{
    return cmp32(millis(),govStateEntryTime);
}

static inline bool headSpeedInvalid(void)
{
    return (govHeadSpeed < 50 && govOutput[GOV_MAIN] > 0.20f);
}

static inline float rampUpLimit(float target, float current, float rate)
{
    if (target > current)
        return MIN(current + rate, target);

    return target;
}

static inline float rampLimit(float target, float current, float rate)
{
    if (target > current)
        return MIN(current + rate, target);
    else
        return MAX(current - rate, target);
}

static inline float idleMap(float throttle)
{
    // Map throttle in IDLE
    //   0%..5%   -> 0%
    //   5%..20%  -> 0%..15%
    //   >20%     -> 15%
    return constrainf(throttle - 0.05f, 0, 0.15f);
}


static void govUpdateFeedForward(void)
{
    // Calculate linear feedforward vs. collective absolute position
    govCollectiveFF = govColKf * getCollectiveDeflectionAbs();

    // Collective pitch impulse feed-forward
    govCollectivePulseFF = govColPulseKf * getCollectiveDeflectionAbsHPF();

    // Additional torque is required from the motor when adding cyclic pitch, just like collective
    govCyclicFF = govCycKf * getCyclicDeflection();

    // Total FeedForward
    govFeedForward = govCollectiveFF + govCollectivePulseFF + govCyclicFF;
}


/*
 * Throttle passthrough with spoolup
 */

static void governorUpdatePassthrough(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const float throttle = mixerGetThrottle();
    const bool throttleLow = (throttleStatus == THROTTLE_LOW);

    float govMainPrevious = govOutput[GOV_MAIN];
    float govMain = 0;

    // Calculate collective/cyclic feedforward
    govUpdateFeedForward();

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GS_THROTTLE_OFF);
    }
    else {
        switch (govState)
        {
            // Throttle is OFF
            case GS_THROTTLE_OFF:
                govMain = 0;
                if (!throttleLow)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > 20%, move to SPOOLUP
            case GS_THROTTLE_IDLE:
                govMain = rampUpLimit(throttle, govMainPrevious, govRampRate);
                if (throttleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (throttle > 0.20f)
                    govChangeState(GS_PASSTHROUGH_SPOOLING_UP);
                break;

            // Follow the throttle, with a limited rampup rate.
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- If 0% < throttle < 20%, stay in spoolup
            //  -- Once throttle >20% and not ramping up, move to ACTIVE
            case GS_PASSTHROUGH_SPOOLING_UP:
                govMain = rampUpLimit(throttle, govMainPrevious, govRampRate);
                if (throttleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (throttle < 0.20f)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govMain >= throttle)
                    govChangeState(GS_PASSTHROUGH_ACTIVE);
                break;

            // Follow the throttle without ramp limits.
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If throttle <20%, move to AUTO or SPOOLING_UP
            case GS_PASSTHROUGH_ACTIVE:
                govMain = throttle;
                if (throttleLow)
                    govChangeState(GS_PASSTHROUGH_LOST_THROTTLE);
                else if (govMain < 0.20f) {
                    if (govAutoEnabled && govStateTime() > govAutoMinEntry)
                        govChangeState(GS_AUTOROTATION_CLASSIC);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                }
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to RECOVERY
            //  -- When timer expires, move to OFF
            case GS_PASSTHROUGH_LOST_THROTTLE:
                govMain = 0;
                if (!throttleLow)
                    govChangeState(GS_THROTTLE_RECOVERY);
                else if (govStateTime() > govLostThrottleTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE
            case GS_AUTOROTATION_CLASSIC:
                govMain = rampUpLimit(throttle, govMainPrevious, govBailoutRate);
                if (throttleLow)
                    govChangeState(GS_PASSTHROUGH_LOST_THROTTLE);
                else if (throttle > 0.20f)
                    govChangeState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > govAutoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move back to AUTO
            case GS_AUTOROTATION_BAILOUT:
                govMain = rampUpLimit(throttle, govMainPrevious, govBailoutRate);
                if (throttleLow)
                    govChangeState(GS_PASSTHROUGH_LOST_THROTTLE);
                else if (throttle < 0.20f)
                    govChangeState(GS_AUTOROTATION_CLASSIC);
                else if (govMain >= throttle)
                    govChangeState(GS_PASSTHROUGH_ACTIVE);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move to IDLE
            case GS_THROTTLE_RECOVERY:
                govMain = rampUpLimit(throttle, govMainPrevious, govRecoveryRate);
                if (throttleLow)
                    govChangeState(GS_PASSTHROUGH_LOST_THROTTLE);
                else if (throttle < 0.20f)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govMain >= throttle)
                    govChangeState(GS_PASSTHROUGH_ACTIVE);
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                break;
        }
    }

    // Update output variables
    govOutput[GOV_MAIN] = govMain;
}


/*
 * State machine for governed speed control
 */

static void governorUpdateState(throttle_f govCalc)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const float throttle = mixerGetThrottle();
    const bool throttleLow = (throttleStatus == THROTTLE_LOW);

    float govMainPrevious = govOutput[GOV_MAIN];
    float govMain = 0;

    // Update headspeed target
    govSetpoint = throttle * govMaxHeadspeed;

    // Calculate collective/cyclic feedforward
    govUpdateFeedForward();

    // Handle throttle off separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GS_THROTTLE_OFF);
    }
    else {
        switch (govState)
        {
            // Throttle is OFF
            case GS_THROTTLE_OFF:
                govMain = 0;
                if (!throttleLow)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Throttle is IDLE, follow with a limited ramupup rate.
            //  -- Map throttle to motor output
            //  -- If NO throttle, move to THROTTLE_OFF
            //  -- if throttle > 20%, move to SPOOLUP
            case GS_THROTTLE_IDLE:
                govMain = rampUpLimit(idleMap(throttle), govMainPrevious, govRampRate);
                if (throttleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (throttle > 0.20f)
                    govChangeState(GS_GOVERNOR_SPOOLING_UP);
                break;

            // Ramp up throttle until headspeed target is reached.
            //  -- Once 95% headspeed reached, move to ACTIVE.
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE.
            //  -- If no headspeed detected and throttle >20%, move to LOST_HEADSPEED.
            //  -- If throttle <20%, move back to IDLE.
            //  -- If NO throttle, move to THROTTLE_OFF
            case GS_GOVERNOR_SPOOLING_UP:
                govMain = rampUpLimit(0.99f, govMainPrevious, govRampRate);
                if (throttleLow)
                    govChangeState(GS_THROTTLE_OFF);
                else if (throttle < 0.20f)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (headSpeedInvalid())
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                else if (govHeadSpeed > govSetpoint * 0.95f || govMain > 0.95f) {
                    govChangeState(GS_GOVERNOR_ACTIVE);
                    govSetpointLimited = govHeadSpeed;
                    govBaseThrottle = govMain;
                    govP = govI = 0;
                }
                break;

            // Governor active, maintain headspeed
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If no headspeed signal, move to LOST_HEADSPEED
            //  -- If throttle <20%, move to AUTOROTATION_CLASSIC or IDLE.
            case GS_GOVERNOR_ACTIVE:
                govMain = govMainPrevious;
                if (throttleLow)
                    govChangeState(GS_GOVERNOR_LOST_THROTTLE);
                else if (headSpeedInvalid())
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                else if (throttle < 0.20f) {
                    if (govAutoEnabled && govStateTime() > govAutoMinEntry)
                        govChangeState(GS_AUTOROTATION_CLASSIC);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                } else {
                    govMain = govCalc();
                }
                break;

            // Throttle is low. If it is a mistake, give a chance to recover.
            //  -- If throttle returns, move to AUTO
            //  -- When timer expires, move to OFF
            case GS_GOVERNOR_LOST_THROTTLE:
                govMain = 0;
                if (!throttleLow)
                    govChangeState(GS_THROTTLE_RECOVERY);
                else if (govStateTime() > govLostThrottleTimeout)
                    govChangeState(GS_THROTTLE_OFF);
                break; 

            // No headspeed signal. Ramp down throttle.
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If headspeed recovers, move to RECOVERY
            //  -- When timer expires, move to IDLE
            case GS_GOVERNOR_LOST_HEADSPEED:
                govMain = rampLimit(0.20f, govMainPrevious, govRampRate);
                if (throttleLow)
                    govChangeState(GS_GOVERNOR_LOST_THROTTLE);
                else if (!headSpeedInvalid())
                    govChangeState(GS_THROTTLE_RECOVERY);
                else if (govStateTime() > govLostHeadspeedTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Throttle passthrough with ramup limit
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If throttle >20%, move to AUTOROTATION_BAILOUT
            //  -- If timer expires, move to IDLE.
            //  -- Map throttle to motor output:
            //           0%..5%   -> motor 0%
            //           5%..20%  -> motor 0%..15%
            case GS_AUTOROTATION_CLASSIC:
                govMain = rampUpLimit(idleMap(throttle), govMainPrevious, govBailoutRate);
                if (throttleLow)
                    govChangeState(GS_THROTTLE_OFF);
                if (throttle > 0.20f)
                    govChangeState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > govAutoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- If NO throttle, move to LOST_THROTTLE
            //  -- If no headspeed detected, move to LOST_HEADSPEED
            //  -- If throttle <20%, move back to AUTOROTATION_CLASSIC
            //  -- If throttle reaches 95%, move to ACTIVE
            //  -- Once 90% headspeed reached, move to ACTIVE
            case GS_AUTOROTATION_BAILOUT:
                govMain = rampUpLimit(0.99f, govMainPrevious, govBailoutRate);
                if (throttleLow)
                    govChangeState(GS_GOVERNOR_LOST_THROTTLE);
                else if (headSpeedInvalid())
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                else if (throttle < 0.20f)
                    govChangeState(GS_AUTOROTATION_CLASSIC);
                else if (govHeadSpeed > govSetpoint * 0.90f || govMain > 0.95f) {
                    govChangeState(GS_GOVERNOR_ACTIVE);
                    govSetpointLimited = govHeadSpeed;
                    govBaseThrottle = govMain;
                    govP = govI = 0;
                }
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE
            //  -- If throttle <20%, move to IDLE
            case GS_THROTTLE_RECOVERY:
                govMain = rampUpLimit(throttle, govMainPrevious, govRecoveryRate);
                if (throttleLow)
                    govChangeState(GS_GOVERNOR_LOST_THROTTLE);
                else if (throttle < 0.20f)
                    govChangeState(GS_THROTTLE_IDLE);
                else if (govMain >= throttle)
                    govChangeState(GS_GOVERNOR_ACTIVE);
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
        }
    }

    // Update output variables
    govOutput[GOV_MAIN] = govMain;
}



/*
 * Standard Proportional-Integral-Feedforward (PIF) controller
 */

static float govPIFControl(void)
{
    float output = 0;

    // Adjust the rate limited governor setpoint
    govSetpointLimited = rampLimit(govSetpoint, govSetpointLimited, govSetpointRate);

    // Move any base throttle to I-term
    if (govBaseThrottle > 0) {
        govI += govBaseThrottle;
        govBaseThrottle = 0;
    }

    // Calculate error ratio
    govError = (govSetpointLimited - govHeadSpeed) / govMaxHeadspeed;

    // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
    govP = govKp * govError;

    // PID limits
    govP = constrainf(govP, -0.20f, 0.20f);
    govI = constrainf(govI, -1.00f, 1.00f);

    // if gov_i_gain = 10 (govKi = 1), we will get 1% change in throttle for 1% error in headspeed after 1 second
    float deltaI = govKi * govError * pidGetDT();

    // Governor PI sum
    govPidSum = govP + govI + deltaI;

    // Generate new governed throttle signal
    output = govPidSum + govFeedForward;

    // Apply deltaI if output not saturated
    if (!((output > 1 && deltaI > 0) || (output < 0 && deltaI < 0)))
        govI += deltaI;

    // Limit output to 0%..100%
    output = constrainf(output, 0, 1);

    return output;
}



void governorUpdate(void)
{
    switch (govMode) {
        case GM_PASSTHROUGH:
            governorUpdatePassthrough();
            break;
        case GM_PIF:
            governorUpdateState(govPIFControl);
            break;
    }
}


void governorInit(void)
{
    // Must have at least one motor
    if (getMotorCount() > 0) {

        govMode = governorConfig()->gov_mode;
        govGearRatio = (float)governorConfig()->gov_gear_ratio / 1000.0;

        govState = GS_THROTTLE_OFF;

        govKp           = (float)governorConfig()->gov_p_gain / 10.0;
        govKi           = (float)governorConfig()->gov_i_gain / 10.0;
        govCycKf        = (float)governorConfig()->gov_cyclic_ff_gain / 100.0;
        govColKf        = (float)governorConfig()->gov_collective_ff_gain / 100.0;
        govColPulseKf   = (float)governorConfig()->gov_collective_ff_impulse_gain / 100.0;

        govMaxHeadspeed = constrainf(governorConfig()->gov_max_headspeed, 100, 10000);

        govRampRate     = pidGetDT() / constrainf(governorConfig()->gov_spoolup_time, 1, 60);
        govSetpointRate = govRampRate * govMaxHeadspeed;
        govRecoveryRate = pidGetDT() / constrainf(governorConfig()->gov_recovery_time, 1, 50) * 10;

        govAutoEnabled  = (governorConfig()->gov_autorotation_timeout > 0 &&
                           governorConfig()->gov_autorotation_bailout_time > 0);
        govBailoutRate  = pidGetDT() / constrainf(governorConfig()->gov_autorotation_bailout_time, 1, 100) * 10;
        govAutoTimeout  = governorConfig()->gov_autorotation_timeout * 100;
        govAutoMinEntry = governorConfig()->gov_autorotation_min_entry_time * 1000;

        govLostThrottleTimeout  = governorConfig()->gov_lost_throttle_timeout * 100;
        govLostHeadspeedTimeout = governorConfig()->gov_lost_headspeed_timeout * 100;
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

float getGovernorOutput(uint8_t motor)
{
    return govOutput[motor];
}

bool isHeliSpooledUp(void)
{
    switch (govState)
    {
        case GS_THROTTLE_OFF:
        case GS_THROTTLE_IDLE:
        case GS_THROTTLE_RECOVERY:
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

