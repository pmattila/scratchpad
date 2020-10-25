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

static FAST_RAM_ZERO_INIT float ffExponent;
static FAST_RAM_ZERO_INIT float vbOffset;

static FAST_RAM_ZERO_INIT float inFilter;
static FAST_RAM_ZERO_INIT float stFilter;
static FAST_RAM_ZERO_INIT float csFilter;
static FAST_RAM_ZERO_INIT float cfFilter;
static FAST_RAM_ZERO_INIT float cgFilter;
static FAST_RAM_ZERO_INIT float ptFilter;

static FAST_RAM_ZERO_INIT timeMs_t govStateEntryTime;


//// Statistics

static FAST_RAM_ZERO_INIT uint32_t stCount;
static FAST_RAM_ZERO_INIT uint32_t stActiveCount;
static FAST_RAM_ZERO_INIT uint32_t stSpoolupCount;

static FAST_RAM_ZERO_INIT float vbValue;
static FAST_RAM_ZERO_INIT float vbMean;

static FAST_RAM_ZERO_INIT float ibValue;
static FAST_RAM_ZERO_INIT float ibMean;

static FAST_RAM_ZERO_INIT float hsValue;
static FAST_RAM_ZERO_INIT float hsDiff;
static FAST_RAM_ZERO_INIT float hsMean;
static FAST_RAM_ZERO_INIT float hsVar;

static FAST_RAM_ZERO_INIT float vtValue;
static FAST_RAM_ZERO_INIT float vtDiff;
static FAST_RAM_ZERO_INIT float vtMean;
static FAST_RAM_ZERO_INIT float vtVar;

static FAST_RAM_ZERO_INIT float cxValue;
static FAST_RAM_ZERO_INIT float cxDiff;
static FAST_RAM_ZERO_INIT float cxMean;
static FAST_RAM_ZERO_INIT float cxVar;

static FAST_RAM_ZERO_INIT float ffValue;
static FAST_RAM_ZERO_INIT float ffDiff;
static FAST_RAM_ZERO_INIT float ffMean;
static FAST_RAM_ZERO_INIT float ffVar;

static FAST_RAM_ZERO_INIT float hsvtCovar;
static FAST_RAM_ZERO_INIT float cxffCovar;

static FAST_RAM_ZERO_INIT float ccValue;
static FAST_RAM_ZERO_INIT float csValue;
static FAST_RAM_ZERO_INIT float cfValue;

static FAST_RAM_ZERO_INIT float csEstimate;
static FAST_RAM_ZERO_INIT float cfEstimate;


typedef float (*throttle_f)(void);


void governorInitModels(void)
{
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

    ffExponent = (float)governorConfig()->gov_ff_exponent / 100.0f;
    vbOffset   = (float)governorConfig()->gov_vbat_offset / 100.0f;

    inFilter = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_in_filter);
    stFilter = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_st_filter);
    csFilter = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_cs_filter);
    cfFilter = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_cf_filter);
    cgFilter = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_cg_filter);
    ptFilter = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_pt_filter);
}


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


static void govSaveCalibration(void)
{
    governorConfigMutable()->gov_calibration[0] = 1e6f * csEstimate;
    governorConfigMutable()->gov_calibration[1] = 1e6f * cfEstimate;
    governorConfigMutable()->gov_calibration[2] = 1e3f * cfEstimate / csEstimate;

    saveConfigAndNotify();
}

static void govResetStats(void)
{
    vtValue   = cxValue   = 0;
    ccValue   = csValue   = cfValue  = 0;
    vtMean    = cxMean    = hsMean   = ffMean = 0;
    vtDiff    = cxDiff    = hsDiff   = ffDiff = 0;
    vtVar     = cxVar     = hsVar    = ffVar  = 0;
    cxffCovar = hsvtCovar = 0;
    stCount = stActiveCount = stSpoolupCount = 0;
}

static void govDebugStats(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const float throttle = mixerGetThrottle();
    const float govMain = govOutput[GOV_MAIN];

#ifdef USE_DEBUG32
    DEBUG32U( 0, govMode);
    DEBUG32U( 1, govState);
    DEBUG32U( 2, throttleStatus);
    DEBUG32F( 3, throttle * 1e6f);
    DEBUG32F( 4, govSetpoint * 1e3f);
    DEBUG32F( 5, govSetpointLimited * 1e3f);
    DEBUG32F( 6, govHeadSpeed * 1e3f);
    DEBUG32F( 7, govMain * 1e6f);
    DEBUG32F( 8, govBaseThrottle * 1e6f);
    DEBUG32F( 9, govFeedForward * 1e6f);
    DEBUG32F(10, govCollectiveFF * 1e6f);
    DEBUG32F(11, govCollectivePulseFF * 1e6f);
    DEBUG32F(12, govCyclicFF * 1e6f);
    DEBUG32F(13, govPidSum * 1e6f);
    DEBUG32F(14, govError * 1e6f);
    DEBUG32F(15, govP * 1e6f);
    DEBUG32F(16, govI * 1e6f);

    DEBUG32F(17, vbValue * 1e3f);
    DEBUG32F(18, vbMean * 1e3f);
    DEBUG32F(19, ibValue * 1e3f);
    DEBUG32F(20, ibMean * 1e3f);

    DEBUG32F(21, hsValue * 1e3f);
    DEBUG32F(22, hsDiff * 1e3f);
    DEBUG32F(23, hsMean * 1e3f);
    DEBUG32F(24, hsVar * 1e0f);

    DEBUG32F(25, vtValue * 1e3f);
    DEBUG32F(26, vtDiff * 1e3f);
    DEBUG32F(27, vtMean * 1e3f);
    DEBUG32F(28, vtVar * 1e0f);

    DEBUG32F(29, cxValue * 1e6f);
    DEBUG32F(30, cxDiff * 1e6f);
    DEBUG32F(31, cxMean * 1e6f);
    DEBUG32F(32, cxVar * 1e12);

    DEBUG32F(33, ffValue * 1e6f);
    DEBUG32F(34, ffDiff * 1e6f);
    DEBUG32F(35, ffMean * 1e6f);
    DEBUG32F(36, ffVar * 1e6f);

    DEBUG32F(37, hsvtCovar * 1e9f);
    DEBUG32F(38, cxffCovar * 1e9f);

    DEBUG32F(39, ccValue * 1e3f);
    DEBUG32F(40, csValue * 1e6f);
    DEBUG32F(41, cfValue * 1e6f);

    DEBUG32F(42, csEstimate * 1e6f);
    DEBUG32F(43, cfEstimate * 1e6f);
#endif

    DEBUG_SET(DEBUG_GOVERNOR,  0, govSetpointLimited);
    DEBUG_SET(DEBUG_GOVERNOR,  1, govHeadSpeed);
    DEBUG_SET(DEBUG_GOVERNOR,  2, govPidSum * 1000.0f);
    DEBUG_SET(DEBUG_GOVERNOR,  3, govMain * 1000.0f);
}


static void govUpdateStats(void)
{
    float govMain = govOutput[GOV_MAIN];

    // Increment statistics count
    stCount++;

    // ESC voltage
    vbValue = getBatteryVoltageLatest() * 0.01f;
    vbMean += (vbValue - vbMean) * inFilter;

    // ESC Current
    ibValue = getAmperageLatest() * 0.01f;
    ibMean += (ibValue - ibMean) * inFilter;

    // Headspeed value
    hsValue += (govHeadSpeed - hsValue) * inFilter;

    // Analysis
    if (govMain > 0.10f && hsValue > 100)
    {
        vtValue = (vbMean - vbOffset) * govMain;
        cxValue = vtValue / hsValue;

        hsDiff = hsValue - hsMean;
        vtDiff = vtValue - vtMean;
        cxDiff = cxValue - cxMean;
        ffDiff = ffValue - ffMean;

        hsMean += hsDiff * stFilter;
        vtMean += vtDiff * stFilter;
        cxMean += cxDiff * stFilter;
        ffMean += ffDiff * stFilter;

        float tsFilter = 1.0f - stFilter;

        hsVar = (hsDiff * hsDiff * stFilter + hsVar) * tsFilter;
        vtVar = (vtDiff * vtDiff * stFilter + vtVar) * tsFilter;
        cxVar = (cxDiff * cxDiff * stFilter + cxVar) * tsFilter;
        ffVar = (ffDiff * ffDiff * stFilter + ffVar) * tsFilter;

        hsvtCovar = (hsDiff * vtDiff * stFilter + hsvtCovar) * tsFilter;
        cxffCovar = (cxDiff * ffDiff * stFilter + cxffCovar) * tsFilter;
    }
}

static void govUpdateSpoolupStats(float govMain)
{
    if (govMain > 0.10f && govHeadSpeed > 100) {
        // Increment statistics count
        stSpoolupCount++;

        // For analysis -- not used in the model
        if (hsVar > 100) {
            csValue = hsvtCovar / hsVar;
            ccValue = vtMean - hsMean * csValue;
        }

        // Initial value for Cs
        csEstimate = cxMean;

        // Initial value for Cf
        if (governorConfig()->gov_calibration[2] > 0)
            cfEstimate = csEstimate * governorConfig()->gov_calibration[2] / 1000.0f;
        else
            cfEstimate = csEstimate;
    }
}

static void govUpdateActiveStats(float govMain)
{
    bool mainCheck = true, pidCheck = true, hsCheck = true;

    // Increment statistics count
    stActiveCount++;

    // throttle must be between 25%..95%
    mainCheck = (govMain > 0.25f && govMain < 0.95f);

    // In MODEL3, I-term must be -2.5%..2.5%
    //if (govMode == GM_MODEL3)
    //    pidCheck = (govI > -0.025f && govI < 0.025f);

    // Headspeed must be reasonable
    if (govMode == GM_MODEL1)
        hsCheck = (govHeadSpeed > 100);
    else
        hsCheck = (govHeadSpeed > govMaxHeadspeed * 0.1f && govHeadSpeed < govMaxHeadspeed * 1.1f);

    // System must stay in linear & controllable area for the stats to be correct
    if (mainCheck && pidCheck && hsCheck)
    {
        if (ffVar > 0.01f)
            cfValue = cxffCovar / ffVar;
        else
            cfValue = cfEstimate;

        cfValue = constrainf(cfValue, 0, 2*csEstimate);

        if (cfValue > cfEstimate)
            cfEstimate += (cfValue - cfEstimate) * ffVar * ffMean * cfFilter;
        else
            cfEstimate += (cfValue - cfEstimate) * ffVar * ffMean * cgFilter;

        csValue = cxMean - ffMean * cfValue;

        csEstimate += (csValue - csEstimate) * csFilter;
    }
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

    // Aerodynamic drag vs angle of attack approx
    ffValue = powf( 10.0f * govFeedForward, ffExponent );
}


/*
 * This is generally a throttle passthrough, but with extra stats.
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

    // Update statistics
    govUpdateStats();

    // Handle DISARM separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GS_THROTTLE_OFF);
        govResetStats();
    }
    else {
        switch (govState) {
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
                else
                    govUpdateSpoolupStats(govMain);
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
                } else
                    govUpdateActiveStats(govMain);
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
                govResetStats();
                break;
        }
    }

    // Update output variables
    govOutput[GOV_MAIN] = govMain;

    // Set debug
    govDebugStats();
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

    // Update statistics
    govUpdateStats();

    // Handle throttle off separately for SAFETY!
    if (!ARMING_FLAG(ARMED)) {
        govChangeState(GS_THROTTLE_OFF);
        govResetStats();
    }
    else {
        switch (govState) {
            // Throttle is OFF
            case GS_THROTTLE_OFF:
                govMain = 0;
                if (!throttleLow)
                    govChangeState(GS_THROTTLE_IDLE);
                if (stActiveCount > pidGetPidFrequency() * 30) {
                    govSaveCalibration();
                    govResetStats();
                }
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
                } else
                    govUpdateSpoolupStats(govMain);
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
                    govUpdateActiveStats(govMain);
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
                govResetStats();
        }
    }

    // Update output variables
    govOutput[GOV_MAIN] = govMain;

    // Set debug
    govDebugStats();
}



/*
 * Standard PIF controller
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
    govError = (govSetpointLimited - hsValue) / govMaxHeadspeed;

    // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
    govP += (govKp * govError - govP) * ptFilter;

    // PID limits
    govP = constrainf(govP, -0.20f, 0.20f);
    govI = constrainf(govI, -1, 1);

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


/*
 * Extended PIF controller
 */

static float govEPIFControl(void)
{
    float output = 0;

    // Adjust the rate limited governor setpoint
    govSetpointLimited = rampLimit(govSetpoint, govSetpointLimited, govSetpointRate);

    // Calculate error
    govError = (govSetpointLimited - hsValue) / govMaxHeadspeed;

    // P-term filter
    govP += (govKp * govError - govP) * ptFilter;

    // PID limits
    govP = constrainf(govP, -0.20f, 0.20f);
    govI = constrainf(govI, -1, 1);

    // I-term change
    float deltaI = govKi * govError * pidGetDT();

    // Governor PI sum
    govPidSum = govP + govI + deltaI;

    // Generate throttle signal
    output = govSetpointLimited * (govPidSum + govFeedForward) * 0.005f / (vbMean - vbOffset);

    // Apply deltaI if output not saturated
    if (!((output > 1 && deltaI > 0) || (output < 0 && deltaI < 0)))
        govI += deltaI;

    // Limit output to 0%..100%
    output = constrainf(output, 0, 1);

    return output;
}


/*
 * This is an advanced estimator model, with system identification.
 */

static float govMEPIControl(void)
{
    float output = 0;

    // Adjust the rate limited governor setpoint
    govSetpointLimited = rampLimit(govSetpoint, govSetpointLimited, govSetpointRate);

    // Calculate error ratio
    govError = (govSetpointLimited - hsValue) / govMaxHeadspeed;

    // P-term filter
    govP += (govKp * govError - govP) * ptFilter;

    // PID limits
    govP = constrainf(govP, -0.10f, 0.10f);
    govI = constrainf(govI, -0.05f, 0.05f);

    // I-term change
    float deltaI = govKi * govError * pidGetDT();

    // Governor PI sum
    govPidSum = govP + govI + deltaI;

    // Calculate throttle estimate from the model
    float tqEstimate = cfEstimate * ffValue + csEstimate;
    float thEstimate = (govSetpointLimited * tqEstimate) / (vbMean - vbOffset);

    // Generate throttle signal
    output = thEstimate + govPidSum;

    // Apply deltaI if output not saturated
    if (!((output > 1 && deltaI > 0) || (output < 0 && deltaI < 0)))
        govI += deltaI;

    // Limit output to 0%..100%
    output = constrainf(output, 0, 1);

    return output;
}


/*
 * This is an advanced estimator model, with system identification.
 */

static float govAEPIControl(void)
{
    float output = 0;

    // Adjust the rate limited governor setpoint
    govSetpointLimited = rampLimit(govSetpoint, govSetpointLimited, govSetpointRate);

    // Calculate error ratio
    govError = (govSetpointLimited - hsValue) / govMaxHeadspeed;

    // P-term filter
    govP += (govKp * govError - govP) * ptFilter;

    // PID limits
    govP = constrainf(govP, -0.20f, 0.20f);
    govI = constrainf(govI, -0.10f, 0.10f);

    // I-term change
    float deltaI = govKi * govError * pidGetDT();

    // Governor PI sum
    govPidSum = govP + govI + deltaI;

    // Calculate throttle estimate from the model
    float tqEstimate = cfEstimate * ffValue + csEstimate + govPidSum * 0.005f;
    float thEstimate = (govSetpointLimited * tqEstimate) / (vbMean - vbOffset);

    // Generate throttle signal
    output = thEstimate;

    // Apply deltaI if output not saturated
    if (!((output > 1 && deltaI > 0) || (output < 0 && deltaI < 0)))
        govI += deltaI;

    // Limit output to 0%..100%
    output = constrainf(output, 0, 1);

    return output;
}


void governorUpdateModels(void)
{
    switch (govMode) {
        case GM_MODEL1:
            governorUpdatePassthrough();
            break;
        case GM_MODEL2:
            governorUpdateState(govPIFControl);
            break;
        case GM_MODEL3:
            governorUpdateState(govEPIFControl);
            break;
        case GM_MODEL4:
            governorUpdateState(govMEPIControl);
            break;
        case GM_MODEL5:
            governorUpdateState(govAEPIControl);
            break;
    }
}
