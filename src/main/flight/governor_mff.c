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

#include "pg/motor.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/freq.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "io/motors.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/esc_sensor.h"

#include "flight/rpm_filter.h"
#include "flight/governor.h"
#include "flight/motors.h"
#include "flight/mixer.h"
#include "flight/pid.h"


static FAST_RAM_ZERO_INIT timeMs_t govStateEntryTime;

static FAST_RAM_ZERO_INIT bool govRampEnabled;
static FAST_RAM_ZERO_INIT bool govAutoEnabled;

static FAST_RAM_ZERO_INIT float govMaxHeadspeed;

static FAST_RAM_ZERO_INIT long  govAutoTimeout;

static FAST_RAM_ZERO_INIT float govRampRate;
static FAST_RAM_ZERO_INIT float govHSRampRate;
static FAST_RAM_ZERO_INIT float govBailoutRate;

static FAST_RAM_ZERO_INIT float govKp;
static FAST_RAM_ZERO_INIT float govKi;
static FAST_RAM_ZERO_INIT float govCycKf;
static FAST_RAM_ZERO_INIT float govColKf;
static FAST_RAM_ZERO_INIT float govColPulseKf;

static FAST_RAM_ZERO_INIT float govError;
static FAST_RAM_ZERO_INIT float govI;
static FAST_RAM_ZERO_INIT float govP;
static FAST_RAM_ZERO_INIT float govPidSum;
static FAST_RAM_ZERO_INIT float govFeedForward;
static FAST_RAM_ZERO_INIT float govBaseThrottle;

static FAST_RAM_ZERO_INIT float govCyclicFF;
static FAST_RAM_ZERO_INIT float govCollectiveFF;
static FAST_RAM_ZERO_INIT float govCollectivePulseFF;

static FAST_RAM_ZERO_INIT float govSetpoint;
static FAST_RAM_ZERO_INIT float govSetpointLimited;

static FAST_RAM_ZERO_INIT float ffExponent;

static FAST_RAM_ZERO_INIT float vbOffset;
static FAST_RAM_ZERO_INIT float vbFilter;
static FAST_RAM_ZERO_INIT float ptFilter;
static FAST_RAM_ZERO_INIT float csFilter;
static FAST_RAM_ZERO_INIT float cfFilter;
static FAST_RAM_ZERO_INIT float cgFilter;
static FAST_RAM_ZERO_INIT float stFilter;
static FAST_RAM_ZERO_INIT float stAlpha;

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

static FAST_RAM_ZERO_INIT float ccEstimate;
static FAST_RAM_ZERO_INIT float csEstimate;
static FAST_RAM_ZERO_INIT float cfEstimate;


void governorInitModels(void)
{
    govState = GS_THROTTLE_OFF;

    govKp           = (float)governorConfig()->gov_p_gain / 10.0;
    govKi           = (float)governorConfig()->gov_i_gain / 10.0;
    govCycKf        = (float)governorConfig()->gov_cyclic_ff_gain / 100.0;
    govColKf        = (float)governorConfig()->gov_collective_ff_gain / 100.0;
    govColPulseKf   = (float)governorConfig()->gov_collective_ff_impulse_gain / 100.0;

    govMaxHeadspeed = constrainf(governorConfig()->gov_max_headspeed, 100, 10000);

    govRampEnabled  = (governorConfig()->gov_spoolup_time > 0);
    govRampRate     = pidGetDT() / constrainf(governorConfig()->gov_spoolup_time, 1, 120);
    govHSRampRate   = govRampRate * govMaxHeadspeed;

    govAutoEnabled  = (governorConfig()->gov_auto_timeout > 0 && governorConfig()->gov_bailout_time > 0);
    govAutoTimeout  = governorConfig()->gov_auto_timeout * 100;
    govBailoutRate  = pidGetDT() / constrainf(governorConfig()->gov_bailout_time, 1, 100) * 10;

    ffExponent      = (float)governorConfig()->gov_ff_exponent / 100.0f;
    vbOffset        = (float)governorConfig()->gov_vbat_offset / 100.0f;
    vbFilter        = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_vbat_filter);
    ptFilter        = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_pt_filter);
    csFilter        = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_cs_filter);
    cfFilter        = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_cf_filter);
    cgFilter        = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_cg_filter);
    stFilter        = (float)1000.0f / (pidGetPidFrequency() * governorConfig()->gov_st_filter);
    stAlpha         = (float)1.0f - stFilter;
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

static inline bool headSpeedValid(void)
{
    return (govHeadSpeed > 50);
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


static void govSaveCalibration(void)
{
    governorConfigMutable()->gov_calibration[0] = 1e3f * ccEstimate;
    governorConfigMutable()->gov_calibration[1] = 1e6f * cfEstimate;
    governorConfigMutable()->gov_calibration[2] = 1e6f * csEstimate;
    governorConfigMutable()->gov_calibration[3] = 1e3f * cfEstimate / csEstimate;

    saveConfigAndNotify();
}


static void govResetStats(void)
{
    vtValue   = cxValue   = 0;
    vtMean    = cxMean    = hsMean = 0;
    vtDiff    = cxDiff    = hsDiff = 0;
    vtVar     = cxVar     = hsVar  = 0;
    cxffCovar = hsvtCovar = 0;
    ccValue   = csValue   = cfValue   = 0;
}

static void govDebugStats(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const float throttle = mixerGetThrottle();
    const float govMain = govOutput[GOV_MAIN];

    DEBUG32U( 0, govState);
    DEBUG32U( 1, throttleStatus);
    DEBUG32F( 2, throttle * 1e6f);
    DEBUG32F( 3, govSetpoint * 1e3f);
    DEBUG32F( 4, govSetpointLimited * 1e3f);
    DEBUG32F( 5, govHeadSpeed * 1e3f);
    DEBUG32F( 6, govMain * 1e6f);
    DEBUG32F( 7, govBaseThrottle * 1e6f);
    DEBUG32F( 8, govFeedForward * 1e6f);
    DEBUG32F( 9, govCollectiveFF * 1e6f);
    DEBUG32F(10, govCollectivePulseFF * 1e6f);
    DEBUG32F(11, govCyclicFF * 1e6f);
    DEBUG32F(12, govPidSum * 1e6f);
    DEBUG32F(13, govError * 1e6f);
    DEBUG32F(14, govP * 1e6f);
    DEBUG32F(15, govI * 1e6f);

    DEBUG32F(16, vbValue * 1e3f);
    DEBUG32F(17, vbMean * 1e3f);
    DEBUG32F(18, ibValue * 1e3f);
    DEBUG32F(19, ibMean * 1e3f);

    DEBUG32F(20, hsValue * 1e3f);
    DEBUG32F(21, hsDiff * 1e3f);
    DEBUG32F(22, hsMean * 1e3f);
    DEBUG32F(23, hsVar * 1e0f);

    DEBUG32F(24, vtValue * 1e3f);
    DEBUG32F(25, vtDiff * 1e3f);
    DEBUG32F(26, vtMean * 1e3f);
    DEBUG32F(27, vtVar * 1e0f);

    DEBUG32F(28, cxValue * 1e6f);
    DEBUG32F(29, cxDiff * 1e6f);
    DEBUG32F(30, cxMean * 1e6f);
    DEBUG32F(31, cxVar * 1e12);

    DEBUG32F(32, ffValue * 1e6f);
    DEBUG32F(33, ffDiff * 1e6f);
    DEBUG32F(34, ffMean * 1e6f);
    DEBUG32F(35, ffVar * 1e6f);

    DEBUG32F(36, hsvtCovar * 1e9f);
    DEBUG32F(37, cxffCovar * 1e9f);

    DEBUG32F(38, ccValue * 1e3f);
    DEBUG32F(39, csValue * 1e6f);
    DEBUG32F(40, cfValue * 1e6f);

    DEBUG32F(41, ccEstimate * 1e3f);
    DEBUG32F(42, csEstimate * 1e6f);
    DEBUG32F(43, cfEstimate * 1e6f);

    DEBUG_SET(DEBUG_GOVERNOR,  0, govSetpointLimited);
    DEBUG_SET(DEBUG_GOVERNOR,  1, govHeadSpeed);
    DEBUG_SET(DEBUG_GOVERNOR,  2, govPidSum * 1000.0f);
    DEBUG_SET(DEBUG_GOVERNOR,  3, govMain * 1000.0f);
}


static void govUpdateStats(void)
{
    // ESC voltage
    vbValue = getBatteryVoltageLatest() * 0.01f;
    vbMean += (vbValue - vbMean) * vbFilter;

    // ESC Current
    ibValue = getAmperageLatest() * 0.01f;
    ibMean += (ibValue - ibMean) * vbFilter;

    // Smoothed headspeed
    //hsValue = govHeadSpeed;
    hsValue += (govHeadSpeed - hsValue) * vbFilter;

    // Analysis
    if (govOutput[GOV_MAIN] > 0.10f && hsValue > 100)
    {
        vtValue = (vbMean - vbOffset) * govOutput[GOV_MAIN];
        cxValue = vtValue / hsValue;

        hsDiff = hsValue - hsMean;
        vtDiff = vtValue - vtMean;
        cxDiff = cxValue - cxMean;
        ffDiff = ffValue - ffMean;

        hsMean += hsDiff * stFilter;
        vtMean += vtDiff * stFilter;
        cxMean += cxDiff * stFilter;
        ffMean += ffDiff * stFilter;

        hsVar = (hsDiff * hsDiff * stFilter + hsVar) * stAlpha;
        vtVar = (vtDiff * vtDiff * stFilter + vtVar) * stAlpha;
        cxVar = (cxDiff * cxDiff * stFilter + cxVar) * stAlpha;
        ffVar = (ffDiff * ffDiff * stFilter + ffVar) * stAlpha;

        hsvtCovar = (hsDiff * vtDiff * stFilter + hsvtCovar) * stAlpha;
        cxffCovar = (cxDiff * ffDiff * stFilter + cxffCovar) * stAlpha;
    }
}

static void govUpdateSpoolupStats(float govMain)
{
    if (govMain > 0.10f && govHeadSpeed > 100) {
        // For analysis -- not used in the model
        if (hsVar > 100) {
            csValue = hsvtCovar / hsVar;
            ccValue = vtMean - hsMean * csValue;
        }

        // Initial value for Cs
        csEstimate = cxMean;
        ccEstimate = 0;

        // Initial value for Cf
        if (governorConfig()->gov_calibration[3] > 0)
            cfEstimate = csEstimate * governorConfig()->gov_calibration[3] / 1000.0f;
        else
            cfEstimate = csEstimate;
    }
}

static void govUpdateActiveStats(float govMain)
{
    bool mainCheck = true, pidCheck = true, hsCheck = true;

    // throttle must be between 25%..95%
    mainCheck = (govMain > 0.25f && govMain < 0.95f);

    // In MODEL3, I-term must be -2.5%..2.5%
    if (false) // (govMode == GM_MODEL3)
        pidCheck = (govI > -0.025f && govI < 0.025f);

    // Headspeed must be reasonable
    if (govMode == GM_MODEL1)
        hsCheck = (govHeadSpeed > 100);
    else
        hsCheck = (govHeadSpeed > govMaxHeadspeed * 0.1f && govHeadSpeed < govMaxHeadspeed * 1.1f);

    // System must stay in linear & controllable area for the stats to be correct
    if (mainCheck && pidCheck && hsCheck)
    {
        if (ffVar > 0.001f)
            cfValue = cxffCovar / ffVar;
        else
            cfValue = cfEstimate;

        cfValue = constrainf(cfValue, 0, 10*csEstimate);
        csValue = cxMean - ffMean * cfValue;

        csEstimate += (csValue - csEstimate) * csFilter;

        if (cfValue > cfEstimate)
            cfEstimate += (cfValue - cfEstimate) * cfFilter * ffVar;
        else
            cfEstimate += (cfValue - cfEstimate) * cgFilter * ffVar;
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
    //  -- 5x is for making this compatible with the STANDARD gov
    ffValue = powf( 5.0f * govFeedForward, ffExponent );
}


void governorUpdateModel1(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const float throttle = mixerGetThrottle();

    float govMainPrevious = govOutput[GOV_MAIN];
    float govMain = 0;

    // Calculate collective/cyclic feedforward
    govUpdateFeedForward();

    // Update statistics
    govUpdateStats();

    // Handle throttle off separately for SAFETY!
    if (!ARMING_FLAG(ARMED) || throttleStatus == THROTTLE_LOW) {
        govChangeState(GS_THROTTLE_OFF);
        govResetStats();
    }
    else {
        switch (govState) {
            // throttleStatus is now HIGH, go to SPOOLING UP or ACTIVE
            case GS_THROTTLE_OFF:
                if (govRampEnabled)
                    govChangeState(GS_PASSTHROUGH_SPOOLING_UP);
                else
                    govChangeState(GS_PASSTHROUGH_ACTIVE);
                break;

            // Follow the throttle, with a limited rampup rate.
            //  -- Once throttle is >20% and not ramping up any more, move to ACTIVE.
            case GS_PASSTHROUGH_SPOOLING_UP:
                govMain = rampUpLimit(throttle, govMainPrevious, govRampRate);
                if (govMain >= throttle && throttle > 0.20f)
                    govChangeState(GS_PASSTHROUGH_ACTIVE);
                else
                    govUpdateSpoolupStats(govMain);
                break;

            // Follow the throttle without ramp limits.
            //  -- If throttle <20%, move to IDLE or AUTO.
            case GS_PASSTHROUGH_ACTIVE:
                govMain = throttle;
                if (throttle < 0.20f && govRampEnabled) {
                    if (govAutoEnabled && govStateTime() > 5000)
                        govChangeState(GS_AUTOROTATION_CLASSIC);
                    else
                        govChangeState(GS_PASSTHROUGH_SPOOLING_UP);
                } else {
                    govUpdateActiveStats(govMain);
                }
                break;

            // Follow throttle with high(er) ramp rate.
            //  -- If throttle is >20%, move to AUTOROTATION_BAILOUT.
            //  -- If timer expires, move to IDLE.
            case GS_AUTOROTATION_CLASSIC:
                govMain = rampUpLimit(throttle, govMainPrevious, govBailoutRate);
                if (throttle > 0.20f)
                    govChangeState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > govAutoTimeout)
                    govChangeState(GS_PASSTHROUGH_SPOOLING_UP);
                break;

            // Follow the throttle, with a high(er) ramp rate.
            //  -- Once throttle is not ramping up any more, move to ACTIVE.
            //  -- If throttle <20%, move back to AUTO.
            case GS_AUTOROTATION_BAILOUT:
                govMain = rampUpLimit(throttle, govMainPrevious, govBailoutRate);
                if (throttle < 0.20f)
                    govChangeState(GS_AUTOROTATION_CLASSIC);
                else if (govMain >= throttle)
                    govChangeState(GS_PASSTHROUGH_ACTIVE);
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                govResetStats();
        }
    }

    // Update output variables
    govOutput[GOV_MAIN] = govMain;
    govOutput[GOV_TAIL] = 0;

    // Set debug
    govDebugStats();
}


static float govPIControl(void)
{
    float govMain = 0;

    // Adjust the rate limited governor setpoint
    govSetpointLimited = rampLimit(govSetpoint, govSetpointLimited, govHSRampRate);

    // Calculate error ratio
    govError = (govSetpointLimited - govHeadSpeed) / govMaxHeadspeed;

    // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
    //govP = govKp * govError;
    govP += (govKp * govError - govP) * ptFilter;

    // Make sure govI is valid
    govI = constrainf(govI, 0, 1);

    // if gov_i_gain = 10 (govKi = 1), we will get 1% change in throttle for 1% error in headspeed after 1 second
    float deltaI = constrainf(govKi * govError * pidGetDT(), 0 - govI, 1 - govI);

    // Governor PID sum
    govPidSum = govP + govI + deltaI;

    // Generate new governed throttle signal
    govMain = govPidSum + govFeedForward;

    // Apply deltaI if govMain not saturated
    if ( ! ((govMain > 1 && deltaI > 0) || (govMain < 0 && deltaI < 0)) )
        govI += deltaI;

    // Limit output to 0%..100%
    govMain = constrainf(govMain, 0, 1);

    return govMain;
}


void governorUpdateModel2(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const float throttle = mixerGetThrottle();

    float govMainPrevious = govOutput[GOV_MAIN];
    float govMain = 0;

    // Update headspeed target
    govSetpoint = throttle * govMaxHeadspeed;

    // Calculate collective/cyclic feedforward
    govUpdateFeedForward();

    // Update statistics
    govUpdateStats();

    // Handle throttle off separately for SAFETY!
    if (!ARMING_FLAG(ARMED) || throttleStatus == THROTTLE_LOW) {
        govChangeState(GS_THROTTLE_OFF);
        govResetStats();
    }
    else {
        switch (govState) {
            // throttleStatus is now HIGH, go to IDLE
            case GS_THROTTLE_OFF:
                govChangeState(GS_THROTTLE_IDLE);
                break;

            // Motor is idling, follow the throttle (with a limited rampup rate and offset).
            //  -- If throttle goes above 20%, move to SPOOLUP.
            //  -- Map throttle to motor output:
            //           0%..5%   -> motor 0%
            //           5%..20%  -> motor 0%..15%
            case GS_THROTTLE_IDLE:
                //govMain = rampUpLimit(throttle, govMainPrevious, govRampRate);
                govMain = rampUpLimit(constrainf(throttle - 0.05f, 0, 0.15f), govMainPrevious, govRampRate);
                if (throttle > 0.20f)
                    govChangeState(GS_GOVERNOR_SPOOLING_UP);
                break;

            // Ramp up throttle until headspeed target is reached.
            //  -- Once 97.5% headspeed reached, move to ACTIVE.
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE.
            //  -- If throttle <20%, move back to IDLE.
            //  -- If no headspeed detected and throttle >20%, move to ERROR.
            case GS_GOVERNOR_SPOOLING_UP:
                govMain = rampUpLimit(0.99f, govMainPrevious, govRampRate);
                if (!headSpeedValid() && govMain > 0.20f) {
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                } else if (throttle < 0.20f) {
                    govChangeState(GS_THROTTLE_IDLE);
                } else if (govHeadSpeed > govSetpoint * 0.975f || govMain > 0.95f) {
                    govChangeState(GS_GOVERNOR_ACTIVE);
                    govSetpointLimited = govHeadSpeed;
                    govI = govMain;
                } else {
                    govUpdateSpoolupStats(govMain);
                }
                break;

            // Governor active, maintain headspeed
            //  -- If throttle <20%, move to AUTOROTATION_CLASSIC or IDLE.
            //  -- If no headspeed signal, move to ERROR
            case GS_GOVERNOR_ACTIVE:
                if (!headSpeedValid() && govMain > 0.20f) {
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                } else if (throttle < 0.20f) {
                    govMain = govMainPrevious;
                    if (govStateTime() > 10000)
                        govSaveCalibration();
                    if (govAutoEnabled && govStateTime() > 5000)
                        govChangeState(GS_AUTOROTATION_CLASSIC);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                } else {
                    govMain = govPIControl();
                    govUpdateActiveStats(govMain);
                }
                break;

            // Throttle passthrough with ramup limit
            //  -- If throttle >20%, move to AUTOROTATION_BAILOUT.
            //  -- If timer expires, move to IDLE.
            //  -- Map throttle to motor output:
            //           0%..5%   -> motor 0%
            //           5%..20%  -> motor 0%..15%
            case GS_AUTOROTATION_CLASSIC:
                govMain = rampUpLimit(constrainf(throttle - 0.05f, 0, 0.15f), govMainPrevious, govBailoutRate);
                if (throttle > 0.20f)
                    govChangeState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > govAutoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- Once 97.5% headspeed reached, move to ACTIVE.
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE.
            //  -- If throttle <20%, move back to AUTOROTATION_CLASSIC
            //  -- If no headspeed detected and throttle >20%, move to ERROR.
            case GS_AUTOROTATION_BAILOUT:
                govMain = rampUpLimit(0.99f, govMainPrevious, govBailoutRate);
                if (!headSpeedValid() && govMain > 0.20f) {
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                } else if (throttle < 0.20f) {
                    govChangeState(GS_AUTOROTATION_CLASSIC);
                } else if (govHeadSpeed > govSetpoint * 0.975f || govMain > 0.95f) {
                    govChangeState(GS_GOVERNOR_ACTIVE);
                    govSetpointLimited = govHeadSpeed;
                    govI = govMain;
                }
                break;

            // No headspeed signal. Very bad. Ramp down throttle.
            //  -- Once throttle reaches 5%, move to SPOOLUP.
            //  -- If headspeed recovers, move to SPOOLUP.
            case GS_GOVERNOR_LOST_HEADSPEED:
                govMain = rampLimit(0, govMainPrevious, govRampRate);
                if (govMain < 0.05f)
                    govChangeState(GS_GOVERNOR_SPOOLING_UP);
                else if (headSpeedValid())
                    govChangeState(GS_GOVERNOR_SPOOLING_UP);
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                govResetStats();
        }
    }

    // Update output variables
    govOutput[GOV_MAIN] = govMain;
    govOutput[GOV_TAIL] = 0;

    // Set debug
    govDebugStats();
}


static float govMPIControl(void)
{
    float govMain = 0;

    // Adjust the rate limited governor setpoint
    govSetpointLimited = rampLimit(govSetpoint, govSetpointLimited, govHSRampRate);

    // Calculate throttle estimate from the physical model
    float tqEstimate = cfEstimate * ffValue + csEstimate;
    float thEstimate = (govSetpointLimited * tqEstimate + ccEstimate) / (vbMean - vbOffset);

    // Set base throttle
    govBaseThrottle = constrainf(thEstimate, 0, 0.99f);

    // Calculate error ratio
    govError = (govSetpointLimited - govHeadSpeed) / govMaxHeadspeed;

    // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
    //govP = govKp * govError;
    govP += (govKp * govError - govP) * ptFilter;

    // Make sure govI is valid
    govI = constrainf(govI, -0.05f, 0.05f);

    // if gov_i_gain = 10 (govKi = 1), we will get 1% change in throttle for 1% error in headspeed after 1 second
    float deltaI = constrainf(govKi * govError * pidGetDT(), -0.05f - govI, 0.05f - govI);

    // Governor PID sum
    govPidSum = govP + govI + deltaI;

    // Generate new governed throttle signal
    govMain = govBaseThrottle + govPidSum;

    // Apply deltaI if govMain not saturated
    if ( ! ((govMain > 1 && deltaI > 0) || (govMain < 0 && deltaI < 0)) )
        govI += deltaI;

    // Limit output to 0%..100%
    govMain = constrainf(govMain, 0, 1);

    return govMain;
}


void governorUpdateModel3(void)
{
    const throttleStatus_e throttleStatus = calculateThrottleStatus();
    const float throttle = mixerGetThrottle();

    float govMainPrevious = govOutput[GOV_MAIN];
    float govMain = 0;

    // Update headspeed target
    govSetpoint = throttle * govMaxHeadspeed;

    // Calculate collective/cyclic feedforward
    govUpdateFeedForward();

    // Update statistics
    govUpdateStats();

    // Handle throttle off separately for SAFETY!
    if (!ARMING_FLAG(ARMED) || throttleStatus == THROTTLE_LOW) {
        govChangeState(GS_THROTTLE_OFF);
        govResetStats();
    }
    else {
        switch (govState) {
            // throttleStatus is now HIGH, go to IDLE
            case GS_THROTTLE_OFF:
                govChangeState(GS_THROTTLE_IDLE);
                break;

            // Motor is idling, follow the throttle (with a limited rampup rate).
            //  -- If throttle goes above 20%, move to SPOOLUP.
            case GS_THROTTLE_IDLE:
                govMain = rampUpLimit(throttle, govMainPrevious, govRampRate);
                if (throttle > 0.20f)
                    govChangeState(GS_GOVERNOR_SPOOLING_UP);
                break;

            // Ramp up throttle until headspeed target is reached.
            //  -- Once 97.5% headspeed reached, move to ACTIVE.
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE.
            //  -- If throttle <20%, move back to IDLE.
            //  -- If no headspeed detected and throttle >20%, move to ERROR.
            case GS_GOVERNOR_SPOOLING_UP:
                govMain = rampUpLimit(0.99f, govMainPrevious, govRampRate);
                if (!headSpeedValid() && govMain > 0.20f) {
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                } else if (throttle < 0.20f) {
                    govChangeState(GS_THROTTLE_IDLE);
                } else if (govHeadSpeed > govSetpoint * 0.975f || govMain > 0.95f) {
                    govChangeState(GS_GOVERNOR_ACTIVE);
                    govSetpointLimited = govHeadSpeed;
                } else {
                    govUpdateSpoolupStats(govMain);
                }
                break;

            // Governor active, maintain headspeed
            //  -- If throttle <20%, move to AUTOROTATION_CLASSIC or IDLE.
            //  -- If no headspeed signal, move to ERROR
            case GS_GOVERNOR_ACTIVE:
                if (!headSpeedValid() && govMain > 0.20f) {
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                } else if (throttle < 0.20f) {
                    govMain = govMainPrevious;
                    if (govStateTime() > 10000)
                        govSaveCalibration();
                    if (govAutoEnabled && govStateTime() > 5000)
                        govChangeState(GS_AUTOROTATION_CLASSIC);
                    else
                        govChangeState(GS_THROTTLE_IDLE);
                } else {
                    govMain = govMPIControl();
                    govUpdateActiveStats(govMain);
                }
                break;

            // Throttle passthrough with ramup limit
            //  -- If throttle >20%, move to AUTOROTATION_BAILOUT.
            //  -- If timer expires, move to IDLE.
            //  -- Map throttle to motor output:
            //           0%..5%   -> motor 0%
            //           5%..20%  -> motor 0%..15%
            case GS_AUTOROTATION_CLASSIC:
                govMain = rampUpLimit(constrainf(throttle - 0.05f, 0, 0.15f), govMainPrevious, govBailoutRate);
                if (throttle > 0.20f)
                    govChangeState(GS_AUTOROTATION_BAILOUT);
                else if (govStateTime() > govAutoTimeout)
                    govChangeState(GS_THROTTLE_IDLE);
                break;

            // Ramp up throttle until target headspeed is reached, with fast(er) rampup.
            //  -- Once 97.5% headspeed reached, move to ACTIVE.
            //  -- If throttle reaches 95% before headspeed target, also move to ACTIVE.
            //  -- If throttle <20%, move back to AUTOROTATION_CLASSIC
            //  -- If no headspeed detected and throttle >20%, move to ERROR.
            case GS_AUTOROTATION_BAILOUT:
                govMain = rampUpLimit(0.99f, govMainPrevious, govBailoutRate);
                if (!headSpeedValid() && govMain > 0.20f) {
                    govChangeState(GS_GOVERNOR_LOST_HEADSPEED);
                } else if (throttle < 0.20f) {
                    govChangeState(GS_AUTOROTATION_CLASSIC);
                } else if (govHeadSpeed > govSetpoint * 0.975f || govMain > 0.95f) {
                    govChangeState(GS_GOVERNOR_ACTIVE);
                    govSetpointLimited = govHeadSpeed;
                }
                break;

            // No headspeed signal. Very bad. Ramp down throttle.
            //  -- Once throttle reaches 5%, move to SPOOLUP.
            //  -- If headspeed recovers, move to SPOOLUP.
            case GS_GOVERNOR_LOST_HEADSPEED:
                govMain = rampLimit(0, govMainPrevious, govRampRate);
                if (govMain < 0.05f)
                    govChangeState(GS_GOVERNOR_SPOOLING_UP);
                else if (headSpeedValid())
                    govChangeState(GS_GOVERNOR_SPOOLING_UP);
                break;

            // Should not be here
            default:
                govChangeState(GS_THROTTLE_OFF);
                govResetStats();
        }
    }

    // Update output variables
    govOutput[GOV_MAIN] = govMain;
    govOutput[GOV_TAIL] = 0;

    // Set debug
    govDebugStats();
}

