/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/feature.h"

#include "pg/motor.h"
#include "pg/rx.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "io/motors.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = DEFAULT_MIXER,
    .yaw_motors_reversed = false,
    .gov_max_headspeed = 7000,
    .gov_gear_ratio = 1000,
    .gov_p_gain = 0,
    .gov_i_gain = 0,
    .gov_cyclic_ff_gain = 0,
    .gov_collective_ff_gain = 0,
    .gov_collective_ff_impulse_gain = 0,
    .spoolup_time = 10
);

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

#define PWM_RANGE_MID 1500

static FAST_RAM_ZERO_INIT uint8_t motorCount;

float FAST_RAM_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];


static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};
#ifndef USE_QUAD_MIXER_ONLY
static const motorMixer_t mixerTricopter[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerBicopter[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};
#else
#define mixerBicopter NULL
#endif

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};


#if defined(USE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 6)
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};

static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};
static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};
#else
#define mixerHex6H NULL
#define mixerHex6P NULL
#define mixerY6 NULL
#define mixerHex6X NULL
#endif // MAX_SUPPORTED_MOTORS >= 6

#if defined(USE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 8)
static const motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] = {
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f,  1.0f, -0.414178f,  1.0f },      // MIDFRONT_L
    { 1.0f, -0.414178f, -1.0f,  1.0f },      // FRONT_R
    { 1.0f, -1.0f,  0.414178f,  1.0f },      // MIDREAR_R
    { 1.0f,  0.414178f,  1.0f,  1.0f },      // REAR_L
    { 1.0f,  0.414178f, -1.0f, -1.0f },      // FRONT_L
    { 1.0f, -1.0f, -0.414178f, -1.0f },      // MIDFRONT_R
    { 1.0f, -0.414178f,  1.0f, -1.0f },      // REAR_R
    { 1.0f,  1.0f,  0.414178f, -1.0f },      // MIDREAR_L
};
#else
#define mixerOctoX8 NULL
#define mixerOctoFlatP NULL
#define mixerOctoFlatX NULL
#endif

static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  -0.58f,  0.58f, 1.0f },        // REAR_R
    { 1.0f,  -0.46f, -0.39f, -0.5f },       // FRONT_R
    { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f, 0.5f },         // FRONT_L
};

static const motorMixer_t mixerAtail4[] = {
    { 1.0f, -0.58f,  0.58f, -1.0f },          // REAR_R
    { 1.0f, -0.46f, -0.39f,  0.5f },          // FRONT_R
    { 1.0f,  0.58f,  0.58f,  1.0f },          // REAR_L
    { 1.0f,  0.46f, -0.39f, -0.5f },          // FRONT_L
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};
#else
#define mixerDualcopter NULL
#endif

static const motorMixer_t mixerSingleProp[] = {
    { 1.0f,  0.0f,  0.0f, 0.0f },
};

static const motorMixer_t mixerQuadX1234[] = {
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
};

// Keep synced with mixerMode_e
// Some of these entries are bogus when servos (USE_SERVOS) are not configured,
// but left untouched to keep ordinals synced with mixerMode_e (and configurator).
const mixer_t mixers[] = {
    // motors, use servo, motor mixer
    { 0, false, NULL },                // entry 0
    { 3, true,  mixerTricopter },      // MIXER_TRI
    { 4, false, mixerQuadP },          // MIXER_QUADP
    { 4, false, mixerQuadX },          // MIXER_QUADX
    { 2, true,  mixerBicopter },       // MIXER_BICOPTER
    { 0, true,  NULL },                // * MIXER_GIMBAL
    { 6, false, mixerY6 },             // MIXER_Y6
    { 6, false, mixerHex6P },          // MIXER_HEX6
    { 1, true,  mixerSingleProp },     // * MIXER_FLYING_WING
    { 4, false, mixerY4 },             // MIXER_Y4
    { 6, false, mixerHex6X },          // MIXER_HEX6X
    { 8, false, mixerOctoX8 },         // MIXER_OCTOX8
    { 8, false, mixerOctoFlatP },      // MIXER_OCTOFLATP
    { 8, false, mixerOctoFlatX },      // MIXER_OCTOFLATX
    { 1, true,  mixerSingleProp },     // * MIXER_AIRPLANE
    { 1, true,  mixerSingleProp },     // * MIXER_HELI_120_CCPM
    { 0, true,  NULL },                // * MIXER_HELI_90_DEG
    { 4, false, mixerVtail4 },         // MIXER_VTAIL4
    { 6, false, mixerHex6H },          // MIXER_HEX6H
    { 0, true,  NULL },                // * MIXER_PPM_TO_SERVO
    { 2, true,  mixerDualcopter },     // MIXER_DUALCOPTER
    { 1, true,  NULL },                // MIXER_SINGLECOPTER
    { 4, false, mixerAtail4 },         // MIXER_ATAIL4
    { 0, false, NULL },                // MIXER_CUSTOM
    { 2, true,  NULL },                // MIXER_CUSTOM_AIRPLANE
    { 3, true,  NULL },                // MIXER_CUSTOM_TRI
    { 4, false, mixerQuadX1234 },
};
#endif // !USE_QUAD_MIXER_ONLY

FAST_RAM_ZERO_INIT float motorOutputHigh, motorOutputLow;

static FAST_RAM_ZERO_INIT float disarmMotorOutput, deadbandMotor3dHigh, deadbandMotor3dLow;
static FAST_RAM_ZERO_INIT float rcCommandThrottleRange;

// Governor & Spool-up
static FAST_RAM_ZERO_INIT float govMaxHeadspeed;
static FAST_RAM_ZERO_INIT float govGearRatio;
static FAST_RAM_ZERO_INIT float govRampRate;
static FAST_RAM_ZERO_INIT float govKp;
static FAST_RAM_ZERO_INIT float govKi;
static FAST_RAM_ZERO_INIT float govCycKf;
static FAST_RAM_ZERO_INIT float govColKf;
static FAST_RAM_ZERO_INIT float govColPulseKf;

uint8_t getMotorCount(void)
{
    return motorCount;
}

bool areMotorsRunning(void)
{
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < motorCount; i++) {
            if (motor_disarmed[i] != disarmMotorOutput) {
                motorsRunning = true;

                break;
            }
        }
    }

    return motorsRunning;
}

// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
void initEscEndpoints(void)
{
    float motorOutputLimit = 1.0f;
    if (currentPidProfile->motor_output_limit < 100) {
        motorOutputLimit = currentPidProfile->motor_output_limit / 100.0f;
    }

    motorInitEndpoints(motorConfig(), motorOutputLimit, &motorOutputLow, &motorOutputHigh, &disarmMotorOutput, &deadbandMotor3dHigh, &deadbandMotor3dLow);

    rcCommandThrottleRange = PWM_RANGE_MAX - PWM_RANGE_MIN;
}

// Initialize pidProfile related mixer settings
void mixerInitProfile(void)
{

}

void mixerInit(mixerMode_e mixerMode)
{
    currentMixerMode = mixerMode;

    initEscEndpoints();

    mixerInitProfile();

    // Initialize governor settings
    govMaxHeadspeed = mixerConfig()->gov_max_headspeed;
    govGearRatio = (float)mixerConfig()->gov_gear_ratio / 1000.0f;
    govKp = (float)mixerConfig()->gov_p_gain / 10.0f;
    govKi = (float)mixerConfig()->gov_i_gain / 10.0f;
    govCycKf = (float)mixerConfig()->gov_cyclic_ff_gain / 100.0f;
    govColKf = (float)mixerConfig()->gov_collective_ff_gain / 10000.0f;
    govColPulseKf = (float)mixerConfig()->gov_collective_ff_impulse_gain / 10000.0f;
}

#ifndef USE_QUAD_MIXER_ONLY

void mixerConfigureOutput(void)
{
    motorCount = 0;

    if (currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        // load custom mixer into currentMixer
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            // Check if done by seeing if this motor has any mixing
            if ((customMotorMixer(i)->throttle == 0.0f) && (customMotorMixer(i)->yaw == 0.0f)) {
                break;
            }
            currentMixer[i] = *customMotorMixer(i);
            motorCount++;
        }
    } else {
        motorCount = mixers[currentMixerMode].motorCount;
        if (motorCount > MAX_SUPPORTED_MOTORS) {
            motorCount = MAX_SUPPORTED_MOTORS;
        }
        // copy motor-based mixers
        if (mixers[currentMixerMode].motor) {
            for (int i = 0; i < motorCount; i++)
                currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }
    mixerResetDisarmedMotors();
}

void mixerLoadMix(int index, motorMixer_t *customMixers)
{
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        customMixers[i].throttle = 0.0f;
    }
    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (int i = 0; i < mixers[index].motorCount; i++) {
            customMixers[i] = mixers[index].motor[i];
        }
    }
}
#else
void mixerConfigureOutput(void)
{
    motorCount = QUAD_MOTOR_COUNT;
    for (int i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
    mixerResetDisarmedMotors();
}
#endif // USE_QUAD_MIXER_ONLY

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = disarmMotorOutput;
    }
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_RAM_ZERO_INIT float throttle = 0;
static FAST_RAM_ZERO_INIT float mixerThrottle = 0;
static FAST_RAM_ZERO_INIT float motorOutputMin;
static FAST_RAM_ZERO_INIT float motorRangeMin;
static FAST_RAM_ZERO_INIT float motorRangeMax;
static FAST_RAM_ZERO_INIT float motorOutputRange;
static FAST_RAM_ZERO_INIT int8_t motorOutputMixSign;


static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    static float motorRangeMinIncrease = 0;
    float currentThrottleInputRange = 0;

    {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN;
        float appliedMotorOutputLow = motorOutputLow;
        motorRangeMax = motorOutputHigh;

        currentThrottleInputRange = rcCommandThrottleRange;
        motorRangeMin = appliedMotorOutputLow + motorRangeMinIncrease * (motorOutputHigh - appliedMotorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;
    }

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

// HF3D TODO:  Move governor logic to separate source file
static FAST_RAM_ZERO_INIT float lastSpoolThrottle = 0;
static FAST_RAM_ZERO_INIT uint8_t spooledUp = 0;
static FAST_RAM_ZERO_INIT float governorSetpoint;
static FAST_RAM_ZERO_INIT float governorSetpointLimited = 0.0f;
static FAST_RAM_ZERO_INIT float govBaseThrottle;
static FAST_RAM_ZERO_INIT float govPidSum = 0;
static FAST_RAM_ZERO_INIT float govI = 0;
static FAST_RAM_ZERO_INIT float govCollectiveFF = 0;
static FAST_RAM_ZERO_INIT float govCollectivePulseFF = 0;
static FAST_RAM_ZERO_INIT timeMs_t lastSpoolEndTime = 0;
static FAST_RAM_ZERO_INIT float headspeed = 0;

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    UNUSED(activeMixer);
    
    // HF3D: Re-wrote this section for main and optional tail motor use.  No longer valid for multirotors.
    //   Main motor must be motor[0] (Motor 0)
    //     * Use resource assignment to reassign output pin on board for main motor ESC to Motor 1
    //     * mmix 0 uses the Throttle% to determine ramp rate
    //   Tail motor is optional and must be motor[1] (Motor 1)
    //     * If used, tail motor should have 100% yaw mixing
    //     * mmix 1 (tail motor) uses the Throttle% to determine tailMotorBaseThrustGain
    //     *    1.0 would be way too high.  Full tail thrust when head is at 100% rpm.
    
    // Store mainMotorThrottle so we can override setting later if we're in pass-through mode
    float mainMotorThrottle = throttle;         // Used later by the tail code to set the base tail motor output as a fraction of main motor output

    // Calculate rampRate.  Must do it here vs. in init because when mixer is initialized we don't know dT yet.
    //  HF3D TODO:  This could be moved to a separate "init" function that's called from pid.c's function that sets the dT variable.
    //     Not sure that complication would really help reduce CPU load much.
    if (mixerConfig()->spoolup_time > 0) {
        govRampRate = pidGetDT() / (float)mixerConfig()->spoolup_time;
    } else {
        // Assume a ramprate of 8 seconds for external governor / ESC when in pass-through mode.
        // Will allow us to set spooled-up flag if we see headspeed > 1000rpm on a RPM sensor or telemetry.
        govRampRate = pidGetDT() / 8.0f;
    }

    // Handle MAIN motor (motor[0]) throttle output & spool-up
    if (motorCount > 0) {

        // Calculate headspeed
#ifdef USE_RPM_FILTER_TODO
        float mainMotorRPM = rpmGetFilteredMotorRPM(0);    // Get filtered main motor rpm from rpm_filter's source
#else
        float mainMotorRPM = 0.0f;
#endif        
        headspeed = mainMotorRPM / govGearRatio;
        
        // Some logic to help us come back from a stage 1 failsafe / glitch / RPM loss / accidental throttle hold quickly
        // We're going to use this time to lock in our spooledUp state for a few seconds after throttle = 0 when we were just spooledUp on the last pass through
        // Also gives us a few second window if we lose headspeed signal... in that case we'll fall back to the commanded throttle value
        if (spooledUp && (throttle == 0.0f || headspeed < 1000.0f) && cmp32(millis(), lastSpoolEndTime) > 5500) {
            // Time check above must be set just a little longer than any of the lastSpoolEndTime checks below.
            // HF3D TODO:  Maybe change the throttle check above to something >0.0f or add some more logic to allow for a faster ramp for autorotation bailout of some kind?
            lastSpoolEndTime = millis();
        }
                
        // Determine governor setpoint (use governor if throttle setting is >50%)
        // HF3D TODO:  Check !isDshotMotorTelemetryActive(i)
        if (throttle > 0.50 && govMaxHeadspeed > 0) {

            // Set the user requested governor headspeed setting
            governorSetpoint = throttle * govMaxHeadspeed;
            
            // If we don't have a non-zero rate limited setpoint yet, set it to the headspeed
            if (governorSetpointLimited <= 0) {
                governorSetpointLimited = headspeed;
            }
            
            // Increment or decrement the rate limited governor setpoint if needed
            // If ramp is set to 5s then this will allow 20% change in 1 second, or 10% headspeed in 0.5 seconds.
            float rampRate = govRampRate * govMaxHeadspeed;
            // Check to see if we've been spooledUp recently .  If so, increase our rampRate greatly to help recover from temporary loss of throttle signal.
            // HF3D TODO:  If someone immediately plugged in their heli, armed, and took off throttle hold we could accidentally hit this fast governor ramp.  That would be crazy... but possible I guess?
            if ( spooledUp && cmp32(millis(), lastSpoolEndTime) < 5000 ) {
                rampRate *= 7.0f;
            }
            if ((governorSetpoint - governorSetpointLimited) > rampRate) {
                // Setpoint is higher than the rate limited setpoint, so increment limited setpoint higher
                governorSetpointLimited = constrainf(governorSetpointLimited + rampRate, governorSetpointLimited, governorSetpoint);                
            
            } else if ((governorSetpointLimited - governorSetpoint) > rampRate)  {
                // Setpoint is lower than the rate limited setpoint, so decrement limited setpoint lower
                governorSetpointLimited = constrainf(governorSetpointLimited - rampRate, governorSetpoint, governorSetpointLimited);
            }
        } else {
            // Throttle is less than 50%, don't use governor.
            governorSetpoint = 0;
            governorSetpointLimited = 0;
        }

        if (headspeed == 0) {
            // Disable governor if main motor RPM is not available
            governorSetpoint = 0;
            governorSetpointLimited = 0;
        }
    
        //----------------- Spoolup Logic ----------------
        // MVP:  If Main Motor RPM <1000, then reset spooledUp flag and use spooledUp logic
        // HF3D TODO:  Add auto-rotation mode that ignores this check, or at least decreases the ramp time significantly.
        // HF3D TODO:  Clean-up spool-up logic that allows for 2 modes:  Spool on throttle % only (no RPM telemetry), and Spool on Governor (Headspeed)
        //    May be easier to just have 2 sets of code... 1 for spooling on governor and one for spooling without governor??
        // HF3D TODO:
        //  Idea for RPM-based spool-up logic when governor is active...
        //  Bring throttle up to ~25%.  If no RPM detected, then disarm.
        //  Once RPM is detected, start tracking PWM vs. RPM.  
        //  If there becomes a large discrepency between PWM ramp and RPM ramp, then stop ramping PWM until ratio comes back in line.
        //  This will prevent the "lag" that sometimes occurs where we end up ramping PWM into oblivion and then the ESC "catches up" later on.
        //  See 4/9/20 log #7

        // Spool-up using governorSetpoint (not the rate limited GovernorSetpoint)
        // Determine spoolup status
        if (headspeed < 1000.0f && cmp32(millis(), lastSpoolEndTime) > 3000) {
            // Require heli to spin up slowly if it's been more than 3 seconds since we last had a headspeed below 1000 rpm
            spooledUp = 0;

        } else if (!governorSetpoint && throttle <= lastSpoolThrottle) {
            // Governor is disabled, running on throttle % only.
            // If user spools up above 1000rpm, then lowers throttle below the last spool target, allow the heli to be considered spooled up
            spooledUp = 1;
            lastSpoolThrottle = throttle;        // Allow spool target to reduce with throttle freely even if already spooledUp.
            // HF3D TODO:  There's a bug with this block....
            //   If you spool up above 1000rpm and then come back below that throttle setting (normal mode)
            //   lastSpoolThrottle will ONLY go down since that's all it's allowed to do here.
            //   Then when the governor turns on it gets this really low lastSpoolThrottle value (maybe even zero)
            //      and then throttle drops to zero and the i-term has to wind up the entire amount.
            
        } else if (!spooledUp && governorSetpoint && (headspeed > governorSetpoint*0.97)) {
            // Governor is enabled, running on headspeed
            // If headspeed is within 3% of governorSetpoint, consider the heli to be spooled up
            spooledUp = 1;
            // Set the governor's base throttle % to our last spooled throttle value
            govBaseThrottle = lastSpoolThrottle;
            // Jump the rate limited Setpoint up to the setpoint.
            governorSetpointLimited = governorSetpoint*0.97f;
        
        } else if (!spooledUp && governorSetpoint && (lastSpoolThrottle > 0.90f)) {
            // Governor is enabled, and we've hit 90% throttle trying to get within 97% the requested headspeed.
            // HF3D TODO:  Flag and alert user in the logs and/or with beep tones after flight that govMaxHeadspeed is set too high.
            spooledUp = 1;
            govBaseThrottle = lastSpoolThrottle;
            governorSetpointLimited = headspeed;
        }

        // Handle ramping of throttle
        // HF3D TODO:  Eventually add a "govEnabled" user setting flag.  If the user doesn't have the governor enabled then we need to not use headspeed to perform our spooledUp check.
        //   Right now we have governor enabled all of of the time, but eventually the code needs to support running with or without an RPM signal (gov or no gov)
        // Skip spooling logic if no throttle signal or if we're disarmed
        if ((throttle == 0.0f) || !ARMING_FLAG(ARMED)) {
            // Don't reset spooledUp flag because we want throttle to respond quickly if user drops throttle in normal mode or re-arms while blades are still spinning > 1000rpm
            //   spooledUp flag will reset anyway if RPM < 1000
            lastSpoolThrottle = 0.0f;         // Require re-spooling to start from zero if RPM<1000 and throttle=0 or disarmed
        
        // If not spooled up and throttle is higher than our last spooled-up output, spool some more
        } else if (!governorSetpoint && !spooledUp && (lastSpoolThrottle < throttle)) {
            // Governor is disabled, running on throttle % only.
            throttle = lastSpoolThrottle + govRampRate;
            lastSpoolThrottle = throttle;

        // If not spooled up and headspeed is lower than our governorSetpoint, spool some more
        } else if (governorSetpoint && !spooledUp && (headspeed < governorSetpoint)) {
            // Governor is enabled, running on headspeed.
            throttle = lastSpoolThrottle + govRampRate;
            lastSpoolThrottle = throttle;
        
        }
        // --------------- End of Spoolup Logic --------------
        
        // --------------- Feedforward Calculations ----------
        
        // Quick and dirty collective pitch linear feed-forward for the main motor
        // Calculate linear feedforward vs. collective stick position (always positive adder)
        //   Reasonable value would be 0.15 throttle addition for 12-degree collective throw..
        //   So gains in the 0.0015 - 0.0032 range depending on where max collective pitch is on the heli
        //   HF3D TODO:  Set this up so works off of a calibrated pitch value for the heli taken during setup
        govCollectiveFF = govColKf * pidGetCollectiveStickPercent();
        
        // Collective pitch impulse feed-forward for the main motor
        govCollectivePulseFF = govColPulseKf * pidGetCollectiveStickHPF();

        // HF3D TODO:  Add a cyclic stick feedforward to the governor - linear gain should be fine.
        // Additional torque is required from the motor when adding cyclic pitch, just like collective (although less)
        // Maybe use this?:  float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1;
        // It's calculated like this in the pid.c code:
        //   Calculate absolute value of the percentage of cyclic stick throw (both combined... but swash ring is the real issue).
        //   servosGetCyclicDeflection() is a 0..1.0f value that is a fraction of the total cyclic travel allowed (usually 10 degrees)
        float govCyclicFF = govCycKf * servosGetCyclicDeflection();
        
        // --------------- End of Feedforward Calculations ---

        // --------------- Governor Logic --------------------
        if (spooledUp && governorSetpointLimited) {
                
            // Calculate error as a percentage of the max headspeed, since 100% throttle should be close to max headspeed
            // HF3D TODO:  Do we really want the governor to respond the same even if setpoint is only 60% of max?
            //   100 rpm error on 60% of max would "feel" a lot different than 100 rpm error on 90% of max headspeed.
            //   But would it really require any torque differences for the same response??  Maybe, since less inertia in head?
            float govError = (governorSetpointLimited - headspeed) / govMaxHeadspeed;
            
            // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
            float govP = govKp * govError;
            // if gov_i_gain = 10 (govKi = 1), we will get 1% change in throttle for 1% error in headspeed after 1 second
            govI = constrainf(govI + govKi * govError * pidGetDT(), -50.0f, 50.0f);
            govPidSum = govP + govI;
            // float govPidSum = govP + govI;
            
            // HF3D TODO:  Scale the sums based on the average battery voltage?
            //  Note:  This should NOT apply to the tail feedforward compensations that go into the PID controller!
            //         Those compensations are related to the amount of TORQUE only... and this comp would be trying
            //            to keep torque equal, so those shouldn't have to change.
            
            // Generate our new governed throttle signal
            throttle = govBaseThrottle + govCollectiveFF + govCollectivePulseFF + govCyclicFF + govPidSum;
            // Reset any wind-up due to excess control signal
            if (throttle > 1.0f) {
                // Remove last addition to I-term to prevent further wind-up if it was moving us towards this over-control
                if (govError > 0.0f) {
                    govI = govI - govKi * govError * pidGetDT();
                }
                throttle = 1.0f;
           
            } else if (throttle < 0.0f) {
                // Remove last addition to I-term to prevent further wind-up if it was moving us towards this over-control
                // HF3D TODO:  What if I-term was at contraints before we did this?
                if (govError < 0.0f) {
                    govI = govI - govKi * govError * pidGetDT();
                }

                throttle = 0.0f;
            }

            // pidGetdT() = targetPidLooptime * 1e-6f;  // 0.00125 for 8kHz
            // Calculate similar to pt1FilterGain with cutoff frequency of 0.05Hz (20s)
            //   RC = 1 / ( 2 * M_PI_FLOAT * f_cut);  ==> RC = 3.183
            //   k = dT / (RC + dT);                  ==>  k = 0.0000393
            // Slowly adapt our govBaseThrottle over time (LPF) as a fallback in case we lose RPM data.
            // HF3D TODO:  If always flying at high collective pitch, that govCollectiveFF will end up adding into the govBaseThrottle
            //    This means the base throttle will be ramped way up if then go back towards a lower average pitch... is that a problem?
            //float govBaseThrottleChange = (throttle - govBaseThrottle) * govBaseThrottleFilterGain;
            //govBaseThrottle += govBaseThrottleChange;
            // Adjust the I-term to account for the base throttle adjustment we just made
            // HF3D TODO:  What if I-term was at contraints before we did this?
            //govI -= govBaseThrottleChange;
        
            // Set lastSpoolThrottle to track the governor throttle signal when the governor is active
            lastSpoolThrottle = throttle;

            DEBUG_SET(DEBUG_GOVERNOR, 2, govPidSum*1000.0f);             // Max pidsum will be around 1, so increase by 1000x
        }
        
        // Enable pass-through of throttle signal if spoolup_time setting = 0 and we are armed.
        //   Must be used in conjunction with setting   gov_max_headspeed = 0 to have the spoolUp flag be set if an RPM sensor/telemetry is working.
        //   spooledUp flag will be set 8 seconds after throttle hold is released if headspeed > 1000 rpm.
        if ((mixerConfig()->spoolup_time == 0) && (ARMING_FLAG(ARMED))) {
            throttle = mainMotorThrottle;    // We stored the original value of the throttle signal into mainMotorThrottle variable at the beginning of the governor/spoolup code
        }
        
        mainMotorThrottle = throttle;        // Used by the tail motor code to set the base tail motor output as a fraction of main motor output

        DEBUG_SET(DEBUG_GOVERNOR, 0, governorSetpointLimited);
        DEBUG_SET(DEBUG_GOVERNOR, 1, headspeed);
#ifdef USE_RPM_FILTER_TODO
        DEBUG_SET(DEBUG_GOVERNOR, 3, rpmGetFilteredMotorRPM(1));  // Tail motor RPM
#endif

        // HF3D:  Modified original code to ignore any idle offset value when scaling main motor output -- we should always ensure that the main motor will be 100% stopped at zero throttle.
        //   motorOutputMin = motorRangeMin = motorOutputLow = DSHOT_MIN_THROTTLE
        float motorOutput;
#ifdef USE_DSHOT
        if (isMotorProtocolDshot()) {
            motorOutput = DSHOT_MIN_THROTTLE + (motorOutputHigh - DSHOT_MIN_THROTTLE) * throttle;
        } else
#endif
        {
            // For analog PWM, use disarmMotorOutput for safety
            // if min_command = 1000 and max_command = 1940  =>  motorOutput = 1000 + (940)*throttle
            motorOutput = disarmMotorOutput + (motorOutputHigh - disarmMotorOutput) * throttle;
        }

        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < DSHOT_MIN_THROTTLE) ? disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);        // motorRangeMax = motorOutputHigh   for uni-directional motor rotation
        } else {
            // HF3D:  Prevent main motor from running or twitching when dshot_idle_value is used.
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                // Use DSHOT_MIN_THROTTLE to allow the main motor to come to a complete stop at any time
                // Also prevents the main motor from "twitching" if the dshot_idle_value is setup to prevent the tail motor from stopping
                motorOutput = constrain(motorOutput, DSHOT_MIN_THROTTLE, motorRangeMax);
            } else
#endif
            {
                // HF3D:  For analog PWM control of main motor ESC, motorRangeMin includes motorConfig()->minthrottle parameter.  
                //   This is NOT good.  Any idle minthrottle setting will cause the ESC to twitch when at zero throttle.  Set to disarmMotorOutput for now just to be safe.
                motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
            }
        }
        motor[0] = motorOutput;
        
    } // end of Main Motor handling (motor[0])

    // Handle the TAIL motor mixing & control (motor[1])
    // HF3D TODO:  Eventually need to support motor driven + variable pitch combination tails
    if (motorCount > 1) {

        // motorMix for tail motor should be 100% stabilized yaw channel
        float motorOutput = motorOutputMixSign * motorMix[1];
        
        //  For a tail motor.. we don't really want it spinning like crazy from base thrust anytime we're armed,
        //   so tone the motorOutput down a bit using the mainMotorThrottle as a gain until we're at half our throttle setting or something.
        if (!spooledUp) {
            // Track the main motor output while spooling up so that we don't have our tail motor going nuts at zero throttle
            motorOutput = mainMotorThrottle * motorOutput;
        }
        
        // Linearize the tail motor thrust  (pidApplyThrustLinearization)
#ifdef USE_THRUST_LINEARIZATION
        // Scale PID sums and throttle to linearize the system (thrust varies with rpm^2)
        //   https://github.com/betaflight/betaflight/pull/7304
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif

        // Just using the base thrust from the PID controller now.  Eventually need to revisit this.
        // Base thrust should vary with main motor RPM^2, but our tail motor also has thrust^2, so increase in base thrust will be linear
        //   Divider should be the maximum headspeed the heli can achieve
        //   Tail motor thrust needs to track main motor torque.  On a non-motor tail the tail RPM tracks the main motor RPM by default.
        
        //motorOutput += (mainMotorThrottle * tailMotorBaseThrustGain);       // Probably something like 0.2 would be a good setting for this?  Just a guess.
        //  ^^ Actually, I think this is a bad idea.  Base tail thrust will actually need to INCREASE for lower headspeeds.  Lower mainshaft rpm with some power input = more torque necessary.
        //motorOutput += (mainMotorRPM/6000.0f)*tailMotorBaseThrustGain;    // Just guessing that 20% thrust on tail will be about right for 100% rpm on head.  Nevermind, see above.  This is wrong.

        // scale tail motor output to full motor output range, including impact of any idle offset.  
        // Note that motorOutput here can still be < 0 if the motorMix is sufficiently negative.  Idle offset will be taken into account again down below.
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
        } else {
            // HF3D:  Only use dshot_idle_value when main motor is spinning.
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                if (mainMotorThrottle > 0.0) {
                    // Use dshot_idle_value to prevent tail from stopping when main motor is running
                    // motorRangeMin = DSHOT_MIN_THROTTLE + ((DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
                    motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);
                } else {
                    // Use DSHOT_MIN_THROTTLE to allow the tail to come to a complete stop when main motor isn't running
                    motorOutput = constrain(motorOutput, DSHOT_MIN_THROTTLE, motorRangeMax);
                }
            } else
#endif
            {
                // Handle analog PWM tail motor ESC.  Not sure why anyone would do that... but, okay.
                if (mainMotorThrottle > 0.0) {
                    // Use minThrottle value to prevent tail from stopping when main motor is running
                    motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);
                } else {
                    motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
                }
            }
        }
        motor[1] = motorOutput;     // Set final tail motor output

    }  // end of tail motor handling
     
    // HF3D does not support more than 1 main and 1 tail motor.  Turn any additional motors off.
    for (int i = 2; i < motorCount; i++) {
        motor[i] = disarmMotorOutput;
    }

    // Disarmed mode check
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

static void applyMotorStop(void)
{
    for (int i = 0; i < motorCount; i++) {
        motor[i] = disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    motorMixer_t * activeMixer = &currentMixer[0];

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

    // Find roll/pitch/yaw desired output
    float motorMix[MAX_SUPPORTED_MOTORS];
    for (int i = 0; i < motorCount; i++) {
        motorMix[i] =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;
    }

#ifdef USE_DYN_LPF
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    mixerThrottle = throttle;

    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}

mixerMode_e getMixerMode(void)
{
    return currentMixerMode;
}

bool isFixedWing(void)
{
    switch (currentMixerMode) {
    case MIXER_FLYING_WING:
    case MIXER_AIRPLANE:
    case MIXER_CUSTOM_AIRPLANE:
        return true;

        break;
    default:
        return false;

        break;
    }
}

uint8_t isHeliSpooledUp(void)
{
    return spooledUp;
}

float mixerGetGovGearRatio(void)
{
    return govGearRatio;
}

float mixerGetHeadSpeed(void)
{
    return headspeed;
}

