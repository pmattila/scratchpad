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

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/gimbal.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoCenterPulse = 1500;
    servoConfig->dev.servoPwmRate = 50;
    servoConfig->tri_unarmed_servo = 1;
    servoConfig->servo_lowpass_freq = 0;
    servoConfig->channelForwardingStartChannel = AUX1;

    for (unsigned servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        servoConfig->dev.ioTags[servoIndex] = timerioTagGetByUsage(TIM_USE_SERVO, servoIndex);
    }
}

PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
            .min = DEFAULT_SERVO_MIN,
            .max = DEFAULT_SERVO_MAX,
            .middle = DEFAULT_SERVO_MIDDLE,
            .rate = 100,
            .forwardFromChannel = CHANNEL_FORWARDING_DISABLED
        );
    }
}

// no template required since default is zero
PG_REGISTER(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);

int16_t servo[MAX_SUPPORTED_SERVOS];

static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];

int servo_override[MAX_SUPPORTED_SERVOS];
int servo_input_override[5];


int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeState.channelCount) {
        return rcData[channelToForwardFrom];
    }

    return servoParams(servoIndex)->middle;
}

int servoDirection(int servoIndex, int inputSource)
{
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoParams(servoIndex)->reversedSources & (1 << inputSource)) {
        return -1;
    } else {
        return 1;
    }
}

void servosInit(void)
{
    // Reset servo position override
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo_override[i] = 2001;
    }
    for (int i = 0; i < 5; i++) {
        servo_input_override[i] = 0;
    }

    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }
}

void loadCustomServoMixer(void)
{
    // reset settings
    servoRuleCount = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers(i)->rate == 0) {
            break;
        }
        currentServoMixer[i] = *customServoMixers(i);
        servoRuleCount++;
    }
}

void servoConfigureOutput(void)
{
    loadCustomServoMixer();
}


STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
{
    // start forwarding from this channel
    int channelOffset = servoConfig()->channelForwardingStartChannel;
    const int maxAuxChannelCount = MIN(MAX_AUX_CHANNEL_COUNT, rxConfig()->max_aux_channel);
    for (int servoOffset = 0; servoOffset < maxAuxChannelCount && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
        pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
    }
}

// Write and keep track of written servos

static uint32_t servoWritten;

STATIC_ASSERT(sizeof(servoWritten) * 8 >= MAX_SUPPORTED_SERVOS, servoWritten_is_too_small);

static void writeServoWithTracking(uint8_t index, servoIndex_e servoname)
{
    pwmWriteServo(index, servo[servoname]);
    servoWritten |= (1 << servoname);
}

static void updateGimbalServos(uint8_t firstServoIndex)
{
    writeServoWithTracking(firstServoIndex + 0, SERVO_GIMBAL_PITCH);
    writeServoWithTracking(firstServoIndex + 1, SERVO_GIMBAL_ROLL);
}

static void servoTable(void);
static void filterServos(void);

void writeServos(void)
{
    servoTable();
    filterServos();

    uint8_t servoIndex = 0;

    // HF3D: Legacy...
    for (int i = SERVO_HELI_FIRST; i <= SERVO_HELI_LAST; i++) {
        writeServoWithTracking(servoIndex++, i);
    }

    // Two servos for SERVO_TILT, if enabled
    if (featureIsEnabled(FEATURE_SERVO_TILT)) {
        updateGimbalServos(servoIndex);
        servoIndex += 2;
    }

    // Scan servos and write those marked forwarded and not written yet
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
        if ((channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) && !(servoWritten & (1 << i))) {
            pwmWriteServo(servoIndex++, servo[i]);
        }
    }

    // forward AUX to remaining servo outputs (not constrained)
    if (featureIsEnabled(FEATURE_CHANNEL_FORWARDING)) {
        forwardAuxChannelsToServos(servoIndex);
        servoIndex += MAX_AUX_CHANNEL_COUNT;
    }
}

static float cyclicTotal = 0;
static float cyclicLimit = 0;

// Generic servo mixing from Cleanflight using user-defined smix values for each servo
void servoMixer(void)
{
    int16_t input[INPUT_SOURCE_COUNT];    // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];

    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL] = rcCommand[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommand[YAW];
    } else {
        // Assisted modes (gyro only or gyro+acc according to flight mode / AUX switch configuration)
        // Default PID_SERVO_MIXER_SCALING = 0.7f
        // HF3D TODO:  Consider using yawPidSumLimit like mixer.c does for motors?
        //  * Already added roll/pitch pidSumLimit to this code.
        //    Default pidSumLimit is 500 on roll/pitch and 400 on yaw, with min/max settable range of 100-1000
        input[INPUT_STABILIZED_ROLL] = constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_PITCH] = constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) * PID_SERVO_MIXER_SCALING;
        // NOTE:  WIth servo mixer scaling applied to yaw, it means that a pidSum of 1428 (143% in BB Explorer) is needed to max out the yaw channel.
        input[INPUT_STABILIZED_YAW] = pidData[FD_YAW].Sum * PID_SERVO_MIXER_SCALING;
    }

    input[INPUT_GIMBAL_PITCH] = scaleRange(attitude.values.pitch, -1800, 1800, -500, +500);
    input[INPUT_GIMBAL_ROLL] = scaleRange(attitude.values.roll, -1800, 1800, -500, +500);

    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]
    input[INPUT_STABILIZED_COLLECTIVE] = rcCommand[COLLECTIVE];

    // center the RC input value around the RC middle value
    input[INPUT_RC_ROLL]       = rcData[ROLL]       - rxConfig()->midrc;
    input[INPUT_RC_PITCH]      = rcData[PITCH]      - rxConfig()->midrc;
    input[INPUT_RC_YAW]        = rcData[YAW]        - rxConfig()->midrc;
    input[INPUT_RC_THROTTLE]   = rcData[THROTTLE]   - rxConfig()->midrc;
    input[INPUT_RC_COLLECTIVE] = rcData[COLLECTIVE] - rxConfig()->midrc;
    input[INPUT_RC_AUX1]       = rcData[AUX1]       - rxConfig()->midrc;
    input[INPUT_RC_AUX2]       = rcData[AUX2]       - rxConfig()->midrc;
    input[INPUT_RC_AUX3]       = rcData[AUX3]       - rxConfig()->midrc;
    input[INPUT_RC_AUX4]       = rcData[AUX4]       - rxConfig()->midrc;

    // HF3D TODO:  Implement collective max/min limit settings and then scale the input RC command to those limits.

    // HF3D TODO:  Does swash ring need to be implemented on the rcCommand side also for roll & pitch?
    //          Or does that just sort of happen by default when it's implemented on the back-end like this?

    // HF3D:  Swash ring (cyclic ring) functionality and maximum swash tilt limiting (maximum cyclic pitch)
    //   Without swash ring a full corner cyclic stick deflection would result in up to 141% of the maximum tilt in a single axis
    //   Default pidSum limit = 500 * 0.7 scale factor = 350 for each of roll and pitch
    //   Combined output for both axis = sqrt(350^2+350^2) = 495 for the maximum roll+pitch command from the pid loop.  
    //   Divide each by the combined total to scale them down such that the total combined cyclic command equals the max in one axis.
    cyclicTotal = sqrt(input[INPUT_STABILIZED_ROLL]*input[INPUT_STABILIZED_ROLL] + input[INPUT_STABILIZED_PITCH]*input[INPUT_STABILIZED_PITCH]);

    // Check if cyclicTotal combination exceeds the maximum possible deflection in any one direction
    // HF3D TODO:  Be very cautious of increasing PID_SERVO_MIXER_SCALING in the future code!!  
    //   ** Users may unexpectedly end up with more cyclic pitch than they originally setup if you change it!
    //   Maybe change the max criteria to something else?  It's actually the physical servo output we want to limit...
    cyclicLimit = currentPidProfile->pidSumLimit * PID_SERVO_MIXER_SCALING;

    if (cyclicTotal > cyclicLimit) {
        // Limit deflection off-axis if total requested servo deflection is greater than the maximum deflection on any one axis.
        input[INPUT_STABILIZED_ROLL] = input[INPUT_STABILIZED_ROLL] * cyclicLimit / cyclicTotal;
        input[INPUT_STABILIZED_PITCH] = input[INPUT_STABILIZED_PITCH] * cyclicLimit / cyclicTotal;
    }    
    // NOTE:  pidSumLimit for roll & pitch should be increased until exactly 10 degrees of cyclic pitch is achieved at maximum swash deflection and zero collective pitch
    //   .... Actually, maybe the rates should be increased/decreased instead of pidSumLimit.  This would allow very similar gains to be used across different size helis as long as max cyclic pitch is similar.
    
    // HF3D:  Override servo mixer inputs if user asks us to (via CLI or MSP/Configurator)
    //   "servo_input_override ON" sets servo_input_override[4] to 1.  OFF sets it to 0.
    //   "servo_input_override 0 50" sets collective input to 50 (inputs are generally on a -500 to +500 range)
    //   NOTE:  The only limitations to these overrides are the servo max/min PWM!  Be careful!
    if (!ARMING_FLAG(ARMED) && servo_input_override[4]) {
        input[INPUT_STABILIZED_COLLECTIVE] = servo_input_override[0];
        input[INPUT_STABILIZED_ROLL] = servo_input_override[1];
        input[INPUT_STABILIZED_PITCH] = servo_input_override[2];
        input[INPUT_STABILIZED_YAW] = servo_input_override[3];
    }

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = 0;
    }

    // mix servos according to smix rules
    //   https://github.com/cleanflight/cleanflight/blob/master/docs/Mixer.md
    for (int i = 0; i < servoRuleCount; i++) {
        // consider rule if no box assigned or box is active
        if (currentServoMixer[i].box == 0 || IS_RC_MODE_ACTIVE(BOXSERVO1 + currentServoMixer[i].box - 1)) {
            uint8_t target = currentServoMixer[i].targetChannel;
            uint8_t from = currentServoMixer[i].inputSource;
            uint16_t servo_width = servoParams(target)->max - servoParams(target)->min;
            int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
            int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;

            // See if the smix was setup as being speed limited.
            if (currentServoMixer[i].speed == 0)
                currentOutput[i] = input[from];
            else {
                if (currentOutput[i] < input[from])
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                else if (currentOutput[i] > input[from])
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
            }

            servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
        } else {
            currentOutput[i] = 0;
        }
    }

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoParams(i)->rate * servo[i]) / 100L;
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}


static void servoTable(void)
{
    servoMixer();

    // camera stabilization
    if (featureIsEnabled(FEATURE_SERVO_TILT)) {
        // center at fixed position, or vary either pitch or roll by RC channel
        servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);

        if (IS_RC_MODE_ACTIVE(BOXCAMSTAB)) {
            if (gimbalConfig()->mode == GIMBAL_MODE_MIXTILT) {
                servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 - (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
                servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 + (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
            } else {
                servo[SERVO_GIMBAL_PITCH] += (int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate * attitude.values.pitch / 50;
                servo[SERVO_GIMBAL_ROLL] += (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll  / 50;
            }
        }
    }

    // HF3D:  add offset to servo center position if user asks us to (via CLI or MSP/Configurator)
    //  When combined with "servo_input_override on" this allows the user to determine servo centers for swashplate leveling
    //  Recommend first turning on servo_input_override on, adjusting swash level, and then adjusting collective zero point
    //  Then type "servo_position" to see the new servo output values.  Use the "servo" command to make these the new center point for each servo.
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            if (servo_override[i] < 2000) {
                servo[i] += servo_override[i];
            }
        }
    }

    // constrain servos
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = constrain(servo[i], servoParams(i)->min, servoParams(i)->max); // limit the values
    }
}

bool isMixerUsingServos(void)
{
    return true;
}

static biquadFilter_t servoFilter[MAX_SUPPORTED_SERVOS];

void servosFilterInit(void)
{
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            biquadFilterInitLPF(&servoFilter[servoIdx], servoConfig()->servo_lowpass_freq, targetPidLooptime);
        }
    }

}
static void filterServos(void)
{
#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = lrintf(biquadFilterApply(&servoFilter[servoIdx], (float)servo[servoIdx]));
            // Sanity check
            servo[servoIdx] = constrain(servo[servoIdx], servoParams(servoIdx)->min, servoParams(servoIdx)->max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
}
#endif // USE_SERVOS

float servosGetCyclicDeflection(void)
{
    return MIN(cyclicTotal / cyclicLimit, 1.0f);
}

