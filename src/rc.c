#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "board.h"
#include "param.h"
#include "rc.h"
#include "mux.h"
#include "mode.h"

#include "mavlink_util.h"
#include "mavlink_log.h"

static rc_switch_t switches[4];
bool _calibrate_rc;
void calibrate_rc();

static rc_channel_t channels[4];

void init_rc()
{
    _calibrate_rc = false;
    _rc_control.x.type = ANGLE;
    _rc_control.y.type = ANGLE;
    _rc_control.z.type = RATE;
    _rc_control.F.type = THROTTLE;

    _rc_control.x.value = 0;
    _rc_control.y.value = 0;
    _rc_control.z.value = 0;
    _rc_control.F.value = 0;

    _offboard_control.x.active = false;
    _offboard_control.y.active = false;
    _offboard_control.z.active = false;
    _offboard_control.F.active = false;

    switches[0].channel = 4;
    switches[0].direction = get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
    switches[1].channel = 5;
    switches[1].direction = get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
    switches[2].channel = 6;
    switches[2].direction = get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
    switches[3].channel = 7;
    switches[3].direction = get_param_int(PARAM_RC_SWITCH_8_DIRECTION);

    channels[RC_X].channel_param = PARAM_RC_X_CHANNEL;
    channels[RC_X].max_angle_param = PARAM_MAX_ROLL_ANGLE;
    channels[RC_X].max_rate_param = PARAM_MAX_ROLL_RATE;
    channels[RC_X].bottom_param = 0;
    channels[RC_X].center_param = PARAM_RC_X_CENTER;
    channels[RC_X].range_param = PARAM_RC_X_RANGE;
    channels[RC_X].control_channel_ptr = &(_rc_control.x);

    channels[RC_Y].channel_param = PARAM_RC_Y_CHANNEL;
    channels[RC_Y].max_angle_param = PARAM_MAX_PITCH_ANGLE;
    channels[RC_Y].max_rate_param = PARAM_MAX_PITCH_RATE;
    channels[RC_Y].bottom_param = 0;
    channels[RC_Y].center_param = PARAM_RC_Y_CENTER;
    channels[RC_Y].range_param = PARAM_RC_Y_RANGE;
    channels[RC_Y].control_channel_ptr = &(_rc_control.y);

    channels[RC_Z].channel_param = PARAM_RC_Z_CHANNEL;
    channels[RC_Z].max_angle_param = 0;
    channels[RC_Z].max_rate_param = PARAM_MAX_YAW_RATE;
    channels[RC_Z].bottom_param = 0;
    channels[RC_Z].center_param = PARAM_RC_Z_CENTER;
    channels[RC_Z].range_param = PARAM_RC_Z_RANGE;
    channels[RC_Z].control_channel_ptr = &(_rc_control.z);

    channels[RC_F].channel_param = PARAM_RC_F_CHANNEL;
    channels[RC_F].max_angle_param = 0;
    channels[RC_F].max_rate_param = PARAM_MAX_ROLL_RATE;
    channels[RC_F].bottom_param = PARAM_RC_F_BOTTOM;
    channels[RC_F].center_param = 0;
    channels[RC_F].range_param = PARAM_RC_F_RANGE;
    channels[RC_F].control_channel_ptr = &(_rc_control.F);
}

bool rc_switch(int16_t channel)
{
    if (channel < 4 || channel > 8)
    {
        return false;
    }
    if (switches[channel - 4].direction < 0)
    {
        return pwm_read(channel) < 1500;
    }
    else
    {
        return pwm_read(channel) > 1500;
    }
}

static void convertPWMtoRad()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        rc_channel_t* chan = &(channels[i]);
        int16_t pwm = pwm_read(get_param_int(chan->channel_param));

        // If this is the throttle channel, we need to go from 0.0 to 1.0
        // Otherwise, we need to go from -1.0 to 1.0
        if(i == RC_F)
        {
            chan->control_channel_ptr->value = (float)(pwm - get_param_int(chan->bottom_param)) /
                    (float)(get_param_int(chan->range_param));
        }
        else
        {
            chan->control_channel_ptr->value = 2.0*(float)(pwm - get_param_int(chan->center_param))/
                    (float)(get_param_int(chan->range_param));
        }

        // Now, check the mode and convert the normalized value to the appropriate units
        switch(chan->control_channel_ptr->type)
        {
        case RATE:
            chan->control_channel_ptr->value *= get_param_float(chan->max_rate_param);
            break;
        case ANGLE:
            chan->control_channel_ptr->value *= get_param_float(chan->max_angle_param);
            break;
        default: // default and altitude modes, leave as normalized
            break;
        }
    }
}

void interpret_command_type()
{
    // Figure out the desired control type from the switches and params
    if (get_param_int(PARAM_FIXED_WING))
    {
        // for using fixedwings
        _rc_control.x.type = _rc_control.y.type = _rc_control.z.type = PASSTHROUGH;
        _rc_control.F.type = THROTTLE;
    }
    else
    {
        _rc_control.x.type = _rc_control.y.type = rc_switch(get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)) ? ANGLE : RATE;
        _rc_control.z.type = RATE;
        _rc_control.F.type = rc_switch(get_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL)) ? ALTITUDE : THROTTLE;
    }
}

bool sticks_deviated(uint32_t now_ms)
{
    static uint32_t time_of_last_stick_deviation = 0;

    // If we are in the lag time, return true;
    if(now_ms - time_of_last_stick_deviation < (uint64_t)(get_param_int(PARAM_OVERRIDE_LAG_TIME)))
    {
        return true;
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            int16_t deviation = pwm_read(get_param_int(channels[i].channel_param)) - get_param_int(channels[i].center_param);
            if (abs(deviation) > get_param_int(PARAM_RC_OVERRIDE_DEVIATION))
            {
                time_of_last_stick_deviation = now_ms;
                return true;
            }
        }
        // None of the three sticks are deviated more than the override threshold
        return false;
    }
}


bool receive_rc()
{
    if (_calibrate_rc)
    {
        calibrate_rc();
    }

    uint32_t now = clock_millis();

    // if it has been more than 20ms then look for new RC values and parse them
    static uint32_t last_rc_receive_time = 0;

    if (now - last_rc_receive_time < 20)
        return false;

    last_rc_receive_time = now;

    interpret_command_type();

    // Interpret PWM Values from RC
    convertPWMtoRad();

    sticks_deviated(now);

    // Set flags for attitude channels
    if (rc_switch(get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL)) || sticks_deviated(now))
    {
        // Pilot is in full control
        _rc_control.x.active = true;
        _rc_control.y.active = true;
        _rc_control.z.active = true;
    }
    else
    {
        _rc_control.x.active = false;
        _rc_control.x.active = false;
        _rc_control.x.active = false;
    }

    // Set flags for throttle channel
    if (rc_switch(get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL)))
    {
        // RC Pilot is in full control
        _rc_control.F.active = true;
    }
    else
    {
        // Onboard Control - min throttle Checking will be done in mux and in the controller.
        _rc_control.F.active = false;
    }

    _new_command = true;
    return true;
}

void calibrate_rc()
{
    if (_armed_state == ARMED || _armed_state == FAILSAFE_ARMED || _armed_state == FAILSAFE_DISARMED)
    {
        mavlink_log_error("Cannot calibrate RC when FCU is armed or in failsafe", NULL);
    }
    else
    {
        // Calibrate Extents of RC Transmitter
        mavlink_log_warning("Calibrating RC, move sticks to full extents", NULL);
        mavlink_log_warning("in the next 10s", NULL);
        uint32_t start = clock_millis();
        static int32_t max[4] = {0, 0, 0, 0};
        static int32_t min[4] = {10000, 10000, 10000, 10000};
        while(clock_millis() - start < 1e4)
        {
            for (int16_t i = 0; i < 4; i++)
            {
                int32_t read_value = (int32_t)pwm_read(i);
                if(read_value > max[i])
                {
                    max[i] = read_value;
                }
                if (read_value < min[i])
                {
                    min[i] = read_value;
                }
            }
            clock_delay(10);
        }
        set_param_int(PARAM_RC_X_RANGE, max[get_param_int(PARAM_RC_X_CHANNEL)] - min[get_param_int(PARAM_RC_X_CHANNEL)]);
        set_param_int(PARAM_RC_Y_RANGE, max[get_param_int(PARAM_RC_Y_CHANNEL)] - min[get_param_int(PARAM_RC_Y_CHANNEL)]);
        set_param_int(PARAM_RC_Z_RANGE, max[get_param_int(PARAM_RC_Z_CHANNEL)] - min[get_param_int(PARAM_RC_Z_CHANNEL)]);
        set_param_int(PARAM_RC_F_RANGE, max[get_param_int(PARAM_RC_F_CHANNEL)] - min[get_param_int(PARAM_RC_F_CHANNEL)]);

        // Calibrate Trimmed Centers
        mavlink_log_warning("Calibrating RC, leave sticks at center", NULL);
        mavlink_log_warning("and throttle low for next 10 seconds", NULL);
        clock_delay(5000);
        start = clock_millis();
        static int32_t sum[4] = {0, 0, 0, 0};
        static int32_t count[4] = {0, 0, 0, 0};

        while(clock_millis() - start < 5e3)
        {
            for (int16_t i = 0; i < 4; i++)
            {
                int32_t read_value = (int32_t)pwm_read(i);
                sum[i] = sum[i] + read_value;
                count[i] = count[i] + 1;
            }
            clock_delay(20); // RC is updated at 50 Hz
        }

        set_param_int(PARAM_RC_X_CENTER, sum[get_param_int(PARAM_RC_X_CHANNEL)]/count[get_param_int(PARAM_RC_X_CHANNEL)]);
        set_param_int(PARAM_RC_Y_CENTER, sum[get_param_int(PARAM_RC_Y_CHANNEL)]/count[get_param_int(PARAM_RC_Y_CHANNEL)]);
        set_param_int(PARAM_RC_Z_CENTER, sum[get_param_int(PARAM_RC_Z_CHANNEL)]/count[get_param_int(PARAM_RC_Z_CHANNEL)]);
        set_param_int(PARAM_RC_F_BOTTOM, sum[get_param_int(PARAM_RC_F_CHANNEL)]/count[get_param_int(PARAM_RC_F_CHANNEL)]);
        write_params();
        mavlink_log_warning("Completed RC calibration", NULL);
    }
    _calibrate_rc = false;
}
