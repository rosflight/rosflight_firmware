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

typedef struct
{
  uint8_t channel;
  uint16_t trim;
  uint16_t range;
  bool one_sided;
} rc_stick_config_t;

typedef struct
{
  uint8_t channel;
  uint8_t direction;
  bool mapped;
} rc_switch_config_t;

bool _calibrate_rc;
void calibrate_rc();

static rc_stick_config_t sticks[RC_STICKS_COUNT];
static rc_switch_config_t switches[RC_SWITCHES_COUNT];

void init_sticks(void)
{
  sticks[RC_STICK_X].channel = get_param_int(PARAM_RC_X_CHANNEL);
  sticks[RC_STICK_X].trim = get_param_int(PARAM_RC_X_CENTER);
  sticks[RC_STICK_X].range = get_param_int(PARAM_RC_X_RANGE);
  sticks[RC_STICK_X].one_sided = false;

  sticks[RC_STICK_Y].channel = get_param_int(PARAM_RC_Y_CHANNEL);
  sticks[RC_STICK_Y].trim = get_param_int(PARAM_RC_Y_CENTER);
  sticks[RC_STICK_Y].range = get_param_int(PARAM_RC_Y_RANGE);
  sticks[RC_STICK_Y].one_sided = false;

  sticks[RC_STICK_Z].channel = get_param_int(PARAM_RC_Z_CHANNEL);
  sticks[RC_STICK_Z].trim = get_param_int(PARAM_RC_Z_CENTER);
  sticks[RC_STICK_Z].range = get_param_int(PARAM_RC_Z_RANGE);
  sticks[RC_STICK_Z].one_sided = false;

  sticks[RC_STICK_F].channel = get_param_int(PARAM_RC_F_CHANNEL);
  sticks[RC_STICK_F].trim = get_param_int(PARAM_RC_F_BOTTOM);
  sticks[RC_STICK_F].range = get_param_int(PARAM_RC_F_RANGE);
  sticks[RC_STICK_F].one_sided = true;
}

static uint8_t resolve_switch_direction(uint8_t chan)
{
  uint8_t direction = 1;
  switch (chan)
  {
  case 4:
    direction = get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
    break;
  case 5:
    direction = get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
    break;
  case 6:
    direction = get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
    break;
  case 7:
    direction = get_param_int(PARAM_RC_SWITCH_8_DIRECTION);
    break;
  }

  return direction;
}

static void init_switches()
{
  switches[RC_SWITCH_ARM] = get_param_int(PARAM_ARM_CHANNEL);
  switches[RC_SWITCH_ATT_OVERRIDE].channel = get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_THROTTLE_OVERRIDE].channel = get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_ATT_TYPE].channel = get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL);
  switches[RC_SWITCH_THROTTLE_TYPE].channel = get_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL);

  for (rc_switch_t chan = 0; chan < RC_SWITCHES_COUNT; chan++)
  {
    switches[chan].mapped = switches[chan].channel > 3 && switches[chan].channel < get_param_int(PARAM_RC_NUM_CHANNELS);
    if (!switches[chan].mapped)
    {
      mavlink_log_error("invalid RC switch channel assignment: %d", switches[chan].channel); // TODO use parameter name
    }

    switches[chan].direction = 1;
    switch (chan)
    {
    case 4:
      direction = get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
      break;
    case 5:
      direction = get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
      break;
    case 6:
      direction = get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
      break;
    case 7:
      direction = get_param_int(PARAM_RC_SWITCH_8_DIRECTION);
      break;
    }
  }
}

void init_rc()
{
    _calibrate_rc = false;

//    _rc_control.x.type = ANGLE;
//    _rc_control.y.type = ANGLE;
//    _rc_control.z.type = RATE;
//    _rc_control.F.type = THROTTLE;

//    _rc_control.x.value = 0;
//    _rc_control.y.value = 0;
//    _rc_control.z.value = 0;
//    _rc_control.F.value = 0;

//    _offboard_control.x.active = false;
//    _offboard_control.y.active = false;
//    _offboard_control.z.active = false;
//    _offboard_control.F.active = false;

    init_sticks();
    init_switches();

//    sticks[RC_STICK_X].max_angle_param = PARAM_MAX_ROLL_ANGLE;
//    sticks[RC_STICK_X].max_rate_param = PARAM_MAX_ROLL_RATE;
//    sticks[RC_STICK_X].control_channel_ptr = &(_rc_control.x);

//    sticks[RC_STICK_Y].max_angle_param = PARAM_MAX_PITCH_ANGLE;
//    sticks[RC_STICK_Y].max_rate_param = PARAM_MAX_PITCH_RATE;
//    sticks[RC_STICK_Y].control_channel_ptr = &(_rc_control.y);

//    sticks[RC_STICK_Z].max_angle_param = 0;
//    sticks[RC_STICK_Z].max_rate_param = PARAM_MAX_YAW_RATE;
//    sticks[RC_STICK_Z].control_channel_ptr = &(_rc_control.z);

//    sticks[RC_STICK_F].max_angle_param = 0;
//    sticks[RC_STICK_F].max_rate_param = PARAM_MAX_ROLL_RATE;
//    sticks[RC_STICK_F].control_channel_ptr = &(_rc_control.F);
}

float rc_stick(rc_stick_t channel)
{
  uint16_t pwm = pwm_read(sticks[channel].channel);
  return (sticks[channel].one_sided ? 1.0f : 2.0f) * (float)(pwm - sticks[channel].trim) / (float) sticks[channel].range;
}

bool rc_switch(rc_switch_t channel)
{
  if (!switches[channel].mapped)
  {
    return false;
  }

  if (switches[channel].direction < 0)
  {
    return pwm_read(switches[channel].channel) < 1500;
  }
  else
  {
    return pwm_read(switches[channel].channel) >= 1500;
  }
}

bool rc_low(int16_t channel)
{
  if (channel < 4)
  {
    rc_stick_config_t* rc_ptr = &(sticks[channel]);
    int16_t pwm = pwm_read(get_param_int(rc_ptr->channel_param));
    if(channel != RC_STICK_F)
      return pwm < get_param_int(rc_ptr->center_param) - get_param_int(rc_ptr->range_param)/2 +get_param_int(PARAM_ARM_THRESHOLD);
    else
      return pwm < get_param_int(rc_ptr->bottom_param) + get_param_int(PARAM_ARM_THRESHOLD);
  }
  return false;
}

bool rc_high(int16_t channel)
{
  if(channel < 4)
  {
    rc_stick_config_t* rc_ptr = &(sticks[channel]);
    int16_t pwm = pwm_read(get_param_int(rc_ptr->channel_param));
    if(channel != RC_STICK_F)
      return pwm > get_param_int(rc_ptr->center_param) + get_param_int(rc_ptr->range_param)/2 - get_param_int(PARAM_ARM_THRESHOLD);
    else
      return pwm > get_param_int(rc_ptr->bottom_param) + get_param_int(rc_ptr->range_param) - get_param_int(PARAM_ARM_THRESHOLD);
  }
  return false;
}

static void interpret_command_values()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        rc_stick_config_t* chan = &(sticks[i]);
        int16_t pwm = pwm_read(get_param_int(chan->channel_param));

        // If this is the throttle channel, we need to go from 0.0 to 1.0
        // Otherwise, we need to go from -1.0 to 1.0
        if(i == RC_STICK_F)
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
            int16_t deviation = pwm_read(get_param_int(sticks[i].channel_param)) - get_param_int(sticks[i].center_param);
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
    static uint32_t last_rc_receive_time = 0;
    // If the calibrate_rc flag is set, perform a calibration (blocking)
    if (_calibrate_rc)
    {
        calibrate_rc();
    }

    uint32_t now = clock_millis();

    // if it has been more than 20ms then look for new RC values and parse them
    if (now - last_rc_receive_time < 20)
    {
        return false;
    }
    last_rc_receive_time = now;

    // Determine whether we are in ANGLE, PASSTHROUGH, ALTIIUDE or RATE mode
    interpret_command_type();

    // Interpret PWM Values from RC
    interpret_command_values();

    // Set flags for attitude channels
    if (rc_switch(get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL)) || sticks_deviated(now))
    {
        // Pilot is in full control
        _rc_control.x.active = _rc_control.y.active = _rc_control.z.active = true;
    }
    else
    {
        // Give computer control
        _rc_control.x.active = _rc_control.x.active = _rc_control.x.active = false;
    }

    // Set flags for throttle channel
    if (rc_switch(get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL)))
    {
        // RC Pilot is in control
        _rc_control.F.active = true;
    }
    else
    {
        // Onboard computer has control - min throttle Checking will be done in mux and in the controller.
        _rc_control.F.active = false;
    }

    // Signal to the mux that we need to compute a new combined command
    _new_command = true;
    return true;
}

void calibrate_rc()
{
    if (_armed_state & ARMED || _armed_state & FAILSAFE)
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
