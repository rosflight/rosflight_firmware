#include "rc.h"

namespace rosflight {

void RC::init_rc(Arming_FSM* _fsm, Board* _board, Params* _params, Mux* _mux)
{
  fsm = _fsm;
  board = _board;
  params = _params;
  mux = _mux;

  _calibrate_rc = false;
  mux->_rc_control.x.type = Mux::ANGLE;
  mux->_rc_control.y.type = Mux::ANGLE;
  mux->_rc_control.z.type = Mux::RATE;
  mux->_rc_control.F.type = Mux::THROTTLE;

  mux->_rc_control.x.value = 0;
  mux->_rc_control.y.value = 0;
  mux->_rc_control.z.value = 0;
  mux->_rc_control.F.value = 0;

  mux->_offboard_control.x.active = false;
  mux->_offboard_control.y.active = false;
  mux->_offboard_control.z.active = false;
  mux->_offboard_control.F.active = false;

  init_switches();

  channels[RC_X].channel_param = PARAM_RC_X_CHANNEL;
  channels[RC_X].max_angle_param = PARAM_MAX_ROLL_ANGLE;
  channels[RC_X].max_rate_param = PARAM_MAX_ROLL_RATE;
  channels[RC_X].bottom_param = PARAMS_COUNT;
  channels[RC_X].center_param = PARAM_RC_X_CENTER;
  channels[RC_X].range_param = PARAM_RC_X_RANGE;
  channels[RC_X].control_channel_ptr = &(mux->_rc_control.x);

  channels[RC_Y].channel_param = PARAM_RC_Y_CHANNEL;
  channels[RC_Y].max_angle_param = PARAM_MAX_PITCH_ANGLE;
  channels[RC_Y].max_rate_param = PARAM_MAX_PITCH_RATE;
  channels[RC_Y].bottom_param = PARAMS_COUNT;
  channels[RC_Y].center_param = PARAM_RC_Y_CENTER;
  channels[RC_Y].range_param = PARAM_RC_Y_RANGE;
  channels[RC_Y].control_channel_ptr = &(mux->_rc_control.y);

  channels[RC_Z].channel_param = PARAM_RC_Z_CHANNEL;
  channels[RC_Z].max_angle_param = PARAMS_COUNT;
  channels[RC_Z].max_rate_param = PARAM_MAX_YAW_RATE;
  channels[RC_Z].bottom_param = PARAMS_COUNT;
  channels[RC_Z].center_param = PARAM_RC_Z_CENTER;
  channels[RC_Z].range_param = PARAM_RC_Z_RANGE;
  channels[RC_Z].control_channel_ptr = &(mux->_rc_control.z);

  channels[RC_F].channel_param = PARAM_RC_F_CHANNEL;
  channels[RC_F].max_angle_param = PARAMS_COUNT;
  channels[RC_F].max_rate_param = PARAM_MAX_ROLL_RATE;
  channels[RC_F].bottom_param = PARAM_RC_F_BOTTOM;
  channels[RC_F].center_param = PARAMS_COUNT;
  channels[RC_F].range_param = PARAM_RC_F_RANGE;
  channels[RC_F].control_channel_ptr = &(mux->_rc_control.F);
}


void RC::init_switches()
{
  // Make sure that parameters for switch channels are correct
  uint16_t channel_parameters[3] = {PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL,
                                    PARAM_RC_THROTTLE_OVERRIDE_CHANNEL,
                                    PARAM_RC_ATT_CONTROL_TYPE_CHANNEL};

  // Make sure that channel numbers are specified correctly (between 4 and 8)
  for (uint8_t i = 0; i < 3; i++)
  {
    uint32_t channel_index = static_cast<uint32_t>(params->get_param_int(channel_parameters[i]));
    if (channel_index > 8 || channel_index < 4)
    {
      //            mavlink_log_error("incorrect channel specification for");
      //            mavlink_log_error("switch parameter s", get_param_name((param_id_t)channel_parameters[i]));
      //            mavlink_log_error("setting to channel 8");
      params->set_param_int(channel_parameters[i], 8);
    }
  }

  // Set up the switch structs
  switches[0].channel = 4;
  switches[0].direction = params->get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
  switches[1].channel = 5;
  switches[1].direction = params->get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
  switches[2].channel = 6;
  switches[2].direction = params->get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
  switches[3].channel = 7;
  switches[3].direction = params->get_param_int(PARAM_RC_SWITCH_8_DIRECTION);
}

bool RC::rc_switch(int16_t channel)
{
  if (channel < 4 || channel > 8)
  {
    return false;
  }
  if (switches[channel - 4].direction < 0)
  {
    return board->pwm_read(channel) < 1500;
  }
  else
  {
    return board->pwm_read(channel) > 1500;
  }
}

bool RC::rc_low(int16_t channel)
{
  if (channel < 4)
  {
    rc_channel_t* rc_ptr = &(channels[channel]);
    int16_t pwm = board->pwm_read(params->get_param_int(rc_ptr->channel_param));
    if(channel != RC_F)
      return pwm < params->get_param_int(rc_ptr->center_param) - params->get_param_int(rc_ptr->range_param)/2 +params->get_param_int(PARAM_ARM_THRESHOLD);
    else
      return pwm < params->get_param_int(rc_ptr->bottom_param) + params->get_param_int(PARAM_ARM_THRESHOLD);
  }
}

bool RC::rc_high(int16_t channel)
{
  if(channel < 4)
  {
    rc_channel_t* rc_ptr = &(channels[channel]);
    int16_t pwm = board->pwm_read(params->get_param_int(rc_ptr->channel_param));
    if(channel != RC_F)
      return pwm > params->get_param_int(rc_ptr->center_param) + params->get_param_int(rc_ptr->range_param)/2 - params->get_param_int(PARAM_ARM_THRESHOLD);
    else
      return pwm > params->get_param_int(rc_ptr->bottom_param) + params->get_param_int(rc_ptr->range_param) - params->get_param_int(PARAM_ARM_THRESHOLD);
  }
  return false;
}

void RC::interpret_command_values()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    rc_channel_t* chan = &(channels[i]);
    int16_t pwm = board->pwm_read(params->get_param_int(chan->channel_param));

    // If this is the throttle channel, we need to go from 0.0 to 1.0
    // Otherwise, we need to go from -1.0 to 1.0
    if(i == RC_F)
    {
      chan->control_channel_ptr->value = (float)(pwm - params->get_param_int(chan->bottom_param)) /
          (float)(params->get_param_int(chan->range_param));
    }
    else
    {
      chan->control_channel_ptr->value = 2.0*(float)(pwm - params->get_param_int(chan->center_param))/
          (float)(params->get_param_int(chan->range_param));
    }

    // Now, check the mode and convert the normalized value to the appropriate units
    switch(chan->control_channel_ptr->type)
    {
    case Mux::RATE:
      chan->control_channel_ptr->value *= params->get_param_float(chan->max_rate_param);
      break;
    case Mux::ANGLE:
      chan->control_channel_ptr->value *= params->get_param_float(chan->max_angle_param);
      break;
    default: // default and altitude modes, leave as normalized
      break;
    }
  }
}

void RC::interpret_command_type()
{
  // Figure out the desired control type from the switches and params
  if (params->get_param_int(PARAM_FIXED_WING))
  {
    // for using fixedwings
    mux->_rc_control.x.type = mux->_rc_control.y.type = mux->_rc_control.z.type = Mux::PASSTHROUGH;
    mux->_rc_control.F.type = Mux::THROTTLE;
  }
  else
  {
    mux->_rc_control.x.type = mux->_rc_control.y.type = rc_switch(params->get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)) ? Mux::ANGLE : Mux::RATE;
    mux->_rc_control.z.type = Mux::RATE;
    mux->_rc_control.F.type = rc_switch(params->get_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL)) ? Mux::ALTITUDE : Mux::THROTTLE;
  }
}

bool RC::sticks_deviated(uint32_t now_ms)
{
  static uint32_t time_of_last_stick_deviation = 0;

  // If we are in the lag time, return true;
  if(now_ms - time_of_last_stick_deviation < (uint64_t)(params->get_param_int(PARAM_OVERRIDE_LAG_TIME)))
  {
    return true;
  }
  else
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      int16_t deviation = board->pwm_read(params->get_param_int(channels[i].channel_param)) - params->get_param_int(channels[i].center_param);
      if (abs(deviation) > params->get_param_int(PARAM_RC_OVERRIDE_DEVIATION))
      {
        time_of_last_stick_deviation = now_ms;
        return true;
      }
    }
    // None of the three sticks are deviated more than the override threshold
    return false;
  }
}


bool RC::receive_rc()
{
  static uint32_t last_rc_receive_time = 0;
  // If the calibrate_rc flag is set, perform a calibration (blocking)
  if (_calibrate_rc)
  {
    calibrate_rc();
  }

  uint32_t now = board->clock_millis();

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
  if (rc_switch(params->get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL)) || sticks_deviated(now))
  {
    // Pilot is in full control
    mux->_rc_control.x.active = mux->_rc_control.y.active = mux->_rc_control.z.active = true;
  }
  else
  {
    // Give computer control
    mux->_rc_control.x.active = mux->_rc_control.x.active = mux->_rc_control.x.active = false;
  }

  // Set flags for throttle channel
  if (rc_switch(params->get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL)))
  {
    // RC Pilot is in control
    mux->_rc_control.F.active = true;
  }
  else
  {
    // Onboard computer has control - min throttle Checking will be done in mux and in the controller.
    mux->_rc_control.F.active = false;
  }

  // Signal to the mux that we need to compute a new combined command
  mux->_new_command = true;
  return true;
}

void RC::calibrate_rc()
{
  if (fsm->_armed_state == ARMED || fsm->_armed_state == DISARMED_FAILSAFE || fsm->_armed_state == ARMED_FAILSAFE)
  {
    //    mavlink_log_error("Cannot calibrate RC when FCU is armed or in failsafe", NULL);
  }
  else
  {
    // Calibrate Extents of RC Transmitter
    //    mavlink_log_warning("Calibrating RC, move sticks to full extents", NULL);
    //    mavlink_log_warning("in the next 10s", NULL);
    uint32_t start = board->clock_millis();
    static int32_t max[4] = {0, 0, 0, 0};
    static int32_t min[4] = {10000, 10000, 10000, 10000};
    while(board->clock_millis() - start < 1e4)
    {
      for (int16_t i = 0; i < 4; i++)
      {
        int32_t read_value = (int32_t)board->pwm_read(i);
        if(read_value > max[i])
        {
          max[i] = read_value;
        }
        if (read_value < min[i])
        {
          min[i] = read_value;
        }
      }
      board->clock_delay(10);
    }
    params->set_param_int(PARAM_RC_X_RANGE, max[params->get_param_int(PARAM_RC_X_CHANNEL)] - min[params->get_param_int(PARAM_RC_X_CHANNEL)]);
    params->set_param_int(PARAM_RC_Y_RANGE, max[params->get_param_int(PARAM_RC_Y_CHANNEL)] - min[params->get_param_int(PARAM_RC_Y_CHANNEL)]);
    params->set_param_int(PARAM_RC_Z_RANGE, max[params->get_param_int(PARAM_RC_Z_CHANNEL)] - min[params->get_param_int(PARAM_RC_Z_CHANNEL)]);
    params->set_param_int(PARAM_RC_F_RANGE, max[params->get_param_int(PARAM_RC_F_CHANNEL)] - min[params->get_param_int(PARAM_RC_F_CHANNEL)]);

    // Calibrate Trimmed Centers
    //    mavlink_log_warning("Calibrating RC, leave sticks at center", NULL);
    //    mavlink_log_warning("and throttle low for next 10 seconds", NULL);
    board->clock_delay(5000);
    start = board->clock_millis();
    static int32_t sum[4] = {0, 0, 0, 0};
    static int32_t count[4] = {0, 0, 0, 0};

    while(board->clock_millis() - start < 5e3)
    {
      for (int16_t i = 0; i < 4; i++)
      {
        int32_t read_value = (int32_t)board->pwm_read(i);
        sum[i] = sum[i] + read_value;
        count[i] = count[i] + 1;
      }
      board->clock_delay(20); // RC is updated at 50 Hz
    }

    params->set_param_int(PARAM_RC_X_CENTER, sum[params->get_param_int(PARAM_RC_X_CHANNEL)]/count[params->get_param_int(PARAM_RC_X_CHANNEL)]);
    params->set_param_int(PARAM_RC_Y_CENTER, sum[params->get_param_int(PARAM_RC_Y_CHANNEL)]/count[params->get_param_int(PARAM_RC_Y_CHANNEL)]);
    params->set_param_int(PARAM_RC_Z_CENTER, sum[params->get_param_int(PARAM_RC_Z_CHANNEL)]/count[params->get_param_int(PARAM_RC_Z_CHANNEL)]);
    params->set_param_int(PARAM_RC_F_BOTTOM, sum[params->get_param_int(PARAM_RC_F_CHANNEL)]/count[params->get_param_int(PARAM_RC_F_CHANNEL)]);
    params->write_params();
    //    mavlink_log_warning("Completed RC calibration", NULL);
  }
  _calibrate_rc = false;
}
}
