#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "mavlink_util.h"

void mavlink_send_named_value_int(const char *const name, int32_t value)
{
  mavlink_msg_named_value_int_send(MAVLINK_COMM_0, clock_millis(), name, value);
}

void mavlink_send_named_value_float(const char *const name, float value)
{
  mavlink_msg_named_value_float_send(MAVLINK_COMM_0, clock_millis(), name, value);
}

void mavlink_send_named_command_struct(const char *const name, control_t command_struct)
{
  uint8_t control_mode;
  if(command_struct.x.type == RATE && command_struct.y.type == RATE)
  {
    control_mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  }
  else if(command_struct.x.type == ANGLE && command_struct.y.type == ANGLE)
  {
    if(command_struct.x.type == ALTITUDE)
    {
      control_mode = MODE_ROLL_PITCH_YAWRATE_ALTITUDE;
    }
    else
    {
      control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    }
  }
  else
  {
    control_mode = MODE_PASS_THROUGH;
  }
  uint8_t ignore = !(command_struct.x.active) ||
                   !(command_struct.y.active) << 1 ||
                   !(command_struct.z.active) << 2 ||
                   !(command_struct.F.active) << 3;
  mavlink_msg_named_command_struct_send(MAVLINK_COMM_0, name,
                                        control_mode,
                                        ignore,
                                        command_struct.x.value,
                                        command_struct.y.value,
                                        command_struct.z.value,
                                        command_struct.F.value);
}
