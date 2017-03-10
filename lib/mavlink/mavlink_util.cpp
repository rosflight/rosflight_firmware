#include "controller.h"

#include "mavlink.h"

namespace rosflight{

void Mavlink::mavlink_send_named_value_int(const char *const name, int32_t value)
{
  mavlink_message_t msg;
  mavlink_msg_named_value_int_pack(sysid, compid, &msg, board_->clock_millis(), name, value);
  send_message(msg);
}

void Mavlink::mavlink_send_named_value_float(const char *const name, float value)
{
  mavlink_message_t msg;
  mavlink_msg_named_value_float_pack(sysid, compid, &msg, board_->clock_millis(), name, value);
  send_message(msg);
}

void Mavlink::mavlink_send_named_command_struct(const char *const name, control_t command_struct)
{
  uint8_t control_mode;
  if (command_struct.x.type == RATE && command_struct.y.type == RATE)
  {
    control_mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  }
  else if (command_struct.x.type == ANGLE && command_struct.y.type == ANGLE)
  {
    control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }
  else
  {
    control_mode = MODE_PASS_THROUGH;
  }
  uint8_t ignore = !(command_struct.x.active) ||
                   !(command_struct.y.active) << 1 ||
                   !(command_struct.z.active) << 2 ||
                   !(command_struct.F.active) << 3;
  mavlink_message_t msg;
  mavlink_msg_named_command_struct_pack(sysid, compid, &msg, name,
                                        control_mode,
                                        ignore,
                                        command_struct.x.value,
                                        command_struct.y.value,
                                        command_struct.z.value,
                                        command_struct.F.value);
  send_message(msg);
}

}
