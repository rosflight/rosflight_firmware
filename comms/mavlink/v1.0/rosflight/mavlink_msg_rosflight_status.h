// MESSAGE ROSFLIGHT_STATUS PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_STATUS 191

typedef struct __mavlink_rosflight_status_t
{
 int16_t num_errors; /*< */
 int16_t loop_time_us; /*< */
 uint8_t armed; /*< */
 uint8_t failsafe; /*< */
 uint8_t rc_override; /*< */
 uint8_t offboard; /*< */
 uint8_t error_code; /*< */
 uint8_t control_mode; /*< */
} mavlink_rosflight_status_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN 10
#define MAVLINK_MSG_ID_191_LEN 10

#define MAVLINK_MSG_ID_ROSFLIGHT_STATUS_CRC 183
#define MAVLINK_MSG_ID_191_CRC 183



#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_STATUS { \
	"ROSFLIGHT_STATUS", \
	8, \
	{  { "num_errors", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_rosflight_status_t, num_errors) }, \
         { "loop_time_us", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_rosflight_status_t, loop_time_us) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_rosflight_status_t, armed) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_rosflight_status_t, failsafe) }, \
         { "rc_override", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_rosflight_status_t, rc_override) }, \
         { "offboard", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_rosflight_status_t, offboard) }, \
         { "error_code", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_rosflight_status_t, error_code) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_rosflight_status_t, control_mode) }, \
         } \
}


/**
 * @brief Pack a rosflight_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param armed 
 * @param failsafe 
 * @param rc_override 
 * @param offboard 
 * @param error_code 
 * @param control_mode 
 * @param num_errors 
 * @param loop_time_us 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t armed, uint8_t failsafe, uint8_t rc_override, uint8_t offboard, uint8_t error_code, uint8_t control_mode, int16_t num_errors, int16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN];
	_mav_put_int16_t(buf, 0, num_errors);
	_mav_put_int16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, armed);
	_mav_put_uint8_t(buf, 5, failsafe);
	_mav_put_uint8_t(buf, 6, rc_override);
	_mav_put_uint8_t(buf, 7, offboard);
	_mav_put_uint8_t(buf, 8, error_code);
	_mav_put_uint8_t(buf, 9, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#else
	mavlink_rosflight_status_t packet;
	packet.num_errors = num_errors;
	packet.loop_time_us = loop_time_us;
	packet.armed = armed;
	packet.failsafe = failsafe;
	packet.rc_override = rc_override;
	packet.offboard = offboard;
	packet.error_code = error_code;
	packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a rosflight_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param armed 
 * @param failsafe 
 * @param rc_override 
 * @param offboard 
 * @param error_code 
 * @param control_mode 
 * @param num_errors 
 * @param loop_time_us 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t armed,uint8_t failsafe,uint8_t rc_override,uint8_t offboard,uint8_t error_code,uint8_t control_mode,int16_t num_errors,int16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN];
	_mav_put_int16_t(buf, 0, num_errors);
	_mav_put_int16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, armed);
	_mav_put_uint8_t(buf, 5, failsafe);
	_mav_put_uint8_t(buf, 6, rc_override);
	_mav_put_uint8_t(buf, 7, offboard);
	_mav_put_uint8_t(buf, 8, error_code);
	_mav_put_uint8_t(buf, 9, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#else
	mavlink_rosflight_status_t packet;
	packet.num_errors = num_errors;
	packet.loop_time_us = loop_time_us;
	packet.armed = armed;
	packet.failsafe = failsafe;
	packet.rc_override = rc_override;
	packet.offboard = offboard;
	packet.error_code = error_code;
	packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif
}

/**
 * @brief Encode a rosflight_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_status_t* rosflight_status)
{
	return mavlink_msg_rosflight_status_pack(system_id, component_id, msg, rosflight_status->armed, rosflight_status->failsafe, rosflight_status->rc_override, rosflight_status->offboard, rosflight_status->error_code, rosflight_status->control_mode, rosflight_status->num_errors, rosflight_status->loop_time_us);
}

/**
 * @brief Encode a rosflight_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_status_t* rosflight_status)
{
	return mavlink_msg_rosflight_status_pack_chan(system_id, component_id, chan, msg, rosflight_status->armed, rosflight_status->failsafe, rosflight_status->rc_override, rosflight_status->offboard, rosflight_status->error_code, rosflight_status->control_mode, rosflight_status->num_errors, rosflight_status->loop_time_us);
}

/**
 * @brief Send a rosflight_status message
 * @param chan MAVLink channel to send the message
 *
 * @param armed 
 * @param failsafe 
 * @param rc_override 
 * @param offboard 
 * @param error_code 
 * @param control_mode 
 * @param num_errors 
 * @param loop_time_us 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_status_send(mavlink_channel_t chan, uint8_t armed, uint8_t failsafe, uint8_t rc_override, uint8_t offboard, uint8_t error_code, uint8_t control_mode, int16_t num_errors, int16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN];
	_mav_put_int16_t(buf, 0, num_errors);
	_mav_put_int16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, armed);
	_mav_put_uint8_t(buf, 5, failsafe);
	_mav_put_uint8_t(buf, 6, rc_override);
	_mav_put_uint8_t(buf, 7, offboard);
	_mav_put_uint8_t(buf, 8, error_code);
	_mav_put_uint8_t(buf, 9, control_mode);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif
#else
	mavlink_rosflight_status_t packet;
	packet.num_errors = num_errors;
	packet.loop_time_us = loop_time_us;
	packet.armed = armed;
	packet.failsafe = failsafe;
	packet.rc_override = rc_override;
	packet.offboard = offboard;
	packet.error_code = error_code;
	packet.control_mode = control_mode;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t armed, uint8_t failsafe, uint8_t rc_override, uint8_t offboard, uint8_t error_code, uint8_t control_mode, int16_t num_errors, int16_t loop_time_us)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, num_errors);
	_mav_put_int16_t(buf, 2, loop_time_us);
	_mav_put_uint8_t(buf, 4, armed);
	_mav_put_uint8_t(buf, 5, failsafe);
	_mav_put_uint8_t(buf, 6, rc_override);
	_mav_put_uint8_t(buf, 7, offboard);
	_mav_put_uint8_t(buf, 8, error_code);
	_mav_put_uint8_t(buf, 9, control_mode);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif
#else
	mavlink_rosflight_status_t *packet = (mavlink_rosflight_status_t *)msgbuf;
	packet->num_errors = num_errors;
	packet->loop_time_us = loop_time_us;
	packet->armed = armed;
	packet->failsafe = failsafe;
	packet->rc_override = rc_override;
	packet->offboard = offboard;
	packet->error_code = error_code;
	packet->control_mode = control_mode;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_STATUS UNPACKING


/**
 * @brief Get field armed from rosflight_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_status_get_armed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field failsafe from rosflight_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_status_get_failsafe(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field rc_override from rosflight_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_status_get_rc_override(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field offboard from rosflight_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_status_get_offboard(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field error_code from rosflight_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_status_get_error_code(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field control_mode from rosflight_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_status_get_control_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field num_errors from rosflight_status message
 *
 * @return 
 */
static inline int16_t mavlink_msg_rosflight_status_get_num_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field loop_time_us from rosflight_status message
 *
 * @return 
 */
static inline int16_t mavlink_msg_rosflight_status_get_loop_time_us(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a rosflight_status message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_status_decode(const mavlink_message_t* msg, mavlink_rosflight_status_t* rosflight_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_status->num_errors = mavlink_msg_rosflight_status_get_num_errors(msg);
	rosflight_status->loop_time_us = mavlink_msg_rosflight_status_get_loop_time_us(msg);
	rosflight_status->armed = mavlink_msg_rosflight_status_get_armed(msg);
	rosflight_status->failsafe = mavlink_msg_rosflight_status_get_failsafe(msg);
	rosflight_status->rc_override = mavlink_msg_rosflight_status_get_rc_override(msg);
	rosflight_status->offboard = mavlink_msg_rosflight_status_get_offboard(msg);
	rosflight_status->error_code = mavlink_msg_rosflight_status_get_error_code(msg);
	rosflight_status->control_mode = mavlink_msg_rosflight_status_get_control_mode(msg);
#else
	memcpy(rosflight_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_STATUS_LEN);
#endif
}
