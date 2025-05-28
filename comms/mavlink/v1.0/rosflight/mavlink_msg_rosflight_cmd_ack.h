// MESSAGE ROSFLIGHT_CMD_ACK PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK 189

typedef struct __mavlink_rosflight_cmd_ack_t
{
 uint8_t command; /*< */
 uint8_t success; /*< */
} mavlink_rosflight_cmd_ack_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN 2
#define MAVLINK_MSG_ID_189_LEN 2

#define MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_CRC 113
#define MAVLINK_MSG_ID_189_CRC 113



#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_CMD_ACK { \
	"ROSFLIGHT_CMD_ACK", \
	2, \
	{  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rosflight_cmd_ack_t, command) }, \
         { "success", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rosflight_cmd_ack_t, success) }, \
         } \
}


/**
 * @brief Pack a rosflight_cmd_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command 
 * @param success 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_cmd_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t command, uint8_t success)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN];
	_mav_put_uint8_t(buf, 0, command);
	_mav_put_uint8_t(buf, 1, success);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#else
	mavlink_rosflight_cmd_ack_t packet;
	packet.command = command;
	packet.success = success;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif
}

/**
 * @brief Pack a rosflight_cmd_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command 
 * @param success 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_cmd_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t command,uint8_t success)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN];
	_mav_put_uint8_t(buf, 0, command);
	_mav_put_uint8_t(buf, 1, success);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#else
	mavlink_rosflight_cmd_ack_t packet;
	packet.command = command;
	packet.success = success;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif
}

/**
 * @brief Encode a rosflight_cmd_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_cmd_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_cmd_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_cmd_ack_t* rosflight_cmd_ack)
{
	return mavlink_msg_rosflight_cmd_ack_pack(system_id, component_id, msg, rosflight_cmd_ack->command, rosflight_cmd_ack->success);
}

/**
 * @brief Encode a rosflight_cmd_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_cmd_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_cmd_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_cmd_ack_t* rosflight_cmd_ack)
{
	return mavlink_msg_rosflight_cmd_ack_pack_chan(system_id, component_id, chan, msg, rosflight_cmd_ack->command, rosflight_cmd_ack->success);
}

/**
 * @brief Send a rosflight_cmd_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param command 
 * @param success 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_cmd_ack_send(mavlink_channel_t chan, uint8_t command, uint8_t success)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN];
	_mav_put_uint8_t(buf, 0, command);
	_mav_put_uint8_t(buf, 1, success);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif
#else
	mavlink_rosflight_cmd_ack_t packet;
	packet.command = command;
	packet.success = success;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_cmd_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command, uint8_t success)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, command);
	_mav_put_uint8_t(buf, 1, success);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif
#else
	mavlink_rosflight_cmd_ack_t *packet = (mavlink_rosflight_cmd_ack_t *)msgbuf;
	packet->command = command;
	packet->success = success;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_CMD_ACK UNPACKING


/**
 * @brief Get field command from rosflight_cmd_ack message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_cmd_ack_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field success from rosflight_cmd_ack message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_cmd_ack_get_success(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a rosflight_cmd_ack message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_cmd_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_cmd_ack_decode(const mavlink_message_t* msg, mavlink_rosflight_cmd_ack_t* rosflight_cmd_ack)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_cmd_ack->command = mavlink_msg_rosflight_cmd_ack_get_command(msg);
	rosflight_cmd_ack->success = mavlink_msg_rosflight_cmd_ack_get_success(msg);
#else
	memcpy(rosflight_cmd_ack, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_CMD_ACK_LEN);
#endif
}
