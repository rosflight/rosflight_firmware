// MESSAGE ROSFLIGHT_AUX_CMD PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD 193

typedef struct __mavlink_rosflight_aux_cmd_t
{
 float aux_cmd_array[14]; /*< */
 uint8_t type_array[14]; /*< */
} mavlink_rosflight_aux_cmd_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN 70
#define MAVLINK_MSG_ID_193_LEN 70

#define MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_CRC 1
#define MAVLINK_MSG_ID_193_CRC 1

#define MAVLINK_MSG_ROSFLIGHT_AUX_CMD_FIELD_AUX_CMD_ARRAY_LEN 14
#define MAVLINK_MSG_ROSFLIGHT_AUX_CMD_FIELD_TYPE_ARRAY_LEN 14

#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_AUX_CMD { \
	"ROSFLIGHT_AUX_CMD", \
	2, \
	{  { "aux_cmd_array", NULL, MAVLINK_TYPE_FLOAT, 14, 0, offsetof(mavlink_rosflight_aux_cmd_t, aux_cmd_array) }, \
         { "type_array", NULL, MAVLINK_TYPE_UINT8_T, 14, 56, offsetof(mavlink_rosflight_aux_cmd_t, type_array) }, \
         } \
}


/**
 * @brief Pack a rosflight_aux_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type_array 
 * @param aux_cmd_array 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_aux_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint8_t *type_array, const float *aux_cmd_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN];

	_mav_put_float_array(buf, 0, aux_cmd_array, 14);
	_mav_put_uint8_t_array(buf, 56, type_array, 14);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#else
	mavlink_rosflight_aux_cmd_t packet;

	mav_array_memcpy(packet.aux_cmd_array, aux_cmd_array, sizeof(float)*14);
	mav_array_memcpy(packet.type_array, type_array, sizeof(uint8_t)*14);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif
}

/**
 * @brief Pack a rosflight_aux_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type_array 
 * @param aux_cmd_array 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_aux_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint8_t *type_array,const float *aux_cmd_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN];

	_mav_put_float_array(buf, 0, aux_cmd_array, 14);
	_mav_put_uint8_t_array(buf, 56, type_array, 14);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#else
	mavlink_rosflight_aux_cmd_t packet;

	mav_array_memcpy(packet.aux_cmd_array, aux_cmd_array, sizeof(float)*14);
	mav_array_memcpy(packet.type_array, type_array, sizeof(uint8_t)*14);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif
}

/**
 * @brief Encode a rosflight_aux_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_aux_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_aux_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_aux_cmd_t* rosflight_aux_cmd)
{
	return mavlink_msg_rosflight_aux_cmd_pack(system_id, component_id, msg, rosflight_aux_cmd->type_array, rosflight_aux_cmd->aux_cmd_array);
}

/**
 * @brief Encode a rosflight_aux_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_aux_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_aux_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_aux_cmd_t* rosflight_aux_cmd)
{
	return mavlink_msg_rosflight_aux_cmd_pack_chan(system_id, component_id, chan, msg, rosflight_aux_cmd->type_array, rosflight_aux_cmd->aux_cmd_array);
}

/**
 * @brief Send a rosflight_aux_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param type_array 
 * @param aux_cmd_array 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_aux_cmd_send(mavlink_channel_t chan, const uint8_t *type_array, const float *aux_cmd_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN];

	_mav_put_float_array(buf, 0, aux_cmd_array, 14);
	_mav_put_uint8_t_array(buf, 56, type_array, 14);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, buf, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, buf, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif
#else
	mavlink_rosflight_aux_cmd_t packet;

	mav_array_memcpy(packet.aux_cmd_array, aux_cmd_array, sizeof(float)*14);
	mav_array_memcpy(packet.type_array, type_array, sizeof(uint8_t)*14);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_aux_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *type_array, const float *aux_cmd_array)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_float_array(buf, 0, aux_cmd_array, 14);
	_mav_put_uint8_t_array(buf, 56, type_array, 14);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, buf, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, buf, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif
#else
	mavlink_rosflight_aux_cmd_t *packet = (mavlink_rosflight_aux_cmd_t *)msgbuf;

	mav_array_memcpy(packet->aux_cmd_array, aux_cmd_array, sizeof(float)*14);
	mav_array_memcpy(packet->type_array, type_array, sizeof(uint8_t)*14);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_AUX_CMD UNPACKING


/**
 * @brief Get field type_array from rosflight_aux_cmd message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_rosflight_aux_cmd_get_type_array(const mavlink_message_t* msg, uint8_t *type_array)
{
	return _MAV_RETURN_uint8_t_array(msg, type_array, 14,  56);
}

/**
 * @brief Get field aux_cmd_array from rosflight_aux_cmd message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_rosflight_aux_cmd_get_aux_cmd_array(const mavlink_message_t* msg, float *aux_cmd_array)
{
	return _MAV_RETURN_float_array(msg, aux_cmd_array, 14,  0);
}

/**
 * @brief Decode a rosflight_aux_cmd message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_aux_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_aux_cmd_decode(const mavlink_message_t* msg, mavlink_rosflight_aux_cmd_t* rosflight_aux_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_rosflight_aux_cmd_get_aux_cmd_array(msg, rosflight_aux_cmd->aux_cmd_array);
	mavlink_msg_rosflight_aux_cmd_get_type_array(msg, rosflight_aux_cmd->type_array);
#else
	memcpy(rosflight_aux_cmd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD_LEN);
#endif
}
