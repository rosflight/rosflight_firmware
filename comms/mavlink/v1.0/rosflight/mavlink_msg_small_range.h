// MESSAGE SMALL_RANGE PACKING

#define MAVLINK_MSG_ID_SMALL_RANGE 187

typedef struct __mavlink_small_range_t
{
 float range; /*< Range Measurement (m)*/
 float max_range; /*< Max Range (m)*/
 float min_range; /*< Min Range (m)*/
 uint8_t type; /*< Sensor type*/
} mavlink_small_range_t;

#define MAVLINK_MSG_ID_SMALL_RANGE_LEN 13
#define MAVLINK_MSG_ID_187_LEN 13

#define MAVLINK_MSG_ID_SMALL_RANGE_CRC 60
#define MAVLINK_MSG_ID_187_CRC 60



#define MAVLINK_MESSAGE_INFO_SMALL_RANGE { \
	"SMALL_RANGE", \
	4, \
	{  { "range", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_small_range_t, range) }, \
         { "max_range", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_small_range_t, max_range) }, \
         { "min_range", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_small_range_t, min_range) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_small_range_t, type) }, \
         } \
}


/**
 * @brief Pack a small_range message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Sensor type
 * @param range Range Measurement (m)
 * @param max_range Max Range (m)
 * @param min_range Min Range (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_range_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, float range, float max_range, float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_RANGE_LEN];
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);
	_mav_put_uint8_t(buf, 12, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#else
	mavlink_small_range_t packet;
	packet.range = range;
	packet.max_range = max_range;
	packet.min_range = min_range;
	packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SMALL_RANGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMALL_RANGE_LEN, MAVLINK_MSG_ID_SMALL_RANGE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif
}

/**
 * @brief Pack a small_range message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Sensor type
 * @param range Range Measurement (m)
 * @param max_range Max Range (m)
 * @param min_range Min Range (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_range_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,float range,float max_range,float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_RANGE_LEN];
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);
	_mav_put_uint8_t(buf, 12, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#else
	mavlink_small_range_t packet;
	packet.range = range;
	packet.max_range = max_range;
	packet.min_range = min_range;
	packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SMALL_RANGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMALL_RANGE_LEN, MAVLINK_MSG_ID_SMALL_RANGE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif
}

/**
 * @brief Encode a small_range struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param small_range C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_range_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_small_range_t* small_range)
{
	return mavlink_msg_small_range_pack(system_id, component_id, msg, small_range->type, small_range->range, small_range->max_range, small_range->min_range);
}

/**
 * @brief Encode a small_range struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param small_range C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_range_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_small_range_t* small_range)
{
	return mavlink_msg_small_range_pack_chan(system_id, component_id, chan, msg, small_range->type, small_range->range, small_range->max_range, small_range->min_range);
}

/**
 * @brief Send a small_range message
 * @param chan MAVLink channel to send the message
 *
 * @param type Sensor type
 * @param range Range Measurement (m)
 * @param max_range Max Range (m)
 * @param min_range Min Range (m)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_small_range_send(mavlink_channel_t chan, uint8_t type, float range, float max_range, float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_RANGE_LEN];
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);
	_mav_put_uint8_t(buf, 12, type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, buf, MAVLINK_MSG_ID_SMALL_RANGE_LEN, MAVLINK_MSG_ID_SMALL_RANGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, buf, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif
#else
	mavlink_small_range_t packet;
	packet.range = range;
	packet.max_range = max_range;
	packet.min_range = min_range;
	packet.type = type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, (const char *)&packet, MAVLINK_MSG_ID_SMALL_RANGE_LEN, MAVLINK_MSG_ID_SMALL_RANGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, (const char *)&packet, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SMALL_RANGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_small_range_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, float range, float max_range, float min_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, range);
	_mav_put_float(buf, 4, max_range);
	_mav_put_float(buf, 8, min_range);
	_mav_put_uint8_t(buf, 12, type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, buf, MAVLINK_MSG_ID_SMALL_RANGE_LEN, MAVLINK_MSG_ID_SMALL_RANGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, buf, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif
#else
	mavlink_small_range_t *packet = (mavlink_small_range_t *)msgbuf;
	packet->range = range;
	packet->max_range = max_range;
	packet->min_range = min_range;
	packet->type = type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, (const char *)packet, MAVLINK_MSG_ID_SMALL_RANGE_LEN, MAVLINK_MSG_ID_SMALL_RANGE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_RANGE, (const char *)packet, MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SMALL_RANGE UNPACKING


/**
 * @brief Get field type from small_range message
 *
 * @return Sensor type
 */
static inline uint8_t mavlink_msg_small_range_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field range from small_range message
 *
 * @return Range Measurement (m)
 */
static inline float mavlink_msg_small_range_get_range(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field max_range from small_range message
 *
 * @return Max Range (m)
 */
static inline float mavlink_msg_small_range_get_max_range(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field min_range from small_range message
 *
 * @return Min Range (m)
 */
static inline float mavlink_msg_small_range_get_min_range(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a small_range message into a struct
 *
 * @param msg The message to decode
 * @param small_range C-struct to decode the message contents into
 */
static inline void mavlink_msg_small_range_decode(const mavlink_message_t* msg, mavlink_small_range_t* small_range)
{
#if MAVLINK_NEED_BYTE_SWAP
	small_range->range = mavlink_msg_small_range_get_range(msg);
	small_range->max_range = mavlink_msg_small_range_get_max_range(msg);
	small_range->min_range = mavlink_msg_small_range_get_min_range(msg);
	small_range->type = mavlink_msg_small_range_get_type(msg);
#else
	memcpy(small_range, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SMALL_RANGE_LEN);
#endif
}
