// MESSAGE EXTERNAL_ATTITUDE PACKING

#define MAVLINK_MSG_ID_EXTERNAL_ATTITUDE 195

typedef struct __mavlink_external_attitude_t
{
 float qw; /*< Quaternion scalar value*/
 float qx; /*< Quaternion x value*/
 float qy; /*< Quaternion y value*/
 float qz; /*< Quaternion z value*/
} mavlink_external_attitude_t;

#define MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN 16
#define MAVLINK_MSG_ID_195_LEN 16

#define MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_CRC 65
#define MAVLINK_MSG_ID_195_CRC 65



#define MAVLINK_MESSAGE_INFO_EXTERNAL_ATTITUDE { \
	"EXTERNAL_ATTITUDE", \
	4, \
	{  { "qw", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_external_attitude_t, qw) }, \
         { "qx", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_external_attitude_t, qx) }, \
         { "qy", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_external_attitude_t, qy) }, \
         { "qz", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_external_attitude_t, qz) }, \
         } \
}


/**
 * @brief Pack a external_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param qw Quaternion scalar value
 * @param qx Quaternion x value
 * @param qy Quaternion y value
 * @param qz Quaternion z value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_external_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float qw, float qx, float qy, float qz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN];
	_mav_put_float(buf, 0, qw);
	_mav_put_float(buf, 4, qx);
	_mav_put_float(buf, 8, qy);
	_mav_put_float(buf, 12, qz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#else
	mavlink_external_attitude_t packet;
	packet.qw = qw;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EXTERNAL_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a external_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param qw Quaternion scalar value
 * @param qx Quaternion x value
 * @param qy Quaternion y value
 * @param qz Quaternion z value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_external_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float qw,float qx,float qy,float qz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN];
	_mav_put_float(buf, 0, qw);
	_mav_put_float(buf, 4, qx);
	_mav_put_float(buf, 8, qy);
	_mav_put_float(buf, 12, qz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#else
	mavlink_external_attitude_t packet;
	packet.qw = qw;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EXTERNAL_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif
}

/**
 * @brief Encode a external_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param external_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_external_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_external_attitude_t* external_attitude)
{
	return mavlink_msg_external_attitude_pack(system_id, component_id, msg, external_attitude->qw, external_attitude->qx, external_attitude->qy, external_attitude->qz);
}

/**
 * @brief Encode a external_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param external_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_external_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_external_attitude_t* external_attitude)
{
	return mavlink_msg_external_attitude_pack_chan(system_id, component_id, chan, msg, external_attitude->qw, external_attitude->qx, external_attitude->qy, external_attitude->qz);
}

/**
 * @brief Send a external_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param qw Quaternion scalar value
 * @param qx Quaternion x value
 * @param qy Quaternion y value
 * @param qz Quaternion z value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_external_attitude_send(mavlink_channel_t chan, float qw, float qx, float qy, float qz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN];
	_mav_put_float(buf, 0, qw);
	_mav_put_float(buf, 4, qx);
	_mav_put_float(buf, 8, qy);
	_mav_put_float(buf, 12, qz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, buf, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, buf, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif
#else
	mavlink_external_attitude_t packet;
	packet.qw = qw;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_external_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float qw, float qx, float qy, float qz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, qw);
	_mav_put_float(buf, 4, qx);
	_mav_put_float(buf, 8, qy);
	_mav_put_float(buf, 12, qz);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, buf, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, buf, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif
#else
	mavlink_external_attitude_t *packet = (mavlink_external_attitude_t *)msgbuf;
	packet->qw = qw;
	packet->qx = qx;
	packet->qy = qy;
	packet->qz = qz;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE EXTERNAL_ATTITUDE UNPACKING


/**
 * @brief Get field qw from external_attitude message
 *
 * @return Quaternion scalar value
 */
static inline float mavlink_msg_external_attitude_get_qw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field qx from external_attitude message
 *
 * @return Quaternion x value
 */
static inline float mavlink_msg_external_attitude_get_qx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field qy from external_attitude message
 *
 * @return Quaternion y value
 */
static inline float mavlink_msg_external_attitude_get_qy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field qz from external_attitude message
 *
 * @return Quaternion z value
 */
static inline float mavlink_msg_external_attitude_get_qz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a external_attitude message into a struct
 *
 * @param msg The message to decode
 * @param external_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_external_attitude_decode(const mavlink_message_t* msg, mavlink_external_attitude_t* external_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	external_attitude->qw = mavlink_msg_external_attitude_get_qw(msg);
	external_attitude->qx = mavlink_msg_external_attitude_get_qx(msg);
	external_attitude->qy = mavlink_msg_external_attitude_get_qy(msg);
	external_attitude->qz = mavlink_msg_external_attitude_get_qz(msg);
#else
	memcpy(external_attitude, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_EXTERNAL_ATTITUDE_LEN);
#endif
}
