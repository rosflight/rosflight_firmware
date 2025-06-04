// MESSAGE OFFBOARD_CONTROL PACKING

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL 180

typedef struct __mavlink_offboard_control_t
{
 float Qx; /*< Qx control channel interpreted according to mode*/
 float Qy; /*< Qy control channel, interpreted according to mode*/
 float Qz; /*< Qz control channel, interpreted according to mode*/
 float Fx; /*< Fx control channel, interpreted according to mode*/
 float Fy; /*< Fy control channel, interpreted according to mode*/
 float Fz; /*< Fz control channel, interpreted according to mode*/
 uint8_t mode; /*< Offboard control mode, see OFFBOARD_CONTROL_MODE*/
 uint8_t ignore; /*< Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE*/
} mavlink_offboard_control_t;

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN 26
#define MAVLINK_MSG_ID_180_LEN 26

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC 190
#define MAVLINK_MSG_ID_180_CRC 190



#define MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL { \
	"OFFBOARD_CONTROL", \
	8, \
	{  { "Qx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_offboard_control_t, Qx) }, \
         { "Qy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_offboard_control_t, Qy) }, \
         { "Qz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_offboard_control_t, Qz) }, \
         { "Fx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_offboard_control_t, Fx) }, \
         { "Fy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_offboard_control_t, Fy) }, \
         { "Fz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_offboard_control_t, Fz) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_offboard_control_t, mode) }, \
         { "ignore", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_offboard_control_t, ignore) }, \
         } \
}


/**
 * @brief Pack a offboard_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param Qx Qx control channel interpreted according to mode
 * @param Qy Qy control channel, interpreted according to mode
 * @param Qz Qz control channel, interpreted according to mode
 * @param Fx Fx control channel, interpreted according to mode
 * @param Fy Fy control channel, interpreted according to mode
 * @param Fz Fz control channel, interpreted according to mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t mode, uint8_t ignore, float Qx, float Qy, float Qz, float Fx, float Fy, float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
	_mav_put_float(buf, 0, Qx);
	_mav_put_float(buf, 4, Qy);
	_mav_put_float(buf, 8, Qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 24, mode);
	_mav_put_uint8_t(buf, 25, ignore);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
	mavlink_offboard_control_t packet;
	packet.Qx = Qx;
	packet.Qy = Qy;
	packet.Qz = Qz;
	packet.Fx = Fx;
	packet.Fy = Fy;
	packet.Fz = Fz;
	packet.mode = mode;
	packet.ignore = ignore;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a offboard_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param Qx Qx control channel interpreted according to mode
 * @param Qy Qy control channel, interpreted according to mode
 * @param Qz Qz control channel, interpreted according to mode
 * @param Fx Fx control channel, interpreted according to mode
 * @param Fy Fy control channel, interpreted according to mode
 * @param Fz Fz control channel, interpreted according to mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t mode,uint8_t ignore,float Qx,float Qy,float Qz,float Fx,float Fy,float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
	_mav_put_float(buf, 0, Qx);
	_mav_put_float(buf, 4, Qy);
	_mav_put_float(buf, 8, Qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 24, mode);
	_mav_put_uint8_t(buf, 25, ignore);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
	mavlink_offboard_control_t packet;
	packet.Qx = Qx;
	packet.Qy = Qy;
	packet.Qz = Qz;
	packet.Fx = Fx;
	packet.Fy = Fy;
	packet.Fz = Fz;
	packet.mode = mode;
	packet.ignore = ignore;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
}

/**
 * @brief Encode a offboard_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param offboard_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_offboard_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_offboard_control_t* offboard_control)
{
	return mavlink_msg_offboard_control_pack(system_id, component_id, msg, offboard_control->mode, offboard_control->ignore, offboard_control->Qx, offboard_control->Qy, offboard_control->Qz, offboard_control->Fx, offboard_control->Fy, offboard_control->Fz);
}

/**
 * @brief Encode a offboard_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param offboard_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_offboard_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_offboard_control_t* offboard_control)
{
	return mavlink_msg_offboard_control_pack_chan(system_id, component_id, chan, msg, offboard_control->mode, offboard_control->ignore, offboard_control->Qx, offboard_control->Qy, offboard_control->Qz, offboard_control->Fx, offboard_control->Fy, offboard_control->Fz);
}

/**
 * @brief Send a offboard_control message
 * @param chan MAVLink channel to send the message
 *
 * @param mode Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param Qx Qx control channel interpreted according to mode
 * @param Qy Qy control channel, interpreted according to mode
 * @param Qz Qz control channel, interpreted according to mode
 * @param Fx Fx control channel, interpreted according to mode
 * @param Fy Fy control channel, interpreted according to mode
 * @param Fz Fz control channel, interpreted according to mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_offboard_control_send(mavlink_channel_t chan, uint8_t mode, uint8_t ignore, float Qx, float Qy, float Qz, float Fx, float Fy, float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
	_mav_put_float(buf, 0, Qx);
	_mav_put_float(buf, 4, Qy);
	_mav_put_float(buf, 8, Qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 24, mode);
	_mav_put_uint8_t(buf, 25, ignore);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#else
	mavlink_offboard_control_t packet;
	packet.Qx = Qx;
	packet.Qy = Qy;
	packet.Qz = Qz;
	packet.Fx = Fx;
	packet.Fy = Fy;
	packet.Fz = Fz;
	packet.mode = mode;
	packet.ignore = ignore;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_offboard_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode, uint8_t ignore, float Qx, float Qy, float Qz, float Fx, float Fy, float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, Qx);
	_mav_put_float(buf, 4, Qy);
	_mav_put_float(buf, 8, Qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 24, mode);
	_mav_put_uint8_t(buf, 25, ignore);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#else
	mavlink_offboard_control_t *packet = (mavlink_offboard_control_t *)msgbuf;
	packet->Qx = Qx;
	packet->Qy = Qy;
	packet->Qz = Qz;
	packet->Fx = Fx;
	packet->Fy = Fy;
	packet->Fz = Fz;
	packet->mode = mode;
	packet->ignore = ignore;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE OFFBOARD_CONTROL UNPACKING


/**
 * @brief Get field mode from offboard_control message
 *
 * @return Offboard control mode, see OFFBOARD_CONTROL_MODE
 */
static inline uint8_t mavlink_msg_offboard_control_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field ignore from offboard_control message
 *
 * @return Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 */
static inline uint8_t mavlink_msg_offboard_control_get_ignore(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field Qx from offboard_control message
 *
 * @return Qx control channel interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_Qx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Qy from offboard_control message
 *
 * @return Qy control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_Qy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Qz from offboard_control message
 *
 * @return Qz control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_Qz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Fx from offboard_control message
 *
 * @return Fx control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_Fx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field Fy from offboard_control message
 *
 * @return Fy control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_Fy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Fz from offboard_control message
 *
 * @return Fz control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_Fz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a offboard_control message into a struct
 *
 * @param msg The message to decode
 * @param offboard_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_offboard_control_decode(const mavlink_message_t* msg, mavlink_offboard_control_t* offboard_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	offboard_control->Qx = mavlink_msg_offboard_control_get_Qx(msg);
	offboard_control->Qy = mavlink_msg_offboard_control_get_Qy(msg);
	offboard_control->Qz = mavlink_msg_offboard_control_get_Qz(msg);
	offboard_control->Fx = mavlink_msg_offboard_control_get_Fx(msg);
	offboard_control->Fy = mavlink_msg_offboard_control_get_Fy(msg);
	offboard_control->Fz = mavlink_msg_offboard_control_get_Fz(msg);
	offboard_control->mode = mavlink_msg_offboard_control_get_mode(msg);
	offboard_control->ignore = mavlink_msg_offboard_control_get_ignore(msg);
#else
	memcpy(offboard_control, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
}
