// MESSAGE NAMED_COMMAND_STRUCT PACKING

#define MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT 186

typedef struct __mavlink_named_command_struct_t
{
 float qx; /*< Qx value in the command struct*/
 float qy; /*< Qy value in the command struct*/
 float qz; /*< Qz value in the command struct*/
 float Fx; /*< Fx value in the command struct*/
 float Fy; /*< Fy value in the command struct*/
 float Fz; /*< Fz value in the command struct*/
 char name[10]; /*< Name of the command struct*/
 uint8_t type; /*< Type of command struct*/
 uint8_t ignore; /*< Type of command struct*/
} mavlink_named_command_struct_t;

#define MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN 36
#define MAVLINK_MSG_ID_186_LEN 36

#define MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC 221
#define MAVLINK_MSG_ID_186_CRC 221

#define MAVLINK_MSG_NAMED_COMMAND_STRUCT_FIELD_NAME_LEN 10

#define MAVLINK_MESSAGE_INFO_NAMED_COMMAND_STRUCT { \
	"NAMED_COMMAND_STRUCT", \
	9, \
	{  { "qx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_named_command_struct_t, qx) }, \
         { "qy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_named_command_struct_t, qy) }, \
         { "qz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_named_command_struct_t, qz) }, \
         { "Fx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_named_command_struct_t, Fx) }, \
         { "Fy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_named_command_struct_t, Fy) }, \
         { "Fz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_named_command_struct_t, Fz) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 24, offsetof(mavlink_named_command_struct_t, name) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_named_command_struct_t, type) }, \
         { "ignore", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_named_command_struct_t, ignore) }, \
         } \
}


/**
 * @brief Pack a named_command_struct message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name of the command struct
 * @param type Type of command struct
 * @param ignore Type of command struct
 * @param qx Qx value in the command struct
 * @param qy Qy value in the command struct
 * @param qz Qz value in the command struct
 * @param Fx Fx value in the command struct
 * @param Fy Fy value in the command struct
 * @param Fz Fz value in the command struct
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_command_struct_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const char *name, uint8_t type, uint8_t ignore, float qx, float qy, float qz, float Fx, float Fy, float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN];
	_mav_put_float(buf, 0, qx);
	_mav_put_float(buf, 4, qy);
	_mav_put_float(buf, 8, qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 34, type);
	_mav_put_uint8_t(buf, 35, ignore);
	_mav_put_char_array(buf, 24, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#else
	mavlink_named_command_struct_t packet;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;
	packet.Fx = Fx;
	packet.Fy = Fy;
	packet.Fz = Fz;
	packet.type = type;
	packet.ignore = ignore;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
}

/**
 * @brief Pack a named_command_struct message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the command struct
 * @param type Type of command struct
 * @param ignore Type of command struct
 * @param qx Qx value in the command struct
 * @param qy Qy value in the command struct
 * @param qz Qz value in the command struct
 * @param Fx Fx value in the command struct
 * @param Fy Fy value in the command struct
 * @param Fz Fz value in the command struct
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_command_struct_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const char *name,uint8_t type,uint8_t ignore,float qx,float qy,float qz,float Fx,float Fy,float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN];
	_mav_put_float(buf, 0, qx);
	_mav_put_float(buf, 4, qy);
	_mav_put_float(buf, 8, qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 34, type);
	_mav_put_uint8_t(buf, 35, ignore);
	_mav_put_char_array(buf, 24, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#else
	mavlink_named_command_struct_t packet;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;
	packet.Fx = Fx;
	packet.Fy = Fy;
	packet.Fz = Fz;
	packet.type = type;
	packet.ignore = ignore;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
}

/**
 * @brief Encode a named_command_struct struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_command_struct C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_command_struct_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_command_struct_t* named_command_struct)
{
	return mavlink_msg_named_command_struct_pack(system_id, component_id, msg, named_command_struct->name, named_command_struct->type, named_command_struct->ignore, named_command_struct->qx, named_command_struct->qy, named_command_struct->qz, named_command_struct->Fx, named_command_struct->Fy, named_command_struct->Fz);
}

/**
 * @brief Encode a named_command_struct struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param named_command_struct C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_command_struct_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_named_command_struct_t* named_command_struct)
{
	return mavlink_msg_named_command_struct_pack_chan(system_id, component_id, chan, msg, named_command_struct->name, named_command_struct->type, named_command_struct->ignore, named_command_struct->qx, named_command_struct->qy, named_command_struct->qz, named_command_struct->Fx, named_command_struct->Fy, named_command_struct->Fz);
}

/**
 * @brief Send a named_command_struct message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name of the command struct
 * @param type Type of command struct
 * @param ignore Type of command struct
 * @param qx Qx value in the command struct
 * @param qy Qy value in the command struct
 * @param qz Qz value in the command struct
 * @param Fx Fx value in the command struct
 * @param Fy Fy value in the command struct
 * @param Fz Fz value in the command struct
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_command_struct_send(mavlink_channel_t chan, const char *name, uint8_t type, uint8_t ignore, float qx, float qy, float qz, float Fx, float Fy, float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN];
	_mav_put_float(buf, 0, qx);
	_mav_put_float(buf, 4, qy);
	_mav_put_float(buf, 8, qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 34, type);
	_mav_put_uint8_t(buf, 35, ignore);
	_mav_put_char_array(buf, 24, name, 10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#else
	mavlink_named_command_struct_t packet;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;
	packet.Fx = Fx;
	packet.Fy = Fy;
	packet.Fz = Fz;
	packet.type = type;
	packet.ignore = ignore;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)&packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)&packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_named_command_struct_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *name, uint8_t type, uint8_t ignore, float qx, float qy, float qz, float Fx, float Fy, float Fz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, qx);
	_mav_put_float(buf, 4, qy);
	_mav_put_float(buf, 8, qz);
	_mav_put_float(buf, 12, Fx);
	_mav_put_float(buf, 16, Fy);
	_mav_put_float(buf, 20, Fz);
	_mav_put_uint8_t(buf, 34, type);
	_mav_put_uint8_t(buf, 35, ignore);
	_mav_put_char_array(buf, 24, name, 10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#else
	mavlink_named_command_struct_t *packet = (mavlink_named_command_struct_t *)msgbuf;
	packet->qx = qx;
	packet->qy = qy;
	packet->qz = qz;
	packet->Fx = Fx;
	packet->Fy = Fy;
	packet->Fz = Fz;
	packet->type = type;
	packet->ignore = ignore;
	mav_array_memcpy(packet->name, name, sizeof(char)*10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE NAMED_COMMAND_STRUCT UNPACKING


/**
 * @brief Get field name from named_command_struct message
 *
 * @return Name of the command struct
 */
static inline uint16_t mavlink_msg_named_command_struct_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 10,  24);
}

/**
 * @brief Get field type from named_command_struct message
 *
 * @return Type of command struct
 */
static inline uint8_t mavlink_msg_named_command_struct_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field ignore from named_command_struct message
 *
 * @return Type of command struct
 */
static inline uint8_t mavlink_msg_named_command_struct_get_ignore(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field qx from named_command_struct message
 *
 * @return Qx value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_qx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field qy from named_command_struct message
 *
 * @return Qy value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_qy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field qz from named_command_struct message
 *
 * @return Qz value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_qz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field Fx from named_command_struct message
 *
 * @return Fx value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_Fx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field Fy from named_command_struct message
 *
 * @return Fy value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_Fy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Fz from named_command_struct message
 *
 * @return Fz value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_Fz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a named_command_struct message into a struct
 *
 * @param msg The message to decode
 * @param named_command_struct C-struct to decode the message contents into
 */
static inline void mavlink_msg_named_command_struct_decode(const mavlink_message_t* msg, mavlink_named_command_struct_t* named_command_struct)
{
#if MAVLINK_NEED_BYTE_SWAP
	named_command_struct->qx = mavlink_msg_named_command_struct_get_qx(msg);
	named_command_struct->qy = mavlink_msg_named_command_struct_get_qy(msg);
	named_command_struct->qz = mavlink_msg_named_command_struct_get_qz(msg);
	named_command_struct->Fx = mavlink_msg_named_command_struct_get_Fx(msg);
	named_command_struct->Fy = mavlink_msg_named_command_struct_get_Fy(msg);
	named_command_struct->Fz = mavlink_msg_named_command_struct_get_Fz(msg);
	mavlink_msg_named_command_struct_get_name(msg, named_command_struct->name);
	named_command_struct->type = mavlink_msg_named_command_struct_get_type(msg);
	named_command_struct->ignore = mavlink_msg_named_command_struct_get_ignore(msg);
#else
	memcpy(named_command_struct, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
}
