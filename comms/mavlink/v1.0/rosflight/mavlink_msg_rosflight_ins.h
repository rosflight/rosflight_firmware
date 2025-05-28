// MESSAGE ROSFLIGHT_INS PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_INS 194

typedef struct __mavlink_rosflight_ins_t
{
 float pos_north; /*< */
 float pos_east; /*< */
 float pos_down; /*< */
 float qw; /*< */
 float qx; /*< */
 float qy; /*< */
 float qz; /*< */
 float u; /*< */
 float v; /*< */
 float w; /*< */
 float p; /*< */
 float q; /*< */
 float r; /*< */
} mavlink_rosflight_ins_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN 52
#define MAVLINK_MSG_ID_194_LEN 52

#define MAVLINK_MSG_ID_ROSFLIGHT_INS_CRC 125
#define MAVLINK_MSG_ID_194_CRC 125



#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_INS { \
	"ROSFLIGHT_INS", \
	13, \
	{  { "pos_north", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rosflight_ins_t, pos_north) }, \
         { "pos_east", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rosflight_ins_t, pos_east) }, \
         { "pos_down", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rosflight_ins_t, pos_down) }, \
         { "qw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rosflight_ins_t, qw) }, \
         { "qx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rosflight_ins_t, qx) }, \
         { "qy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rosflight_ins_t, qy) }, \
         { "qz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rosflight_ins_t, qz) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rosflight_ins_t, u) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rosflight_ins_t, v) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rosflight_ins_t, w) }, \
         { "p", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rosflight_ins_t, p) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rosflight_ins_t, q) }, \
         { "r", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rosflight_ins_t, r) }, \
         } \
}


/**
 * @brief Pack a rosflight_ins message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_north 
 * @param pos_east 
 * @param pos_down 
 * @param qw 
 * @param qx 
 * @param qy 
 * @param qz 
 * @param u 
 * @param v 
 * @param w 
 * @param p 
 * @param q 
 * @param r 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_ins_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float pos_north, float pos_east, float pos_down, float qw, float qx, float qy, float qz, float u, float v, float w, float p, float q, float r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN];
	_mav_put_float(buf, 0, pos_north);
	_mav_put_float(buf, 4, pos_east);
	_mav_put_float(buf, 8, pos_down);
	_mav_put_float(buf, 12, qw);
	_mav_put_float(buf, 16, qx);
	_mav_put_float(buf, 20, qy);
	_mav_put_float(buf, 24, qz);
	_mav_put_float(buf, 28, u);
	_mav_put_float(buf, 32, v);
	_mav_put_float(buf, 36, w);
	_mav_put_float(buf, 40, p);
	_mav_put_float(buf, 44, q);
	_mav_put_float(buf, 48, r);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#else
	mavlink_rosflight_ins_t packet;
	packet.pos_north = pos_north;
	packet.pos_east = pos_east;
	packet.pos_down = pos_down;
	packet.qw = qw;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;
	packet.u = u;
	packet.v = v;
	packet.w = w;
	packet.p = p;
	packet.q = q;
	packet.r = r;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_INS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_INS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif
}

/**
 * @brief Pack a rosflight_ins message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_north 
 * @param pos_east 
 * @param pos_down 
 * @param qw 
 * @param qx 
 * @param qy 
 * @param qz 
 * @param u 
 * @param v 
 * @param w 
 * @param p 
 * @param q 
 * @param r 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_ins_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float pos_north,float pos_east,float pos_down,float qw,float qx,float qy,float qz,float u,float v,float w,float p,float q,float r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN];
	_mav_put_float(buf, 0, pos_north);
	_mav_put_float(buf, 4, pos_east);
	_mav_put_float(buf, 8, pos_down);
	_mav_put_float(buf, 12, qw);
	_mav_put_float(buf, 16, qx);
	_mav_put_float(buf, 20, qy);
	_mav_put_float(buf, 24, qz);
	_mav_put_float(buf, 28, u);
	_mav_put_float(buf, 32, v);
	_mav_put_float(buf, 36, w);
	_mav_put_float(buf, 40, p);
	_mav_put_float(buf, 44, q);
	_mav_put_float(buf, 48, r);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#else
	mavlink_rosflight_ins_t packet;
	packet.pos_north = pos_north;
	packet.pos_east = pos_east;
	packet.pos_down = pos_down;
	packet.qw = qw;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;
	packet.u = u;
	packet.v = v;
	packet.w = w;
	packet.p = p;
	packet.q = q;
	packet.r = r;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_INS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_INS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif
}

/**
 * @brief Encode a rosflight_ins struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_ins C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_ins_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_ins_t* rosflight_ins)
{
	return mavlink_msg_rosflight_ins_pack(system_id, component_id, msg, rosflight_ins->pos_north, rosflight_ins->pos_east, rosflight_ins->pos_down, rosflight_ins->qw, rosflight_ins->qx, rosflight_ins->qy, rosflight_ins->qz, rosflight_ins->u, rosflight_ins->v, rosflight_ins->w, rosflight_ins->p, rosflight_ins->q, rosflight_ins->r);
}

/**
 * @brief Encode a rosflight_ins struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_ins C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_ins_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_ins_t* rosflight_ins)
{
	return mavlink_msg_rosflight_ins_pack_chan(system_id, component_id, chan, msg, rosflight_ins->pos_north, rosflight_ins->pos_east, rosflight_ins->pos_down, rosflight_ins->qw, rosflight_ins->qx, rosflight_ins->qy, rosflight_ins->qz, rosflight_ins->u, rosflight_ins->v, rosflight_ins->w, rosflight_ins->p, rosflight_ins->q, rosflight_ins->r);
}

/**
 * @brief Send a rosflight_ins message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_north 
 * @param pos_east 
 * @param pos_down 
 * @param qw 
 * @param qx 
 * @param qy 
 * @param qz 
 * @param u 
 * @param v 
 * @param w 
 * @param p 
 * @param q 
 * @param r 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_ins_send(mavlink_channel_t chan, float pos_north, float pos_east, float pos_down, float qw, float qx, float qy, float qz, float u, float v, float w, float p, float q, float r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN];
	_mav_put_float(buf, 0, pos_north);
	_mav_put_float(buf, 4, pos_east);
	_mav_put_float(buf, 8, pos_down);
	_mav_put_float(buf, 12, qw);
	_mav_put_float(buf, 16, qx);
	_mav_put_float(buf, 20, qy);
	_mav_put_float(buf, 24, qz);
	_mav_put_float(buf, 28, u);
	_mav_put_float(buf, 32, v);
	_mav_put_float(buf, 36, w);
	_mav_put_float(buf, 40, p);
	_mav_put_float(buf, 44, q);
	_mav_put_float(buf, 48, r);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, buf, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_INS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, buf, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif
#else
	mavlink_rosflight_ins_t packet;
	packet.pos_north = pos_north;
	packet.pos_east = pos_east;
	packet.pos_down = pos_down;
	packet.qw = qw;
	packet.qx = qx;
	packet.qy = qy;
	packet.qz = qz;
	packet.u = u;
	packet.v = v;
	packet.w = w;
	packet.p = p;
	packet.q = q;
	packet.r = r;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_INS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_ins_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_north, float pos_east, float pos_down, float qw, float qx, float qy, float qz, float u, float v, float w, float p, float q, float r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, pos_north);
	_mav_put_float(buf, 4, pos_east);
	_mav_put_float(buf, 8, pos_down);
	_mav_put_float(buf, 12, qw);
	_mav_put_float(buf, 16, qx);
	_mav_put_float(buf, 20, qy);
	_mav_put_float(buf, 24, qz);
	_mav_put_float(buf, 28, u);
	_mav_put_float(buf, 32, v);
	_mav_put_float(buf, 36, w);
	_mav_put_float(buf, 40, p);
	_mav_put_float(buf, 44, q);
	_mav_put_float(buf, 48, r);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, buf, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_INS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, buf, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif
#else
	mavlink_rosflight_ins_t *packet = (mavlink_rosflight_ins_t *)msgbuf;
	packet->pos_north = pos_north;
	packet->pos_east = pos_east;
	packet->pos_down = pos_down;
	packet->qw = qw;
	packet->qx = qx;
	packet->qy = qy;
	packet->qz = qz;
	packet->u = u;
	packet->v = v;
	packet->w = w;
	packet->p = p;
	packet->q = q;
	packet->r = r;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_INS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_INS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_INS UNPACKING


/**
 * @brief Get field pos_north from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_pos_north(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_east from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_pos_east(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pos_down from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_pos_down(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field qw from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_qw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field qx from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_qx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field qy from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_qy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field qz from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_qz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field u from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_u(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field v from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field w from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_w(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field p from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field q from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_q(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field r from rosflight_ins message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_ins_get_r(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Decode a rosflight_ins message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_ins C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_ins_decode(const mavlink_message_t* msg, mavlink_rosflight_ins_t* rosflight_ins)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_ins->pos_north = mavlink_msg_rosflight_ins_get_pos_north(msg);
	rosflight_ins->pos_east = mavlink_msg_rosflight_ins_get_pos_east(msg);
	rosflight_ins->pos_down = mavlink_msg_rosflight_ins_get_pos_down(msg);
	rosflight_ins->qw = mavlink_msg_rosflight_ins_get_qw(msg);
	rosflight_ins->qx = mavlink_msg_rosflight_ins_get_qx(msg);
	rosflight_ins->qy = mavlink_msg_rosflight_ins_get_qy(msg);
	rosflight_ins->qz = mavlink_msg_rosflight_ins_get_qz(msg);
	rosflight_ins->u = mavlink_msg_rosflight_ins_get_u(msg);
	rosflight_ins->v = mavlink_msg_rosflight_ins_get_v(msg);
	rosflight_ins->w = mavlink_msg_rosflight_ins_get_w(msg);
	rosflight_ins->p = mavlink_msg_rosflight_ins_get_p(msg);
	rosflight_ins->q = mavlink_msg_rosflight_ins_get_q(msg);
	rosflight_ins->r = mavlink_msg_rosflight_ins_get_r(msg);
#else
	memcpy(rosflight_ins, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_INS_LEN);
#endif
}
