// MESSAGE ROSFLIGHT_GNSS PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS 197

typedef struct __mavlink_rosflight_gnss_t
{
 uint64_t time; /*< */
 uint64_t nanos; /*< */
 uint64_t rosflight_timestamp; /*< */
 uint32_t time_of_week; /*< */
 int32_t lat; /*< */
 int32_t lon; /*< */
 int32_t height; /*< */
 int32_t vel_n; /*< */
 int32_t vel_e; /*< */
 int32_t vel_d; /*< */
 uint32_t h_acc; /*< */
 uint32_t v_acc; /*< */
 int32_t ecef_x; /*< */
 int32_t ecef_y; /*< */
 int32_t ecef_z; /*< */
 uint32_t p_acc; /*< */
 int32_t ecef_v_x; /*< */
 int32_t ecef_v_y; /*< */
 int32_t ecef_v_z; /*< */
 uint32_t s_acc; /*< */
 uint8_t fix_type; /*< */
} mavlink_rosflight_gnss_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN 93
#define MAVLINK_MSG_ID_197_LEN 93

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC 9
#define MAVLINK_MSG_ID_197_CRC 9



#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_GNSS { \
	"ROSFLIGHT_GNSS", \
	21, \
	{  { "time", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rosflight_gnss_t, time) }, \
         { "nanos", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_rosflight_gnss_t, nanos) }, \
         { "rosflight_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_rosflight_gnss_t, rosflight_timestamp) }, \
         { "time_of_week", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_rosflight_gnss_t, time_of_week) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_rosflight_gnss_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_rosflight_gnss_t, lon) }, \
         { "height", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_rosflight_gnss_t, height) }, \
         { "vel_n", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_rosflight_gnss_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_rosflight_gnss_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_rosflight_gnss_t, vel_d) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 52, offsetof(mavlink_rosflight_gnss_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 56, offsetof(mavlink_rosflight_gnss_t, v_acc) }, \
         { "ecef_x", NULL, MAVLINK_TYPE_INT32_T, 0, 60, offsetof(mavlink_rosflight_gnss_t, ecef_x) }, \
         { "ecef_y", NULL, MAVLINK_TYPE_INT32_T, 0, 64, offsetof(mavlink_rosflight_gnss_t, ecef_y) }, \
         { "ecef_z", NULL, MAVLINK_TYPE_INT32_T, 0, 68, offsetof(mavlink_rosflight_gnss_t, ecef_z) }, \
         { "p_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 72, offsetof(mavlink_rosflight_gnss_t, p_acc) }, \
         { "ecef_v_x", NULL, MAVLINK_TYPE_INT32_T, 0, 76, offsetof(mavlink_rosflight_gnss_t, ecef_v_x) }, \
         { "ecef_v_y", NULL, MAVLINK_TYPE_INT32_T, 0, 80, offsetof(mavlink_rosflight_gnss_t, ecef_v_y) }, \
         { "ecef_v_z", NULL, MAVLINK_TYPE_INT32_T, 0, 84, offsetof(mavlink_rosflight_gnss_t, ecef_v_z) }, \
         { "s_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 88, offsetof(mavlink_rosflight_gnss_t, s_acc) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 92, offsetof(mavlink_rosflight_gnss_t, fix_type) }, \
         } \
}


/**
 * @brief Pack a rosflight_gnss message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_of_week 
 * @param fix_type 
 * @param time 
 * @param nanos 
 * @param lat 
 * @param lon 
 * @param height 
 * @param vel_n 
 * @param vel_e 
 * @param vel_d 
 * @param h_acc 
 * @param v_acc 
 * @param ecef_x 
 * @param ecef_y 
 * @param ecef_z 
 * @param p_acc 
 * @param ecef_v_x 
 * @param ecef_v_y 
 * @param ecef_v_z 
 * @param s_acc 
 * @param rosflight_timestamp 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_gnss_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_of_week, uint8_t fix_type, uint64_t time, uint64_t nanos, int32_t lat, int32_t lon, int32_t height, int32_t vel_n, int32_t vel_e, int32_t vel_d, uint32_t h_acc, uint32_t v_acc, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, uint32_t p_acc, int32_t ecef_v_x, int32_t ecef_v_y, int32_t ecef_v_z, uint32_t s_acc, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_uint64_t(buf, 8, nanos);
	_mav_put_uint64_t(buf, 16, rosflight_timestamp);
	_mav_put_uint32_t(buf, 24, time_of_week);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, height);
	_mav_put_int32_t(buf, 40, vel_n);
	_mav_put_int32_t(buf, 44, vel_e);
	_mav_put_int32_t(buf, 48, vel_d);
	_mav_put_uint32_t(buf, 52, h_acc);
	_mav_put_uint32_t(buf, 56, v_acc);
	_mav_put_int32_t(buf, 60, ecef_x);
	_mav_put_int32_t(buf, 64, ecef_y);
	_mav_put_int32_t(buf, 68, ecef_z);
	_mav_put_uint32_t(buf, 72, p_acc);
	_mav_put_int32_t(buf, 76, ecef_v_x);
	_mav_put_int32_t(buf, 80, ecef_v_y);
	_mav_put_int32_t(buf, 84, ecef_v_z);
	_mav_put_uint32_t(buf, 88, s_acc);
	_mav_put_uint8_t(buf, 92, fix_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#else
	mavlink_rosflight_gnss_t packet;
	packet.time = time;
	packet.nanos = nanos;
	packet.rosflight_timestamp = rosflight_timestamp;
	packet.time_of_week = time_of_week;
	packet.lat = lat;
	packet.lon = lon;
	packet.height = height;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.h_acc = h_acc;
	packet.v_acc = v_acc;
	packet.ecef_x = ecef_x;
	packet.ecef_y = ecef_y;
	packet.ecef_z = ecef_z;
	packet.p_acc = p_acc;
	packet.ecef_v_x = ecef_v_x;
	packet.ecef_v_y = ecef_v_y;
	packet.ecef_v_z = ecef_v_z;
	packet.s_acc = s_acc;
	packet.fix_type = fix_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_GNSS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
}

/**
 * @brief Pack a rosflight_gnss message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_of_week 
 * @param fix_type 
 * @param time 
 * @param nanos 
 * @param lat 
 * @param lon 
 * @param height 
 * @param vel_n 
 * @param vel_e 
 * @param vel_d 
 * @param h_acc 
 * @param v_acc 
 * @param ecef_x 
 * @param ecef_y 
 * @param ecef_z 
 * @param p_acc 
 * @param ecef_v_x 
 * @param ecef_v_y 
 * @param ecef_v_z 
 * @param s_acc 
 * @param rosflight_timestamp 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_gnss_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_of_week,uint8_t fix_type,uint64_t time,uint64_t nanos,int32_t lat,int32_t lon,int32_t height,int32_t vel_n,int32_t vel_e,int32_t vel_d,uint32_t h_acc,uint32_t v_acc,int32_t ecef_x,int32_t ecef_y,int32_t ecef_z,uint32_t p_acc,int32_t ecef_v_x,int32_t ecef_v_y,int32_t ecef_v_z,uint32_t s_acc,uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_uint64_t(buf, 8, nanos);
	_mav_put_uint64_t(buf, 16, rosflight_timestamp);
	_mav_put_uint32_t(buf, 24, time_of_week);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, height);
	_mav_put_int32_t(buf, 40, vel_n);
	_mav_put_int32_t(buf, 44, vel_e);
	_mav_put_int32_t(buf, 48, vel_d);
	_mav_put_uint32_t(buf, 52, h_acc);
	_mav_put_uint32_t(buf, 56, v_acc);
	_mav_put_int32_t(buf, 60, ecef_x);
	_mav_put_int32_t(buf, 64, ecef_y);
	_mav_put_int32_t(buf, 68, ecef_z);
	_mav_put_uint32_t(buf, 72, p_acc);
	_mav_put_int32_t(buf, 76, ecef_v_x);
	_mav_put_int32_t(buf, 80, ecef_v_y);
	_mav_put_int32_t(buf, 84, ecef_v_z);
	_mav_put_uint32_t(buf, 88, s_acc);
	_mav_put_uint8_t(buf, 92, fix_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#else
	mavlink_rosflight_gnss_t packet;
	packet.time = time;
	packet.nanos = nanos;
	packet.rosflight_timestamp = rosflight_timestamp;
	packet.time_of_week = time_of_week;
	packet.lat = lat;
	packet.lon = lon;
	packet.height = height;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.h_acc = h_acc;
	packet.v_acc = v_acc;
	packet.ecef_x = ecef_x;
	packet.ecef_y = ecef_y;
	packet.ecef_z = ecef_z;
	packet.p_acc = p_acc;
	packet.ecef_v_x = ecef_v_x;
	packet.ecef_v_y = ecef_v_y;
	packet.ecef_v_z = ecef_v_z;
	packet.s_acc = s_acc;
	packet.fix_type = fix_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_GNSS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
}

/**
 * @brief Encode a rosflight_gnss struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_gnss C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_gnss_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_gnss_t* rosflight_gnss)
{
	return mavlink_msg_rosflight_gnss_pack(system_id, component_id, msg, rosflight_gnss->time_of_week, rosflight_gnss->fix_type, rosflight_gnss->time, rosflight_gnss->nanos, rosflight_gnss->lat, rosflight_gnss->lon, rosflight_gnss->height, rosflight_gnss->vel_n, rosflight_gnss->vel_e, rosflight_gnss->vel_d, rosflight_gnss->h_acc, rosflight_gnss->v_acc, rosflight_gnss->ecef_x, rosflight_gnss->ecef_y, rosflight_gnss->ecef_z, rosflight_gnss->p_acc, rosflight_gnss->ecef_v_x, rosflight_gnss->ecef_v_y, rosflight_gnss->ecef_v_z, rosflight_gnss->s_acc, rosflight_gnss->rosflight_timestamp);
}

/**
 * @brief Encode a rosflight_gnss struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_gnss C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_gnss_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_gnss_t* rosflight_gnss)
{
	return mavlink_msg_rosflight_gnss_pack_chan(system_id, component_id, chan, msg, rosflight_gnss->time_of_week, rosflight_gnss->fix_type, rosflight_gnss->time, rosflight_gnss->nanos, rosflight_gnss->lat, rosflight_gnss->lon, rosflight_gnss->height, rosflight_gnss->vel_n, rosflight_gnss->vel_e, rosflight_gnss->vel_d, rosflight_gnss->h_acc, rosflight_gnss->v_acc, rosflight_gnss->ecef_x, rosflight_gnss->ecef_y, rosflight_gnss->ecef_z, rosflight_gnss->p_acc, rosflight_gnss->ecef_v_x, rosflight_gnss->ecef_v_y, rosflight_gnss->ecef_v_z, rosflight_gnss->s_acc, rosflight_gnss->rosflight_timestamp);
}

/**
 * @brief Send a rosflight_gnss message
 * @param chan MAVLink channel to send the message
 *
 * @param time_of_week 
 * @param fix_type 
 * @param time 
 * @param nanos 
 * @param lat 
 * @param lon 
 * @param height 
 * @param vel_n 
 * @param vel_e 
 * @param vel_d 
 * @param h_acc 
 * @param v_acc 
 * @param ecef_x 
 * @param ecef_y 
 * @param ecef_z 
 * @param p_acc 
 * @param ecef_v_x 
 * @param ecef_v_y 
 * @param ecef_v_z 
 * @param s_acc 
 * @param rosflight_timestamp 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_gnss_send(mavlink_channel_t chan, uint32_t time_of_week, uint8_t fix_type, uint64_t time, uint64_t nanos, int32_t lat, int32_t lon, int32_t height, int32_t vel_n, int32_t vel_e, int32_t vel_d, uint32_t h_acc, uint32_t v_acc, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, uint32_t p_acc, int32_t ecef_v_x, int32_t ecef_v_y, int32_t ecef_v_z, uint32_t s_acc, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_uint64_t(buf, 8, nanos);
	_mav_put_uint64_t(buf, 16, rosflight_timestamp);
	_mav_put_uint32_t(buf, 24, time_of_week);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, height);
	_mav_put_int32_t(buf, 40, vel_n);
	_mav_put_int32_t(buf, 44, vel_e);
	_mav_put_int32_t(buf, 48, vel_d);
	_mav_put_uint32_t(buf, 52, h_acc);
	_mav_put_uint32_t(buf, 56, v_acc);
	_mav_put_int32_t(buf, 60, ecef_x);
	_mav_put_int32_t(buf, 64, ecef_y);
	_mav_put_int32_t(buf, 68, ecef_z);
	_mav_put_uint32_t(buf, 72, p_acc);
	_mav_put_int32_t(buf, 76, ecef_v_x);
	_mav_put_int32_t(buf, 80, ecef_v_y);
	_mav_put_int32_t(buf, 84, ecef_v_z);
	_mav_put_uint32_t(buf, 88, s_acc);
	_mav_put_uint8_t(buf, 92, fix_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
#else
	mavlink_rosflight_gnss_t packet;
	packet.time = time;
	packet.nanos = nanos;
	packet.rosflight_timestamp = rosflight_timestamp;
	packet.time_of_week = time_of_week;
	packet.lat = lat;
	packet.lon = lon;
	packet.height = height;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.h_acc = h_acc;
	packet.v_acc = v_acc;
	packet.ecef_x = ecef_x;
	packet.ecef_y = ecef_y;
	packet.ecef_z = ecef_z;
	packet.p_acc = p_acc;
	packet.ecef_v_x = ecef_v_x;
	packet.ecef_v_y = ecef_v_y;
	packet.ecef_v_z = ecef_v_z;
	packet.s_acc = s_acc;
	packet.fix_type = fix_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_gnss_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_of_week, uint8_t fix_type, uint64_t time, uint64_t nanos, int32_t lat, int32_t lon, int32_t height, int32_t vel_n, int32_t vel_e, int32_t vel_d, uint32_t h_acc, uint32_t v_acc, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, uint32_t p_acc, int32_t ecef_v_x, int32_t ecef_v_y, int32_t ecef_v_z, uint32_t s_acc, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_uint64_t(buf, 8, nanos);
	_mav_put_uint64_t(buf, 16, rosflight_timestamp);
	_mav_put_uint32_t(buf, 24, time_of_week);
	_mav_put_int32_t(buf, 28, lat);
	_mav_put_int32_t(buf, 32, lon);
	_mav_put_int32_t(buf, 36, height);
	_mav_put_int32_t(buf, 40, vel_n);
	_mav_put_int32_t(buf, 44, vel_e);
	_mav_put_int32_t(buf, 48, vel_d);
	_mav_put_uint32_t(buf, 52, h_acc);
	_mav_put_uint32_t(buf, 56, v_acc);
	_mav_put_int32_t(buf, 60, ecef_x);
	_mav_put_int32_t(buf, 64, ecef_y);
	_mav_put_int32_t(buf, 68, ecef_z);
	_mav_put_uint32_t(buf, 72, p_acc);
	_mav_put_int32_t(buf, 76, ecef_v_x);
	_mav_put_int32_t(buf, 80, ecef_v_y);
	_mav_put_int32_t(buf, 84, ecef_v_z);
	_mav_put_uint32_t(buf, 88, s_acc);
	_mav_put_uint8_t(buf, 92, fix_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
#else
	mavlink_rosflight_gnss_t *packet = (mavlink_rosflight_gnss_t *)msgbuf;
	packet->time = time;
	packet->nanos = nanos;
	packet->rosflight_timestamp = rosflight_timestamp;
	packet->time_of_week = time_of_week;
	packet->lat = lat;
	packet->lon = lon;
	packet->height = height;
	packet->vel_n = vel_n;
	packet->vel_e = vel_e;
	packet->vel_d = vel_d;
	packet->h_acc = h_acc;
	packet->v_acc = v_acc;
	packet->ecef_x = ecef_x;
	packet->ecef_y = ecef_y;
	packet->ecef_z = ecef_z;
	packet->p_acc = p_acc;
	packet->ecef_v_x = ecef_v_x;
	packet->ecef_v_y = ecef_v_y;
	packet->ecef_v_z = ecef_v_z;
	packet->s_acc = s_acc;
	packet->fix_type = fix_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_GNSS UNPACKING


/**
 * @brief Get field time_of_week from rosflight_gnss message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_get_time_of_week(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field fix_type from rosflight_gnss message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_get_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  92);
}

/**
 * @brief Get field time from rosflight_gnss message
 *
 * @return 
 */
static inline uint64_t mavlink_msg_rosflight_gnss_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field nanos from rosflight_gnss message
 *
 * @return 
 */
static inline uint64_t mavlink_msg_rosflight_gnss_get_nanos(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field lat from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field lon from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field height from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field vel_n from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_vel_n(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Get field vel_e from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_vel_e(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field vel_d from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_vel_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  48);
}

/**
 * @brief Get field h_acc from rosflight_gnss message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_get_h_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  52);
}

/**
 * @brief Get field v_acc from rosflight_gnss message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_get_v_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  56);
}

/**
 * @brief Get field ecef_x from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_ecef_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  60);
}

/**
 * @brief Get field ecef_y from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_ecef_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  64);
}

/**
 * @brief Get field ecef_z from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_ecef_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  68);
}

/**
 * @brief Get field p_acc from rosflight_gnss message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_get_p_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  72);
}

/**
 * @brief Get field ecef_v_x from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_ecef_v_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  76);
}

/**
 * @brief Get field ecef_v_y from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_ecef_v_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  80);
}

/**
 * @brief Get field ecef_v_z from rosflight_gnss message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_ecef_v_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  84);
}

/**
 * @brief Get field s_acc from rosflight_gnss message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_get_s_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  88);
}

/**
 * @brief Get field rosflight_timestamp from rosflight_gnss message
 *
 * @return 
 */
static inline uint64_t mavlink_msg_rosflight_gnss_get_rosflight_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Decode a rosflight_gnss message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_gnss C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_gnss_decode(const mavlink_message_t* msg, mavlink_rosflight_gnss_t* rosflight_gnss)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_gnss->time = mavlink_msg_rosflight_gnss_get_time(msg);
	rosflight_gnss->nanos = mavlink_msg_rosflight_gnss_get_nanos(msg);
	rosflight_gnss->rosflight_timestamp = mavlink_msg_rosflight_gnss_get_rosflight_timestamp(msg);
	rosflight_gnss->time_of_week = mavlink_msg_rosflight_gnss_get_time_of_week(msg);
	rosflight_gnss->lat = mavlink_msg_rosflight_gnss_get_lat(msg);
	rosflight_gnss->lon = mavlink_msg_rosflight_gnss_get_lon(msg);
	rosflight_gnss->height = mavlink_msg_rosflight_gnss_get_height(msg);
	rosflight_gnss->vel_n = mavlink_msg_rosflight_gnss_get_vel_n(msg);
	rosflight_gnss->vel_e = mavlink_msg_rosflight_gnss_get_vel_e(msg);
	rosflight_gnss->vel_d = mavlink_msg_rosflight_gnss_get_vel_d(msg);
	rosflight_gnss->h_acc = mavlink_msg_rosflight_gnss_get_h_acc(msg);
	rosflight_gnss->v_acc = mavlink_msg_rosflight_gnss_get_v_acc(msg);
	rosflight_gnss->ecef_x = mavlink_msg_rosflight_gnss_get_ecef_x(msg);
	rosflight_gnss->ecef_y = mavlink_msg_rosflight_gnss_get_ecef_y(msg);
	rosflight_gnss->ecef_z = mavlink_msg_rosflight_gnss_get_ecef_z(msg);
	rosflight_gnss->p_acc = mavlink_msg_rosflight_gnss_get_p_acc(msg);
	rosflight_gnss->ecef_v_x = mavlink_msg_rosflight_gnss_get_ecef_v_x(msg);
	rosflight_gnss->ecef_v_y = mavlink_msg_rosflight_gnss_get_ecef_v_y(msg);
	rosflight_gnss->ecef_v_z = mavlink_msg_rosflight_gnss_get_ecef_v_z(msg);
	rosflight_gnss->s_acc = mavlink_msg_rosflight_gnss_get_s_acc(msg);
	rosflight_gnss->fix_type = mavlink_msg_rosflight_gnss_get_fix_type(msg);
#else
	memcpy(rosflight_gnss, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
}
