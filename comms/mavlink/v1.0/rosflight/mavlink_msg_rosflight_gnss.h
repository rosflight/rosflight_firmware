#pragma once
// MESSAGE ROSFLIGHT_GNSS PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS 197


typedef struct __mavlink_rosflight_gnss_t {
 int64_t seconds; /*<  Unix time, in seconds*/
 double lat; /*<  In deg DDS format*/
 double lon; /*<  In deg DDs format*/
 uint64_t rosflight_timestamp; /*<  microseconds, estimated firmware timestamp for the time of validity of the gnss data*/
 int32_t nanos; /*<  Fractional Unix time*/
 float height; /*<  meters, MSL*/
 float vel_n; /*<  meters per second*/
 float vel_e; /*<  meters per second*/
 float vel_d; /*<  meters per second*/
 float h_acc; /*<  meters*/
 float v_acc; /*<  meters*/
 float s_acc; /*<  meters*/
 uint8_t fix_type; /*<  GNSS fix type*/
 uint8_t num_sat; /*<  Number of satellites seen*/
} mavlink_rosflight_gnss_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN 66
#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN 66
#define MAVLINK_MSG_ID_197_LEN 66
#define MAVLINK_MSG_ID_197_MIN_LEN 66

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC 221
#define MAVLINK_MSG_ID_197_CRC 221



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_GNSS { \
    197, \
    "ROSFLIGHT_GNSS", \
    14, \
    {  { "seconds", NULL, MAVLINK_TYPE_INT64_T, 0, 0, offsetof(mavlink_rosflight_gnss_t, seconds) }, \
         { "nanos", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_rosflight_gnss_t, nanos) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_rosflight_gnss_t, fix_type) }, \
         { "num_sat", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_rosflight_gnss_t, num_sat) }, \
         { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_rosflight_gnss_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_rosflight_gnss_t, lon) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rosflight_gnss_t, height) }, \
         { "vel_n", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rosflight_gnss_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rosflight_gnss_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rosflight_gnss_t, vel_d) }, \
         { "h_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rosflight_gnss_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rosflight_gnss_t, v_acc) }, \
         { "s_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rosflight_gnss_t, s_acc) }, \
         { "rosflight_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_rosflight_gnss_t, rosflight_timestamp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_GNSS { \
    "ROSFLIGHT_GNSS", \
    14, \
    {  { "seconds", NULL, MAVLINK_TYPE_INT64_T, 0, 0, offsetof(mavlink_rosflight_gnss_t, seconds) }, \
         { "nanos", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_rosflight_gnss_t, nanos) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_rosflight_gnss_t, fix_type) }, \
         { "num_sat", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_rosflight_gnss_t, num_sat) }, \
         { "lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_rosflight_gnss_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_rosflight_gnss_t, lon) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rosflight_gnss_t, height) }, \
         { "vel_n", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rosflight_gnss_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rosflight_gnss_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rosflight_gnss_t, vel_d) }, \
         { "h_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rosflight_gnss_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rosflight_gnss_t, v_acc) }, \
         { "s_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rosflight_gnss_t, s_acc) }, \
         { "rosflight_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 24, offsetof(mavlink_rosflight_gnss_t, rosflight_timestamp) }, \
         } \
}
#endif

/**
 * @brief Pack a rosflight_gnss message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seconds  Unix time, in seconds
 * @param nanos  Fractional Unix time
 * @param fix_type  GNSS fix type
 * @param num_sat  Number of satellites seen
 * @param lat  In deg DDS format
 * @param lon  In deg DDs format
 * @param height  meters, MSL
 * @param vel_n  meters per second
 * @param vel_e  meters per second
 * @param vel_d  meters per second
 * @param h_acc  meters
 * @param v_acc  meters
 * @param s_acc  meters
 * @param rosflight_timestamp  microseconds, estimated firmware timestamp for the time of validity of the gnss data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_gnss_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int64_t seconds, int32_t nanos, uint8_t fix_type, uint8_t num_sat, double lat, double lon, float height, float vel_n, float vel_e, float vel_d, float h_acc, float v_acc, float s_acc, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN];
    _mav_put_int64_t(buf, 0, seconds);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_uint64_t(buf, 24, rosflight_timestamp);
    _mav_put_int32_t(buf, 32, nanos);
    _mav_put_float(buf, 36, height);
    _mav_put_float(buf, 40, vel_n);
    _mav_put_float(buf, 44, vel_e);
    _mav_put_float(buf, 48, vel_d);
    _mav_put_float(buf, 52, h_acc);
    _mav_put_float(buf, 56, v_acc);
    _mav_put_float(buf, 60, s_acc);
    _mav_put_uint8_t(buf, 64, fix_type);
    _mav_put_uint8_t(buf, 65, num_sat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#else
    mavlink_rosflight_gnss_t packet;
    packet.seconds = seconds;
    packet.lat = lat;
    packet.lon = lon;
    packet.rosflight_timestamp = rosflight_timestamp;
    packet.nanos = nanos;
    packet.height = height;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.fix_type = fix_type;
    packet.num_sat = num_sat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_GNSS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
}

/**
 * @brief Pack a rosflight_gnss message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param seconds  Unix time, in seconds
 * @param nanos  Fractional Unix time
 * @param fix_type  GNSS fix type
 * @param num_sat  Number of satellites seen
 * @param lat  In deg DDS format
 * @param lon  In deg DDs format
 * @param height  meters, MSL
 * @param vel_n  meters per second
 * @param vel_e  meters per second
 * @param vel_d  meters per second
 * @param h_acc  meters
 * @param v_acc  meters
 * @param s_acc  meters
 * @param rosflight_timestamp  microseconds, estimated firmware timestamp for the time of validity of the gnss data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_gnss_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int64_t seconds, int32_t nanos, uint8_t fix_type, uint8_t num_sat, double lat, double lon, float height, float vel_n, float vel_e, float vel_d, float h_acc, float v_acc, float s_acc, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN];
    _mav_put_int64_t(buf, 0, seconds);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_uint64_t(buf, 24, rosflight_timestamp);
    _mav_put_int32_t(buf, 32, nanos);
    _mav_put_float(buf, 36, height);
    _mav_put_float(buf, 40, vel_n);
    _mav_put_float(buf, 44, vel_e);
    _mav_put_float(buf, 48, vel_d);
    _mav_put_float(buf, 52, h_acc);
    _mav_put_float(buf, 56, v_acc);
    _mav_put_float(buf, 60, s_acc);
    _mav_put_uint8_t(buf, 64, fix_type);
    _mav_put_uint8_t(buf, 65, num_sat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#else
    mavlink_rosflight_gnss_t packet;
    packet.seconds = seconds;
    packet.lat = lat;
    packet.lon = lon;
    packet.rosflight_timestamp = rosflight_timestamp;
    packet.nanos = nanos;
    packet.height = height;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.fix_type = fix_type;
    packet.num_sat = num_sat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_GNSS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif
}

/**
 * @brief Pack a rosflight_gnss message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seconds  Unix time, in seconds
 * @param nanos  Fractional Unix time
 * @param fix_type  GNSS fix type
 * @param num_sat  Number of satellites seen
 * @param lat  In deg DDS format
 * @param lon  In deg DDs format
 * @param height  meters, MSL
 * @param vel_n  meters per second
 * @param vel_e  meters per second
 * @param vel_d  meters per second
 * @param h_acc  meters
 * @param v_acc  meters
 * @param s_acc  meters
 * @param rosflight_timestamp  microseconds, estimated firmware timestamp for the time of validity of the gnss data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_gnss_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int64_t seconds,int32_t nanos,uint8_t fix_type,uint8_t num_sat,double lat,double lon,float height,float vel_n,float vel_e,float vel_d,float h_acc,float v_acc,float s_acc,uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN];
    _mav_put_int64_t(buf, 0, seconds);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_uint64_t(buf, 24, rosflight_timestamp);
    _mav_put_int32_t(buf, 32, nanos);
    _mav_put_float(buf, 36, height);
    _mav_put_float(buf, 40, vel_n);
    _mav_put_float(buf, 44, vel_e);
    _mav_put_float(buf, 48, vel_d);
    _mav_put_float(buf, 52, h_acc);
    _mav_put_float(buf, 56, v_acc);
    _mav_put_float(buf, 60, s_acc);
    _mav_put_uint8_t(buf, 64, fix_type);
    _mav_put_uint8_t(buf, 65, num_sat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#else
    mavlink_rosflight_gnss_t packet;
    packet.seconds = seconds;
    packet.lat = lat;
    packet.lon = lon;
    packet.rosflight_timestamp = rosflight_timestamp;
    packet.nanos = nanos;
    packet.height = height;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.fix_type = fix_type;
    packet.num_sat = num_sat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_GNSS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
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
    return mavlink_msg_rosflight_gnss_pack(system_id, component_id, msg, rosflight_gnss->seconds, rosflight_gnss->nanos, rosflight_gnss->fix_type, rosflight_gnss->num_sat, rosflight_gnss->lat, rosflight_gnss->lon, rosflight_gnss->height, rosflight_gnss->vel_n, rosflight_gnss->vel_e, rosflight_gnss->vel_d, rosflight_gnss->h_acc, rosflight_gnss->v_acc, rosflight_gnss->s_acc, rosflight_gnss->rosflight_timestamp);
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
    return mavlink_msg_rosflight_gnss_pack_chan(system_id, component_id, chan, msg, rosflight_gnss->seconds, rosflight_gnss->nanos, rosflight_gnss->fix_type, rosflight_gnss->num_sat, rosflight_gnss->lat, rosflight_gnss->lon, rosflight_gnss->height, rosflight_gnss->vel_n, rosflight_gnss->vel_e, rosflight_gnss->vel_d, rosflight_gnss->h_acc, rosflight_gnss->v_acc, rosflight_gnss->s_acc, rosflight_gnss->rosflight_timestamp);
}

/**
 * @brief Encode a rosflight_gnss struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_gnss C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_gnss_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rosflight_gnss_t* rosflight_gnss)
{
    return mavlink_msg_rosflight_gnss_pack_status(system_id, component_id, _status, msg,  rosflight_gnss->seconds, rosflight_gnss->nanos, rosflight_gnss->fix_type, rosflight_gnss->num_sat, rosflight_gnss->lat, rosflight_gnss->lon, rosflight_gnss->height, rosflight_gnss->vel_n, rosflight_gnss->vel_e, rosflight_gnss->vel_d, rosflight_gnss->h_acc, rosflight_gnss->v_acc, rosflight_gnss->s_acc, rosflight_gnss->rosflight_timestamp);
}

/**
 * @brief Send a rosflight_gnss message
 * @param chan MAVLink channel to send the message
 *
 * @param seconds  Unix time, in seconds
 * @param nanos  Fractional Unix time
 * @param fix_type  GNSS fix type
 * @param num_sat  Number of satellites seen
 * @param lat  In deg DDS format
 * @param lon  In deg DDs format
 * @param height  meters, MSL
 * @param vel_n  meters per second
 * @param vel_e  meters per second
 * @param vel_d  meters per second
 * @param h_acc  meters
 * @param v_acc  meters
 * @param s_acc  meters
 * @param rosflight_timestamp  microseconds, estimated firmware timestamp for the time of validity of the gnss data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_gnss_send(mavlink_channel_t chan, int64_t seconds, int32_t nanos, uint8_t fix_type, uint8_t num_sat, double lat, double lon, float height, float vel_n, float vel_e, float vel_d, float h_acc, float v_acc, float s_acc, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN];
    _mav_put_int64_t(buf, 0, seconds);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_uint64_t(buf, 24, rosflight_timestamp);
    _mav_put_int32_t(buf, 32, nanos);
    _mav_put_float(buf, 36, height);
    _mav_put_float(buf, 40, vel_n);
    _mav_put_float(buf, 44, vel_e);
    _mav_put_float(buf, 48, vel_d);
    _mav_put_float(buf, 52, h_acc);
    _mav_put_float(buf, 56, v_acc);
    _mav_put_float(buf, 60, s_acc);
    _mav_put_uint8_t(buf, 64, fix_type);
    _mav_put_uint8_t(buf, 65, num_sat);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    mavlink_rosflight_gnss_t packet;
    packet.seconds = seconds;
    packet.lat = lat;
    packet.lon = lon;
    packet.rosflight_timestamp = rosflight_timestamp;
    packet.nanos = nanos;
    packet.height = height;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.s_acc = s_acc;
    packet.fix_type = fix_type;
    packet.num_sat = num_sat;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#endif
}

/**
 * @brief Send a rosflight_gnss message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rosflight_gnss_send_struct(mavlink_channel_t chan, const mavlink_rosflight_gnss_t* rosflight_gnss)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rosflight_gnss_send(chan, rosflight_gnss->seconds, rosflight_gnss->nanos, rosflight_gnss->fix_type, rosflight_gnss->num_sat, rosflight_gnss->lat, rosflight_gnss->lon, rosflight_gnss->height, rosflight_gnss->vel_n, rosflight_gnss->vel_e, rosflight_gnss->vel_d, rosflight_gnss->h_acc, rosflight_gnss->v_acc, rosflight_gnss->s_acc, rosflight_gnss->rosflight_timestamp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, (const char *)rosflight_gnss, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_gnss_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int64_t seconds, int32_t nanos, uint8_t fix_type, uint8_t num_sat, double lat, double lon, float height, float vel_n, float vel_e, float vel_d, float h_acc, float v_acc, float s_acc, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int64_t(buf, 0, seconds);
    _mav_put_double(buf, 8, lat);
    _mav_put_double(buf, 16, lon);
    _mav_put_uint64_t(buf, 24, rosflight_timestamp);
    _mav_put_int32_t(buf, 32, nanos);
    _mav_put_float(buf, 36, height);
    _mav_put_float(buf, 40, vel_n);
    _mav_put_float(buf, 44, vel_e);
    _mav_put_float(buf, 48, vel_d);
    _mav_put_float(buf, 52, h_acc);
    _mav_put_float(buf, 56, v_acc);
    _mav_put_float(buf, 60, s_acc);
    _mav_put_uint8_t(buf, 64, fix_type);
    _mav_put_uint8_t(buf, 65, num_sat);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#else
    mavlink_rosflight_gnss_t *packet = (mavlink_rosflight_gnss_t *)msgbuf;
    packet->seconds = seconds;
    packet->lat = lat;
    packet->lon = lon;
    packet->rosflight_timestamp = rosflight_timestamp;
    packet->nanos = nanos;
    packet->height = height;
    packet->vel_n = vel_n;
    packet->vel_e = vel_e;
    packet->vel_d = vel_d;
    packet->h_acc = h_acc;
    packet->v_acc = v_acc;
    packet->s_acc = s_acc;
    packet->fix_type = fix_type;
    packet->num_sat = num_sat;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_GNSS UNPACKING


/**
 * @brief Get field seconds from rosflight_gnss message
 *
 * @return  Unix time, in seconds
 */
static inline int64_t mavlink_msg_rosflight_gnss_get_seconds(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field nanos from rosflight_gnss message
 *
 * @return  Fractional Unix time
 */
static inline int32_t mavlink_msg_rosflight_gnss_get_nanos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field fix_type from rosflight_gnss message
 *
 * @return  GNSS fix type
 */
static inline uint8_t mavlink_msg_rosflight_gnss_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field num_sat from rosflight_gnss message
 *
 * @return  Number of satellites seen
 */
static inline uint8_t mavlink_msg_rosflight_gnss_get_num_sat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  65);
}

/**
 * @brief Get field lat from rosflight_gnss message
 *
 * @return  In deg DDS format
 */
static inline double mavlink_msg_rosflight_gnss_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field lon from rosflight_gnss message
 *
 * @return  In deg DDs format
 */
static inline double mavlink_msg_rosflight_gnss_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field height from rosflight_gnss message
 *
 * @return  meters, MSL
 */
static inline float mavlink_msg_rosflight_gnss_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vel_n from rosflight_gnss message
 *
 * @return  meters per second
 */
static inline float mavlink_msg_rosflight_gnss_get_vel_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vel_e from rosflight_gnss message
 *
 * @return  meters per second
 */
static inline float mavlink_msg_rosflight_gnss_get_vel_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field vel_d from rosflight_gnss message
 *
 * @return  meters per second
 */
static inline float mavlink_msg_rosflight_gnss_get_vel_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field h_acc from rosflight_gnss message
 *
 * @return  meters
 */
static inline float mavlink_msg_rosflight_gnss_get_h_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field v_acc from rosflight_gnss message
 *
 * @return  meters
 */
static inline float mavlink_msg_rosflight_gnss_get_v_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field s_acc from rosflight_gnss message
 *
 * @return  meters
 */
static inline float mavlink_msg_rosflight_gnss_get_s_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field rosflight_timestamp from rosflight_gnss message
 *
 * @return  microseconds, estimated firmware timestamp for the time of validity of the gnss data
 */
static inline uint64_t mavlink_msg_rosflight_gnss_get_rosflight_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  24);
}

/**
 * @brief Decode a rosflight_gnss message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_gnss C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_gnss_decode(const mavlink_message_t* msg, mavlink_rosflight_gnss_t* rosflight_gnss)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rosflight_gnss->seconds = mavlink_msg_rosflight_gnss_get_seconds(msg);
    rosflight_gnss->lat = mavlink_msg_rosflight_gnss_get_lat(msg);
    rosflight_gnss->lon = mavlink_msg_rosflight_gnss_get_lon(msg);
    rosflight_gnss->rosflight_timestamp = mavlink_msg_rosflight_gnss_get_rosflight_timestamp(msg);
    rosflight_gnss->nanos = mavlink_msg_rosflight_gnss_get_nanos(msg);
    rosflight_gnss->height = mavlink_msg_rosflight_gnss_get_height(msg);
    rosflight_gnss->vel_n = mavlink_msg_rosflight_gnss_get_vel_n(msg);
    rosflight_gnss->vel_e = mavlink_msg_rosflight_gnss_get_vel_e(msg);
    rosflight_gnss->vel_d = mavlink_msg_rosflight_gnss_get_vel_d(msg);
    rosflight_gnss->h_acc = mavlink_msg_rosflight_gnss_get_h_acc(msg);
    rosflight_gnss->v_acc = mavlink_msg_rosflight_gnss_get_v_acc(msg);
    rosflight_gnss->s_acc = mavlink_msg_rosflight_gnss_get_s_acc(msg);
    rosflight_gnss->fix_type = mavlink_msg_rosflight_gnss_get_fix_type(msg);
    rosflight_gnss->num_sat = mavlink_msg_rosflight_gnss_get_num_sat(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN? msg->len : MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN;
        memset(rosflight_gnss, 0, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_LEN);
    memcpy(rosflight_gnss, _MAV_PAYLOAD(msg), len);
#endif
}
