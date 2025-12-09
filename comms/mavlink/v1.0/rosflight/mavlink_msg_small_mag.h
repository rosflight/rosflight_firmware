#pragma once
// MESSAGE SMALL_MAG PACKING

#define MAVLINK_MSG_ID_SMALL_MAG 182


typedef struct __mavlink_small_mag_t {
 float xmag; /*<  Magnetic field along X axis*/
 float ymag; /*<  Magnetic field along Y axis*/
 float zmag; /*<  Magnetic field along Z axis*/
} mavlink_small_mag_t;

#define MAVLINK_MSG_ID_SMALL_MAG_LEN 12
#define MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN 12
#define MAVLINK_MSG_ID_182_LEN 12
#define MAVLINK_MSG_ID_182_MIN_LEN 12

#define MAVLINK_MSG_ID_SMALL_MAG_CRC 218
#define MAVLINK_MSG_ID_182_CRC 218



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SMALL_MAG { \
    182, \
    "SMALL_MAG", \
    3, \
    {  { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_small_mag_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_small_mag_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_small_mag_t, zmag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SMALL_MAG { \
    "SMALL_MAG", \
    3, \
    {  { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_small_mag_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_small_mag_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_small_mag_t, zmag) }, \
         } \
}
#endif

/**
 * @brief Pack a small_mag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param xmag  Magnetic field along X axis
 * @param ymag  Magnetic field along Y axis
 * @param zmag  Magnetic field along Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_mag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMALL_MAG_LEN];
    _mav_put_float(buf, 0, xmag);
    _mav_put_float(buf, 4, ymag);
    _mav_put_float(buf, 8, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_MAG_LEN);
#else
    mavlink_small_mag_t packet;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SMALL_MAG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
}

/**
 * @brief Pack a small_mag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param xmag  Magnetic field along X axis
 * @param ymag  Magnetic field along Y axis
 * @param zmag  Magnetic field along Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_mag_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMALL_MAG_LEN];
    _mav_put_float(buf, 0, xmag);
    _mav_put_float(buf, 4, ymag);
    _mav_put_float(buf, 8, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_MAG_LEN);
#else
    mavlink_small_mag_t packet;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SMALL_MAG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN);
#endif
}

/**
 * @brief Pack a small_mag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param xmag  Magnetic field along X axis
 * @param ymag  Magnetic field along Y axis
 * @param zmag  Magnetic field along Z axis
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_mag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float xmag,float ymag,float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMALL_MAG_LEN];
    _mav_put_float(buf, 0, xmag);
    _mav_put_float(buf, 4, ymag);
    _mav_put_float(buf, 8, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_MAG_LEN);
#else
    mavlink_small_mag_t packet;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SMALL_MAG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
}

/**
 * @brief Encode a small_mag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param small_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_mag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_small_mag_t* small_mag)
{
    return mavlink_msg_small_mag_pack(system_id, component_id, msg, small_mag->xmag, small_mag->ymag, small_mag->zmag);
}

/**
 * @brief Encode a small_mag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param small_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_mag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_small_mag_t* small_mag)
{
    return mavlink_msg_small_mag_pack_chan(system_id, component_id, chan, msg, small_mag->xmag, small_mag->ymag, small_mag->zmag);
}

/**
 * @brief Encode a small_mag struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param small_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_mag_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_small_mag_t* small_mag)
{
    return mavlink_msg_small_mag_pack_status(system_id, component_id, _status, msg,  small_mag->xmag, small_mag->ymag, small_mag->zmag);
}

/**
 * @brief Send a small_mag message
 * @param chan MAVLink channel to send the message
 *
 * @param xmag  Magnetic field along X axis
 * @param ymag  Magnetic field along Y axis
 * @param zmag  Magnetic field along Z axis
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_small_mag_send(mavlink_channel_t chan, float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMALL_MAG_LEN];
    _mav_put_float(buf, 0, xmag);
    _mav_put_float(buf, 4, ymag);
    _mav_put_float(buf, 8, zmag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_MAG, buf, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
#else
    mavlink_small_mag_t packet;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_MAG, (const char *)&packet, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
#endif
}

/**
 * @brief Send a small_mag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_small_mag_send_struct(mavlink_channel_t chan, const mavlink_small_mag_t* small_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_small_mag_send(chan, small_mag->xmag, small_mag->ymag, small_mag->zmag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_MAG, (const char *)small_mag, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
#endif
}

#if MAVLINK_MSG_ID_SMALL_MAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_small_mag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, xmag);
    _mav_put_float(buf, 4, ymag);
    _mav_put_float(buf, 8, zmag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_MAG, buf, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
#else
    mavlink_small_mag_t *packet = (mavlink_small_mag_t *)msgbuf;
    packet->xmag = xmag;
    packet->ymag = ymag;
    packet->zmag = zmag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_MAG, (const char *)packet, MAVLINK_MSG_ID_SMALL_MAG_MIN_LEN, MAVLINK_MSG_ID_SMALL_MAG_LEN, MAVLINK_MSG_ID_SMALL_MAG_CRC);
#endif
}
#endif

#endif

// MESSAGE SMALL_MAG UNPACKING


/**
 * @brief Get field xmag from small_mag message
 *
 * @return  Magnetic field along X axis
 */
static inline float mavlink_msg_small_mag_get_xmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ymag from small_mag message
 *
 * @return  Magnetic field along Y axis
 */
static inline float mavlink_msg_small_mag_get_ymag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field zmag from small_mag message
 *
 * @return  Magnetic field along Z axis
 */
static inline float mavlink_msg_small_mag_get_zmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a small_mag message into a struct
 *
 * @param msg The message to decode
 * @param small_mag C-struct to decode the message contents into
 */
static inline void mavlink_msg_small_mag_decode(const mavlink_message_t* msg, mavlink_small_mag_t* small_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    small_mag->xmag = mavlink_msg_small_mag_get_xmag(msg);
    small_mag->ymag = mavlink_msg_small_mag_get_ymag(msg);
    small_mag->zmag = mavlink_msg_small_mag_get_zmag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SMALL_MAG_LEN? msg->len : MAVLINK_MSG_ID_SMALL_MAG_LEN;
        memset(small_mag, 0, MAVLINK_MSG_ID_SMALL_MAG_LEN);
    memcpy(small_mag, _MAV_PAYLOAD(msg), len);
#endif
}
