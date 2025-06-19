#pragma once
// MESSAGE ROSFLIGHT_VERSION PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_VERSION 192


typedef struct __mavlink_rosflight_version_t {
 char version[50]; /*<  */
} mavlink_rosflight_version_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN 50
#define MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN 50
#define MAVLINK_MSG_ID_192_LEN 50
#define MAVLINK_MSG_ID_192_MIN_LEN 50

#define MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC 134
#define MAVLINK_MSG_ID_192_CRC 134

#define MAVLINK_MSG_ROSFLIGHT_VERSION_FIELD_VERSION_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_VERSION { \
    192, \
    "ROSFLIGHT_VERSION", \
    1, \
    {  { "version", NULL, MAVLINK_TYPE_CHAR, 50, 0, offsetof(mavlink_rosflight_version_t, version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_VERSION { \
    "ROSFLIGHT_VERSION", \
    1, \
    {  { "version", NULL, MAVLINK_TYPE_CHAR, 50, 0, offsetof(mavlink_rosflight_version_t, version) }, \
         } \
}
#endif

/**
 * @brief Pack a rosflight_version message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param version  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_version_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN];

    _mav_put_char_array(buf, 0, version, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
#else
    mavlink_rosflight_version_t packet;

    mav_array_memcpy(packet.version, version, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_VERSION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
}

/**
 * @brief Pack a rosflight_version message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param version  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_version_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN];

    _mav_put_char_array(buf, 0, version, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
#else
    mavlink_rosflight_version_t packet;

    mav_array_memcpy(packet.version, version, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_VERSION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
#endif
}

/**
 * @brief Pack a rosflight_version message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param version  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_version_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN];

    _mav_put_char_array(buf, 0, version, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
#else
    mavlink_rosflight_version_t packet;

    mav_array_memcpy(packet.version, version, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_VERSION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
}

/**
 * @brief Encode a rosflight_version struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_version_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_version_t* rosflight_version)
{
    return mavlink_msg_rosflight_version_pack(system_id, component_id, msg, rosflight_version->version);
}

/**
 * @brief Encode a rosflight_version struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_version_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_version_t* rosflight_version)
{
    return mavlink_msg_rosflight_version_pack_chan(system_id, component_id, chan, msg, rosflight_version->version);
}

/**
 * @brief Encode a rosflight_version struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_version C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_version_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rosflight_version_t* rosflight_version)
{
    return mavlink_msg_rosflight_version_pack_status(system_id, component_id, _status, msg,  rosflight_version->version);
}

/**
 * @brief Send a rosflight_version message
 * @param chan MAVLink channel to send the message
 *
 * @param version  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_version_send(mavlink_channel_t chan, const char *version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN];

    _mav_put_char_array(buf, 0, version, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_VERSION, buf, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
#else
    mavlink_rosflight_version_t packet;

    mav_array_memcpy(packet.version, version, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_VERSION, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
#endif
}

/**
 * @brief Send a rosflight_version message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rosflight_version_send_struct(mavlink_channel_t chan, const mavlink_rosflight_version_t* rosflight_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rosflight_version_send(chan, rosflight_version->version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_VERSION, (const char *)rosflight_version, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_version_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, version, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_VERSION, buf, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
#else
    mavlink_rosflight_version_t *packet = (mavlink_rosflight_version_t *)msgbuf;

    mav_array_memcpy(packet->version, version, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_VERSION, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_CRC);
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_VERSION UNPACKING


/**
 * @brief Get field version from rosflight_version message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_rosflight_version_get_version(const mavlink_message_t* msg, char *version)
{
    return _MAV_RETURN_char_array(msg, version, 50,  0);
}

/**
 * @brief Decode a rosflight_version message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_version C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_version_decode(const mavlink_message_t* msg, mavlink_rosflight_version_t* rosflight_version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rosflight_version_get_version(msg, rosflight_version->version);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN? msg->len : MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN;
        memset(rosflight_version, 0, MAVLINK_MSG_ID_ROSFLIGHT_VERSION_LEN);
    memcpy(rosflight_version, _MAV_PAYLOAD(msg), len);
#endif
}
