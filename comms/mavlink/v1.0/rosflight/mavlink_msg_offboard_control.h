#pragma once
// MESSAGE OFFBOARD_CONTROL PACKING

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL 180


typedef struct __mavlink_offboard_control_t {
 float u[10]; /*<  u control channel interpreted according to mode*/
 uint16_t ignore; /*<  Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE*/
 uint8_t mode; /*<  Offboard control mode, see OFFBOARD_CONTROL_MODE*/
} mavlink_offboard_control_t;

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN 43
#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN 43
#define MAVLINK_MSG_ID_180_LEN 43
#define MAVLINK_MSG_ID_180_MIN_LEN 43

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC 90
#define MAVLINK_MSG_ID_180_CRC 90

#define MAVLINK_MSG_OFFBOARD_CONTROL_FIELD_U_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL { \
    180, \
    "OFFBOARD_CONTROL", \
    3, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_offboard_control_t, mode) }, \
         { "ignore", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_offboard_control_t, ignore) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 10, 0, offsetof(mavlink_offboard_control_t, u) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL { \
    "OFFBOARD_CONTROL", \
    3, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_offboard_control_t, mode) }, \
         { "ignore", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_offboard_control_t, ignore) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 10, 0, offsetof(mavlink_offboard_control_t, u) }, \
         } \
}
#endif

/**
 * @brief Pack a offboard_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore  Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param u  u control channel interpreted according to mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mode, uint16_t ignore, const float *u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
    _mav_put_uint16_t(buf, 40, ignore);
    _mav_put_uint8_t(buf, 42, mode);
    _mav_put_float_array(buf, 0, u, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
    mavlink_offboard_control_t packet;
    packet.ignore = ignore;
    packet.mode = mode;
    mav_array_memcpy(packet.u, u, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
}

/**
 * @brief Pack a offboard_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore  Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param u  u control channel interpreted according to mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t mode, uint16_t ignore, const float *u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
    _mav_put_uint16_t(buf, 40, ignore);
    _mav_put_uint8_t(buf, 42, mode);
    _mav_put_float_array(buf, 0, u, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
    mavlink_offboard_control_t packet;
    packet.ignore = ignore;
    packet.mode = mode;
    mav_array_memcpy(packet.u, u, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a offboard_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore  Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param u  u control channel interpreted according to mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mode,uint16_t ignore,const float *u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
    _mav_put_uint16_t(buf, 40, ignore);
    _mav_put_uint8_t(buf, 42, mode);
    _mav_put_float_array(buf, 0, u, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
    mavlink_offboard_control_t packet;
    packet.ignore = ignore;
    packet.mode = mode;
    mav_array_memcpy(packet.u, u, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
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
    return mavlink_msg_offboard_control_pack(system_id, component_id, msg, offboard_control->mode, offboard_control->ignore, offboard_control->u);
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
    return mavlink_msg_offboard_control_pack_chan(system_id, component_id, chan, msg, offboard_control->mode, offboard_control->ignore, offboard_control->u);
}

/**
 * @brief Encode a offboard_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param offboard_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_offboard_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_offboard_control_t* offboard_control)
{
    return mavlink_msg_offboard_control_pack_status(system_id, component_id, _status, msg,  offboard_control->mode, offboard_control->ignore, offboard_control->u);
}

/**
 * @brief Send a offboard_control message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore  Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param u  u control channel interpreted according to mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_offboard_control_send(mavlink_channel_t chan, uint8_t mode, uint16_t ignore, const float *u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
    _mav_put_uint16_t(buf, 40, ignore);
    _mav_put_uint8_t(buf, 42, mode);
    _mav_put_float_array(buf, 0, u, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    mavlink_offboard_control_t packet;
    packet.ignore = ignore;
    packet.mode = mode;
    mav_array_memcpy(packet.u, u, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#endif
}

/**
 * @brief Send a offboard_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_offboard_control_send_struct(mavlink_channel_t chan, const mavlink_offboard_control_t* offboard_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_offboard_control_send(chan, offboard_control->mode, offboard_control->ignore, offboard_control->u);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)offboard_control, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_offboard_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode, uint16_t ignore, const float *u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 40, ignore);
    _mav_put_uint8_t(buf, 42, mode);
    _mav_put_float_array(buf, 0, u, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    mavlink_offboard_control_t *packet = (mavlink_offboard_control_t *)msgbuf;
    packet->ignore = ignore;
    packet->mode = mode;
    mav_array_memcpy(packet->u, u, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE OFFBOARD_CONTROL UNPACKING


/**
 * @brief Get field mode from offboard_control message
 *
 * @return  Offboard control mode, see OFFBOARD_CONTROL_MODE
 */
static inline uint8_t mavlink_msg_offboard_control_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field ignore from offboard_control message
 *
 * @return  Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 */
static inline uint16_t mavlink_msg_offboard_control_get_ignore(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field u from offboard_control message
 *
 * @return  u control channel interpreted according to mode
 */
static inline uint16_t mavlink_msg_offboard_control_get_u(const mavlink_message_t* msg, float *u)
{
    return _MAV_RETURN_float_array(msg, u, 10,  0);
}

/**
 * @brief Decode a offboard_control message into a struct
 *
 * @param msg The message to decode
 * @param offboard_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_offboard_control_decode(const mavlink_message_t* msg, mavlink_offboard_control_t* offboard_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_offboard_control_get_u(msg, offboard_control->u);
    offboard_control->ignore = mavlink_msg_offboard_control_get_ignore(msg);
    offboard_control->mode = mavlink_msg_offboard_control_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN;
        memset(offboard_control, 0, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
    memcpy(offboard_control, _MAV_PAYLOAD(msg), len);
#endif
}
