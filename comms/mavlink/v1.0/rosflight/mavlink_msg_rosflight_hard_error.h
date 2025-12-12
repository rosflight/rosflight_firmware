#pragma once
// MESSAGE ROSFLIGHT_HARD_ERROR PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR 196


typedef struct __mavlink_rosflight_hard_error_t {
 uint32_t error_code; /*<  */
 uint32_t pc; /*<  */
 uint32_t reset_count; /*<  */
 uint32_t doRearm; /*<  */
} mavlink_rosflight_hard_error_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN 16
#define MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN 16
#define MAVLINK_MSG_ID_196_LEN 16
#define MAVLINK_MSG_ID_196_MIN_LEN 16

#define MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC 10
#define MAVLINK_MSG_ID_196_CRC 10



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_HARD_ERROR { \
    196, \
    "ROSFLIGHT_HARD_ERROR", \
    4, \
    {  { "error_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rosflight_hard_error_t, error_code) }, \
         { "pc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_rosflight_hard_error_t, pc) }, \
         { "reset_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_rosflight_hard_error_t, reset_count) }, \
         { "doRearm", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_rosflight_hard_error_t, doRearm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_HARD_ERROR { \
    "ROSFLIGHT_HARD_ERROR", \
    4, \
    {  { "error_code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rosflight_hard_error_t, error_code) }, \
         { "pc", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_rosflight_hard_error_t, pc) }, \
         { "reset_count", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_rosflight_hard_error_t, reset_count) }, \
         { "doRearm", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_rosflight_hard_error_t, doRearm) }, \
         } \
}
#endif

/**
 * @brief Pack a rosflight_hard_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param error_code  
 * @param pc  
 * @param reset_count  
 * @param doRearm  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_hard_error_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t error_code, uint32_t pc, uint32_t reset_count, uint32_t doRearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, pc);
    _mav_put_uint32_t(buf, 8, reset_count);
    _mav_put_uint32_t(buf, 12, doRearm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
#else
    mavlink_rosflight_hard_error_t packet;
    packet.error_code = error_code;
    packet.pc = pc;
    packet.reset_count = reset_count;
    packet.doRearm = doRearm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
}

/**
 * @brief Pack a rosflight_hard_error message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param error_code  
 * @param pc  
 * @param reset_count  
 * @param doRearm  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_hard_error_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t error_code, uint32_t pc, uint32_t reset_count, uint32_t doRearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, pc);
    _mav_put_uint32_t(buf, 8, reset_count);
    _mav_put_uint32_t(buf, 12, doRearm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
#else
    mavlink_rosflight_hard_error_t packet;
    packet.error_code = error_code;
    packet.pc = pc;
    packet.reset_count = reset_count;
    packet.doRearm = doRearm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
#endif
}

/**
 * @brief Pack a rosflight_hard_error message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param error_code  
 * @param pc  
 * @param reset_count  
 * @param doRearm  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_hard_error_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t error_code,uint32_t pc,uint32_t reset_count,uint32_t doRearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, pc);
    _mav_put_uint32_t(buf, 8, reset_count);
    _mav_put_uint32_t(buf, 12, doRearm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
#else
    mavlink_rosflight_hard_error_t packet;
    packet.error_code = error_code;
    packet.pc = pc;
    packet.reset_count = reset_count;
    packet.doRearm = doRearm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
}

/**
 * @brief Encode a rosflight_hard_error struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_hard_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_hard_error_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_hard_error_t* rosflight_hard_error)
{
    return mavlink_msg_rosflight_hard_error_pack(system_id, component_id, msg, rosflight_hard_error->error_code, rosflight_hard_error->pc, rosflight_hard_error->reset_count, rosflight_hard_error->doRearm);
}

/**
 * @brief Encode a rosflight_hard_error struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_hard_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_hard_error_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_hard_error_t* rosflight_hard_error)
{
    return mavlink_msg_rosflight_hard_error_pack_chan(system_id, component_id, chan, msg, rosflight_hard_error->error_code, rosflight_hard_error->pc, rosflight_hard_error->reset_count, rosflight_hard_error->doRearm);
}

/**
 * @brief Encode a rosflight_hard_error struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_hard_error C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_hard_error_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rosflight_hard_error_t* rosflight_hard_error)
{
    return mavlink_msg_rosflight_hard_error_pack_status(system_id, component_id, _status, msg,  rosflight_hard_error->error_code, rosflight_hard_error->pc, rosflight_hard_error->reset_count, rosflight_hard_error->doRearm);
}

/**
 * @brief Send a rosflight_hard_error message
 * @param chan MAVLink channel to send the message
 *
 * @param error_code  
 * @param pc  
 * @param reset_count  
 * @param doRearm  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_hard_error_send(mavlink_channel_t chan, uint32_t error_code, uint32_t pc, uint32_t reset_count, uint32_t doRearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN];
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, pc);
    _mav_put_uint32_t(buf, 8, reset_count);
    _mav_put_uint32_t(buf, 12, doRearm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR, buf, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
#else
    mavlink_rosflight_hard_error_t packet;
    packet.error_code = error_code;
    packet.pc = pc;
    packet.reset_count = reset_count;
    packet.doRearm = doRearm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
#endif
}

/**
 * @brief Send a rosflight_hard_error message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rosflight_hard_error_send_struct(mavlink_channel_t chan, const mavlink_rosflight_hard_error_t* rosflight_hard_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rosflight_hard_error_send(chan, rosflight_hard_error->error_code, rosflight_hard_error->pc, rosflight_hard_error->reset_count, rosflight_hard_error->doRearm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR, (const char *)rosflight_hard_error, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_hard_error_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t error_code, uint32_t pc, uint32_t reset_count, uint32_t doRearm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, error_code);
    _mav_put_uint32_t(buf, 4, pc);
    _mav_put_uint32_t(buf, 8, reset_count);
    _mav_put_uint32_t(buf, 12, doRearm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR, buf, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
#else
    mavlink_rosflight_hard_error_t *packet = (mavlink_rosflight_hard_error_t *)msgbuf;
    packet->error_code = error_code;
    packet->pc = pc;
    packet->reset_count = reset_count;
    packet->doRearm = doRearm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_CRC);
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_HARD_ERROR UNPACKING


/**
 * @brief Get field error_code from rosflight_hard_error message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_rosflight_hard_error_get_error_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field pc from rosflight_hard_error message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_rosflight_hard_error_get_pc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field reset_count from rosflight_hard_error message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_rosflight_hard_error_get_reset_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field doRearm from rosflight_hard_error message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_rosflight_hard_error_get_doRearm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a rosflight_hard_error message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_hard_error C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_hard_error_decode(const mavlink_message_t* msg, mavlink_rosflight_hard_error_t* rosflight_hard_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rosflight_hard_error->error_code = mavlink_msg_rosflight_hard_error_get_error_code(msg);
    rosflight_hard_error->pc = mavlink_msg_rosflight_hard_error_get_pc(msg);
    rosflight_hard_error->reset_count = mavlink_msg_rosflight_hard_error_get_reset_count(msg);
    rosflight_hard_error->doRearm = mavlink_msg_rosflight_hard_error_get_doRearm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN? msg->len : MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN;
        memset(rosflight_hard_error, 0, MAVLINK_MSG_ID_ROSFLIGHT_HARD_ERROR_LEN);
    memcpy(rosflight_hard_error, _MAV_PAYLOAD(msg), len);
#endif
}
