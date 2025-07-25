#pragma once
// MESSAGE ROSFLIGHT_CMD PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_CMD 188


typedef struct __mavlink_rosflight_cmd_t {
 uint8_t command; /*<  */
} mavlink_rosflight_cmd_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN 1
#define MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN 1
#define MAVLINK_MSG_ID_188_LEN 1
#define MAVLINK_MSG_ID_188_MIN_LEN 1

#define MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC 249
#define MAVLINK_MSG_ID_188_CRC 249



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_CMD { \
    188, \
    "ROSFLIGHT_CMD", \
    1, \
    {  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rosflight_cmd_t, command) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_CMD { \
    "ROSFLIGHT_CMD", \
    1, \
    {  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rosflight_cmd_t, command) }, \
         } \
}
#endif

/**
 * @brief Pack a rosflight_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN];
    _mav_put_uint8_t(buf, 0, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
#else
    mavlink_rosflight_cmd_t packet;
    packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
}

/**
 * @brief Pack a rosflight_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param command  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_cmd_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN];
    _mav_put_uint8_t(buf, 0, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
#else
    mavlink_rosflight_cmd_t packet;
    packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CMD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
#endif
}

/**
 * @brief Pack a rosflight_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN];
    _mav_put_uint8_t(buf, 0, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
#else
    mavlink_rosflight_cmd_t packet;
    packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
}

/**
 * @brief Encode a rosflight_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_cmd_t* rosflight_cmd)
{
    return mavlink_msg_rosflight_cmd_pack(system_id, component_id, msg, rosflight_cmd->command);
}

/**
 * @brief Encode a rosflight_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_cmd_t* rosflight_cmd)
{
    return mavlink_msg_rosflight_cmd_pack_chan(system_id, component_id, chan, msg, rosflight_cmd->command);
}

/**
 * @brief Encode a rosflight_cmd struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_cmd_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rosflight_cmd_t* rosflight_cmd)
{
    return mavlink_msg_rosflight_cmd_pack_status(system_id, component_id, _status, msg,  rosflight_cmd->command);
}

/**
 * @brief Send a rosflight_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param command  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_cmd_send(mavlink_channel_t chan, uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN];
    _mav_put_uint8_t(buf, 0, command);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD, buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
#else
    mavlink_rosflight_cmd_t packet;
    packet.command = command;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
#endif
}

/**
 * @brief Send a rosflight_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rosflight_cmd_send_struct(mavlink_channel_t chan, const mavlink_rosflight_cmd_t* rosflight_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rosflight_cmd_send(chan, rosflight_cmd->command);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD, (const char *)rosflight_cmd, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, command);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD, buf, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
#else
    mavlink_rosflight_cmd_t *packet = (mavlink_rosflight_cmd_t *)msgbuf;
    packet->command = command;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CMD, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_CMD_MIN_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_CMD UNPACKING


/**
 * @brief Get field command from rosflight_cmd message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_rosflight_cmd_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a rosflight_cmd message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_cmd_decode(const mavlink_message_t* msg, mavlink_rosflight_cmd_t* rosflight_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rosflight_cmd->command = mavlink_msg_rosflight_cmd_get_command(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN? msg->len : MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN;
        memset(rosflight_cmd, 0, MAVLINK_MSG_ID_ROSFLIGHT_CMD_LEN);
    memcpy(rosflight_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
