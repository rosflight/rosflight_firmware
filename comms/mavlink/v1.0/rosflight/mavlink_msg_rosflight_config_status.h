// MESSAGE ROSFLIGHT_CONFIG_STATUS PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS 204

typedef struct __mavlink_rosflight_config_status_t
{
 uint8_t device; /*< */
 uint8_t success; /*< */
 uint8_t reboot_required; /*< */
 uint8_t error_message[50]; /*< */
} mavlink_rosflight_config_status_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN 53
#define MAVLINK_MSG_ID_204_LEN 53

#define MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_CRC 116
#define MAVLINK_MSG_ID_204_CRC 116

#define MAVLINK_MSG_ROSFLIGHT_CONFIG_STATUS_FIELD_ERROR_MESSAGE_LEN 50

#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_CONFIG_STATUS { \
	"ROSFLIGHT_CONFIG_STATUS", \
	4, \
	{  { "device", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rosflight_config_status_t, device) }, \
         { "success", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rosflight_config_status_t, success) }, \
         { "reboot_required", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rosflight_config_status_t, reboot_required) }, \
         { "error_message", NULL, MAVLINK_TYPE_UINT8_T, 50, 3, offsetof(mavlink_rosflight_config_status_t, error_message) }, \
         } \
}


/**
 * @brief Pack a rosflight_config_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param device 
 * @param success 
 * @param reboot_required 
 * @param error_message 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_config_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t device, uint8_t success, uint8_t reboot_required, const uint8_t *error_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, success);
	_mav_put_uint8_t(buf, 2, reboot_required);
	_mav_put_uint8_t_array(buf, 3, error_message, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#else
	mavlink_rosflight_config_status_t packet;
	packet.device = device;
	packet.success = success;
	packet.reboot_required = reboot_required;
	mav_array_memcpy(packet.error_message, error_message, sizeof(uint8_t)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif
}

/**
 * @brief Pack a rosflight_config_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device 
 * @param success 
 * @param reboot_required 
 * @param error_message 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_config_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t device,uint8_t success,uint8_t reboot_required,const uint8_t *error_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, success);
	_mav_put_uint8_t(buf, 2, reboot_required);
	_mav_put_uint8_t_array(buf, 3, error_message, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#else
	mavlink_rosflight_config_status_t packet;
	packet.device = device;
	packet.success = success;
	packet.reboot_required = reboot_required;
	mav_array_memcpy(packet.error_message, error_message, sizeof(uint8_t)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif
}

/**
 * @brief Encode a rosflight_config_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_config_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_config_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_config_status_t* rosflight_config_status)
{
	return mavlink_msg_rosflight_config_status_pack(system_id, component_id, msg, rosflight_config_status->device, rosflight_config_status->success, rosflight_config_status->reboot_required, rosflight_config_status->error_message);
}

/**
 * @brief Encode a rosflight_config_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_config_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_config_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_config_status_t* rosflight_config_status)
{
	return mavlink_msg_rosflight_config_status_pack_chan(system_id, component_id, chan, msg, rosflight_config_status->device, rosflight_config_status->success, rosflight_config_status->reboot_required, rosflight_config_status->error_message);
}

/**
 * @brief Send a rosflight_config_status message
 * @param chan MAVLink channel to send the message
 *
 * @param device 
 * @param success 
 * @param reboot_required 
 * @param error_message 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_config_status_send(mavlink_channel_t chan, uint8_t device, uint8_t success, uint8_t reboot_required, const uint8_t *error_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, success);
	_mav_put_uint8_t(buf, 2, reboot_required);
	_mav_put_uint8_t_array(buf, 3, error_message, 50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif
#else
	mavlink_rosflight_config_status_t packet;
	packet.device = device;
	packet.success = success;
	packet.reboot_required = reboot_required;
	mav_array_memcpy(packet.error_message, error_message, sizeof(uint8_t)*50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_config_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t device, uint8_t success, uint8_t reboot_required, const uint8_t *error_message)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, success);
	_mav_put_uint8_t(buf, 2, reboot_required);
	_mav_put_uint8_t_array(buf, 3, error_message, 50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif
#else
	mavlink_rosflight_config_status_t *packet = (mavlink_rosflight_config_status_t *)msgbuf;
	packet->device = device;
	packet->success = success;
	packet->reboot_required = reboot_required;
	mav_array_memcpy(packet->error_message, error_message, sizeof(uint8_t)*50);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_CONFIG_STATUS UNPACKING


/**
 * @brief Get field device from rosflight_config_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_config_status_get_device(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field success from rosflight_config_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_config_status_get_success(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field reboot_required from rosflight_config_status message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_config_status_get_reboot_required(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field error_message from rosflight_config_status message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_rosflight_config_status_get_error_message(const mavlink_message_t* msg, uint8_t *error_message)
{
	return _MAV_RETURN_uint8_t_array(msg, error_message, 50,  3);
}

/**
 * @brief Decode a rosflight_config_status message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_config_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_config_status_decode(const mavlink_message_t* msg, mavlink_rosflight_config_status_t* rosflight_config_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_config_status->device = mavlink_msg_rosflight_config_status_get_device(msg);
	rosflight_config_status->success = mavlink_msg_rosflight_config_status_get_success(msg);
	rosflight_config_status->reboot_required = mavlink_msg_rosflight_config_status_get_reboot_required(msg);
	mavlink_msg_rosflight_config_status_get_error_message(msg, rosflight_config_status->error_message);
#else
	memcpy(rosflight_config_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_STATUS_LEN);
#endif
}
