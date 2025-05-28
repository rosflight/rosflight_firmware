// MESSAGE ROSFLIGHT_CONFIG PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_CONFIG 201

typedef struct __mavlink_rosflight_config_t
{
 uint8_t device; /*< */
 uint8_t config; /*< */
} mavlink_rosflight_config_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN 2
#define MAVLINK_MSG_ID_201_LEN 2

#define MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_CRC 248
#define MAVLINK_MSG_ID_201_CRC 248



#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_CONFIG { \
	"ROSFLIGHT_CONFIG", \
	2, \
	{  { "device", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rosflight_config_t, device) }, \
         { "config", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rosflight_config_t, config) }, \
         } \
}


/**
 * @brief Pack a rosflight_config message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param device 
 * @param config 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_config_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t device, uint8_t config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN];
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, config);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#else
	mavlink_rosflight_config_t packet;
	packet.device = device;
	packet.config = config;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CONFIG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif
}

/**
 * @brief Pack a rosflight_config message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device 
 * @param config 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_config_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t device,uint8_t config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN];
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, config);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#else
	mavlink_rosflight_config_t packet;
	packet.device = device;
	packet.config = config;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_CONFIG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif
}

/**
 * @brief Encode a rosflight_config struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_config_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_config_t* rosflight_config)
{
	return mavlink_msg_rosflight_config_pack(system_id, component_id, msg, rosflight_config->device, rosflight_config->config);
}

/**
 * @brief Encode a rosflight_config struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_config_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_config_t* rosflight_config)
{
	return mavlink_msg_rosflight_config_pack_chan(system_id, component_id, chan, msg, rosflight_config->device, rosflight_config->config);
}

/**
 * @brief Send a rosflight_config message
 * @param chan MAVLink channel to send the message
 *
 * @param device 
 * @param config 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_config_send(mavlink_channel_t chan, uint8_t device, uint8_t config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN];
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, config);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif
#else
	mavlink_rosflight_config_t packet;
	packet.device = device;
	packet.config = config;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_config_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t device, uint8_t config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, device);
	_mav_put_uint8_t(buf, 1, config);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, buf, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif
#else
	mavlink_rosflight_config_t *packet = (mavlink_rosflight_config_t *)msgbuf;
	packet->device = device;
	packet->config = config;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_CONFIG UNPACKING


/**
 * @brief Get field device from rosflight_config message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_config_get_device(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field config from rosflight_config message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_config_get_config(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a rosflight_config message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_config C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_config_decode(const mavlink_message_t* msg, mavlink_rosflight_config_t* rosflight_config)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_config->device = mavlink_msg_rosflight_config_get_device(msg);
	rosflight_config->config = mavlink_msg_rosflight_config_get_config(msg);
#else
	memcpy(rosflight_config, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_CONFIG_LEN);
#endif
}
