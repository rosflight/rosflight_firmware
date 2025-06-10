// MESSAGE ROSFLIGHT_BATTERY_STATUS PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS 199

typedef struct __mavlink_rosflight_battery_status_t
{
 float battery_voltage; /*< */
 float battery_current; /*< */
} mavlink_rosflight_battery_status_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN 8
#define MAVLINK_MSG_ID_199_LEN 8

#define MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_CRC 48
#define MAVLINK_MSG_ID_199_CRC 48



#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_BATTERY_STATUS { \
	"ROSFLIGHT_BATTERY_STATUS", \
	2, \
	{  { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rosflight_battery_status_t, battery_voltage) }, \
         { "battery_current", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rosflight_battery_status_t, battery_current) }, \
         } \
}


/**
 * @brief Pack a rosflight_battery_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param battery_voltage 
 * @param battery_current 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_battery_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float battery_voltage, float battery_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN];
	_mav_put_float(buf, 0, battery_voltage);
	_mav_put_float(buf, 4, battery_current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#else
	mavlink_rosflight_battery_status_t packet;
	packet.battery_voltage = battery_voltage;
	packet.battery_current = battery_current;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif
}

/**
 * @brief Pack a rosflight_battery_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_voltage 
 * @param battery_current 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_battery_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float battery_voltage,float battery_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN];
	_mav_put_float(buf, 0, battery_voltage);
	_mav_put_float(buf, 4, battery_current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#else
	mavlink_rosflight_battery_status_t packet;
	packet.battery_voltage = battery_voltage;
	packet.battery_current = battery_current;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif
}

/**
 * @brief Encode a rosflight_battery_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_battery_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_battery_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_battery_status_t* rosflight_battery_status)
{
	return mavlink_msg_rosflight_battery_status_pack(system_id, component_id, msg, rosflight_battery_status->battery_voltage, rosflight_battery_status->battery_current);
}

/**
 * @brief Encode a rosflight_battery_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_battery_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_battery_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_battery_status_t* rosflight_battery_status)
{
	return mavlink_msg_rosflight_battery_status_pack_chan(system_id, component_id, chan, msg, rosflight_battery_status->battery_voltage, rosflight_battery_status->battery_current);
}

/**
 * @brief Send a rosflight_battery_status message
 * @param chan MAVLink channel to send the message
 *
 * @param battery_voltage 
 * @param battery_current 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_battery_status_send(mavlink_channel_t chan, float battery_voltage, float battery_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN];
	_mav_put_float(buf, 0, battery_voltage);
	_mav_put_float(buf, 4, battery_current);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif
#else
	mavlink_rosflight_battery_status_t packet;
	packet.battery_voltage = battery_voltage;
	packet.battery_current = battery_current;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_battery_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float battery_voltage, float battery_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, battery_voltage);
	_mav_put_float(buf, 4, battery_current);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, buf, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif
#else
	mavlink_rosflight_battery_status_t *packet = (mavlink_rosflight_battery_status_t *)msgbuf;
	packet->battery_voltage = battery_voltage;
	packet->battery_current = battery_current;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_BATTERY_STATUS UNPACKING


/**
 * @brief Get field battery_voltage from rosflight_battery_status message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_battery_status_get_battery_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field battery_current from rosflight_battery_status message
 *
 * @return 
 */
static inline float mavlink_msg_rosflight_battery_status_get_battery_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a rosflight_battery_status message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_battery_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_battery_status_decode(const mavlink_message_t* msg, mavlink_rosflight_battery_status_t* rosflight_battery_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_battery_status->battery_voltage = mavlink_msg_rosflight_battery_status_get_battery_voltage(msg);
	rosflight_battery_status->battery_current = mavlink_msg_rosflight_battery_status_get_battery_current(msg);
#else
	memcpy(rosflight_battery_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_BATTERY_STATUS_LEN);
#endif
}
