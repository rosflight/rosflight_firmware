// MESSAGE SMALL_BARO PACKING

#define MAVLINK_MSG_ID_SMALL_BARO 183

typedef struct __mavlink_small_baro_t
{
 float altitude; /*< Calculated Altitude (m)*/
 float pressure; /*< Measured Differential Pressure (Pa)*/
 float temperature; /*< Measured Temperature (K)*/
} mavlink_small_baro_t;

#define MAVLINK_MSG_ID_SMALL_BARO_LEN 12
#define MAVLINK_MSG_ID_183_LEN 12

#define MAVLINK_MSG_ID_SMALL_BARO_CRC 206
#define MAVLINK_MSG_ID_183_CRC 206



#define MAVLINK_MESSAGE_INFO_SMALL_BARO { \
	"SMALL_BARO", \
	3, \
	{  { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_small_baro_t, altitude) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_small_baro_t, pressure) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_small_baro_t, temperature) }, \
         } \
}


/**
 * @brief Pack a small_baro message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param altitude Calculated Altitude (m)
 * @param pressure Measured Differential Pressure (Pa)
 * @param temperature Measured Temperature (K)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_baro_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float altitude, float pressure, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_BARO_LEN];
	_mav_put_float(buf, 0, altitude);
	_mav_put_float(buf, 4, pressure);
	_mav_put_float(buf, 8, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#else
	mavlink_small_baro_t packet;
	packet.altitude = altitude;
	packet.pressure = pressure;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SMALL_BARO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMALL_BARO_LEN, MAVLINK_MSG_ID_SMALL_BARO_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif
}

/**
 * @brief Pack a small_baro message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param altitude Calculated Altitude (m)
 * @param pressure Measured Differential Pressure (Pa)
 * @param temperature Measured Temperature (K)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_small_baro_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float altitude,float pressure,float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_BARO_LEN];
	_mav_put_float(buf, 0, altitude);
	_mav_put_float(buf, 4, pressure);
	_mav_put_float(buf, 8, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#else
	mavlink_small_baro_t packet;
	packet.altitude = altitude;
	packet.pressure = pressure;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SMALL_BARO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMALL_BARO_LEN, MAVLINK_MSG_ID_SMALL_BARO_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif
}

/**
 * @brief Encode a small_baro struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param small_baro C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_baro_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_small_baro_t* small_baro)
{
	return mavlink_msg_small_baro_pack(system_id, component_id, msg, small_baro->altitude, small_baro->pressure, small_baro->temperature);
}

/**
 * @brief Encode a small_baro struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param small_baro C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_small_baro_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_small_baro_t* small_baro)
{
	return mavlink_msg_small_baro_pack_chan(system_id, component_id, chan, msg, small_baro->altitude, small_baro->pressure, small_baro->temperature);
}

/**
 * @brief Send a small_baro message
 * @param chan MAVLink channel to send the message
 *
 * @param altitude Calculated Altitude (m)
 * @param pressure Measured Differential Pressure (Pa)
 * @param temperature Measured Temperature (K)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_small_baro_send(mavlink_channel_t chan, float altitude, float pressure, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SMALL_BARO_LEN];
	_mav_put_float(buf, 0, altitude);
	_mav_put_float(buf, 4, pressure);
	_mav_put_float(buf, 8, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, buf, MAVLINK_MSG_ID_SMALL_BARO_LEN, MAVLINK_MSG_ID_SMALL_BARO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, buf, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif
#else
	mavlink_small_baro_t packet;
	packet.altitude = altitude;
	packet.pressure = pressure;
	packet.temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, (const char *)&packet, MAVLINK_MSG_ID_SMALL_BARO_LEN, MAVLINK_MSG_ID_SMALL_BARO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, (const char *)&packet, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SMALL_BARO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_small_baro_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float altitude, float pressure, float temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, altitude);
	_mav_put_float(buf, 4, pressure);
	_mav_put_float(buf, 8, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, buf, MAVLINK_MSG_ID_SMALL_BARO_LEN, MAVLINK_MSG_ID_SMALL_BARO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, buf, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif
#else
	mavlink_small_baro_t *packet = (mavlink_small_baro_t *)msgbuf;
	packet->altitude = altitude;
	packet->pressure = pressure;
	packet->temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, (const char *)packet, MAVLINK_MSG_ID_SMALL_BARO_LEN, MAVLINK_MSG_ID_SMALL_BARO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMALL_BARO, (const char *)packet, MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SMALL_BARO UNPACKING


/**
 * @brief Get field altitude from small_baro message
 *
 * @return Calculated Altitude (m)
 */
static inline float mavlink_msg_small_baro_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pressure from small_baro message
 *
 * @return Measured Differential Pressure (Pa)
 */
static inline float mavlink_msg_small_baro_get_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field temperature from small_baro message
 *
 * @return Measured Temperature (K)
 */
static inline float mavlink_msg_small_baro_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a small_baro message into a struct
 *
 * @param msg The message to decode
 * @param small_baro C-struct to decode the message contents into
 */
static inline void mavlink_msg_small_baro_decode(const mavlink_message_t* msg, mavlink_small_baro_t* small_baro)
{
#if MAVLINK_NEED_BYTE_SWAP
	small_baro->altitude = mavlink_msg_small_baro_get_altitude(msg);
	small_baro->pressure = mavlink_msg_small_baro_get_pressure(msg);
	small_baro->temperature = mavlink_msg_small_baro_get_temperature(msg);
#else
	memcpy(small_baro, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SMALL_BARO_LEN);
#endif
}
