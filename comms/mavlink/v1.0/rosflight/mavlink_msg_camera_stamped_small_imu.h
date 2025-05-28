// MESSAGE CAMERA_STAMPED_SMALL_IMU PACKING

#define MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU 185

typedef struct __mavlink_camera_stamped_small_imu_t
{
 uint64_t time_boot_us; /*< Measurement timestamp as microseconds since boot*/
 float xacc; /*< Acceleration along X axis*/
 float yacc; /*< Acceleration along Y axis*/
 float zacc; /*< Acceleration along Z axis*/
 float xgyro; /*< Angular speed around X axis*/
 float ygyro; /*< Angular speed around Y axis*/
 float zgyro; /*< Angular speed around Z axis*/
 float temperature; /*< Internal temperature measurement*/
 uint8_t image; /*< True if there is an image associated with this measurement*/
} mavlink_camera_stamped_small_imu_t;

#define MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN 37
#define MAVLINK_MSG_ID_185_LEN 37

#define MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_CRC 209
#define MAVLINK_MSG_ID_185_CRC 209



#define MAVLINK_MESSAGE_INFO_CAMERA_STAMPED_SMALL_IMU { \
	"CAMERA_STAMPED_SMALL_IMU", \
	9, \
	{  { "time_boot_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_camera_stamped_small_imu_t, time_boot_us) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_stamped_small_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_stamped_small_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_stamped_small_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_stamped_small_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_stamped_small_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_stamped_small_imu_t, zgyro) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_stamped_small_imu_t, temperature) }, \
         { "image", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_camera_stamped_small_imu_t, image) }, \
         } \
}


/**
 * @brief Pack a camera_stamped_small_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_us Measurement timestamp as microseconds since boot
 * @param xacc Acceleration along X axis
 * @param yacc Acceleration along Y axis
 * @param zacc Acceleration along Z axis
 * @param xgyro Angular speed around X axis
 * @param ygyro Angular speed around Y axis
 * @param zgyro Angular speed around Z axis
 * @param temperature Internal temperature measurement
 * @param image True if there is an image associated with this measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_stamped_small_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_boot_us, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float temperature, uint8_t image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN];
	_mav_put_uint64_t(buf, 0, time_boot_us);
	_mav_put_float(buf, 8, xacc);
	_mav_put_float(buf, 12, yacc);
	_mav_put_float(buf, 16, zacc);
	_mav_put_float(buf, 20, xgyro);
	_mav_put_float(buf, 24, ygyro);
	_mav_put_float(buf, 28, zgyro);
	_mav_put_float(buf, 32, temperature);
	_mav_put_uint8_t(buf, 36, image);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#else
	mavlink_camera_stamped_small_imu_t packet;
	packet.time_boot_us = time_boot_us;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.temperature = temperature;
	packet.image = image;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif
}

/**
 * @brief Pack a camera_stamped_small_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_us Measurement timestamp as microseconds since boot
 * @param xacc Acceleration along X axis
 * @param yacc Acceleration along Y axis
 * @param zacc Acceleration along Z axis
 * @param xgyro Angular speed around X axis
 * @param ygyro Angular speed around Y axis
 * @param zgyro Angular speed around Z axis
 * @param temperature Internal temperature measurement
 * @param image True if there is an image associated with this measurement
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_stamped_small_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_boot_us,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float temperature,uint8_t image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN];
	_mav_put_uint64_t(buf, 0, time_boot_us);
	_mav_put_float(buf, 8, xacc);
	_mav_put_float(buf, 12, yacc);
	_mav_put_float(buf, 16, zacc);
	_mav_put_float(buf, 20, xgyro);
	_mav_put_float(buf, 24, ygyro);
	_mav_put_float(buf, 28, zgyro);
	_mav_put_float(buf, 32, temperature);
	_mav_put_uint8_t(buf, 36, image);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#else
	mavlink_camera_stamped_small_imu_t packet;
	packet.time_boot_us = time_boot_us;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.temperature = temperature;
	packet.image = image;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif
}

/**
 * @brief Encode a camera_stamped_small_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_stamped_small_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_stamped_small_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_stamped_small_imu_t* camera_stamped_small_imu)
{
	return mavlink_msg_camera_stamped_small_imu_pack(system_id, component_id, msg, camera_stamped_small_imu->time_boot_us, camera_stamped_small_imu->xacc, camera_stamped_small_imu->yacc, camera_stamped_small_imu->zacc, camera_stamped_small_imu->xgyro, camera_stamped_small_imu->ygyro, camera_stamped_small_imu->zgyro, camera_stamped_small_imu->temperature, camera_stamped_small_imu->image);
}

/**
 * @brief Encode a camera_stamped_small_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_stamped_small_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_stamped_small_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_stamped_small_imu_t* camera_stamped_small_imu)
{
	return mavlink_msg_camera_stamped_small_imu_pack_chan(system_id, component_id, chan, msg, camera_stamped_small_imu->time_boot_us, camera_stamped_small_imu->xacc, camera_stamped_small_imu->yacc, camera_stamped_small_imu->zacc, camera_stamped_small_imu->xgyro, camera_stamped_small_imu->ygyro, camera_stamped_small_imu->zgyro, camera_stamped_small_imu->temperature, camera_stamped_small_imu->image);
}

/**
 * @brief Send a camera_stamped_small_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_us Measurement timestamp as microseconds since boot
 * @param xacc Acceleration along X axis
 * @param yacc Acceleration along Y axis
 * @param zacc Acceleration along Z axis
 * @param xgyro Angular speed around X axis
 * @param ygyro Angular speed around Y axis
 * @param zgyro Angular speed around Z axis
 * @param temperature Internal temperature measurement
 * @param image True if there is an image associated with this measurement
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_stamped_small_imu_send(mavlink_channel_t chan, uint64_t time_boot_us, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float temperature, uint8_t image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN];
	_mav_put_uint64_t(buf, 0, time_boot_us);
	_mav_put_float(buf, 8, xacc);
	_mav_put_float(buf, 12, yacc);
	_mav_put_float(buf, 16, zacc);
	_mav_put_float(buf, 20, xgyro);
	_mav_put_float(buf, 24, ygyro);
	_mav_put_float(buf, 28, zgyro);
	_mav_put_float(buf, 32, temperature);
	_mav_put_uint8_t(buf, 36, image);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, buf, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, buf, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif
#else
	mavlink_camera_stamped_small_imu_t packet;
	packet.time_boot_us = time_boot_us;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.temperature = temperature;
	packet.image = image;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_stamped_small_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_boot_us, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float temperature, uint8_t image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_boot_us);
	_mav_put_float(buf, 8, xacc);
	_mav_put_float(buf, 12, yacc);
	_mav_put_float(buf, 16, zacc);
	_mav_put_float(buf, 20, xgyro);
	_mav_put_float(buf, 24, ygyro);
	_mav_put_float(buf, 28, zgyro);
	_mav_put_float(buf, 32, temperature);
	_mav_put_uint8_t(buf, 36, image);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, buf, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, buf, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif
#else
	mavlink_camera_stamped_small_imu_t *packet = (mavlink_camera_stamped_small_imu_t *)msgbuf;
	packet->time_boot_us = time_boot_us;
	packet->xacc = xacc;
	packet->yacc = yacc;
	packet->zacc = zacc;
	packet->xgyro = xgyro;
	packet->ygyro = ygyro;
	packet->zgyro = zgyro;
	packet->temperature = temperature;
	packet->image = image;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, (const char *)packet, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU, (const char *)packet, MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CAMERA_STAMPED_SMALL_IMU UNPACKING


/**
 * @brief Get field time_boot_us from camera_stamped_small_imu message
 *
 * @return Measurement timestamp as microseconds since boot
 */
static inline uint64_t mavlink_msg_camera_stamped_small_imu_get_time_boot_us(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from camera_stamped_small_imu message
 *
 * @return Acceleration along X axis
 */
static inline float mavlink_msg_camera_stamped_small_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yacc from camera_stamped_small_imu message
 *
 * @return Acceleration along Y axis
 */
static inline float mavlink_msg_camera_stamped_small_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field zacc from camera_stamped_small_imu message
 *
 * @return Acceleration along Z axis
 */
static inline float mavlink_msg_camera_stamped_small_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xgyro from camera_stamped_small_imu message
 *
 * @return Angular speed around X axis
 */
static inline float mavlink_msg_camera_stamped_small_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ygyro from camera_stamped_small_imu message
 *
 * @return Angular speed around Y axis
 */
static inline float mavlink_msg_camera_stamped_small_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zgyro from camera_stamped_small_imu message
 *
 * @return Angular speed around Z axis
 */
static inline float mavlink_msg_camera_stamped_small_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field temperature from camera_stamped_small_imu message
 *
 * @return Internal temperature measurement
 */
static inline float mavlink_msg_camera_stamped_small_imu_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field image from camera_stamped_small_imu message
 *
 * @return True if there is an image associated with this measurement
 */
static inline uint8_t mavlink_msg_camera_stamped_small_imu_get_image(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Decode a camera_stamped_small_imu message into a struct
 *
 * @param msg The message to decode
 * @param camera_stamped_small_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_stamped_small_imu_decode(const mavlink_message_t* msg, mavlink_camera_stamped_small_imu_t* camera_stamped_small_imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	camera_stamped_small_imu->time_boot_us = mavlink_msg_camera_stamped_small_imu_get_time_boot_us(msg);
	camera_stamped_small_imu->xacc = mavlink_msg_camera_stamped_small_imu_get_xacc(msg);
	camera_stamped_small_imu->yacc = mavlink_msg_camera_stamped_small_imu_get_yacc(msg);
	camera_stamped_small_imu->zacc = mavlink_msg_camera_stamped_small_imu_get_zacc(msg);
	camera_stamped_small_imu->xgyro = mavlink_msg_camera_stamped_small_imu_get_xgyro(msg);
	camera_stamped_small_imu->ygyro = mavlink_msg_camera_stamped_small_imu_get_ygyro(msg);
	camera_stamped_small_imu->zgyro = mavlink_msg_camera_stamped_small_imu_get_zgyro(msg);
	camera_stamped_small_imu->temperature = mavlink_msg_camera_stamped_small_imu_get_temperature(msg);
	camera_stamped_small_imu->image = mavlink_msg_camera_stamped_small_imu_get_image(msg);
#else
	memcpy(camera_stamped_small_imu, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CAMERA_STAMPED_SMALL_IMU_LEN);
#endif
}
