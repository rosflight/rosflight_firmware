/** @file
 *	@brief MAVLink comm protocol testsuite generated from rosflight.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ROSFLIGHT_TESTSUITE_H
#define ROSFLIGHT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_rosflight(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_rosflight(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_offboard_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_offboard_control_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,77,144
    };
	mavlink_offboard_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Qx = packet_in.Qx;
        	packet1.Qy = packet_in.Qy;
        	packet1.Qz = packet_in.Qz;
        	packet1.Fx = packet_in.Fx;
        	packet1.Fy = packet_in.Fy;
        	packet1.Fz = packet_in.Fz;
        	packet1.mode = packet_in.mode;
        	packet1.ignore = packet_in.ignore;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_offboard_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_pack(system_id, component_id, &msg , packet1.mode , packet1.ignore , packet1.Qx , packet1.Qy , packet1.Qz , packet1.Fx , packet1.Fy , packet1.Fz );
	mavlink_msg_offboard_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode , packet1.ignore , packet1.Qx , packet1.Qy , packet1.Qz , packet1.Fx , packet1.Fy , packet1.Fz );
	mavlink_msg_offboard_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_offboard_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_offboard_control_send(MAVLINK_COMM_1 , packet1.mode , packet1.ignore , packet1.Qx , packet1.Qy , packet1.Qz , packet1.Fx , packet1.Fy , packet1.Fz );
	mavlink_msg_offboard_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_imu_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
	mavlink_small_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_us = packet_in.time_boot_us;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_pack(system_id, component_id, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature );
	mavlink_msg_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature );
	mavlink_msg_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_imu_send(MAVLINK_COMM_1 , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature );
	mavlink_msg_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_mag(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_mag_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_small_mag_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.xmag = packet_in.xmag;
        	packet1.ymag = packet_in.ymag;
        	packet1.zmag = packet_in.zmag;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_mag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_pack(system_id, component_id, &msg , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_small_mag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_small_mag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_mag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_mag_send(MAVLINK_COMM_1 , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_small_mag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_baro(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_baro_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_small_baro_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.altitude = packet_in.altitude;
        	packet1.pressure = packet_in.pressure;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_baro_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_pack(system_id, component_id, &msg , packet1.altitude , packet1.pressure , packet1.temperature );
	mavlink_msg_small_baro_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.altitude , packet1.pressure , packet1.temperature );
	mavlink_msg_small_baro_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_baro_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_baro_send(MAVLINK_COMM_1 , packet1.altitude , packet1.pressure , packet1.temperature );
	mavlink_msg_small_baro_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_diff_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_diff_pressure_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_diff_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.velocity = packet_in.velocity;
        	packet1.diff_pressure = packet_in.diff_pressure;
        	packet1.temperature = packet_in.temperature;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_diff_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_pack(system_id, component_id, &msg , packet1.velocity , packet1.diff_pressure , packet1.temperature );
	mavlink_msg_diff_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.velocity , packet1.diff_pressure , packet1.temperature );
	mavlink_msg_diff_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_diff_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_diff_pressure_send(MAVLINK_COMM_1 , packet1.velocity , packet1.diff_pressure , packet1.temperature );
	mavlink_msg_diff_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_camera_stamped_small_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_camera_stamped_small_imu_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113
    };
	mavlink_camera_stamped_small_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_us = packet_in.time_boot_us;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.temperature = packet_in.temperature;
        	packet1.image = packet_in.image;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_camera_stamped_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_pack(system_id, component_id, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature , packet1.image );
	mavlink_msg_camera_stamped_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature , packet1.image );
	mavlink_msg_camera_stamped_small_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_camera_stamped_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_camera_stamped_small_imu_send(MAVLINK_COMM_1 , packet1.time_boot_us , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.temperature , packet1.image );
	mavlink_msg_camera_stamped_small_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_named_command_struct(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_named_command_struct_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,"YZABCDEFG",235,46
    };
	mavlink_named_command_struct_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.qx = packet_in.qx;
        	packet1.qy = packet_in.qy;
        	packet1.qz = packet_in.qz;
        	packet1.Fx = packet_in.Fx;
        	packet1.Fy = packet_in.Fy;
        	packet1.Fz = packet_in.Fz;
        	packet1.type = packet_in.type;
        	packet1.ignore = packet_in.ignore;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_named_command_struct_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_pack(system_id, component_id, &msg , packet1.name , packet1.type , packet1.ignore , packet1.qx , packet1.qy , packet1.qz , packet1.Fx , packet1.Fy , packet1.Fz );
	mavlink_msg_named_command_struct_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.name , packet1.type , packet1.ignore , packet1.qx , packet1.qy , packet1.qz , packet1.Fx , packet1.Fy , packet1.Fz );
	mavlink_msg_named_command_struct_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_named_command_struct_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_command_struct_send(MAVLINK_COMM_1 , packet1.name , packet1.type , packet1.ignore , packet1.qx , packet1.qy , packet1.qz , packet1.Fx , packet1.Fy , packet1.Fz );
	mavlink_msg_named_command_struct_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_small_range(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_small_range_t packet_in = {
		17.0,45.0,73.0,41
    };
	mavlink_small_range_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.range = packet_in.range;
        	packet1.max_range = packet_in.max_range;
        	packet1.min_range = packet_in.min_range;
        	packet1.type = packet_in.type;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_range_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_small_range_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_range_pack(system_id, component_id, &msg , packet1.type , packet1.range , packet1.max_range , packet1.min_range );
	mavlink_msg_small_range_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_range_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.range , packet1.max_range , packet1.min_range );
	mavlink_msg_small_range_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_small_range_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_small_range_send(MAVLINK_COMM_1 , packet1.type , packet1.range , packet1.max_range , packet1.min_range );
	mavlink_msg_small_range_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_cmd_t packet_in = {
		5
    };
	mavlink_rosflight_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command = packet_in.command;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_pack(system_id, component_id, &msg , packet1.command );
	mavlink_msg_rosflight_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command );
	mavlink_msg_rosflight_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_send(MAVLINK_COMM_1 , packet1.command );
	mavlink_msg_rosflight_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_cmd_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_cmd_ack_t packet_in = {
		5,72
    };
	mavlink_rosflight_cmd_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command = packet_in.command;
        	packet1.success = packet_in.success;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_ack_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_ack_pack(system_id, component_id, &msg , packet1.command , packet1.success );
	mavlink_msg_rosflight_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command , packet1.success );
	mavlink_msg_rosflight_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_cmd_ack_send(MAVLINK_COMM_1 , packet1.command , packet1.success );
	mavlink_msg_rosflight_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_output_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_output_raw_t packet_in = {
		93372036854775807ULL,{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0 }
    };
	mavlink_rosflight_output_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.stamp = packet_in.stamp;
        
        	mav_array_memcpy(packet1.values, packet_in.values, sizeof(float)*14);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_output_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_output_raw_pack(system_id, component_id, &msg , packet1.stamp , packet1.values );
	mavlink_msg_rosflight_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_output_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stamp , packet1.values );
	mavlink_msg_rosflight_output_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_output_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_output_raw_send(MAVLINK_COMM_1 , packet1.stamp , packet1.values );
	mavlink_msg_rosflight_output_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_status_t packet_in = {
		17235,17339,17,84,151,218,29,96
    };
	mavlink_rosflight_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.num_errors = packet_in.num_errors;
        	packet1.loop_time_us = packet_in.loop_time_us;
        	packet1.armed = packet_in.armed;
        	packet1.failsafe = packet_in.failsafe;
        	packet1.rc_override = packet_in.rc_override;
        	packet1.offboard = packet_in.offboard;
        	packet1.error_code = packet_in.error_code;
        	packet1.control_mode = packet_in.control_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_status_pack(system_id, component_id, &msg , packet1.armed , packet1.failsafe , packet1.rc_override , packet1.offboard , packet1.error_code , packet1.control_mode , packet1.num_errors , packet1.loop_time_us );
	mavlink_msg_rosflight_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.armed , packet1.failsafe , packet1.rc_override , packet1.offboard , packet1.error_code , packet1.control_mode , packet1.num_errors , packet1.loop_time_us );
	mavlink_msg_rosflight_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_status_send(MAVLINK_COMM_1 , packet1.armed , packet1.failsafe , packet1.rc_override , packet1.offboard , packet1.error_code , packet1.control_mode , packet1.num_errors , packet1.loop_time_us );
	mavlink_msg_rosflight_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_version(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_version_t packet_in = {
		"ABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVW"
    };
	mavlink_rosflight_version_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.version, packet_in.version, sizeof(char)*50);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_version_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_version_pack(system_id, component_id, &msg , packet1.version );
	mavlink_msg_rosflight_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_version_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.version );
	mavlink_msg_rosflight_version_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_version_send(MAVLINK_COMM_1 , packet1.version );
	mavlink_msg_rosflight_version_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_aux_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_aux_cmd_t packet_in = {
		{ 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0 },{ 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186 }
    };
	mavlink_rosflight_aux_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.aux_cmd_array, packet_in.aux_cmd_array, sizeof(float)*14);
        	mav_array_memcpy(packet1.type_array, packet_in.type_array, sizeof(uint8_t)*14);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_aux_cmd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_aux_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_aux_cmd_pack(system_id, component_id, &msg , packet1.type_array , packet1.aux_cmd_array );
	mavlink_msg_rosflight_aux_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_aux_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type_array , packet1.aux_cmd_array );
	mavlink_msg_rosflight_aux_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_aux_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_aux_cmd_send(MAVLINK_COMM_1 , packet1.type_array , packet1.aux_cmd_array );
	mavlink_msg_rosflight_aux_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_ins(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_ins_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0
    };
	mavlink_rosflight_ins_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.pos_north = packet_in.pos_north;
        	packet1.pos_east = packet_in.pos_east;
        	packet1.pos_down = packet_in.pos_down;
        	packet1.qw = packet_in.qw;
        	packet1.qx = packet_in.qx;
        	packet1.qy = packet_in.qy;
        	packet1.qz = packet_in.qz;
        	packet1.u = packet_in.u;
        	packet1.v = packet_in.v;
        	packet1.w = packet_in.w;
        	packet1.p = packet_in.p;
        	packet1.q = packet_in.q;
        	packet1.r = packet_in.r;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_ins_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_ins_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_ins_pack(system_id, component_id, &msg , packet1.pos_north , packet1.pos_east , packet1.pos_down , packet1.qw , packet1.qx , packet1.qy , packet1.qz , packet1.u , packet1.v , packet1.w , packet1.p , packet1.q , packet1.r );
	mavlink_msg_rosflight_ins_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_ins_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_north , packet1.pos_east , packet1.pos_down , packet1.qw , packet1.qx , packet1.qy , packet1.qz , packet1.u , packet1.v , packet1.w , packet1.p , packet1.q , packet1.r );
	mavlink_msg_rosflight_ins_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_ins_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_ins_send(MAVLINK_COMM_1 , packet1.pos_north , packet1.pos_east , packet1.pos_down , packet1.qw , packet1.qx , packet1.qy , packet1.qz , packet1.u , packet1.v , packet1.w , packet1.p , packet1.q , packet1.r );
	mavlink_msg_rosflight_ins_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_external_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_external_attitude_t packet_in = {
		17.0,45.0,73.0,101.0
    };
	mavlink_external_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.qw = packet_in.qw;
        	packet1.qx = packet_in.qx;
        	packet1.qy = packet_in.qy;
        	packet1.qz = packet_in.qz;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_external_attitude_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_external_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_external_attitude_pack(system_id, component_id, &msg , packet1.qw , packet1.qx , packet1.qy , packet1.qz );
	mavlink_msg_external_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_external_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.qw , packet1.qx , packet1.qy , packet1.qz );
	mavlink_msg_external_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_external_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_external_attitude_send(MAVLINK_COMM_1 , packet1.qw , packet1.qx , packet1.qy , packet1.qz );
	mavlink_msg_external_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_hard_error(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_hard_error_t packet_in = {
		963497464,963497672,963497880,963498088
    };
	mavlink_rosflight_hard_error_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.error_code = packet_in.error_code;
        	packet1.pc = packet_in.pc;
        	packet1.reset_count = packet_in.reset_count;
        	packet1.doRearm = packet_in.doRearm;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_hard_error_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_hard_error_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_hard_error_pack(system_id, component_id, &msg , packet1.error_code , packet1.pc , packet1.reset_count , packet1.doRearm );
	mavlink_msg_rosflight_hard_error_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_hard_error_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.error_code , packet1.pc , packet1.reset_count , packet1.doRearm );
	mavlink_msg_rosflight_hard_error_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_hard_error_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_hard_error_send(MAVLINK_COMM_1 , packet1.error_code , packet1.pc , packet1.reset_count , packet1.doRearm );
	mavlink_msg_rosflight_hard_error_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_gnss(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_gnss_t packet_in = {
		93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,963498712,963498920,963499128,963499336,963499544,963499752,963499960,963500168,963500376,963500584,963500792,963501000,963501208,963501416,963501624,963501832,963502040,25
    };
	mavlink_rosflight_gnss_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time = packet_in.time;
        	packet1.nanos = packet_in.nanos;
        	packet1.rosflight_timestamp = packet_in.rosflight_timestamp;
        	packet1.time_of_week = packet_in.time_of_week;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.height = packet_in.height;
        	packet1.vel_n = packet_in.vel_n;
        	packet1.vel_e = packet_in.vel_e;
        	packet1.vel_d = packet_in.vel_d;
        	packet1.h_acc = packet_in.h_acc;
        	packet1.v_acc = packet_in.v_acc;
        	packet1.ecef_x = packet_in.ecef_x;
        	packet1.ecef_y = packet_in.ecef_y;
        	packet1.ecef_z = packet_in.ecef_z;
        	packet1.p_acc = packet_in.p_acc;
        	packet1.ecef_v_x = packet_in.ecef_v_x;
        	packet1.ecef_v_y = packet_in.ecef_v_y;
        	packet1.ecef_v_z = packet_in.ecef_v_z;
        	packet1.s_acc = packet_in.s_acc;
        	packet1.fix_type = packet_in.fix_type;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_gnss_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_pack(system_id, component_id, &msg , packet1.time_of_week , packet1.fix_type , packet1.time , packet1.nanos , packet1.lat , packet1.lon , packet1.height , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.h_acc , packet1.v_acc , packet1.ecef_x , packet1.ecef_y , packet1.ecef_z , packet1.p_acc , packet1.ecef_v_x , packet1.ecef_v_y , packet1.ecef_v_z , packet1.s_acc , packet1.rosflight_timestamp );
	mavlink_msg_rosflight_gnss_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_of_week , packet1.fix_type , packet1.time , packet1.nanos , packet1.lat , packet1.lon , packet1.height , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.h_acc , packet1.v_acc , packet1.ecef_x , packet1.ecef_y , packet1.ecef_z , packet1.p_acc , packet1.ecef_v_x , packet1.ecef_v_y , packet1.ecef_v_z , packet1.s_acc , packet1.rosflight_timestamp );
	mavlink_msg_rosflight_gnss_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_gnss_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_send(MAVLINK_COMM_1 , packet1.time_of_week , packet1.fix_type , packet1.time , packet1.nanos , packet1.lat , packet1.lon , packet1.height , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.h_acc , packet1.v_acc , packet1.ecef_x , packet1.ecef_y , packet1.ecef_z , packet1.p_acc , packet1.ecef_v_x , packet1.ecef_v_y , packet1.ecef_v_z , packet1.s_acc , packet1.rosflight_timestamp );
	mavlink_msg_rosflight_gnss_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_gnss_full(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_gnss_full_t packet_in = {
		93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,963498920,963499128,963499336,963499544,963499752,963499960,963500168,963500376,963500584,963500792,963501000,20979,21083,233,44,111,178,245,56,123,190
    };
	mavlink_rosflight_gnss_full_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rosflight_timestamp = packet_in.rosflight_timestamp;
        	packet1.time_of_week = packet_in.time_of_week;
        	packet1.t_acc = packet_in.t_acc;
        	packet1.nano = packet_in.nano;
        	packet1.lon = packet_in.lon;
        	packet1.lat = packet_in.lat;
        	packet1.height = packet_in.height;
        	packet1.height_msl = packet_in.height_msl;
        	packet1.h_acc = packet_in.h_acc;
        	packet1.v_acc = packet_in.v_acc;
        	packet1.vel_n = packet_in.vel_n;
        	packet1.vel_e = packet_in.vel_e;
        	packet1.vel_d = packet_in.vel_d;
        	packet1.g_speed = packet_in.g_speed;
        	packet1.head_mot = packet_in.head_mot;
        	packet1.s_acc = packet_in.s_acc;
        	packet1.head_acc = packet_in.head_acc;
        	packet1.year = packet_in.year;
        	packet1.p_dop = packet_in.p_dop;
        	packet1.month = packet_in.month;
        	packet1.day = packet_in.day;
        	packet1.hour = packet_in.hour;
        	packet1.min = packet_in.min;
        	packet1.sec = packet_in.sec;
        	packet1.valid = packet_in.valid;
        	packet1.fix_type = packet_in.fix_type;
        	packet1.num_sat = packet_in.num_sat;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_full_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_gnss_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_full_pack(system_id, component_id, &msg , packet1.time_of_week , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.valid , packet1.t_acc , packet1.nano , packet1.fix_type , packet1.num_sat , packet1.lon , packet1.lat , packet1.height , packet1.height_msl , packet1.h_acc , packet1.v_acc , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.g_speed , packet1.head_mot , packet1.s_acc , packet1.head_acc , packet1.p_dop , packet1.rosflight_timestamp );
	mavlink_msg_rosflight_gnss_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_full_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_of_week , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.valid , packet1.t_acc , packet1.nano , packet1.fix_type , packet1.num_sat , packet1.lon , packet1.lat , packet1.height , packet1.height_msl , packet1.h_acc , packet1.v_acc , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.g_speed , packet1.head_mot , packet1.s_acc , packet1.head_acc , packet1.p_dop , packet1.rosflight_timestamp );
	mavlink_msg_rosflight_gnss_full_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_gnss_full_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_gnss_full_send(MAVLINK_COMM_1 , packet1.time_of_week , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.valid , packet1.t_acc , packet1.nano , packet1.fix_type , packet1.num_sat , packet1.lon , packet1.lat , packet1.height , packet1.height_msl , packet1.h_acc , packet1.v_acc , packet1.vel_n , packet1.vel_e , packet1.vel_d , packet1.g_speed , packet1.head_mot , packet1.s_acc , packet1.head_acc , packet1.p_dop , packet1.rosflight_timestamp );
	mavlink_msg_rosflight_gnss_full_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_battery_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_battery_status_t packet_in = {
		17.0,45.0
    };
	mavlink_rosflight_battery_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.battery_voltage = packet_in.battery_voltage;
        	packet1.battery_current = packet_in.battery_current;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_battery_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_battery_status_pack(system_id, component_id, &msg , packet1.battery_voltage , packet1.battery_current );
	mavlink_msg_rosflight_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_battery_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.battery_voltage , packet1.battery_current );
	mavlink_msg_rosflight_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_battery_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_battery_status_send(MAVLINK_COMM_1 , packet1.battery_voltage , packet1.battery_current );
	mavlink_msg_rosflight_battery_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_config_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_config_request_t packet_in = {
		5
    };
	mavlink_rosflight_config_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.device = packet_in.device;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_request_pack(system_id, component_id, &msg , packet1.device );
	mavlink_msg_rosflight_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device );
	mavlink_msg_rosflight_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_config_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_request_send(MAVLINK_COMM_1 , packet1.device );
	mavlink_msg_rosflight_config_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_config(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_config_t packet_in = {
		5,72
    };
	mavlink_rosflight_config_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.device = packet_in.device;
        	packet1.config = packet_in.config;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_pack(system_id, component_id, &msg , packet1.device , packet1.config );
	mavlink_msg_rosflight_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device , packet1.config );
	mavlink_msg_rosflight_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_send(MAVLINK_COMM_1 , packet1.device , packet1.config );
	mavlink_msg_rosflight_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_device_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_device_info_t packet_in = {
		5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 },199
    };
	mavlink_rosflight_device_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.device = packet_in.device;
        	packet1.max_config = packet_in.max_config;
        	packet1.num_devices = packet_in.num_devices;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(uint8_t)*20);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_device_info_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_device_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_device_info_pack(system_id, component_id, &msg , packet1.device , packet1.max_config , packet1.name , packet1.num_devices );
	mavlink_msg_rosflight_device_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_device_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device , packet1.max_config , packet1.name , packet1.num_devices );
	mavlink_msg_rosflight_device_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_device_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_device_info_send(MAVLINK_COMM_1 , packet1.device , packet1.max_config , packet1.name , packet1.num_devices );
	mavlink_msg_rosflight_device_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_config_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_config_info_t packet_in = {
		5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158 }
    };
	mavlink_rosflight_config_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.device = packet_in.device;
        	packet1.config = packet_in.config;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(uint8_t)*20);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_info_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_config_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_info_pack(system_id, component_id, &msg , packet1.device , packet1.config , packet1.name );
	mavlink_msg_rosflight_config_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device , packet1.config , packet1.name );
	mavlink_msg_rosflight_config_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_config_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_info_send(MAVLINK_COMM_1 , packet1.device , packet1.config , packet1.name );
	mavlink_msg_rosflight_config_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight_config_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rosflight_config_status_t packet_in = {
		5,72,139,{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255 }
    };
	mavlink_rosflight_config_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.device = packet_in.device;
        	packet1.success = packet_in.success;
        	packet1.reboot_required = packet_in.reboot_required;
        
        	mav_array_memcpy(packet1.error_message, packet_in.error_message, sizeof(uint8_t)*50);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rosflight_config_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_status_pack(system_id, component_id, &msg , packet1.device , packet1.success , packet1.reboot_required , packet1.error_message );
	mavlink_msg_rosflight_config_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device , packet1.success , packet1.reboot_required , packet1.error_message );
	mavlink_msg_rosflight_config_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rosflight_config_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rosflight_config_status_send(MAVLINK_COMM_1 , packet1.device , packet1.success , packet1.reboot_required , packet1.error_message );
	mavlink_msg_rosflight_config_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rosflight(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_offboard_control(system_id, component_id, last_msg);
	mavlink_test_small_imu(system_id, component_id, last_msg);
	mavlink_test_small_mag(system_id, component_id, last_msg);
	mavlink_test_small_baro(system_id, component_id, last_msg);
	mavlink_test_diff_pressure(system_id, component_id, last_msg);
	mavlink_test_camera_stamped_small_imu(system_id, component_id, last_msg);
	mavlink_test_named_command_struct(system_id, component_id, last_msg);
	mavlink_test_small_range(system_id, component_id, last_msg);
	mavlink_test_rosflight_cmd(system_id, component_id, last_msg);
	mavlink_test_rosflight_cmd_ack(system_id, component_id, last_msg);
	mavlink_test_rosflight_output_raw(system_id, component_id, last_msg);
	mavlink_test_rosflight_status(system_id, component_id, last_msg);
	mavlink_test_rosflight_version(system_id, component_id, last_msg);
	mavlink_test_rosflight_aux_cmd(system_id, component_id, last_msg);
	mavlink_test_rosflight_ins(system_id, component_id, last_msg);
	mavlink_test_external_attitude(system_id, component_id, last_msg);
	mavlink_test_rosflight_hard_error(system_id, component_id, last_msg);
	mavlink_test_rosflight_gnss(system_id, component_id, last_msg);
	mavlink_test_rosflight_gnss_full(system_id, component_id, last_msg);
	mavlink_test_rosflight_battery_status(system_id, component_id, last_msg);
	mavlink_test_rosflight_config_request(system_id, component_id, last_msg);
	mavlink_test_rosflight_config(system_id, component_id, last_msg);
	mavlink_test_rosflight_device_info(system_id, component_id, last_msg);
	mavlink_test_rosflight_config_info(system_id, component_id, last_msg);
	mavlink_test_rosflight_config_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ROSFLIGHT_TESTSUITE_H
