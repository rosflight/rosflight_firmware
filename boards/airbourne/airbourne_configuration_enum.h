#ifndef AIRBOURNE_CONFIGURATION_ENUM_H
#define AIRBOURNE_CONFIGURATION_ENUM_H
#include <cstdint>
namespace rosflight_firmware
{
namespace AirbourneConfiguration
{
enum serial_config_t: uint8_t
{
  SERIAL_VCP,
  SERIAL_UART1,
  SERIAL_UART2,
  SERIAL_UART3
};

enum rc_config_t: uint8_t
{
  RC_PPM,
  RC_SBUS
};

enum airspeed_config_t: uint8_t
{
  AIRSPEED_DISABLED,
  AIRSPEED_I2C2
};

enum gnss_config_t: uint8_t
{
  GNSS_DISABLED,
  GNSS_UART1,
  GNSS_UART2,
  GNSS_UART3
};

enum sonar_config_t: uint8_t
{
  SONAR_DISABLED,
  SONAR_I2C2
};

enum battery_monitor_config_t: uint8_t
{
  BATTERY_MONITOR_DISABLED,
  BATTERY_MONITOR_ADC3
};

enum barometer_config_t: uint8_t
{
  BAROMETER_DISABLED,
  BAROMETER_ONBOARD
};

enum magnetometer_config_t: uint8_t
{
  MAGNETOMETER_DISABLED,
  MAGNETOMETER_ONBOARD
};

}
} // namespace rosflight_firmware
#endif // AIRBOURNE_CONFIGURATION_ENUM_H
