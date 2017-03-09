extern "C"
{
#include <breezystm32.h>
#include "flash.h"
extern void SetSysClock(bool overclock);
}

#include "naze32.h"

namespace rosflight {

void Naze32::init_board(void)
{
  // Configure clock, this figures out HSE for hardware autodetect
  SetSysClock(0);
  systemInit();
  _board_revision = 2;
}

void Naze32::board_reset(bool bootloader)
{
  systemReset(bootloader);
}

// clock

uint32_t Naze32::clock_millis()
{
  return millis();
}

uint64_t Naze32::clock_micros()
{
  return micros();
}

void Naze32::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial

void Naze32::serial_init(uint32_t baud_rate)
{
  Serial1 = uartOpen(USART1, NULL, baud_rate, MODE_RXTX);
}

void Naze32::serial_write(uint8_t byte)
{
  serialWrite(Serial1, byte);
}

uint16_t Naze32::serial_bytes_available(void)
{
  return serialTotalBytesWaiting(Serial1);
}

uint8_t Naze32::serial_read(void)
{
  return serialRead(Serial1);
}

// sensors

void Naze32::sensors_init(int board_revision)
{
    // Initialize I2c
    i2cInit(I2CDEV_2);

    while(millis() < 50);

    i2cWrite(0,0,0);
    _baro_present = ms5611_init();
    _mag_present = hmc5883lInit(_board_revision);
    _sonar_present = mb1242_init();
    _diff_pressure_present = ms4525_init();

    // IMU
    uint16_t acc1G;
    mpu6050_init(true, &acc1G, &_gyro_scale, _board_revision);
    _accel_scale = 9.80665f/acc1G;
}

void Naze32::imu_register_callback(void (*callback)(void))
{
  mpu6050_register_interrupt_cb(callback);
}

void Naze32::imu_read_accel(float accel[3])
{
  // Convert to NED
  int16_t accel_raw[3];
  mpu6050_read_accel(accel_raw);
  accel[0] = accel_raw[0] * _accel_scale;
  accel[1] = -accel_raw[1] * _accel_scale;
  accel[2] = -accel_raw[2] * _accel_scale;
}

void Naze32::imu_read_gyro(float gyro[3])
{
  //  Convert to NED
  int16_t gyro_raw[3];
  mpu6050_read_gyro(gyro_raw);
  gyro[0] = gyro_raw[0] * _gyro_scale;
  gyro[1] = -gyro_raw[1] * _gyro_scale;
  gyro[2] = -gyro_raw[2] * _gyro_scale;
}

float Naze32::imu_read_temperature(void)
{
  int16_t temperature_raw;
  mpu6050_read_temperature(&temperature_raw);
  return temperature_raw/340.0f + 36.53f;
}

bool Naze32::mag_present(void)
{
  return _mag_present;
}

void Naze32::mag_read(float mag[3])
{
  // Convert to NED
  int16_t raw_mag[3];
  hmc5883l_update();
  hmc5883l_read(raw_mag);
  mag[0] = (float)raw_mag[0];
  mag[1] = -(float)raw_mag[1];
  mag[2] = -(float)raw_mag[2];
}

bool Naze32::mag_check(void)
{
  _mag_present = hmc5883lInit(_board_revision);
  return _mag_present;
}

bool Naze32::baro_present(void)
{
  return _baro_present;
}

void Naze32::baro_read(float *altitude, float *pressure, float *temperature)
{
  ms5611_update();
  ms5611_read(altitude, pressure, temperature);
  (*altitude) *= -1.0; // Convert to NED
}

void Naze32::baro_calibrate()
{
  ms5611_start_calibration();
}

bool Naze32::diff_pressure_present(void)
{
  return _diff_pressure_present;
}

bool Naze32::diff_pressure_check(void)
{
  _diff_pressure_present = ms4525_init();
  return _diff_pressure_present;
}

void Naze32::diff_pressure_calibrate()
{
  ms4525_start_calibration();
}

void Naze32::diff_pressure_set_atm(float barometric_pressure)
{
  ms4525_set_atm((uint32_t) barometric_pressure);
}

void Naze32::diff_pressure_read(float *diff_pressure, float *temperature, float *velocity)
{
  ms4525_update();
  ms4525_read(diff_pressure, temperature, velocity);
}

bool Naze32::sonar_present(void)
{
  return _sonar_present;
}

bool Naze32::sonar_check(void)
{
  mb1242_update();
  _sonar_present = (mb1242_read() > 0.2);
  return _sonar_present;
}

float Naze32::sonar_read(void)
{
  mb1242_update();
  return mb1242_read();
}

uint16_t num_sensor_errors(void)
{
  return i2cGetErrorCounter();
}

// PWM

void Naze32::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  pwmInit(cppm, false, false, refresh_rate, idle_pwm);
}

uint16_t Naze32::pwm_read(uint8_t channel)
{
  return pwmRead(channel);
}

void Naze32::pwm_write(uint8_t channel, uint16_t value)
{
  pwmWriteMotor(channel, value);
}

bool Naze32::pwm_lost()
{
    return ((millis() - pwmLastUpdate()) > 40);
}

// non-volatile memory

void Naze32::memory_init(void)
{
  initEEPROM();
}

bool Naze32::memory_read(void * dest, size_t len)
{
  return readEEPROM(dest, len);
}

bool Naze32::memory_write(const void * src, size_t len)
{
  return writeEEPROM(src, len);
}

// LED

void Naze32::led0_on(void) { LED0_ON; }
void Naze32::led0_off(void) { LED0_OFF; }
void Naze32::led0_toggle(void) { LED0_TOGGLE; }

void Naze32::led1_on(void) { LED1_ON; }
void Naze32::led1_off(void) { LED1_OFF; }
void Naze32::led1_toggle(void) { LED1_TOGGLE; }

}
