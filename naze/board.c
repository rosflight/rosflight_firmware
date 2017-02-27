#include "board.h"

#include "flash.h"

#include <breezystm32/breezystm32.h>

extern void SetSysClock(bool overclock);
serialPort_t *Serial1;

void init_board(void)
{
  // Configure clock, this figures out HSE for hardware autodetect
  SetSysClock(0);
  systemInit();
}

// serial

void init_serial(uint32_t baud_rate)
{
  Serial1 = uartOpen(USART1, NULL, baud_rate, MODE_RXTX);
}

void serial_write(uint8_t byte)
{
  serialWrite(Serial1, byte);
}

uint16_t serial_bytes_available(void)
{
  return serialTotalBytesWaiting(Serial1);
}

uint8_t serial_read(void)
{
  return serialRead(Serial1);
}

// sensors

static bool _baro_present;
static bool _mag_present;
static bool _sonar_present;
static bool _diff_pressure_present;

static float _accel_scale;
static float _gyro_scale;

void sensors_init(int board_revision)
{
    // Initialize I2c
    i2cInit(I2CDEV_2);

    while(millis() < 50);

    i2cWrite(0,0,0);
    _baro_present = ms5611_init();
    _mag_present = hmc5883lInit(board_revision);
    _sonar_present = mb1242_init();
    _diff_pressure_present = ms4525_init();

    // IMU
    uint16_t acc1G;
    mpu6050_init(true, &acc1G, &_gyro_scale, board_revision);
    _accel_scale = 9.80665f/acc1G;
}

void imu_register_callback(void (*callback)(void))
{
  mpu6050_register_interrupt_cb(callback);
}

void imu_read_accel(float accel[3])
{
  mpu6050_read_accel(accel);
  accel[0] *= _accel_scale;
  accel[1] *= -_accel_scale;
  accel[2] *= -_accel_scale;
}

void imu_read_gyro(float gyro[3])
{
  mpu_6050_read_gyro(gyro);
  gyro[0] *= _gyro_scale;
  gyro[1] *= -_gyro_scale;
  gyro[2] *= -_gyro_scale;
}

float imu_read_temperature(void)
{
  float temperature_raw;
  mpu6050_read_temperature(&temperature_raw);
  return temperature_raw/340.0f + 36.53f;
}

bool mag_present(void)
{
  return _mag_present;
}

void mag_read(float mag[3])
{
  int16_t raw_mag[3];
  hmc5883l_update();
  hmc5883l_read(raw_mag);
  mag[0] = (float)raw_mag[0];
  mag[1] = (float)raw_mag[1];
  mag[2] = (float)raw_mag[2];
}

bool baro_present(void)
{
  return _baro_present;
}

void baro_read(float *altitude, float *pressure, float *temperature)
{
  ms5611_update();
  ms5611_read(altitude, pressure, temperature);
}

bool diff_pressure_present(void)
{
  return _diff_pressure_present;
}

bool diff_pressure_check(void)
{
  _diff_pressure_present = ms4525_init();
  return _diff_pressure_present;
}

void diff_pressure_read(float *diff_pressure, float *temperature, float *velocity)
{
  ms4525_update();
  ms4525_read(diff_pressure, temperature, velocity);
}

bool sonar_present(void)
{
  return _sonar_present;
}

bool sonar_check(void)
{
  mb1242_update();
  _sonar_present = (mb1242_read() > 0.2);
  return _sonar_present;
}

float sonar_read(void)
{
  mb1242_update();
  return mb1242_read();
}

// PWM

void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  pwmInit(cppm, false, false, refresh_rate, idle_pwm);
}

uint16_t pwm_read(uint8_t channel);
{
  return pwmRead(channel);
}

void pwm_write(uint8_t channel, uint16_t value)
{
  pwmWriteMotor(channel, value);
}

// non-volatile memory

void memory_init(void)
{
  initEEPROM();
}

bool memory_read(void * dest, size_t len)
{
  return readEEPROM(dest, len);
}

bool memory_write(const void * src, size_t len)
{
  return writeEEPROM(src, len);
}

// LED

void led0_on(void) { LED0_ON; }
void led0_off(void) { LED0_OFF; }
void led0_toggle(void) { LED0_TOGGLE; }

void led1_on(void) { LED1_ON; }
void led1_off(void) { LED1_OFF; }
void led1_toggle(void) { LED1_TOGGLE; }
