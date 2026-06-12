#include "mavlink.h"
#include "rosflight.h"
#include "test_board.h"

#include <gtest/gtest.h>

using namespace rosflight_firmware;

TEST(Sensors, BatteryFilterStartsFromMaxVoltage)
{
  testBoard board;
  Mavlink mavlink(board);
  ROSflight rf(board, mavlink);

  rf.init();

  const float voltage = 12.0f;
  const float current = 1.5f;
  board.set_battery(voltage, current, 25.0f, 1000);

  got_flags got = rf.sensors_.run();
  ASSERT_TRUE(got.battery);

  const float alpha = rf.params_.get_param_float(PARAM_BATTERY_VOLTAGE_ALPHA);
  const float max_voltage = rf.params_.get_param_float(PARAM_BATT_VOLT_MAX);
  const float expected_voltage = voltage * (1.0f - alpha) + max_voltage * alpha;
  EXPECT_NEAR(expected_voltage, rf.sensors_.get_battery()->voltage, 1e-4f);
}
