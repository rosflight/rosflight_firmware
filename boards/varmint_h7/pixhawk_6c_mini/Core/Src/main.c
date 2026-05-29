#include "main.h"

#include "usb_device.h"
#include "usbd_cdc_acm_if.h"
#include "v1.0/rosflight/mavlink.h"

#include <stddef.h>

PCD_HandleTypeDef hpcd_USB_OTG_FS;

static void MX_GPIO_Init(void);
static void LED_Red_On(void);
static void LED_Red_Off(void);
static void LED_Blue_On(void);
static void LED_Blue_Off(void);
static void LED_Update(uint32_t now_ms);
static void Send_Mavlink_Smoke_Test(void);
static void Send_Mavlink_Message(const mavlink_message_t * msg);
static void USB_CDC_Write(const uint8_t * data, uint16_t len);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USB_DEVICE_Init();

  LED_Red_Off();
  LED_Blue_Off();

  uint32_t next_mavlink_ms = 0;

  while (1) {
    const uint32_t now_ms = HAL_GetTick();

    LED_Update(now_ms);

    if ((int32_t)(now_ms - next_mavlink_ms) >= 0) {
      Send_Mavlink_Smoke_Test();
      next_mavlink_ms = now_ms + 1000;
    }

    HAL_Delay(1);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(FMU_LED_RED_GPIO_Port, FMU_LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FMU_LED_BLUE_GPIO_Port, FMU_LED_BLUE_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = FMU_LED_RED_Pin | FMU_LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;

  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_PCD_MspInit(PCD_HandleTypeDef * hpcd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hpcd->Instance == USB_OTG_FS) {
    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = USB_DM_Pin | USB_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef * hpcd)
{
  if (hpcd->Instance == USB_OTG_FS) {
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, USB_DM_Pin | USB_DP_Pin);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  }
}

void Error_Handler(void)
{
  __disable_irq();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = FMU_LED_RED_Pin | FMU_LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  LED_Red_Off();
  LED_Blue_On();

  while (1) {
  }
}

static void LED_Red_On(void)
{
  HAL_GPIO_WritePin(FMU_LED_RED_GPIO_Port, FMU_LED_RED_Pin, GPIO_PIN_RESET);
}

static void LED_Red_Off(void)
{
  HAL_GPIO_WritePin(FMU_LED_RED_GPIO_Port, FMU_LED_RED_Pin, GPIO_PIN_SET);
}

static void LED_Blue_On(void)
{
  HAL_GPIO_WritePin(FMU_LED_BLUE_GPIO_Port, FMU_LED_BLUE_Pin, GPIO_PIN_RESET);
}

static void LED_Blue_Off(void)
{
  HAL_GPIO_WritePin(FMU_LED_BLUE_GPIO_Port, FMU_LED_BLUE_Pin, GPIO_PIN_SET);
}

static void LED_Update(uint32_t now_ms)
{
  const uint32_t cycle_ms = now_ms % 2800;

  if (cycle_ms < 1000 || (cycle_ms >= 1200 && cycle_ms < 1400) ||
      (cycle_ms >= 1600 && cycle_ms < 1800)) {
    LED_Red_On();
  } else {
    LED_Red_Off();
  }
}

static void Send_Mavlink_Smoke_Test(void)
{
  mavlink_message_t msg;

  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_ROSFLIGHT_FIRMWARE, &msg, MAV_TYPE_QUADROTOR,
                             0, 0, 0, 0);
  Send_Mavlink_Message(&msg);

  mavlink_msg_diff_pressure_pack(1, MAV_COMP_ID_ROSFLIGHT_FIRMWARE, &msg, 1.0f, 1.0f, 1.0f);
  Send_Mavlink_Message(&msg);
}

static void Send_Mavlink_Message(const mavlink_message_t * msg)
{
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
  USB_CDC_Write(buffer, len);
}

static void USB_CDC_Write(const uint8_t * data, uint16_t len)
{
  for (uint8_t tries = 0; tries < 3; tries++) {
    if (CDC_Transmit(0, (uint8_t *) data, len) == USBD_OK) {
      return;
    }
    HAL_Delay(1);
  }
}

void CDC_Receive_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  (void) chan;
  (void) buffer;
  (void) size;
}

void CDC_TransmitCplt_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  (void) chan;
  (void) buffer;
  (void) size;
}
