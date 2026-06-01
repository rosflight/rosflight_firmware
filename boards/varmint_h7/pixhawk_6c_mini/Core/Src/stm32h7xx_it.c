#include "main.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

void NMI_Handler(void)
{
  while (1) {
  }
}

void HardFault_Handler(void)
{
  Error_Handler();
}

void MemManage_Handler(void)
{
  Error_Handler();
}

void BusFault_Handler(void)
{
  Error_Handler();
}

void UsageFault_Handler(void)
{
  Error_Handler();
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(IMU_ICM42688_DRDY_Pin);
}

void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi1);
}
