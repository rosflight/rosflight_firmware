/**
 ******************************************************************************
 * File     : DLHRL20G.cpp
 * Date     : Sep 28, 2023
 ******************************************************************************
 *
 * Copyright (c) 2023, AeroVironment, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2.Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3.Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 **/
#include <misc.h>
#include <DlhrL20G.h>

extern Time64 time64;

__attribute__((section("my_dma_buffers"))) __attribute__((aligned (32))) static uint8_t dlhr_i2c_dma_buf[I2C_DMA_MAX_BUFFER_SIZE]={0};

__attribute__((section("my_buffers"))) __attribute__((aligned (32))) static  uint8_t dlhr_fifo_rx_buffer[DLHRL20G_FIFO_BUFFERS*sizeof(PitotPacket)]={0};

#define DLHR_I2C_STATUS_SIZE 1
#define DLHR_I2C_DMA_SIZE 7

uint32_t DlhrL20G::init
(
		// Driver initializers
		uint16_t					sample_rate_hz,
		GPIO_TypeDef 			*drdy_port,	// Reset GPIO Port
		uint16_t      		drdy_pin,  	// Reset GPIO Pin
		// I2C initializers
		I2C_HandleTypeDef 	*hi2c,		// The SPI handle
		uint16_t 						i2c_address	// Chip select Port
)
{
	uint32_t status = DRIVER_OK;
	sampleRateHz_	= sample_rate_hz;
	drdyPort_ 		= drdy_port;
	drdyPin_  		= drdy_pin;

	hi2c_ 				=	hi2c;
	address_ 			= i2c_address<<1;
	launchUs_			= 0;
	// groupDelay_ = 0; //Computed later based on launchUs_ and drdy_ timestamps.

	rxFifo_.init(DLHRL20G_FIFO_BUFFERS, sizeof(PitotPacket), dlhr_fifo_rx_buffer);

	dtMs_ = 1000./(double)sampleRateHz_;

	if(dtMs_<=8.0)  			 cmdByte_ = 	0xAA;
	else if(dtMs_<= 15.7)  cmdByte_ = 	0xAC;
	else if(dtMs_<= 31.1)  cmdByte_ = 	0xAD;
	else if(dtMs_<= 61.9)  cmdByte_ = 	0xAE;
	else 	   							 cmdByte_ = 	0xAF;

	// Read the status register
	dlhr_i2c_dma_buf[0] = cmdByte_;
	dlhr_i2c_dma_buf[1] = 0x00;
	uint8_t sensor_status;

	HAL_I2C_Master_Receive(hi2c_, address_, &sensor_status, 1,1000); //Receive 7 bytes of data over I2C

	misc_printf("DLHRL20G Status = 0x%02X (0x40) - ", sensor_status);
	if(sensor_status==0x40) misc_printf("OK\n\r");
	else
	{ misc_printf("ERROR\n\r");
	status |= DRIVER_SELF_DIAG_ERROR;}

	return status;
}

bool DlhrL20G::poll(uint16_t poll_offset)
{
	bool status = false;
	static bool previous_drdy=0;
	bool current_drdy = HAL_GPIO_ReadPin(drdyPort_, drdyPin_);

	if(poll_offset==0) // polled sensor measurement start
	{
		dlhr_i2c_dma_buf[0] = cmdByte_;
		dlhr_i2c_dma_buf[1] = 0x00;
		#if USE_D_CACHE_MANAGEMENT_FUNCTIONS
				SCB_CleanDCache_by_Addr((uint32_t *)dlhr_i2c_dma_buf, 1) ; // push data from cache to SRAM
		#endif
		launchUs_ = time64.Us();
	  status = (HAL_OK==HAL_I2C_Master_Transmit_DMA(hi2c_, address_, dlhr_i2c_dma_buf, 1));
	}
	else if(!previous_drdy && current_drdy) // drdy triggers a read.
	{
		drdy_ = time64.Us();
		status = startDma();
	}
	previous_drdy=current_drdy;
	return status;
}

bool DlhrL20G::startDma(void)
{
	#if USE_D_CACHE_MANAGEMENT_FUNCTIONS
		SCB_CleanDCache_by_Addr((uint32_t *)dlhr_i2c_dma_buf, DLHR_I2C_DMA_SIZE) ; // push data from cache to SRAM, unclear if this is needed.
	#endif
	return HAL_OK == HAL_I2C_Master_Receive_DMA(hi2c_, address_, dlhr_i2c_dma_buf, DLHR_I2C_DMA_SIZE); //Receive 7 bytes of data over I2C
}

void DlhrL20G::endDma(void)
{
	#if USE_D_CACHE_MANAGEMENT_FUNCTIONS
		SCB_InvalidateDCache_by_Addr((uint32_t *)dlhr_i2c_dma_buf, DLHR_I2C_DMA_SIZE) ; // force fetch from SRAM
	#endif

	PitotPacket p;
	p.status      = dlhr_i2c_dma_buf[0];
	if(p.status & 0x0040)
	{
		p.timestamp 	= time64.Us();
		p.drdy				= drdy_;
		p.groupDelay	= (p.drdy-launchUs_)/2;
		int32_t i_pressure    =  ( (uint32_t)dlhr_i2c_dma_buf[1]<<24 | (uint32_t)dlhr_i2c_dma_buf[2]<<16 | (uint32_t)dlhr_i2c_dma_buf[3]<<8 )>>8;
		int32_t i_temperature =  ( (uint32_t)dlhr_i2c_dma_buf[4]<<24 | (uint32_t)dlhr_i2c_dma_buf[5]<<16 | (uint32_t)dlhr_i2c_dma_buf[6]<<8 )>>8;

		double FS   = 5000; //Pa
		double OSdig = 0.1; // Offset percent of full scale.

		p.pressure    = 1.25*FS*( (double)i_pressure/16777216.0 - OSdig ); // Pa
		p.temperature = 125.0*(double)i_temperature/16777216.0  - 40.0+273.15; // K

		rxFifo_.write((uint8_t*)&p,sizeof(p));
	}
}

bool DlhrL20G::display(void)
{
	PitotPacket p;
	char name[] = "DlhrL20G (pitot)";
	if(rxFifo_.readMostRecent((uint8_t*)&p,sizeof(p)))
	{
		misc_header(name,p.drdy, p.timestamp, p.groupDelay);
		misc_printf("%10.3f Pa                          |                                        | %7.1f C |              | 0x%04X\n\r",
									 p.pressure, p.temperature-273.15, p.status);
		return 1;
	}
	else
	{
		misc_printf("%s\n\r",name);
	}	return true;
}




