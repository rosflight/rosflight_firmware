/**
 ******************************************************************************
 * File     : Pwm.cpp
 * Date     : Nov 3, 2023
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

#include <BoardConfig.h>
#include <Driver.h>
#include <Pwm.h>

// Notes on DSHOT
// DSHOT  Bitrate	T1H      T0H    Bit(µs)	Frame (µs)
//  150	  150kbit/s	5.00	2.50	6.67	106.72
//  300	  300kbit/s	2.50	1.25	3.33	53.28 < we use this
//  600	  600kbit/s	1.25	0.625	1.67	26.72
// 1200	 1200kbit/s	0.625	0.313	0.83	13.28

typedef enum
{
    PWM_STD,
    PWM_DSHOT,
} PwmProtocol;

//typedef struct __attribute__((__packed__))
//{
//	TIM_HandleTypeDef *htim;
//	uint16_t channel;
//	uint16_t min;
//	uint16_t center;
//	uint16_t max;
//} PwmChannelCfg;

typedef struct __attribute__((__packed__))
{
	TIM_TypeDef *instance;
	PwmProtocol protocol;
	uint32_t	period_counts;
} PwmBlockCfg;

__attribute__((section(".data"))) PwmChannelCfg pwm_ch[PWM_CHANNELS] = PWM_CHANNELS_DEFINE;
__attribute__((section(".data"))) PwmBlockCfg pwm_blk[PWM_BLOCKS] = PWM_BLOCKS_DEFINE;

//#define PWM_CLOCK (2e8) // 200 MHz

uint32_t Pwm::init(void)
{
	chan_ = pwm_ch;
	for(uint8_t bk=0;bk<PWM_BLOCKS;bk++)
	{
		TIM_HandleTypeDef *htim;
		if(pwm_blk[bk].instance==TIM1) htim = &htim1;
		else if(pwm_blk[bk].instance==TIM3) htim = &htim3;
		else if(pwm_blk[bk].instance==TIM4) htim = &htim4;
		else if(pwm_blk[bk].instance==TIM8) htim = &htim8;

	    uint32_t prescaler; // yields 1us per count with a 200MHz clock input
        if(pwm_blk[bk].protocol==PWM_DSHOT)
        	prescaler = 0;   // 5ns per count.
        else // PWM_STD
        	prescaler = 199; // 1us per count

        TIM_MasterConfigTypeDef sMasterConfig = {0};
        TIM_OC_InitTypeDef sConfigOC = {0};

        htim->Instance = pwm_blk[bk].instance;
        htim->Init.Prescaler = prescaler;
        htim->Init.CounterMode = TIM_COUNTERMODE_UP;
        htim->Init.Period = pwm_blk[bk].period_counts;
        htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim->Init.RepetitionCounter = 0;
        htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_PWM_Init(htim) != HAL_OK)
            return DRIVER_HAL_ERROR;
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
            return DRIVER_HAL_ERROR;
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
 //       sConfigOC.Pulse = (SERVO_PWM_CENTER);
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
        sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

        for(uint8_t ch=0;ch<PWM_CHANNELS;ch++)
        {
        	if(pwm_ch[ch].htim==htim)
        	{
        		sConfigOC.Pulse = pwm_ch[ch].center;
 				if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, pwm_ch[ch].channel) != HAL_OK)
					return DRIVER_HAL_ERROR;
        	}
        }

        if( (pwm_blk[bk].instance==TIM1) || (pwm_blk[bk].instance==TIM8))
        {
			TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
			sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
			sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
			sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
			sBreakDeadTimeConfig.DeadTime = 0;
			sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
			sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
			sBreakDeadTimeConfig.BreakFilter = 0;
			sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
			sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
			sBreakDeadTimeConfig.Break2Filter = 0;
			sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
			if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig) != HAL_OK)
			 return DRIVER_HAL_ERROR;
        }

         HAL_TIM_MspPostInit(htim);

	}
    return DRIVER_OK;
}
