/**
 ******************************************************************************
 * File     : Liv4f.cpp
 * Date     : Jun 18, 2025
 ******************************************************************************
 *
 * Copyright (c) 2025, AeroVironment, Inc.
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

#include "Liv4f.h"

#include "Time64.h"

#include "Packets.h"
#include "misc.h"
#include <ctime>

extern Time64 time64;

// 201 NMEA Port Msg-List 0 (LOW)
// Default low is 0x4088535F = 0100 0000 1000 1000 0101 0011 0101 1111
#define GPGNS_MSG            0x01  // Low 32 bits CDB-ID 0 (default)
#define GPGGA_MSG            0x02  // Low 32 bits CDB-ID 1 (default)
#define GPGSA_MSG            0x04  // Low 32 bits CDB-ID 2 (default)
#define GPGST_MSG            0x08  // Low 32 bits CDB-ID 3 (default)
#define GPVTG_MSG            0x10  // Low 32 bits CDB-ID 4 (default)
#define PSTMNOISE_MSG        0x20  // Low 32 bits CDB-ID 5
#define GPRMC_MSG            0x40  // Low 32 bits CDB-ID 6 (default)
#define PSTMRF_MSG           0x80  // Low 32 bits CDB-ID 7
#define PSTMTG_MSG           0x100 // Low 32 bits CDB-ID 8 (default)
#define PSTMTS_MSG           0x200 // Low 32 bits CDB-ID 9 (default)
#define PSTMPA_MSG           0x400 // Low 32 bits CDB-ID 10
#define PSTMSAT_MSG          0x800 // Low 32 bits CDB-ID 11
#define PSTMRES_MSG          0x1000 // Low 32 bits CDB-ID 12 (default)
#define PSTMTIM_MSG          0x2000 // Low 32 bits CDB-ID 13
#define PSTMWAAS_MSG         0x4000 // Low 32 bits CDB-ID 14 (default)
#define PSTMDIFF_MSG         0x8000 // Low 32 bits CDB-ID 15
#define PSTMCORR_MSG         0x10000 // Low 32 bits CDB-ID 16
#define PSTMSBAS_MSG         0x20000 // Low 32 bits CDB-ID 17
#define PSTMTESTRF_MSG       0x40000 // Low 32 bits CDB-ID 18
#define GPGSV_MSG            0x80000 // Low 32 bits CDB-ID 19 (default)
#define GPGLL_MSG            0x100000 // Low 32 bits CDB-ID 20
#define PSTMPPSDATA_MSG      0x200000 // Low 32 bits CDB-ID 21
#define RESERVED_MSG         0x400000 // Low 32 bits CDB-ID 22
#define PSTMCPU_MSG          0x800000 // Low 32 bits CDB-ID 23 (default)
#define GPZDA_MSG            0x1000000 // Low 32 bits CDB-ID 24
#define PSTMTRAIMSTATUS_MSG  0x2000000 // Low 32 bits CDB-ID 25
#define PSTMPOSHOLD_MSG      0x4000000 // Low 32 bits CDB-ID 26
#define PSTMKFCOV_MSG        0x8000000 // Low 32 bits CDB-ID 27
#define PSTMAGPS_MSG         0x10000000 // Low 32 bits CDB-ID 28
#define PSTMLOWPOWERDATA_MSG 0x20000000 // Low 32 bits CDB-ID 29
#define PSTMNOTCHSTATUS_MSG  0x40000000 // Low 32 bits CDB-ID 30 (default)
#define PSTMTM_MSG           0x80000000 // Low 32 bits CDB-ID 31


// 228 NMEA Port Msg-List 0 (HIGH)
// Default hi  is 0x7EC02320 = 0111 1110 1100 0000 0010 0011 0010 0000
#define PSTMPV_MSG            0x1          // High 32 bits CDB-ID 32
#define PSTMPVQ_MSG           0x2          // High 32 bits CDB-ID 33
#define PSTMUTC_MSG           0x4          // High 32 bits CDB-ID 34
#define PSTMADCDATA_MSG       0x8          // High 32 bits CDB-ID 35
#define RESERVED_36_MSG       0x10         // High 32 bits CDB-ID 36
#define RESERVED_37_MSG       0x20         // High 32 bits CDB-ID 37 (default)
#define PSTMUSEDSATS_MSG      0x40         // High 32 bits CDB-ID 38
#define GPDTM_MSG             0x80         // High 32 bits CDB-ID 39
#define PSTMEPHEM_MSG         0x100        // High 32 bits CDB-ID 40 (default)
#define PSTMALMANAC_MSG       0x200        // High 32 bits CDB-ID 41 (default)
#define PSTMIONOPARAMS_MSG    0x400        // High 32 bits CDB-ID 42
#define RESERVED_43_MSG       0x800        // High 32 bits CDB-ID 43
#define PSTMBIASDATA_MSG      0x1000       // High 32 bits CDB-ID 44
#define GPGBS_MSG             0x2000       // High 32 bits CDB-ID 45 (default)
#define PSTMPVRAW_MSG         0x4000       // High 32 bits CDB-ID 46
#define RESERVED_47_MSG       0x8000       // High 32 bits CDB-ID 47
#define PSTMFEDATA_MSG        0x10000      // High 32 bits CDB-ID 48
#define RESERVED_49_MSG       0x20000      // High 32 bits CDB-ID 49
#define PSTMODO_MSG           0x40000      // High 32 bits CDB-ID 50
#define PSTMGEOFENCESTATUS_MSG 0x80000     // High 32 bits CDB-ID 51
#define PSTMLOGSTATUS_MSG      0x100000    // High 32 bits CDB-ID 52
#define PSTMGNSSINTEGRITY_MSG  0x200000    // High 32 bits CDB-ID 53
#define RESERVED_54_MSG        0x400000    // High 32 bits CDB-ID 54 (default)
#define RESERVED_55_MSG        0x800000    // High 32 bits CDB-ID 55 (default)
#define RESERVED_56_MSG        0x1000000   // High 32 bits CDB-ID 56
#define RESERVED_57_MSG        0x2000000   // High 32 bits CDB-ID 57 (default)
#define RESERVED_58_MSG        0x4000000   // High 32 bits CDB-ID 58 (default)
#define RESERVED_59_MSG        0x8000000   // High 32 bits CDB-ID 59 (default)
#define RESERVED_60_MSG        0x10000000  // High 32 bits CDB-ID 60 (default)
#define RESERVED_61_MSG        0x20000000  // High 32 bits CDB-ID 61 (default)
#define RESERVED_62_MSG        0x40000000  // High 32 bits CDB-ID 62 (default)
#define RLM_MSG                0x80000000  // High 32 bits CDB-ID 63



#define LIV4F_DMA_BUFFER_SIZE (16*4) //(16 * 2) // must be multiple of 16
DMA_RAM uint8_t liv4f_dma_rxbuf[LIV4F_DMA_BUFFER_SIZE];

DTCM_RAM uint8_t liv4f_double_buffer[2 * sizeof(GnssPacket)];

void Liv4f::pps(uint64_t pps_timestamp)
{
  static bool first_time = true;
  if(!first_time) pps_ = pps_timestamp;
  first_time = false;
  // Clear packet acq flags;
  gotPvraw_ = 0;
  gotZda_ = 0;
  gotKfcov_= 0;
}

bool Liv4f::lookFor(const char* match, char* buffer, int num_received)
{
  bool found = false;
  int N = strlen(match);
  char *p = buffer;
  for(int i=0;i<(num_received-N); i++) {
	if( *p ==0 ) break;
	if( strncmp (p,match,N)==0 ) {
	   found = true;
	   break;
	 }
	 p++;
  }
	return found;
}


uint32_t Liv4f::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * pps_port, uint16_t pps_pin,
  // UART initializers
  UART_HandleTypeDef * huart, USART_TypeDef * huart_instance,
  DMA_HandleTypeDef * hdma_uart_rx, uint32_t baud)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Liv4f");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  ppsPin_ = pps_pin;
  ppsHz_ = 1; // To match top of second.

  dtimeout_ = 1000000; // 1 seconds
  timeout_ = 0;

  huart_ = huart;
  hdmaUartRx_ = hdma_uart_rx; // huart->hdmarx;

  baud_initial_ = 115200;
  baud_ = baud; // This is not used

  gnss_.pps = 0;

  gotPvraw_ = 0;
  gotZda_ = 0;
  gotKfcov_= 0;

  // USART initialization
   huart_->Instance = huart_instance;
   huart_->Init.BaudRate = baud_initial_;
   huart_->Init.WordLength = UART_WORDLENGTH_8B;
   huart_->Init.StopBits = UART_STOPBITS_1;
   huart_->Init.Parity = UART_PARITY_NONE;
   huart_->Init.Mode = UART_MODE_TX_RX;
   huart_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart_->Init.OverSampling = UART_OVERSAMPLING_16;
   huart_->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
   huart_->Init.ClockPrescaler = UART_PRESCALER_DIV1;
   huart_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
   //  huart_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
   //  huart_->AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
   if (HAL_UART_Init(huart_) != HAL_OK) {
     initializationStatus_ |= DRIVER_HAL_ERROR;
     return initializationStatus_;
   }
   if (HAL_UARTEx_SetTxFifoThreshold(huart_, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK) {
     initializationStatus_ |= DRIVER_HAL_ERROR;
     return initializationStatus_;
   }
   if (HAL_UARTEx_SetRxFifoThreshold(huart_, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK) {
     initializationStatus_ |= DRIVER_HAL_ERROR;
     return initializationStatus_;
   }
   //    if (HAL_UARTEx_DisableFifoMode (huart_) != HAL_OK)
   //        return DRIVER_HAL_ERROR;
   if (HAL_UARTEx_EnableFifoMode(huart_) != HAL_OK) {
     initializationStatus_ |= DRIVER_HAL_ERROR;
     return initializationStatus_;
   }

   double_buffer_.init(liv4f_double_buffer, sizeof(liv4f_double_buffer) );

// Check if we are an ST GPS

  //char match[] = "$PST";
  //uint16_t N = strlen(match);

  bool found = false;
  char buffer[11520]; // 1 second of data at 115200 baud
  uint16_t num_received = 0;
  for(int j=0;j<4;j++) // try 4 times
  {
    char message[] = "$PSTMGETSWVER*59\r\n\n";
    HAL_UART_Transmit(huart_, (uint8_t*)message, strlen(message), 0xFFFFFFFF); // send remove trailing zero

	memset(buffer,0,sizeof(buffer));
	HAL_UART_Receive(huart_, (uint8_t *)buffer, sizeof(buffer), 1000); // start next read, timeout after 1.5 seconds
	num_received = sizeof(buffer) - huart_->RxXferCount;

    found = lookFor("$PSTMGETSWVER",buffer,num_received);

    if(found) {
    	misc_printf("Got response for $PSTMGETSWVER - It's a Teseo\n");
    	break;
    }
  }

  if(!found) {
    initializationStatus_ |= DRIVER_ID_MISMATCH;
    return initializationStatus_;
  }


  // Check Configuration
//  $GNZDA,,,,,,*56
//  $PSTMKFCOV,,,,,,,,*4D
//  $PSTMPVRAW,000152.400,0000.00000,N,00000.00000,E,0,00,0.0,000.00,M,0.0,M,0.000,0.000,0.000*53

  if ( !( lookFor("$GNZDA",buffer,num_received) && lookFor("$PSTMKFCOV",buffer,num_received) && lookFor("$PSTMPVRAW",buffer,num_received)))
  {
	  misc_printf("Setting LIV4 Packets: $GNZDA, $PSTMKFCOV, and $PSTMPVRAW\n");

	  char message[256]={0}, checksum[256]={0};
	  unsigned int message_low  =  PSTMKFCOV_MSG | GPZDA_MSG | (PSTMTG_MSG | PSTMTS_MSG | PSTMRES_MSG | PSTMWAAS_MSG | PSTMCPU_MSG | PSTMNOTCHSTATUS_MSG );
	  sprintf(message,"$PSTMSETPAR,1201,0x%08X,0*",message_low);
	  sprintf(checksum,"%02X\r\n\n",calculateNMEAChecksum(message));
	  strcat(message,checksum);
	  HAL_UART_Transmit(huart_, (uint8_t*)message, strlen(message), 0xFFFFFFFF); // send remove trailing zero
	  //  $PSTMSETPAR,1228, 0x4000,1               // this enables PSTMPVRAW
	  unsigned int message_high = PSTMPVRAW_MSG | (RESERVED_37_MSG | PSTMEPHEM_MSG | PSTMALMANAC_MSG | RESERVED_54_MSG | RESERVED_55_MSG |  RESERVED_57_MSG | RESERVED_58_MSG | RESERVED_59_MSG | RESERVED_60_MSG | RESERVED_61_MSG | RESERVED_62_MSG);
	  sprintf(message,"$PSTMSETPAR,1228,0x%08X,0*",message_high);
	  sprintf(checksum,"%02X\r\n\n", calculateNMEAChecksum(message));
	  strcat(message,checksum);
	  HAL_UART_Transmit(huart_, (uint8_t*)message, strlen(message), 0xFFFFFFFF); // send remove trailing zero

	  //  Update rate
	  float period = 1.0/sampleRateHz_;
	  period = (period<0.1)?0.1:period;
	  period = (period>1.0)?1.0:period;
	  sprintf(message,"$PSTMSETPAR,1303,%3.1f,0*",period);
	  sprintf(checksum,"%02X\r\n\n", calculateNMEAChecksum(message));
	  strcat(message,checksum);
	  HAL_UART_Transmit(huart_, (uint8_t*)message, strlen(message), 0xFFFFFFFF); // send remove trailing zero

	  // Save Parameters to Backup Memory
	  sprintf(message,"$PSTMSAVEPAR*");
	  sprintf(checksum,"%02X\r\n\n", calculateNMEAChecksum(message));
	  strcat(message,checksum);
	  HAL_UART_Transmit(huart_, (uint8_t*)message, strlen(message), 0xFFFFFFFF); // send remove trailing zero

	  // Reboot GPS Software
	  sprintf(message,"$PSTMSRR*");
	  sprintf(checksum,"%02X\r\n\n", calculateNMEAChecksum(message));
	  strcat(message,checksum);
	  HAL_UART_Transmit(huart_, (uint8_t*)message, strlen(message), 0xFFFFFFFF); // send remove trailing zero
  } else {
	  misc_printf("Found $GNZDA, $PSTMKFCOV, and $PSTMPVRAW\n");
  }

  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_IDLE);

   return initializationStatus_;
}

bool Liv4f::startDma(void)
{
  PROBE1_HI;
  timeout_ = time64.Us() + dtimeout_;
  HAL_StatusTypeDef hal_status = HAL_UART_Receive_DMA(huart_, liv4f_dma_rxbuf, LIV4F_DMA_BUFFER_SIZE);
  PROBE1_LO;
  return HAL_OK == hal_status;
}

char Liv4f::calculateNMEAChecksum(char *sentence) {
    char checksum = 0;
    int i = 1; // Start after the '$' character

    // Iterate through each character until '*' or end of the string
    while (sentence[i] != '*' && sentence[i] != '\0') {
        checksum ^= sentence[i];
        i++;
    }
    return checksum;
}

void Liv4f::endDma(void)
{
  PROBE2_HI;
  uint16_t bytes_in_dma_buffer = misc_bytes_in_dma(hdmaUartRx_, LIV4F_DMA_BUFFER_SIZE);

  static Liv4fFrame p;

  for (int i = 0; i < bytes_in_dma_buffer; i++) {
    bool found = parseByte(liv4f_dma_rxbuf[i], &p);

    if (found) {

      char pstmpvraw[] = "$PSTMPVRAW";
      char zda[] = "ZDA";
      char pstmkfcov[] = "$PSTMKFCOV";

      if( 0==strncmp((char *)(p.payload+3),zda,3))
      {
        // $GPZDA,<Timestamp>,<Day>,<Month>,<Year>,00,00*<checksum><cr><lf>
        //$GNZDA,230427.799,13,08,2025,,*40
        char message_id[32];
        int32_t hours, minutes, day, month, year, seconds, millis;
        char checksum[3];
        int n = sscanf((char *)p.payload, "$%31[^,],%2ld%2ld%2ld.%3ld,%2ld,%2ld,%4ld,,*%2s\r\n",
           message_id,
           &hours, &minutes, &seconds, &millis,
           &day, &month, &year,
           checksum);

        if(n==9)
        {
          gotZda_ = time64.Us();
          struct tm tm;
          tm.tm_sec  = seconds;
          tm.tm_min  = minutes;
          tm.tm_hour = hours;
          tm.tm_mday = day;
          tm.tm_mon  = month - 1;
          tm.tm_year = year - 1900;
          gnss_.unix_seconds = mktime(&tm);
          gnss_.unix_nanos = millis*1000000;
        }
      }
      else if( 0==strncmp((char *)p.payload,pstmpvraw,10))
      {
        // $PSTMPVRAW,<Timestamp>,<Lat>,<N/S>,<Long>,<E/W>,<GPSQual>,<Sats>,<HDOP>,<Alt>,<AltVal>,<GeoSep>,<GeoVal>,<Vel_N>,<Vel_E>,<Vel_V>*<checksum><cr><lf>
        // $PSTMPVRAW,230427.799,3412.86484,N,11853.76962,W,2,11,1.3,191.51,M,-32.1,M,0.042,-0.036,0.067*7B

        char message_id[32];
        int hours, minutes;
        float seconds;
        int lat_deg, lon_deg;
        float lat_min, lon_min;
        char ns, ew;
        int qual;
        int sats;
        float hdop;
        float alt;
        char alt_unit;
        float geo_sep;
        char geo_sep_unit;
        float v_n, v_e, v_up;
        char checksum[3];
        int n = sscanf((char *)p.payload, "$%31[^,],%2d%2d%f,%2d%f,%c,%3d%f,%c,%u,%u,%f,%f,%c,%f,%c,%f,%f,%f*%2s\r\n",
          message_id,
          &hours, &minutes, &seconds,
          &lat_deg, &lat_min, &ns, &lon_deg, &lon_min, &ew,
          &qual, &sats, &hdop,
          &alt, &alt_unit, &geo_sep, &geo_sep_unit,
          &v_n, &v_e, &v_up,
          checksum);

        if(n==21)
        {
          gotPvraw_ = time64.Us();

          if(qual==2) gnss_.fix_type = (uint8_t)rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_3D_FIX;
          else if(qual==1) gnss_.fix_type = (uint8_t)rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_2D_FIX;
          else gnss_.fix_type = (uint8_t)rosflight_firmware::GNSSFixType::GNSS_FIX_TYPE_NO_FIX;

          gnss_.num_sat = sats;

          gnss_.lon = (double)lon_deg + (double)lon_min/60.0;
          gnss_.lat = (double)lat_deg + (double)lat_min/60.0;
          // Adjust latitude and longitude based on hemisphere
          if (ns == 'S') gnss_.lat = -gnss_.lat;
          if (ew == 'W') gnss_.lon = -gnss_.lon;

          gnss_.height_msl = alt;
          //gnss_.geo_sep = geo_sep;
          gnss_.vel_n = v_n;
          gnss_.vel_e = v_e;
          gnss_.vel_d = -v_up;
        }
      }
      else if( 0==strncmp((char *)p.payload,pstmkfcov,10))
      {
        // $PSTMKFCOV,<PosStd>,<PosNcov>,<PosEcov>,<PosVcov>,<VelStd>,<VelNcov><VelEcov>,<VelVcov>*<checksum><cr><lf>
        // $PSTMKFCOV,4.1,8.2,8.3,9.0,0.922,0.540,0.310,1.051*4F
        char message_id[32];
        float pos_std, pos_cov_n, pos_cov_e, pos_cov_d, vel_std, vel_cov_n, vel_cov_e, vel_cov_d;
        char checksum[3];
        int n = sscanf((char *)p.payload, "$%31[^,],%f,%f,%f,%f,%f,%f,%f,%f*%2s\r\n",
            message_id,
            &pos_std, &pos_cov_n, &pos_cov_e, &pos_cov_d, &vel_std, &vel_cov_n, &vel_cov_e, &vel_cov_d,
            checksum);

        if(n==10)
        {
          gotKfcov_ = time64.Us();
          gnss_.h_acc = pos_std; // horizontal position accuracy
          gnss_.v_acc = sqrt(pos_cov_d); // vertical position accuracy
          gnss_.speed_accy = vel_std; // speed accuracy
        }
      }

     if(gotPvraw_ && gotKfcov_ && gotZda_)
      {
       if ((time64.Us()-pps_)<20000 ) gnss_.pps = pps_- 1e6;
       else gnss_.pps = pps_;

        gnss_.header.complete  = time64.Us();
        gnss_.header.status = 0;
        // Compute time of validity
        gnss_.header.timestamp = gnss_.pps + (uint64_t)(gnss_.unix_nanos/1000); // microseconds

        write((uint8_t *) &gnss_, sizeof(gnss_));
        gotPvraw_ =0; gotKfcov_ =0; gotZda_=0;
      }
    }
  }
  PROBE2_LO;

  startDma(); // restart the DMA
}

bool Liv4f::parseByte(uint8_t c, Liv4fFrame * p)
{
  static uint16_t n = 0;

  // special case where we get 0xB5 randomly duplicated at the start (DMA wierdness).
  //if ((c == 0xB5) & (n == 1)) n = 0;


  if (n == 0) // header byte 1 "mu" character
  {
    p->checksum = 0; // Reset the checksum calculation
    p->n_star = LIV4F_MAX_PAYLOAD_BYTES;
    if (c == '$')
    {
      p->payload[n++] = c;
    }
  }
  else if (n > LIV4F_MAX_PAYLOAD_BYTES) // overflow, reset also allows for terminating zero
  {
    n = 0; // restart
  }
  else if (c == '*') // located the *
  {
    p->n_star = n; // remember index of *
    p->payload[n++] = c;
  }
  else if ( n == p->n_star+1) // Upper Char of checksum hex
  {
    p->payload[n++] = c;
  }
  else if ( n == p->n_star+2) // lower Char of checksum hex
  {
    p->payload[n++] = c;
  }
  else if ( n == p->n_star+3 )
  {
    if ( c == '\r')
    {
      p->payload[n++] = c;
    }
    else n=0; // error
  }
  else if ( n == p->n_star+4 ) // End of message
  {
    if (c == '\n')
    {
      p->payload[n++] = c;
      p->payload[n++] = 0; // add terminating zero
      p->length = n; // includes terminating zero
      n = 0;
      char checksum[3];
      sprintf(checksum, "%02X",p->checksum);

      if( (checksum[0] == p->payload[p->n_star+1]) && (checksum[1] == p->payload[p->n_star+2]) )  // checksum good
      {
        return true; // p contains a validated packet
      }
     }
  }
  else // starts with n==1
  {
    p->payload[n++] = c;
    p->checksum ^= c;
  }

  return false; // p does not contain a validated packet
}



