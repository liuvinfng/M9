/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    24-May-2016
  * @brief   WIFI ESP8266 Header
  ******************************************************************************

  ******************************************************************************
  */  

#ifndef __CRC8_H
#define __CRC8_H

#include "gd32f30x.h"

void crc8(unsigned char *crc, unsigned char m);
void init_crc8(void);
uint8_t calcBufCrc8(char *inBuf, uint32_t inBufSz);


#endif


