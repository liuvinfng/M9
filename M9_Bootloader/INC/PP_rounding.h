/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  Wfliu
  * @version V0.0
  * @date    09-Aug-2016
  * @brief   System Initialize
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __PP_ROUNDING_H
#define __PP_ROUNDING_H

#include "gd32f30x.h"

#include "Map.h"

typedef enum {
	PP_ROUNDING_LEFT = 0,
	PP_ROUNDING_RIGHT,
} PpRoundingType;

uint8_t pp_rounding(PpRoundingType type, Point32_t target);

#endif
