/*
 * @Date: 2025-03-01 15:00:33
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2025-10-24 14:18:43
 * @FilePath: \MDK-ARM\USER\Peripheral\drv8323_util.h
 * @Description: Do not edit
 */
#ifndef __DRV8323_UTIL_H
#define __DRV8323_UTIL_H

#include "configure.h"
#include "stm32g4xx_hal.h"
#include "main.h"

#define DRV8323_GAIN 20 //放大器增益
#define CURRENT_DETECTION_RES 0.1f//检流电阻
#define DRV8323_ADC_GAIN (DRV8323_GAIN * CURRENT_DETECTION_RES)
#define DRV8323_VREF 3.32f
#define DRV8323_VREF_DIV_TWO 1.66f

void DRV8323_Init(void);					
void DRV8323_Enable(void);
void DRV8323_Disable(void);
void DRV8323_CAL_Align(void);	//运放校准, 保持高电平

#endif
