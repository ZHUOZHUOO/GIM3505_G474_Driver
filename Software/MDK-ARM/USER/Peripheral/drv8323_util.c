/*
 * @Date: 2025-09-24 11:25:03
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2025-10-23 22:14:20
 * @FilePath: \MDK-ARM\USER\Peripheral\drv8323_util.c
 * @Description: Do not edit
 */
/*
 * @Date: 2025-03-01 15:00:33
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2025-03-01 22:11:45
 * @FilePath: \undefinedf:\ZHUOZHUOO--Github\FOC_DRV8323\Software\STM32G431 Cube\FOC_DRV8323\MDK-ARM\USER\Utility\drv8323_util.c
 * @Description: Do not edit
 */
#include "drv8323_util.h"


void DRV8323_Enable(void)
{
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void DRV8323_Disable(void)
{
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
}

void CAL_Pin_Align(void)
{
    HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_RESET);
}

void DRV8323_Init(void)
{
    DRV8323_Enable();
    CAL_Pin_Align();
}
