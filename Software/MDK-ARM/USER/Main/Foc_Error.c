/*
 * @Date: 2025-02-27 19:16:35
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2025-11-19 15:45:18
 * @FilePath: \Software\MDK-ARM\USER\Main\Foc_Error.c
 * @Description: Do not edit
 */
#include "Foc_Error.h"

ERROR_Struct Motor_Error = {1, 1, 1, 1, 1, {0, 0, 0, 0, 0}};

void Error_Struct_Init(ERROR_Struct *error)
{
    error->SAFETY_STATE = 1;
    error->OVER_VOLTAGE_STATE = 1;
    error->UNDER_VOLTAGE_STATE = 1;
    error->OVER_TEMPERATURE_STATE = 1;
    error->DRV8323_ERROR_STATE = 1;

    error->STATE_WINDOW.OVER_VOLTAGE_STATE_WINDOW = 0;
    error->STATE_WINDOW.UNDER_VOLTAGE_STATE_WINDOW = 0;
    error->STATE_WINDOW.OVER_TEMPERATURE_STATE_WINDOW = 0;
    error->STATE_WINDOW.DRV8323_Error_State_Window = 0;
}

void FOC_Error_Handler(void)
{
#if ERROR_MODE == MODE_ON
    Motor_Error.SAFETY_STATE =  Motor_Error.OVER_VOLTAGE_STATE & Motor_Error.UNDER_VOLTAGE_STATE & \
                                Motor_Error.OVER_TEMPERATURE_STATE & Motor_Error.DRV8323_ERROR_STATE;
    
    if(Motor_Error.SAFETY_STATE){return;}
    else {
        uint8_t txdata[1];
        FOC_Comm_TxData_Encoder(CMD_ERROR_FDB, txdata);
        FDCAN_SendMessageWithBaudSwitch(&hfdcan1, txdata, FDCAN_DLC_BYTES_1, Tx_Master_ID | CMD_ERROR_FDB);
    }
#endif
}

void Error_Main_Loop(void)
{
    Motor_Error.STATE_WINDOW.OVER_VOLTAGE_STATE_WINDOW <<= 1;
    Motor_Error.STATE_WINDOW.UNDER_VOLTAGE_STATE_WINDOW <<= 1;
    Motor_Error.STATE_WINDOW.OVER_TEMPERATURE_STATE_WINDOW <<= 1;
    Motor_Error.STATE_WINDOW.DRV8323_Error_State_Window <<= 1;

	// Check over voltage
    if(Motor_ADC.Valtage_VCC > VOLTAGE_MAX)
    {
        Motor_Error.STATE_WINDOW.OVER_VOLTAGE_STATE_WINDOW |= 1;
        if(Motor_Error.STATE_WINDOW.OVER_VOLTAGE_STATE_WINDOW == 0xFF)
        {
            Motor_Error.OVER_VOLTAGE_STATE = 0;
        }
    }
    // Check under voltage
    if(Motor_ADC.Valtage_VCC < VOLTAGE_MIN)
    {
        Motor_Error.STATE_WINDOW.UNDER_VOLTAGE_STATE_WINDOW |= 1;
        if(Motor_Error.STATE_WINDOW.UNDER_VOLTAGE_STATE_WINDOW == 0xFF)
        {
            Motor_Error.UNDER_VOLTAGE_STATE = 0;
        }
    }
    // Check over temperature
    if(Motor_ADC.Valtage_NTC < TEMPERATURE_MAX)
    {
        Motor_Error.STATE_WINDOW.OVER_TEMPERATURE_STATE_WINDOW |= 1;
        if(Motor_Error.STATE_WINDOW.OVER_TEMPERATURE_STATE_WINDOW == 0xFF)
        {
            Motor_Error.OVER_TEMPERATURE_STATE = 0;
        }
    }
    //Check DRV8323 nFault Pin
    if (HAL_GPIO_ReadPin(nFault_GPIO_Port, nFault_Pin) == GPIO_PIN_RESET)
    {
        Motor_Error.STATE_WINDOW.DRV8323_Error_State_Window |= 1;
        if(Motor_Error.STATE_WINDOW.DRV8323_Error_State_Window == 0xFF)
        {
            Motor_Error.DRV8323_ERROR_STATE = 0;
        }
    }
		
	FOC_Error_Handler();
}

// DRV8323_nFault_Interrupt
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if(GPIO_Pin == nFault_Pin)
//     {
//         Motor_Error.DRV8323_ERROR_STATE = 0;
//         FOC_Error_Handler();
//     }
// }
