/*
 * @Date: 2025-02-26 18:25:59
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2025-11-06 13:42:32
 * @FilePath: \Software\MDK-ARM\USER\Main\Foc_Control.c
 * @Description: Do not edit
 */

#include "Foc_Control.h"
#include "main.h"

#define Min(a, b) ((a) < (b) ? (a) : (b))
#define Max(a, b) ((a) > (b) ? (a) : (b))
#define Max_3(a, b, c) Max(Max(a, b), c)

FOC_Struct Motor_FOC;
FOC_Running_Struct Motor_Run = {0, 0, 0, 0, 0};

PID_TypeDef Current_Id_PID;
PID_TypeDef Current_Iq_PID;
PID_TypeDef Position_Comm_PID;
PID_TypeDef Position_PID;

//===========Elec_Theta_Zero_Point==========//
#if WHO_AM_I == Slave_Upper_Arm_ID
	#define Elec_Theta_Zero_Point 1.0f
	#define Default_Theta_Ref -1065.5f
	#define Trans_Polar -1	//机械传动极性
	//Trans_Polar =  1 ：扭矩方向与位移方向相同 （如同步带传动）
	//Trans_Polar = -1 ：扭矩方向与位移方向相反	（如使用定子为旋转体）
    #define K_Damping_Default 0.0001f
#elif WHO_AM_I == Slave_Fore_Arm_ID
	#define Elec_Theta_Zero_Point -0.5f
	#define Default_Theta_Ref -5.5f 
	#define Trans_Polar 1
    #define K_Damping_Default 0.0001f
#elif WHO_AM_I == Slave_0_Arm_ID
	#define Elec_Theta_Zero_Point 1.8f
	#define Default_Theta_Ref -5.5f
	#define Trans_Polar 1
    #define K_Damping_Default 0.00001f
#endif

#define Position_Polar (Trans_Polar * MOTOR_ENCODER_DIR)

static float inv_motor_voltage;
//static float sqrt3_inv_mv;
//static float half_inv_mv;
static const uint8_t sector_map[] = {0, 2, 6, 1, 4, 3, 5, 0};
static const uint8_t time_order[7][3] = {
	{0}, {0,1,2}, {1,0,2}, {2,0,1}, {2,1,0}, {1,2,0}, {0,2,1}
};

void CALC_SVPWM_Init() {
    // 初始化时计算一次，假设MOTOR_VOLTAGE和T在运行时不变
    inv_motor_voltage = 1.0f / MOTOR_VOLTAGE;
//    sqrt3_inv_mv = sqrtf(3.0f) * inv_motor_voltage;
//    half_inv_mv = 1.5f * inv_motor_voltage; // 调整原计算式中的系数
}

void CALC_SVPWM(float Valpha, float Vbeta) {
    float X, Y, Z, t1, t2;
    uint16_t Sector;
    // 预计算公共项
    float sqrt3_Vbeta = SQRT3 * Vbeta;
    float term3Valpha = 3.0f * Valpha;

    // 计算X, Y, Z（合并常量）
    X = sqrt3_Vbeta * inv_motor_voltage * T;
    Y = (sqrt3_Vbeta + term3Valpha) * (0.5f * inv_motor_voltage) * T;
    Z = (sqrt3_Vbeta - term3Valpha) * (0.5f * inv_motor_voltage) * T;

    // 计算扇区
		uint16_t N = (Vbeta > 0) + 2 * (3*Valpha > sqrt3_Vbeta) + 4 * (-3*Valpha > sqrt3_Vbeta);
    Sector = sector_map[N];

    // 计算t1和t2
    switch (Sector) {
        case 1: t1 = -Z; t2 =  X; break;
        case 2: t1 =  Z; t2 =  Y; break;
        case 3: t1 =  X; t2 = -Y; break;
        case 4: t1 = -X; t2 =  Z; break;
        case 5: t1 = -Y; t2 = -Z; break;
        case 6: t1 =  Y; t2 = -X; break;
        default: t1 = t2 = 0; break;
    }

    // 过调制调整
    float sum = t1 + t2;
    if (sum > T) {
        float scale = T / sum;
        t1 *= scale;
        t2 *= scale;
    }

    // 计算作用时间
    float Ta = (T - t1 - t2) * 0.25f;
    float Tb = Ta + t1 * 0.5f;
    float Tc = Tb + t2 * 0.5f;
    float times[] = {Ta, Tb, Tc};

    // 查表获取时间分配顺序
    const uint8_t *order = time_order[Sector];
    uint16_t hTimePhA = (uint16_t)times[order[0]];
    uint16_t hTimePhB = (uint16_t)times[order[1]];
    uint16_t hTimePhC = (uint16_t)times[order[2]];

		// 更新寄存器
    #if 	THREE_PHASE_LINE_SEQUENCCE == A_B_C
	TIM1->CCR1 = hTimePhA;
	TIM1->CCR2 = hTimePhB;
	TIM1->CCR3 = hTimePhC;
    #elif 	THREE_PHASE_LINE_SEQUENCCE == A_C_B
	TIM1->CCR1 = hTimePhA;
	TIM1->CCR2 = hTimePhC;
	TIM1->CCR3 = hTimePhB;
    #endif
		
//	TIM1->CCR1 = 0;
//	TIM1->CCR2 = 0;
//	TIM1->CCR3 = 0;
		
    // 保存到结构体
    Motor_FOC.hTimePhA = hTimePhA;
    Motor_FOC.hTimePhB = hTimePhB;
    Motor_FOC.hTimePhC = hTimePhC;
}

void Park_transform(float Ialpha, float Ibeta, float *Id, float *Iq, float Theta)
{
  // 帕克变换，将αβ坐标系下的电流转换为dq坐标系下的电流
    *Id = Ialpha * arm_cos_f32(Theta) + Ibeta * arm_sin_f32(Theta);
    *Iq = -Ialpha * arm_sin_f32(Theta) + Ibeta * arm_cos_f32(Theta);
}

void Clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
	// 克拉克变换，将Ia,Ib,Ic转换为Ialpha和Ibeta
    *Ialpha = Ia;
    *Ibeta = SQRT3_DIV3 * (Ib - Ic);
}

void Inv_Park_transform(float Id, float Iq, float *Ialpha, float *Ibeta, float Theta)
{
	// 逆帕克变换，将dq坐标系下的电流转换为αβ坐标系下的电流
    *Ialpha = Id * arm_cos_f32(Theta) - Iq * arm_sin_f32(Theta);
    *Ibeta = Id * arm_sin_f32(Theta) + Iq * arm_cos_f32(Theta);
}

float FOC_ElecTheta_Calc(float Theta)
{
    float electrode_angle;
    electrode_angle = ((int32_t)(MOTOR_POLE_PAIRS * (Theta + 360))% 360) / 360.0f * TWO_PI - Motor_FOC.ElecTheta_Offset;
    return electrode_angle;//rad
}

float FOC_Theta_Calc(float Theta)
{
    float angle;
    angle = ((int32_t) (Theta + 360)) / 360.0f * TWO_PI - Motor_FOC.ElecTheta_Offset / MOTOR_POLE_PAIRS;
    return angle;//rad
}

void FOC_Struct_Init(FOC_Struct *foc)
{
    foc->Ia = 0;
    foc->Ib = 0;
    foc->Ic = 0;
    foc->Ialpha = 0;
    foc->Ibeta = 0;
    foc->Id = 0;
	foc->Id_ref = 0.0f;
    foc->Iq = 0;
    foc->Iq_ref = 0.0f;
    foc->Vd = 0;
    foc->Vq = 0;
    foc->Valpha = 0;
    foc->Vbeta = 0;

    foc->Speed_Rpm_Ref = (float)(MOTOR_SPEED_MAX - 200.0f);
    foc->Speed_Rpm = 0;
    foc->Theta = 0;
    foc->Theta_Ref = 0.0f;
    foc->Theta_Exp_Comm = Default_Theta_Ref / 360.0f * TWO_PI;
    foc->ElecTheta = 0;
	foc->ElecTheta_Offset = Elec_Theta_Zero_Point;

    foc->hTimePhA = 0;
    foc->hTimePhB = 0;
    foc->hTimePhC = 0;

    foc->Iq_Damping = 0.0f;
    foc->K_Damping = K_Damping_Default;

    foc->Motor_Close_Loop_Mode = (Close_Loop_Mode_t) FOC_CLOSE_LOOP_MODE;
}

void FOC_PID_Init(void)
{
	//===========HT4315==========//
	#if MOTOR_TYPE == HT4315
		#if WHO_AM_I == Slave_Upper_Arm_ID
		PID_Init(&Current_Id_PID, PID_DELTA, 1.5f, 0.3f, 0.00f, 0.0f, 0.0f, 20.0f, 0.2f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Current_Iq_PID, PID_DELTA, 1.5f, 0.2f, 0.00f, 0.0f, 0.0f, 5.8f, 0.2f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Position_Comm_PID, PID_DELTA, 0.0f, 0.002f, 0.0f, 0.0f, 0.0f, 100.0f, 0.01f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Position_PID, PID_POSITION, 2.0f, 0.0008f, 0.003f, 0.0f, 0.0f, 500.0f, 0.9f, 0.1f, 0.1f, 0.1f);
		#elif WHO_AM_I == Slave_Fore_Arm_ID
		PID_Init(&Current_Id_PID, PID_DELTA, 1.5f, 0.3f, 0.00f, 0.0f, 0.0f, 20.0f, 0.2f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Current_Iq_PID, PID_DELTA, 1.5f, 0.2f, 0.00f, 0.0f, 0.0f, 5.8f, 0.2f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Position_Comm_PID, PID_DELTA, 0.0f, 0.002f, 0.0f, 0.0f, 0.0f, 100.0f, 0.01f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Position_PID, PID_POSITION, 1.2f, 0.0005f, 0.003f, 0.0f, 0.0f, 500.0f, 0.9f, 0.1f, 0.1f, 0.1f);
		#elif WHO_AM_I == Slave_0_Arm_ID
		PID_Init(&Current_Id_PID, PID_DELTA, 4.5f, 0.3f, 0.00f, 0.0f, 0.0f, 50.0f, 0.5f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Current_Iq_PID, PID_DELTA, 3.5f, 0.2f, 0.00f, 0.0f, 0.0f, 15.8f, 0.5f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Position_Comm_PID, PID_DELTA, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 100.0f, 0.03f, 0.1f, 0.1f, 0.1f);
		PID_Init(&Position_PID, PID_POSITION, 5.2f, 0.002f, 0.003f, 0.0f, 0.0f, 2.0f, 1.6f, 0.1f, 0.1f, 0.1f);
		#endif
	#endif
}

//======================FOC main=======================//

void FOC_Main_Init(void)
{
	FOC_Struct_Init(&Motor_FOC);
	Error_Struct_Init(&Motor_Error);

	DWT_Init(170);
	Encoder_SPI_Init(&MT6825_spi, 
                    &MT6825_diff_Filter , MT6825_diff_buffer, 
					&MT6825_angle_Filter, MT6825_angle_buffer,
					&MT6825_acc_Filter, MT6825_acc_buffer,
					&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0.01f);
	FDCAN_IntFilterAndStart();
	Adc_Init();
	DRV8323_Init();
	FOC_PID_Init();
	CALC_SVPWM_Init();

	HAL_TIM_Base_Start (&htim1);
	HAL_TIM_Base_Start_IT(&htim1);

	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	setVolume(10); //音量设置
	playStartupTune();    //start music
	
	#if ZERO_POINT_MODE == MODE_ON
	//--------电角度零点校准----------//
	Motor_FOC.ElecTheta = 0;
	Motor_FOC.Iq_ref = 0.0f;
	Motor_FOC.Id_ref = 1.5f;
	
	Inv_Park_transform(Motor_FOC.Vd, Motor_FOC.Vq, &Motor_FOC.Valpha, &Motor_FOC.Vbeta, Motor_FOC.ElecTheta);
	CALC_SVPWM(Motor_FOC.Valpha, Motor_FOC.Vbeta);
	HAL_Delay(1000);

	Encoder_Read_Reg(&MT6825_spi);
	Encoder_Read_Reg(&MT6825_spi);
	Motor_FOC.ElecTheta_Offset = ((int32_t)(MOTOR_POLE_PAIRS * (MT6825_spi.last_angle + 360))% 360) / 360.0f;
	
	Motor_FOC.Iq_ref = 0.0f;
	Motor_FOC.Id_ref = 0.0f;
	#endif

	HAL_TIM_Base_Start_IT(&htim3);
}

void FOC_Main_Loop_H_Freq(void)
{
//----------Theta_Calc---------// 
    Motor_FOC.ElecTheta = FOC_ElecTheta_Calc(MT6825_spi.last_angle);
	
//----------Ia Ib Ic Calc----------//
    Motor_FOC.Ia = (DRV8323_VREF_DIV_TWO - Motor_ADC.Valtage_Current_A);
    Motor_FOC.Ib = (DRV8323_VREF_DIV_TWO - Motor_ADC.Valtage_Current_B);
    Motor_FOC.Ic = (DRV8323_VREF_DIV_TWO - Motor_ADC.Valtage_Current_C);
		
//----------Clarke_Park_transform----------//
    Clarke_transform(Motor_FOC.Ia, Motor_FOC.Ib, Motor_FOC.Ic, &Motor_FOC.Ialpha, &Motor_FOC.Ibeta);
		
    Park_transform(Motor_FOC.Ialpha, Motor_FOC.Ibeta, &Motor_FOC.Id, &Motor_FOC.Iq, Motor_FOC.ElecTheta);

//----------Vd_Vq_Calc----------//
    PID_SetFdb(&Current_Id_PID, Motor_FOC.Id);
    PID_SetRef(&Current_Id_PID, Motor_FOC.Id_ref);
    Motor_FOC.Vd += PID_Calc(&Current_Id_PID);

    PID_SetFdb(&Current_Iq_PID, Motor_FOC.Iq);
    PID_SetRef(&Current_Iq_PID, Motor_FOC.Iq_ref);
    Motor_FOC.Vq += PID_Calc(&Current_Iq_PID);

	static float Vd_Vq_sum_Squares; // Vd_Vq_sum_Squares = Vq * Vq + Vd * Vd
	Vd_Vq_sum_Squares = Motor_FOC.Vq * Motor_FOC.Vq + Motor_FOC.Vd * Motor_FOC.Vd;
	if(Vd_Vq_sum_Squares > MAX_VQVQ_VDVD || Vd_Vq_sum_Squares < - MAX_VQVQ_VDVD)
	{
		float V_k;
		V_k = __sqrtf( MAX_VQVQ_VDVD / Vd_Vq_sum_Squares);
		Motor_FOC.Vq *= V_k;
		Motor_FOC.Vd *= V_k;
	}

//---------Inv_Park_transform----------//
	Inv_Park_transform(Motor_FOC.Vd, Motor_FOC.Vq, &Motor_FOC.Valpha, &Motor_FOC.Vbeta, Motor_FOC.ElecTheta);

//---------SVPWM_transform----------//
    CALC_SVPWM(Motor_FOC.Valpha, Motor_FOC.Vbeta);
		
//---------Running_Frec_Calc----------//
	if (Motor_Run.state_led_flag == 10000)
	{
		Motor_Run.state_led_flag = 0;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
	if ((Motor_Run.state_led_flag & 0x07) == 0)
    {
		Adc_Val_Decode();
		Get_ADC_Value();
		Motor_Run.Adc_flag++;
	}

	Motor_Run.state_led_flag++;
	Motor_Run.run_flag++;
}

void FOC_Main_Loop_L_Freq(void)
{		
	Encoder_Read_Reg(&MT6825_spi);
	Motor_FOC.Theta = Position_Polar * FOC_Theta_Calc(Encoder_SPI_Get_Angle(&MT6825_spi));//单位：rad
	
	if(Motor_FOC.Motor_Close_Loop_Mode == Position_Mode){
        PID_SetFdb(&Position_Comm_PID, Motor_FOC.Theta_Ref);
        PID_SetRef(&Position_Comm_PID, Motor_FOC.Theta_Exp_Comm);
        PID_Calc(&Position_Comm_PID);
        Motor_FOC.Theta_Ref = Motor_FOC.Theta_Ref + Position_Comm_PID.output;
        PID_SetFdb(&Position_PID, Motor_FOC.Theta);//rad
        PID_SetRef(&Position_PID, Motor_FOC.Theta_Ref);//rad
        PID_Calc(&Position_PID);
        // Motor_FOC.Iq_Damping = Motor_FOC.K_Damping * Encoder_SPI_Get_Angular_Acc(&MT6825_spi);
		if(Position_PID.pid_mode == PID_POSITION)
		{
			// Motor_FOC.Iq_ref = -PID_GetOutput(&Position_PID) - Motor_FOC.Iq_Damping;
            Motor_FOC.Iq_ref = -PID_GetOutput(&Position_PID);
		}
    }
}

