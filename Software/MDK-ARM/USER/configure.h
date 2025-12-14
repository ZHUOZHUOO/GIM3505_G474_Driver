/*
 * @Date: 2025-03-09 20:13:14
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2025-11-19 19:38:46
 * @FilePath: \Software\MDK-ARM\USER\configure.h
 * @Description: Do not edit
 */
#ifndef __CONFIGURE_H
#define __CONFIGURE_H

//-----------FDCAN ID Setting------------//
#define WHO_AM_I Slave3_Jaw_ID

#define Tx_Master_ID 	WHO_AM_I | 0x100
#define Rx_Master_ID 	WHO_AM_I

#define Master_ID 			0x410
#define Slave_Upper_Arm_ID 	0x420
#define Slave_Fore_Arm_ID 	0x430
#define Slave_Test_ID 		0x440
#define Slave0_Jaw_ID 	    0x450
#define Slave1_Jaw_ID 	    0x460
#define Slave2_Jaw_ID 	    0x470
#define Slave3_Jaw_ID 	    0x480


//-----------Mode Setting--------------//

//闭环模式
#define FOC_CLOSE_LOOP_MODE MODE_POSITION
//零点校准模式
#define ZERO_POINT_MODE MODE_OFF
//ADC电压校准模式
#define ADC_VREF_MODE MODE_OFF
//滤波模式, Sliding Window Filter
#define ADC_FILTER_MODE MODE_ON
#define SLIDING_WINDOW_SIZE 8
//错误处理模式
#define ERROR_MODE MODE_OFF
//磁编码器角度滤波窗口宽度
#define ANGLE_SLIDING_WINDOW_SIZE 1
//磁编码器差分滤波窗口宽度
#define DIFF_SLIDING_WINDOW_SIZE 128
//磁编码器加速度滤波窗口宽度
#define ACC_SLIDING_WINDOW_SIZE 32

#define MODE_OFF 0
#define MODE_ON 1
#define MODE_POSITION 2
#define MODE_FORCE 3

//------------PWM Setting-------------//
//#define CKTIM 170000000//定时器时钟频率
//#define PWM_PRSC 1-1//PWM预分频
//#define PWM_FREQ 20000//PWM频率, T=1000us
//#define PWM_PERIOD CKTIM/(2*PWM_FREQ*(PWM_PRSC+1))  //计数器计数上限 ARR=3400
//#define REP_RATE 1 //电流环刷新频率为(REP_RATE+1)/(2*PWM_FREQ)=40us, f=25kHz
//#define DEADTIME_NS 1000//死区时间ns
//#define DEADTIME CKTIM/1000000/2*DEADTIME_NS/1000  //死区时间计数值 85us

#define SPEED_STEP (float)(MOTOR_POLE_PAIRS / (float)PWM_FREQ) //速度步进

#define ALIGNMENT_ANGLE 300
#define COUNTER_RESET (ALIGNMENT_ANGLE*4*ENCODER_PPR/360-1)/POLE_PAIR_NUM
#define ICx_FILTER 8

//---------Motor Parameter Define---------//
#define MOTOR_POLE_PAIRS 	11 			//电机极对数
#define MOTOR_VOLTAGE 		24 			//电机电压
#define MOTOR_RESISTANCE 	5.06 		//电机电阻
#define MOTOR_INDUCTANCE 	0.00062 	//电机电感
#define MOTOR_SPEED_MAX  	384			//电机最大速度(空载)
#define MOTOR_CURRENT_MAX 	2.4 		//电机最大电流
#define SPEED_CONSTANT 		16 			//转速常数Kn
#define TORQUE_CONSTANT 	0.52 		//转矩常数KT

//---------Encoder Parameter Define---------//
#define MOTOR_ENCODER_DIR 	1 //电机编码器方向
#define MOTOR_ENCODER_LINES 262144.0f //电机编码器线数 18bits
#define THREE_PHASE_LINE_SEQUENCCE A_C_B
#define MAX_IQ 2.0f
#define MAX_VQ 12.0f
#define MAX_VD 10.0f

#define MAX_VQVQ_VDVD 144.0f

//--------Three phase line sequence Define-------//
#define A_B_C  1
#define A_C_B -1

#endif
