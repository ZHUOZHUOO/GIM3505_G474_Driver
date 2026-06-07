/*
 * @Date: 2025-03-09 20:13:14
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2026-03-22 15:37:22
 * @FilePath: \SoftwareV2\MDK-ARM\USER\configure.h
 * @Description: Do not edit
 */
#ifndef __CONFIGURE_H
#define __CONFIGURE_H

//-----------FDCAN ID Setting------------//
#define WHO_AM_I Slave_Upper_Arm_ID

#define Tx_Master_ID 	WHO_AM_I | 0x100
#define Rx_Master_ID 	WHO_AM_I

#define Slave_Base_Yaw_ID 	0x410
#define Slave_Upper_Arm_ID 	0x420
#define Slave_Fore_Arm_ID 	0x430
#define Slave0_Jaw_ID 	    0x450 	//21
#define Slave1_Jaw_ID 	    0x460	//11
#define Slave2_Jaw_ID 	    0x470	//12
#define Slave3_Jaw_ID 	    0x480	//22


//-----------Mode Setting--------------//

//闭环模式
#define FOC_CLOSE_LOOP_MODE MODE_FORCE
//零点校准模式
#define ZERO_POINT_MODE MODE_OFF
//ADC电压校准模式
#define ADC_VREF_MODE MODE_OFF
//滤波模式, Sliding Window Filter
#define ADC_FILTER_MODE MODE_ON
#define SLIDING_WINDOW_SIZE 4
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
#define MOTOR_MODEL          GIM4310     //电机型号

#define GIM3505 1
#define GIM4305 2
#define GIM4310 3

#if MOTOR_MODEL == GIM3505
#define MOTOR_POLE_PAIRS 	(float)11.0f 			//电机极对数
#define MOTOR_VOLTAGE 		(float)24.0f 			//电机电压
#define MOTOR_RESISTANCE 	(float)5.06f 		    //电机电阻
#define MOTOR_INDUCTANCE 	(float)0.00062f 	    //电机电感
#define MOTOR_SPEED_MAX  	(float)384			    //电机最大速度(空载)
#define MOTOR_CURRENT_MAX 	(float)2.4f 		    //电机最大电流
#define SPEED_CONSTANT 		(float)16.0f 		    //转速常数Kn
#define TORQUE_CONSTANT 	(float)0.52f 		    //转矩常数KT
#endif

#if MOTOR_MODEL == GIM4305
#define MOTOR_POLE_PAIRS 	(float)14.0f 			//电机极对数
#define MOTOR_VOLTAGE 		(float)24.0f 			//电机电压
#define MOTOR_RESISTANCE 	(float)0.638f 		    //电机电阻
#define MOTOR_INDUCTANCE 	(float)0.000169f 	    //电机电感
#define MOTOR_SPEED_MAX  	(float)540			    //电机最大速度(空载)
#define MOTOR_CURRENT_MAX 	(float)3.0f 		    //电机最大电流
#define SPEED_CONSTANT 		(float)15.4f 		    //转速常数Kn
#define TORQUE_CONSTANT 	(float)0.21f 		    //转矩常数KT
#endif

#if MOTOR_MODEL == GIM4310
#define MOTOR_POLE_PAIRS 	(float)14.0f 			//电机极对数
#define MOTOR_VOLTAGE 		(float)24.0f 			//电机电压
#define MOTOR_RESISTANCE 	(float)1.046f 		    //电机电阻
#define MOTOR_INDUCTANCE 	(float)0.000344f 	    //电机电感
#define MOTOR_SPEED_MAX  	(float)100			    //电机最大速度(空载)
#define MOTOR_CURRENT_MAX 	(float)8.0f 		    //电机最大电流
#define SPEED_CONSTANT 		(float)9.25f 		    //转速常数Kn
#define TORQUE_CONSTANT 	(float)0.66f 		    //转矩常数KT
#endif

//---------Encoder Parameter Define---------//
#define MOTOR_ENCODER_DIR 	(float)1.0f            //电机编码器方向，与编码器安装方向有关
#define MOTOR_ENCODER_LINES (float)262144.0f        //电机编码器线数 18bits
#define MAX_IQ (float)2.0f
#define MAX_VQ (float)12.0f
#define MAX_VD (float)10.0f

#define MAX_VQVQ_VDVD (float)144.0f

#endif
