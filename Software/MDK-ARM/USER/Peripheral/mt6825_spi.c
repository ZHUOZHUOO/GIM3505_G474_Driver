#include "mt6825_spi.h"

Encoder_SPI_HandleTypeDef MT6825_spi;
SlidingWindowFilter MT6825_diff_Filter;
SlidingWindowFilter MT6825_angle_Filter;
SlidingWindowFilter MT6825_acc_Filter;
float MT6825_diff_buffer[DIFF_SLIDING_WINDOW_SIZE];
float MT6825_angle_buffer[ANGLE_SLIDING_WINDOW_SIZE];
float MT6825_acc_buffer[ACC_SLIDING_WINDOW_SIZE];

void MT6825_Continuous_Reading_Init(Encoder_SPI_HandleTypeDef *encoder)
{
	uint8_t tx_buffer[1] = {0};
	
	tx_buffer[0] = 0x83;
	HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(encoder->hspi, tx_buffer, 1, 100);
}

void Encoder_SPI_Init(  Encoder_SPI_HandleTypeDef *encoder,
					    SlidingWindowFilter *diff_filter,float *diff_buffer,
                        SlidingWindowFilter *angle_filter,float *angle_buffer,
                        SlidingWindowFilter *acc_filter,float *acc_buffer,
                        SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                        uint16_t cs_pin, float radius) {
    encoder->hspi = hspi;
    encoder->cs_port = cs_port;
    encoder->cs_pin = cs_pin;
    encoder->radius = radius;
    encoder->multi_turn = 0;
    encoder->last_multi_turn = 0;
    encoder->last_angle = 0;
    encoder->angle_diff = 0;
    encoder->turns = 0;
    encoder->angular_speed = 0;
    encoder->last_angular_speed = 0;
    encoder->linear_speed = 0;
    encoder->rawAngle = 0;
    encoder->last_rawAngle = 0;
    encoder->rawAngle_diff = 0;
    encoder->last_update_time = HAL_GetTick();
    encoder->angle_filter = angle_filter;
    encoder->angle_buffer = angle_buffer;
    encoder->angle_diff_Filter = diff_filter;
    encoder->angle_diff_buffer = diff_buffer;
    encoder->acc_filter = acc_filter;
    encoder->acc_buffer = acc_buffer;
    SlidingWindowFilter_Init(encoder->angle_diff_Filter, encoder->angle_diff_buffer, DIFF_SLIDING_WINDOW_SIZE);
    SlidingWindowFilter_Init(encoder->angle_filter, encoder->angle_buffer, ANGLE_SLIDING_WINDOW_SIZE);
    SlidingWindowFilter_Init(encoder->acc_filter, encoder->acc_buffer, ACC_SLIDING_WINDOW_SIZE);
		
	MT6825_Continuous_Reading_Init(encoder);
}

uint8_t rx_flag[8];

void Encoder_SPI_Data_Process(Encoder_SPI_HandleTypeDef *encoder,
                              uint8_t *buffer) {
    uint64_t now_time = DWT_GetTimeline_us();
    // 读取编码器角度值（0-262144范围）
    int32_t encoderRawData = ((buffer[0] & 0xFF) << 10) | ((buffer[1] & 0xFC) << 2) | ((buffer[2] & 0xF0) >> 4);
    rx_flag[0] = (buffer[2] & 0x01);
    rx_flag[1] = (buffer[2] & 0x02)>>1;
    rx_flag[2] = (buffer[2] & 0x04)>>2;
    rx_flag[3] = (buffer[2] & 0x08)>>3;
    rx_flag[4] = (buffer[2] & 0x10)>>4;
    rx_flag[5] = (buffer[2] & 0x20)>>5;
    rx_flag[6] = (buffer[2] & 0x40)>>6;
    rx_flag[7] = (buffer[2] & 0x80)>>7;
    // 将编码器值转换为-180°~180°的角度
    encoder->rawAngle = (float)encoderRawData * (360.0f / MOTOR_ENCODER_LINES);

    // 计算角度的差值，考虑过零点情况
    encoder->rawAngle_diff = encoder->rawAngle - encoder->last_rawAngle;
    if (encoder->rawAngle_diff > 180.0f) {
    // 说明角度从负值跳到正值（即过零点）
        encoder->rawAngle_diff -= 360.0f;
    } else if (encoder->rawAngle_diff < -180.0f) {
    // 说明角度从正值跳到负值（即过零点）
        encoder->rawAngle_diff += 360.0f;
    }

    // 根据角度差值判断圈数变化
    if (encoder->rawAngle_diff < 0 && encoder->last_rawAngle < 90 && encoder->rawAngle > 270) {
        encoder->multi_turn--;
    } else if (encoder->rawAngle_diff > 0 && encoder->last_rawAngle > 270 && encoder->rawAngle < 90) {
        encoder->multi_turn++;
    }
    // 计算角度差分值
    encoder->angle_diff = SlidingWindowFilter_Update(encoder->angle_diff_Filter, encoder->rawAngle_diff);

    // 将角度差转换为角速度 (rad/s)
    float sampling_period = (float)(now_time - encoder->last_update_time) / 1000000.0f; // us->s
    encoder->last_update_time = now_time;
    encoder->last_angular_speed = encoder->angular_speed;
    encoder->angular_speed = (encoder->angle_diff / 180.0f * 3.1415926f) / sampling_period; // rad/s
	
	// 计算加速度
	float acc_temp = (encoder->angular_speed - encoder->last_angular_speed) / sampling_period;
    encoder->acc = SlidingWindowFilter_Update(encoder->acc_filter, acc_temp);

    // 根据角速度计算线速度 (m/s)
    encoder->linear_speed = encoder->angular_speed * encoder->radius;

    // 保存上次的角度和多圈数
    encoder->last_rawAngle = encoder->rawAngle;
    encoder->last_angle = SlidingWindowFilter_Update(encoder->angle_filter, encoder->rawAngle);
    encoder->last_multi_turn = encoder->multi_turn;
}

void Encoder_SPI_Reset(Encoder_SPI_HandleTypeDef *encoder) {
  encoder->multi_turn = 0;
  encoder->last_multi_turn = 0;
  encoder->last_angle = 0;
  encoder->angle_diff = 0;
  encoder->turns = 0;
  encoder->angular_speed = 0;
  encoder->linear_speed = 0;
}

/**
 * @brief get the angle 角度
 *
 * @param encoder
 * @return float
 */
float Encoder_SPI_Get_Angle(Encoder_SPI_HandleTypeDef *encoder) {
  return encoder->last_angle + encoder->multi_turn * 360.0f;
}

/**
 * @brief get the angular speed 角速度
 *
 * @param encoder
 * @return float
 */
float Encoder_SPI_Get_Angular_Speed(Encoder_SPI_HandleTypeDef *encoder) {
  return encoder->angular_speed;
}

/**
 * @brief get the angular acceleration 角加速度
 *
 * @param encoder
 * @return float
 */
float Encoder_SPI_Get_Angular_Acc(Encoder_SPI_HandleTypeDef *encoder) {
  return encoder->acc;
}

void Encoder_Read_Reg(Encoder_SPI_HandleTypeDef *encoder) {
	static uint8_t txbuffer[3] = {0x00, 0x00, 0x00};

	HAL_SPI_Transmit(encoder->hspi, &txbuffer[0], 1, 0);
	HAL_SPI_Transmit(encoder->hspi, &txbuffer[1], 1, 0);
	HAL_SPI_Transmit(encoder->hspi, &txbuffer[2], 1, 0);
	
	Encoder_SPI_Data_Process(encoder, encoder->rx_buffer);
	HAL_SPI_Receive(encoder->hspi, encoder->rx_buffer, 3, 10);

	Motor_Run.spi_flag++;
}
