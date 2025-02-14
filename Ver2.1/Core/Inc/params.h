/*
 * params.h
 *
 *  Created on: Dec 8, 2024
 *      Author: KayChip
 */

// toc do 200-400 -> kP = 0.3, kD = 0.01;
// toc do 500 -> kP = 0.3, kD = 0.02;
// toc do 700 -> kP = 1, kD = 0.02;
// toc do 900 -> kP = 4, kD = 0.3;

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

#define FLASH_ADDR_BASE 0x08000000
#define FLASH_ADDR_TARGET_PAGE 127
#define FLASH_ADDR_TARGET (FLASH_ADDR_BASE + 1024*FLASH_ADDR_TARGET_PAGE)

/*---------------------------BEGIN: PARAMETERS SETTING----------------------*/
// Ngưỡng học màu
#define MIN_ADC_VAL 2000		// Giá trị thấp nhất của cảm biến
#define MAX_ADC_VAL 2000		// Giá trị cao nhất của cảm biến

// Giá trị bắt xa nhất của cảm biến tiệm cận
#define TC_DETECT_VALUE 700

// Độ lợi cảm biến (hiệu chỉnh để bắt line ổn hơn)
#define CALIB_LINE_SENSOR1_VALUE 0
#define CALIB_LINE_SENSOR2_VALUE 200
#define CALIB_LINE_SENSOR3_VALUE 300
/*---------------------------END: PARAMETERS SETTING------------------------*/

// Giới hạn PID
#define PID_LIMIT_TOP 1000
#define PID_LIMIT_BOT -1000

// Thời gian học màu
#define TIME_LEARN_AUTO 500000	// có thể thay đổi
#define SPEED_LEARN_AUTO 200	// tốc độ quay khi học màu

// Số lượng cảm biến
#define NUM_OF_LINE_SENSOR 3
#define NUM_OF_ALL_SENSORS 4
#define NUM_OF_LED 3			// LED cảm biến

// Mặt nạ
#define MASK_4BIT 0x0F			// 0b00001111
#define MASK_111 0x07			// 0b00000111
#define MASK_010 0x02			// 0b00000010
#define MASK_110 0x06			// 0b00000110
#define MASK_001 0x01			// 0b00000001



// UART - APP COMMUNICATION
#define SIZE_TX_DATA 39
#define SIZE_COMMAND 2
#define SIZE_DATA 39
#define TIMEOUT_RECEIVING_DATA 800
#define WORD_DISTANCE_BETWEEN 4

// RGB
#define TIMEBLINK_RGB 100
#define TIMEBLINK_RGB_END 1000

// BUZZER
#define DEFAULT_TIME_BEEP 100
#define DEFAULT_BEEP_NUMS 3
#define LEARN_AUTO_TIME_BEEP 500
#define LEARN_MANUAL_TIME_BEEP 200
#define RUN_NO_TC_TIME_BEEP 1000

// BUTTON
#define BT1_PRESSED 7
#define BT2_PRESSED 11
#define BT3_PRESSED 13
#define BT4_PRESSED 14
#define ALL_BT_UP 15

// Gia tốc cộng dồn
#define ACCEL_SPEED 2

#define SPEED_NORMAL 900
#define SPEED_BRAKE 800

#define TIMEOUT_WRITE_FLASH 500


// Các trường hợp chạy
typedef enum RUN_CASE
{
	STOP = 0,
	LEARN_AUTO,
	LEARN_MANUAL,
	RUN_NO_TC,
	RUN_WITH_TC
} RUN_CASE;


// Các phần chỉnh sửa chính
float deltaT = 0.001;
float kP;
float kD;
uint16_t targetSpeed;
uint8_t accel;

float kP_1;
float kD_1;
uint16_t targetSpeed_1;
uint8_t accel_1;

float kP_2;
float kD_2;
uint16_t targetSpeed_2;
uint8_t accel_2;

uint16_t calib_weight[NUM_OF_LINE_SENSOR] = {0};
uint16_t gate_sensor_value;

int16_t center_val;
int16_t pid_limit_top;
int16_t pid_limit_bot;
// Các khâu PID
float uP, uD, u;

// Biến đếm theo chu kỳ
uint16_t cnt = 0;

// Trường hợp chạy
uint8_t run_case;

// Tốc độ
int16_t left_speed, right_speed, speed;

// Chiều quay
uint8_t Dir_Left = 0, Dir_Right = 0, Dir_PID = 0;

// ADC
uint8_t sensor_mask;	// Trạng thái hiện tại của cảm biến
uint16_t sensor_value[NUM_OF_ALL_SENSORS];	// Giá trị cảm biên
uint16_t minOfMax[NUM_OF_LINE_SENSOR] = {MIN_ADC_VAL};		// Màu trắng
uint16_t maxOfMin[NUM_OF_LINE_SENSOR] = {MAX_ADC_VAL};		// Màu đen
uint16_t v_compare[NUM_OF_LINE_SENSOR];		// Màu xám

// LED cảm biển
GPIO_TypeDef *sensor_led_port[NUM_OF_LED] = {LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port};		// Port
uint16_t sensor_led_pin[NUM_OF_LED] = {LED1_Pin, LED2_Pin, LED3_Pin};	// Chân

// Cờ - Flag
uint8_t pid_enable = 0;
uint8_t run_with_sensor = 0;
uint8_t readyToSend = 0;
uint8_t readyToAssign = 0;
uint8_t readyToWrite = 0;
uint8_t isConnected = 0;

// Lỗi PID
float err, pre_err;

// Trạng thái nút nhấn
uint8_t button_event;

// Mảng nhận dữ liệu từ UART
uint8_t tx_data[SIZE_TX_DATA];
uint8_t cmd[SIZE_COMMAND];
uint8_t data[SIZE_DATA];

#endif /* INC_PARAMS_H_ */
