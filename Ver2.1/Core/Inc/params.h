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

#define MIN_ADC_VAL 1200
#define MAX_ADC_VAL 2500

#define CALIB_LINE_SENSOR1_VALUE 0
#define CALIB_LINE_SENSOR2_VALUE 279
#define CALIB_LINE_SENSOR3_VALUE 345

#define PID_LIMIT_TOP 1000
#define PID_LIMIT_BOT -1000

#define TIME_LEARN_AUTO 500000
#define SPEED_LEARN_AUTO 200

#define NUM_OF_LINE_SENSOR 3
#define NUM_OF_ALL_SENSORS 4


#define TC_DETECT_VALUE 700

#define MASK_4BIT 0x0F
#define MASK_111 0x07
#define MASK_010 0x02
#define MASK_110 0x06
#define MASK_001 0x01

#define SIZE_TX_DATA 10
#define SIZE_COMMAND 2
#define SIZE_DATA 10
#define TIMEOUT_RECEIVING_DATA 200
#define WORD_DISTANCE_BETWEEN 4

#define TIMEBLINK_RGB 100
#define TIMEBLINK_RGB_END 1000

#define DEFAULT_TIME_BEEP 100
#define DEFAULT_BEEP_NUMS 3
#define LEARN_AUTO_TIME_BEEP 500
#define LEARN_MANUAL_TIME_BEEP 200
#define RUN_NO_TC_TIME_BEEP 1000

#define BT1_PRESSED 7
#define BT2_PRESSED 11
#define BT3_PRESSED 13
#define BT4_PRESSED 14
#define ALL_BT_UP 15

#define NUM_OF_LED 3

#define ACCEL_SPEED 2

/* -------------- BEGIN: CONFIG PARAMETER--------------*/
uint16_t intialSpeed;
float deltaT = 0.001;
float kP;
float kD;
/* -------------- END: CONFIG PARAMETER----------------*/


typedef enum RUN_CASE
{
	STOP = 0,
	LEARN_AUTO,
	LEARN_MANUAL,
	RUN_NO_TC,
	RUN_WITH_TC
} RUN_CASE;

uint16_t cnt = 0;

uint8_t run_case;

int16_t left_speed, right_speed, intial_speed;

uint8_t sensor_mask;
uint16_t sensor_value[NUM_OF_ALL_SENSORS];
uint16_t minOfMax[NUM_OF_LINE_SENSOR] = {MIN_ADC_VAL};
uint16_t maxOfMin[NUM_OF_LINE_SENSOR] = {MAX_ADC_VAL};
uint16_t v_compare[NUM_OF_LINE_SENSOR] = {0};
uint16_t calib_weight[NUM_OF_LINE_SENSOR] = {CALIB_LINE_SENSOR1_VALUE, CALIB_LINE_SENSOR2_VALUE, CALIB_LINE_SENSOR3_VALUE};
GPIO_TypeDef *sensor_led_port[NUM_OF_LED] = {LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port};
uint16_t sensor_led_pin[NUM_OF_LED] = {LED1_Pin, LED2_Pin, LED3_Pin};

uint8_t pid_enable = 0, run_with_sensor = 0;

float err, pre_err;
float uP, uD, u;

uint8_t button_event;

uint8_t tx_data[SIZE_TX_DATA], cmd[SIZE_COMMAND], data[SIZE_DATA];
uint8_t readyToAssign = 0;

#endif /* INC_PARAMS_H_ */
