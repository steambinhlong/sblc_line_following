#define INTIAL_SPEED 30

typedef enum MODES{
  STOP = 0,
  LEARN_WHITE,
  LEARN_BLACK,
  AUTO_CALIB,
  RUN
} MODES;

typedef struct Robot
{
  uint8_t PWM1_Pin;
  uint8_t PWM2_Pin;
  uint8_t PWM3_Pin;
  uint8_t PWM4_Pin;

  uint8_t BT1_Pin;
  uint8_t BT2_Pin;
  uint8_t BT3_Pin;

  uint8_t sensor[3];

  uint8_t R_LED_Pin;
  uint8_t G_LED_Pin; 
  uint8_t B_LED_Pin; 

  uint8_t BUZZER_Pin;
  uint8_t ADC_LED[3];

  uint8_t sensor_mask;
  uint16_t sensor_value[3];
  uint16_t white_buffer[3];
  uint16_t black_buffer[3];
  uint16_t v_compare[3];
  uint16_t calib_weight[3];

  uint8_t run_case;

  int16_t left_speed;
  int16_t right_speed;

  float kP;
  float kI;
  float kD;
  float error;
  float pre_error;
  float uP;
  float uI;
  float pre_uI;
  float uD;
  float u;
} Robot;
Robot SBLC;


void robot_setPin(Robot *robot, 
            uint8_t PWM1_Pin,
            uint8_t PWM2_Pin,
            uint8_t PWM3_Pin,
            uint8_t PWM4_Pin,
            uint8_t BT1_Pin,
            uint8_t BT2_Pin,
            uint8_t BT3_Pin,
            uint8_t ADC1_Pin,
            uint8_t ADC2_Pin,
            uint8_t ADC3_Pin,
            uint8_t LED1_Pin,
            uint8_t LED2_Pin,
            uint8_t LED3_Pin,
            uint8_t R_LED_Pin, 
            uint8_t G_LED_Pin, 
            uint8_t B_LED_Pin,
            uint8_t BUZZER_Pin
            )
{
  robot -> PWM1_Pin = PWM1_Pin;
  robot -> PWM2_Pin = PWM2_Pin;
  robot -> PWM3_Pin = PWM3_Pin;
  robot -> PWM4_Pin = PWM4_Pin;

  robot -> BT1_Pin = BT1_Pin;
  robot -> BT2_Pin = BT2_Pin;
  robot -> BT3_Pin = BT3_Pin;

  robot -> sensor[0] = ADC1_Pin;
  robot -> sensor[1] = ADC2_Pin;
  robot -> sensor[2] = ADC3_Pin;

  robot -> ADC_LED[0] = LED1_Pin;
  robot -> ADC_LED[1] = LED2_Pin;
  robot -> ADC_LED[2] = LED3_Pin;

  robot -> R_LED_Pin = R_LED_Pin;
  robot -> G_LED_Pin = G_LED_Pin;
  robot -> B_LED_Pin = B_LED_Pin;

  robot -> BUZZER_Pin = BUZZER_Pin;

}

void robot_configPin(Robot *robot)
{
  // PWR->CSR &= ~PWR_CSR_EWUP;
  pinMode(robot -> PWM1_Pin, OUTPUT);
  pinMode(robot -> PWM2_Pin, OUTPUT);
  pinMode(robot -> PWM3_Pin, OUTPUT);
  pinMode(robot -> PWM4_Pin, OUTPUT);

  pinMode(robot -> BT1_Pin, INPUT);
  pinMode(robot -> BT2_Pin, INPUT);
  pinMode(robot -> BT3_Pin, INPUT);

  pinMode(robot -> sensor[0], INPUT_ANALOG);
  pinMode(robot -> sensor[1], INPUT_ANALOG);
  pinMode(robot -> sensor[2], INPUT_ANALOG);


  pinMode(robot -> ADC_LED[0], OUTPUT);
  pinMode(robot -> ADC_LED[1], OUTPUT);
  pinMode(robot -> ADC_LED[2], OUTPUT);

  pinMode(robot -> R_LED_Pin, OUTPUT);
  pinMode(robot -> G_LED_Pin, OUTPUT);
  pinMode(robot -> B_LED_Pin, OUTPUT);

  pinMode(robot -> BUZZER_Pin, OUTPUT);
}


void robot_init(Robot *robot)
{
  robot_setPin(robot,
               PA1,       // PWM1
               PA0,       // PWM2
               PA3,       // PWM3
               PA4,       // PWM4
               PA13,      // BT1
               PA7,       // BT2
               PB0,       // BT3
               PA6,       // ADC1
               PA5,       // ADC2
               PA4,       // ADC3
               PB10,      // LED1
               PB2,       // LED2
               PA8,       // LED3
               PB15,      // GREEN LED
               PB14,      // BLUE LED
               PB13,      // RED LED
               PB3        // BUZZER
               );
  robot_configPin(robot);
  robot_setPIDParams(robot, 0.01, 0, 0);
  // Serial.begin(9600);
}

void robot_setPIDParams(Robot *robot, float kP, float kI, float kD)
{
  robot -> kP = kP;
  robot -> kI = kI;
  robot -> kD = kD;
}

void robot_beepLong(Robot *robot, uint16_t delay_ms)
{
  digitalWrite(robot -> BUZZER_Pin, 1);
  delay(delay_ms);
  digitalWrite(robot -> BUZZER_Pin, 0);
}

void robot_setRGB(Robot *robot, uint8_t R, uint8_t G, uint8_t B)
{
  digitalWrite(robot -> R_LED_Pin, R);
  digitalWrite(robot -> G_LED_Pin, G);
  digitalWrite(robot -> B_LED_Pin, B);
}

void robot_setSpeed(Robot *robot, int16_t left_speed, int16_t right_speed)
{
  if(left_speed > 0)
  {
    analogWrite(robot -> PWM1_Pin, left_speed);
    analogWrite(robot -> PWM2_Pin, 0);
  }
  else if(left_speed < 0)
  {
    analogWrite(robot -> PWM1_Pin, 0);
    analogWrite(robot -> PWM2_Pin, abs(left_speed));
  }
  else
  {
    analogWrite(robot -> PWM1_Pin, 0);
    analogWrite(robot -> PWM2_Pin, 0);
  }
  if(right_speed > 0)
  {
    analogWrite(robot -> PWM3_Pin, right_speed);
    analogWrite(robot -> PWM4_Pin, 0);
  }
  else if(right_speed < 0)
  {
    analogWrite(robot -> PWM3_Pin, 0);
    analogWrite(robot -> PWM4_Pin, abs(right_speed));
  }
  else
  {
    analogWrite(robot -> PWM3_Pin, 0);
    analogWrite(robot -> PWM4_Pin, 0);
  }
}

void robot_readSensor(Robot *robot)
{
  for(uint8_t i = 0; i < 3; i++)
  {
    robot -> sensor_value[i] = analogRead(robot -> sensor[i]);
  }
  // Serial.print("S1: ");
  // Serial.print(robot -> sensor_value[0]);
  // Serial.print(" - S2: ");
  // Serial.print(robot -> sensor_value[1]);
  // Serial.print(" - S3: ");
  // Serial.println(robot -> sensor_value[2]);
}

void robot_learnWhite(Robot *robot)
{
  for(uint8_t i = 0; i < 3; i++)
  {
    robot -> white_buffer[i] = robot -> sensor_value[i];
  }
  for(uint8_t i = 0; i < 3; i++)
  {
    robot -> v_compare[i] = (robot -> white_buffer[i] + robot -> black_buffer[i]) / 2 + robot -> calib_weight[i];
  }
}

void robot_learnBlack(Robot *robot)
{
  for(uint8_t i = 0; i < 3; i++)
  {
    robot -> black_buffer[i] = robot -> sensor_value[i];
  }
  for(uint8_t i = 0; i < 3; i++)
  {
    robot -> v_compare[i] = (robot -> white_buffer[i] + robot -> black_buffer[i]) / 2 + robot -> calib_weight[i];
  }
}

uint8_t robot_getSensorMask(Robot *robot)
{
  uint8_t temp = 0;
  for(uint8_t i = 0; i < 3; i++)
  {
    temp <<= 1;
    if(robot -> sensor_value[i] > robot -> v_compare[i])
    {
      temp |= 0x01;
    }
    else
    {
      temp &= 0x06;
    }
  }
  return temp; 
}

void robot_writeADCLED(Robot *robot, uint8_t sensor_mask)
{
  switch(sensor_mask)
  {
    case 0x01:
      digitalWrite(robot -> ADC_LED[0], 0);
      digitalWrite(robot -> ADC_LED[1], 0);
      digitalWrite(robot -> ADC_LED[2], 1);
      break;
    case 0x02:
      digitalWrite(robot -> ADC_LED[0], 0);
      digitalWrite(robot -> ADC_LED[1], 1);
      digitalWrite(robot -> ADC_LED[2], 0);
      break;
    case 0x03:
      digitalWrite(robot -> ADC_LED[0], 0);
      digitalWrite(robot -> ADC_LED[1], 1);
      digitalWrite(robot -> ADC_LED[2], 1);
      break;
    case 0x04:
      digitalWrite(robot -> ADC_LED[0], 1);
      digitalWrite(robot -> ADC_LED[1], 0);
      digitalWrite(robot -> ADC_LED[2], 0);
      break;
    case 0x05:
      digitalWrite(robot -> ADC_LED[0], 1);
      digitalWrite(robot -> ADC_LED[1], 0);
      digitalWrite(robot -> ADC_LED[2], 1);
      break;
    case 0x06:
      digitalWrite(robot -> ADC_LED[0], 1);
      digitalWrite(robot -> ADC_LED[1], 1);
      digitalWrite(robot -> ADC_LED[2], 0);
      break;
    case 0x07:
      digitalWrite(robot -> ADC_LED[0], 1);
      digitalWrite(robot -> ADC_LED[1], 1);
      digitalWrite(robot -> ADC_LED[2], 1);
      break;
    case 0x00:
      digitalWrite(robot -> ADC_LED[0], 0);
      digitalWrite(robot -> ADC_LED[1], 0);
      digitalWrite(robot -> ADC_LED[2], 0);
      break;
  }
}

void robot_autoCalib(Robot *robot)
{
  for(uint16_t i = 0; i < 30000; i++)
  {
    robot_setSpeed(robot, -30, 30);
    for(uint8_t i = 0; i < 3; i++)
    {
      if(analogRead(robot -> sensor[i]) > 2000)
      {
        robot -> black_buffer[i] = analogRead(robot -> sensor[i]);
      }
      else
      {
        robot -> white_buffer[i] = analogRead(robot -> sensor[i]);
      }
    }
  }
  for(uint8_t i = 0; i < 3; i++)
  {
    robot -> v_compare[i] = (robot -> white_buffer[i] + robot -> black_buffer[i]) / 2;
  }
  robot_setSpeed(robot, 0, 0);
}

void robot_buttonHandle(Robot *robot)
{
  if(!digitalRead(robot -> BT1_Pin))
  {
    robot -> run_case = LEARN_WHITE;
  }
  if(!digitalRead(robot -> BT2_Pin))
  {
    robot -> run_case = LEARN_BLACK;
  }
  if(!digitalRead(robot -> BT3_Pin))
  {
    robot -> run_case = RUN;
  }
}

void robot_PIDCalib(Robot *robot)
{
  robot -> error = analogRead(robot -> sensor[2]) - analogRead(robot -> sensor[0]);

  robot -> uP = robot -> kP * robot -> error;
  robot -> uI = robot -> pre_uI + robot -> kI * robot -> error;

  if(robot -> uI > 50) robot -> uI = 50;
  else if(robot -> uI < -50)  robot -> uI = -50;

  robot -> uD = robot -> kD * (robot -> pre_error - robot -> error);

  robot -> u = robot -> uP + robot -> uI + robot -> uD;

  if(robot -> u > 50) robot -> u = 50;
  else if(robot -> u < -50)  robot -> u = -50;
  
  robot -> pre_uI = robot -> uI;
  robot -> pre_error = robot -> error;

  robot -> left_speed = INTIAL_SPEED + robot -> u;
  robot -> right_speed = INTIAL_SPEED - robot -> u;
}

void robot_run(Robot *robot)
{
  switch(robot -> run_case)
  {
    case STOP:
      robot_readSensor(robot);
      robot_writeADCLED(robot, robot_getSensorMask(robot));
      robot_setRGB(robot, 1, 0, 0);
      robot_getSensorMask(robot);
      break;
    case LEARN_WHITE:
      robot_setRGB(robot, 1, 1, 0);
      robot_learnWhite(robot);
      robot -> run_case = STOP;
      break;
    case LEARN_BLACK:
      robot_setRGB(robot, 1, 1, 0);
      robot_learnBlack(robot);
      robot -> run_case = STOP;
      break;
    case RUN:
      robot_PIDCalib(robot);
      robot_setSpeed(robot, robot -> left_speed, robot -> right_speed);
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  robot_init(&SBLC);
}

void loop() {
  // put your main code here, to run repeatedly:
  robot_run(&SBLC);
}
