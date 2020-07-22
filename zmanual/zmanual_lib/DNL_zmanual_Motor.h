#include "math.h"
#define PI 3.14159265

int16_t joyLeftHor;
int16_t joyLeftVer;
int16_t joyRigtHor;
int16_t joyRigtVer;

double factorSpeed = 2.7;//cux == 2.5
double factorYawPID = 1.5;
double angle;
double speed;

//int16_t yawError, yawPreError;
//double yawP, yawI, yawD;
//double yawKp = 0.55, yawKd = 0.0, yawKi = 0.0;//0.7//0.6//0.6
//double yawPID;
//#define MAX_YAW_PID 100 // 50// 70 //85
//#define MIN_YAW_PID -MAX_YAW_PID//-50//-70
//double PIDyaw(int _yawValue, int _yawSetpoint)
//{
//	yawError = _yawSetpoint - _yawValue;
//	yawP = yawError;
//	yawD = yawError - yawPreError;
//	yawI = yawError + yawI;
//	yawPID = yawKp*yawP + yawKd*yawD + yawKi*yawI;
//	if(yawPID > MAX_YAW_PID)
//	{
//		yawPID = MAX_YAW_PID;
//	}
//	if(yawPID < MIN_YAW_PID)
//	{
//		yawPID = MIN_YAW_PID;
//	}
//	yawPreError = yawError;
//	return yawPID;
//}



#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#define motor1 htim8
#define motor2 htim8
#define motor3 htim1
#define motor4 htim1
#define motor1_channel TIM_CHANNEL_1
#define motor2_channel TIM_CHANNEL_2
#define motor3_channel TIM_CHANNEL_3
#define motor4_channel TIM_CHANNEL_4

#define FCW GPIO_PIN_RESET
#define CCW GPIO_PIN_SET


// float motor1Speed;
// float motor2Speed;
// float motor3Speed;
// float motor4Speed;
// int motor1Dir;
// int motor2Dir;
// int motor3Dir;
// int motor4Dir;
// int receiveCplt;

uint8_t motor1Speed;
uint8_t motor2Speed;
uint8_t motor3Speed;
uint8_t motor4Speed;
uint8_t motor1Dir;
uint8_t motor2Dir;
uint8_t motor3Dir;
uint8_t motor4Dir;


void peripheralPWM_Init()
{
  HAL_TIM_PWM_Start(&motor1, motor1_channel);
  HAL_TIM_PWM_Start(&motor2, motor2_channel);
  HAL_TIM_PWM_Start(&motor3, motor3_channel);
  HAL_TIM_PWM_Start(&motor4, motor4_channel);	
}

void controlMotor1(int _speed)
{
	if(_speed >= 0)
	{
		HAL_GPIO_WritePin(motor1Dir_GPIO_Port, motor1Dir_Pin, CCW);
	}
	else
	{
		HAL_GPIO_WritePin(motor1Dir_GPIO_Port, motor1Dir_Pin, FCW);
	}
	if((_speed <= 3)&&(_speed >= -3))
		_speed = 3;
	if(_speed > 250)
	{
		_speed = 250;
	}
	if(_speed < -250)
	{
		_speed = -250;
	}
	debugSpeed1 = abs(_speed);
	__HAL_TIM_SetCompare(&motor1, motor1_channel, abs(_speed));
}
void controlMotor2(int _speed)
{
	if(_speed >= 0)
	{
		HAL_GPIO_WritePin(motor2Dir_GPIO_Port, motor2Dir_Pin, CCW);
	}
	else
	{
		HAL_GPIO_WritePin(motor2Dir_GPIO_Port, motor2Dir_Pin, FCW);
	}
	if((_speed <= 3)&&(_speed >= -3))
			_speed = 3;
	if(_speed > 250)
	{
		_speed = 250;
	}
	if(_speed < -250)
	{
		_speed = -250;
	}
	debugSpeed2 = abs(_speed);
	__HAL_TIM_SetCompare(&motor2, motor2_channel, abs(_speed));
}
void controlMotor3(int _speed)
{
	if(_speed >= 0)
	{
		HAL_GPIO_WritePin(motor3Dir_GPIO_Port, motor3Dir_Pin, CCW);
	}
	else
	{
		HAL_GPIO_WritePin(motor3Dir_GPIO_Port, motor3Dir_Pin, FCW);
	}
	if((_speed <= 3)&&(_speed >= -3))
				_speed = 3;
	if(_speed > 250)
	{
		_speed = 250;
	}
	if(_speed < -250)
	{
		_speed = -250;
	}
	debugSpeed3 = abs(_speed);
	__HAL_TIM_SetCompare(&motor3, motor3_channel, abs(_speed));
}
void controlMotor4(int _speed)
{
	if(_speed >= 0)
	{
		HAL_GPIO_WritePin(motor4Dir_GPIO_Port, motor4Dir_Pin, CCW);
	}
	else
	{
		HAL_GPIO_WritePin(motor4Dir_GPIO_Port, motor4Dir_Pin, FCW);
	}
	if((_speed <= 3)&&(_speed >= -3))
		_speed = 3;
	if(_speed > 250)
	{
		_speed = 250;
	}
	if(_speed < -250)
	{
		_speed = -250;
	}
	debugSpeed4 = abs(_speed);
	__HAL_TIM_SetCompare(&motor4, motor4_channel, abs(_speed));
}

void testPWM(void)
{
  for(int i = -1; i > -255; --i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
	  tracking++;
  }
  for(int i = -255; i < 0; ++i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
	  tracking++;
  }
  for(int i = 1 ; i < 255; ++i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
	  tracking++;
  }
  for(int i = 255; i > 0; --i)
  {
	  controlMotor1(i);
	  controlMotor2(i);
	  controlMotor3(i);
	  controlMotor4(i);
	  HAL_Delay(20);
	  tracking++;
  }
}
