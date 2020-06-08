#include "math.h"

#define PI 3.14159265
#define deg2Rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define rad2Deg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

int16_t yawError, yawPreError;
double yawP, yawI, yawD;
double yawKp = 0.05;
double yawKd = 20;
double yawKi = 0.0000000001;
double yawPID;
#define MAX_YAW_PID 100 // 50// 70 //85
#define MIN_YAW_PID -MAX_YAW_PID//-50//-70

double PIDyaw(int _yawValue, int _yawSetpoint)
{
	yawError = -_yawSetpoint + _yawValue;
	yawP = yawError;
	yawD = yawError - yawPreError;
	yawI = yawError + yawI;
	yawPID = yawKp*yawP + yawKd*yawD + yawKi*yawI;
	if(yawPID > MAX_YAW_PID)
	{
		yawPID = MAX_YAW_PID;
	}
	if(yawPID < MIN_YAW_PID)
	{
		yawPID = MIN_YAW_PID;
	}
	yawPreError = yawError;
	return yawPID;
}
