#define hand htim5

#define handForward GPIO_PIN_RESET
#define handBackward GPIO_PIN_SET

#define HAND_STATUS_READY 	0
#define HAND_STATUS_PUT 	1
#define HAND_STATUS_WAIT 	2

#define HAND_PUL_RUNUP 200
#define HAND_PUL_RUNUP2 1400
#define HAND_PUL_END 450
#define HAND_PUL_SHOOT 1300
#define HAND_DELAYUS_SHOOT 100

int trackingHand;
int trackingHandShoot;

int handEn;

int handStatus;

int handElapsedPulses;

uint32_t delayTick001;

void handControl_Init(void)
{
	  HAL_TIM_Base_Start_IT(&hand);
	  HAL_GPIO_WritePin(handEn_GPIO_Port, handEn_Pin, GPIO_PIN_RESET);
}

void handControl(int _handStatus)
{
	handEn = 1;
	handStatus = _handStatus;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == hand.Instance)
	{
		delayTick001++;
		if(handEn == 1)
		{
			if(handStatus == HAND_STATUS_PUT)
			{
				if(HAL_GPIO_ReadPin(switchOut_GPIO_Port, switchOut_Pin) == GPIO_PIN_RESET)
				{
					handEn = 0;
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_SET);	//tắt motor
				}
				else
				{
					HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, GPIO_PIN_RESET);	//xoay ra
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_RESET);	//bật motor
					HAL_GPIO_WritePin(gripper_GPIO_Port, gripper_Pin, GPIO_PIN_RESET);	//đóng gripper
				}
			}
			else if(handStatus == HAND_STATUS_WAIT)
			{
				if(HAL_GPIO_ReadPin(switchIn_GPIO_Port, switchIn_Pin) == GPIO_PIN_RESET)
				{
					handEn = 0;
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_SET);	//tắt motor
				}
				else
				{
					HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, GPIO_PIN_SET);	//xoay ra
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_RESET);	//bật motor
					HAL_GPIO_WritePin(gripper_GPIO_Port, gripper_Pin, GPIO_PIN_RESET);	//mở gripper
				}
			}
		}
	}
}
