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

#define HAND_DIR_OUT GPIO_PIN_RESET	//done
#define HAND_DIR_IN  GPIO_PIN_SET	//done

#define DOOR_CLOSE 	GPIO_PIN_SET	//cần thay đổi
#define DOOR_OPEN 	GPIO_PIN_RESET	//cần thay đổi

#define GRIPPER_OPEN	GPIO_PIN_SET	//cần thay đổi
#define GRIPPER_CLOSE	GPIO_PIN_RESET	//cần thay đổi

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
	  HAL_GPIO_WritePin(gripper_GPIO_Port, gripper_Pin, GRIPPER_OPEN);	//mở gripper
	  HAL_GPIO_WritePin(door_GPIO_Port, door_Pin, DOOR_CLOSE);			//đóng cửa
	  while(HAL_GPIO_ReadPin(switchIn_GPIO_Port, switchIn_Pin) == GPIO_PIN_SET)
	  {
		HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, HAND_DIR_IN);
		HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_RESET);
		trackingSwitchIn = HAL_GPIO_ReadPin(switchIn_GPIO_Port, switchIn_Pin);
			  trackingSwitchOut = HAL_GPIO_ReadPin(switchOut_GPIO_Port, switchOut_Pin);
	  }
	  HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_SET);	//tắt động cơ

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
				if(HAL_GPIO_ReadPin(switchOut_GPIO_Port, switchOut_Pin) == GPIO_PIN_RESET)	//nếu chạm limSwit ngoài
				{
					handEn = 0;
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_SET);	//tắt motor
//					HAL_GPIO_WritePin(gripper_GPIO_Port, gripper_Pin, GRIPPER_OPEN);	//mở gripper
//					HAL_GPIO_WritePin(door_GPIO_Port, door_Pin, DOOR_OPEN);				//mở cửa
				}
				else//(HAL_GPIO_ReadPin(switchOut_GPIO_Port, switchOut_Pin) == GPIO_PIN_SET)	//nếu chưa chạm limSwit ngoài
				{
					HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, GPIO_PIN_RESET);	//xoay ra
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_RESET);	//bật motor
//					HAL_GPIO_WritePin(gripper_GPIO_Port, gripper_Pin, GPIO_PIN_RESET);	//đóng gripper
//					HAL_GPIO_WritePin(door_GPIO_Port, door_Pin, DOOR_OPEN);				//mở cửa
				}
			}
			else if(handStatus == HAND_STATUS_WAIT)
			{
				if(HAL_GPIO_ReadPin(switchIn_GPIO_Port, switchIn_Pin) == GPIO_PIN_RESET)	//nếu chạm limSwit trong
				{
					handEn = 0;
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_SET);	//tắt motor
//					HAL_GPIO_WritePin(gripper_GPIO_Port, gripper_Pin, GRIPPER_OPEN);	//mở gripper
//					HAL_GPIO_WritePin(door_GPIO_Port, door_Pin, DOOR_CLOSE);			//đóng cửa
				}
				else//(HAL_GPIO_ReadPin(switchIn_GPIO_Port, switchIn_Pin) == GPIO_PIN_SET)	//nếu chưa chạm limSwit trong
				{
					HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, GPIO_PIN_SET);	//xoay vào
					HAL_GPIO_WritePin(handPul_GPIO_Port, handPul_Pin, GPIO_PIN_RESET);	//bật motor
//					HAL_GPIO_WritePin(gripper_GPIO_Port, gripper_Pin, GRIPPER_OPEN);	//mở gripper
//					HAL_GPIO_WritePin(door_GPIO_Port, door_Pin, DOOR_OPEN);				//mở cửa
				}
			}
		}
	}
}
