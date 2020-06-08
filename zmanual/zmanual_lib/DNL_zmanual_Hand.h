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
			if(handStatus == HAND_STATUS_READY)
			{
				HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, handForward);		//cấu hình chân handDir để tiến
				HAL_GPIO_TogglePin(handPul_GPIO_Port, handPul_Pin);					//tạo xung chân handPul
				trackingHand++;
				handElapsedPulses++;												//đếm số xung
				debug_HAND_STATUS_READY = handElapsedPulses;
				if (handElapsedPulses >= HAND_PUL_RUNUP)
				{
					handEn = 0;														//kết thúc quá trình điều khiển
					handElapsedPulses = 0;											//reset số xung
				}
			}
			if(handStatus == HAND_STATUS_PUT)
			{
				HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, handForward);		//cấu hình chân handDir để tiến
				HAL_GPIO_TogglePin(handPul_GPIO_Port, handPul_Pin);					//tạo xung chân handPul
				trackingHand++;
				handElapsedPulses++;												//đếm số xung
				debug_HAND_STATUS_PUT = handElapsedPulses;
				if (handElapsedPulses >= HAND_PUL_RUNUP2)
				{
					handEn = 0;														//kết thúc quá trình điều khiển
					handElapsedPulses = 0;											//reset số xung
				}
			}
			if(handStatus == HAND_STATUS_WAIT)
			{
				HAL_GPIO_WritePin(handDir_GPIO_Port, handDir_Pin, handBackward);	//cấu hình chân handDir để lùi
				HAL_GPIO_TogglePin(handPul_GPIO_Port, handPul_Pin);					//tạo xung chân legPul
				trackingHand++;
				handElapsedPulses++;												//đếm số xung
				debug_HAND_STATUS_WAIT = handElapsedPulses;
				if (handElapsedPulses >= HAND_PUL_END)
				{
					handEn = 0;														//kết thúc quá trình điều khiển
					handElapsedPulses = 0;											//reset số xung
				}
			}
		}
	}
}
