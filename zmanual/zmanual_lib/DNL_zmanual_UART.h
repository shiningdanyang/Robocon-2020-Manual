void peripheralUART_Init(void);

#define PS2 huart1
uint8_t PS2TxPacket[8] = "PS2Tx123";
uint8_t PS2RxPacket[8];
uint8_t PS2TxCplt;
uint8_t PS2RxCplt;
void PS2Trans(void);
void PS2Recei(void);
void wait4PS2Tx(void);
void wait4PS2Rx(void);
void PS2DeInit(void);
void PS2Init(void);
uint8_t PS2CheckbyteCount = 0, PS2Data[6], PS2DataIndex;
int16_t PS2Button;
int16_t joyLeftHor;
int16_t joyLeftVer;
int16_t joyRigtHor;
int16_t joyRigtVer;
//char* controlData;
int16_t joyLeftMidVer = 123;
int16_t joyLeftMidHor = 123;
int16_t joyRigtMidVer = 123;
int16_t joyRigtMidHor = 123;
uint8_t btn_leftLeft, btn_leftRigt, btn_leftUp, btn_leftDown;
uint8_t btn_Sta, btn_joyLeft, btn_joyRigt, btn_Sel;
uint8_t btn_A, btn_X, btn_D, btn_W, btn_E, btn_Q, btn_C, btn_Z;
uint8_t btn_E_preStatus = 0;
uint8_t btn_A_preStatus = 0;
#define compass huart4
uint8_t compassTxPacket[9] = "compassTx";
uint8_t compassRxPacket[9];
uint8_t compassTxCplt;
uint8_t compassRxCplt;
int16_t compassData;
uint8_t compassGetDataPeriod;
int trackingWait4CompassTx;
int trackingWait4CompassRx;
void compassReset(void);
void compassRequest(void);
void compassGetData(void);
void compassDecode(void);
void wait4CompassTx(void);
void wait4CompassRx(void);
void compassDeInit(void);
void compassInit(void);

#define semiAuto huart2
uint8_t semiAutoTxPacket[2] = "DA";


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == compass.Instance)
	{
		compassTxCplt = 1;
	}
}

void peripheralUART_Init(void)
{
	HAL_UART_Receive_DMA(&PS2, PS2RxPacket, 1);
	HAL_UART_Receive_DMA(&compass, compassRxPacket, 2);
}

void PS2DeInit(void)
{
	HAL_UART_DeInit(&PS2);
}

void PS2Init(void)
{
	HAL_UART_Init(&PS2);
}

void PS2Trans(void)
{
	HAL_UART_Transmit_IT(&PS2, PS2TxPacket, 8);
	wait4PS2Tx();
}
void PS2Recei(void)
{
	HAL_UART_Receive_IT(&PS2, PS2RxPacket, 1);
	wait4PS2Rx();
}

int PS2TxTracking;
void wait4PS2Tx(void)
{
	while(PS2TxCplt == 0)
	{
		PS2TxTracking++;
	}
	PS2TxCplt = 0;
}
int PS2RxTracking;
void wait4PS2Rx(void)
{
	while(PS2RxCplt == 0)
	{
		PS2RxTracking++;
	}
	PS2RxCplt = 0;
}

//put in function handles DMA stream global interrupt of brainDMA Rx in stm32f4xx_it.c
void PS2DMA_ProcessingData(void)
{
	if(PS2CheckbyteCount == 4 )
	{
	  PS2Data[PS2DataIndex++] = PS2RxPacket[0];
		if(PS2DataIndex > 5)
		{
			PS2DataIndex = 0;
			PS2CheckbyteCount = 0;
			PS2Button = (PS2Data[0]<<8) | PS2Data[1];
			joyRigtHor = PS2Data[2] - joyRigtMidHor;
			joyRigtVer = PS2Data[3] - joyRigtMidVer;
			joyLeftHor = PS2Data[4] - joyLeftMidHor;
			joyLeftVer = PS2Data[5] - joyLeftMidVer;
			btn_leftLeft = (PS2Button >> 15) & 1U;
			btn_leftDown = (PS2Button >> 14) & 1U;
			btn_leftRigt = (PS2Button >> 13) & 1U;
			btn_leftUp   = (PS2Button >> 12) & 1U;
			btn_Sta		 = (PS2Button >> 11) & 1U;
			btn_joyRigt  = (PS2Button >> 10) & 1U;
			btn_joyLeft  = (PS2Button >>  9) & 1U;
			btn_Sel  	 = (PS2Button >>  8) & 1U;
			btn_A  		 = (PS2Button >>  7) & 1U;
			btn_X  		 = (PS2Button >>  6) & 1U;
			btn_D  		 = (PS2Button >>  5) & 1U;
			btn_W  		 = (PS2Button >>  4) & 1U;
			btn_E  		 = (PS2Button >>  3) & 1U;
			btn_Q  		 = (PS2Button >>  2) & 1U;
			btn_C  		 = (PS2Button >>  1) & 1U;
			btn_Z  		 = (PS2Button >>  0) & 1U;
			if(!btn_W)		//nếu nút W được nhấn
			{
				if(HAL_GPIO_ReadPin(door_GPIO_Port, door_Pin)==GPIO_PIN_RESET)
				{
					handEn = 1;
					handStatus = HAND_STATUS_PUT;
				}
			}
			else if(!btn_X)	//nếu nút X được nhấn
			{
				handEn = 1;
				handStatus = HAND_STATUS_WAIT;
			}
			if(btn_A==0)	//nếu nút A được nhấn
			{
				if(btn_A_preStatus == 0)
				{
					semiAutoTxPacket[0] = 'D';
					HAL_UART_Transmit_IT(&semiAuto, semiAutoTxPacket, 1);
				}
				btn_A_preStatus = 1;
			}
			else
			{
//				tracking_btn_A++;
				semiAutoTxPacket[0] = 'A';
				HAL_UART_Transmit_IT(&semiAuto, semiAutoTxPacket, 1);
				btn_A_preStatus = 0;
			}
			if(!btn_E)	//nếu nút E được nhấn
			{
				if(btn_E_preStatus == 0)
				{
					HAL_GPIO_TogglePin(door_GPIO_Port, door_Pin);
					HAL_GPIO_TogglePin(gripper_GPIO_Port, gripper_Pin);
				}
				btn_E_preStatus = 1;
			}
			else
			{
				btn_E_preStatus = 0;
			}
		}
	}
	if(PS2RxPacket[0] == 0xAA)
		PS2CheckbyteCount++;
	else
		if(PS2CheckbyteCount != 4)
			PS2CheckbyteCount = 0;
}
////////////////////////////////////////////////////////////
void compassDeInit()
{
	HAL_UART_DeInit(&compass);
}

void compassInit()
{
	HAL_UART_Init(&compass);
}

void compassReset(void)
{
	compassTxPacket[0] = 'a';
	HAL_UART_Transmit_IT(&compass, compassTxPacket, 1);
	wait4CompassTx();
	compassTxPacket[0] = 'z';
}
void compassRequest(void)
{
	compassTxPacket[0]='z';
	HAL_UART_Transmit_IT(&compass, compassTxPacket, 1);
//	trackingWait4CompassTx = 0;
//	wait4CompassTx();
//	HAL_UART_Transmit(&compass, compassTxPacket, 1, 50);
}
void compassGetData(void)
{
	HAL_UART_Receive_IT(&compass, compassRxPacket, 2);
	wait4CompassRx();
//	HAL_UART_Receive(&compass, compassRxPacket, 2, 50);

	compassData = (compassRxPacket[0]<<8)|compassRxPacket[1];
}
void compassDecode(void)
{
	compassData = (compassRxPacket[0]<<8)|compassRxPacket[1];
}
void wait4CompassTx(void)
{
	while(compassTxCplt == 0)
	{
		trackingWait4CompassTx++;
	}
	compassTxCplt = 0;
}

void wait4CompassRx(void)
{
	while(compassRxCplt == 0)
	{
		trackingWait4CompassRx++;
	}
	compassRxCplt = 0;
}
////////////////////////////////////////////////////////////
