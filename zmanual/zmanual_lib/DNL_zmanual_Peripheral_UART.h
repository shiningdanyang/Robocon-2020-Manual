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
int16_t PS2Button, PS2JoyLeft,PS2JoyRigt;
char* controlData;

void peripheralUART_Init(void)
{
	HAL_UART_Receive_DMA(&PS2, PS2RxPacket, 1);
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
