/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtCommunication
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtCommunication.c
*********************************************************************/

#include "mtCommunication.h"
#include "mMotorOut.h"
/* 도어상태확인을 위해 인클루드 */
/*## dependency mtDecisionControl */
#include "mtDecisionControl.h"
/* 엔코더 펄스값 확인을 위해 인클루드 */
/*## dependency mtMotorFeedback */
#include "mtMotorFeedback.h"
/* 장애물감지정보, 강제열림정보를 CLI를 통해 설정하기 위해 인클루드 */
/*## dependency mtObstacleDetect */
#include "mtObstacleDetect.h"
#include "string.h"
#include "mtInputProcessing.h"
#include "stm32f7xx_hal_uart.h"
#include "cmsis_os.h"
#include "mtMonitoring.h"
#include "ring_buffer.h"

//#define MVB_PACKET_MONITORING						// 활성화 시 MVB 송/수신 패킷 모니터링 가능
//#define MVB_RX_PACKET_MONITORING						// 활성화 시 MVB 송/수신 패킷 모니터링 가능
//#define ALIVE_PACKET_MONITORING
//#define RS485_PACKET_MONITORING

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi5;
extern osMessageQId hRx1Queue;
extern osMessageQId hRx5Queue;
extern osThreadId alivecheckTaskHandle;
extern uint8_t aUartRx1Buffer[11];
extern uint8_t aUartRx3Buffer[1];
extern uint8_t aUartRx5Buffer[8];
extern uint32_t g_unDeviceID;
int8_t data_buf[10];	/* 구조체 변수 선언 */	/* 구조체 변수 선언 */
uint8_t mvbTxDataLength;
uint8_t mvbRxDataLength;
uint8_t spiTxBuffer[35]={0};
uint8_t spiRxBuffer[32]={0};
Queue rx_Int_queue;
uint8_t fault_L1_count1=0;
uint8_t fault_L1_count2=0;
uint8_t fault_L2_count1=0;
uint8_t fault_L2_count2=0;
uint8_t fault_L3_count1=0;
uint8_t fault_L3_count2=0;
uint8_t fault_L4_count1=0;
uint8_t fault_L4_count2=0;
uint8_t fault_R1_count1=0;
uint8_t fault_R1_count2=0;
uint8_t fault_R2_count1=0;
uint8_t fault_R2_count2=0;
uint8_t fault_R3_count1=0;
uint8_t fault_R3_count2=0;
uint8_t fault_R4_count1=0;
uint8_t fault_R4_count2=0;

uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_5 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_6 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_7 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_8 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_11 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_12 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_13 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_14 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_17 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_18 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_19 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_20 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_23 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_24 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_25 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_L1_26 =0;

uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_5 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_6 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_7 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_8 = 0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_11 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_12 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_13 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_14 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_17 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_18 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_19 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_20 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_23 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_24 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_25 =0;
uint8_t comp_spiTxbuffer_DCU_RS485_ERROR_R1_26 =0;
				
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_5 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_6 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_7 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_8 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_11 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_12 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_13 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_14 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_17 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_18 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_19 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_20 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_23 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_24 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_25 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_L1_26 =0;

uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_5 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_6 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_7 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_8 = 0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_11 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_12 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_13 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_14 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_17 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_18 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_19 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_20 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_23 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_24 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_25 =0;
uint8_t curr_spiTxbuffer_DCU_RS485_ERROR_R1_26 =0;

static uint16_t CalcCrc(unsigned char* pbyMsg, int IngNumBytes);
static void Comm_data_tx(uint32_t port, int8_t* rx_buf, uint16_t rx_size);
static void CommandExcute(int8_t* String);
static void isReceivePacket(void);

void MemHandling_data(uint8_t arr[],uint16_t size)
{
	/*
	 *  arr[0] : 장치 주소
	 *  arr[1] : 기능 코드 (명령어) 0x80
	 *  arr[2] : 시작 주소 (하위 바이트)
	 *  arr[3] : 시작 주소 (상위 바이트)
	 *  arr[4] : 데이터 크기(하위 바이트)
	 *  arr[5] : 데이터 크기(상위 바이트)
	 *  arr[6~데이터크기] : 데이터
	 */
	uint8_t tx_buf[8] ={0};
	uint16_t i ,q=0;
	uint16_t tx_crc =0;
	uint16_t Start_Addr = 0x00;
	uint8_t result_data =0;
	uint16_t openConfig_value =0;
	uint16_t CloseConfig_value=0;
	uint16_t ObstaclConfig_value=0;
	uint16_t temp_value =0;

	for(i=0; i<6; i++)
	{
		tx_buf[i] = arr[i];
	}
	tx_crc= CalcCrc(tx_buf,6);
	tx_buf[6] = (tx_crc & 0xff00) >> 8;
	tx_buf[7] = (tx_crc & 0x00ff);
	HAL_UART_Transmit(&huart3,tx_buf,8,10);						//프로토콜에의해 8바이트를 다시 pc 쪽으로 보낸다

	Start_Addr |= ((arr[3] & 0x00FF) << 8);
	Start_Addr |= arr[2];

	for(i=0; i<size; i++)
	{
		mram_byte_write(MRAM_DATA_ADDR+Start_Addr+i,arr[6+i]);
	}

	if(Start_Addr == 0x35)										//53번지
	{
		result_data = mram_byte_read(MRAM_DATA_ADDR+Start_Addr);
		if((result_data & 0x01) == true)						//유지보수프로그램 열림 버튼
		{
			mip_Input.CommandOpen = true;
			#ifdef DEBUG_SIMULATOR
			mip_Input.di0_OpenCloseButton = true;
			#endif
		}
		if(((result_data>>4) & 0x01) == true)					//유지보수프로그램 닫힘 버튼
		{
			mip_Input.CommandClose = true;
			#ifdef DEBUG_SIMULATOR
			mip_Input.di0_OpenCloseButton = false;
			#endif
		}

		if(((result_data>>7) & 0x01) == true)
		{
			//시험 모드 활성
			mip_Input.CommandTestMode = true;
		}
		else
		{
			//시험 모드 비활성
			mip_Input.CommandTestMode = false;
		}
	}
	/*
	 * 고장 기록 탭에서 고장 삭제 버튼 누를 경우
	 */
	else if(Start_Addr == 0x36)
	{
		result_data = mram_byte_read(MRAM_DATA_ADDR+Start_Addr);
		if(result_data & 0x01)
		{
			mram_byte_write(MRAM_FAULT_ADDR,0);
		}
	}
	/*
	 * 파라미터 탭에서 전송 버튼을 누를 경우
	 */
	else if(Start_Addr == 0x5F)
	{
		/* dcu의 메모리 시작 주소 부터 데이터 크기까지 mram에 데이터를 저장한다.*/
		for(i=Start_Addr; i<=(Start_Addr+size); i++)
		{
			/* 여섯 번쨰 배열부터 데이터가 들어오므로 mram에 데이터를 기록한다.*/
			mram_byte_write((uint32_t)(MRAM_DATA_ADDR+i),arr[6+q]);
			q++;
		}

		openConfig_value    |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x005F);					//95번지
		temp_value 		     = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0060)<<8);			//96번지
		openConfig_value    |= temp_value;
		CloseConfig_value   |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0061);					//97번지
		temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0062)<<8);		    //98번지
		CloseConfig_value   |= temp_value;
		ObstaclConfig_value |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0063);				    //99번지
		temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0064)<<8);			//100번지
		ObstaclConfig_value |= temp_value;

		mdm_time.OpenConfigtime  = (uint32_t)openConfig_value;
		mdm_time.CloseConfigtime = (uint32_t)CloseConfig_value;
		mod_Detect.ObstacleConfigValue = ObstaclConfig_value;
		/*
		 * 해당부분은 모터 feedback의 레퍼런스 속도 값을 지정하는데 추후 pid 구현이 완료되면 코드 구성을 다시해야함
		 */
		switch(mdm_time.OpenConfigtime)
		{
			case Ms_2000:
				OpenPowerValue = 0;
				break;
			case Ms_2500:
				OpenPowerValue = 1;
				break;
			case Ms_3000:
				OpenPowerValue = 2;
				break;
			case Ms_3500:
				OpenPowerValue = 3;
				break;
			case Ms_4000:
				OpenPowerValue = 4;
				break;
			default: OpenPowerValue = 1;
				break;
		}
		switch(mdm_time.CloseConfigtime)
		{
			case Ms_2500:
				ClosePowerValue = 0;
				break;
			case Ms_3000:
				ClosePowerValue = 1;
				break;
			case Ms_3500:
				ClosePowerValue = 2;
				break;
			case Ms_4000:
				ClosePowerValue = 3;
				break;
			case Ms_4500:
				ClosePowerValue = 4;
				break;
			case Ms_5000:
				ClosePowerValue = 5;
				break;
			default: ClosePowerValue = 1;
				break;
		}

	}
}

void Comm_Fault_tx(uint16_t block_num)
{
	uint8_t tx_buf[128] ={0};
	uint16_t tx_crc = 0;
	uint8_t i =0;
	if(block_num < mram_byte_read(MRAM_FAULT_ADDR))
	{
		tx_buf[0] = 0x01;
		tx_buf[1] = 0xff;
		tx_buf[2] = 0x01;
		tx_buf[3] = 0x02;				//데이터 전송 명령어 : 데이터 전송
		tx_buf[4] = block_num;
		tx_buf[5] = block_num>>8;
		tx_buf[6] = 61;					//블록안의 데이터 개수
		for(i=0; i<61; i++)
		{
			tx_buf[i+7] = mram_byte_read(MRAM_FAULT_ADDR+i+1+(block_num*0x64));
		}
		tx_crc = CalcCrc(tx_buf,68);	//61+7
		tx_buf[68] = (tx_crc & 0xff00) >> 8;
		tx_buf[69] = (tx_crc & 0x00ff);

		HAL_UART_Transmit(&huart3,tx_buf,70,100);
	}
	else
	{
		tx_buf[0] = 0x01;
		tx_buf[1] = 0xff;
		tx_buf[2] = 0x01;
		tx_buf[3] = 0x04;				//데이터 전송 명령어  :전송완료
		tx_crc = CalcCrc(tx_buf ,4);
		tx_buf[4] = (tx_crc & 0xff00) >> 8;
		tx_buf[5] = (tx_crc & 0x00ff);

		HAL_UART_Transmit(&huart3,tx_buf,6,10);
	}
}

/*
 *  유지보수 프로그램과 상태 정보들을 전송하기 위한 함수
 */
void StateData_Send(uint8_t arr[])
{
	uint8_t tx_buf[128] = {0};
	uint16_t i = 0;
	uint16_t tx_crc = 0;
	uint16_t arr_size = 0;
	uint16_t Start_Addr = 0;

	tx_buf[0] = arr[0]; //temp
	tx_buf[1] = arr[1]; //temp
	tx_buf[2] = arr[2]; //데이터 전송을 읽기 시작할 dcu의 시작 주소(하위)
	tx_buf[3] = arr[3]; //데이터 전송을 읽기 시작할 dcu의 시작 주소(상위)
	tx_buf[4] = arr[4]; //시작 주소부터 읽을 메모리 바이트 크기 (하위)
	tx_buf[5] = arr[5]; //시작 주소부터 읽을 메모리 바이트 크기 (상위)

	//arr_size = ((arr[5] << 8) | arr[4]);  //상위 하위 바이트를 논리 합으로 연산해서 사이즈를 계산한다.
	arr_size |= ((arr[5] & 0x00FF) <<8);
	arr_size |= (arr[4] & 0x00FF);
	Start_Addr |= (((uint16_t)arr[3] & 0x00FF) << 8);
	Start_Addr |= (uint16_t)arr[2];

	/*
	 * 스위치 입력등 상태 정보들을 전송
	 */
	if(Start_Addr == 0x0000u)
	{
		for(i=Start_Addr; i<arr_size; i++)
		{
			tx_buf[i+6] = mram_byte_read(MRAM_DATA_ADDR+i);
		}
		tx_crc = CalcCrc(tx_buf,(arr_size+6));							//유지보수프로그램으로 보낼 모든 바이트 crc 계산
		tx_buf[arr_size+6] =(uint8_t)((tx_crc & 0xff00) >> 8);					//프로토콜에의해 마지막 배열의 2바이트는 crc 데이터를 계산해서 더한다.
		tx_buf[arr_size+7] =(uint8_t)(tx_crc & 0x00ff);
		HAL_UART_Transmit(&huart3,tx_buf,(arr_size+8),100);				// +8은 배열 [0]~[5]의 개수와 마지막 crc 2바이트를 더한 숫자
	}
	/*
	 * 유지보수프로그램에서 불러오기 버튼을 누를 경우
	 */
	else if(Start_Addr == 0x003F)										//63번지
	{
		for(i=0; i<arr_size; i++)
		{
			tx_buf[i+6] = mram_byte_read((uint32_t)(MRAM_DATA_ADDR+0x005F+i));		//93번지부터의 설정 값들을 불러들여와서 저장한다.
		}
		tx_crc = CalcCrc(tx_buf,(arr_size+6));							//유지보수프로그램으로 보낼 모든 바이트 crc 계산
		tx_buf[arr_size+6] =(uint8_t)((tx_crc & 0xff00) >> 8);					//프로토콜에의해 마지막 배열의 2바이트는 crc 데이터를 계산해서 더한다.
		tx_buf[arr_size+7] =(uint8_t)(tx_crc & 0x00ff);
		HAL_UART_Transmit(&huart3,tx_buf,(uint8_t)(arr_size+8),100);				// +8은 배열 [0]~[5]의 개수와 마지막 crc 2바이트를 더한 숫자
	}
}

void SlaveRunning_Check(osEvent Pevent)
{
	/*
	 *  jjkim 1901011 : 마스터 dcu는 전원 on 시, 슬레이브의 동작 상태를 통신으로 받아서 로직 구동의 여부를 결정 한다.
	 *  예를들어 마스터의 퓨즈를 뽑은 상태에서 전원을 키면 슬레이브가 동작하게 된다.
	 *  이상태에서 마스터의 퓨즈를 삽입하게 되면 마스터에서 제어권을 가져오면 안되므로 (오 동작의 여지가 있음)
	 *  초기화 동작때 슬레이브의 동작 여부를 파악한 뒤 로직을 멈추게 한다.
	 */
	uint8_t CheckSumHigh=0, CheckSumLow=0;
	uint16_t CheckSumRxPacket=0;

    if(m_isMasterSlave == MASTER_DCU)
    {

        	HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);


    	Pevent = osMessageGet(hRx5Queue, 100);														// 정해진 시간 동안 Slave가 보내는 메시지 수신 대기
		if(Pevent.status == osEventMessage)															// Slave로부터 메시지 수신 성공
		{
			if(aUartRx5Buffer[0]==0xBB)
			{
				CheckSumRxPacket = aUartRx5Buffer[1]+aUartRx5Buffer[2]+aUartRx5Buffer[3]+aUartRx5Buffer[4]+aUartRx5Buffer[5];
				CheckSumHigh = (uint8_t)((CheckSumRxPacket & 0xFF00)>>8);
				CheckSumLow = (uint8_t)(CheckSumRxPacket & 0x00FF);

				/*
				 * Slave가 송신한 8byte 패킷 정상적으로 수신
				 */
				if((CheckSumHigh==aUartRx5Buffer[6]) && (CheckSumLow==aUartRx5Buffer[7]))
				{
					if(aUartRx5Buffer[3]==true)														// Slave가 절체했다고 알려주면
					{
						m_isSlaveRunCommand = true;
						mip_MasterCodeMasterLED(0);													// Master는 실행을 중지 함
						mip_SleveCodeSlaveRun(1);													// 릴레이 fnd등 모든 제어권을 슬레이브로 넘긴다.
						debug("초기화 동작중 슬레이브 동작 확인 \r\n");
					}
				}
				else
				{
					printf("오수신 \r\n");
				}
			}
		}
		else
		{
			debug("초기화 동작중 슬레이브 미동작 확인 \r\n");
		}
    }
}
#if 0
void mvbSaveDataDCUL(uint8_t dcu_id)								// Big-Endian (Motorola) Byte Order
{
	volatile static uint16_t WatchdogCnt=0;

	WatchdogCnt++;
	spiTxBuffer[3] = (uint8_t)((WatchdogCnt&0xFF00)>>8);
	spiTxBuffer[4] = (uint8_t)(WatchdogCnt&0x00FF);
	if((dcu_id == DCU_L2_MCAR) || (dcu_id == DCU_L2_TCAR))
	{
		spiTxBuffer[11] = aUartRx1Buffer[3];						// 상태 2
		spiTxBuffer[12] = aUartRx1Buffer[2];						// 상태 1
		spiTxBuffer[13] = aUartRx1Buffer[5];						// 고장 2
		spiTxBuffer[14] = aUartRx1Buffer[4];						// 고장 1
		spiTxBuffer[15] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[16] = aUartRx1Buffer[6];						// 롬 버전
		spiTxBuffer[30] = aUartRx1Buffer[8];						// 닫힘 시간
	}
	else if((dcu_id == DCU_L3_MCAR) || (dcu_id == DCU_L3_TCAR))
	{
		spiTxBuffer[17] = aUartRx1Buffer[3];						// 상태 2
		spiTxBuffer[18] = aUartRx1Buffer[2];						// 상태 1
		spiTxBuffer[19] = aUartRx1Buffer[5];						// 고장 2
		spiTxBuffer[20] = aUartRx1Buffer[4];						// 고장 1
		spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[22] = aUartRx1Buffer[6];						// 롬 버전
		spiTxBuffer[31] = aUartRx1Buffer[8];						// 닫힘 시간
	}
	else if((dcu_id == DCU_L1_MCAR) || (dcu_id == DCU_L1_TCAR))
	{
		spiTxBuffer[23] = aUartRx1Buffer[3];						// 상태 2
		spiTxBuffer[24] = aUartRx1Buffer[2];						// 상태 1
		spiTxBuffer[25] = aUartRx1Buffer[5];						// 고장 2
		spiTxBuffer[26] = aUartRx1Buffer[4];						// 고장 1
		spiTxBuffer[27] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[28] = aUartRx1Buffer[6];						// 롬 버전
		spiTxBuffer[32] = aUartRx1Buffer[8];						// 닫힘 시간
	}
	else //	DCU_L4_MCAR || DCU_L4_TCAR
	{
		spiTxBuffer[5] = Statement_DCU(3);							// 상태 2
		spiTxBuffer[6] = Statement_DCU(2);							// 상태 1
		spiTxBuffer[7] = FalutStatement_DCU(5);						// 고장 2
		spiTxBuffer[8] = FalutStatement_DCU(4);						// 고장 1
		spiTxBuffer[9] = m_isFaultCount;										// TDNO
		spiTxBuffer[10] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// 롬 버전
		spiTxBuffer[29] = (uint8_t)(mdm_time.Closing / 100);										// 닫힘 시간
	}
	spiTxBuffer[33] = 0x00;
	spiTxBuffer[34] = 0x00;
}

void mvbSaveDataDCUR(uint8_t dcu_id)								// Big-Endian (Motorola) Byte Order
{
	volatile static uint16_t WatchdogCnt=0;

	WatchdogCnt++;
	spiTxBuffer[3] = (uint8_t)((WatchdogCnt&0xFF00)>>8);
	spiTxBuffer[4] = (uint8_t)(WatchdogCnt&0x00FF);
	if((dcu_id == DCU_R2_MCAR) || (dcu_id == DCU_R2_TCAR))
	{
		spiTxBuffer[11] = aUartRx1Buffer[3];						// 상태 2
		spiTxBuffer[12] = aUartRx1Buffer[2];						// 상태 1
		spiTxBuffer[13] = aUartRx1Buffer[5];						// 고장 2
		spiTxBuffer[14] = aUartRx1Buffer[4];						// 고장 1
		spiTxBuffer[15] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[16] = aUartRx1Buffer[6];						// 롬 버전
		spiTxBuffer[30] = aUartRx1Buffer[8];						// DCU2 닫힘 시간
	}
	else if((dcu_id == DCU_R3_MCAR) || (dcu_id == DCU_R3_TCAR))
	{
		spiTxBuffer[17] = aUartRx1Buffer[3];						// 상태 2
		spiTxBuffer[18] = aUartRx1Buffer[2];						// 상태 1
		spiTxBuffer[19] = aUartRx1Buffer[5];						// 고장 2
		spiTxBuffer[20] = aUartRx1Buffer[4];						// 고장 1
		spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[22] = aUartRx1Buffer[6];						// 롬 버전
		spiTxBuffer[31] = aUartRx1Buffer[8];						// DCU3 닫힘 시간
	}
	else if((dcu_id == DCU_R4_MCAR) || (dcu_id == DCU_R4_TCAR))
	{
		spiTxBuffer[23] = aUartRx1Buffer[3];						// 상태 2
		spiTxBuffer[24] = aUartRx1Buffer[2];						// 상태 1
		spiTxBuffer[25] = aUartRx1Buffer[5];						// 고장 2
		spiTxBuffer[26] = aUartRx1Buffer[4];						// 고장 1
		spiTxBuffer[27] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[28] = aUartRx1Buffer[6];						// 롬 버전
		spiTxBuffer[32] = aUartRx1Buffer[8];						// DCU4 닫힘 시간
	}
	else //	DCU_R1_MCAR || DCU_R1_TCAR
	{
		spiTxBuffer[5] = Statement_DCU(3);							// 상태 2
		spiTxBuffer[6] = Statement_DCU(2);							// 상태 1
		spiTxBuffer[7] = FalutStatement_DCU(5);						// 고장 2
		spiTxBuffer[8] = FalutStatement_DCU(4);						// 고장 1
		spiTxBuffer[9] = m_isFaultCount;										// TDNO
		spiTxBuffer[10] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// 롬 버전
		spiTxBuffer[29] = (uint8_t)(mdm_time.Closing / 100);		// DCU1 닫힘 시간
	}
	spiTxBuffer[33] = 0x00;
	spiTxBuffer[34] = 0x00;
}
#else


uint8_t DCUL_COMM_FAULT_FLAF=0;
uint8_t DCUR_COMM_FAULT_FLAF=0;
void mvbSaveDataDCUL(uint8_t dcu_id)								// Big-Endian (Motorola) Byte Order
{
	volatile static uint16_t WatchdogCnt=0;
	
	WatchdogCnt++;
	if(WatchdogCnt > 0xfff0) WatchdogCnt = 0;
	
	spiTxBuffer[3] = (uint8_t)((WatchdogCnt&0xFF00)>>8);
	spiTxBuffer[4] = (uint8_t)(WatchdogCnt&0x00FF);
	if((dcu_id == DCU_L1_MCAR) || (dcu_id == DCU_L1_TCAR))
	{
		if(error_485_flag[DCU_RS485_ERROR_L1] > 10)
		{
			spiTxBuffer[5] = 0x00;						// 상태 2
			spiTxBuffer[6] = 0x00;						// 상태 1
			spiTxBuffer[7] = 0x00;						// 고장 2
			spiTxBuffer[8] = 0x80; //jeon_20190423
			spiTxBuffer[9] = 0x00;						// TDNO
			spiTxBuffer[10] = 0x00;						// 롬 버전
			spiTxBuffer[30] = 0x00;						// 닫힘 시간
		}
		else
		{
			if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_7 == aUartRx1Buffer[5]) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_7 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_7))
			{
				fault_L1_count1++;
			}
			else
			{
				fault_L1_count1=0;
			}
			
			if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_8 == aUartRx1Buffer[4]) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_8 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_8))
			{
				fault_L1_count2++;
			}	
			else
			{
				fault_L1_count2=0;
			}
				
			if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_5 == aUartRx1Buffer[3])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_5 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_5;
			}	
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_6 == aUartRx1Buffer[2])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_6 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_6;
			}	
			
			if(fault_L1_count1 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_7 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_7;
			}
			
			if(fault_L1_count2 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_8 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_8;
			}
			
			spiTxBuffer[5] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_5;
			spiTxBuffer[6] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_6;	
			spiTxBuffer[7] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_7;
			spiTxBuffer[8] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_8;
			spiTxBuffer[9] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[10] = aUartRx1Buffer[6];						// 롬 버전
			spiTxBuffer[30] = aUartRx1Buffer[8];						// 닫힘 시간
		}
		
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_5 = aUartRx1Buffer[3];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_6 = aUartRx1Buffer[2];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_7 = aUartRx1Buffer[5];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_8 = aUartRx1Buffer[4];
	}
	else if((dcu_id == DCU_L2_MCAR) || (dcu_id == DCU_L2_TCAR))
	{
		if(error_485_flag[DCU_RS485_ERROR_L2] > 10)
		{
			spiTxBuffer[11] = 0x00;						// 상태 2
			spiTxBuffer[12] = 0x00;						// 상태 1
			spiTxBuffer[13] = 0x00;						// 고장 2
			spiTxBuffer[14] = 0x80; 					// 고장 1
			spiTxBuffer[15] = 0x00;						// TDNO
			spiTxBuffer[16] = 0x00;						// 롬 버전
			spiTxBuffer[29] = 0x00;						// 닫힘 시간
		}
		else
		{
			if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_13 == aUartRx1Buffer[5]) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_13 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_13))
			{
				fault_L2_count1++;
			}
			else
			{
				fault_L2_count1=0;
			}

			if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_14 == aUartRx1Buffer[4]) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_14 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_14))
			{
				fault_L2_count2++;
			}
			else
			{
				fault_L2_count2=0;
			}

			if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_11 == aUartRx1Buffer[3])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_11 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_11;
			}	
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_12 == aUartRx1Buffer[2])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_12 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_12;
			}
			
			if(fault_L2_count1 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_13 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_13;
			}

			if(fault_L2_count2 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_14 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_14;
			}
			
			spiTxBuffer[11] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_11;					// 상태 2
			spiTxBuffer[12] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_12;						// 상태 1
			spiTxBuffer[13] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_13;						// 고장 2
			spiTxBuffer[14] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_14;						// 고장 1
			spiTxBuffer[15] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[16] = aUartRx1Buffer[6];						// 롬 버전
			spiTxBuffer[29] = aUartRx1Buffer[8];						// 닫힘 시간
									
		}
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_11 = aUartRx1Buffer[3];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_12 = aUartRx1Buffer[2];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_13 = aUartRx1Buffer[5];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_14 = aUartRx1Buffer[4]; //jeon_190712 orig:0x3f
	}
	else if((dcu_id == DCU_L3_MCAR) || (dcu_id == DCU_L3_TCAR))
	{
		if(error_485_flag[DCU_RS485_ERROR_L3] > 10)
		{
			spiTxBuffer[17] = 0x00;						// 상태 2
			spiTxBuffer[18] = 0x00;						// 상태 1
			spiTxBuffer[19] = 0x00;						// 고장 2
			spiTxBuffer[20] = 0x80; //jeon_20190423
			spiTxBuffer[21] = 0x00;						// TDNO
			spiTxBuffer[22] = 0x00;						// 롬 버전
			spiTxBuffer[32] = 0x00;						// 닫힘 시간
		}
		else
		{
			if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_19 == aUartRx1Buffer[5]) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_19 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_19))
			{
				fault_L3_count1++;
			}
			else
			{
				fault_L3_count1=0;
			}

			if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_20 == aUartRx1Buffer[4]) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_20 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_20))
			{
				fault_L3_count2++;
			}
			else
			{
				fault_L3_count2=0;
			}

			if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_17 == aUartRx1Buffer[3])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_17 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_17;
			}	
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_18 == aUartRx1Buffer[2])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_18 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_18;
			}	
			
			if(fault_L3_count1 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_19 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_19;
			}

			if(fault_L3_count2 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_L1_20 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_20;
			}
			
			spiTxBuffer[17] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_17;					// 상태 2
			spiTxBuffer[18] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_18;						// 상태 1
			spiTxBuffer[19] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_19;						// 고장 2
			spiTxBuffer[20] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_20;						// 고장 2
			spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[22] = aUartRx1Buffer[6];						// 롬 버전
			spiTxBuffer[32] = aUartRx1Buffer[8];						// 닫힘 시간
		}
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_17 = aUartRx1Buffer[3];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_18 = aUartRx1Buffer[2];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_19 = aUartRx1Buffer[5];
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_20 = aUartRx1Buffer[4]; //jeon_190712 orig:0x3f		
	}
	else //	DCU_L4_MCAR || DCU_L4_TCAR
	{
		if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_25 == FalutStatement_DCU(5)) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_25 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_25))
		{
			fault_L4_count1++;
		}
		else
		{
			fault_L4_count1=0;
		}

		if((comp_spiTxbuffer_DCU_RS485_ERROR_L1_26 == FalutStatement_DCU(4)) && (curr_spiTxbuffer_DCU_RS485_ERROR_L1_26 != comp_spiTxbuffer_DCU_RS485_ERROR_L1_26))
		{
			fault_L4_count2++;
		}
		else
		{
			fault_L4_count2=0;
		}

		if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_23 == Statement_DCU(3))
		{	
			curr_spiTxbuffer_DCU_RS485_ERROR_L1_23 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_23;
		}	
		
		if(comp_spiTxbuffer_DCU_RS485_ERROR_L1_24 == Statement_DCU(2))
		{	
			curr_spiTxbuffer_DCU_RS485_ERROR_L1_24 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_24;
		}	
		
		if(fault_L4_count1 > 5)
		{
			curr_spiTxbuffer_DCU_RS485_ERROR_L1_25 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_25;
		}
	
		if(fault_L4_count2 > 5)
		{
			curr_spiTxbuffer_DCU_RS485_ERROR_L1_26 = comp_spiTxbuffer_DCU_RS485_ERROR_L1_26;
		}
	
		spiTxBuffer[23] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_23;					// 상태 2
		spiTxBuffer[24] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_24;						// 상태 1
		spiTxBuffer[25] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_25;						// 고장 2
		spiTxBuffer[26] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_26;						// 고장 2
		spiTxBuffer[27] = m_isFaultCount;										// TDNO
		spiTxBuffer[28] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// 롬 버전
		spiTxBuffer[31] = (uint8_t)(mdm_time.Closing / 100);										// 닫힘 시간
		
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_23 = Statement_DCU(3);
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_24 = Statement_DCU(2);
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_25 = FalutStatement_DCU(5);
		comp_spiTxbuffer_DCU_RS485_ERROR_L1_26 = FalutStatement_DCU(4); //jeon_190712 orig:0x3f				
	}
	
	spiTxBuffer[33] = 0x00;
	spiTxBuffer[34] = 0x00;

}


void mvbSaveDataDCUR(uint8_t dcu_id)								// Big-Endian (Motorola) Byte Order
{
	volatile static uint16_t WatchdogCnt=0;

	WatchdogCnt++;
	if(WatchdogCnt > 0xfff0) WatchdogCnt = 0;  
	spiTxBuffer[3] = (uint8_t)((WatchdogCnt&0xFF00)>>8);
	spiTxBuffer[4] = (uint8_t	)(WatchdogCnt&0x00FF);
	if((dcu_id == DCU_R1_MCAR) || (dcu_id == DCU_R1_TCAR))
	{
		if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_7 == FalutStatement_DCU(5)) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_7 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_7))
		{
			fault_R1_count1++;
		}
		else
		{
			fault_R1_count1=0;
		}
		
		if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_8 == FalutStatement_DCU(4)) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_8 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_8))
		{
			fault_R1_count2++;
		}	
		else
		{
			fault_R1_count2=0;
		}

		if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_5 == Statement_DCU(3))
		{	
			curr_spiTxbuffer_DCU_RS485_ERROR_R1_5 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_5;
		}	
		
		if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_6 == Statement_DCU(2))
		{	
			curr_spiTxbuffer_DCU_RS485_ERROR_R1_6 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_6;
		}	
		
		if(fault_R1_count1 > 5)
		{
			curr_spiTxbuffer_DCU_RS485_ERROR_R1_7 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_7;
		}
	
		if(fault_R1_count2 > 5)
		{
			curr_spiTxbuffer_DCU_RS485_ERROR_R1_8 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_8;
		}
				
		spiTxBuffer[5] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_5;					// 상태 2
		spiTxBuffer[6] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_6;						// 상태 1
		spiTxBuffer[7] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_7;						// 고장 2
		spiTxBuffer[8] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_8;						// 고장 2
		spiTxBuffer[9] = m_isFaultCount;										// TDNO
		spiTxBuffer[10] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// 롬 버전
		spiTxBuffer[30] = (uint8_t)(mdm_time.Closing / 100);										// 닫힘 시간
		
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_5 = Statement_DCU(3);
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_6 = Statement_DCU(2);
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_7 = FalutStatement_DCU(5);
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_8 = FalutStatement_DCU(4); //jeon_190712 orig:0x3f		
	}
	else if((dcu_id == DCU_R2_MCAR) || (dcu_id == DCU_R2_TCAR))
	{
		if(error_485_flag[DCU_RS485_ERROR_R2] > 10)
		{
			spiTxBuffer[11] = 0x00;						// 8 상태 2
			spiTxBuffer[12] = 0x00;						// 9 상태 1
			spiTxBuffer[13] = 0x00;						// 10 고장 2
			spiTxBuffer[14] = 0x80; //jeon_20190423
			spiTxBuffer[15] = 0x00;						// 12 TDNO
			spiTxBuffer[16] = 0x00;						// 13 롬 버전
			spiTxBuffer[29] = 0x00;						// 닫힘 시간
		}
		else
		{
			if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_13 == aUartRx1Buffer[5]) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_13 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_13))
			{
				fault_R2_count1++;
			}
			else
			{
				fault_R2_count1=0;
			}
			
			if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_14 == aUartRx1Buffer[4]) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_14 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_14))
			{
				fault_R2_count2++;
			}	
			else
			{
				fault_R2_count2=0;
			}
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_11 == aUartRx1Buffer[3])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_11 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_11;
			}	
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_12 == aUartRx1Buffer[2])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_12 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_12;
			}	
			
			if(fault_R2_count1 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_13 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_13;
			}
	
			if(fault_R2_count2 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_14 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_14;
			}
		
			spiTxBuffer[11] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_11;					// 상태 2
			spiTxBuffer[12] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_12;						// 상태 1
			spiTxBuffer[13] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_13;						// 고장 2
			spiTxBuffer[14] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_14;						// 고장 2
			spiTxBuffer[15] = aUartRx1Buffer[7];						// 12 TDNO
			spiTxBuffer[16] = aUartRx1Buffer[6];						// 13 롬 버전
			spiTxBuffer[29] = aUartRx1Buffer[8];						// 닫힘 시간
		}
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_11 = aUartRx1Buffer[3];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_12 = aUartRx1Buffer[2];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_13 = aUartRx1Buffer[5];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_14 = aUartRx1Buffer[4]; //jeon_190712 orig:0x3f

	}
	else if((dcu_id == DCU_R3_MCAR) || (dcu_id == DCU_R3_TCAR))
	{
		if(error_485_flag[DCU_RS485_ERROR_R3] > 10)
		{
			spiTxBuffer[17] = 0x00;						// 상태 2
			spiTxBuffer[18] = 0x00;						// 상태 1
			spiTxBuffer[19] = 0x00;						// 고장 2
			spiTxBuffer[20] = 0x80; //jeon_20190423
			spiTxBuffer[21] = 0x00;						// TDNO
			spiTxBuffer[22] = 0x00;						// 롬 버전
			spiTxBuffer[32] = 0x00;						// 닫힘 시간
		}
		else
		{
			if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_19 == aUartRx1Buffer[5]) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_19 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_19))
			{
				fault_R3_count1++;
			}
			else
			{
				fault_R3_count1=0;
			}
			
			if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_20 == aUartRx1Buffer[4]) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_20 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_20))
			{
				fault_R3_count2++;
			}	
			else
			{
				fault_R3_count2=0;
			}
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_17 == aUartRx1Buffer[3])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_17 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_17;
			}	
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_18 == aUartRx1Buffer[2])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_18 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_18;
			}	
			
			if(fault_R3_count1 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_19 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_19;
			}
	
			if(fault_R3_count2 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_20 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_20;
			}
		
			spiTxBuffer[17] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_17;					// 상태 2
			spiTxBuffer[18] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_18;						// 상태 1
			spiTxBuffer[19] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_19;						// 고장 2
			spiTxBuffer[20] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_20;						// 고장 2
			spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[22] = aUartRx1Buffer[6];						// 롬 버전
			spiTxBuffer[32] = aUartRx1Buffer[8];						// 닫힘 시간
		}
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_17 = aUartRx1Buffer[3];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_18 = aUartRx1Buffer[2];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_19 = aUartRx1Buffer[5];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_20 = aUartRx1Buffer[4]; //jeon_190712 orig:0x3f
	}
	else //	DCU_R4_MCAR || DCU_R4_TCAR
	{
		if(error_485_flag[DCU_RS485_ERROR_R4] > 10)
		{
			spiTxBuffer[23] = 0x00;						// 상태 2
			spiTxBuffer[24] = 0x00;						// 상태 1
			spiTxBuffer[25] = 0x00;						// 고장 2
			spiTxBuffer[26] = 0x80; //jeon_20190423
			spiTxBuffer[27] = 0x00;						// TDNO
			spiTxBuffer[28] = 0x00;						// 롬 버전
			spiTxBuffer[31] = 0x00;						// 닫힘 시간
		}
		else
		{
			if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_25 == aUartRx1Buffer[5]) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_25 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_25))
			{
				fault_R4_count1++;
			}
			else
			{
				fault_R4_count1=0;
			}
			
			if((comp_spiTxbuffer_DCU_RS485_ERROR_R1_26 == aUartRx1Buffer[4]) && (curr_spiTxbuffer_DCU_RS485_ERROR_R1_26 != comp_spiTxbuffer_DCU_RS485_ERROR_R1_26))
			{
				fault_R4_count2++;
			}	
			else
			{
				fault_R4_count2=0;
			}

			if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_23 == aUartRx1Buffer[3])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_23 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_23;
			}	
			
			if(comp_spiTxbuffer_DCU_RS485_ERROR_R1_24 == aUartRx1Buffer[2])
			{	
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_24 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_24;
			}	
			
			if(fault_R4_count1 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_25 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_25;
			}
	
			if(fault_R4_count2 > 5)
			{
				curr_spiTxbuffer_DCU_RS485_ERROR_R1_26 = comp_spiTxbuffer_DCU_RS485_ERROR_R1_26;
			}
			
			spiTxBuffer[23] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_23;					// 상태 2
			spiTxBuffer[24] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_24;						// 상태 1
			spiTxBuffer[25] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_25;						// 고장 2
			spiTxBuffer[26] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_26;						// 고장 2
			spiTxBuffer[27] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[28] = aUartRx1Buffer[6];						// 롬 버전
			spiTxBuffer[31] = aUartRx1Buffer[8];						// 닫힘 시간
		}
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_23 = aUartRx1Buffer[3];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_24 = aUartRx1Buffer[2];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_25 = aUartRx1Buffer[5];
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_26 = aUartRx1Buffer[4]; //jeon_190712 orig:0x3f
	}
	spiTxBuffer[33] = 0x00;
	spiTxBuffer[34] = 0x00;
}
#endif

uint8_t FalutStatement_DCU(uint8_t byte_address)
{
	/*============================================================================ F16    : RS485 고장
	 *  							고장 정보 표시									   F15    : MVB 고장
	 *============================================================================ F14    : SAFETY LOOP 고장
	 *    |  bit7  |  bit6 |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  | F13    : 엔코더 고장
	 *============================================================================ F12    : 이중계 절체 불가
	 *[4] |  F16   |  F15  |  F14   |  F13   |  F12   |  F11   |  F10   |  F09   | F11    : 이중계 동작 중
	 *============================================================================ F10    : 열림 실패
	 *[5] |  F08   |  F07  |  F06   |  F05   |  F04   |  F03   |  F02   |   F01  | F09    : 장애물 감지
	 *============================================================================ F08    : DLS 2 고장
	 *																			   F07    : 예기치 못한 잠김 풀림
	 *																			   F06    : DCS2 고장
	 *																			   F05    : DCS1 고장
	 *																			   F04    : DLS1 고장
	 *																			   F03    : MOTOR 고장
	 *																			   F02    : 경고장
	 *																			   F01    : 중고장
	 */
	uint8_t return_statement = 0;
	if(byte_address == 4)
	{
		if(error_list_flag[DCU_RS485_ERROR] == true) 			return_statement |= 0x80;
		if(error_list_flag[DCU_MVB_ERROR] == true) 				return_statement |= 0x40;
		if(error_list_flag[DCU_SAFETYLOOP_FAULT] == true) 		return_statement |= 0x20;
		if(error_list_flag[DCU_ENCODER_ERROR] == true)			return_statement |= 0x10;
		if(error_list_flag[DCU_CANT_SLAVE] == true)				return_statement |= 0x08;
		if(error_list_flag[DCU_SLAVE_RUN] == true)				return_statement |= 0x04;
		if(error_list_flag[DCU_OPEN_FAULT] == true)				return_statement |= 0x02;
		if(error_list_flag[DCU_OBSTACLE] == true)				return_statement |= 0x01;
	}
	else if(byte_address ==5)
	{
		if(error_list_flag[DCU_DLS2_FAULT] == true)				return_statement |= 0x80;
		if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true)		return_statement |= 0x40;
		if(error_list_flag[DCU_DCS2_FAULT] == true)				return_statement |= 0x20;
		if(error_list_flag[DCU_DCS1_FAULT] == true)				return_statement |= 0x10;
		if(error_list_flag[DCU_DLS1_FAULT] == true)				return_statement |= 0x08;
		if(error_list_flag[DCU_MOTOR_ERROR] == true)			return_statement |= 0x04;
		if(error_list_flag[DCU_MINOR_FAULT] == true)			return_statement |= 0x02;
		if(error_list_flag[DCU_HARD_FAULT] == true)				return_statement |= 0x01;
	}
	return return_statement;
}

uint8_t Statement_DCU(uint8_t byte_address)
{
	/*
	 * 								상태 정보 표시
	 *============================================================================	REOPEN : REOPEN 버튼 상태
	 *    |  bit7  |  bit6 |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0	 |	CLOSEPB: CLOSE 버튼 상태
	 *============================================================================	OPENPB : OPEN 버튼 상태
	 *[2] | Spare | Spare  | REOPEN | CLOSEPB| OPENPB |  EAD   |  EED   |  OBS   |	EAD    : 외부 비상 핸들
	 *============================================================================	EED    : 내부 비상 핸들
	 *[3] |  FP   |  DFO   |  DI    |  DNC   | DCU OK |   X    |   X    |   X    |	OBS    : 장애물 감지
	 *============================================================================	FP     : DCU가 잠기지 않은 상태
	 *[4] |							고장 정보(1)										DFO    : DCU가 완전 열린 상태
	 *============================================================================	DI     : Isolation 상태
	 *[5] |                         고장 정보(2)										DNC    : 닫히지 않은 상태
	 *============================================================================	DCU OK : DCU 작동 중 ??
	 */
	uint8_t return_statement =0;

	if(byte_address == 2)
	{
		if(mod_Detect.ObstacleDetectCnt > 1)	  return_statement |= 0x01;		// S09_ODS //jeon_190822 ObstacleDetectCnt 0->1
		if(mip_Input.di1_EED == true)			  return_statement |= 0x02;		// S10_EED
		if(mip_Input.di1_EAD == true)			  return_statement |= 0x04;		// S11_EAD
		if(mip_Input.di0_OpenCloseButton == true) return_statement |= 0x08;		// S12_OPENPB
		else /* di0_OpenCloseButton == false */	  return_statement |= 0x10;		// S13_CLOSEPB
		if(mip_Input.di0_ReOpen == true) 		  return_statement |= 0x20;		// S14_REOPENPB
	}
	else if(byte_address == 3)
	{
		//if((mdc_DoorState == DOOR_ERROR) && (mdc_PreDoorState == DOOR_CLOSED)) return_statement |= 0x00;
		//if(mdc_DoorState != DOOR_CLOSED)	      return_statement |= 0x10;		// S05_DNC, 닫히지 않은 상태
		if((mip_Input.di0_DLS1 == false) && (mip_Input.di1_DLS2 == false))	return_statement |= 0x10;		// S05_DNC, 닫히지 않은 상태
		if(mip_Input.di0_Isolation == true)       return_statement |= 0x20;		// S06_DI, 차단 스위치 상태
		if(mdc_DoorState == DOOR_OPENED)		  return_statement |= 0x40;		// S07_DFO, 완전 열림 상태
		if(mip_Input.DoorClosedSwitchOn == false) return_statement |= 0x80;		// S08_FP, dcu가 잠기지 않은 상태
	}

	return return_statement;
}

void callback_test_state(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pStringValue = NULL;
	char *pEnd = NULL;
	uint32_t pValue = 0;
	
	pStringValue = strtok(string," ");
	pValue = strtol(pStringValue,&pEnd,10);
	debug("Call command : test state (input:%d)\r\n", pValue);
	
    switch(pValue)
    {
    	case S09_ODS:		debug("## Set S09\r\n"); mod_Detect.ObstacleDetectCnt ^= 1; break;						// [2] 0x01
    	case S10_EED:		debug("## Set S10\r\n"); mip_Input.di1_EED ^= 1; break;								// [2] 0x02
    	case S11_EAD:		debug("## Set S11\r\n"); mip_Input.di1_EAD ^= 1; break;								// [2] 0x04
    	case S12_OPENPB:	debug("## Set S12\r\n"); mip_Input.di0_OpenCloseButton ^= 1; break;					// [2] 0x08
    	case S13_CLOSEPB:	debug("## Set S13\r\n"); mip_Input.di0_OpenCloseButton ^= 1; break;					// [2] 0x10
    	case S14_REOPENPB:	debug("## Set S14\r\n"); mip_Input.di0_ReOpen ^= 1; break;								// [2] 0x20
    	case S15_SPARE:		debug("## Set S15\r\n"); break;														// [2]
    	case S16_SPARE:		debug("## Set S16\r\n"); break;														// [2]
    	
    	case S01_SPARE:		debug("## Set S01\r\n"); break;														// [3]
    	case S02_SPARE:		debug("## Set S02\r\n"); break;														// [3]
    	case S03_SPARE:		debug("## Set S03\r\n"); break;														// [3]
    	case S04_DCU_OK:	debug("## Set S04\r\n"); break;														// [3] 0x08
    	case S05_DNC:		debug("## Set S05, 닫히지 않은 상태\r\n");	mdc_DoorState = DOOR_OPENING; break;			// [3] 0x10
    	case S06_DI:		debug("## Set S06, 차단 스위치 상태\r\n");	mip_Input.di0_Isolation ^= 1; break;			// [3] 0x20
    	case S07_DFO:		debug("## Set S07, 완전 열림 상태\r\n"); mdc_DoorState = DOOR_OPENED; break;				// [3] 0x40
    	case S08_FP:		debug("## Set S08, dcu가 잠기지 않은 상태\r\n"); mip_Input.DoorClosedSwitchOn ^= 1; break;	// [3] 0x80
    	
    	default: debug("# CLI Undefined Command\r\n"); break;
    }
    if((pValue>0) && (pValue<18))		error_list_flag[DCU_ERROR]=true;
}

void callback_test_fault(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pStringValue = NULL;
	char *pEnd = NULL;
	uint32_t pValue = 0;
	
	pStringValue = strtok(string," ");
	pValue = strtol(pStringValue,&pEnd,10);
	debug("Call command : test fault (input:%d)\r\n", pValue);
	
    switch(pValue)
    {
    	case DCU_HARD_FAULT:		debug("## Set F01 중고장\r\n");				error_list_flag[DCU_HARD_FAULT] ^= 1; break;
    	case DCU_MINOR_FAULT:		debug("## Set F02 경고장\r\n");				error_list_flag[DCU_MINOR_FAULT] ^= 1; break;
    	case DCU_MOTOR_ERROR:		debug("## Set F03 모터 회로 고장\r\n");		error_list_flag[DCU_MOTOR_ERROR] ^= 1; break;
    	case DCU_DLS1_FAULT:		debug("## Set F04 DLS1고장\r\n");			error_list_flag[DCU_DLS1_FAULT] ^= 1; break;
    	case DCU_DCS1_FAULT:		debug("## Set F05 DCS1고장\r\n");			error_list_flag[DCU_DCS1_FAULT] ^= 1; break;
    	case DCU_DCS2_FAULT:		debug("## Set F06 DCS2고장\r\n");			error_list_flag[DCU_DCS2_FAULT] ^= 1; break;
    	case DCU_UNEXPECTED_UNLOCK:	debug("## Set F07 예기치 못한 잠김 풀림\r\n");	error_list_flag[DCU_UNEXPECTED_UNLOCK] ^= 1; break;
    	case DCU_DLS2_FAULT:		debug("## Set F08 DLS2고장\r\n");			error_list_flag[DCU_DLS2_FAULT] ^= 1; break;
    	case DCU_OBSTACLE:			debug("## Set F09 닫힘중 장애감지 고장\r\n");	error_list_flag[DCU_OBSTACLE] ^= 1; break;
    	case DCU_OPEN_FAULT:		debug("## Set F10 도어 열림 실패 고장\r\n");	error_list_flag[DCU_OPEN_FAULT] ^= 1; break;
    	case DCU_SLAVE_RUN:			debug("## Set F11 이중계 동작 중\r\n");		error_list_flag[DCU_SLAVE_RUN] ^= 1; break;
    	case DCU_CANT_SLAVE:		debug("## Set F12 이중계 전환 불가\r\n");		error_list_flag[DCU_CANT_SLAVE] ^= 1; break;
    	case DCU_ENCODER_ERROR:		debug("## Set F13 엔코더 고장\r\n");			error_list_flag[DCU_ENCODER_ERROR] ^= 1; break;
    	case DCU_SAFETYLOOP_FAULT:	debug("## Set F14 Safety Loop 이상\r\n");	error_list_flag[DCU_SAFETYLOOP_FAULT] ^= 1; break;
    	case DCU_MVB_ERROR:			debug("## Set F15 MVB통신 고장\r\n");			error_list_flag[DCU_MVB_ERROR] ^= 1; break;
    	case DCU_RS485_ERROR:		debug("## Set F16 RS-485통신 고장\r\n");		error_list_flag[DCU_RS485_ERROR] ^= 1; break;
    	case DCU_LOCK_FAULT:		debug("## Set F17 잠김 실패 -> 없음\r\n");		error_list_flag[DCU_LOCK_FAULT] ^= 1; break;
    	default: debug("# CLI Undefined Command\r\n"); break;
    }
    if((pValue>0) && (pValue<18))		error_list_flag[DCU_ERROR]=true;
}

void callback_test_door_free(int8_t* string, uint8_t ucNumOfParameters)
{
    static int32_t PhsuCnt=0;
    PhsuCnt++;
    if((PhsuCnt%2)==1)
    {
    	debug("Call command : test door free\r\n");
        mip_Input.di0_Isolation = true;
    }
    else
    {
    	debug("Call command : test door un_free\r\n");
        mdc_DoorState = DOOR_CLOSING;
        mip_Input.di0_Isolation = false;
    }
}

void callback_test_reset(int8_t* string, uint8_t ucNumOfParameters)
{
	debug("Call command : test mcu reset by software\r\n");
	NVIC_SystemReset();
}

void callback_test_mram(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pSubCommand=NULL;
	char *pStringValue=NULL;
	char *pEnd=NULL;
	uint32_t pValue1=0, pValue2=0;

	debug("Call command : test spi mram read/write\r\n");

	pSubCommand = strtok(string," ");
	pStringValue = strtok(NULL," ");
	pValue1 = strtol(pStringValue,&pEnd,16);								// 문자열을 16진수 정수 숫자로 변환
	pStringValue = strtok(NULL," ");
	pValue2 = strtol(pStringValue,&pEnd,16);								// 문자열을 16진수 정수 숫자로 변환
	
	if(strstr(pSubCommand,"write") != NULL)
	{
		debug("## Write data to 0x%02X: %02X\r\n", pValue1, (uint8_t)pValue2);
		mram_byte_write(pValue1, (uint8_t)pValue2);										// MRAM 0x1FFFF번지에 0xAA Write
	}
	else if(strstr(pSubCommand,"block") != NULL)
	{
		mram_block_write();
	}
	else if(strstr(pSubCommand,"read") != NULL)
	{
		//debug("## Read data: %02X\r\n", mram_byte_read(0x1FFFF));
		debug("## Read data from 0x%02x : %02X\r\n", pValue1, mram_byte_read(pValue1));
	}
	else if(strstr(pSubCommand,"erase") != NULL)
	{
		//mram_erase(0, 0x100);
		mram_erase_Fault_data();
	}
	else
		debug("CLI Undefined Command\r\n");
}

void callback_test_obs(int8_t* string, uint8_t ucNumOfParameters)
{
	mod_Detect.ObstacleDetectCnt++;
	mdc_isOpeningByObstacle = true;
	debug("obs flag set \r\n");
}

void callback_test_debug(int8_t* string, uint8_t ucNumOfParameters)
{
    switch(string[0])
    {
    	/*
    	 * Closed/Closing 상태에서 열림버튼(3s)를 누른 경우
    	 */
    	case 0x31:
    		debugprint.Data_Print_flag ^= 0x01;
    		if(debugprint.Data_Print_flag == true)	{debug("데이터 출력 활성화 \r\n");}
    		else									{debug("데이터 출력 비활성화 \r\n");}
     		break;
    	default:
    		debug("CLI Undefined Command\r\n");
    		break;
    }
}

void callback_test_RTCSET(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pSubCommand=NULL;
	char *pValue = NULL;
	RTCTIME rtctime_set;
	uint8_t ucSeconds = 0;
	uint8_t ucMinutes = 0;
	uint8_t ucHours = 0;
	uint8_t ucDate = 0;
	uint8_t ucMonth = 0;
	uint8_t ucYears = 0;
	
	debug("Call command : test RTC SET\r\n");
	pSubCommand = strtok(string," ");
	pValue = strtok(NULL," ");
	if(strstr(pSubCommand,"set") != NULL)
	{
		sscanf(pValue,"%d-%d-%d-%d-%d-%d",&ucYears,&ucMonth,&ucDate,&ucHours,&ucMinutes,&ucSeconds);
		rtc_set_time(ucYears,ucMonth,ucDate,ucHours,ucMinutes,ucSeconds);
	}
	else if(strstr(pSubCommand,"get") != NULL)
	{
		rtctime_set = rtc_get_time();
		//PRINT 구문 없애니까 동작함 PRINT구문 추가하면 뻗음
		printf("%d-%d-%d-%d-%d-%d-%d",rtctime_set.ucYears,rtctime_set.ucMonth,rtctime_set.ucDate,rtctime_set.ucDay,rtctime_set.ucHours,rtctime_set.ucMinutes,rtctime_set.ucSeconds);
	}
}

void callback_test_DCUID(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pSubCommand=NULL;
	
	debug("Call command : test DCU ID Select\r\n");
	pSubCommand = strtok(string," ");
	
	if(strstr(pSubCommand,"ml1") != NULL)
	{
		g_unDeviceID = DCU_L1_MCAR;
		switch(g_unDeviceID)
		{
			case DCU_L1_MCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD4);							// DCU L1 M	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD4
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xDC);							// DCU L1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xDC
				break;
			case DCU_R1_MCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD8);							// DCU R1 M --> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD8
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE0);							// DCU R1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xE0
				break;
			case DCU_L1_TCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE4);							// DCU L1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE4
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xEC);							// DCU L1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xEC
				break;
			case DCU_R1_TCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE8);							// DCU R1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE8
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xF0);							// DCU R1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xF0
				break;
			default: debug("No MVB Source Configuration\r\n"); break;
		}
		debug("DCU L1 MCAR Select \r\n");
	}
	else if(strstr(pSubCommand,"ml2") != NULL)
	{
		g_unDeviceID = DCU_L2_MCAR;
		debug("DCU L2 MCAR Select \r\n");
	}
	else if(strstr(pSubCommand,"ml3") != NULL)
	{
		g_unDeviceID = DCU_L3_MCAR;
		debug("DCU L3 MCAR Select \r\n");
	}
	else if(strstr(pSubCommand,"ml4") != NULL)
	{
		g_unDeviceID = DCU_L4_MCAR;
		debug("DCU L4 MCAR Select \r\n");
	}
	else if(strstr(pSubCommand,"mr1") != NULL)
	{
		g_unDeviceID = DCU_R1_MCAR;
		switch(g_unDeviceID)
		{
			case DCU_L1_MCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD4);							// DCU L1 M	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD4
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xDC);							// DCU L1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xDC
				break;
			case DCU_R1_MCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD8);							// DCU R1 M --> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD8
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE0);							// DCU R1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xE0
				break;
			case DCU_L1_TCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE4);							// DCU L1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE4
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xEC);							// DCU L1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xEC
				break;
			case DCU_R1_TCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE8);							// DCU R1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE8
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xF0);							// DCU R1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xF0
				break;
			default: debug("No MVB Source Configuration\r\n"); break;
		}
		debug("DCU R1 MCAR Select \r\n");
	}
	else if(strstr(pSubCommand,"mr2") != NULL)
	{
		g_unDeviceID = DCU_R2_MCAR;
		debug("DCU R2 MCAR Select \r\n");
	}
	else if(strstr(pSubCommand,"mr3") != NULL)
	{
		g_unDeviceID = DCU_R3_MCAR;
		debug("DCU R3 MCAR Select \r\n");
	}
	else if(strstr(pSubCommand,"mr4") != NULL)
	{
		g_unDeviceID = DCU_R4_MCAR;
		debug("DCU R4 MCAR Select \r\n");
	}
	else
	{
		debug("not configure \r\n");
	}
}

void callback_test_Switch(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pSubCommand=NULL;
	
	debug("Call command : test Switch_command \r\n");
	pSubCommand = strtok(string," ");
	
	if(strstr(pSubCommand,"tdls1") != NULL)
	{
		mip_Test.DLS1_Test_flag ^= 0x01;
		if(mip_Test.DLS1_Test_flag == true)	{debug("DLS 1 스위치 입력을 받지 않음 \r\n");}
		else								{debug("DLS 1 스위치 입력을 받음 \r\n");}
	}
	else if(strstr(pSubCommand,"tdls2") != NULL)
	{
		mip_Test.DLS2_Test_flag ^= 0x01;
		if(mip_Test.DLS2_Test_flag == true)	{debug("DLS 2 스위치 입력을 받지 않음 \r\n");}
		else								{debug("DLS 2 스위치 입력을 받음 \r\n");}
	}
	else if(strstr(pSubCommand,"tdcs1") != NULL)
	{
		mip_Test.DCS1_Test_flag ^= 0x01;
		if(mip_Test.DCS1_Test_flag == true)	{debug("DCS 1 스위치 입력을 받지 않음 \r\n");}
		else								{debug("DCS 1 스위치 입력을 받음 \r\n");}
	}
	else if(strstr(pSubCommand,"tdcs2") != NULL)
	{
		mip_Test.DCS2_Test_flag ^= 0x01;
		if(mip_Test.DCS2_Test_flag == true)	{debug("DCS 2 스위치 입력을 받지 않음 \r\n");}
		else								{debug("DCS 2 스위치 입력을 받음 \r\n");}
	}
	else if(strstr(pSubCommand,"topen") != NULL)
	{
		mip_Test.OpenButton_Test_flag ^= 0x01;
		if(mip_Test.DCS2_Test_flag == true)	{debug("열림/닫힘 스위치 입력을 받음 \r\n");}
		else								{debug("열림/닫힘 스위치 입력을 받지않음 \r\n");}
	}
	else if(strstr(pSubCommand,"tzvr") != NULL)
	{
		mip_Input.di0_ZVR ^= 0x01;
		if(mip_Input.di0_ZVR == true)		{debug("ZVR On \r\n");}
		else								{debug("ZVR Off \r\n");}
	}
	else
		debug("CLI Undefined Command\r\n");
}

void callback_test_door_open(int8_t* string, uint8_t ucNumOfParameters)
{
    switch(string[0])
    {
    	/*
    	 * Closed/Closing 상태에서 열림버튼(3s)를 누른 경우
    	 */
    	case 0x31:
    		mip_Input.di0_OpenCloseButton ^= 0x01;
    		/*
    		 * todo : 이거 살리면 뻗을 때 있음
    		 */
//    		if(mip_Input.di0_OpenCloseButton)	debug("door open\r\n");
//    		else								debug("door close\r\n");
    		break;
    	/*
    	 * Closing 상태에서 장애물 감지인 경우
    	 */
    	case 0x32:
    		mip_Input.di0_DLS1 ^= 0x01;
    		mip_Input.di1_DLS2 ^= 0x01;
    		debug("dls off \r\n");
    		//debug("dls 입력 반전 \r\n");
    		break;
    	case 0x33:
    		mip_Input.di0_ReOpen ^= 0x01;
    		debug("Reopen 입력\r\n");
    		break;
    	case 0x34:
    		mip_Input.di0_Isolation ^= 0x01;
    		if(mip_Input.di0_Isolation)		{debug("차단\r\n");}
    		else							{debug("복귀\r\n");}
    		break;
    	case 0x35:
    		mip_Input.di0_Bypass ^= 0x01;
    		if(mip_Input.di0_Bypass)		{debug("DODBPS On\r\n");}
    		else							{debug("DODBPS Off");mdc_FlagTestDODBPS=false;}
    		break;
    	case 0x36:
    		mmf_EndDecision.OpenedByCurrent = true;
    		mip_Input.DoorClosedSwitchOn = false;
    		debug("전류로 완전열림판단\r\n");
    		break;
       	case 0x38:
			mip_Input.di0_DLS1 ^= 0x01;
			mip_Input.di1_DCS1 ^= 0x01;
    		debug("DLS1(%d) DLS2(%d), DCS1(%d), DCS2(%d)\r\n",
					mip_Input.di0_DLS1, mip_Input.di1_DLS2, 
					mip_Input.di1_DCS1, mip_Input.di1_DCS2);
			break;
       	case 0x39:
			mip_Input.di1_DLS2 ^= 0x01;
    		mip_Input.di1_DCS2 ^= 0x01;
    		debug("DLS1(%d) DLS2(%d), DCS1(%d), DCS2(%d)\r\n",
					mip_Input.di0_DLS1, mip_Input.di1_DLS2, 
					mip_Input.di1_DCS1, mip_Input.di1_DCS2);
			break;
    	default:
    		debug("CLI Undefined Command\r\n");
    		break;
    }
}

void callback_test_motor_pwm(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pSubCommand=NULL;
	char *pStringValue=NULL;
	char *pEnd=NULL;
	uint32_t pValue = 0;
	
	debug("Call command : test motor pwm\r\n");
    mmo_ConnectHbridgeGND(true);
    
	pSubCommand = strtok(string," ");
	pStringValue = strtok(NULL," ");
	pValue = strtol(pStringValue,&pEnd,10);				// 문자열을 10진수 정수 숫자로 변환
	
	if(strstr(pSubCommand,"open") != NULL)
	{
		if(mdc_DoorState==DOOR_CLOSING)
		{
			mmo_DoorFree();
//			osDelay(100);								// pwm 800으로 closing 하다가 delay 없이 바로 pwm 800으로 바로 opening 해도 안죽네...
		}
		if(pValue>800)	pValue = 800;
		mdc_DoorState=DOOR_OPENING;
		mmo_DoorOpening(pValue);
		/*
		 * todo : pValue prtinf로 출력하면 Hard_Fault발생하면서 CPU 다 죽음
		 * 그냥 printf하면 안죽고 잘 동작 함
		 */
//		debug("motor open pwm: %d\r\n", pValue);		// CPU Die
		debug("motor open pwm\r\n");					// 정상동작
	}
	else if(strstr(pSubCommand,"close") != NULL)
	{
		if(mdc_DoorState==DOOR_OPENING)
		{
			mmo_DoorFree();
//			osDelay(100);								// pwm 800으로 opening 하다가 delay 없이 바로 pwm 800으로 바로 closing 해도 안죽네...
		}
		if(pValue>800)	pValue = 800;
		mdc_DoorState=DOOR_CLOSING;
		mmo_DoorClosing(pValue);
		debug("motor close pwm: %d\r\n", pValue);
	}
	else if(strstr(pSubCommand,"brake") != NULL)
	{
		mmo_DoorFree();
//		osDelay(100);									// pwm 800으로 opening 또는 closing 하다가 delay 없이 바로 brake 해도 안죽네... -> 죽는 현상 로직때문인거 같은데...
		mmo_DoorBrake();
		debug("motor brake\r\n");
	}
	else if(strstr(pSubCommand,"free") != NULL)
	{
		mmo_DoorFree();
		debug("motor free\r\n");
	}
	else if(strstr(pSubCommand,"disable") != NULL)
	{
		mmo_MotorDisable();								// PWM Gate 4개 Off 시키면 (의도하지 않은) Brake 효과가 나타남
		debug("motor Disable\r\n");
	}
	else if(strstr(pSubCommand,"enable") != NULL)
	{
		#ifdef HW_DEPENDENT_CODE
			HAL_GPIO_WritePin(MShutDown_GPIO_Port, MShutDown_Pin, GPIO_PIN_RESET); // Low 출력 -> AND Gate true 입력 -> PWM이 H-Bridge로 출력 됨
		#endif
		debug("motor Enable\r\n");
	}
	#ifdef EMC_TEST
	if(strstr(pSubCommand,"init-pwm") != NULL)
	{
		mmf_Encoder.InitVelocity = pValue;
		debug("motor init pwm: %d\r\n", mmf_Encoder.InitVelocity);
	}
	if(strstr(pSubCommand,"init-pos") != NULL)
	{
		mmf_Encoder.InitPosition = pValue;
		debug("motor init pos: %d\r\n", mmf_Encoder.InitPosition);
	}
	if(strstr(pSubCommand,"dist") != NULL)
	{
		m_OpeningEncoderPulse = pValue;
		debug("motor total distance: %d\r\n", m_OpeningEncoderPulse);
	}
	#endif
	else
		debug("CLI Undefined Command\r\n");
}

void callback_test_dcu(int8_t* string, uint8_t ucNumOfParameters)
{
	char *pSubCommand1=NULL;
	char *pSubCommand2=NULL;
	
    debug("Call command : Set DCU ID\r\n");
    
	pSubCommand1 = strtok(string," ");
	pSubCommand2 = strtok(NULL," ");
	
	if(strstr(pSubCommand2,"m") != NULL)
	{
		if(strstr(pSubCommand1,"L1") != NULL)		{g_unDeviceID=DCU_L1_MCAR; debug("직접설정, DCU ID: DCU_L1_MCAR\r\n");}
		else if(strstr(pSubCommand1,"L2") != NULL)	{g_unDeviceID=DCU_L2_MCAR; debug("직접설정, DCU ID: DCU_L2_MCAR\r\n");}
		else if(strstr(pSubCommand1,"L3") != NULL)	{g_unDeviceID=DCU_L3_MCAR; debug("직접설정, DCU ID: DCU_L3_MCAR\r\n");}
		else if(strstr(pSubCommand1,"L4") != NULL)	{g_unDeviceID=DCU_L4_MCAR; debug("직접설정, DCU ID: DCU_L4_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R1") != NULL)	{g_unDeviceID=DCU_R1_MCAR; debug("직접설정, DCU ID: DCU_R1_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R2") != NULL)	{g_unDeviceID=DCU_R2_MCAR; debug("직접설정, DCU ID: DCU_R2_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R3") != NULL)	{g_unDeviceID=DCU_R3_MCAR; debug("직접설정, DCU ID: DCU_R3_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R4") != NULL)	{g_unDeviceID=DCU_R4_MCAR; debug("직접설정, DCU ID: DCU_R4_MCAR\r\n");}
	}
	else if(strstr(pSubCommand2,"t") != NULL)
	{
		if(strstr(pSubCommand1,"L1") != NULL)		{g_unDeviceID=DCU_L1_TCAR; debug("직접설정, DCU ID: DCU_L1_TCAR\r\n");}
		else if(strstr(pSubCommand1,"L2") != NULL)	{g_unDeviceID=DCU_L2_TCAR; debug("직접설정, DCU ID: DCU_L2_TCAR\r\n");}
		else if(strstr(pSubCommand1,"L3") != NULL)	{g_unDeviceID=DCU_L3_TCAR; debug("직접설정, DCU ID: DCU_L3_TCAR\r\n");}
		else if(strstr(pSubCommand1,"L4") != NULL)	{g_unDeviceID=DCU_L4_TCAR; debug("직접설정, DCU ID: DCU_L4_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R1") != NULL)	{g_unDeviceID=DCU_R1_TCAR; debug("직접설정, DCU ID: DCU_R1_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R2") != NULL)	{g_unDeviceID=DCU_R2_TCAR; debug("직접설정, DCU ID: DCU_R2_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R3") != NULL)	{g_unDeviceID=DCU_R3_TCAR; debug("직접설정, DCU ID: DCU_R3_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R4") != NULL)	{g_unDeviceID=DCU_R4_TCAR; debug("직접설정, DCU ID: DCU_R4_TCAR\r\n");}
	}
	
	if(strstr(pSubCommand1," "))
	{
		debug("CLI Undefined Command\r\n");
		debug("'dcu L1 m' : Set DCU ID to DCU_L1_MCAR\r\n");				// DCU MCAR 설정
		debug("'dcu L2 m' : Set DCU ID to DCU_L2_MCAR\r\n");
		debug("'dcu L3 m' : Set DCU ID to DCU_L3_MCAR\r\n");
		debug("'dcu L4 m' : Set DCU ID to DCU_L4_MCAR\r\n");
		debug("'dcu R1 m' : Set DCU ID to DCU_R1_MCAR\r\n");
		debug("'dcu R2 m' : Set DCU ID to DCU_R2_MCAR\r\n");
		debug("'dcu R3 m' : Set DCU ID to DCU_R3_MCAR\r\n");
		debug("'dcu R4 m' : Set DCU ID to DCU_R4_MCAR\r\n");
		
		debug("'dcu L1 t' : Set DCU ID to DCU_L1_TCAR\r\n");				// DCU TCAR 설정
		debug("'dcu L2 t' : Set DCU ID to DCU_L2_TCAR\r\n");
		debug("'dcu L3 t' : Set DCU ID to DCU_L3_TCAR\r\n");
		debug("'dcu L4 t' : Set DCU ID to DCU_L4_TCAR\r\n");
		debug("'dcu R1 t' : Set DCU ID to DCU_R1_TCAR\r\n");
		debug("'dcu R2 t' : Set DCU ID to DCU_R2_TCAR\r\n");
		debug("'dcu R3 t' : Set DCU ID to DCU_R3_TCAR\r\n");
		debug("'dcu R4 t' : Set DCU ID to DCU_R4_TCAR\r\n");
	}
}

void callback_version(int8_t* string, uint8_t ucNumOfParameters)
{
    debug("Call command : Show Version\r\n");
    debug("DCU Software Version : %d.%d\r\n",VERSION_MAJOR,VERSION_MINOR);
}

void mac_TaskRS485(void const * argument)
{
	osEvent event;
	uint8_t aTxBuffer[11] = {0,};
	uint8_t Packet_Number_count = 0;
	uint8_t Slave_up_data = 0, Slave_down_data = 0;
	uint8_t Master_up_data = 0, Master_down_data = 0, id_address =0;
	uint8_t dcu_address_MCAR[6] = {DCU_L1_MCAR,DCU_L2_MCAR,DCU_L3_MCAR,DCU_R2_MCAR,DCU_R3_MCAR,DCU_R4_MCAR};
	uint8_t dcu_address_TCAR[6] = {DCU_L1_TCAR,DCU_L2_TCAR,DCU_L3_TCAR,DCU_R2_TCAR,DCU_R3_TCAR,DCU_R4_TCAR};
	uint16_t checksum_value = 0;
	uint32_t PreviousWakeTime = osKernelSysTick();
	uint8_t loopcnt = 0;
	
	RTCTIME rs485_rtctime;
	
	SlaveRunning_Check(event);
	if(m_isMasterSlave == MASTER_DCU)
	{
//		osDelay(2000); //jeon_191007 2s dealy

		debug("5. [Master Start] RS485 Task\\r\n");
	}
	else // SLAVE_DCU
	{
		__HAL_UART_DISABLE(&huart1);
		while(m_isTaskExecution == false)
		{
			osDelay(200);
		}
		debug("5. [Slave Start] rs485 Task\r\n");
		  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
		  {
			//_Error_Handler(__FILE__, __LINE__);
		  }
		PreviousWakeTime = osKernelSysTick();
	}

	error_485list_time[DCU_RS485_ERROR_L1] = osKernelSysTick(); //jeon_20190423
	error_485list_time[DCU_RS485_ERROR_L2] = osKernelSysTick(); //jeon_20190423
	error_485list_time[DCU_RS485_ERROR_L3] = osKernelSysTick(); //jeon_20190423
	error_485list_time[DCU_RS485_ERROR_R2] = osKernelSysTick(); //jeon_20190423
	error_485list_time[DCU_RS485_ERROR_R3] = osKernelSysTick(); //jeon_20190423
	error_485list_time[DCU_RS485_ERROR_R4] = osKernelSysTick(); //jeon_20190423

	for(;;)
	{
		/*
		 * 절체 시 Master는 아무것도 수행안함
		 */
		if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
		{
			osDelay(500);
			loopcnt++;
			if(loopcnt>2)
			{
				loopcnt=0;
				__HAL_UART_DISABLE(&huart1);
				HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
				//debug("# Master 485 Task 실행 안함\r\n");
			}
			continue;
		}
		
		/*
		 * L4인 경우 L1~L3의 정보를 수신, L1~L4의 정보룰 취합
		 */
		if((g_unDeviceID == DCU_L4_MCAR) || (g_unDeviceID == DCU_L4_TCAR))
		{
#if 0 //jeon_190821
			osDelayUntil(&PreviousWakeTime, 120UL); //jeon_190716 orig:60UL
#else	        
	        osDelay(120);
#endif
			if(get_diff_tick(osKernelSysTick(), error_485list_time[DCU_RS485_ERROR_L1]) > 100000)
			{
				error_485_flag[DCU_RS485_ERROR_L1] += 1;
//				debug("DCU_RS485_ERROR_L1..\r\n");
				if((g_unDeviceID == DCU_L4_MCAR) && (error_485_flag[DCU_RS485_ERROR_L1] > 10)) mvbSaveDataDCUL(DCU_L1_MCAR);
				else if((g_unDeviceID == DCU_L4_TCAR) && (error_485_flag[DCU_RS485_ERROR_L1] > 10)) mvbSaveDataDCUL(DCU_L1_TCAR);
			}

			if(get_diff_tick(osKernelSysTick(), error_485list_time[DCU_RS485_ERROR_L2]) > 100000)
			{
				error_485_flag[DCU_RS485_ERROR_L2] += 1;
//				debug("DCU_RS485_ERROR_L2..\r\n");
				if((g_unDeviceID == DCU_L4_MCAR) && (error_485_flag[DCU_RS485_ERROR_L2] > 10)) mvbSaveDataDCUL(DCU_L2_MCAR);
				else if((g_unDeviceID == DCU_L4_TCAR) && (error_485_flag[DCU_RS485_ERROR_L2] > 10)) mvbSaveDataDCUL(DCU_L2_TCAR);
			}

			if(get_diff_tick(osKernelSysTick(), error_485list_time[DCU_RS485_ERROR_L3]) > 100000)
			{
				error_485_flag[DCU_RS485_ERROR_L3] += 1;
//				debug("DCU_RS485_ERROR_L3..\r\n");
				if((g_unDeviceID == DCU_L4_MCAR) && (error_485_flag[DCU_RS485_ERROR_L3] > 10)) mvbSaveDataDCUL(DCU_L3_MCAR);
				else if((g_unDeviceID == DCU_L4_TCAR) && (error_485_flag[DCU_RS485_ERROR_L3] > 10)) mvbSaveDataDCUL(DCU_L3_TCAR);
			}
			/*
			 * Data Request 전송
			 */

			if(id_address<8) // DCU1 -> DCU2~DCU7
			{
				//(((Second/10)<<4) & 0x70) + ((Second%10) & 0x0f)
				if(id_address >= 6)	id_address = 0;
				aTxBuffer[0] = 0xAA;																// start packet
				if(m_CardefineID == Define_MCAR) aTxBuffer[1] = dcu_address_MCAR[id_address];		// M카일떈 M카의 ID를 호출
				else if(m_CardefineID == Define_TCAR) aTxBuffer[1] = dcu_address_TCAR[id_address];	// T카일 떈 T카의 ID를 호출
				aTxBuffer[2] = Packet_Number_count++;												// 0~255 카운트 하는 값을 전송
				checksum_value = (aTxBuffer[0] + aTxBuffer[1] + aTxBuffer[2]);						// [0] , [1] , [2] 를 더한 값을 저장
				aTxBuffer[3] = m_isMasterSlaveChange;												// 절체 명령 정보
				aTxBuffer[4] = rtc_time.ucYears;
				aTxBuffer[5] = rtc_time.ucMonth;
				aTxBuffer[6] = rtc_time.ucDate;
				aTxBuffer[7] = rtc_time.ucHours;
				aTxBuffer[8] = rtc_time.ucMinutes;
				aTxBuffer[9] = ((checksum_value & 0xFF00) >>8);										// 체크섬 high(상위 바이트를 3 바이트에 저장)
				aTxBuffer[10] = (checksum_value & 0x00FF);											// 체크섬 low(하위 바이트를 4 바이트에 저장 시켜 전송)
				if(HAL_UART_Transmit(&huart1, (uint8_t*)aTxBuffer, 11, 1000) != HAL_OK)
				{
					debug("## Transmit Error\r\n");
				}
				else
				{
					#ifdef RS485_PACKET_MONITORING
					printf("## [RS485] DCU1 패킷 송신:");
						for(int i=0; i<11; i++)	debug("%02X ", aTxBuffer[i]);
						printf("\r\n");
					#endif
					id_address++;
				}
			}

			/*
			 * Slave(L1~L3)가 보낸 Data 수신 후 MVB Tx Buffer에 저장
			 */
			HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
			osDelay(1);

			event = osMessageGet(hRx1Queue, 200);
			if(event.status == osEventMessage)
			{
				#ifdef RS485_PACKET_MONITORING
				printf("\r\n## [RS485] DCU1 패킷 수신: ");
					for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
					printf("\r\n\r\n");
				#endif
				error_list_time[DCU_RS485_ERROR] = osKernelSysTick();
				Erase_fault(DCU_RS485_ERROR);
				if(aUartRx1Buffer[0] == 0xBB)
				{
					/* CC + 도어정보 + CHECK SUM HIGH + CHECK SUM LOW */
					checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
					Master_up_data = ((checksum_value & 0xFF00) >> 8);
					Master_down_data =(checksum_value & 0x00FF);
					if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //체크섬 이 맞으면
					{
						if((g_unDeviceID == DCU_L4_MCAR) || (g_unDeviceID == DCU_L4_TCAR))
						{
							if((aUartRx1Buffer[1] == DCU_L1_MCAR) || (aUartRx1Buffer[1] == DCU_L1_TCAR))
							{
								error_485list_time[DCU_RS485_ERROR_L1] = osKernelSysTick();
								error_485_flag[DCU_RS485_ERROR_L1] = 0;
							}
							else if((aUartRx1Buffer[1] == DCU_L2_MCAR) || (aUartRx1Buffer[1] == DCU_L2_TCAR))
							{
								error_485list_time[DCU_RS485_ERROR_L2] = osKernelSysTick();
								error_485_flag[DCU_RS485_ERROR_L2] = 0;
							}
							else if((aUartRx1Buffer[1] == DCU_L3_MCAR) || (aUartRx1Buffer[1] == DCU_L3_TCAR))
							{
								error_485list_time[DCU_RS485_ERROR_L3] = osKernelSysTick();
								error_485_flag[DCU_RS485_ERROR_L3] = 0;
							}
						}
//						osDelay(10);
//						for(int i =0; i<11; i++)
//						{
//							aUartRx1Buffer[i] = 0x00;
//						}
//						HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);

						if(((aUartRx1Buffer[1] == DCU_L2_MCAR) || (aUartRx1Buffer[1] == DCU_L3_MCAR) || (aUartRx1Buffer[1] == DCU_L1_MCAR)) ||
						   ((aUartRx1Buffer[1] == DCU_L2_TCAR) || (aUartRx1Buffer[1] == DCU_L3_TCAR) || (aUartRx1Buffer[1] == DCU_L1_TCAR))	)
						{
							mvbSaveDataDCUL(aUartRx1Buffer[1]);
						}
					}

				}
				else /* 첫번째 바이트가 bb를 받지 못한 경우*/
				{
					#ifdef RS485_PACKET_MONITORING
					printf("\r\n## [RS485] DCU1 패킷 오수신: ");
						for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						printf("\r\n\r\n");
					#endif
					osDelay(1);
					//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx1Buffer, 11,5);
					/*
					 * todo : 밀린 패킷 수신 시 초기화 동작 Master/Slave 통신에도 추가할 것
					 */
					  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
					  {
						//_Error_Handler(__FILE__, __LINE__);
					  }
//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}

				}
			}
			else /* 11바이트를 받지 못한 경우*/
			{
				#ifdef RS485_PACKET_MONITORING
				printf("\r\n## [RS485] DCU1 패킷 미수신: ");
				//	for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
					printf("\r\n\r\n");
				#endif
				osDelay(1);
				//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx1Buffer, 11,5);
//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}

				if(get_diff_tick(osKernelSysTick(), error_list_time[DCU_RS485_ERROR]) > 100000)
				{
					FaultFlag_SET(DCU_RS485_ERROR);
				}


			}
//				for(int i =0; i<11; i++)
//				{
//					aUartRx1Buffer[i] = 0x00;
//				}
		}
		/*
		 * R1인 경우  R2~R4가 전송한 정보를 수신, R1~R4의 정보를 취합
		 */
		else if((g_unDeviceID == DCU_R1_MCAR) || (g_unDeviceID == DCU_R1_TCAR))
		{
			if(get_diff_tick(osKernelSysTick(), error_485list_time[DCU_RS485_ERROR_R2]) > 100000)
			{
				error_485_flag[DCU_RS485_ERROR_R2] += 1;
//				debug("DCU_RS485_ERROR_R2..\r\n");
				if((g_unDeviceID == DCU_R1_MCAR) && (error_485_flag[DCU_RS485_ERROR_R2] > 10)) mvbSaveDataDCUR(DCU_R2_MCAR);
				else if((g_unDeviceID == DCU_R1_TCAR) && (error_485_flag[DCU_RS485_ERROR_R2] > 10)) mvbSaveDataDCUR(DCU_R2_TCAR);
			}

			if(get_diff_tick(osKernelSysTick(), error_485list_time[DCU_RS485_ERROR_R3]) > 100000)
			{
				error_485_flag[DCU_RS485_ERROR_R3] += 1;
//				debug("DCU_RS485_ERROR_R3..\r\n");
				if((g_unDeviceID == DCU_R1_MCAR) && (error_485_flag[DCU_RS485_ERROR_R3] > 10)) mvbSaveDataDCUR(DCU_R3_MCAR);
				else if((g_unDeviceID == DCU_R1_TCAR) && (error_485_flag[DCU_RS485_ERROR_R3] > 10)) mvbSaveDataDCUR(DCU_R3_TCAR);
			}

			if(get_diff_tick(osKernelSysTick(), error_485list_time[DCU_RS485_ERROR_R4]) > 100000)
			{
				error_485_flag[DCU_RS485_ERROR_R4] += 1;
//				debug("DCU_RS485_ERROR_R4..\r\n");
				if((g_unDeviceID == DCU_R1_MCAR) && (error_485_flag[DCU_RS485_ERROR_R4] > 10)) mvbSaveDataDCUR(DCU_R4_MCAR);
				else if((g_unDeviceID == DCU_R1_TCAR) && (error_485_flag[DCU_RS485_ERROR_R4] > 10)) mvbSaveDataDCUR(DCU_R4_TCAR);
			}

			HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
			osDelay(1);
			
			event = osMessageGet(hRx1Queue, 200);
			if(event.status == osEventMessage)
			{

				error_list_time[DCU_RS485_ERROR] = osKernelSysTick();
				Erase_fault(DCU_RS485_ERROR);
				if(aUartRx1Buffer[0] == 0xBB)
				{
					/* CC + 도어정보 + CHECK SUM HIGH + CHECK SUM LOW */
					checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
					Master_up_data = ((checksum_value & 0xFF00) >> 8);
					Master_down_data =(checksum_value & 0x00FF);
					if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //체크섬 이 맞으면
					{
						if((g_unDeviceID == DCU_R1_MCAR) || (g_unDeviceID == DCU_R1_TCAR))
						{
							if((aUartRx1Buffer[1] == DCU_R2_MCAR) || (aUartRx1Buffer[1] == DCU_R2_TCAR))
							{
								error_485list_time[DCU_RS485_ERROR_R2] = osKernelSysTick();
								error_485_flag[DCU_RS485_ERROR_R2] = 0;
							}
							else if((aUartRx1Buffer[1] == DCU_R3_MCAR) || (aUartRx1Buffer[1] == DCU_R3_TCAR))
							{
								error_485list_time[DCU_RS485_ERROR_R3] = osKernelSysTick();
								error_485_flag[DCU_RS485_ERROR_R3] = 0;
							}
							else if((aUartRx1Buffer[1] == DCU_R4_MCAR) || (aUartRx1Buffer[1] == DCU_R4_TCAR))
							{
								error_485list_time[DCU_RS485_ERROR_R4] = osKernelSysTick();
								error_485_flag[DCU_RS485_ERROR_R4] = 0;
							}
						}

						if(((aUartRx1Buffer[1] == DCU_R2_MCAR) || (aUartRx1Buffer[1] == DCU_R3_MCAR) || (aUartRx1Buffer[1] == DCU_R4_MCAR)) ||
						   ((aUartRx1Buffer[1] == DCU_R2_TCAR) || (aUartRx1Buffer[1] == DCU_R3_TCAR) || (aUartRx1Buffer[1] == DCU_R4_TCAR))	)
						{
							#ifdef RS485_PACKET_MONITORING
							printf("\r\n## [RS485] DCU8 패킷 수신: ");
							for(int i=0; i<11; i++)
							 printf("%02X ", aUartRx1Buffer[i]);
								printf("\r\n\r\n");
							#endif
							mvbSaveDataDCUR(aUartRx1Buffer[1]);

						}



					}
//					osDelay(10);
//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}
//					HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);

				}
				else if(aUartRx1Buffer[0] == 0xAA)
				{
					#ifdef RS485_PACKET_MONITORING
						debug("\r\n\## [RS485]  DCU%d 마스터 패킷 수신: ", g_unDeviceID);
					//	for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						debug("\r\n\\r\n");
					#endif
				}
				else
				{
					#ifdef RS485_PACKET_MONITORING
					printf("\r\n## [RS485] DCU1 패킷 오수신: ");
						for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						printf("\r\n\r\n");
					#endif
//					HAL_UART_Receive(&huart1, (uint8_t *)aUartRx1Buffer, 11,5);
					osDelay(1);
					  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
					  {
						//_Error_Handler(__FILE__, __LINE__);
					  }
					  HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}
				}
			}
			else
			{
				#ifdef RS485_PACKET_MONITORING
				printf("\r\n## [RS485] DCU1 패킷 미수신: ");
				//	for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
					printf("\r\n\r\n");
				#endif
				osDelay(1);
				//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx1Buffer, 11,5);

//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}

				if(get_diff_tick(osKernelSysTick(), error_list_time[DCU_RS485_ERROR]) > 100000)
				{
					FaultFlag_SET(DCU_RS485_ERROR);
				}

			}
//			osDelay(10);
//			for(int i =0; i<11; i++)
//			{
//				aUartRx1Buffer[i] = 0x00;
//			}
//			HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);

		}
		/*
		 * L1~L3, R2~R4는 자신의 id 요청이 들어왔을 경우 정보를 보낸다.
		 */
		else
		{
			/*
			 * [중요] 인터럽트 발생 후 HAL_UART_Receive_IT를 호출해야마 HAL_OK를 return
			 *       이후 계속 HAL_UART_Receive_IT를 호출하면 HAL_BUSY를 return
			 *       만약 인터럽트 ready가 안된 상태에서 HAL_UART_Receive_IT를 호출할 경우 더이상 인터럽트가 발생하지 않을 수 있으므로
			 *       할꺼 없으면 주기적으로 HAL_UART_Receive_IT 계속 호출
			 */
			HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
			osDelay(1);
			
			event = osMessageGet(hRx1Queue, 200);
			if(event.status == osEventMessage)
			{
				error_list_time[DCU_RS485_ERROR] = osKernelSysTick();
				Erase_fault(DCU_RS485_ERROR);
				if(aUartRx1Buffer[0] == 0xAA)
				{
					checksum_value = (aUartRx1Buffer[0] + aUartRx1Buffer[1] + aUartRx1Buffer[2]);
					Slave_up_data = ((checksum_value & 0xFF00)>>8);
					Slave_down_data = (checksum_value & 0x00FF);
					if((Slave_up_data == aUartRx1Buffer[9]) && (Slave_down_data == aUartRx1Buffer[10]))
					{
						if(aUartRx1Buffer[1] == g_unDeviceID) // 마스터에서 자신을 요청한 경우
						{
							m_isMasterSlaveChange = aUartRx1Buffer[3];

							aTxBuffer[0] = 0xBB;
							aTxBuffer[1] = g_unDeviceID;
							aTxBuffer[2] = Statement_DCU(2);	 // 상태 정보
							aTxBuffer[3] = Statement_DCU(3);	 // 상태 정보 2
							aTxBuffer[4] = FalutStatement_DCU(4);// 고장 정보
							aTxBuffer[5] = FalutStatement_DCU(5);// 고장 정보 2
							aTxBuffer[6] = (uint8_t)((VERSION_MAJOR*10) + VERSION_MINOR);				 // 소프트웨어 버전 1.0이면 10의 데이터를 보내야함
							aTxBuffer[7] = m_isFaultCount;		 // 고장 갯수
							aTxBuffer[8] = (uint8_t)(mdm_time.Closing / 100);				 // 닫힘 시간
							checksum_value = (aTxBuffer[0]+aTxBuffer[1]+aTxBuffer[2]+aTxBuffer[3]);
							Slave_up_data = ((checksum_value & 0xFF00)>>8);
							Slave_down_data = (checksum_value & 0x00FF);
							aTxBuffer[9] = Slave_up_data;
							aTxBuffer[10] = Slave_down_data;
							osDelay(3);
							if(HAL_UART_Transmit(&huart1, (uint8_t*)aTxBuffer, 11, 1000) != HAL_OK)
							{
								debug(" [RS485] transmit error\r\n");
							}
							else
							{
								#ifdef RS485_PACKET_MONITORING
									debug("## [RS485] DCU%d 패킷 송신: ", g_unDeviceID);
									for(int i=0; i<11; i++)	debug("%02X ", aTxBuffer[i]);
									debug("\r\n");
								#endif
							}
							
							if(m_RtcDefine == SET_NONE)
							{
								rtc_time.ucYears = aUartRx1Buffer[4];
								rtc_time.ucMonth = aUartRx1Buffer[5];
								rtc_time.ucDate  = aUartRx1Buffer[6];
								rtc_time.ucHours = aUartRx1Buffer[7];
								rtc_time.ucMinutes = aUartRx1Buffer[8];
								rtc_time.ucSeconds = 0x00;

								
								rs485_rtctime = rtc_get_time();
															
								if((rs485_rtctime.ucYears == aUartRx1Buffer[4]) && (rs485_rtctime.ucMonth == aUartRx1Buffer[5]) && (rs485_rtctime.ucDate == aUartRx1Buffer[6]) &&
								  (rs485_rtctime.ucHours == aUartRx1Buffer[7]) && (rs485_rtctime.ucMinutes == aUartRx1Buffer[8]))			
								{	
									printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",
										((rs485_rtctime.ucYears>>4)*10)+(rs485_rtctime.ucYears&0x0f),
										((rs485_rtctime.ucMonth>>4)*10)+(rs485_rtctime.ucMonth&0x0f),
										((rs485_rtctime.ucDate>>4)*10)+(rs485_rtctime.ucDate&0x0f),
										((rs485_rtctime.ucHours>>4)*10)+(rs485_rtctime.ucHours&0x0f),
										((rs485_rtctime.ucMinutes>>4)*10)+(rs485_rtctime.ucMinutes&0x0f),
										((rs485_rtctime.ucSeconds>>4)*10)+(rs485_rtctime.ucSeconds&0x0f));
									
									m_RtcDefine = SET_OK;
								}	
								else
								{
									rtc_set_time(((aUartRx1Buffer[4]>>4)*10)+(aUartRx1Buffer[4]&0x0f),  //년도
											 ((aUartRx1Buffer[5]>>4)*10)+(aUartRx1Buffer[5]&0x0f),	//월
											 ((aUartRx1Buffer[6]>>4)*10)+(aUartRx1Buffer[6]&0x0f),  //일
											 ((aUartRx1Buffer[7]>>4)*10)+(aUartRx1Buffer[7]&0x0f),  //시
											 ((aUartRx1Buffer[8]>>4)*10)+(aUartRx1Buffer[8]&0x0f),  //분
											 ((0x00>>4)*10)+(0x00&0x0f)); 			   //초
								
									rs485_rtctime = rtc_get_time();
									
									if((rs485_rtctime.ucYears == aUartRx1Buffer[4]) && (rs485_rtctime.ucMonth == aUartRx1Buffer[5]) && (rs485_rtctime.ucDate == aUartRx1Buffer[6]) &&
									  (rs485_rtctime.ucHours == aUartRx1Buffer[7]) && (rs485_rtctime.ucMinutes == aUartRx1Buffer[8]))			
									{	
										printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",
											((rs485_rtctime.ucYears>>4)*10)+(rs485_rtctime.ucYears&0x0f),
											((rs485_rtctime.ucMonth>>4)*10)+(rs485_rtctime.ucMonth&0x0f),
											((rs485_rtctime.ucDate>>4)*10)+(rs485_rtctime.ucDate&0x0f),
											((rs485_rtctime.ucHours>>4)*10)+(rs485_rtctime.ucHours&0x0f),
											((rs485_rtctime.ucMinutes>>4)*10)+(rs485_rtctime.ucMinutes&0x0f),
											((rs485_rtctime.ucSeconds>>4)*10)+(rs485_rtctime.ucSeconds&0x0f));
										
										m_RtcDefine = SET_OK;
									}
									else
									{
										rtc_set_time(((aUartRx1Buffer[4]>>4)*10)+(aUartRx1Buffer[4]&0x0f),  //년도
													 ((aUartRx1Buffer[5]>>4)*10)+(aUartRx1Buffer[5]&0x0f),	//월
													 ((aUartRx1Buffer[6]>>4)*10)+(aUartRx1Buffer[6]&0x0f),  //일
													 ((aUartRx1Buffer[7]>>4)*10)+(aUartRx1Buffer[7]&0x0f),  //시
													 ((aUartRx1Buffer[8]>>4)*10)+(aUartRx1Buffer[8]&0x0f),  //분
													 ((0x00>>4)*10)+(0x00&0x0f)); 			   //초
									
										rs485_rtctime = rtc_get_time();
										
										printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",
											((rs485_rtctime.ucYears>>4)*10)+(rs485_rtctime.ucYears&0x0f),
											((rs485_rtctime.ucMonth>>4)*10)+(rs485_rtctime.ucMonth&0x0f),
											((rs485_rtctime.ucDate>>4)*10)+(rs485_rtctime.ucDate&0x0f),
											((rs485_rtctime.ucHours>>4)*10)+(rs485_rtctime.ucHours&0x0f),
											((rs485_rtctime.ucMinutes>>4)*10)+(rs485_rtctime.ucMinutes&0x0f),
											((rs485_rtctime.ucSeconds>>4)*10)+(rs485_rtctime.ucSeconds&0x0f));
										
										m_RtcDefine = SET_OK;
									}
								}									
							}
							HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
							osDelay(3);
							#ifdef RS485_PACKET_MONITORING
								debug("## [RS485] DCU%d 패킷 수신: ", g_unDeviceID);
								for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
								debug("\r\n");
							#endif
						}
					}
//					osDelay(10);
//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}
//					HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);

				}
				//if(aUartRx1Buffer[0] != 0xBB) - > BB일경우에는 다른 슬레이브의 데이터를 받았기 떄문에 정상이고, 다른 경우에는 데이터가 밀려서 받은 상황
				else if(aUartRx1Buffer[0] == 0xBB)
				{
					#ifdef RS485_PACKET_MONITORING
						debug("\r\n\## [RS485] 타 DCU%d 패킷 수신: ", g_unDeviceID);
					//	for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						debug("\r\n\\r\n");
					#endif
				}
				else
				{
					#ifdef RS485_PACKET_MONITORING
						debug("\r\n\## [RS485] DCU%d 밀린 패킷 수신: ", g_unDeviceID);
						for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						debug("\r\n\\r\n");
					#endif
					//HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
					//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx1Buffer, 11,5);
					osDelay(1);
					  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
					  {
					    //_Error_Handler(__FILE__, __LINE__);
					  }
//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}
				}

			}
			else
			{

				//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx1Buffer, 11,5);
				osDelay(1);
//					for(int i =0; i<11; i++)
//					{
//						aUartRx1Buffer[i] = 0x00;
//					}

				if(get_diff_tick(osKernelSysTick(), error_list_time[DCU_RS485_ERROR]) > 100000)
				{
					FaultFlag_SET(DCU_RS485_ERROR);
				}
			}

		}

//		if(g_unDeviceID == DCU_R1_MCAR)      mvbSourceWrite(DCU_R1_M_MVB_SOURSE_SD);									// Port 0xD8
//		else if(g_unDeviceID == DCU_L4_MCAR) mvbSourceWrite(DCU_L1_M_MVB_SOURSE_SD);									// Port 0xD4
//		else if(g_unDeviceID == DCU_R1_TCAR) mvbSourceWrite(DCU_R1_T_MVB_SOURSE_SD);									// Port 0xE8
//		else if(g_unDeviceID == DCU_L4_TCAR) mvbSourceWrite(DCU_L1_T_MVB_SOURSE_SD);									// Port 0xE4

	}
}

void mc_TaskCLICommand(void const * argument) {
    uint32_t PreviousWakeTime = osKernelSysTick();

    /*
     * 주기적으로 Master와 Slave 간 통신을 통한 alive check를 위해 둘 다 바로 실행되어야 함.
     */
	debug("6. [Master & Slave Start] Communication Task\r\n");

	InitQueue(&rx_Int_queue);
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_ORE);						//오버런에러,노이즈에러,패리티 에러 발생시 인터럽트 호출한다.

    for(;;)
    {
        /* Place this task in the blocked state until it is time to run again. */
        osDelayUntil(&PreviousWakeTime, 10UL);

        /*
         * USB(Virtual COM Port) 및 UART0 둘 다 받아야 하므로 함수 호출로 패킷 수신해야 함.
         */
		isReceivePacket();		// UART0_CONSOLE
    }
}

void mvbSourceConfigurationRead(uint8_t Fcode, uint8_t PortNumber)
{
	HAL_StatusTypeDef retRXval=5, retTXval=5;
	volatile static uint8_t mvbDataLength=5;
	uint16_t tmpPortNumber=0;
	
	/*
	 * 포트 설정값 읽기 위해 MVB로 패킷 전송
	 */
	#if 1
	{
		tmpPortNumber = (uint16_t)PortNumber<<2;
		
		spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_READ;				// 메모리 명령어, Write
		spiTxBuffer[1] = 0x20|((tmpPortNumber&0xFF00)>>8);							// LA PCS Offset Address (0x2000~0x27FF)
		spiTxBuffer[2] = tmpPortNumber&0x00FF;
		spiTxBuffer[3] = 0x00;
		spiTxBuffer[4] = 0x00;
		
		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
			retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiTxBuffer, mvbDataLength, 5000);	// 1패킷 끊어서 보내고
//jeon_190427		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
	}
	#endif
	
	/*
	 * MVB로부터 포트 설정 값 Read
	 */
	retRXval = HAL_SPI_Receive(&hspi5, (uint8_t*)spiRxBuffer, mvbDataLength, 1000);
	HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
	switch(retRXval)
	{
		case HAL_OK: break;
		case HAL_TIMEOUT: debug("SPI Slave RX Timeout\r\n"); break;
		case HAL_ERROR: debug("SPI Slave RX Error\r\n"); break;
		default: break;
	}
	
	if(retRXval==HAL_OK)
	{
		debug("# mvb Configuration Read: ");
		
		for(uint8_t num=0; num<mvbDataLength; num++)
			debug("%02X ", spiRxBuffer[num]);
		debug("\r\n");
	}
}

void mvbSourceConfiguration(uint8_t Fcode, uint8_t PortNumber)
{
	HAL_StatusTypeDef retTXval=5;
	
	spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;				// 메모리 명령어, Write
	spiTxBuffer[1] = 0x20;														// LA PCS Offset Address (0x2000~0x27FF)
	spiTxBuffer[2] = PortNumber;
	spiTxBuffer[3] = Fcode|MVB_SLAVE_SOURCE;									// F_Code(상위4bit), Source
	spiTxBuffer[4] = 0x00;
	
	HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
		retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiTxBuffer, 5, 5000);	// 1패킷 끊어서 보내고
	HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
	switch(retTXval)
	{
		#ifdef MVB_PACKET_MONITORING
		case HAL_OK: debug(" MVB SRC  -(Fcode %d, Port %02X)-> TCMS SNK\r\n", Fcode>>4, PortNumber); break;
		#else
		case HAL_OK: break;
		#endif
		case HAL_TIMEOUT: debug("SPI5 TX Timeout for MVB Source Configuration\r\n"); break;
		case HAL_ERROR: debug("SPI5 TX Error for MVB Source Configuration\r\n"); break;
		default: break;
	}
	
	/*
	 * Fcode에 따른 데이터 Size 결정
	 */
	switch(Fcode)
	{
		case MVB_FCODE0_2BYTE: mvbTxDataLength = 2; break;
		case MVB_FCODE1_4BYTE: mvbTxDataLength = 4; break;
		case MVB_FCODE2_8BYTE: mvbTxDataLength = 8; break;
		case MVB_FCODE3_16BYTE: mvbTxDataLength = 16; break;
		case MVB_FCODE4_32BYTE: mvbTxDataLength = 32; break;
		default: break;
	}
}


void mvbSourceWrite(uint8_t PortNumber)
{
	HAL_StatusTypeDef retTXval=5;
	volatile static uint8_t num=0;
	uint16_t tmpPortNumber=0;
	uint8_t mvbTxDataTotalLength=0;
	
	mvbTxDataTotalLength = mvbTxDataLength+3;
	tmpPortNumber = (uint16_t)PortNumber<<2;
	
	spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;								// 메모리 명령어, Write
	spiTxBuffer[1] = (tmpPortNumber&0xFF00)>>8;													// LA Data Offset (0x0000~0x1FFFF)
	spiTxBuffer[2] = tmpPortNumber&0x00FF;

	/*
	 * Packet Selection
	 */
	#if 0
		/*
		 * Test Packet
		 */
		num++;
		for(uint8_t i=3; i<mvbTxDataTotalLength; i++)
			spiTxBuffer[i] = num;
	#else
		switch(g_unDeviceID)
		{
			case DCU_L4_MCAR: mvbSaveDataDCUL(DCU_L4_MCAR); break;								// L1 DCU가 모은 정보(L1~L4)를 MVB를 통해 Write
			case DCU_R1_MCAR: mvbSaveDataDCUR(DCU_R1_MCAR); break;								// R1 DCU가 모은 정보(R1~R4)를 MVB를 통해 Write
			case DCU_L4_TCAR: mvbSaveDataDCUL(DCU_L4_TCAR); break;
			case DCU_R1_TCAR: mvbSaveDataDCUR(DCU_R1_TCAR); break;
			default: break;
		}
	#endif

		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
		retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiTxBuffer, mvbTxDataTotalLength, 5000);		// 다시 데이터를 전송
		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);


	switch(retTXval)
	{
		case HAL_OK:
			#ifdef MVB_PACKET_MONITORING
			{
				if(PortNumber==DCU_L1_M_MVB_SOURSE_SD||
				   PortNumber==DCU_R1_M_MVB_SOURSE_SD||
				   PortNumber==DCU_L1_T_MVB_SOURSE_SD||
				   PortNumber==DCU_R1_T_MVB_SOURSE_SD)
					printf("# mvb Source Write(SD, 0x%02X) ", PortNumber);
				else if(PortNumber==DCU_L1_M_MVB_SOURSE_TD||
						PortNumber==DCU_R1_M_MVB_SOURSE_TD||
						PortNumber==DCU_L1_T_MVB_SOURSE_TD||
						PortNumber==DCU_R1_T_MVB_SOURSE_TD)
					printf("# mvb Source Write(TD, 0x%02X) ", PortNumber);
					
				for(uint8_t num=0; num<mvbTxDataTotalLength; num++)
				{
					printf("%02X ", spiTxBuffer[num]);
				}
				printf("\r\n");
			}
			#endif
			break;
		case HAL_TIMEOUT: debug("SPI Master TX2 Timeout\r\n"); break;
		case HAL_ERROR: debug("SPI Master TX2 Error\r\n"); break;
		default: break;
	}

}

void mvbSinkConfiguration(uint8_t Fcode, uint8_t PortNumber)
{
	HAL_StatusTypeDef retTXval=5;
	
	spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;			// 메모리 명령어, Write
	spiTxBuffer[1] = 0x20;													// LA PCS Offset Address (0x2000~0x27FF)
	spiTxBuffer[2] = PortNumber;												// PortNumber: 2 (MVB 테스트 할때 쓰는 포트번호)
	spiTxBuffer[3] = Fcode|MVB_SLAVE_SINK;									// F_Code(상위4bit), Sink
	spiTxBuffer[4] = 0x00;
	
	HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
		retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiTxBuffer, 5, 5000);
	HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
	switch(retTXval)
	{
		#ifdef MVB_PACKET_MONITORING
		case HAL_OK: debug("MVB SNK <-(Fcode %d, Port %02X)- TCMS SRC\r\n", Fcode>>4, PortNumber); break;
		#else
		case HAL_OK: break;
		#endif
		case HAL_TIMEOUT: debug("SPI5 TX Timeout for MVB Sink Configuration\r\n"); break;
		case HAL_ERROR: debug("SPI5 TX Error for MVB Sink Configuration\r\n"); break;
		default: break;
	}
}

uint8_t mvbRxData_check_flag=0;
void mvbSinkRead(uint8_t Fcode, uint8_t PortNumber)
{
	HAL_StatusTypeDef retTXval=5, retRXval=5;
	volatile static uint16_t WatchdogCount=0, PrevWatchdogCount=0, MatchCount=0;
	uint16_t tmpPortNumber=0;
    uint8_t mvbRxDataLength=0;
    RTCTIME rtctime;

 	/*
	 * Fcode에 따른 데이터 Size 결정
	 */
	switch(Fcode)
	{
		case MVB_FCODE0_2BYTE: mvbRxDataLength = 2; break;
		case MVB_FCODE1_4BYTE: mvbRxDataLength = 4; break;
		case MVB_FCODE2_8BYTE: mvbRxDataLength = 8; break;
		case MVB_FCODE3_16BYTE: mvbRxDataLength = 16; break;
		case MVB_FCODE4_32BYTE: mvbRxDataLength = 32; break;
		default: break;
	}
	
	/*
	 * MVB Read를 위해 SPI로 MVB 설정
	 */
	#if 1
	{
	/*
		 * Port Number를 <<2 한 후 Offset Address와 or 연산 해야 함(MVB Protocol)
		 */
		tmpPortNumber = (uint16_t)PortNumber<<2;								// 8byte인 경우 Port Number에서 최상위 bit가 1인 경우 <<2를 하면 overflow가 일어나므로 형변환 후 <<2 수행
		
		/*
		 ** 5 byte 송신 후
		 */
		spiRxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_READ;			// 메모리 명령어, Read
		spiRxBuffer[1] = (tmpPortNumber&0xFF00)>>8;								// LA Data Offset (0x0000~0x1FFFF)
		spiRxBuffer[2] = tmpPortNumber&0x00FF;
		spiRxBuffer[3] = 0x00;													// 더미데이터는 무조껀 2byte를 전송해야 함
		spiRxBuffer[4] = 0x00;													// 더미 데이터를 전송해야 클럭이 MVB로 공급되어 이후 동작이 수행됨
		
		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
		retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiRxBuffer, 5, 5000);
		switch(retTXval)
		{
			case HAL_OK: break;
			case HAL_TIMEOUT: debug("SPI Master TX1-2 Timeout\r\n"); break;
			case HAL_ERROR: debug("SPI Master TX1-2 Error\r\n"); break;
			default: break;
		}
	}
	#endif
	
	/*
	 * SPI를 통해 MVB에 저장된 데이터(TCMS가 전송한) Read
	 */
	#if 1
	{
	/*
		 ** Fcode에 해당하는 data length만큼 데이터 수신
		 */
		retRXval = HAL_SPI_Receive(&hspi5, (uint8_t*)spiRxBuffer, mvbRxDataLength, 1000);
		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
		switch(retRXval)
		{
			case HAL_OK: break;
			case HAL_TIMEOUT: debug("SPI Slave RX Timeout\r\n"); break;
			case HAL_ERROR: debug("SPI Slave RX Error\r\n"); break;
			default: break;
		}

		/*
		 * 패킷 수신 완료
		 */
		if(retRXval==HAL_OK)
		{
			/*
			 * 수신 패킷 확인
			 */
#ifdef MVB_RX_PACKET_MONITORING
			{
		   	switch(g_unDeviceID)
				{
					case DCU_R1_MCAR:
						if(PortNumber==DCU_R1_M_MVB_SOURSE_SD)
						{
							printf("# mvb Sink Read");
							printf("(source, 0x%02X) ", PortNumber);
							for(uint8_t num=0; num<mvbRxDataLength; num++)
								printf("%02X ", spiRxBuffer[num]);
							printf("\r\n");
						}	
						break;
					case DCU_L4_MCAR:
						if(PortNumber==DCU_L1_M_MVB_SOURSE_SD)
						{
							printf("# mvb Sink Read");
							printf("(source, 0x%02X) ", PortNumber);
							for(uint8_t num=0; num<mvbRxDataLength; num++)
								printf("%02X ", spiRxBuffer[num]);
							printf("\r\n");
						}	
						break;
					case DCU_R1_TCAR:
						if(PortNumber==DCU_R1_T_MVB_SOURSE_SD)
						{
							printf("# mvb Sink Read");
							printf("(source, 0x%02X) ", PortNumber);
							for(uint8_t num=0; num<mvbRxDataLength; num++)
								printf("%02X ", spiRxBuffer[num]);
							printf("\r\n");
						}	
						break;
					case DCU_L4_TCAR:
						if(PortNumber==DCU_L1_T_MVB_SOURSE_SD)
						{
							printf("# mvb Sink Read");
							printf("(source, 0x%02X) ", PortNumber);
							for(uint8_t num=0; num<mvbRxDataLength; num++)
								printf("%02X ", spiRxBuffer[num]);
							printf("\r\n");
						}	
						break;
					default: break;
				}
				
				/*
				if(PortNumber==DCU_ALL_MVB_SINK_COMMON)
				{
					printf("(COMMON, 0x%02X) ", PortNumber);
					for(uint8_t num=0; num<mvbRxDataLength; num++)
						printf("%02X ", spiRxBuffer[num]);
					printf("\r\n");
				}	
				else if(PortNumber==DCU_ALL_MVB_SINK_SDR)
				{
					printf("(SDR, 0x%02X) ", PortNumber);
					for(uint8_t num=0; num<mvbRxDataLength; num++)
						printf("%02X ", spiRxBuffer[num]);
					printf("\r\n");
				}	
				else if(PortNumber==DCU_M_MVB_SINK_TDR||PortNumber==DCU_T_MVB_SINK_TDR)
				{
					printf("(TDR, 0x%02X) ", PortNumber);
					for(uint8_t num=0; num<mvbRxDataLength; num++)
						printf("%02X ", spiRxBuffer[num]);
					printf("\r\n");
				}
*/
			}
			#endif
			
			/*
			 * Common Data 패킷 수신 시 시간 업데이트
			 */
			if(PortNumber==DCU_ALL_MVB_SINK_COMMON)
			{
				// rtc 시간 업데이트
				WatchdogCount = (uint16_t)spiRxBuffer[0];
				WatchdogCount |= ((uint16_t)spiRxBuffer[1]<<8);
				if(PrevWatchdogCount != WatchdogCount)
				{
					MatchCount++;
					if(MatchCount>6)
					{
						/* 이 hex값이 rs485로 전송하게 된다.*/
						rtc_time.ucYears = spiRxBuffer[2];
						rtc_time.ucMonth = spiRxBuffer[3];
						rtc_time.ucDate	 = spiRxBuffer[4];
						rtc_time.ucHours = spiRxBuffer[5];
						rtc_time.ucMinutes = spiRxBuffer[6];
						rtc_time.ucSeconds = spiRxBuffer[7];
						MatchCount=0;
						if(m_RtcDefine == SET_NONE)
						{
							rtctime = rtc_get_time();
							
							if((rtctime.ucYears == spiRxBuffer[2]) && (rtctime.ucMonth == spiRxBuffer[3]) && (rtctime.ucDate == spiRxBuffer[4]) &&
							  (rtctime.ucHours == spiRxBuffer[5]) && (rtctime.ucMinutes == spiRxBuffer[6]))			
							{	
								printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",
										((spiRxBuffer[2]>>4)*10)+(spiRxBuffer[2]&0x0f),
										((spiRxBuffer[3]>>4)*10)+(spiRxBuffer[3]&0x0f),
										((spiRxBuffer[4]>>4)*10)+(spiRxBuffer[4]&0x0f),
										((spiRxBuffer[5]>>4)*10)+(spiRxBuffer[5]&0x0f),
										((spiRxBuffer[6]>>4)*10)+(spiRxBuffer[6]&0x0f),
										((spiRxBuffer[7]>>4)*10)+(spiRxBuffer[7]&0x0f));
								//rtc_set_time(rtc_time.ucYears,rtc_time.ucMonth,rtc_time.ucDay,rtc_time.ucHours,rtc_time.ucMinutes,rtc_time.ucSeconds);
								m_RtcDefine = SET_OK;
							}	
							else
							{
								rtc_set_time(((spiRxBuffer[2]>>4)*10)+(spiRxBuffer[2]&0x0f),  //년도
											 ((spiRxBuffer[3]>>4)*10)+(spiRxBuffer[3]&0x0f),  //월
											 ((spiRxBuffer[4]>>4)*10)+(spiRxBuffer[4]&0x0f),  //일
											 ((spiRxBuffer[5]>>4)*10)+(spiRxBuffer[5]&0x0f),  //시
											 ((spiRxBuffer[6]>>4)*10)+(spiRxBuffer[6]&0x0f),  //분
											 ((0x00>>4)*10)+(0x00&0x0f)); 			   //초
							
								rtc_time = rtc_get_time();
								
								if((rtc_time.ucYears == spiRxBuffer[2]) && (rtc_time.ucMonth == spiRxBuffer[3]) && (rtc_time.ucDate == spiRxBuffer[4]) &&
								  (rtc_time.ucHours == spiRxBuffer[5])	&& (rtc_time.ucMinutes == spiRxBuffer[6]))			
								{	
									printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",
											((spiRxBuffer[2]>>4)*10)+(spiRxBuffer[2]&0x0f),
											((spiRxBuffer[3]>>4)*10)+(spiRxBuffer[3]&0x0f),
											((spiRxBuffer[4]>>4)*10)+(spiRxBuffer[4]&0x0f),
											((spiRxBuffer[5]>>4)*10)+(spiRxBuffer[5]&0x0f),
											((spiRxBuffer[6]>>4)*10)+(spiRxBuffer[6]&0x0f),
											((spiRxBuffer[7]>>4)*10)+(spiRxBuffer[7]&0x0f));
									//rtc_set_time(rtc_time.ucYears,rtc_time.ucMonth,rtc_time.ucDay,rtc_time.ucHours,rtc_time.ucMinutes,rtc_time.ucSeconds);
									m_RtcDefine = SET_OK;
								}	
								else
								{
									rtc_set_time(((spiRxBuffer[2]>>4)*10)+(spiRxBuffer[2]&0x0f),  //년도
												 ((spiRxBuffer[3]>>4)*10)+(spiRxBuffer[3]&0x0f),  //월
												 ((spiRxBuffer[4]>>4)*10)+(spiRxBuffer[4]&0x0f),  //일
												 ((spiRxBuffer[5]>>4)*10)+(spiRxBuffer[5]&0x0f),  //시
												 ((spiRxBuffer[6]>>4)*10)+(spiRxBuffer[6]&0x0f),  //분
												 ((0x00>>4)*10)+(0x00&0x0f)); 			   //초
								
									rtc_time = rtc_get_time();
									
									printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",
											((spiRxBuffer[2]>>4)*10)+(spiRxBuffer[2]&0x0f),
											((spiRxBuffer[3]>>4)*10)+(spiRxBuffer[3]&0x0f),
											((spiRxBuffer[4]>>4)*10)+(spiRxBuffer[4]&0x0f),
											((spiRxBuffer[5]>>4)*10)+(spiRxBuffer[5]&0x0f),
											((spiRxBuffer[6]>>4)*10)+(spiRxBuffer[6]&0x0f),
											((spiRxBuffer[7]>>4)*10)+(spiRxBuffer[7]&0x0f));
									m_RtcDefine = SET_OK;
								}
							}	
						}
					}
				}
				else
				{

					if(error_list_flag[DCU_MVB_ERROR] == true)
					{
						if(m_RtcDefine == SET_NONE)
						{
//							rtc_set_time(((0x19>>4)*10)+(0x19&0x0f),	//년도
//										 ((0x03>>4)*10)+(0x19&0x0f),	//월
//										 ((0x18>>4)*10)+(0x19&0x0f),  //일
//										 ((0x21>>4)*10)+(0x19&0x0f),  //시
//										 ((0x18>>4)*10)+(0x19&0x0f),  //분
//										 ((0x00>>4)*10)+(0x19&0x0f)); //초
							rtc_time = rtc_get_time();
							printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",
									rtc_time.ucYears,
									rtc_time.ucMonth,
									rtc_time.ucDate,
									rtc_time.ucHours,
									rtc_time.ucMinutes,
									rtc_time.ucSeconds);
							rtc_time.ucYears = (((rtc_time.ucYears/10)<<4) & 0x70) + ((rtc_time.ucYears%10) & 0x0f);
							rtc_time.ucMonth = (((rtc_time.ucMonth/10)<<4) & 0x70) + ((rtc_time.ucMonth%10) & 0x0f);
							rtc_time.ucDate	 = (((rtc_time.ucDate/10)<<4) & 0x70) + ((rtc_time.ucDate%10) & 0x0f);
							rtc_time.ucHours = (((rtc_time.ucHours/10)<<4) & 0x70) + ((rtc_time.ucHours%10) & 0x0f);
							rtc_time.ucMinutes = (((rtc_time.ucMinutes/10)<<4) & 0x70) + ((rtc_time.ucMinutes%10) & 0x0f);
							rtc_time.ucSeconds =(((rtc_time.ucSeconds/10)<<4) & 0x70) + ((rtc_time.ucSeconds%10) & 0x0f);
							m_RtcDefine = SET_OK;
						}
					}
				}
			}
			/*
			 * SDR 패킷 수신 시 Watchdog Update
			 */
			else if(PortNumber==DCU_ALL_MVB_SINK_SDR)
			{
				/*
				 * WatchdogCount 정보 업데이트 여부를 통한 MVB 통신 고장 체크
				 */
//				WatchdogCount = (uint16_t)spiRxBuffer[0];
//				WatchdogCount |= ((uint16_t)spiRxBuffer[1]<<8);
//				#ifdef MVB_RX_PACKET_MONITORING
//					debug("Watchdog Count: %d\r\n", WatchdogCount);
//				#endif
				if(PrevWatchdogCount != WatchdogCount)										// 업데이트 된  Watchdog Counter 수신 OK --> TCMS가 패킷을 제대로 보내고 있음
				{
					/*
					 * 처음 부팅 시 쓰레기 값으로 WatchdogCount가 이전값과 다르게 나오는 경우 바로 절체하면 안됨
					 */
					MatchCount++;
					if(MatchCount>6)
					{
						MatchCount=0;
						error_list_time[DCU_MVB_ERROR] = osKernelSysTick();
						#ifdef MVB_RX_PACKET_MONITORING
							debug("Watchdog Count 증가해서 MVB(F15) 고장삭제\r\n");
						#endif
						if(mvbRxData_check_flag == 1)
						{
								Erase_fault(DCU_MVB_ERROR);
								HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_SET);
								HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);
						}
						/*
						 * TCMS-MVB통신이 가능한 상태(Watchdog Update)에서만 TESTMODE 비트(절체명령) 수신 시 절체
						 */
						if((spiRxBuffer[2]&0x80)==0x80)
						{
							/* todo 19.03.11: mvb 카운트 하는거 추가 완성차 시험전 절체비트 수신 이벤트 기록하게끔 */
							spiRxBuffer[2] = 0;
							//if(m_isMasterSlaveChange)
	//							debug("TCMS->MVB로부터 젤체 비트 수신\r\n");
							m_isMasterSlaveChange = false;
						}
						else
						{
							//if(m_isMasterSlaveChange == false)
	//							debug("TCMS->MVB로부터 젤체 원복 비트 수신 \r\n");
							m_isMasterSlaveChange = true;
						}
					}
				}
				else																		// TCMS가 패킷을 보내고 있지 않아서 같은 패킷이 계속 읽힘
				{
					/*
					 * 100초 뒤에 mvb 통신 고장
					 */
					if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_MVB_ERROR]) > 100000)
					{
						FaultFlag_SET(DCU_MVB_ERROR);
						HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);

					}
					else if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_MVB_ERROR]) > 40000) && (get_diff_tick(osKernelSysTick(),error_list_time[DCU_MVB_ERROR]) < 50000))
//					else if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_MVB_ERROR]) > 20000) && (get_diff_tick(osKernelSysTick(),error_list_time[DCU_MVB_ERROR]) < 30000))
					{
						#if 1
						{
							switch(g_unDeviceID)
							{
								case DCU_R1_MCAR:
									mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD8);							// DCU R1 M --> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD8
									mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
									mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
									break;
								case DCU_L4_MCAR:
									mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD4);							// DCU L1 M	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD4
									mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
									mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
									break;
								case DCU_R1_TCAR:
									mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE8);							// DCU R1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE8
									mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
									mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
									break;
								case DCU_L4_TCAR:
									mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE4);							// DCU L1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE4
									mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
									mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
									break;
								default: debug("No MVB Source Configuration\r\n"); break;
							}
						}
						#endif
					}
					else if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_MVB_ERROR]) > 30000)
//					else if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_MVB_ERROR]) > 10000)
					{
//						FaultFlag_SET(DCU_MVB_ERROR);
						if(mvbRxData_check_flag == 0)
						{	
							mvbRxData_check_flag = 1;
							HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);
						} 
					}

					#ifdef MVB_RX_PACKET_MONITORING
					printf("# MVB-SDR WatchdogCnt Update Fail(F15) (Prev:%d,Curr:%d)\r\n",
								PrevWatchdogCount, WatchdogCount);
					#endif
				}
				PrevWatchdogCount = WatchdogCount;
			}
			/*
			 * TDR패킷 수신 시 
			 */
			else if(PortNumber==DCU_M_MVB_SINK_TDR)
			{
				
			}
			else if(PortNumber==DCU_T_MVB_SINK_TDR)
			{
				
			}
		}
		else
		{
			/*
			 * MVB통신 에러
			 */
			debug("# SPI(for MVB) Communication Error\r\n");
		}
	}
	#endif
}

void mc_TaskMVB(void const * argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    uint8_t loopcnt = 0;
    /*
     * 주기적으로 Master와 Slave 간 통신을 통한 alive check를 위해 둘 다 바로 실행되어야 함.
     */
	if(m_isMasterSlave == MASTER_DCU)
	{
//		osDelay(2000); //jeon_191007 2s dealy

		debug("6. [Master Start] MVB Task\r\n");
	    osDelay(500);
	}
	else // SLAVE_DCU
	{
    	while(m_isTaskExecution == false)
    	{
    		osDelay(200);
    	}
	    debug("6. [Slave Start] MVB Task\r\n");
	    PreviousWakeTime = osKernelSysTick();
	}
	
	/*
	 * MVB 설정
	 */
	#if 0
	{
		/*
		 * 명령어 기준 MVB Configuration
		 * 1. SDR(Status Data Request)
		 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
		 * 		DCU_R1_MCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
		 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
		 * 		DCU_R1_TCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
		 * 
		 * 2. SD(Status Data)
		 * 		DCU_L1_MCAR Source  --(Fcode 4, Port 0xD4)-->  TCMS Sink
		 * 		DCU_R1_MCAR Source  --(Fcode 4, Port 0xD8)-->  TCMS Sink
		 * 		DCU_L1_TCAR Source  --(Fcode 4, Port 0xE4)-->  TCMS Sink
		 * 		DCU_R1_TCAR Source  --(Fcode 4, Port 0xE8)-->  TCMS Sink
		 * 
		 * 3. TDR(Trace Data Request)
		 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xF8)--  TCMS Source
		 * 		DCU_R1_MCAR Sink    <--(Fcode 2, Port 0xF8)--  TCMS Source
		 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xFC)--  TCMS Source
		 * 		DCU_R1_TCAR Sink    <--(Fcode 2, Port 0xFC)--  TCMS Source
		 * 
		 * 4. TD(Trace Data)
		 * 		DCU_L1_MCAR Source  --(Fcode 4, Port 0xDC)-->  TCMS Sink
		 * 		DCU_R1_MCAR Source  --(Fcode 4, Port 0xE0)-->  TCMS Sink
		 * 		DCU_L1_TCAR Source  --(Fcode 4, Port 0xEC)-->  TCMS Sink
		 * 		DCU_R1_TCAR Source  --(Fcode 4, Port 0xF0)-->  TCMS Sink
		 */
		/*
		 * DCU 기준 MVB Configuration
		 * 1. DCU1_MVB
		 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
		 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xD4, SD )-->  TCMS Sink
		 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xF8, TDR)--  TCMS Source
		 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xDC, TD )-->  TCMS Sink
		 * 					
		 * 2. DCU2_MVB
		 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
		 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xD8, SD )-->  TCMS Sink
		 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xF8, TDR)--  TCMS Source
		 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xE0, TD )-->  TCMS Sink
		 * 
		 * 3. DCU3_MVB
		 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
		 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xE4, SD )-->  TCMS Sink
		 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xFC, TDR)--  TCMS Source
		 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xEC, TD )-->  TCMS Sink
		 * 		
		 * 4. DCU4_MVB
		 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
		 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xE8, SD )-->  TCMS Sink
		 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xFC, TDR)--  TCMS Source
		 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xF0, TD )-->  TCMS Sink
		 */
		switch(g_unDeviceID)
		{
			case DCU_R1_MCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD8);							// DCU R1 M --> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD8
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE0);							// DCU R1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xE0
				break;
			case DCU_L4_MCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD4);							// DCU L1 M	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD4
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xDC);							// DCU L1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xDC
				break;
			case DCU_R1_TCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
	    		mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE8);							// DCU R1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE8
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xF0);							// DCU R1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xF0
				break;
			case DCU_L4_TCAR:
				mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE4);							// DCU L1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE4
				mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
				mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xEC);							// DCU L1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xEC
				break;
			default: debug("No MVB Source Configuration\r\n"); break;
		}
	}
	#endif

    for(;;)
    {
		if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
		{
			osDelay(500);
			loopcnt++;
			if(loopcnt>2)
			{
				loopcnt=0;

			}
			continue;
		}
        /* Place this task in the blocked state until it is time to run again. */
#if 0 //jeon_190821
			osDelayUntil(&PreviousWakeTime, 511UL); //jeon_190716 orig:60UL
#else	        
	        osDelay(500);
#endif
 //       osDelay(500);
 /*       Motor voltage and current check - 2019.05.14
  * 70V : 1900~2300
  * 80V : 2300~2600
  * 90V : 2600~2900
  * 100V: 2900~3300
  * 110V: 3300~3600
        printf("\r\n");
       printf("(m_adc.MotorVoltage, %d) ", m_adc.MotorVoltage);
        printf("\r\n");
        printf("(m_adc.MotorCurrent, %d) ", m_adc.MotorCurrent);
        printf("\r\n");
        printf("(m_adc.PowerVoltage, %d) ", m_adc.PowerVoltage);
        printf("\r\n");
        printf("\r\n");
*/      
        
    	switch(g_unDeviceID)
		{
			case DCU_R1_MCAR:
		    	mvbSinkRead(MVB_FCODE3_16BYTE, DCU_ALL_MVB_SINK_COMMON);				// Port 0x2C
		    	osDelay(1);
		    	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_ALL_MVB_SINK_SDR);					// Port 0xD0
		    	osDelay(1);
				mvbSourceWrite(DCU_R1_M_MVB_SOURSE_SD);									// Port 0xD8
		    	osDelay(1);
				mvbSinkRead(MVB_FCODE4_32BYTE,DCU_R1_M_MVB_SOURSE_SD);									// Port 0xD8
		    	osDelay(1);
	        	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_M_MVB_SINK_TDR);						// Port 0xF8
		    	osDelay(1);
	    		mvbSourceWrite(DCU_R1_M_MVB_SOURSE_TD);									// Port 0xE0
		    	osDelay(1);
				break;
			case DCU_L4_MCAR:
		    	mvbSinkRead(MVB_FCODE3_16BYTE, DCU_ALL_MVB_SINK_COMMON);				// Port 0x2C
		    	osDelay(1);
		    	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_ALL_MVB_SINK_SDR);					// Port 0xD0
		    	osDelay(1);
				mvbSourceWrite(DCU_L1_M_MVB_SOURSE_SD);									// Port 0xD4
		    	osDelay(1);
				mvbSinkRead(MVB_FCODE4_32BYTE,DCU_L1_M_MVB_SOURSE_SD);									// Port 0xD8
		    	osDelay(1);
	        	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_M_MVB_SINK_TDR);						// Port 0xF8
		    	osDelay(1);
	    		mvbSourceWrite(DCU_L1_M_MVB_SOURSE_TD);									// Port 0xDC
		    	osDelay(1);
				break;
			case DCU_R1_TCAR:
		    	mvbSinkRead(MVB_FCODE3_16BYTE, DCU_ALL_MVB_SINK_COMMON);				// Port 0x2C
		    	osDelay(1);
		    	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_ALL_MVB_SINK_SDR);					// Port 0xD0
		    	osDelay(1);
				mvbSourceWrite(DCU_R1_T_MVB_SOURSE_SD);									// Port 0xE8
		    	osDelay(1);
				mvbSinkRead(MVB_FCODE4_32BYTE,DCU_R1_T_MVB_SOURSE_SD);									// Port 0xD8
		    	osDelay(1);
	        	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_T_MVB_SINK_TDR);						// Port 0xFC
		    	osDelay(1);
	    		mvbSourceWrite(DCU_R1_T_MVB_SOURSE_TD);									// Port 0xF0
		    	osDelay(1);
				break;
			case DCU_L4_TCAR:
		    	mvbSinkRead(MVB_FCODE3_16BYTE, DCU_ALL_MVB_SINK_COMMON);				// Port 0x2C
		    	osDelay(1);
		    	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_ALL_MVB_SINK_SDR);					// Port 0xD0
		    	osDelay(1);
				mvbSourceWrite(DCU_L1_T_MVB_SOURSE_SD);									// Port 0xE4
		    	osDelay(1);
				mvbSinkRead(MVB_FCODE4_32BYTE,DCU_L1_T_MVB_SOURSE_SD);									// Port 0xD8
		    	osDelay(1);
	        	mvbSinkRead(MVB_FCODE2_8BYTE, DCU_T_MVB_SINK_TDR);						// Port 0xFC
		    	osDelay(1);
	    		mvbSourceWrite(DCU_L1_T_MVB_SOURSE_TD);									// Port 0xEC
		    	osDelay(1);
				break;
			default: break;
		}
		#ifdef MVB_PACKET_MONITORING
			debug("\r\n");
		#endif
    }
}

void mac_TaskAliveCheck(void const * argument)
{
	osEvent event;
    uint32_t PreviousWakeTime = osKernelSysTick();
	volatile uint8_t ucSendData[8] = {0x00,0x00,0x00,0X00,0x00,0x00,0x00,0x00};
	uint8_t PacketCount=0, ucTemp=0, CheckSumHigh=0, CheckSumLow=0, i;
	uint8_t NoResponseCnt=0;
	uint16_t CheckSumTxPacket=0, CheckSumRxPacket=0;
	//SlaveRunning_Check(event);
    debug("7. Start Master-Slave AliveCheck Task\r\n");

    for(;;)
    {
        /*
         * MASTER DCU 실행코드
         */
        if(m_isMasterSlave == MASTER_DCU)
        {
            /* Place this task in the blocked state until it is time to run again. */
            osDelayUntil(&PreviousWakeTime, 50UL);
            
            /*
             * Slave로 패킷 송신(절체명령 정보 포함)
             */
			#if 1
			{
				/*
				 * Slave로 전송 할 패킷 정의
				 *  [0]      [1]       [2]        [3]        [4] [5]   [6] [7]
				 * 0xCC   IndexCnt   도어상태    isMasterDie     고장정보     CheckSum
				 */
				ucSendData[0] = 0xAA;
				ucSendData[1] = PacketCount++;
				ucSendData[2] = mdc_DoorState;
				ucSendData[3] = 0x0;//(uint8_t)m_isSlaveRunCommand;												// 통신로직에서 Slave로 'm_isSlaveRunCommand=true' 정보 전송
				ucSendData[4] = 0x0;
				ucSendData[5] = 0x0;
				CheckSumTxPacket = ucSendData[1]+ucSendData[2]+ucSendData[3]+ucSendData[4]+ucSendData[5];
				ucSendData[6] = (uint8_t)((CheckSumTxPacket&0xFF00)>>8);
				ucSendData[7] = (uint8_t)(CheckSumTxPacket&0x00FF);
				
				/*
				 * Slave로 8byte 패킷 전송
				 */
				if((m_isPacketSendToSlave == true) && (m_is485changeflag == true))																	// 절체 테스트 용
				{
					if(HAL_UART_Transmit(&huart5, (uint8_t*)ucSendData, 8, 2000)!= HAL_OK)					// 7byte 데이터 전송 (송신은 큐를 사용하지 않음)
					{
						debug("## [alive] Master Alive Packet Transmit Error\r\n");
					}
					else
					{
						#ifdef ALIVE_PACKET_MONITORING
							debug("## [alive] Master -> Slave: ");
							for(i=0; i<8; i++)	debug("%02X ", ucSendData[i]);
							debug("\r\n");
						#endif
					}
				}
			}
			#endif
        	
			/*
			 * 패킷 송신 후 Slave가 보낸 패킷 바로 받을 대기하고 있음
			 */
			#if 1
			{
				HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);
				osDelay(3);
				/*
				 * 절체 후 Slave로부터 SlaveRun 정보 수신 시 Master LED Off
				 */
				event = osMessageGet(hRx5Queue, 100);														// 정해진 시간 동안 Slave가 보내는 메시지 수신 대기
				if(event.status == osEventMessage)															// Slave로부터 메시지 수신 성공
				{
					/*
					 * 패킷은 수신했으므로 카운트 값 0으로 리셋
					 */
					NoResponseCnt = 0;
					error_list_flag[DCU_CANT_SLAVE] = false;												// 어쨌꺼나 패킷은 수신했으니까 Slave는 살아 있다고 봐야 함
					#ifdef ALIVE_PACKET_MONITORING
						debug("## [alive] Master <- Slave: ");
						for(i=0; i<8; i++)	debug("%02X ", aUartRx5Buffer[i]);
						debug("\r\n");
					#endif
					if(aUartRx5Buffer[0]==0xBB)
					{
						CheckSumRxPacket = aUartRx5Buffer[1]+aUartRx5Buffer[2]+aUartRx5Buffer[3]+aUartRx5Buffer[4]+aUartRx5Buffer[5];
						CheckSumHigh = (uint8_t)((CheckSumRxPacket & 0xFF00)>>8);
						CheckSumLow = (uint8_t)(CheckSumRxPacket & 0x00FF);
						
						/*
						 * Slave가 송신한 8byte 패킷 정상적으로 수신
						 */
						if((CheckSumHigh==aUartRx5Buffer[6]) && (CheckSumLow==aUartRx5Buffer[7]))
						{
							if(aUartRx5Buffer[3]==true)														// Slave가 절체했다고 알려주면
							{
								m_isSlaveRunCommand = true;
								m_isPacketSendToSlave = true;												// 슬레이브에서 절체했으므로 마스터는 다시 패킷을 보낸다.
								mip_MasterCodeMasterLED(0);													// Master는 실행을 중지 함
								mip_SleveCodeSlaveRun(1);													// 릴레이 fnd등 모든 제어권을 슬레이브로 넘긴다.
								//printf("loop in 2 \r\n");
							}
						}
						else
						{
							debug("## [alive] 비정상 Slave패킷 수신\r\n");
							HAL_UART_Init(&huart5);
						}
					}
					else
					{
						HAL_UART_Init(&huart5);
						//쓰레기 값 처리
						osDelay(1);
						//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx5Buffer, 11,5);
					}
				}
				else																						// Slave로부터 메시지 수신 실패
				{
					NoResponseCnt++;
					if(NoResponseCnt>60) //jeon_190715 orig:10 0.5s => 3s
					{
						debug("## [alive] Slave패킷 미수신 (Slave Die)\r\n");
						FaultFlag_SET(DCU_CANT_SLAVE);
						NoResponseCnt = 0;
					}
					else
					{
						HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);
					}
				}
			}
			#endif
        }
        /*
         * SLAVE DCU 실행코드
         */
        else
        {
            /* Place this task in the blocked state until it is time to run again. */
            osDelayUntil(&PreviousWakeTime, 50UL);
            
            /*
             * Master가 보낸 패킷 Slave수신 
             */
			#if 1
			{
				//여기 놔둬야 슬레이브 CPU 안뻗음 이유 분석해야함
            	HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);
            	osDelay(3);
				/*
				 * Master로부터 8byte 패킷 수신하면 Rx5 수신 인터럽트에서 메시지 Put
				 */
				event = osMessageGet(hRx5Queue, 100);
				if(event.status == osEventMessage)												// Master로부터 8byte를 다 받은경우 이벤트 발생
				{
					/*
					 * 패킷은 수신했으므로 카운트 값 0으로 리셋
					 */
					NoResponseCnt = 0;
					
					#ifdef ALIVE_PACKET_MONITORING
						debug("## Alive Master -> Slave: ");
						for(i=0; i<8; i++)	debug("%02X ", aUartRx5Buffer[i]);
						debug("\r\n");
					#endif
					
					if(aUartRx5Buffer[0]==0xAA)
					{
						CheckSumRxPacket = aUartRx5Buffer[1]+aUartRx5Buffer[2]+aUartRx5Buffer[3]+aUartRx5Buffer[4]+aUartRx5Buffer[5];
						CheckSumHigh = (uint8_t)((CheckSumRxPacket & 0xFF00)>>8);
						CheckSumLow = (uint8_t)(CheckSumRxPacket & 0x00FF);
						
						/*
						 * Master가 송신한 8byte 패킷 정상적으로 수신
						 */
						if((CheckSumHigh==aUartRx5Buffer[6]) && (CheckSumLow==aUartRx5Buffer[7]))
						{
							mdc_InitDoorState = aUartRx5Buffer[2];									// Master가 보낸 도어 상태정보 Slave로 저장
							if(m_isSlaveRunCommand == false) m_isSlaveRunCommand = aUartRx5Buffer[3];							// Master가 보낸 'm_isSlaveRunCommand=true' 정보 수신 -> Slave 동작 시작

						}
						else
						{
							HAL_UART_Init(&huart5);
							debug("## [alive] 비정상 Master패킷 수신\r\n");
						}
					}
					else
					{
						//쓰레기 값 처리
						osDelay(1);
						HAL_UART_Init(&huart5);
					}
				}
				else																			// Master로부터 메시지 수신 실패
				{
					NoResponseCnt++;
					#if 1
					if(NoResponseCnt>20)	//jeon_190715	orig:20 1s => 2s 													// 마스터에서 2초 정도 응답이 없을 경우 바로 슬레이브가 가져온다.
					#else
					if(NoResponseCnt>500)														// 다운로드할 때 자꾸 절체 되서 테스트 시에만 시간 늘려 놓음
					#endif
					{
						//debug("## [alive] Master패킷 미수신 (Master Die)\r\n");
						m_isSlaveRunCommand = true;												// Master로부터 패킷 미수신 시 Slave가 스스로 젤체명령 셋 -> DecisionControl에서 젤체 수행
						NoResponseCnt = 0;
						//mip_SleveCodeSlaveRun(1);
						HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);
					}
					else
					{

					}

				}
			}
			#endif
        	
			/*
			 * Master로 패킷 송신(절체명령 정보 포함)
			 */
			#if 1
			{
				/*
				 * Master로 전송 할 패킷 정의
				 */
				ucSendData[0] = 0xBB;
	            ucSendData[1] = PacketCount++;
	        	ucSendData[2] = 0x0;
	        	ucSendData[3] = (uint8_t)m_isSlaveRunCommand;												// 통신로직에서 Slave로 'm_isSlaveRunCommand=true' 정보 전송
	        	ucSendData[4] = 0x0;
	        	ucSendData[5] = 0x0;
	        	CheckSumTxPacket = ucSendData[1]+ucSendData[2]+ucSendData[3]+ucSendData[4]+ucSendData[5];
	        	ucSendData[6] = (uint8_t)((CheckSumTxPacket&0xFF00)>>8);
	        	ucSendData[7] = (uint8_t)(CheckSumTxPacket&0x00FF);
	        	
	        	/*
	        	 * Master로 8byte 패킷 전송
	        	 */
	        	if(HAL_UART_Transmit(&huart5, (uint8_t*)ucSendData, 8, 2000)!= HAL_OK)		// 7byte 데이터 전송 (송신은 큐를 사용하지 않음)
	        	{
					debug("## [alive] Master Alive Packet Transmit Error\r\n");
	        	}
	        	else
	        	{
					#ifdef ALIVE_PACKET_MONITORING
						debug("## [alive] Master <- Slave: ");
						for(i=0; i<8; i++)	debug("%02X ", ucSendData[i]);
						debug("\r\n");
					#endif
	        	}
			}
			#endif
        }
    }
}

static uint16_t CalcCrc(unsigned char* pbyMsg, int IngNumBytes)
{
    unsigned int IngCrc=0xFFFF;
    unsigned int IngRes=0;
    
	int i=0,j=0;
	
	for(i=0; i<IngNumBytes; i++)
	{
		/*
		 * todo : 형변환 안하고 바로 int형 변수에 저장하는데 정적분석 할 때 괜찮은지 고려해 봐야 함
		 */
		IngCrc ^= *(pbyMsg+i);

		for(j=0; j<8; j++)
		{
			if(IngCrc & 1) IngCrc ^= 0x14002;
			IngCrc >>= 1;
		}
	}

	IngRes=(IngCrc & 0xff) << 8;
	IngRes=IngRes | ((IngCrc >> 8) & 0xff);
    return (unsigned short)(IngRes);
}

static void Comm_data_tx(uint32_t port, int8_t* rx_buf, uint16_t rx_size)
{
	uint16_t tx_crc=0, tmptx_crc=0;
	uint8_t tx_index=6;
	int8_t tx_buf[128]={0}, i=0;
	
	tx_buf[0] = 0x01; //temp
	tx_buf[1] = 0x70; //temp
	tx_buf[2] = 0x00; //temp
	tx_buf[3] = 0x00; //temp
	tx_buf[4] = 53; //temp
	tx_buf[5] = 0x00; //temp
	// 	save_data();
	for(i=0; i<53; i++)
	{
		tx_buf[6+i]=data_buf[i];
	}
	
	tx_crc = CalcCrc(tx_buf, 59);
	//tx_buf[59] = (tx_crc & 0xff00) >> 8;
	tmptx_crc = (tx_crc >> 8) & 0x00ffu;
	tx_buf[59] = (int8_t)tmptx_crc;
	//tx_buf[60] = (tx_crc & 0x00ffu);
	tx_buf[60] = (int8_t)tx_crc;
	
	if(port==0u)
	{
		for(i=0; i<61; i++)
		{
			#ifdef HW_DEPENDENT_CODE
//				xSerialPutChar(0, tx_buf[i], 10);
			#endif
		}
	}
	else if(port==3u)
	{
		for(i=0; i<61; i++)
		{
			#ifdef HW_DEPENDENT_CODE
//				xSerialPutChar(3, tx_buf[i], 10);
			#endif
		}
	}
}

static void CommandExcute(int8_t *String)
{
	int8_t *pCommand = NULL;
	int8_t *pParameter = NULL;
	static void (*pf[])(int8_t* string, uint8_t ucNumOfParameters) = {
					callback_version,										// ShowVersion
					callback_test_state,									// adc
					callback_test_motor_pwm,								// motor
					callback_test_dcu,										// dcu
					callback_test_door_open,								// open
					callback_test_door_free,								// free
					callback_test_Switch,									// switch
					callback_test_fault,									// fault
					callback_test_reset,									// reset
					callback_test_mram,	 								// mram
					callback_test_obs,
					callback_test_debug,
					callback_test_DCUID,
					callback_test_RTCSET};
	int8_t *command_list[14];
	uint8_t isCommand=0;
	
	command_list[0] = (int8_t *)"ShowVersion";
	command_list[1] = (int8_t *)"state";
	command_list[2] = (int8_t *)"motor";
	command_list[3] = (int8_t *)"dcu";
//	command_list[4] = (int8_t *)"open";
	command_list[4] = (int8_t *)"oo";
	command_list[5] = (int8_t *)"free";
	command_list[6] = (int8_t *)"sw";
	command_list[7] = (int8_t *)"fault";
	command_list[8] = (int8_t *)"reset";
	command_list[9] = (int8_t *)"mram";
	command_list[10] = (int8_t *)"obs";
	command_list[11] = (int8_t *)"debug";
	command_list[12] = (int8_t *)"dcuid";
	command_list[13] = (int8_t *)"rtc";
	pCommand = strtok(String," ");
	pParameter = &String[strlen(pCommand)+1u];
	
	for(int i=0; i<14; i++)
	{
		if(strstr(pCommand,command_list[i]) != NULL)
		{
			pf[i](pParameter, 0);
			isCommand = 1;
		}
	}
	if(isCommand == 0)
		debug("Wrong command, Try again!\r\n");
}

static void isReceivePacket(void)
{
	uint16_t cnt_rcv,i;
	static uint8_t ucOffset = 0;
	static uint8_t ucTemp = 0;
	static int8_t ucString[128] = {0};
	uint8_t tx_buf[128] = {0};

	cnt_rcv = Len_queue(&rx_Int_queue);

	if(cnt_rcv == 1)									//콘솔로 명령어 치는 경우
	{
		if(ucOffset > 126) ucOffset = 0;
		ucTemp = Dequeue(&rx_Int_queue);
		ucString[ucOffset] = ucTemp;

		if((ucTemp == 0x0D) || (ucTemp == 0x0A))	// 0x0D(Carriage Return), 0x0A(Line Feed)
		{
			/*
			 * 글자 입력 후 엔터키 누름
			 */
			if(ucOffset != 0u)
			{
				printf("\r\n\r\n");
				CommandExcute(ucString);
				printf("\r\ndcu-test $ \r\n");
				fflush(stdout);
			}
			/*
			 * 글자 입력 없이 엔터키 누름
			 */
			else
			{
				printf("\r\ndcu-test $ ");
				fflush(stdout);
			}
			ucOffset = 0u;
			memset(ucString,0,128);
		}
		/*
		 * BackSpace를 누른 경우 한글자 삭제
		 */
		else if(ucTemp == 0x08)
		{
			if(ucOffset != 0u)
			{
				ucString[ucOffset] = 0;
				ucOffset--;
				debug("\b \b");
				fflush(stdout);
			}
		}
		else
		{
			debug("%c",ucTemp);
			fflush(stdout);
			ucOffset++;
		}
	}
	else																		//유지 보수 프로그램으로 통신하는 경우
	{
		for(i=0; i<cnt_rcv; i++)
		{
			tx_buf[i] = Dequeue(&rx_Int_queue);
		}
		if((m_isSlaveRunCommand == true) && (m_isMasterSlave == MASTER_DCU))
		{
			return;
		}
		else if((m_isSlaveRunCommand == false) && (m_isMasterSlave == SLAVE_DCU)) //제어권이 없는 경우 유지보수프로그램과 통신하지 못하게 함
		{
			return;
		}
		if((tx_buf[0] == 0x01) && (tx_buf[1] == 0x70))							//PC -> DCU 상태 정보 요청
		{
			/*
			 * DCU 에서도 CRC 체크를 해야함
			 * PC에서 보내주는 마지막 2바이트(CRC 1/2)는 FIX값으로 나옴
			 * 그러므로 DCU는 마지막 2BYTE를 제외한 모든 데이터를 CRC 체크를 한 뒤에
			 * PC에서 보내준 crc 2바이트와 비교후 맞으면 상태정보들을 전송한다.
			 */
			uint16_t rx_crc = 0;
			uint8_t rx_buf[2] = {0,0};
			rx_crc = CalcCrc(tx_buf,(int32_t)(cnt_rcv-2));						//crc 2바이트를 제외하고 받은데이터들 crc 체크한다
			rx_buf[0] = (uint8_t)(((rx_crc) & 0xFF00) >> 8);
			rx_buf[1] = (uint8_t)((rx_crc) & 0x00FF);
			/*
			 * 총 8개의 데이터가 들어오니까 cnt_rcv값은 8이다. 6,7번째 배열과 crc 체크를 해야한다.
			 * ex ) 01 70 01 03 04 01 / 02 03     : pc 에서 보내는 값 뒤의 2바이트는 crc
			 *      01 70 02 00 01 01 : dcu가 노이즈 영향으로 이상한 값을 받으면 crc 체크 할때 해당 값은 틀린 값을 받게 된다.
			 */
			if((rx_buf[0] == tx_buf[(cnt_rcv-2)]) && (rx_buf[1] == tx_buf[(cnt_rcv-1)]))
			{
				StateData_Send(tx_buf);
			}
		}
		else if((tx_buf[0] == 0x01) && (tx_buf[1] == 0x80))						//pc -> DCU 열림 명령,고장삭제등 메모리 쓰기 동작
		{
			uint16_t rx_size = 0;
			uint16_t rx_crc = 0;
			uint8_t rx_buf[2] = {0,0};
			rx_crc = CalcCrc(tx_buf,(int32_t)(cnt_rcv-2));						//crc 2바이트를 제외하고 받은데이터들 crc 체크한다
			rx_buf[0] = (uint8_t)(((rx_crc) & 0xFF00) >> 8);
			rx_buf[1] = (uint8_t)((rx_crc) & 0x00FF);
			if((rx_buf[0] == tx_buf[cnt_rcv-2]) && (rx_buf[1] == tx_buf[cnt_rcv-1]))
			{
				rx_size |= (uint16_t)(tx_buf[5] & 0x00ff) << 8;
				rx_size |= (uint16_t)tx_buf[4];
				MemHandling_data(tx_buf,rx_size);
			}
		}
		else if((tx_buf[0] == 0x01) && (tx_buf[1] == 0xFF))					   // pc <-> DCU 고장 정보
		{
			uint16_t rx_crc = 0;
			uint8_t rx_buf[2] = {0,0};
			rx_crc = CalcCrc(tx_buf,(int32_t)(cnt_rcv-2));
			rx_buf[0] = (uint8_t)(((rx_crc) & 0xFF00) >> 8);
			rx_buf[1] = (uint8_t)((rx_crc) & 0x00FF);
			if((rx_buf[0] == tx_buf[cnt_rcv-2]) && (rx_buf[1] == tx_buf[cnt_rcv-1]))
			{
				if(tx_buf[3] == 0x01)									//데이터 전송 명령어
				{
					Comm_Fault_tx(0);
				}
				else if(tx_buf[3] == 0x03)								//수신 확인 명령어
				{
					uint16_t Block_Number =0;
					Block_Number = tx_buf[4] +1;						// 다음 블럭 정보들을 전송 한다.
					Comm_Fault_tx(Block_Number);
				}
			}
		}
	}
}

#if 0
void mac_TaskRS485(void const * argument)
{
	osEvent event;
	uint8_t SlaveDcuAddress[7] = {DCU_L2_MCAR,DCU_L3_MCAR,DCU_L4_MCAR,DCU_R1_MCAR,DCU_R2_MCAR,DCU_R3_MCAR,DCU_R4_MCAR};	//slave dcu들의 도어 ID들을 저장해놓는다.
	//uint8_t SlaveDcuAddress[7] = {DCU_L2_MCAR,DCU_L3_MCAR,DCU_L4_MCAR,DCU_L2_MCAR,DCU_L3_MCAR,DCU_L4_MCAR,DCU_L2_MCAR};
	uint8_t aTxBuffer[11] = {0,};
	uint8_t aRxBuffer[11] = {0,};
	uint8_t Packet_Number_count = 0;
	uint16_t checksum_value = 0;
	uint8_t Slave_up_data = 0;
	uint8_t Slave_down_data = 0;
	uint8_t Master_up_data = 0;
	uint8_t Master_down_data = 0;
	HAL_StatusTypeDef retval=0;
	uint8_t address_count = 0;
	uint16_t  loopcnt=0;
	uint8_t tx_result = 0;
	aTxBuffer[0] = 0xCC;					// start packet
	aTxBuffer[1] = 0x00;					// 도어 정보
	aTxBuffer[2] = Packet_Number_count;		// 4패킷 넘버 카운트
	aTxBuffer[3] = 0x01;					// 체크섬 high
	aTxBuffer[4] = 0x00;					// 체크섬 low
	debug("## Start RS485 Task\r\n");
	
	if(m_isMasterSlave == MASTER_DCU)
	{
	    debug("5. [Master Start] Monitoring Task\r\n");
	}
	else // SLAVE_DCU
	{
    	while(!m_isSlaveRunCommand)
    	{
    		osDelay(200);
    		debug("waiting..\r\n");
    	}
	    debug("5. [Slave Start] rs485 Task\r\n");
	}
	
	for(;;)
	{

		/*
		 * RS485_Master_DCU_L1인 경우 RS485_Slave_DCU_L2~L4에게 정보를 요청
		 */
		/*
		 * RS485_Master_DCU_L1은 RS485_Slave_DCU_L2 RS485_Slave_DCU_L3 RS485_Slave_DCU_L4의 정보들 취합
		 */
		if(g_unDeviceID == DCU_L1_MCAR)
		{
			if(address_count > 6) address_count = 0;
			aTxBuffer[0] = 0xCC;											//start packet
			aTxBuffer[1] = SlaveDcuAddress[address_count];					// 요청할 slave의 id 정보
			aTxBuffer[2] = Packet_Number_count;								// 0~255 카운트 하는 값을 전송
			checksum_value = (aTxBuffer[0] + aTxBuffer[1] + aTxBuffer[2]);	//  [0] , [1] , [2] 를 더한 값을 저장
			aTxBuffer[3] = ((checksum_value & 0xFF00) >>8);					//  상위 바이트를 3 바이트에 저장
			aTxBuffer[4] = (checksum_value & 0x00FF);						//  하위 바이트를 4 바이트에 저장 시켜 전송
			
			#if 0
				debug("send :");
				for(int i =0; i<11; i++)
				{
					debug("%02x ",aTxBuffer[i]);
				}
				debug("\r\n");
			#endif
			for(int i=0; i<3; i++)
			{
				tx_result = HAL_UART_Transmit(&huart1, (uint8_t*)aTxBuffer, 5, 2000);
				osDelay(10);
			}
			if(tx_result != HAL_OK)
			{
				debug("## Transmit Error(%d)\r\n", retval);
			}
			else
			{
				event = osMessageGet(hRx1Queue, 50);
				if(event.status == osEventMessage)
				{
					if(aUartRx1Buffer[0] == 0xCC)
					{
						/* CC + 도어정보 + CHECK SUM HIGH + CHECK SUM LOW */
						checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
						Master_up_data = ((checksum_value & 0xFF00) >> 8);
						Master_down_data =(checksum_value & 0x00FF);
						if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //체크섬 이 맞으면
						{
							if((aUartRx1Buffer[1] == DCU_L2_MCAR) || (aUartRx1Buffer[1] == DCU_L3_MCAR) || (aUartRx1Buffer[1] == DCU_L4_MCAR))
							{
								//이부분이 mvb로 보내는 데이터를 취급할 곳
								#if 0
								debug("data :");
								for(int i =0; i<11; i++)
								{
									debug("%02x ",aUartRx1Buffer[i]);
								}
								debug("\r\n");
								#endif
								mvbSaveDataDCUL(aUartRx1Buffer[1]);
							}
						}
					}
				}
				else
				{
					// L2 L3 L4 의 데이터만 받을수 있도록 대기한다.
					if((aTxBuffer[1] == DCU_L2_MCAR) || (aTxBuffer[1] == DCU_L3_MCAR) || (aTxBuffer[1] == DCU_L4_MCAR))
					{
						//11byte의 데이터가 올때 까지 대기 한다
						while(HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11) != HAL_OK)
						{

							loopcnt++;
							if(loopcnt >7)
							{

								//debug("master test \r\n");
								loopcnt = 0;
								break;
							}
							vTaskDelay(10);									// 11byte 응답이 오지 않았으면 break
						}
					}

				}
//				HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
				address_count++;
			}
			Packet_Number_count++;
		}
		else if(g_unDeviceID == DCU_R1_MCAR)	//R1의 DCU라면 버스에 떠 있는 R2 R3 R4의 정보들을 취합한다.
		{
			while(HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11) != HAL_OK)
			{
				osDelay(10);
			}
			
			event = osMessageGet(hRx1Queue, 10);
			if(event.status == osEventMessage)
			{
				if(aUartRx1Buffer[0] == 0xCC)
				{
					if(aUartRx1Buffer[0] == 0xCC)
					{
						/* CC + 도어정보 + CHECK SUM HIGH + CHECK SUM LOW */
						checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
						Master_up_data = ((checksum_value & 0xFF00) >> 8);
						Master_down_data =(checksum_value & 0x00FF);
						if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //체크섬 이 맞으면
						{
							if((aUartRx1Buffer[1] == DCU_R2_MCAR) || (aUartRx1Buffer[1] == DCU_R3_MCAR) || (aUartRx1Buffer[1] == DCU_R4_MCAR))
							{
								//이부분이 mvb로 보내는 데이터를 취급할 곳
								#if 0
								debug("data :");
								for(int i =0; i<11; i++)
								{
									debug("%02x ",aUartRx1Buffer[i]);
								}
								debug("\r\n");
								#endif
								mvbSaveDataDCUR(aUartRx1Buffer[1]);
							}
						}
					}
				}
				//debug(" result : %02x \r\n ",osPoolFree(hRx1Queue,aUartRx1Buffer));
			}
		}
		else if(g_unDeviceID != DCU_L1_MCAR)	//slave dcu 라면 자신의 id 요청이 들어왔을 경우 정보를 보낸다.
		{
			while(HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11) != HAL_OK)
			{
				osDelay(10);
			}
			
			event = osMessageGet(hRx1Queue, 10);
			if(event.status == osEventMessage)
			{
				if(aUartRx1Buffer[0] == 0xCC)
				{
					checksum_value = (aUartRx1Buffer[0] + aUartRx1Buffer[1] + aUartRx1Buffer[2]);
					Slave_up_data = ((checksum_value & 0xFF00)>>8);
					Slave_down_data = (checksum_value & 0x00FF);
					if((Slave_up_data == aUartRx1Buffer[3]) && (Slave_down_data == aUartRx1Buffer[4]))
					{
						if(aUartRx1Buffer[1] == g_unDeviceID) // 마스터에서 자신을 요청한 경우
						{
							aTxBuffer[0] = 0xCC;
							aTxBuffer[1] = g_unDeviceID;
							aTxBuffer[2] = Statement_DCU(2);	 // 상태 정보
							aTxBuffer[3] = Statement_DCU(3);	 // 상태 정보 2
							aTxBuffer[4] = FalutStatement_DCU(4);// 고장 정보
							aTxBuffer[5] = FalutStatement_DCU(5);// 고장 정보 2
							aTxBuffer[6] = 0x36;				 // 소프트웨어 버전
							aTxBuffer[7] = 0x97;				 // 고장 갯수
							aTxBuffer[8] = 0x55;				 // 닫힘 시간
							checksum_value = (aTxBuffer[0]+aTxBuffer[1]+aTxBuffer[2]+aTxBuffer[3]);
							Slave_up_data = ((checksum_value & 0xFF00)>>8);
							Slave_down_data = (checksum_value & 0x00FF);
							aTxBuffer[9] = Slave_up_data;
							aTxBuffer[10] = Slave_down_data;
							for(int i =0; i<3; i++)
							{
								if(HAL_UART_Transmit(&huart1, (uint8_t*)aTxBuffer, 11, 2000) != HAL_OK)
								{
									debug("transmit error \r\n");
								}
								else
								{
	//								debug("slave transmit ok \r\n");
	//								HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
								}
							}

						}
					}
					else
					{
//						HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
					}
				}
//				debug("event in \r\n");
//				debug("data :");
//				for(int i =0; i<11; i++)
//				{
//					debug("%02x ",aTxBuffer[i]);
//				}
//				debug("\r\n");
				
				//debug(" result : %02x \r\n ",osPoolFree(hRx1Queue,aUartRx1Buffer));
			}
//			else
//			{
//				if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11) == HAL_BUSY)
//				{
//					debug("%d slave fail / %02x \r\n", loopcnt++,event.status);
//				}
//			}
		}
		vTaskDelay(50);
	}
}
#endif

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtCommunication.c
*********************************************************************/
