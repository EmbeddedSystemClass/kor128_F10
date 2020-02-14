/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtCommunication
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtCommunication.c
*********************************************************************/

#include "mtCommunication.h"
#include "mMotorOut.h"
/* �������Ȯ���� ���� ��Ŭ��� */
/*## dependency mtDecisionControl */
#include "mtDecisionControl.h"
/* ���ڴ� �޽��� Ȯ���� ���� ��Ŭ��� */
/*## dependency mtMotorFeedback */
#include "mtMotorFeedback.h"
/* ��ֹ���������, �������������� CLI�� ���� �����ϱ� ���� ��Ŭ��� */
/*## dependency mtObstacleDetect */
#include "mtObstacleDetect.h"
#include "string.h"
#include "mtInputProcessing.h"
#include "stm32f7xx_hal_uart.h"
#include "cmsis_os.h"
#include "mtMonitoring.h"
#include "ring_buffer.h"

//#define MVB_PACKET_MONITORING						// Ȱ��ȭ �� MVB ��/���� ��Ŷ ����͸� ����
//#define MVB_RX_PACKET_MONITORING						// Ȱ��ȭ �� MVB ��/���� ��Ŷ ����͸� ����
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
int8_t data_buf[10];	/* ����ü ���� ���� */	/* ����ü ���� ���� */
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
	 *  arr[0] : ��ġ �ּ�
	 *  arr[1] : ��� �ڵ� (��ɾ�) 0x80
	 *  arr[2] : ���� �ּ� (���� ����Ʈ)
	 *  arr[3] : ���� �ּ� (���� ����Ʈ)
	 *  arr[4] : ������ ũ��(���� ����Ʈ)
	 *  arr[5] : ������ ũ��(���� ����Ʈ)
	 *  arr[6~������ũ��] : ������
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
	HAL_UART_Transmit(&huart3,tx_buf,8,10);						//�������ݿ����� 8����Ʈ�� �ٽ� pc ������ ������

	Start_Addr |= ((arr[3] & 0x00FF) << 8);
	Start_Addr |= arr[2];

	for(i=0; i<size; i++)
	{
		mram_byte_write(MRAM_DATA_ADDR+Start_Addr+i,arr[6+i]);
	}

	if(Start_Addr == 0x35)										//53����
	{
		result_data = mram_byte_read(MRAM_DATA_ADDR+Start_Addr);
		if((result_data & 0x01) == true)						//�����������α׷� ���� ��ư
		{
			mip_Input.CommandOpen = true;
			#ifdef DEBUG_SIMULATOR
			mip_Input.di0_OpenCloseButton = true;
			#endif
		}
		if(((result_data>>4) & 0x01) == true)					//�����������α׷� ���� ��ư
		{
			mip_Input.CommandClose = true;
			#ifdef DEBUG_SIMULATOR
			mip_Input.di0_OpenCloseButton = false;
			#endif
		}

		if(((result_data>>7) & 0x01) == true)
		{
			//���� ��� Ȱ��
			mip_Input.CommandTestMode = true;
		}
		else
		{
			//���� ��� ��Ȱ��
			mip_Input.CommandTestMode = false;
		}
	}
	/*
	 * ���� ��� �ǿ��� ���� ���� ��ư ���� ���
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
	 * �Ķ���� �ǿ��� ���� ��ư�� ���� ���
	 */
	else if(Start_Addr == 0x5F)
	{
		/* dcu�� �޸� ���� �ּ� ���� ������ ũ����� mram�� �����͸� �����Ѵ�.*/
		for(i=Start_Addr; i<=(Start_Addr+size); i++)
		{
			/* ���� ���� �迭���� �����Ͱ� �����Ƿ� mram�� �����͸� ����Ѵ�.*/
			mram_byte_write((uint32_t)(MRAM_DATA_ADDR+i),arr[6+q]);
			q++;
		}

		openConfig_value    |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x005F);					//95����
		temp_value 		     = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0060)<<8);			//96����
		openConfig_value    |= temp_value;
		CloseConfig_value   |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0061);					//97����
		temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0062)<<8);		    //98����
		CloseConfig_value   |= temp_value;
		ObstaclConfig_value |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0063);				    //99����
		temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0064)<<8);			//100����
		ObstaclConfig_value |= temp_value;

		mdm_time.OpenConfigtime  = (uint32_t)openConfig_value;
		mdm_time.CloseConfigtime = (uint32_t)CloseConfig_value;
		mod_Detect.ObstacleConfigValue = ObstaclConfig_value;
		/*
		 * �ش�κ��� ���� feedback�� ���۷��� �ӵ� ���� �����ϴµ� ���� pid ������ �Ϸ�Ǹ� �ڵ� ������ �ٽ��ؾ���
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
		tx_buf[3] = 0x02;				//������ ���� ��ɾ� : ������ ����
		tx_buf[4] = block_num;
		tx_buf[5] = block_num>>8;
		tx_buf[6] = 61;					//��Ͼ��� ������ ����
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
		tx_buf[3] = 0x04;				//������ ���� ��ɾ�  :���ۿϷ�
		tx_crc = CalcCrc(tx_buf ,4);
		tx_buf[4] = (tx_crc & 0xff00) >> 8;
		tx_buf[5] = (tx_crc & 0x00ff);

		HAL_UART_Transmit(&huart3,tx_buf,6,10);
	}
}

/*
 *  �������� ���α׷��� ���� �������� �����ϱ� ���� �Լ�
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
	tx_buf[2] = arr[2]; //������ ������ �б� ������ dcu�� ���� �ּ�(����)
	tx_buf[3] = arr[3]; //������ ������ �б� ������ dcu�� ���� �ּ�(����)
	tx_buf[4] = arr[4]; //���� �ּҺ��� ���� �޸� ����Ʈ ũ�� (����)
	tx_buf[5] = arr[5]; //���� �ּҺ��� ���� �޸� ����Ʈ ũ�� (����)

	//arr_size = ((arr[5] << 8) | arr[4]);  //���� ���� ����Ʈ�� �� ������ �����ؼ� ����� ����Ѵ�.
	arr_size |= ((arr[5] & 0x00FF) <<8);
	arr_size |= (arr[4] & 0x00FF);
	Start_Addr |= (((uint16_t)arr[3] & 0x00FF) << 8);
	Start_Addr |= (uint16_t)arr[2];

	/*
	 * ����ġ �Էµ� ���� �������� ����
	 */
	if(Start_Addr == 0x0000u)
	{
		for(i=Start_Addr; i<arr_size; i++)
		{
			tx_buf[i+6] = mram_byte_read(MRAM_DATA_ADDR+i);
		}
		tx_crc = CalcCrc(tx_buf,(arr_size+6));							//�����������α׷����� ���� ��� ����Ʈ crc ���
		tx_buf[arr_size+6] =(uint8_t)((tx_crc & 0xff00) >> 8);					//�������ݿ����� ������ �迭�� 2����Ʈ�� crc �����͸� ����ؼ� ���Ѵ�.
		tx_buf[arr_size+7] =(uint8_t)(tx_crc & 0x00ff);
		HAL_UART_Transmit(&huart3,tx_buf,(arr_size+8),100);				// +8�� �迭 [0]~[5]�� ������ ������ crc 2����Ʈ�� ���� ����
	}
	/*
	 * �����������α׷����� �ҷ����� ��ư�� ���� ���
	 */
	else if(Start_Addr == 0x003F)										//63����
	{
		for(i=0; i<arr_size; i++)
		{
			tx_buf[i+6] = mram_byte_read((uint32_t)(MRAM_DATA_ADDR+0x005F+i));		//93���������� ���� ������ �ҷ��鿩�ͼ� �����Ѵ�.
		}
		tx_crc = CalcCrc(tx_buf,(arr_size+6));							//�����������α׷����� ���� ��� ����Ʈ crc ���
		tx_buf[arr_size+6] =(uint8_t)((tx_crc & 0xff00) >> 8);					//�������ݿ����� ������ �迭�� 2����Ʈ�� crc �����͸� ����ؼ� ���Ѵ�.
		tx_buf[arr_size+7] =(uint8_t)(tx_crc & 0x00ff);
		HAL_UART_Transmit(&huart3,tx_buf,(uint8_t)(arr_size+8),100);				// +8�� �迭 [0]~[5]�� ������ ������ crc 2����Ʈ�� ���� ����
	}
}

void SlaveRunning_Check(osEvent Pevent)
{
	/*
	 *  jjkim 1901011 : ������ dcu�� ���� on ��, �����̺��� ���� ���¸� ������� �޾Ƽ� ���� ������ ���θ� ���� �Ѵ�.
	 *  ������� �������� ǻ� ���� ���¿��� ������ Ű�� �����̺갡 �����ϰ� �ȴ�.
	 *  �̻��¿��� �������� ǻ� �����ϰ� �Ǹ� �����Ϳ��� ������� �������� �ȵǹǷ� (�� ������ ������ ����)
	 *  �ʱ�ȭ ���۶� �����̺��� ���� ���θ� �ľ��� �� ������ ���߰� �Ѵ�.
	 */
	uint8_t CheckSumHigh=0, CheckSumLow=0;
	uint16_t CheckSumRxPacket=0;

    if(m_isMasterSlave == MASTER_DCU)
    {

        	HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);


    	Pevent = osMessageGet(hRx5Queue, 100);														// ������ �ð� ���� Slave�� ������ �޽��� ���� ���
		if(Pevent.status == osEventMessage)															// Slave�κ��� �޽��� ���� ����
		{
			if(aUartRx5Buffer[0]==0xBB)
			{
				CheckSumRxPacket = aUartRx5Buffer[1]+aUartRx5Buffer[2]+aUartRx5Buffer[3]+aUartRx5Buffer[4]+aUartRx5Buffer[5];
				CheckSumHigh = (uint8_t)((CheckSumRxPacket & 0xFF00)>>8);
				CheckSumLow = (uint8_t)(CheckSumRxPacket & 0x00FF);

				/*
				 * Slave�� �۽��� 8byte ��Ŷ ���������� ����
				 */
				if((CheckSumHigh==aUartRx5Buffer[6]) && (CheckSumLow==aUartRx5Buffer[7]))
				{
					if(aUartRx5Buffer[3]==true)														// Slave�� ��ü�ߴٰ� �˷��ָ�
					{
						m_isSlaveRunCommand = true;
						mip_MasterCodeMasterLED(0);													// Master�� ������ ���� ��
						mip_SleveCodeSlaveRun(1);													// ������ fnd�� ��� ������� �����̺�� �ѱ��.
						debug("�ʱ�ȭ ������ �����̺� ���� Ȯ�� \r\n");
					}
				}
				else
				{
					printf("������ \r\n");
				}
			}
		}
		else
		{
			debug("�ʱ�ȭ ������ �����̺� �̵��� Ȯ�� \r\n");
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
		spiTxBuffer[11] = aUartRx1Buffer[3];						// ���� 2
		spiTxBuffer[12] = aUartRx1Buffer[2];						// ���� 1
		spiTxBuffer[13] = aUartRx1Buffer[5];						// ���� 2
		spiTxBuffer[14] = aUartRx1Buffer[4];						// ���� 1
		spiTxBuffer[15] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[16] = aUartRx1Buffer[6];						// �� ����
		spiTxBuffer[30] = aUartRx1Buffer[8];						// ���� �ð�
	}
	else if((dcu_id == DCU_L3_MCAR) || (dcu_id == DCU_L3_TCAR))
	{
		spiTxBuffer[17] = aUartRx1Buffer[3];						// ���� 2
		spiTxBuffer[18] = aUartRx1Buffer[2];						// ���� 1
		spiTxBuffer[19] = aUartRx1Buffer[5];						// ���� 2
		spiTxBuffer[20] = aUartRx1Buffer[4];						// ���� 1
		spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[22] = aUartRx1Buffer[6];						// �� ����
		spiTxBuffer[31] = aUartRx1Buffer[8];						// ���� �ð�
	}
	else if((dcu_id == DCU_L1_MCAR) || (dcu_id == DCU_L1_TCAR))
	{
		spiTxBuffer[23] = aUartRx1Buffer[3];						// ���� 2
		spiTxBuffer[24] = aUartRx1Buffer[2];						// ���� 1
		spiTxBuffer[25] = aUartRx1Buffer[5];						// ���� 2
		spiTxBuffer[26] = aUartRx1Buffer[4];						// ���� 1
		spiTxBuffer[27] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[28] = aUartRx1Buffer[6];						// �� ����
		spiTxBuffer[32] = aUartRx1Buffer[8];						// ���� �ð�
	}
	else //	DCU_L4_MCAR || DCU_L4_TCAR
	{
		spiTxBuffer[5] = Statement_DCU(3);							// ���� 2
		spiTxBuffer[6] = Statement_DCU(2);							// ���� 1
		spiTxBuffer[7] = FalutStatement_DCU(5);						// ���� 2
		spiTxBuffer[8] = FalutStatement_DCU(4);						// ���� 1
		spiTxBuffer[9] = m_isFaultCount;										// TDNO
		spiTxBuffer[10] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// �� ����
		spiTxBuffer[29] = (uint8_t)(mdm_time.Closing / 100);										// ���� �ð�
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
		spiTxBuffer[11] = aUartRx1Buffer[3];						// ���� 2
		spiTxBuffer[12] = aUartRx1Buffer[2];						// ���� 1
		spiTxBuffer[13] = aUartRx1Buffer[5];						// ���� 2
		spiTxBuffer[14] = aUartRx1Buffer[4];						// ���� 1
		spiTxBuffer[15] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[16] = aUartRx1Buffer[6];						// �� ����
		spiTxBuffer[30] = aUartRx1Buffer[8];						// DCU2 ���� �ð�
	}
	else if((dcu_id == DCU_R3_MCAR) || (dcu_id == DCU_R3_TCAR))
	{
		spiTxBuffer[17] = aUartRx1Buffer[3];						// ���� 2
		spiTxBuffer[18] = aUartRx1Buffer[2];						// ���� 1
		spiTxBuffer[19] = aUartRx1Buffer[5];						// ���� 2
		spiTxBuffer[20] = aUartRx1Buffer[4];						// ���� 1
		spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[22] = aUartRx1Buffer[6];						// �� ����
		spiTxBuffer[31] = aUartRx1Buffer[8];						// DCU3 ���� �ð�
	}
	else if((dcu_id == DCU_R4_MCAR) || (dcu_id == DCU_R4_TCAR))
	{
		spiTxBuffer[23] = aUartRx1Buffer[3];						// ���� 2
		spiTxBuffer[24] = aUartRx1Buffer[2];						// ���� 1
		spiTxBuffer[25] = aUartRx1Buffer[5];						// ���� 2
		spiTxBuffer[26] = aUartRx1Buffer[4];						// ���� 1
		spiTxBuffer[27] = aUartRx1Buffer[7];						// TDNO
		spiTxBuffer[28] = aUartRx1Buffer[6];						// �� ����
		spiTxBuffer[32] = aUartRx1Buffer[8];						// DCU4 ���� �ð�
	}
	else //	DCU_R1_MCAR || DCU_R1_TCAR
	{
		spiTxBuffer[5] = Statement_DCU(3);							// ���� 2
		spiTxBuffer[6] = Statement_DCU(2);							// ���� 1
		spiTxBuffer[7] = FalutStatement_DCU(5);						// ���� 2
		spiTxBuffer[8] = FalutStatement_DCU(4);						// ���� 1
		spiTxBuffer[9] = m_isFaultCount;										// TDNO
		spiTxBuffer[10] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// �� ����
		spiTxBuffer[29] = (uint8_t)(mdm_time.Closing / 100);		// DCU1 ���� �ð�
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
			spiTxBuffer[5] = 0x00;						// ���� 2
			spiTxBuffer[6] = 0x00;						// ���� 1
			spiTxBuffer[7] = 0x00;						// ���� 2
			spiTxBuffer[8] = 0x80; //jeon_20190423
			spiTxBuffer[9] = 0x00;						// TDNO
			spiTxBuffer[10] = 0x00;						// �� ����
			spiTxBuffer[30] = 0x00;						// ���� �ð�
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
			spiTxBuffer[10] = aUartRx1Buffer[6];						// �� ����
			spiTxBuffer[30] = aUartRx1Buffer[8];						// ���� �ð�
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
			spiTxBuffer[11] = 0x00;						// ���� 2
			spiTxBuffer[12] = 0x00;						// ���� 1
			spiTxBuffer[13] = 0x00;						// ���� 2
			spiTxBuffer[14] = 0x80; 					// ���� 1
			spiTxBuffer[15] = 0x00;						// TDNO
			spiTxBuffer[16] = 0x00;						// �� ����
			spiTxBuffer[29] = 0x00;						// ���� �ð�
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
			
			spiTxBuffer[11] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_11;					// ���� 2
			spiTxBuffer[12] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_12;						// ���� 1
			spiTxBuffer[13] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_13;						// ���� 2
			spiTxBuffer[14] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_14;						// ���� 1
			spiTxBuffer[15] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[16] = aUartRx1Buffer[6];						// �� ����
			spiTxBuffer[29] = aUartRx1Buffer[8];						// ���� �ð�
									
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
			spiTxBuffer[17] = 0x00;						// ���� 2
			spiTxBuffer[18] = 0x00;						// ���� 1
			spiTxBuffer[19] = 0x00;						// ���� 2
			spiTxBuffer[20] = 0x80; //jeon_20190423
			spiTxBuffer[21] = 0x00;						// TDNO
			spiTxBuffer[22] = 0x00;						// �� ����
			spiTxBuffer[32] = 0x00;						// ���� �ð�
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
			
			spiTxBuffer[17] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_17;					// ���� 2
			spiTxBuffer[18] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_18;						// ���� 1
			spiTxBuffer[19] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_19;						// ���� 2
			spiTxBuffer[20] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_20;						// ���� 2
			spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[22] = aUartRx1Buffer[6];						// �� ����
			spiTxBuffer[32] = aUartRx1Buffer[8];						// ���� �ð�
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
	
		spiTxBuffer[23] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_23;					// ���� 2
		spiTxBuffer[24] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_24;						// ���� 1
		spiTxBuffer[25] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_25;						// ���� 2
		spiTxBuffer[26] = curr_spiTxbuffer_DCU_RS485_ERROR_L1_26;						// ���� 2
		spiTxBuffer[27] = m_isFaultCount;										// TDNO
		spiTxBuffer[28] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// �� ����
		spiTxBuffer[31] = (uint8_t)(mdm_time.Closing / 100);										// ���� �ð�
		
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
				
		spiTxBuffer[5] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_5;					// ���� 2
		spiTxBuffer[6] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_6;						// ���� 1
		spiTxBuffer[7] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_7;						// ���� 2
		spiTxBuffer[8] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_8;						// ���� 2
		spiTxBuffer[9] = m_isFaultCount;										// TDNO
		spiTxBuffer[10] = ((uint8_t)(VERSION_MAJOR*10) + (uint8_t)VERSION_MINOR);										// �� ����
		spiTxBuffer[30] = (uint8_t)(mdm_time.Closing / 100);										// ���� �ð�
		
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_5 = Statement_DCU(3);
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_6 = Statement_DCU(2);
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_7 = FalutStatement_DCU(5);
		comp_spiTxbuffer_DCU_RS485_ERROR_R1_8 = FalutStatement_DCU(4); //jeon_190712 orig:0x3f		
	}
	else if((dcu_id == DCU_R2_MCAR) || (dcu_id == DCU_R2_TCAR))
	{
		if(error_485_flag[DCU_RS485_ERROR_R2] > 10)
		{
			spiTxBuffer[11] = 0x00;						// 8 ���� 2
			spiTxBuffer[12] = 0x00;						// 9 ���� 1
			spiTxBuffer[13] = 0x00;						// 10 ���� 2
			spiTxBuffer[14] = 0x80; //jeon_20190423
			spiTxBuffer[15] = 0x00;						// 12 TDNO
			spiTxBuffer[16] = 0x00;						// 13 �� ����
			spiTxBuffer[29] = 0x00;						// ���� �ð�
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
		
			spiTxBuffer[11] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_11;					// ���� 2
			spiTxBuffer[12] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_12;						// ���� 1
			spiTxBuffer[13] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_13;						// ���� 2
			spiTxBuffer[14] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_14;						// ���� 2
			spiTxBuffer[15] = aUartRx1Buffer[7];						// 12 TDNO
			spiTxBuffer[16] = aUartRx1Buffer[6];						// 13 �� ����
			spiTxBuffer[29] = aUartRx1Buffer[8];						// ���� �ð�
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
			spiTxBuffer[17] = 0x00;						// ���� 2
			spiTxBuffer[18] = 0x00;						// ���� 1
			spiTxBuffer[19] = 0x00;						// ���� 2
			spiTxBuffer[20] = 0x80; //jeon_20190423
			spiTxBuffer[21] = 0x00;						// TDNO
			spiTxBuffer[22] = 0x00;						// �� ����
			spiTxBuffer[32] = 0x00;						// ���� �ð�
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
		
			spiTxBuffer[17] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_17;					// ���� 2
			spiTxBuffer[18] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_18;						// ���� 1
			spiTxBuffer[19] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_19;						// ���� 2
			spiTxBuffer[20] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_20;						// ���� 2
			spiTxBuffer[21] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[22] = aUartRx1Buffer[6];						// �� ����
			spiTxBuffer[32] = aUartRx1Buffer[8];						// ���� �ð�
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
			spiTxBuffer[23] = 0x00;						// ���� 2
			spiTxBuffer[24] = 0x00;						// ���� 1
			spiTxBuffer[25] = 0x00;						// ���� 2
			spiTxBuffer[26] = 0x80; //jeon_20190423
			spiTxBuffer[27] = 0x00;						// TDNO
			spiTxBuffer[28] = 0x00;						// �� ����
			spiTxBuffer[31] = 0x00;						// ���� �ð�
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
			
			spiTxBuffer[23] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_23;					// ���� 2
			spiTxBuffer[24] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_24;						// ���� 1
			spiTxBuffer[25] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_25;						// ���� 2
			spiTxBuffer[26] = curr_spiTxbuffer_DCU_RS485_ERROR_R1_26;						// ���� 2
			spiTxBuffer[27] = aUartRx1Buffer[7];						// TDNO
			spiTxBuffer[28] = aUartRx1Buffer[6];						// �� ����
			spiTxBuffer[31] = aUartRx1Buffer[8];						// ���� �ð�
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
	/*============================================================================ F16    : RS485 ����
	 *  							���� ���� ǥ��									   F15    : MVB ����
	 *============================================================================ F14    : SAFETY LOOP ����
	 *    |  bit7  |  bit6 |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0  | F13    : ���ڴ� ����
	 *============================================================================ F12    : ���߰� ��ü �Ұ�
	 *[4] |  F16   |  F15  |  F14   |  F13   |  F12   |  F11   |  F10   |  F09   | F11    : ���߰� ���� ��
	 *============================================================================ F10    : ���� ����
	 *[5] |  F08   |  F07  |  F06   |  F05   |  F04   |  F03   |  F02   |   F01  | F09    : ��ֹ� ����
	 *============================================================================ F08    : DLS 2 ����
	 *																			   F07    : ����ġ ���� ��� Ǯ��
	 *																			   F06    : DCS2 ����
	 *																			   F05    : DCS1 ����
	 *																			   F04    : DLS1 ����
	 *																			   F03    : MOTOR ����
	 *																			   F02    : �����
	 *																			   F01    : �߰���
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
	 * 								���� ���� ǥ��
	 *============================================================================	REOPEN : REOPEN ��ư ����
	 *    |  bit7  |  bit6 |  bit5  |  bit4  |  bit3  |  bit2  |  bit1  |  bit0	 |	CLOSEPB: CLOSE ��ư ����
	 *============================================================================	OPENPB : OPEN ��ư ����
	 *[2] | Spare | Spare  | REOPEN | CLOSEPB| OPENPB |  EAD   |  EED   |  OBS   |	EAD    : �ܺ� ��� �ڵ�
	 *============================================================================	EED    : ���� ��� �ڵ�
	 *[3] |  FP   |  DFO   |  DI    |  DNC   | DCU OK |   X    |   X    |   X    |	OBS    : ��ֹ� ����
	 *============================================================================	FP     : DCU�� ����� ���� ����
	 *[4] |							���� ����(1)										DFO    : DCU�� ���� ���� ����
	 *============================================================================	DI     : Isolation ����
	 *[5] |                         ���� ����(2)										DNC    : ������ ���� ����
	 *============================================================================	DCU OK : DCU �۵� �� ??
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
		//if(mdc_DoorState != DOOR_CLOSED)	      return_statement |= 0x10;		// S05_DNC, ������ ���� ����
		if((mip_Input.di0_DLS1 == false) && (mip_Input.di1_DLS2 == false))	return_statement |= 0x10;		// S05_DNC, ������ ���� ����
		if(mip_Input.di0_Isolation == true)       return_statement |= 0x20;		// S06_DI, ���� ����ġ ����
		if(mdc_DoorState == DOOR_OPENED)		  return_statement |= 0x40;		// S07_DFO, ���� ���� ����
		if(mip_Input.DoorClosedSwitchOn == false) return_statement |= 0x80;		// S08_FP, dcu�� ����� ���� ����
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
    	case S05_DNC:		debug("## Set S05, ������ ���� ����\r\n");	mdc_DoorState = DOOR_OPENING; break;			// [3] 0x10
    	case S06_DI:		debug("## Set S06, ���� ����ġ ����\r\n");	mip_Input.di0_Isolation ^= 1; break;			// [3] 0x20
    	case S07_DFO:		debug("## Set S07, ���� ���� ����\r\n"); mdc_DoorState = DOOR_OPENED; break;				// [3] 0x40
    	case S08_FP:		debug("## Set S08, dcu�� ����� ���� ����\r\n"); mip_Input.DoorClosedSwitchOn ^= 1; break;	// [3] 0x80
    	
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
    	case DCU_HARD_FAULT:		debug("## Set F01 �߰���\r\n");				error_list_flag[DCU_HARD_FAULT] ^= 1; break;
    	case DCU_MINOR_FAULT:		debug("## Set F02 �����\r\n");				error_list_flag[DCU_MINOR_FAULT] ^= 1; break;
    	case DCU_MOTOR_ERROR:		debug("## Set F03 ���� ȸ�� ����\r\n");		error_list_flag[DCU_MOTOR_ERROR] ^= 1; break;
    	case DCU_DLS1_FAULT:		debug("## Set F04 DLS1����\r\n");			error_list_flag[DCU_DLS1_FAULT] ^= 1; break;
    	case DCU_DCS1_FAULT:		debug("## Set F05 DCS1����\r\n");			error_list_flag[DCU_DCS1_FAULT] ^= 1; break;
    	case DCU_DCS2_FAULT:		debug("## Set F06 DCS2����\r\n");			error_list_flag[DCU_DCS2_FAULT] ^= 1; break;
    	case DCU_UNEXPECTED_UNLOCK:	debug("## Set F07 ����ġ ���� ��� Ǯ��\r\n");	error_list_flag[DCU_UNEXPECTED_UNLOCK] ^= 1; break;
    	case DCU_DLS2_FAULT:		debug("## Set F08 DLS2����\r\n");			error_list_flag[DCU_DLS2_FAULT] ^= 1; break;
    	case DCU_OBSTACLE:			debug("## Set F09 ������ ��ְ��� ����\r\n");	error_list_flag[DCU_OBSTACLE] ^= 1; break;
    	case DCU_OPEN_FAULT:		debug("## Set F10 ���� ���� ���� ����\r\n");	error_list_flag[DCU_OPEN_FAULT] ^= 1; break;
    	case DCU_SLAVE_RUN:			debug("## Set F11 ���߰� ���� ��\r\n");		error_list_flag[DCU_SLAVE_RUN] ^= 1; break;
    	case DCU_CANT_SLAVE:		debug("## Set F12 ���߰� ��ȯ �Ұ�\r\n");		error_list_flag[DCU_CANT_SLAVE] ^= 1; break;
    	case DCU_ENCODER_ERROR:		debug("## Set F13 ���ڴ� ����\r\n");			error_list_flag[DCU_ENCODER_ERROR] ^= 1; break;
    	case DCU_SAFETYLOOP_FAULT:	debug("## Set F14 Safety Loop �̻�\r\n");	error_list_flag[DCU_SAFETYLOOP_FAULT] ^= 1; break;
    	case DCU_MVB_ERROR:			debug("## Set F15 MVB��� ����\r\n");			error_list_flag[DCU_MVB_ERROR] ^= 1; break;
    	case DCU_RS485_ERROR:		debug("## Set F16 RS-485��� ����\r\n");		error_list_flag[DCU_RS485_ERROR] ^= 1; break;
    	case DCU_LOCK_FAULT:		debug("## Set F17 ��� ���� -> ����\r\n");		error_list_flag[DCU_LOCK_FAULT] ^= 1; break;
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
	pValue1 = strtol(pStringValue,&pEnd,16);								// ���ڿ��� 16���� ���� ���ڷ� ��ȯ
	pStringValue = strtok(NULL," ");
	pValue2 = strtol(pStringValue,&pEnd,16);								// ���ڿ��� 16���� ���� ���ڷ� ��ȯ
	
	if(strstr(pSubCommand,"write") != NULL)
	{
		debug("## Write data to 0x%02X: %02X\r\n", pValue1, (uint8_t)pValue2);
		mram_byte_write(pValue1, (uint8_t)pValue2);										// MRAM 0x1FFFF������ 0xAA Write
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
    	 * Closed/Closing ���¿��� ������ư(3s)�� ���� ���
    	 */
    	case 0x31:
    		debugprint.Data_Print_flag ^= 0x01;
    		if(debugprint.Data_Print_flag == true)	{debug("������ ��� Ȱ��ȭ \r\n");}
    		else									{debug("������ ��� ��Ȱ��ȭ \r\n");}
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
		//PRINT ���� ���ִϱ� ������ PRINT���� �߰��ϸ� ����
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
		if(mip_Test.DLS1_Test_flag == true)	{debug("DLS 1 ����ġ �Է��� ���� ���� \r\n");}
		else								{debug("DLS 1 ����ġ �Է��� ���� \r\n");}
	}
	else if(strstr(pSubCommand,"tdls2") != NULL)
	{
		mip_Test.DLS2_Test_flag ^= 0x01;
		if(mip_Test.DLS2_Test_flag == true)	{debug("DLS 2 ����ġ �Է��� ���� ���� \r\n");}
		else								{debug("DLS 2 ����ġ �Է��� ���� \r\n");}
	}
	else if(strstr(pSubCommand,"tdcs1") != NULL)
	{
		mip_Test.DCS1_Test_flag ^= 0x01;
		if(mip_Test.DCS1_Test_flag == true)	{debug("DCS 1 ����ġ �Է��� ���� ���� \r\n");}
		else								{debug("DCS 1 ����ġ �Է��� ���� \r\n");}
	}
	else if(strstr(pSubCommand,"tdcs2") != NULL)
	{
		mip_Test.DCS2_Test_flag ^= 0x01;
		if(mip_Test.DCS2_Test_flag == true)	{debug("DCS 2 ����ġ �Է��� ���� ���� \r\n");}
		else								{debug("DCS 2 ����ġ �Է��� ���� \r\n");}
	}
	else if(strstr(pSubCommand,"topen") != NULL)
	{
		mip_Test.OpenButton_Test_flag ^= 0x01;
		if(mip_Test.DCS2_Test_flag == true)	{debug("����/���� ����ġ �Է��� ���� \r\n");}
		else								{debug("����/���� ����ġ �Է��� �������� \r\n");}
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
    	 * Closed/Closing ���¿��� ������ư(3s)�� ���� ���
    	 */
    	case 0x31:
    		mip_Input.di0_OpenCloseButton ^= 0x01;
    		/*
    		 * todo : �̰� �츮�� ���� �� ����
    		 */
//    		if(mip_Input.di0_OpenCloseButton)	debug("door open\r\n");
//    		else								debug("door close\r\n");
    		break;
    	/*
    	 * Closing ���¿��� ��ֹ� ������ ���
    	 */
    	case 0x32:
    		mip_Input.di0_DLS1 ^= 0x01;
    		mip_Input.di1_DLS2 ^= 0x01;
    		debug("dls off \r\n");
    		//debug("dls �Է� ���� \r\n");
    		break;
    	case 0x33:
    		mip_Input.di0_ReOpen ^= 0x01;
    		debug("Reopen �Է�\r\n");
    		break;
    	case 0x34:
    		mip_Input.di0_Isolation ^= 0x01;
    		if(mip_Input.di0_Isolation)		{debug("����\r\n");}
    		else							{debug("����\r\n");}
    		break;
    	case 0x35:
    		mip_Input.di0_Bypass ^= 0x01;
    		if(mip_Input.di0_Bypass)		{debug("DODBPS On\r\n");}
    		else							{debug("DODBPS Off");mdc_FlagTestDODBPS=false;}
    		break;
    	case 0x36:
    		mmf_EndDecision.OpenedByCurrent = true;
    		mip_Input.DoorClosedSwitchOn = false;
    		debug("������ ���������Ǵ�\r\n");
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
	pValue = strtol(pStringValue,&pEnd,10);				// ���ڿ��� 10���� ���� ���ڷ� ��ȯ
	
	if(strstr(pSubCommand,"open") != NULL)
	{
		if(mdc_DoorState==DOOR_CLOSING)
		{
			mmo_DoorFree();
//			osDelay(100);								// pwm 800���� closing �ϴٰ� delay ���� �ٷ� pwm 800���� �ٷ� opening �ص� ���׳�...
		}
		if(pValue>800)	pValue = 800;
		mdc_DoorState=DOOR_OPENING;
		mmo_DoorOpening(pValue);
		/*
		 * todo : pValue prtinf�� ����ϸ� Hard_Fault�߻��ϸ鼭 CPU �� ����
		 * �׳� printf�ϸ� ���װ� �� ���� ��
		 */
//		debug("motor open pwm: %d\r\n", pValue);		// CPU Die
		debug("motor open pwm\r\n");					// ������
	}
	else if(strstr(pSubCommand,"close") != NULL)
	{
		if(mdc_DoorState==DOOR_OPENING)
		{
			mmo_DoorFree();
//			osDelay(100);								// pwm 800���� opening �ϴٰ� delay ���� �ٷ� pwm 800���� �ٷ� closing �ص� ���׳�...
		}
		if(pValue>800)	pValue = 800;
		mdc_DoorState=DOOR_CLOSING;
		mmo_DoorClosing(pValue);
		debug("motor close pwm: %d\r\n", pValue);
	}
	else if(strstr(pSubCommand,"brake") != NULL)
	{
		mmo_DoorFree();
//		osDelay(100);									// pwm 800���� opening �Ǵ� closing �ϴٰ� delay ���� �ٷ� brake �ص� ���׳�... -> �״� ���� ���������ΰ� ������...
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
		mmo_MotorDisable();								// PWM Gate 4�� Off ��Ű�� (�ǵ����� ����) Brake ȿ���� ��Ÿ��
		debug("motor Disable\r\n");
	}
	else if(strstr(pSubCommand,"enable") != NULL)
	{
		#ifdef HW_DEPENDENT_CODE
			HAL_GPIO_WritePin(MShutDown_GPIO_Port, MShutDown_Pin, GPIO_PIN_RESET); // Low ��� -> AND Gate true �Է� -> PWM�� H-Bridge�� ��� ��
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
		if(strstr(pSubCommand1,"L1") != NULL)		{g_unDeviceID=DCU_L1_MCAR; debug("��������, DCU ID: DCU_L1_MCAR\r\n");}
		else if(strstr(pSubCommand1,"L2") != NULL)	{g_unDeviceID=DCU_L2_MCAR; debug("��������, DCU ID: DCU_L2_MCAR\r\n");}
		else if(strstr(pSubCommand1,"L3") != NULL)	{g_unDeviceID=DCU_L3_MCAR; debug("��������, DCU ID: DCU_L3_MCAR\r\n");}
		else if(strstr(pSubCommand1,"L4") != NULL)	{g_unDeviceID=DCU_L4_MCAR; debug("��������, DCU ID: DCU_L4_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R1") != NULL)	{g_unDeviceID=DCU_R1_MCAR; debug("��������, DCU ID: DCU_R1_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R2") != NULL)	{g_unDeviceID=DCU_R2_MCAR; debug("��������, DCU ID: DCU_R2_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R3") != NULL)	{g_unDeviceID=DCU_R3_MCAR; debug("��������, DCU ID: DCU_R3_MCAR\r\n");}
		else if(strstr(pSubCommand1,"R4") != NULL)	{g_unDeviceID=DCU_R4_MCAR; debug("��������, DCU ID: DCU_R4_MCAR\r\n");}
	}
	else if(strstr(pSubCommand2,"t") != NULL)
	{
		if(strstr(pSubCommand1,"L1") != NULL)		{g_unDeviceID=DCU_L1_TCAR; debug("��������, DCU ID: DCU_L1_TCAR\r\n");}
		else if(strstr(pSubCommand1,"L2") != NULL)	{g_unDeviceID=DCU_L2_TCAR; debug("��������, DCU ID: DCU_L2_TCAR\r\n");}
		else if(strstr(pSubCommand1,"L3") != NULL)	{g_unDeviceID=DCU_L3_TCAR; debug("��������, DCU ID: DCU_L3_TCAR\r\n");}
		else if(strstr(pSubCommand1,"L4") != NULL)	{g_unDeviceID=DCU_L4_TCAR; debug("��������, DCU ID: DCU_L4_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R1") != NULL)	{g_unDeviceID=DCU_R1_TCAR; debug("��������, DCU ID: DCU_R1_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R2") != NULL)	{g_unDeviceID=DCU_R2_TCAR; debug("��������, DCU ID: DCU_R2_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R3") != NULL)	{g_unDeviceID=DCU_R3_TCAR; debug("��������, DCU ID: DCU_R3_TCAR\r\n");}
		else if(strstr(pSubCommand1,"R4") != NULL)	{g_unDeviceID=DCU_R4_TCAR; debug("��������, DCU ID: DCU_R4_TCAR\r\n");}
	}
	
	if(strstr(pSubCommand1," "))
	{
		debug("CLI Undefined Command\r\n");
		debug("'dcu L1 m' : Set DCU ID to DCU_L1_MCAR\r\n");				// DCU MCAR ����
		debug("'dcu L2 m' : Set DCU ID to DCU_L2_MCAR\r\n");
		debug("'dcu L3 m' : Set DCU ID to DCU_L3_MCAR\r\n");
		debug("'dcu L4 m' : Set DCU ID to DCU_L4_MCAR\r\n");
		debug("'dcu R1 m' : Set DCU ID to DCU_R1_MCAR\r\n");
		debug("'dcu R2 m' : Set DCU ID to DCU_R2_MCAR\r\n");
		debug("'dcu R3 m' : Set DCU ID to DCU_R3_MCAR\r\n");
		debug("'dcu R4 m' : Set DCU ID to DCU_R4_MCAR\r\n");
		
		debug("'dcu L1 t' : Set DCU ID to DCU_L1_TCAR\r\n");				// DCU TCAR ����
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
		 * ��ü �� Master�� �ƹ��͵� �������
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
				//debug("# Master 485 Task ���� ����\r\n");
			}
			continue;
		}
		
		/*
		 * L4�� ��� L1~L3�� ������ ����, L1~L4�� ������ ����
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
			 * Data Request ����
			 */

			if(id_address<8) // DCU1 -> DCU2~DCU7
			{
				//(((Second/10)<<4) & 0x70) + ((Second%10) & 0x0f)
				if(id_address >= 6)	id_address = 0;
				aTxBuffer[0] = 0xAA;																// start packet
				if(m_CardefineID == Define_MCAR) aTxBuffer[1] = dcu_address_MCAR[id_address];		// Mī�ϋ� Mī�� ID�� ȣ��
				else if(m_CardefineID == Define_TCAR) aTxBuffer[1] = dcu_address_TCAR[id_address];	// Tī�� �� Tī�� ID�� ȣ��
				aTxBuffer[2] = Packet_Number_count++;												// 0~255 ī��Ʈ �ϴ� ���� ����
				checksum_value = (aTxBuffer[0] + aTxBuffer[1] + aTxBuffer[2]);						// [0] , [1] , [2] �� ���� ���� ����
				aTxBuffer[3] = m_isMasterSlaveChange;												// ��ü ��� ����
				aTxBuffer[4] = rtc_time.ucYears;
				aTxBuffer[5] = rtc_time.ucMonth;
				aTxBuffer[6] = rtc_time.ucDate;
				aTxBuffer[7] = rtc_time.ucHours;
				aTxBuffer[8] = rtc_time.ucMinutes;
				aTxBuffer[9] = ((checksum_value & 0xFF00) >>8);										// üũ�� high(���� ����Ʈ�� 3 ����Ʈ�� ����)
				aTxBuffer[10] = (checksum_value & 0x00FF);											// üũ�� low(���� ����Ʈ�� 4 ����Ʈ�� ���� ���� ����)
				if(HAL_UART_Transmit(&huart1, (uint8_t*)aTxBuffer, 11, 1000) != HAL_OK)
				{
					debug("## Transmit Error\r\n");
				}
				else
				{
					#ifdef RS485_PACKET_MONITORING
					printf("## [RS485] DCU1 ��Ŷ �۽�:");
						for(int i=0; i<11; i++)	debug("%02X ", aTxBuffer[i]);
						printf("\r\n");
					#endif
					id_address++;
				}
			}

			/*
			 * Slave(L1~L3)�� ���� Data ���� �� MVB Tx Buffer�� ����
			 */
			HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
			osDelay(1);

			event = osMessageGet(hRx1Queue, 200);
			if(event.status == osEventMessage)
			{
				#ifdef RS485_PACKET_MONITORING
				printf("\r\n## [RS485] DCU1 ��Ŷ ����: ");
					for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
					printf("\r\n\r\n");
				#endif
				error_list_time[DCU_RS485_ERROR] = osKernelSysTick();
				Erase_fault(DCU_RS485_ERROR);
				if(aUartRx1Buffer[0] == 0xBB)
				{
					/* CC + �������� + CHECK SUM HIGH + CHECK SUM LOW */
					checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
					Master_up_data = ((checksum_value & 0xFF00) >> 8);
					Master_down_data =(checksum_value & 0x00FF);
					if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //üũ�� �� ������
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
				else /* ù��° ����Ʈ�� bb�� ���� ���� ���*/
				{
					#ifdef RS485_PACKET_MONITORING
					printf("\r\n## [RS485] DCU1 ��Ŷ ������: ");
						for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						printf("\r\n\r\n");
					#endif
					osDelay(1);
					//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx1Buffer, 11,5);
					/*
					 * todo : �и� ��Ŷ ���� �� �ʱ�ȭ ���� Master/Slave ��ſ��� �߰��� ��
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
			else /* 11����Ʈ�� ���� ���� ���*/
			{
				#ifdef RS485_PACKET_MONITORING
				printf("\r\n## [RS485] DCU1 ��Ŷ �̼���: ");
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
		 * R1�� ���  R2~R4�� ������ ������ ����, R1~R4�� ������ ����
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
					/* CC + �������� + CHECK SUM HIGH + CHECK SUM LOW */
					checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
					Master_up_data = ((checksum_value & 0xFF00) >> 8);
					Master_down_data =(checksum_value & 0x00FF);
					if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //üũ�� �� ������
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
							printf("\r\n## [RS485] DCU8 ��Ŷ ����: ");
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
						debug("\r\n\## [RS485]  DCU%d ������ ��Ŷ ����: ", g_unDeviceID);
					//	for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						debug("\r\n\\r\n");
					#endif
				}
				else
				{
					#ifdef RS485_PACKET_MONITORING
					printf("\r\n## [RS485] DCU1 ��Ŷ ������: ");
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
				printf("\r\n## [RS485] DCU1 ��Ŷ �̼���: ");
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
		 * L1~L3, R2~R4�� �ڽ��� id ��û�� ������ ��� ������ ������.
		 */
		else
		{
			/*
			 * [�߿�] ���ͷ�Ʈ �߻� �� HAL_UART_Receive_IT�� ȣ���ؾ߸� HAL_OK�� return
			 *       ���� ��� HAL_UART_Receive_IT�� ȣ���ϸ� HAL_BUSY�� return
			 *       ���� ���ͷ�Ʈ ready�� �ȵ� ���¿��� HAL_UART_Receive_IT�� ȣ���� ��� ���̻� ���ͷ�Ʈ�� �߻����� ���� �� �����Ƿ�
			 *       �Ҳ� ������ �ֱ������� HAL_UART_Receive_IT ��� ȣ��
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
						if(aUartRx1Buffer[1] == g_unDeviceID) // �����Ϳ��� �ڽ��� ��û�� ���
						{
							m_isMasterSlaveChange = aUartRx1Buffer[3];

							aTxBuffer[0] = 0xBB;
							aTxBuffer[1] = g_unDeviceID;
							aTxBuffer[2] = Statement_DCU(2);	 // ���� ����
							aTxBuffer[3] = Statement_DCU(3);	 // ���� ���� 2
							aTxBuffer[4] = FalutStatement_DCU(4);// ���� ����
							aTxBuffer[5] = FalutStatement_DCU(5);// ���� ���� 2
							aTxBuffer[6] = (uint8_t)((VERSION_MAJOR*10) + VERSION_MINOR);				 // ����Ʈ���� ���� 1.0�̸� 10�� �����͸� ��������
							aTxBuffer[7] = m_isFaultCount;		 // ���� ����
							aTxBuffer[8] = (uint8_t)(mdm_time.Closing / 100);				 // ���� �ð�
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
									debug("## [RS485] DCU%d ��Ŷ �۽�: ", g_unDeviceID);
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
									printf("RTC DATA : %d �� %d�� %d�� %d�� %d�� %d �� \r\n",
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
									rtc_set_time(((aUartRx1Buffer[4]>>4)*10)+(aUartRx1Buffer[4]&0x0f),  //�⵵
											 ((aUartRx1Buffer[5]>>4)*10)+(aUartRx1Buffer[5]&0x0f),	//��
											 ((aUartRx1Buffer[6]>>4)*10)+(aUartRx1Buffer[6]&0x0f),  //��
											 ((aUartRx1Buffer[7]>>4)*10)+(aUartRx1Buffer[7]&0x0f),  //��
											 ((aUartRx1Buffer[8]>>4)*10)+(aUartRx1Buffer[8]&0x0f),  //��
											 ((0x00>>4)*10)+(0x00&0x0f)); 			   //��
								
									rs485_rtctime = rtc_get_time();
									
									if((rs485_rtctime.ucYears == aUartRx1Buffer[4]) && (rs485_rtctime.ucMonth == aUartRx1Buffer[5]) && (rs485_rtctime.ucDate == aUartRx1Buffer[6]) &&
									  (rs485_rtctime.ucHours == aUartRx1Buffer[7]) && (rs485_rtctime.ucMinutes == aUartRx1Buffer[8]))			
									{	
										printf("RTC DATA : %d �� %d�� %d�� %d�� %d�� %d �� \r\n",
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
										rtc_set_time(((aUartRx1Buffer[4]>>4)*10)+(aUartRx1Buffer[4]&0x0f),  //�⵵
													 ((aUartRx1Buffer[5]>>4)*10)+(aUartRx1Buffer[5]&0x0f),	//��
													 ((aUartRx1Buffer[6]>>4)*10)+(aUartRx1Buffer[6]&0x0f),  //��
													 ((aUartRx1Buffer[7]>>4)*10)+(aUartRx1Buffer[7]&0x0f),  //��
													 ((aUartRx1Buffer[8]>>4)*10)+(aUartRx1Buffer[8]&0x0f),  //��
													 ((0x00>>4)*10)+(0x00&0x0f)); 			   //��
									
										rs485_rtctime = rtc_get_time();
										
										printf("RTC DATA : %d �� %d�� %d�� %d�� %d�� %d �� \r\n",
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
								debug("## [RS485] DCU%d ��Ŷ ����: ", g_unDeviceID);
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
				//if(aUartRx1Buffer[0] != 0xBB) - > BB�ϰ�쿡�� �ٸ� �����̺��� �����͸� �޾ұ� ������ �����̰�, �ٸ� ��쿡�� �����Ͱ� �з��� ���� ��Ȳ
				else if(aUartRx1Buffer[0] == 0xBB)
				{
					#ifdef RS485_PACKET_MONITORING
						debug("\r\n\## [RS485] Ÿ DCU%d ��Ŷ ����: ", g_unDeviceID);
					//	for(int i=0; i<11; i++)	debug("%02X ", aUartRx1Buffer[i]);
						debug("\r\n\\r\n");
					#endif
				}
				else
				{
					#ifdef RS485_PACKET_MONITORING
						debug("\r\n\## [RS485] DCU%d �и� ��Ŷ ����: ", g_unDeviceID);
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
     * �ֱ������� Master�� Slave �� ����� ���� alive check�� ���� �� �� �ٷ� ����Ǿ�� ��.
     */
	debug("6. [Master & Slave Start] Communication Task\r\n");

	InitQueue(&rx_Int_queue);
	//__HAL_UART_ENABLE_IT(&huart3, UART_IT_ORE);						//����������,�������,�и�Ƽ ���� �߻��� ���ͷ�Ʈ ȣ���Ѵ�.

    for(;;)
    {
        /* Place this task in the blocked state until it is time to run again. */
        osDelayUntil(&PreviousWakeTime, 10UL);

        /*
         * USB(Virtual COM Port) �� UART0 �� �� �޾ƾ� �ϹǷ� �Լ� ȣ��� ��Ŷ �����ؾ� ��.
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
	 * ��Ʈ ������ �б� ���� MVB�� ��Ŷ ����
	 */
	#if 1
	{
		tmpPortNumber = (uint16_t)PortNumber<<2;
		
		spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_READ;				// �޸� ��ɾ�, Write
		spiTxBuffer[1] = 0x20|((tmpPortNumber&0xFF00)>>8);							// LA PCS Offset Address (0x2000~0x27FF)
		spiTxBuffer[2] = tmpPortNumber&0x00FF;
		spiTxBuffer[3] = 0x00;
		spiTxBuffer[4] = 0x00;
		
		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
			retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiTxBuffer, mvbDataLength, 5000);	// 1��Ŷ ��� ������
//jeon_190427		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
	}
	#endif
	
	/*
	 * MVB�κ��� ��Ʈ ���� �� Read
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
	
	spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;				// �޸� ��ɾ�, Write
	spiTxBuffer[1] = 0x20;														// LA PCS Offset Address (0x2000~0x27FF)
	spiTxBuffer[2] = PortNumber;
	spiTxBuffer[3] = Fcode|MVB_SLAVE_SOURCE;									// F_Code(����4bit), Source
	spiTxBuffer[4] = 0x00;
	
	HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
		retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiTxBuffer, 5, 5000);	// 1��Ŷ ��� ������
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
	 * Fcode�� ���� ������ Size ����
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
	
	spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;								// �޸� ��ɾ�, Write
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
			case DCU_L4_MCAR: mvbSaveDataDCUL(DCU_L4_MCAR); break;								// L1 DCU�� ���� ����(L1~L4)�� MVB�� ���� Write
			case DCU_R1_MCAR: mvbSaveDataDCUR(DCU_R1_MCAR); break;								// R1 DCU�� ���� ����(R1~R4)�� MVB�� ���� Write
			case DCU_L4_TCAR: mvbSaveDataDCUL(DCU_L4_TCAR); break;
			case DCU_R1_TCAR: mvbSaveDataDCUR(DCU_R1_TCAR); break;
			default: break;
		}
	#endif

		HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
		retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)spiTxBuffer, mvbTxDataTotalLength, 5000);		// �ٽ� �����͸� ����
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
	
	spiTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;			// �޸� ��ɾ�, Write
	spiTxBuffer[1] = 0x20;													// LA PCS Offset Address (0x2000~0x27FF)
	spiTxBuffer[2] = PortNumber;												// PortNumber: 2 (MVB �׽�Ʈ �Ҷ� ���� ��Ʈ��ȣ)
	spiTxBuffer[3] = Fcode|MVB_SLAVE_SINK;									// F_Code(����4bit), Sink
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
	 * Fcode�� ���� ������ Size ����
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
	 * MVB Read�� ���� SPI�� MVB ����
	 */
	#if 1
	{
	/*
		 * Port Number�� <<2 �� �� Offset Address�� or ���� �ؾ� ��(MVB Protocol)
		 */
		tmpPortNumber = (uint16_t)PortNumber<<2;								// 8byte�� ��� Port Number���� �ֻ��� bit�� 1�� ��� <<2�� �ϸ� overflow�� �Ͼ�Ƿ� ����ȯ �� <<2 ����
		
		/*
		 ** 5 byte �۽� ��
		 */
		spiRxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_READ;			// �޸� ��ɾ�, Read
		spiRxBuffer[1] = (tmpPortNumber&0xFF00)>>8;								// LA Data Offset (0x0000~0x1FFFF)
		spiRxBuffer[2] = tmpPortNumber&0x00FF;
		spiRxBuffer[3] = 0x00;													// ���̵����ʹ� ������ 2byte�� �����ؾ� ��
		spiRxBuffer[4] = 0x00;													// ���� �����͸� �����ؾ� Ŭ���� MVB�� ���޵Ǿ� ���� ������ �����
		
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
	 * SPI�� ���� MVB�� ����� ������(TCMS�� ������) Read
	 */
	#if 1
	{
	/*
		 ** Fcode�� �ش��ϴ� data length��ŭ ������ ����
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
		 * ��Ŷ ���� �Ϸ�
		 */
		if(retRXval==HAL_OK)
		{
			/*
			 * ���� ��Ŷ Ȯ��
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
			 * Common Data ��Ŷ ���� �� �ð� ������Ʈ
			 */
			if(PortNumber==DCU_ALL_MVB_SINK_COMMON)
			{
				// rtc �ð� ������Ʈ
				WatchdogCount = (uint16_t)spiRxBuffer[0];
				WatchdogCount |= ((uint16_t)spiRxBuffer[1]<<8);
				if(PrevWatchdogCount != WatchdogCount)
				{
					MatchCount++;
					if(MatchCount>6)
					{
						/* �� hex���� rs485�� �����ϰ� �ȴ�.*/
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
								printf("RTC DATA : %d �� %d�� %d�� %d�� %d�� %d �� \r\n",
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
								rtc_set_time(((spiRxBuffer[2]>>4)*10)+(spiRxBuffer[2]&0x0f),  //�⵵
											 ((spiRxBuffer[3]>>4)*10)+(spiRxBuffer[3]&0x0f),  //��
											 ((spiRxBuffer[4]>>4)*10)+(spiRxBuffer[4]&0x0f),  //��
											 ((spiRxBuffer[5]>>4)*10)+(spiRxBuffer[5]&0x0f),  //��
											 ((spiRxBuffer[6]>>4)*10)+(spiRxBuffer[6]&0x0f),  //��
											 ((0x00>>4)*10)+(0x00&0x0f)); 			   //��
							
								rtc_time = rtc_get_time();
								
								if((rtc_time.ucYears == spiRxBuffer[2]) && (rtc_time.ucMonth == spiRxBuffer[3]) && (rtc_time.ucDate == spiRxBuffer[4]) &&
								  (rtc_time.ucHours == spiRxBuffer[5])	&& (rtc_time.ucMinutes == spiRxBuffer[6]))			
								{	
									printf("RTC DATA : %d �� %d�� %d�� %d�� %d�� %d �� \r\n",
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
									rtc_set_time(((spiRxBuffer[2]>>4)*10)+(spiRxBuffer[2]&0x0f),  //�⵵
												 ((spiRxBuffer[3]>>4)*10)+(spiRxBuffer[3]&0x0f),  //��
												 ((spiRxBuffer[4]>>4)*10)+(spiRxBuffer[4]&0x0f),  //��
												 ((spiRxBuffer[5]>>4)*10)+(spiRxBuffer[5]&0x0f),  //��
												 ((spiRxBuffer[6]>>4)*10)+(spiRxBuffer[6]&0x0f),  //��
												 ((0x00>>4)*10)+(0x00&0x0f)); 			   //��
								
									rtc_time = rtc_get_time();
									
									printf("RTC DATA : %d �� %d�� %d�� %d�� %d�� %d �� \r\n",
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
//							rtc_set_time(((0x19>>4)*10)+(0x19&0x0f),	//�⵵
//										 ((0x03>>4)*10)+(0x19&0x0f),	//��
//										 ((0x18>>4)*10)+(0x19&0x0f),  //��
//										 ((0x21>>4)*10)+(0x19&0x0f),  //��
//										 ((0x18>>4)*10)+(0x19&0x0f),  //��
//										 ((0x00>>4)*10)+(0x19&0x0f)); //��
							rtc_time = rtc_get_time();
							printf("RTC DATA : %d �� %d�� %d�� %d�� %d�� %d �� \r\n",
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
			 * SDR ��Ŷ ���� �� Watchdog Update
			 */
			else if(PortNumber==DCU_ALL_MVB_SINK_SDR)
			{
				/*
				 * WatchdogCount ���� ������Ʈ ���θ� ���� MVB ��� ���� üũ
				 */
//				WatchdogCount = (uint16_t)spiRxBuffer[0];
//				WatchdogCount |= ((uint16_t)spiRxBuffer[1]<<8);
//				#ifdef MVB_RX_PACKET_MONITORING
//					debug("Watchdog Count: %d\r\n", WatchdogCount);
//				#endif
				if(PrevWatchdogCount != WatchdogCount)										// ������Ʈ ��  Watchdog Counter ���� OK --> TCMS�� ��Ŷ�� ����� ������ ����
				{
					/*
					 * ó�� ���� �� ������ ������ WatchdogCount�� �������� �ٸ��� ������ ��� �ٷ� ��ü�ϸ� �ȵ�
					 */
					MatchCount++;
					if(MatchCount>6)
					{
						MatchCount=0;
						error_list_time[DCU_MVB_ERROR] = osKernelSysTick();
						#ifdef MVB_RX_PACKET_MONITORING
							debug("Watchdog Count �����ؼ� MVB(F15) �������\r\n");
						#endif
						if(mvbRxData_check_flag == 1)
						{
								Erase_fault(DCU_MVB_ERROR);
								HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_SET);
								HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);
						}
						/*
						 * TCMS-MVB����� ������ ����(Watchdog Update)������ TESTMODE ��Ʈ(��ü���) ���� �� ��ü
						 */
						if((spiRxBuffer[2]&0x80)==0x80)
						{
							/* todo 19.03.11: mvb ī��Ʈ �ϴ°� �߰� �ϼ��� ������ ��ü��Ʈ ���� �̺�Ʈ ����ϰԲ� */
							spiRxBuffer[2] = 0;
							//if(m_isMasterSlaveChange)
	//							debug("TCMS->MVB�κ��� ��ü ��Ʈ ����\r\n");
							m_isMasterSlaveChange = false;
						}
						else
						{
							//if(m_isMasterSlaveChange == false)
	//							debug("TCMS->MVB�κ��� ��ü ���� ��Ʈ ���� \r\n");
							m_isMasterSlaveChange = true;
						}
					}
				}
				else																		// TCMS�� ��Ŷ�� ������ ���� �ʾƼ� ���� ��Ŷ�� ��� ����
				{
					/*
					 * 100�� �ڿ� mvb ��� ����
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
			 * TDR��Ŷ ���� �� 
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
			 * MVB��� ����
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
     * �ֱ������� Master�� Slave �� ����� ���� alive check�� ���� �� �� �ٷ� ����Ǿ�� ��.
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
	 * MVB ����
	 */
	#if 0
	{
		/*
		 * ��ɾ� ���� MVB Configuration
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
		 * DCU ���� MVB Configuration
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
         * MASTER DCU �����ڵ�
         */
        if(m_isMasterSlave == MASTER_DCU)
        {
            /* Place this task in the blocked state until it is time to run again. */
            osDelayUntil(&PreviousWakeTime, 50UL);
            
            /*
             * Slave�� ��Ŷ �۽�(��ü��� ���� ����)
             */
			#if 1
			{
				/*
				 * Slave�� ���� �� ��Ŷ ����
				 *  [0]      [1]       [2]        [3]        [4] [5]   [6] [7]
				 * 0xCC   IndexCnt   �������    isMasterDie     ��������     CheckSum
				 */
				ucSendData[0] = 0xAA;
				ucSendData[1] = PacketCount++;
				ucSendData[2] = mdc_DoorState;
				ucSendData[3] = 0x0;//(uint8_t)m_isSlaveRunCommand;												// ��ŷ������� Slave�� 'm_isSlaveRunCommand=true' ���� ����
				ucSendData[4] = 0x0;
				ucSendData[5] = 0x0;
				CheckSumTxPacket = ucSendData[1]+ucSendData[2]+ucSendData[3]+ucSendData[4]+ucSendData[5];
				ucSendData[6] = (uint8_t)((CheckSumTxPacket&0xFF00)>>8);
				ucSendData[7] = (uint8_t)(CheckSumTxPacket&0x00FF);
				
				/*
				 * Slave�� 8byte ��Ŷ ����
				 */
				if((m_isPacketSendToSlave == true) && (m_is485changeflag == true))																	// ��ü �׽�Ʈ ��
				{
					if(HAL_UART_Transmit(&huart5, (uint8_t*)ucSendData, 8, 2000)!= HAL_OK)					// 7byte ������ ���� (�۽��� ť�� ������� ����)
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
			 * ��Ŷ �۽� �� Slave�� ���� ��Ŷ �ٷ� ���� ����ϰ� ����
			 */
			#if 1
			{
				HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);
				osDelay(3);
				/*
				 * ��ü �� Slave�κ��� SlaveRun ���� ���� �� Master LED Off
				 */
				event = osMessageGet(hRx5Queue, 100);														// ������ �ð� ���� Slave�� ������ �޽��� ���� ���
				if(event.status == osEventMessage)															// Slave�κ��� �޽��� ���� ����
				{
					/*
					 * ��Ŷ�� ���������Ƿ� ī��Ʈ �� 0���� ����
					 */
					NoResponseCnt = 0;
					error_list_flag[DCU_CANT_SLAVE] = false;												// ��·���� ��Ŷ�� ���������ϱ� Slave�� ��� �ִٰ� ���� ��
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
						 * Slave�� �۽��� 8byte ��Ŷ ���������� ����
						 */
						if((CheckSumHigh==aUartRx5Buffer[6]) && (CheckSumLow==aUartRx5Buffer[7]))
						{
							if(aUartRx5Buffer[3]==true)														// Slave�� ��ü�ߴٰ� �˷��ָ�
							{
								m_isSlaveRunCommand = true;
								m_isPacketSendToSlave = true;												// �����̺꿡�� ��ü�����Ƿ� �����ʹ� �ٽ� ��Ŷ�� ������.
								mip_MasterCodeMasterLED(0);													// Master�� ������ ���� ��
								mip_SleveCodeSlaveRun(1);													// ������ fnd�� ��� ������� �����̺�� �ѱ��.
								//printf("loop in 2 \r\n");
							}
						}
						else
						{
							debug("## [alive] ������ Slave��Ŷ ����\r\n");
							HAL_UART_Init(&huart5);
						}
					}
					else
					{
						HAL_UART_Init(&huart5);
						//������ �� ó��
						osDelay(1);
						//HAL_UART_Receive(&huart1, (uint8_t *)aUartRx5Buffer, 11,5);
					}
				}
				else																						// Slave�κ��� �޽��� ���� ����
				{
					NoResponseCnt++;
					if(NoResponseCnt>60) //jeon_190715 orig:10 0.5s => 3s
					{
						debug("## [alive] Slave��Ŷ �̼��� (Slave Die)\r\n");
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
         * SLAVE DCU �����ڵ�
         */
        else
        {
            /* Place this task in the blocked state until it is time to run again. */
            osDelayUntil(&PreviousWakeTime, 50UL);
            
            /*
             * Master�� ���� ��Ŷ Slave���� 
             */
			#if 1
			{
				//���� ���־� �����̺� CPU �Ȼ��� ���� �м��ؾ���
            	HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);
            	osDelay(3);
				/*
				 * Master�κ��� 8byte ��Ŷ �����ϸ� Rx5 ���� ���ͷ�Ʈ���� �޽��� Put
				 */
				event = osMessageGet(hRx5Queue, 100);
				if(event.status == osEventMessage)												// Master�κ��� 8byte�� �� ������� �̺�Ʈ �߻�
				{
					/*
					 * ��Ŷ�� ���������Ƿ� ī��Ʈ �� 0���� ����
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
						 * Master�� �۽��� 8byte ��Ŷ ���������� ����
						 */
						if((CheckSumHigh==aUartRx5Buffer[6]) && (CheckSumLow==aUartRx5Buffer[7]))
						{
							mdc_InitDoorState = aUartRx5Buffer[2];									// Master�� ���� ���� �������� Slave�� ����
							if(m_isSlaveRunCommand == false) m_isSlaveRunCommand = aUartRx5Buffer[3];							// Master�� ���� 'm_isSlaveRunCommand=true' ���� ���� -> Slave ���� ����

						}
						else
						{
							HAL_UART_Init(&huart5);
							debug("## [alive] ������ Master��Ŷ ����\r\n");
						}
					}
					else
					{
						//������ �� ó��
						osDelay(1);
						HAL_UART_Init(&huart5);
					}
				}
				else																			// Master�κ��� �޽��� ���� ����
				{
					NoResponseCnt++;
					#if 1
					if(NoResponseCnt>20)	//jeon_190715	orig:20 1s => 2s 													// �����Ϳ��� 2�� ���� ������ ���� ��� �ٷ� �����̺갡 �����´�.
					#else
					if(NoResponseCnt>500)														// �ٿ�ε��� �� �ڲ� ��ü �Ǽ� �׽�Ʈ �ÿ��� �ð� �÷� ����
					#endif
					{
						//debug("## [alive] Master��Ŷ �̼��� (Master Die)\r\n");
						m_isSlaveRunCommand = true;												// Master�κ��� ��Ŷ �̼��� �� Slave�� ������ ��ü��� �� -> DecisionControl���� ��ü ����
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
			 * Master�� ��Ŷ �۽�(��ü��� ���� ����)
			 */
			#if 1
			{
				/*
				 * Master�� ���� �� ��Ŷ ����
				 */
				ucSendData[0] = 0xBB;
	            ucSendData[1] = PacketCount++;
	        	ucSendData[2] = 0x0;
	        	ucSendData[3] = (uint8_t)m_isSlaveRunCommand;												// ��ŷ������� Slave�� 'm_isSlaveRunCommand=true' ���� ����
	        	ucSendData[4] = 0x0;
	        	ucSendData[5] = 0x0;
	        	CheckSumTxPacket = ucSendData[1]+ucSendData[2]+ucSendData[3]+ucSendData[4]+ucSendData[5];
	        	ucSendData[6] = (uint8_t)((CheckSumTxPacket&0xFF00)>>8);
	        	ucSendData[7] = (uint8_t)(CheckSumTxPacket&0x00FF);
	        	
	        	/*
	        	 * Master�� 8byte ��Ŷ ����
	        	 */
	        	if(HAL_UART_Transmit(&huart5, (uint8_t*)ucSendData, 8, 2000)!= HAL_OK)		// 7byte ������ ���� (�۽��� ť�� ������� ����)
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
		 * todo : ����ȯ ���ϰ� �ٷ� int�� ������ �����ϴµ� �����м� �� �� �������� ����� ���� ��
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

	if(cnt_rcv == 1)									//�ַܼ� ��ɾ� ġ�� ���
	{
		if(ucOffset > 126) ucOffset = 0;
		ucTemp = Dequeue(&rx_Int_queue);
		ucString[ucOffset] = ucTemp;

		if((ucTemp == 0x0D) || (ucTemp == 0x0A))	// 0x0D(Carriage Return), 0x0A(Line Feed)
		{
			/*
			 * ���� �Է� �� ����Ű ����
			 */
			if(ucOffset != 0u)
			{
				printf("\r\n\r\n");
				CommandExcute(ucString);
				printf("\r\ndcu-test $ \r\n");
				fflush(stdout);
			}
			/*
			 * ���� �Է� ���� ����Ű ����
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
		 * BackSpace�� ���� ��� �ѱ��� ����
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
	else																		//���� ���� ���α׷����� ����ϴ� ���
	{
		for(i=0; i<cnt_rcv; i++)
		{
			tx_buf[i] = Dequeue(&rx_Int_queue);
		}
		if((m_isSlaveRunCommand == true) && (m_isMasterSlave == MASTER_DCU))
		{
			return;
		}
		else if((m_isSlaveRunCommand == false) && (m_isMasterSlave == SLAVE_DCU)) //������� ���� ��� �����������α׷��� ������� ���ϰ� ��
		{
			return;
		}
		if((tx_buf[0] == 0x01) && (tx_buf[1] == 0x70))							//PC -> DCU ���� ���� ��û
		{
			/*
			 * DCU ������ CRC üũ�� �ؾ���
			 * PC���� �����ִ� ������ 2����Ʈ(CRC 1/2)�� FIX������ ����
			 * �׷��Ƿ� DCU�� ������ 2BYTE�� ������ ��� �����͸� CRC üũ�� �� �ڿ�
			 * PC���� ������ crc 2����Ʈ�� ���� ������ ������������ �����Ѵ�.
			 */
			uint16_t rx_crc = 0;
			uint8_t rx_buf[2] = {0,0};
			rx_crc = CalcCrc(tx_buf,(int32_t)(cnt_rcv-2));						//crc 2����Ʈ�� �����ϰ� ���������͵� crc üũ�Ѵ�
			rx_buf[0] = (uint8_t)(((rx_crc) & 0xFF00) >> 8);
			rx_buf[1] = (uint8_t)((rx_crc) & 0x00FF);
			/*
			 * �� 8���� �����Ͱ� �����ϱ� cnt_rcv���� 8�̴�. 6,7��° �迭�� crc üũ�� �ؾ��Ѵ�.
			 * ex ) 01 70 01 03 04 01 / 02 03     : pc ���� ������ �� ���� 2����Ʈ�� crc
			 *      01 70 02 00 01 01 : dcu�� ������ �������� �̻��� ���� ������ crc üũ �Ҷ� �ش� ���� Ʋ�� ���� �ް� �ȴ�.
			 */
			if((rx_buf[0] == tx_buf[(cnt_rcv-2)]) && (rx_buf[1] == tx_buf[(cnt_rcv-1)]))
			{
				StateData_Send(tx_buf);
			}
		}
		else if((tx_buf[0] == 0x01) && (tx_buf[1] == 0x80))						//pc -> DCU ���� ���,��������� �޸� ���� ����
		{
			uint16_t rx_size = 0;
			uint16_t rx_crc = 0;
			uint8_t rx_buf[2] = {0,0};
			rx_crc = CalcCrc(tx_buf,(int32_t)(cnt_rcv-2));						//crc 2����Ʈ�� �����ϰ� ���������͵� crc üũ�Ѵ�
			rx_buf[0] = (uint8_t)(((rx_crc) & 0xFF00) >> 8);
			rx_buf[1] = (uint8_t)((rx_crc) & 0x00FF);
			if((rx_buf[0] == tx_buf[cnt_rcv-2]) && (rx_buf[1] == tx_buf[cnt_rcv-1]))
			{
				rx_size |= (uint16_t)(tx_buf[5] & 0x00ff) << 8;
				rx_size |= (uint16_t)tx_buf[4];
				MemHandling_data(tx_buf,rx_size);
			}
		}
		else if((tx_buf[0] == 0x01) && (tx_buf[1] == 0xFF))					   // pc <-> DCU ���� ����
		{
			uint16_t rx_crc = 0;
			uint8_t rx_buf[2] = {0,0};
			rx_crc = CalcCrc(tx_buf,(int32_t)(cnt_rcv-2));
			rx_buf[0] = (uint8_t)(((rx_crc) & 0xFF00) >> 8);
			rx_buf[1] = (uint8_t)((rx_crc) & 0x00FF);
			if((rx_buf[0] == tx_buf[cnt_rcv-2]) && (rx_buf[1] == tx_buf[cnt_rcv-1]))
			{
				if(tx_buf[3] == 0x01)									//������ ���� ��ɾ�
				{
					Comm_Fault_tx(0);
				}
				else if(tx_buf[3] == 0x03)								//���� Ȯ�� ��ɾ�
				{
					uint16_t Block_Number =0;
					Block_Number = tx_buf[4] +1;						// ���� �� �������� ���� �Ѵ�.
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
	uint8_t SlaveDcuAddress[7] = {DCU_L2_MCAR,DCU_L3_MCAR,DCU_L4_MCAR,DCU_R1_MCAR,DCU_R2_MCAR,DCU_R3_MCAR,DCU_R4_MCAR};	//slave dcu���� ���� ID���� �����س��´�.
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
	aTxBuffer[1] = 0x00;					// ���� ����
	aTxBuffer[2] = Packet_Number_count;		// 4��Ŷ �ѹ� ī��Ʈ
	aTxBuffer[3] = 0x01;					// üũ�� high
	aTxBuffer[4] = 0x00;					// üũ�� low
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
		 * RS485_Master_DCU_L1�� ��� RS485_Slave_DCU_L2~L4���� ������ ��û
		 */
		/*
		 * RS485_Master_DCU_L1�� RS485_Slave_DCU_L2 RS485_Slave_DCU_L3 RS485_Slave_DCU_L4�� ������ ����
		 */
		if(g_unDeviceID == DCU_L1_MCAR)
		{
			if(address_count > 6) address_count = 0;
			aTxBuffer[0] = 0xCC;											//start packet
			aTxBuffer[1] = SlaveDcuAddress[address_count];					// ��û�� slave�� id ����
			aTxBuffer[2] = Packet_Number_count;								// 0~255 ī��Ʈ �ϴ� ���� ����
			checksum_value = (aTxBuffer[0] + aTxBuffer[1] + aTxBuffer[2]);	//  [0] , [1] , [2] �� ���� ���� ����
			aTxBuffer[3] = ((checksum_value & 0xFF00) >>8);					//  ���� ����Ʈ�� 3 ����Ʈ�� ����
			aTxBuffer[4] = (checksum_value & 0x00FF);						//  ���� ����Ʈ�� 4 ����Ʈ�� ���� ���� ����
			
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
						/* CC + �������� + CHECK SUM HIGH + CHECK SUM LOW */
						checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
						Master_up_data = ((checksum_value & 0xFF00) >> 8);
						Master_down_data =(checksum_value & 0x00FF);
						if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //üũ�� �� ������
						{
							if((aUartRx1Buffer[1] == DCU_L2_MCAR) || (aUartRx1Buffer[1] == DCU_L3_MCAR) || (aUartRx1Buffer[1] == DCU_L4_MCAR))
							{
								//�̺κ��� mvb�� ������ �����͸� ����� ��
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
					// L2 L3 L4 �� �����͸� ������ �ֵ��� ����Ѵ�.
					if((aTxBuffer[1] == DCU_L2_MCAR) || (aTxBuffer[1] == DCU_L3_MCAR) || (aTxBuffer[1] == DCU_L4_MCAR))
					{
						//11byte�� �����Ͱ� �ö� ���� ��� �Ѵ�
						while(HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11) != HAL_OK)
						{

							loopcnt++;
							if(loopcnt >7)
							{

								//debug("master test \r\n");
								loopcnt = 0;
								break;
							}
							vTaskDelay(10);									// 11byte ������ ���� �ʾ����� break
						}
					}

				}
//				HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
				address_count++;
			}
			Packet_Number_count++;
		}
		else if(g_unDeviceID == DCU_R1_MCAR)	//R1�� DCU��� ������ �� �ִ� R2 R3 R4�� �������� �����Ѵ�.
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
						/* CC + �������� + CHECK SUM HIGH + CHECK SUM LOW */
						checksum_value = (aUartRx1Buffer[0]+aUartRx1Buffer[1]+aUartRx1Buffer[2]+aUartRx1Buffer[3]);
						Master_up_data = ((checksum_value & 0xFF00) >> 8);
						Master_down_data =(checksum_value & 0x00FF);
						if((Master_up_data == aUartRx1Buffer[9]) && (Master_down_data == aUartRx1Buffer[10])) //üũ�� �� ������
						{
							if((aUartRx1Buffer[1] == DCU_R2_MCAR) || (aUartRx1Buffer[1] == DCU_R3_MCAR) || (aUartRx1Buffer[1] == DCU_R4_MCAR))
							{
								//�̺κ��� mvb�� ������ �����͸� ����� ��
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
		else if(g_unDeviceID != DCU_L1_MCAR)	//slave dcu ��� �ڽ��� id ��û�� ������ ��� ������ ������.
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
						if(aUartRx1Buffer[1] == g_unDeviceID) // �����Ϳ��� �ڽ��� ��û�� ���
						{
							aTxBuffer[0] = 0xCC;
							aTxBuffer[1] = g_unDeviceID;
							aTxBuffer[2] = Statement_DCU(2);	 // ���� ����
							aTxBuffer[3] = Statement_DCU(3);	 // ���� ���� 2
							aTxBuffer[4] = FalutStatement_DCU(4);// ���� ����
							aTxBuffer[5] = FalutStatement_DCU(5);// ���� ���� 2
							aTxBuffer[6] = 0x36;				 // ����Ʈ���� ����
							aTxBuffer[7] = 0x97;				 // ���� ����
							aTxBuffer[8] = 0x55;				 // ���� �ð�
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
