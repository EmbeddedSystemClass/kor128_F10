/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtMonitoring
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMonitoring.c
*********************************************************************/

#include "mtMonitoring.h"
/* ���� ����(Opening/Closing)�� ���� ADC���� ������ �����ϱ� ���� ��Ŭ��� */
/*## dependency mtDecisionControl */
#include "mtDecisionControl.h"
#include "mtInputProcessing.h"
#include "mtMotorFeedback.h"
#include "mMotorOut.h"
#include "mtObstacleDetect.h"
#include "mDataManage.h"

extern osThreadId obstackedetectTaskHandle;
extern osThreadId inputTaskHandle;
extern osThreadId decisioncontrolTaskHandle;
extern osThreadId displayfndTaskHandle;

struct adcResults g_afec0_ch4 = {0,false};	/* PB0(AFEC0_AD4) - �������� */
struct adcResults g_afec0_ch5 = {0,false};	/* PB1(AFEC0_AD5) - BEMF����(��) */
struct adcResults g_afec0_ch6 = {0,false};	/* PB2(AFEC0_AD6) - BEMF����(��) */
struct adcResults g_afec0_ch8 = {0,false};	/* PB8(AFEC0_AD8) - 3.3V Monitor */
struct DebugPrint debugprint = {0};
uint8_t mram_fnd_data = 0;					/* MRAM�� �������� FND ������ ���� ���� ����*/
uint8_t mram_fnd_dataB = 0;
uint8_t mram_fnd_dataC = 0;
extern struct _detect mod_Detect;
extern _Bool mdc_FlagSlowMode;

/*## attribute m_FND */
/*
 *                    [0]   [1]  [2]  [3]  [4]  [5]  [6]  [7]  [8]  [9] [10] [11] [12] [13] [14] [15] [16] [17] [18] [19] [20]
 */
/*
 *                     0,    1,   2,   3,   4,   5,   6,   7,   8,   9,   F,   a,   b,   V,   E,   r,   R    L    M    T  -
 */
uint8_t m_FND[21] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x27,0x7F,0x67,0x71,0x5F,0x7C,0x3E,0x79,0x50,0x77,0x38,0x37,0x07,0x40};

void mm_ReadFaultDcuData(void)
{
	uint8_t Trace_count = 0;
	uint8_t read_data = 0;
	int i,j = 0;
	Trace_count = mram_byte_read(MRAM_FAULT_ADDR);
	debug("trace_count = %d \r\n",Trace_count);
	for(i=0; i<Trace_count; i++)
	{
		debug("=================================================\r\n");
		for(j=1; j<61; j++)
		{
			read_data = mram_byte_read(MRAM_FAULT_ADDR+(i)*0x64+j);
			debug(" %x ",read_data);
		}
		debug("\r\n==================================================\r\n");
	}
}

void mm_SaveFaultDcuData(uint8_t ErrorCode)
{
	/*
	 *  1 block  = 100 byte:
	 */
	uint8_t Trace_count = 0;														// ���� ����(=������)�� �ľ��ϴ� ����
	uint8_t save_data = 0;
	/*
	 * ���� ������ �����ϴ� rtc data , ��������(rtc_time)�� �����ϰ� �Ǹ�
	 * hex to demical , demical to hex ������ �ڵ带 �߰��ؾ��ؼ� �ڵ尡 �������� ��
	 */
	RTCTIME fault_rtcdata;
	int i,j = 0;
	
	fault_rtcdata = rtc_get_time();
	
	if(((fault_rtcdata.ucMonth<1)||(fault_rtcdata.ucMonth>12)) || ((fault_rtcdata.ucDate<1)||(fault_rtcdata.ucDate>31)))
	{
		fault_rtcdata = rtc_get_time();
		
		if(((fault_rtcdata.ucMonth<1)||(fault_rtcdata.ucMonth>12)) || ((fault_rtcdata.ucDate<1)||(fault_rtcdata.ucDate>31)))
		{
			fault_rtcdata = rtc_get_time();
		}

	}
	
	mm_SaveCurrentDcuData();														//���� dcu�� ���¸� ���� �Ѵ�. ���� ����� ���¶� �ֱ������� �����ϴ� ���¶� ���� �ʴ� ��쵵 �߻���
	Trace_count  = mram_byte_read(MRAM_FAULT_ADDR);									// ���� ������ �о� �´�.(��� ����)

	if(Trace_count >= 100u)															// ���� ������ 100���� �Ѿ� �� ���
	{
		for(i=0; i<Trace_count; i++)
		{
			for(j=1; j<61; j++)
			{
				save_data = mram_byte_read(MRAM_FAULT_ADDR+(i+1)*0x64+j);			//ù��° �����͸� ����� �״��� �����͸� ����ϹǷ� �����͸� �ҷ� �´�.
				if(j==2)	mram_byte_write(MRAM_FAULT_ADDR+(i)*0x64+j,i);			//�ι�° ����Ʈ�� ù��° ���� ������ ���� �Ѵ�.
				else		mram_byte_write(MRAM_FAULT_ADDR+(i)*0x64+j,save_data);	//�����͸� ����
			}
		}
		Trace_count = 99;
	}

	mram_byte_write(MRAM_FAULT_ADDR,Trace_count+1);									// ���� ������ �����Ѵ�.
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+1,ErrorCode);				// ex) trace_count = 1 : 100+1������ ���� �ڵ� ����
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+2,Trace_count);				// ��ü ���� ������ ���� �Ѵ�.
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+3,fault_rtcdata.ucYears);						// years	���߿� rtc ������ �޾Ƽ� �־����
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+4,fault_rtcdata.ucMonth);						// month
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+5,fault_rtcdata.ucDate);						// date
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+6,fault_rtcdata.ucHours);						// hours
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+7,fault_rtcdata.ucMinutes);						// miniute
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+8,fault_rtcdata.ucSeconds);						// second

	for(i=0; i<53; i++)
	{
		save_data = mram_byte_read(MRAM_DATA_ADDR+i);									//���� �������� �д´�.
		mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+9+i,save_data);				//����� 9��° ����Ʈ ���� ���� �������� �����Ѵ�.
	}
}

void mm_SaveCurrentDcuData(void)
{
	uint8_t Temp_DCU_Data = 0;
	uint8_t Trace_count  = 0;
	uint8_t SWitch_POWER_data =0;
	int32_t RealPosition =0;
	uint16_t powervoltage=0;
	Temp_DCU_Data = 0;

	m_isFaultCount = mram_byte_read(MRAM_FAULT_ADDR);				//Dcu�� ���� ������ ����

	mram_byte_write(MRAM_DATA_ADDR,g_unDeviceID);
	/* DCU OK ������ �Ķ���� �����̳�, �ʱ�ȭ ������ �Ϸ� �Ǹ� SET ��Ű���� ����*/
	Temp_DCU_Data = (error_list_flag[DCU_HARD_FAULT] == false)?0x80:0x00;
	mram_byte_write(MRAM_DATA_ADDR+1,Temp_DCU_Data);

	Temp_DCU_Data =0;
	Temp_DCU_Data |= ((mip_Input.DoorClosedSwitchOn == false)?0x01:0x00);
	Temp_DCU_Data |= ((mip_Input.di0_Isolation == true)?0x01:0x00)<<1;
	Temp_DCU_Data |= (((mdc_DoorState == DOOR_OPENED) && (error_list_flag[DCU_OBSTACLE] == true))?0x01:0x00)<<2;			//���� ���� ���� ����
	Temp_DCU_Data |= ((error_list_flag[DCU_ERROR] == true)?0x01:0x00)<<3;													//���� ���� ����
	Temp_DCU_Data |= ((mod_Detect.ObstacleDetectCnt > 0)?0x01:0x00)<<4;														//��ֹ� ���� ����
	Temp_DCU_Data |= ((mip_Input.di1_EED)?0x01:0x00)<<5;
	Temp_DCU_Data |= ((mip_Input.di1_EAD)?0x01:0x00)<<6;
	mram_byte_write(MRAM_DATA_ADDR+2,Temp_DCU_Data);

//	Temp_DCU_Data = 0;
//	Temp_DCU_Data |= ((mip_Input.di0_OpenCloseButton)?0x01:0x00)<<5;
//	Temp_DCU_Data |= ((!mip_Input.di0_OpenCloseButton)?0x01:0x00)<<6;
//	mram_byte_write(MRAM_DATA_ADDR+3,Temp_DCU_Data);

	Temp_DCU_Data = 0;
	Temp_DCU_Data |= ((error_list_flag[DCU_HARD_FAULT])?1:0);
	Temp_DCU_Data |= ((error_list_flag[DCU_MINOR_FAULT])?1:0)<<1;
	Temp_DCU_Data |= ((error_list_flag[DCU_MOTOR_ERROR])?1:0)<<2;
	Temp_DCU_Data |= ((error_list_flag[DCU_DLS1_FAULT])?1:0)<<3;
	Temp_DCU_Data |= ((error_list_flag[DCU_DCS1_FAULT])?1:0)<<4;
	Temp_DCU_Data |= ((error_list_flag[DCU_DCS2_FAULT])?1:0)<<5;
	Temp_DCU_Data |= ((error_list_flag[DCU_CANT_SLAVE])?1:0)<<6;
	Temp_DCU_Data |= ((error_list_flag[DCU_SLAVE_RUN])?1:0)<<7;
	mram_byte_write(MRAM_DATA_ADDR+4,Temp_DCU_Data);

	Temp_DCU_Data = 0;
	Temp_DCU_Data |= ((error_list_flag[DCU_UNEXPECTED_UNLOCK])?1:0);
	Temp_DCU_Data |= ((error_list_flag[DCU_DLS2_FAULT])?1:0)<<1;
	Temp_DCU_Data |= ((error_list_flag[DCU_OBSTACLE])?1:0)<<2;
	Temp_DCU_Data |= ((error_list_flag[DCU_OPEN_FAULT])?1:0)<<3;
	Temp_DCU_Data |= ((error_list_flag[DCU_ENCODER_ERROR])?1:0)<<4;
	Temp_DCU_Data |= ((error_list_flag[DCU_SAFETYLOOP_FAULT])?1:0)<<5;
	mram_byte_write(MRAM_DATA_ADDR+5,Temp_DCU_Data);

	mram_byte_write(MRAM_DATA_ADDR+6,0*10+0);				// ROM Version

	Trace_count = mram_byte_read(MRAM_FAULT_ADDR);			// mram�� ������ ���� ����
	mram_byte_write(MRAM_DATA_ADDR+7,Trace_count);

	if(HAL_GPIO_ReadPin(SET_MASTER_SLAVE_GPIO_Port, SET_MASTER_SLAVE_Pin)) mram_byte_write(MRAM_DATA_ADDR+8,0x22);
	else mram_byte_write(MRAM_DATA_ADDR+8,0x12);

	mram_byte_write(MRAM_DATA_ADDR+9,VERSION_MINOR);
	mram_byte_write(MRAM_DATA_ADDR+10,VERSION_MAJOR);

	mram_byte_write(MRAM_DATA_ADDR+11,mram_fnd_data);				//fnd �� �ѷ����� ��
	mram_byte_write(MRAM_DATA_ADDR+12,mram_fnd_dataB);
	mram_byte_write(MRAM_DATA_ADDR+13,mram_fnd_dataC);

	//mram_byte_write(MRAM_DATA_ADDR+14,*(unsigned char*)(0x64000004));			//���͸� ����ġ

	if(mdc_DoorState == DOOR_CLOSING) powervoltage =(uint16_t)(m_adc.MotorVoltage / 31);
	else if(mdc_DoorState == DOOR_OPENING) powervoltage =(uint16_t)(m_adc.PowerVoltage / 31);
	else powervoltage = 100;

	mram_byte_write(MRAM_DATA_ADDR+16, powervoltage);										//14,15 dcu ���� ����(100v)
	mram_byte_write(MRAM_DATA_ADDR+17, powervoltage>>8);

	mram_byte_write(MRAM_DATA_ADDR+18,mmf_Encoder.Position);
	mram_byte_write(MRAM_DATA_ADDR+19,mmf_Encoder.Position >>8);

	mram_byte_write(MRAM_DATA_ADDR+20,m_adc.MotorCurrent);
	mram_byte_write(MRAM_DATA_ADDR+21,m_adc.MotorCurrent>>8);

	mram_byte_write(MRAM_DATA_ADDR+22,m_adc.MCUVoltage);
	mram_byte_write(MRAM_DATA_ADDR+23,m_adc.MCUVoltage>>8);

	Temp_DCU_Data = 0;
	Temp_DCU_Data |= ((mip_Input.di0_OpenCloseButton)?1:0);
	Temp_DCU_Data |= ((mip_Input.di0_ReOpen)?1:0)<<1;
	Temp_DCU_Data |= ((mip_Input.di0_ZVR)?1:0)<<2;
	Temp_DCU_Data |= ((mip_Input.di0_Isolation)?1:0)<<3;
	Temp_DCU_Data |= ((mip_Input.di0_Bypass)?1:0)<<4;
	Temp_DCU_Data |= ((mip_Input.di0_SafetyLoopA)?1:0)<<5;
	Temp_DCU_Data |= ((mip_Input.di0_SafetyLoopB)?1:0)<<6;
	Temp_DCU_Data |= ((mip_Input.di0_DLS1)?1:0)<<7;
	mram_byte_write(MRAM_DATA_ADDR+28,Temp_DCU_Data);

	Temp_DCU_Data = 0;
	Temp_DCU_Data |= ((mip_Input.di1_DCS2)?1:0);
	Temp_DCU_Data |= ((mip_Input.di1_DCS1)?1:0)<<1;
	Temp_DCU_Data |= ((mip_Input.di1_EED)?1:0)<<2;
	Temp_DCU_Data |= ((mip_Input.di1_EAD)?1:0)<<3;
	Temp_DCU_Data |= ((mip_Input.di1_DLS2)?1:0)<<4;
	mram_byte_write(MRAM_DATA_ADDR+29,Temp_DCU_Data);

	/*
	 * DO_0(FMC_OUTEN0) - MDO_CTL1~6 ��� -> ��������, 0111.1001 = 0x79 (��� ���� ����)
	 * -------------------------------------------------------------------------------------------
	 * | [7]  |    [6]     |     [5]     |   [4]    |   [3]     |     [2]    |   [1]  |   [0]    |
	 * |------|------------|-------------|----------|-----------|------------|--------|----------|
	 * |      |���ܽ���ġ����  |EAD-EED Power|DCS1 Power|DCS2 Power	| Disconnect |Reserved|����|
	 * -------------------------------------------------------------------------------------------
	 */
	Temp_DCU_Data =0;
	SWitch_POWER_data = FMC_OUTEN0;
	//Temp_DCU_Data |= (SWitch_POWER_data & 0b00000001);						//���� ����
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b00001000)>>1);					//dcs 2 ���� ����
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b00010000)>>1);					//dcs 1 ���� ����
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b00100000)>>1);					//eed-ead ���� ����
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b01000000)>>1);					//���� ����ġ ���� ����
	mram_byte_write(MRAM_DATA_ADDR+31,~SWitch_POWER_data);						// 0xff ��� -> ���� ���� On(����)

	Temp_DCU_Data = 0;
	Temp_DCU_Data |= ((mip_Input.di0_OpenCloseButton)?1:0);
	Temp_DCU_Data |= ((mdc_DoorState == DOOR_OPENING)?1:0)<<1;
	Temp_DCU_Data |= ((mdc_DoorState == DOOR_OPENED)?1:0)<<2;
	mram_byte_write(MRAM_DATA_ADDR+33,Temp_DCU_Data);

	Temp_DCU_Data = 0;
	Temp_DCU_Data |= ((!mip_Input.di0_OpenCloseButton)?1:0);
	Temp_DCU_Data |= ((mdc_DoorState == DOOR_CLOSING)?1:0)<<1;
	Temp_DCU_Data |= ((mdc_DoorState == DOOR_CLOSED)?1:0)<<2;
	mram_byte_write(MRAM_DATA_ADDR+34,Temp_DCU_Data);

	mram_byte_write(MRAM_DATA_ADDR+35,mdm_time.Opening);
	mram_byte_write(MRAM_DATA_ADDR+36,mdm_time.Opening>>8);

	mram_byte_write(MRAM_DATA_ADDR+37,mdm_time.Closing);
	mram_byte_write(MRAM_DATA_ADDR+38,mdm_time.Closing>>8);

	mram_byte_write(MRAM_DATA_ADDR+39,(uint8_t)Current_Velocity_value);						//�ӵ�
	mram_byte_write(MRAM_DATA_ADDR+40,(uint8_t)(Current_Velocity_value>>8));

	RealPosition = (mmf_Encoder.Position * 0.18);
	mram_byte_write(MRAM_DATA_ADDR+41,(uint8_t)(RealPosition));						//�Ÿ��� ȯ��
	mram_byte_write(MRAM_DATA_ADDR+42,(uint8_t)((RealPosition)>>8));

	Temp_DCU_Data =0;
	Temp_DCU_Data |= (mip_Input.CommandTestMode == true)?0x80:0x00;
	mram_byte_write(MRAM_DATA_ADDR+43,Temp_DCU_Data); // 8bit�� ������ ���� �����������α׷� ������ ��ư�� ���� ����
	#if 0
	debug("\r");
	for(int i =0; i<43; i++)
	{
		debug("data : [%d] - [%x]\r\n",i,mram_byte_read(MRAM_DATA_ADDR+i));
		osDelay(10);
	}
	#endif
}

void DisplayMonitoringInfo(uint32_t info)
{
    switch(info)
    {
    	case 0x01:
    		debug("Error Watchdog Configuration\r\n");
    		break;
    	default:
    		break;
    }
}

void DisplaySWVersionToFND(uint8_t major, uint8_t minor, uint8_t second)
{
    uint8_t cnt=0;
    
    for(cnt=0; cnt<3; cnt++)
    {
		FMC_SEG_DISPLAY1 = m_FND[FND_V];
		FMC_SEG_DISPLAY2 = m_FND[FND_E];
		FMC_SEG_DISPLAY3 = m_FND[FND_R];
		FMC_SEG_DISPLAY1 = m_FND[FND_V];
		FMC_SEG_DISPLAY2 = m_FND[FND_E];
		FMC_SEG_DISPLAY3 = m_FND[FND_R];
		osDelay(250);
		FMC_SEG_DISPLAY1 = 0x0;
		FMC_SEG_DISPLAY2 = 0x0;
		FMC_SEG_DISPLAY3 = 0x0;
		FMC_SEG_DISPLAY1 = 0x0;
		FMC_SEG_DISPLAY2 = 0x0;
		FMC_SEG_DISPLAY3 = 0x0;
		osDelay(250);
		FMC_SEG_DISPLAY1 = m_FND[major];
		FMC_SEG_DISPLAY2 = m_FND[minor];
		FMC_SEG_DISPLAY3 = m_FND[second];
		FMC_SEG_DISPLAY1 = m_FND[major];
		FMC_SEG_DISPLAY2 = m_FND[minor];
		FMC_SEG_DISPLAY3 = m_FND[second];
		osDelay(250);
		FMC_SEG_DISPLAY1 = 0x0;
		FMC_SEG_DISPLAY2 = 0x0;
		FMC_SEG_DISPLAY3 = 0x0;
		FMC_SEG_DISPLAY1 = 0x0;
		FMC_SEG_DISPLAY2 = 0x0;
		FMC_SEG_DISPLAY3 = 0x0;
		osDelay(250);
    }
}

void Display3FND(uint8_t fndsel_1, uint8_t fndsel_2, uint8_t fndsel_3, uint32_t delay)
{
	mram_fnd_data    = fndsel_1;
	mram_fnd_dataB   = fndsel_2;
	mram_fnd_dataC   = fndsel_3;
	FMC_SEG_DISPLAY1 = fndsel_1;
	FMC_SEG_DISPLAY2 = fndsel_2;
	FMC_SEG_DISPLAY3 = fndsel_3;
	FMC_SEG_DISPLAY1 = fndsel_1;
	FMC_SEG_DISPLAY2 = fndsel_2;
	FMC_SEG_DISPLAY3 = fndsel_3;
	osDelay(delay);
	FMC_SEG_DISPLAY1 = 0;
	FMC_SEG_DISPLAY2 = 0;
	FMC_SEG_DISPLAY3 = 0;
	FMC_SEG_DISPLAY1 = 0;
	FMC_SEG_DISPLAY2 = 0;
	FMC_SEG_DISPLAY3 = 0;
	osDelay(500);
}

void md_TaskDisplayFND(void const * argument)
{
	uint8_t fndSel1=0, fndSel2=0, fndSel3=0;
	uint8_t error=0, error_cnt=0;

	if(m_isMasterSlave == MASTER_DCU)
	{
	    DisplaySWVersionToFND(VERSION_MAJOR,VERSION_MINOR,VERSION_SECOND);
	}
	else // SLAVE_DCU
	{
    	while(m_isTaskExecution == false)
    	{
    		osDelay(200);
    		//printf("fnd task waiting.. \r\n");
    	}
    	printf("FND Task ���� \r\n");
	    DisplaySWVersionToFND(VERSION_MAJOR,VERSION_MINOR,VERSION_SECOND);
	}
	
	for(;;)
	{
    	if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
    	{
    		osDelay(1000);
    		//debug("# Master input Task ���� ����\r\n");
    	}
    	else
    	{
    		/*
    		 * �����ڵ� or 888 ���
    		 */
    		if(error_list_flag[DCU_ERROR] == true)						// ������ �߻��� ������ �����÷��״� true�� ���� ��
    		{
    			/*
    			 * F01���� F16���� ���� ��ĵ
    			 */
    			error_cnt=0;
    			for(error=1;error<17;error++)
    			{
    				/*
    				 * �߻��� ������ ������ ���������� �߻��� ���� FND Display
    				 */
    				if(error_list_flag[error] == true)
    				{
    					error_cnt++;
    					/*
    					 * FND �ѹ��ѷ��� �� �ȵǴ°� ���Ƽ� �ι��� �Ѹ�
    					 */
    					Display3FND(m_FND[FND_F], m_FND[error/10], m_FND[error%10], 500);
    				}
    			}
    			/*
    			 * F01~F16���� ������ ��ĵ�ߴµ� �߻��� ������ 0���� �����÷��� false�� ����
    			 */
    			if(error_cnt==0)
    			{
    				error_list_flag[DCU_ERROR] = false;
    			}
    		}
        	else
        	{
				/*
				 * ���� FND ���
				 */
				Display3FND(m_FND[FND_8], m_FND[FND_8], m_FND[FND_8], 1000);
        	}
    		
    		/*
    		 * DCU ID ���� ���
    		 */
    		/*
    		 * Slave ǻ�� �� ���¿��� ���� �ΰ� �� RotarrySwitch 0���� ����
    		 */
			if(dcuID.McarTcarPin==0x00)		fndSel1 = FND_M;							// MCar
			else if(dcuID.McarTcarPin==0x01)	fndSel1 = FND_T;							// TCar
			else								{dcuID.McarTcarPin=0;fndSel1 = FND_NONE;}	// No Display
			switch(dcuID.CodingPin)
			{
				case 0x01: fndSel2=FND_R;fndSel3=FND_1; break;					// DCU_R1_MCAR
				case 0x06: fndSel2=FND_R;fndSel3=FND_2; break;					// DCU_R2_MCAR
				case 0x0A: fndSel2=FND_R;fndSel3=FND_3; break;					// DCU_R3_MCAR
				case 0x02: fndSel2=FND_R;fndSel3=FND_4; break;					// DCU_R4_MCAR
				case 0x04: fndSel2=FND_L;fndSel3=FND_1; break;					// DCU_L1_MCAR
				case 0x03: fndSel2=FND_L;fndSel3=FND_2; break;					// DCU_L2_MCAR
				case 0x05: fndSel2=FND_L;fndSel3=FND_3; break;					// DCU_L3_MCAR
				case 0x09: fndSel2=FND_L;fndSel3=FND_4; break;					// DCU_L4_MCAR
				default: dcuID.CodingPin=0;fndSel2=FND_NONE;fndSel3=FND_NONE;break;
			}
			Display3FND(m_FND[fndSel1], m_FND[fndSel2], m_FND[fndSel3], 1000);
    	}
	}
}

void mm_TaskMonitoring(void const * argument)
{
    /*
     * Local variable definition
     */
	int32_t  PreviousEncoderValue = 0;
	uint32_t PreviousWakeTime = osKernelSysTick();
	uint32_t MotorMonitoringTime = 0;
	_Bool    MotorMonitoringFlag = 0;

	volatile static _Bool FlagClosingByUnexpectedUnlock=false, FlagBrakeByUnexpectedUnlock=false;
	volatile static uint32_t EncoderErrorCntOnOpened=0, EncoderErrorCntOnClosed=0;
	volatile static uint32_t safetytime=0, UnExpectedUnlockStartTime=0, UnExpectedUnlockTime=0;
	uint8_t loopcnt =0;
	if(m_isMasterSlave == MASTER_DCU)
	{
	    debug("5. [Master Start] Monitoring Task\r\n");
	}
	else // SLAVE_DCU
	{
    	while(m_isTaskExecution == false)
    	{
    		osDelay(200);
    	}
	    debug("5. [Slave Start] Monitoring Task\r\n");
	    PreviousWakeTime = osKernelSysTick();
	    /*
	     *  �����̺꿡 ������� �Ѿ���� �Ǹ� ���� �ð� ���� ������ ��� �ʱ�ȭ �Ѵ�.
	     */
	}
    for(_ErrorCodeList i=DCU_HARD_FAULT; i<=DCU_LOCK_FAULT; i++)
    {
    	error_list_time[i] = osKernelSysTick();
    }

	for(;;)
	{
		/* Place this task in the blocked state until it is time to run again. */
		osDelayUntil(&PreviousWakeTime, 100UL);
		mmf_Encoder.PrePosition = mmo_getEncoderCountValue();
		if(mmf_Encoder.PreStopPosition != mmf_Encoder.Position) m_isTestEncoderflag = true;
		if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
    	{
    		loopcnt++;
    		if(loopcnt>20)
    		{
    			loopcnt=0;
//    			debug("# Master monitoring Task ���� ����\r\n");
    			
    			error_list_time[DCU_OPEN_FAULT] = osKernelSysTick();	
    			error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
    			error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
    		}
    	}
    	else
    	{
    		mm_SaveCurrentDcuData();																//���� ���� ��� ����
    		mm_485_Change_Detection();
    		if(!mip_Input.di0_ZVR)			//zvr ��ȣ�� ���������� ���� �����ϸ� �ȵ� (������ ����)
    		{
//    			error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
//    			error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
//    			error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();
//    			error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
    			error_list_time[DCU_OPEN_FAULT] = osKernelSysTick();	
    			error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
    			error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
//    			safetytime = osKernelSysTick();
   			continue;
    		}
    		/*
    		 * comment : ���� ������ ����� ������ �ʿ� ��
    		 * modify :  ���� ���� �� ���� �����ϴ� ���� if���� �� �߰�
    		 */
    		/*
    		 * ���� ó��
    		 */
    		#if 0
    		{
    			/*
    			 *  todo <<jjkim_190108>> : ���� ó��
    			 */
            	if(((error_list_flag[DCU_HARD_FAULT]) || //|| error_list_flag[DCU_UNEXPECTED_UNLOCK])  &&
            		(error_list_flag[DCU_DLS1_FAULT] && error_list_flag[DCU_DLS2_FAULT])	    ||
    				(error_list_flag[DCU_DCS1_FAULT] && error_list_flag[DCU_DCS2_FAULT])	    ||
    				(error_list_flag[DCU_MOTOR_ERROR]))											&&
    				(mdc_DoorState == DOOR_ERROR))
            	{
            		/*
            		 * Master���� �߰���(F01)�� ���� �߿� ���� �߻�
            		 * 	1. �ϵ� Master���� Motor Free
            		 * 	2. Master�� ��ü �϶�� ��ü��� ����
            		 * 	3. ��� �½�ũ���� Slave�� ��ü����� ���۵�
            		 * 	4. Slave���� �½�ũ ���� ��
            		 * 	5. Slave�� ����ġ ��ȯ ����
            		 */
            		if(m_isMasterSlave == MASTER_DCU)
            		{
            			mip_EmergencyLamp(1);															// ���� ���
    					if(m_isSlaveRunCommand == false) m_isPacketSendToSlave = false;
    					/*
    					 * �½�ũ�� �ƿ� ���̸�(Kill) Master ���� ���� �� Slave���� Master�� �ٽ� ��ü �� �� ����
    					 */
    					#if 0
    						vTaskSuspend(obstackedetectTaskHandle);
    						vTaskSuspend(inputTaskHandle);
    						vTaskSuspend(decisioncontrolTaskHandle);
    						vTaskSuspend(displayfndTaskHandle);
    						debug("## Suspend the obstacke detect task\r\n");
    						debug("## Suspend the input task\r\n");
    						debug("## Suspend the decision control task\r\n");
    						debug("## Suspend the display fnd task\r\n");
    					#endif
            		}
            		else // == SLAVE_DCU
            		{
            			/*
            			 * Slave������ ���� �߻��ϸ� ������ ����� ��.
            			 */
            			mip_EmergencyLamp(1);															// ���� ���
            		}
            	}
            	else
            	{
            		// ���ڴ� ���� �߻��� ������ �ʾƾ� ��
            		if(m_isMasterSlave == MASTER_DCU)
            		{

            		}
            		else
            		{

            		}
            	}
    		}
    		#endif
    		/* �� ���� ����*/
    		mm_HardFault_Detection();
    		/* DLS ���� ����*/
    		mm_DLSFault_Dectetion();
    		/* DCS ���� ���� */
    		mm_DCSFault_Dectetion();
    		/* ���� - ���ڴ� ���� ����*/
    		if(error_list_flag[DCU_ENCODER_ERROR] == false) mm_Mo_EnFault_Dectetion(); //jeon_190715
    		/* ����ġ ���� ��� Ǯ�� ���� ����*/
    		mm_F07fault_Dectection();
    		/* Safety loop ���� ���� */
    		mm_SafetyFault_Dectection();
    		/* ��ֹ� ���� */
    		mm_ObsFault_Dectection();
    		/* ���� ���� ���� ����*/
    		mm_OpenFault_Detection();
    		/* mvb -rs485�� ���� ��ü ���� ����*/

    		/*
    		 * ��������
    		 */
    		DiagnosisInDecisionControl(1,2);
   	}
		
		// LED �� ���� On/Off
		
		#ifdef USE_WATCHDOG
			/*
			 * todo emu: SW Watchdog Ȯ��
			 * 		 Watchdog �츮�� ������ ����. ��???
			 */
			/*
			 * Restart reinforced safety watch_dog at the given period.
			 */
			rswdt_restart(RSWDT);													// Feed�� �ֹǷ� Watch_dog Reset�� �߻����� ����
		#endif
	}
}

static void DiagnosisInMonitoring(void)
{
    /*
     * �������� ���
     *  - ������ ���� ����͸�
     *  - ������ ���� ����͸�
     */
}

static void SupplyVoltageMonitor(void)
{
    uint32_t MotorPowerVoltage=0;
    
    /*
     * Opening/Closing �ÿ���(??) ������/������ ���� ����
     */
    if(MotorPowerVoltage!=0u)													// Opening/Closing �� ��������
    {
    	if(m_adc.PowerVoltage>2500)											// ������ ����
    	{
    		// ������ ���� �÷��� ��
    		error_list_flag[DCU_ERROR] = true;
//    		error_list_flag[DCU_HIGH_VOLTAGE_ERROR] = true;
    	}
    	else if(m_adc.PowerVoltage<1500)										// ������ ����
    	{
    		// ������ ���� �÷��� ��
    		error_list_flag[DCU_ERROR] = true;
//    		error_list_flag[DCU_LOW_VOLTAGE_ERROR] = true;
    	}
    	else																	// ������������ ����
    	{
    		// ������ ���� �÷��� Ŭ����
//    		error_list_flag[DCU_HIGH_VOLTAGE_ERROR] = false;
    		// ������ ���� �÷��� Ŭ����
//    		error_list_flag[DCU_LOW_VOLTAGE_ERROR] = false;
    	}
    }
    else																		// Opened/Closed ��
    {
    	
    }
}

static void DiagnosisInDecisionControl(uint8_t doorstate, _Bool dcs)
{
	if(m_isMasterSlave == MASTER_DCU)
	{
		if((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true))
		{
			if(error_list_flag[DCU_CANT_SLAVE] == true) mip_EmergencyLamp(1);
			else mip_EmergencyLamp(0);
			if(m_isSlaveRunCommand == false) m_isPacketSendToSlave = false;
		}
		if((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true))
		{
			if(error_list_flag[DCU_CANT_SLAVE] == true) mip_EmergencyLamp(1);
			else mip_EmergencyLamp(0);
			if(m_isSlaveRunCommand == false) m_isPacketSendToSlave = false;
		}
		if((error_list_flag[DCU_MOTOR_ERROR] == true) || (error_list_flag[DCU_ENCODER_ERROR] == true))
		{
			if(error_list_flag[DCU_CANT_SLAVE] == true) mip_EmergencyLamp(1);
			else mip_EmergencyLamp(0);
			if(m_isSlaveRunCommand == false) m_isPacketSendToSlave = false;
		}
		if(error_list_flag[DCU_HARD_FAULT] == true)
		{
			if(error_list_flag[DCU_CANT_SLAVE] == true) mip_EmergencyLamp(1);
			else mip_EmergencyLamp(0);
			if(m_isSlaveRunCommand == false) m_isPacketSendToSlave = false;
		}
		if(error_list_flag[DCU_OPEN_FAULT] == true)
		{
			if(error_list_flag[DCU_CANT_SLAVE] == true) mip_EmergencyLamp(1);
			else mip_EmergencyLamp(0);
#if 0 //jeon_190617
			if(m_isSlaveRunCommand == false) m_isPacketSendToSlave = false;
#endif	
		}
	}
	else if(m_isMasterSlave == SLAVE_DCU)
	{
		if((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true))
		{
			mip_EmergencyLamp(1);
		}
		if((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true))
		{
			mip_EmergencyLamp(1);
		}
		if((error_list_flag[DCU_MOTOR_ERROR] == true) || (error_list_flag[DCU_ENCODER_ERROR] == true))
		{
			mip_EmergencyLamp(1);
		}
	}
}

void FaultFlag_SET(_ErrorCodeList error_code)
{
	if(error_list_flag[error_code] == false)
	{
		error_list_flag[error_code] = true;
		error_list_flag[DCU_ERROR] = true;
		mm_SaveFaultDcuData(error_code);
		switch(error_code)
		{
			case DCU_DLS1_FAULT: mip_EmergencyLamp(1); break;
			case DCU_DLS2_FAULT: mip_EmergencyLamp(1); break;
			case DCU_DCS2_FAULT: mip_EmergencyLamp(1); break;
			case DCU_DCS1_FAULT: mip_EmergencyLamp(1); break;
			case DCU_MOTOR_ERROR: mip_EmergencyLamp(1); break;
			case DCU_ENCODER_ERROR: mip_EmergencyLamp(1); break;
			case DCU_OBSTACLE: mip_EmergencyLamp(1); break;
			case DCU_HARD_FAULT: mip_EmergencyLamp(1); break;
			default: break;
		}
	}
}

void mram_Event_save(_EventCodeList event_code)
{
	volatile static _EventCodeList PreEvnetcode = DCU_EVNET_NONE;
	/*
	 *  ���� �̺�Ʈ �Է°� �ٸ� ��쿡�� mram�� �����Ѵ�. ���ܽ���ġ�� on�� �� ��� mram�� �����ϸ� �ȵǱ� ����
	 */
	if(PreEvnetcode != event_code)
	{
		mm_SaveFaultDcuData(event_code);
		PreEvnetcode = event_code;
	}
}

void mm_DLSFault_Dectetion(void)
{
	/*
	 * DLS,DCS�����Ǵ��� Closing->Closed, Opening->Opened �ÿ��� �����ϸ� �ȵǴ� ���� ����
	 *
	 * Closing->Closed �� DLS or DCS�� �ȴ��� �����ڵ尡 �߻��� ����
	 * Closed->Closed ���¿��� �ٽ� DLS, DCS�� ������ ������ ���͵Ǿ�� �ϴµ�
	 * ���� �ݴ� ������ �������� ������ ���͵��� ����
	 *
	 * -> ���� DLS,DCS����üũ�� �׻� ����Ǿ�� ��
	 *    Closed->Closed ���¿����� DLS,DCS�� ������ �ϰ�
	 *    �̿��� ���¿����� DLS,DCS�� ������ ��
	 */
	/*
	 * �����Ϸ� �� DLS1/2(F04/F08), DCS1/2(F05/F06) ����(Open Circuit) �Ǵ�
	 */
	if( (mdc_DoorState==DOOR_CLOSED)	||
		((mdc_PreDoorState == DOOR_CLOSED) && (mdc_DoorState == DOOR_ERROR)))
	{
		#if 0
		{
			if(!mip_Input.di0_DLS1)														// ������ ������ DLS1 ����(Open Circuit)
			{
				error_list_flag[DCU_DLS1_FAULT] = true;									// �����Ϸ� �� DSL1 ����
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DLS1_FAULT] = false;								// ������ DLS1 ���� ����
			}

			if(!mip_Input.di1_DLS2)														// ������ ������ DLS2 ����(Open Circuit)
			{
				error_list_flag[DCU_DLS2_FAULT] = true;									// �����Ϸ� �� DLS2 ����
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DLS2_FAULT] = false;								// ������ DLS2 ���� ����
			}

			if(!mip_Input.di1_DCS1)														// ������ ������ DCS1 ����(Open Circuit)
			{
				error_list_flag[DCU_DCS1_FAULT] = true;									// �����Ϸ� �� DCS1 ����
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DCS1_FAULT] = false;								// ������ DCS1 ���� ����
			}

			if(!mip_Input.di1_DCS2)														// ������ ������ DCS2 ����(Open Circuit)
			{
				error_list_flag[DCU_DCS2_FAULT] = true;									// �����Ϸ� �� DCS2 ����
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DCS2_FAULT] = false;								// ������ DCS2 ���� ����
			}
		}
		#else
		{
			if((!mip_Input.di0_DLS1) && (!mip_Input.di1_DLS2))								//�Ѵ� Ǯ�� ��쿡�� f07 �߻� ���� dls ������ ���� �Ѵ�.
			{
				if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000) &&
					(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS2_FAULT]) > 5000))
				{
					if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000)
					{
						FaultFlag_SET(DCU_DLS1_FAULT);
					}
					if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS2_FAULT]) > 5000)
					{
						FaultFlag_SET(DCU_DLS2_FAULT);
					}
				}
			}
			else if((mip_Input.di0_DLS1) && (!mip_Input.di1_DLS2))						// DLS 1�� �������� DLS2�� ������ �������
			{
				error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();					// DLS 1�� �����̹Ƿ� ���� �ð��� ���� �Ѵ�.
				Erase_fault(DCU_DLS1_FAULT);
				if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS2_FAULT]) > 5000)
				{
					FaultFlag_SET(DCU_DLS2_FAULT);
				}
			}
			else if((!mip_Input.di0_DLS1) && (mip_Input.di1_DLS2))						// DLS 1�� �������ʾ����� DLS2�� ���� ���
			{
				error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
				Erase_fault(DCU_DLS2_FAULT);
				if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000)
				{
					FaultFlag_SET(DCU_DLS1_FAULT);
				}

			}
			else /* �Ѵ� ���� ���*/
			{
				Erase_fault(DCU_DLS2_FAULT);
				Erase_fault(DCU_DLS1_FAULT);
				error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
				error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();
			}
		}
		#endif
	}
	else if( (mdc_DoorState==DOOR_OPENED)	||
		   ((mdc_PreDoorState == DOOR_OPENED) && (mdc_DoorState == DOOR_ERROR)))
	{
		if((mip_Input.di0_DLS1) && (mip_Input.di1_DLS2))								//�Ѵ� Ǯ�� ��쿡�� f07 �߻� ���� dls ������ ���� �Ѵ�.
		{
			if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000) &&
				(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS2_FAULT]) > 5000))
			{
				if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000)
				{
					FaultFlag_SET(DCU_DLS1_FAULT);
				}
				if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS2_FAULT]) > 5000)
				{
					FaultFlag_SET(DCU_DLS2_FAULT);
				}
				if((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true))
				{
					error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();
					error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
					mdc_PreDoorState = DOOR_OPENED;
					mdc_DoorState = DOOR_ERROR;
					debug("# Opend ���¿��� dls 1/2���� ���� ���� ���� \r\n");
				}
			}
		}
		else if((mip_Input.di0_DLS1) && (!mip_Input.di1_DLS2))						// DLS 1�� �������� DLS2�� ������ �������
		{
			error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();					// DLS 2�� �����̹Ƿ� ���� �ð��� ���� �Ѵ�.
			error_list_flag[DCU_DLS2_FAULT] = false;
			if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000)
			{
				FaultFlag_SET(DCU_DLS1_FAULT);
			}
		}
		else if((!mip_Input.di0_DLS1) && (mip_Input.di1_DLS2))						// DLS 1�� �������ʾ����� DLS2�� ���� ���
		{
			error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();
			error_list_flag[DCU_DLS1_FAULT] = false;
			if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS2_FAULT]) > 5000)
			{
				FaultFlag_SET(DCU_DLS2_FAULT);
			}
		}
		else
		{
#if 1 //jeon_190709			orig:1
			Erase_fault(DCU_DLS2_FAULT);
			Erase_fault(DCU_DLS1_FAULT);
#endif
			error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
			error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();
		}
	}
	else
	{
		error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
		error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();
	}
}

void mm_DCSFault_Dectetion(void)
{
	if( (mdc_DoorState==DOOR_CLOSED)	||
		((mdc_PreDoorState == DOOR_CLOSED) && (mdc_DoorState == DOOR_ERROR))   )
	{
		if((mip_Input.di1_DCS1 == false) && (mip_Input.di1_DCS2 == false))
		{
			if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS2_FAULT]) > 5000) &&
				(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS1_FAULT]) > 5000))
			{
				FaultFlag_SET(DCU_DCS2_FAULT);
				FaultFlag_SET(DCU_DCS1_FAULT);
				if((error_list_flag[DCU_DCS2_FAULT] == true) && (error_list_flag[DCU_DCS1_FAULT] == true))
				{
					error_list_time[DCU_DCS2_FAULT] = osKernelSysTick();
					error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
					mdc_PreDoorState = DOOR_CLOSED;
					mdc_DoorState = DOOR_ERROR;
					debug("# CLOSED ���¿��� ���� ���� ���� ���� \r\n");
				}
			}
		}
		/*
		 *  DCS 1�� �������� DCS2�� ������ �������
		 */
		else if((mip_Input.di1_DCS1 == true) && (mip_Input.di1_DCS2 == false))
		{
			error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
			error_list_flag[DCU_DCS1_FAULT] = false;
			if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS2_FAULT]) > 5000)
			{
				FaultFlag_SET(DCU_DCS2_FAULT);
			}
		}
		else if((mip_Input.di1_DCS1 == false) && (mip_Input.di1_DCS2 == true))
		{
			error_list_time[DCU_DCS2_FAULT] = osKernelSysTick();
			error_list_flag[DCU_DCS2_FAULT] = false;
			if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS1_FAULT]) > 5000)
			{
				FaultFlag_SET(DCU_DCS1_FAULT);
			}
		}
		else
		{
			Erase_fault(DCU_DCS1_FAULT);
			Erase_fault(DCU_DCS2_FAULT);
			error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
			error_list_time[DCU_DCS2_FAULT] = osKernelSysTick();
		}
	}
	else if( (mdc_DoorState==DOOR_OPENED)	||
		   ((mdc_PreDoorState == DOOR_OPENED) && (mdc_DoorState == DOOR_ERROR)))
	{
		if((mip_Input.di1_DCS1 == true) && (mip_Input.di1_DCS2 == true))
		{
			if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS2_FAULT]) > 5000) &&
				(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS1_FAULT]) > 5000))
			{
				FaultFlag_SET(DCU_DCS2_FAULT);
				FaultFlag_SET(DCU_DCS1_FAULT);
				if((error_list_flag[DCU_DCS2_FAULT] == true) && (error_list_flag[DCU_DCS1_FAULT] == true))
				{
					error_list_time[DCU_DCS2_FAULT] = osKernelSysTick();
					error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
					mdc_PreDoorState = DOOR_OPENED;
					mdc_DoorState = DOOR_ERROR;
					debug("# Opened ���¿��� ���� ���� ���� ���� \r\n");
				}
			}
		}
		else if((mip_Input.di1_DCS1 == true) && (mip_Input.di1_DCS2 == false))
		{
			error_list_time[DCU_DCS2_FAULT] = osKernelSysTick();
			error_list_flag[DCU_DCS2_FAULT] = false;
			if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS1_FAULT]) > 5000)
			{
				FaultFlag_SET(DCU_DCS1_FAULT);
			}
		}
		else if((mip_Input.di1_DCS1 == false) && (mip_Input.di1_DCS2 == true))
		{
			error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
			error_list_flag[DCU_DCS1_FAULT] = false;
			if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DCS2_FAULT]) > 5000)
			{
				FaultFlag_SET(DCU_DCS2_FAULT);
			}
		}
		else
		{
			Erase_fault(DCU_DCS1_FAULT);
			Erase_fault(DCU_DCS2_FAULT);
			error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
			error_list_time[DCU_DCS2_FAULT] = osKernelSysTick();
		}
	}
	else
	{
		error_list_time[DCU_DCS1_FAULT] = osKernelSysTick();
		error_list_time[DCU_DCS2_FAULT] = osKernelSysTick();
	}
}

void mm_Mo_EnFault_Dectetion(void)
{
	volatile static int32_t PreviousEncoderValue = 0;
	volatile static _Bool MotorMonitoringFlag = 0;

	if((mdc_DoorState == DOOR_OPENING) || (mdc_DoorState == DOOR_CLOSING))
	{

		if(mdc_DoorState == DOOR_CLOSING)
		{
			if((mip_Input.di0_DLS1 == true) || (mip_Input.di1_DLS2 == true) || (mip_Input.di1_DCS1 == true) || (mip_Input.di1_DCS2 == true))
			{
				/* DLS 1 & 2�� OFF(false)�� ������ �� ��Ȳ������ ���� �Ϸ� �Ǵ��� 6�ʵڿ� �Ѵ� 6�� ���� ��� �����
				 * �б� ������ ���ڴ� ������ �ƴԿ��� �ұ��ϰ� ������ �����
				 * ���� �Ϸ� �κп����� ���ڴ� ���� �Ǻ��� �ϱⰡ �����Ƿ� ���� �Ѵ�.
				 */
				PreviousEncoderValue = mmf_Encoder.Position;
				return;
			}
		}
		
#if 0 //jeon_190924		
		if((mdc_DoorState == DOOR_CLOSING) && (mdc_PreDoorState == DOOR_OPENING))
		{
			error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
			error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
		}
		else if((mdc_DoorState == DOOR_OPENING) && (mdc_PreDoorState == DOOR_CLOSING))
		{
			error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
			error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
		}
#endif
		
			if(PreviousEncoderValue != mmf_Encoder.Position) 						//���ڴ� ���� ��ȭ �� ��� (���� ����)
			{
				MotorMonitoringFlag = false;
				error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
				error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
			}
			else if((PreviousEncoderValue == mmf_Encoder.Position) && (m_adc.MotorCurrent < 500u))															// ���ڴ� ���� ��ȭ ���� ��� (���� ����)
			{
				/*
				 *  ��� ���߸� opening�� ���¸� ���� �� ���̹Ƿ� ���ڴ� ��ȭ�� ���� ���°� ���� ���� �Ǹ�(opening ��Ȳ) ���� �������� �����Ѵ�.
				 */
				if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_MOTOR_ERROR]) > 6000))
				{
					FaultFlag_SET(DCU_MOTOR_ERROR);
					mdc_PreDoorState = mdc_DoorState;
					mdc_DoorState = DOOR_ERROR;
				}
				else if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_ENCODER_ERROR]) > 2000) && (MotorMonitoringFlag == false))
				{
					MotorMonitoringFlag = true;									//���� ������� ���谡 �Һи� �ϱ� ������ �ٷ� ������ ����� �ʴ´�.
					mmf_PIDControl = PID_CONTROL_NONE;
					//mdc_isInitComplete = false;
					if(mdc_DoorState == DOOR_OPENING)
					{
						mmo_DoorOpening(DCU_SLOWMODE_POWER);
						mdc_FlagSlowMode = true;
					}
					else if(mdc_DoorState == DOOR_CLOSING)
					{
						mmo_DoorClosing(DCU_SLOWMODE_POWER);
						mdc_FlagSlowMode = true;
					}
					debug("# ���ڴ� �޽��� �����ϹǷ� �ӵ� ���� �Ұ� slowmode ���� \r\n");
				}
			}
			else	/* �̿��� ��Ȳ�� ����-���ڴ� ������ �������� �ʴ´� (ex) ��ֹ��� ���� ���ڴ� ��ȭ�� ���� ��쿡 �������� ���� ����)*/
			{

			}
			//PreviousEncoderValue = mmf_Encoder.Position;
	}
	else if((mdc_DoorState == DOOR_CLOSED) || (mdc_DoorState == DOOR_OPENED))
	{
		if(MotorMonitoringFlag == true)
		{
			MotorMonitoringFlag = false;
			FaultFlag_SET(DCU_ENCODER_ERROR);
		}
		error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
		error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
	}
	else
	{
		error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
		error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
	}
	PreviousEncoderValue = mmf_Encoder.Position;
}

void mm_F07fault_Dectection(void)
{
	volatile static _Bool Flag_f07 = false;

	/*

	 * ����ġ ���� ��� Ǯ�� ���� : 1)���� ���¿��� DLS1/2�� Ǯ�� ��� ���� ������ ���� �Ҽ� �ֵ��� ��
	 *                				2) ���� ���¸� ERROR�� �ٲٰ� DecisionControl���� ���� ������ ���� �� �� �ֵ��� �Ѵ�.
	 */
#if 0 //jeon_190617 ����
	if(mdc_DoorState == DOOR_CLOSED)
	{
		if((error_list_flag[DCU_UNEXPECTED_UNLOCK] == pdFALSE) && (!mip_Input.di0_Isolation))
		{
			if((!mip_Input.di1_DLS2) || (!mip_Input.di0_DLS1)) //jeon_190514 orig : &&
			{
				error_list_count[DCU_UNEXPECTED_UNLOCK]++;
			}
			else
			{
				error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
				error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();
			}

			if(error_list_count[DCU_UNEXPECTED_UNLOCK] > 1) 		//300ms�� ���� �߷�, decisioncontrol���� ���嵿�� �ǽ�
			{
				error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
				error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();					//f07�� �߻��� �������� �����Ѵ�.
				FaultFlag_SET(DCU_UNEXPECTED_UNLOCK);
				mdc_PreDoorState = DOOR_CLOSED;
				mdc_DoorState = DOOR_ERROR;
			}
		}
		else if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* ���� ���� ������ �ݺ��ϰ� dls �ƴٰ� Ǯ�� ��� �� ����*/
		{
			if((mip_Input.di1_DLS2) || (mip_Input.di0_DLS1))
			{
				//error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
				error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();
				error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
				Erase_fault(DCU_UNEXPECTED_UNLOCK);
			}
		}
	}
#else
	if(mip_Input.di0_OpenCloseButton == true)
	{
		Flag_f07 = false;
		error_list_flag[DCU_UNEXPECTED_UNLOCK] = pdFALSE;
		error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
		
		if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* ���� ���� ������ �ݺ��ϰ� dls �ƴٰ� Ǯ�� ��� �� ����*/
		{

			Erase_fault(DCU_UNEXPECTED_UNLOCK);
		}
	}
	else if((mip_Input.di0_Isolation == true)||(mip_Input.di1_EED == true))
	{
		Flag_f07 = false;
		error_list_flag[DCU_UNEXPECTED_UNLOCK] = pdFALSE;
		error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
		
		if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* ���� ���� ������ �ݺ��ϰ� dls �ƴٰ� Ǯ�� ��� �� ����*/
		{

			Erase_fault(DCU_UNEXPECTED_UNLOCK);
		}
	}
	else
	{
		if(mdc_DoorState == DOOR_CLOSED) 
		{
			if((mip_Input.di1_DLS2 == pdTRUE) || (mip_Input.di0_DLS1 == pdTRUE))
			{
				Flag_f07 = pdTRUE;
				error_list_flag[DCU_UNEXPECTED_UNLOCK] = pdFALSE;
				error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
						
				if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* ���� ���� ������ �ݺ��ϰ� dls �ƴٰ� Ǯ�� ��� �� ����*/
				{

					Erase_fault(DCU_UNEXPECTED_UNLOCK);
				}
			}
			
			if((error_list_flag[DCU_UNEXPECTED_UNLOCK] == pdFALSE) && (Flag_f07 == pdTRUE))
			{
				if((!mip_Input.di1_DLS2) && (!mip_Input.di0_DLS1)) 
				{
					error_list_count[DCU_UNEXPECTED_UNLOCK]++;
				}
				else
				{
					error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
					error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();
				}

				if(error_list_count[DCU_UNEXPECTED_UNLOCK] > 1) 		//300ms�� ���� �߷�, decisioncontrol���� ���嵿�� �ǽ�
				{
					error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
					error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();					//f07�� �߻��� �������� �����Ѵ�.
					FaultFlag_SET(DCU_UNEXPECTED_UNLOCK);
					mdc_PreDoorState = DOOR_CLOSED;
					mdc_DoorState = DOOR_ERROR;
				}
			}
			else if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* ���� ���� ������ �ݺ��ϰ� dls �ƴٰ� Ǯ�� ��� �� ����*/
			{
				if((mip_Input.di1_DLS2) || (mip_Input.di0_DLS1))
				{
					//error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
					error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();
					error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
					Erase_fault(DCU_UNEXPECTED_UNLOCK);
				}
			}
		}
	}
#endif
}

void mm_SafetyFault_Dectection(void)
{
	volatile static uint32_t safetytime = 0;
	if(mdc_DoorState == DOOR_CLOSED)
	{
		if(!mip_Input.di1_EED)
		{
			if(mip_Input.di0_SafetyLoopA != mip_Input.di0_SafetyLoopB)
			{
				if(get_diff_tick(osKernelSysTick(), safetytime) > 10000)		// 10�� ���� Safety Loop�� ���� ������
				{
					FaultFlag_SET(DCU_SAFETYLOOP_FAULT);
				}
			}
			/*
			 * Safety Loop 1/2�� ������ ��� ��� Opening or Closing ������ ���������� �Ϸ��� ������ �Ǵ�
			 */
			else
			{
				safetytime = osKernelSysTick();
				Erase_fault(DCU_SAFETYLOOP_FAULT);
			}
		}
		else
		{
			// No Action
		}
	}
}

void mm_ObsFault_Dectection(void)
{
	/*
	 * ��ֹ� ���� ���� : 1)��ֹ� ����(3ȸ �̻�) �ϰ� ���� ���� �Ҷ� ������ �����Ѵ�
	 *                    2)���� �Ϸᰡ �� ��쿡�� ��ֹ��� �����Ƿ� ���� ����
	 */
	if((mod_Detect.ObstacleDetectCnt >= mod_Detect.ObstacleConfigValue) && (mdc_DoorState != DOOR_CLOSED))
	{
		FaultFlag_SET(DCU_OBSTACLE);
	}
	if(mdc_DoorState == DOOR_CLOSED)
	{
		Erase_fault(DCU_OBSTACLE);
	}
}
void mm_485_Change_Detection(void)
{
	/* rs 485 �����̺��� ��� - > 485 �����ͷκ��� ��ü ��Ʈ�� ���� ����
	 * l4 r1�� ��� -> tcms�� ���� ��ü��Ʈ�� ����
	 */
	//m_isMasterSlaveChange = false;
	if(m_isMasterSlaveChange == false)
	{
		/*
		 *  ��� ������ �߻� �� ��� ��ü�� �����Ѵ�.
		 */
		if((g_unDeviceID == DCU_R1_MCAR) || (g_unDeviceID == DCU_R1_TCAR) || (g_unDeviceID == DCU_L4_MCAR) || (g_unDeviceID == DCU_L4_TCAR))
		{
			if(error_list_flag[DCU_MVB_ERROR] == true)	//R1 , L4�� ��쿡�� MVB ���常 �߻������� ��ü
			{
				m_is485changeflag = false;
			}
		}
		else
		{
			if(error_list_flag[DCU_RS485_ERROR] == true ) // RS485 �����̺�� 485������ �߻������� ��ü
			{
				m_is485changeflag = false;
			}
		}
	}
}


/*
 * SRS-DCU-FR-004
 */
void mm_OpenFault_Detection(void)
{
	/*
	 * ���� ���� ���� ���� : Opening ���°� 10�� �̻� ���� �Ǹ� ���� ���� ���� ����
	 */
	if(mdc_DoorState == DOOR_OPENING)
	{
		if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_OPEN_FAULT]) > 3000)		//orig : 10000
		{
			if(error_list_flag[DCU_OPEN_FAULT] == false)
			{
				printf("F10 - %d - %d\r\n", osKernelSysTick(), error_list_time[DCU_OPEN_FAULT]);
			}
			FaultFlag_SET(DCU_OPEN_FAULT);			
			mmo_DoorFree();
			mdc_PreDoorState = DOOR_OPENING;
			mdc_DoorState = DOOR_ERROR;
		}
	}
	/*
	 * ������� Closed ���°� 10�� �̻� �����Ұ�� �ð� ������ ���� ���Ѵ�.
	 * �׷��ٰ� ���� ������ָ� �������а����� ���� �ǰ� ��ü�������� �����̹߻���
	 * opening ���°� �ƴ� ��쿡�� �ð� ���� �����ϵ��� �������� (�������� opend)
	 */
	else if(mdc_DoorState != DOOR_OPENING)
	{
		error_list_time[DCU_OPEN_FAULT] = osKernelSysTick();
		if(error_list_flag[DCU_OPEN_FAULT] == true)
		{
			if(mdc_DoorState == DOOR_CLOSED)
			{
				Erase_fault(DCU_OPEN_FAULT);
			}
		}
	}
}

/*
 * SRS-DCU-FR-004 , SRS-DCU-FR-010
 */
void mm_HardFault_Detection(void)
{
	/*
	 * ���͸� ����ġ ���� ������ ����
	 * �� ������ DCU -> ������ ǻ�� ������ OO���� ������ �ٽ� ǻ�� ������ ������
	 * �� ������ DCU -> ǻ��Ѵ� ���� ���¿��� ������ Fx ������ ������ �����µ� �ø��� f0 f1 f0 f1 f8 ������� ��µ�
	 */
	if((mip_Input.RotarySwitch & 0x0f) == 0x0f)
	{
		FaultFlag_SET(DCU_HARD_FAULT);
		mdc_PreDoorState = mdc_DoorState;
		mdc_DoorState = DOOR_ERROR;
	}
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMonitoring.c
*********************************************************************/
