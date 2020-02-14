/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtMonitoring
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMonitoring.c
*********************************************************************/

#include "mtMonitoring.h"
/* 도어 상태(Opening/Closing)에 따라 ADC측정 전압을 결정하기 위해 인클루드 */
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

struct adcResults g_afec0_ch4 = {0,false};	/* PB0(AFEC0_AD4) - 모터전류 */
struct adcResults g_afec0_ch5 = {0,false};	/* PB1(AFEC0_AD5) - BEMF전압(정) */
struct adcResults g_afec0_ch6 = {0,false};	/* PB2(AFEC0_AD6) - BEMF전압(역) */
struct adcResults g_afec0_ch8 = {0,false};	/* PB8(AFEC0_AD8) - 3.3V Monitor */
struct DebugPrint debugprint = {0};
uint8_t mram_fnd_data = 0;					/* MRAM에 쓰여지는 FND 정보를 쓰기 위한 변수*/
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
	uint8_t Trace_count = 0;														// 고장 개수(=블럭개수)를 파악하는 변수
	uint8_t save_data = 0;
	/*
	 * 고장 정보만 공유하는 rtc data , 전역변수(rtc_time)를 공유하게 되면
	 * hex to demical , demical to hex 과정의 코드를 추가해야해서 코드가 지저분해 짐
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
	
	mm_SaveCurrentDcuData();														//현재 dcu의 상태를 저장 한다. 저장 당시의 상태랑 주기적으로 저장하는 상태랑 맞지 않는 경우도 발생함
	Trace_count  = mram_byte_read(MRAM_FAULT_ADDR);									// 고장 개수를 읽어 온다.(블록 개수)

	if(Trace_count >= 100u)															// 고장 개수가 100개가 넘어 간 경우
	{
		for(i=0; i<Trace_count; i++)
		{
			for(j=1; j<61; j++)
			{
				save_data = mram_byte_read(MRAM_FAULT_ADDR+(i+1)*0x64+j);			//첫번째 데이터를 지우고 그다음 데이터를 써야하므로 데이터를 불러 온다.
				if(j==2)	mram_byte_write(MRAM_FAULT_ADDR+(i)*0x64+j,i);			//두번째 바이트는 첫번째 고장 개수를 저장 한다.
				else		mram_byte_write(MRAM_FAULT_ADDR+(i)*0x64+j,save_data);	//데이터를 저장
			}
		}
		Trace_count = 99;
	}

	mram_byte_write(MRAM_FAULT_ADDR,Trace_count+1);									// 고장 개수를 갱신한다.
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+1,ErrorCode);				// ex) trace_count = 1 : 100+1번지에 에러 코드 저장
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+2,Trace_count);				// 전체 고장 개수를 저장 한다.
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+3,fault_rtcdata.ucYears);						// years	나중에 rtc 데이터 받아서 넣어야함
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+4,fault_rtcdata.ucMonth);						// month
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+5,fault_rtcdata.ucDate);						// date
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+6,fault_rtcdata.ucHours);						// hours
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+7,fault_rtcdata.ucMinutes);						// miniute
	mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+8,fault_rtcdata.ucSeconds);						// second

	for(i=0; i<53; i++)
	{
		save_data = mram_byte_read(MRAM_DATA_ADDR+i);									//상태 정보들을 읽는다.
		mram_byte_write(MRAM_FAULT_ADDR+(Trace_count)*0x64+9+i,save_data);				//블록의 9번째 바이트 부터 상태 정보들을 저장한다.
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

	m_isFaultCount = mram_byte_read(MRAM_FAULT_ADDR);				//Dcu의 고장 개수를 저장

	mram_byte_write(MRAM_DATA_ADDR,g_unDeviceID);
	/* DCU OK 정보를 파라메터 세팅이나, 초기화 동작이 완료 되면 SET 시키도록 변경*/
	Temp_DCU_Data = (error_list_flag[DCU_HARD_FAULT] == false)?0x80:0x00;
	mram_byte_write(MRAM_DATA_ADDR+1,Temp_DCU_Data);

	Temp_DCU_Data =0;
	Temp_DCU_Data |= ((mip_Input.DoorClosedSwitchOn == false)?0x01:0x00);
	Temp_DCU_Data |= ((mip_Input.di0_Isolation == true)?0x01:0x00)<<1;
	Temp_DCU_Data |= (((mdc_DoorState == DOOR_OPENED) && (error_list_flag[DCU_OBSTACLE] == true))?0x01:0x00)<<2;			//도어 완전 열림 상태
	Temp_DCU_Data |= ((error_list_flag[DCU_ERROR] == true)?0x01:0x00)<<3;													//도어 에러 상태
	Temp_DCU_Data |= ((mod_Detect.ObstacleDetectCnt > 0)?0x01:0x00)<<4;														//장애물 감지 상태
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

	Trace_count = mram_byte_read(MRAM_FAULT_ADDR);			// mram에 저장한 고장 개수
	mram_byte_write(MRAM_DATA_ADDR+7,Trace_count);

	if(HAL_GPIO_ReadPin(SET_MASTER_SLAVE_GPIO_Port, SET_MASTER_SLAVE_Pin)) mram_byte_write(MRAM_DATA_ADDR+8,0x22);
	else mram_byte_write(MRAM_DATA_ADDR+8,0x12);

	mram_byte_write(MRAM_DATA_ADDR+9,VERSION_MINOR);
	mram_byte_write(MRAM_DATA_ADDR+10,VERSION_MAJOR);

	mram_byte_write(MRAM_DATA_ADDR+11,mram_fnd_data);				//fnd 에 뿌려지는 값
	mram_byte_write(MRAM_DATA_ADDR+12,mram_fnd_dataB);
	mram_byte_write(MRAM_DATA_ADDR+13,mram_fnd_dataC);

	//mram_byte_write(MRAM_DATA_ADDR+14,*(unsigned char*)(0x64000004));			//로터리 스위치

	if(mdc_DoorState == DOOR_CLOSING) powervoltage =(uint16_t)(m_adc.MotorVoltage / 31);
	else if(mdc_DoorState == DOOR_OPENING) powervoltage =(uint16_t)(m_adc.PowerVoltage / 31);
	else powervoltage = 100;

	mram_byte_write(MRAM_DATA_ADDR+16, powervoltage);										//14,15 dcu 공급 전압(100v)
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
	 * DO_0(FMC_OUTEN0) - MDO_CTL1~6 출력 -> 전원공급, 0111.1001 = 0x79 (모두 전원 공급)
	 * -------------------------------------------------------------------------------------------
	 * | [7]  |    [6]     |     [5]     |   [4]    |   [3]     |     [2]    |   [1]  |   [0]    |
	 * |------|------------|-------------|----------|-----------|------------|--------|----------|
	 * |      |차단스위치전원  |EAD-EED Power|DCS1 Power|DCS2 Power	| Disconnect |Reserved|비상등|
	 * -------------------------------------------------------------------------------------------
	 */
	Temp_DCU_Data =0;
	SWitch_POWER_data = FMC_OUTEN0;
	//Temp_DCU_Data |= (SWitch_POWER_data & 0b00000001);						//비상등 전원
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b00001000)>>1);					//dcs 2 감시 전원
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b00010000)>>1);					//dcs 1 감시 전원
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b00100000)>>1);					//eed-ead 감시 전원
	//Temp_DCU_Data |= ((SWitch_POWER_data & 0b01000000)>>1);					//차단 스위치 감시 전원
	mram_byte_write(MRAM_DATA_ADDR+31,~SWitch_POWER_data);						// 0xff 출력 -> 전원 전부 On(정상)

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

	mram_byte_write(MRAM_DATA_ADDR+39,(uint8_t)Current_Velocity_value);						//속도
	mram_byte_write(MRAM_DATA_ADDR+40,(uint8_t)(Current_Velocity_value>>8));

	RealPosition = (mmf_Encoder.Position * 0.18);
	mram_byte_write(MRAM_DATA_ADDR+41,(uint8_t)(RealPosition));						//거리값 환산
	mram_byte_write(MRAM_DATA_ADDR+42,(uint8_t)((RealPosition)>>8));

	Temp_DCU_Data =0;
	Temp_DCU_Data |= (mip_Input.CommandTestMode == true)?0x80:0x00;
	mram_byte_write(MRAM_DATA_ADDR+43,Temp_DCU_Data); // 8bit는 시험모드 상태 유지보수프로그램 시험모드 버튼이 불이 들어옴
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
    	printf("FND Task 시작 \r\n");
	    DisplaySWVersionToFND(VERSION_MAJOR,VERSION_MINOR,VERSION_SECOND);
	}
	
	for(;;)
	{
    	if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
    	{
    		osDelay(1000);
    		//debug("# Master input Task 실행 안함\r\n");
    	}
    	else
    	{
    		/*
    		 * 고장코드 or 888 출력
    		 */
    		if(error_list_flag[DCU_ERROR] == true)						// 에러가 발생될 때마다 에러플래그는 true로 설정 됨
    		{
    			/*
    			 * F01부터 F16까지 에러 스캔
    			 */
    			error_cnt=0;
    			for(error=1;error<17;error++)
    			{
    				/*
    				 * 발생된 에러가 있으면 순차적으로 발생된 에러 FND Display
    				 */
    				if(error_list_flag[error] == true)
    				{
    					error_cnt++;
    					/*
    					 * FND 한번뿌려서 잘 안되는거 같아서 두번씩 뿌림
    					 */
    					Display3FND(m_FND[FND_F], m_FND[error/10], m_FND[error%10], 500);
    				}
    			}
    			/*
    			 * F01~F16까지 에러를 스캔했는데 발생된 에러가 0개면 에러플래그 false로 설정
    			 */
    			if(error_cnt==0)
    			{
    				error_list_flag[DCU_ERROR] = false;
    			}
    		}
        	else
        	{
				/*
				 * 정상 FND 출력
				 */
				Display3FND(m_FND[FND_8], m_FND[FND_8], m_FND[FND_8], 1000);
        	}
    		
    		/*
    		 * DCU ID 정보 출력
    		 */
    		/*
    		 * Slave 퓨즈 뺀 상태에서 전원 인가 시 RotarrySwitch 0으로 읽힘
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
	     *  슬레이브에 제어권이 넘어오게 되면 고장 시간 감지 값들을 모두 초기화 한다.
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
//    			debug("# Master monitoring Task 실행 안함\r\n");
    			
    			error_list_time[DCU_OPEN_FAULT] = osKernelSysTick();	
    			error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
    			error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
    		}
    	}
    	else
    	{
    		mm_SaveCurrentDcuData();																//상태 정보 계속 저장
    		mm_485_Change_Detection();
    		if(!mip_Input.di0_ZVR)			//zvr 신호가 없을떄에는 고장 감지하면 안됨 (김재의 부장)
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
    		 * comment : 도어 프리를 만드는 동작이 필요 함
    		 * modify :  고장 났을 때 도어 프리하는 동작 if조건 식 추가
    		 */
    		/*
    		 * 에러 처리
    		 */
    		#if 0
    		{
    			/*
    			 *  todo <<jjkim_190108>> : 고장 처리
    			 */
            	if(((error_list_flag[DCU_HARD_FAULT]) || //|| error_list_flag[DCU_UNEXPECTED_UNLOCK])  &&
            		(error_list_flag[DCU_DLS1_FAULT] && error_list_flag[DCU_DLS2_FAULT])	    ||
    				(error_list_flag[DCU_DCS1_FAULT] && error_list_flag[DCU_DCS2_FAULT])	    ||
    				(error_list_flag[DCU_MOTOR_ERROR]))											&&
    				(mdc_DoorState == DOOR_ERROR))
            	{
            		/*
            		 * Master에서 중고장(F01)과 같은 중요 오류 발생
            		 * 	1. 일딴 Master에서 Motor Free
            		 * 	2. Master는 절체 하라고 절체명령 설정
            		 * 	3. 통신 태스크에서 Slave로 절체명령이 전송됨
            		 * 	4. Slave에서 태스크 시작 됨
            		 * 	5. Slave로 스위치 전환 수행
            		 */
            		if(m_isMasterSlave == MASTER_DCU)
            		{
            			mip_EmergencyLamp(1);															// 비상등 출력
    					if(m_isSlaveRunCommand == false) m_isPacketSendToSlave = false;
    					/*
    					 * 태스크를 아예 죽이면(Kill) Master 에러 복귀 시 Slave에서 Master로 다시 절체 할 수 없음
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
            			 * Slave에서도 에러 발생하면 완전히 뻗어야 함.
            			 */
            			mip_EmergencyLamp(1);															// 비상등 출력
            		}
            	}
            	else
            	{
            		// 엔코더 고장 발생시 뻗지는 않아야 함
            		if(m_isMasterSlave == MASTER_DCU)
            		{

            		}
            		else
            		{

            		}
            	}
    		}
    		#endif
    		/* 중 고장 감지*/
    		mm_HardFault_Detection();
    		/* DLS 고장 감지*/
    		mm_DLSFault_Dectetion();
    		/* DCS 고장 감지 */
    		mm_DCSFault_Dectetion();
    		/* 모터 - 엔코더 고장 감지*/
    		if(error_list_flag[DCU_ENCODER_ERROR] == false) mm_Mo_EnFault_Dectetion(); //jeon_190715
    		/* 예기치 못한 잠김 풀림 고장 감지*/
    		mm_F07fault_Dectection();
    		/* Safety loop 고장 감지 */
    		mm_SafetyFault_Dectection();
    		/* 장애물 감지 */
    		mm_ObsFault_Dectection();
    		/* 열림 실패 고장 감지*/
    		mm_OpenFault_Detection();
    		/* mvb -rs485를 통한 절체 여부 감지*/

    		/*
    		 * 고장진단
    		 */
    		DiagnosisInDecisionControl(1,2);
   	}
		
		// LED 및 비상등 On/Off
		
		#ifdef USE_WATCHDOG
			/*
			 * todo emu: SW Watchdog 확인
			 * 		 Watchdog 살리면 정상동작 안함. 왜???
			 */
			/*
			 * Restart reinforced safety watch_dog at the given period.
			 */
			rswdt_restart(RSWDT);													// Feed를 주므로 Watch_dog Reset이 발생하지 않음
		#endif
	}
}

static void DiagnosisInMonitoring(void)
{
    /*
     * 고장진단 목록
     *  - 고전압 전원 모니터링
     *  - 저전압 전원 모니터링
     */
}

static void SupplyVoltageMonitor(void)
{
    uint32_t MotorPowerVoltage=0;
    
    /*
     * Opening/Closing 시에만(??) 저전압/고전압 오류 검출
     */
    if(MotorPowerVoltage!=0u)													// Opening/Closing 시 전압측정
    {
    	if(m_adc.PowerVoltage>2500)											// 고전압 오류
    	{
    		// 고전압 오류 플래그 셋
    		error_list_flag[DCU_ERROR] = true;
//    		error_list_flag[DCU_HIGH_VOLTAGE_ERROR] = true;
    	}
    	else if(m_adc.PowerVoltage<1500)										// 저전압 오류
    	{
    		// 저전압 오류 플래그 셋
    		error_list_flag[DCU_ERROR] = true;
//    		error_list_flag[DCU_LOW_VOLTAGE_ERROR] = true;
    	}
    	else																	// 정상전압으로 복귀
    	{
    		// 고전압 오류 플래그 클리어
//    		error_list_flag[DCU_HIGH_VOLTAGE_ERROR] = false;
    		// 저전압 오류 플래그 클리어
//    		error_list_flag[DCU_LOW_VOLTAGE_ERROR] = false;
    	}
    }
    else																		// Opened/Closed 시
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
	 *  이전 이벤트 입력과 다른 경우에만 mram에 저장한다. 차단스위치가 on될 때 계속 mram에 저장하면 안되기 때문
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
	 * DLS,DCS고장판단을 Closing->Closed, Opening->Opened 시에만 수행하면 안되는 이유 설명
	 *
	 * Closing->Closed 시 DLS or DCS가 안눌려 고장코드가 발생한 다음
	 * Closed->Closed 상태에서 다시 DLS, DCS가 눌리면 고장이 복귀되어야 하는데
	 * 열고 닫는 동작을 수행하지 않으면 복귀되지 않음
	 *
	 * -> 따라서 DLS,DCS고장체크는 항상 수행되어야 함
	 *    Closed->Closed 상태에서는 DLS,DCS가 눌려야 하고
	 *    이외의 상태에서는 DLS,DCS가 떼져야 함
	 */
	/*
	 * 닫힘완료 시 DLS1/2(F04/F08), DCS1/2(F05/F06) 고장(Open Circuit) 판단
	 */
	if( (mdc_DoorState==DOOR_CLOSED)	||
		((mdc_PreDoorState == DOOR_CLOSED) && (mdc_DoorState == DOOR_ERROR)))
	{
		#if 0
		{
			if(!mip_Input.di0_DLS1)														// 눌리지 않으면 DLS1 고장(Open Circuit)
			{
				error_list_flag[DCU_DLS1_FAULT] = true;									// 닫힘완료 시 DSL1 고장
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DLS1_FAULT] = false;								// 눌리면 DLS1 고장 복귀
			}

			if(!mip_Input.di1_DLS2)														// 눌리지 않으면 DLS2 고장(Open Circuit)
			{
				error_list_flag[DCU_DLS2_FAULT] = true;									// 닫힘완료 시 DLS2 고장
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DLS2_FAULT] = false;								// 눌리면 DLS2 고장 복귀
			}

			if(!mip_Input.di1_DCS1)														// 눌리지 않으면 DCS1 고장(Open Circuit)
			{
				error_list_flag[DCU_DCS1_FAULT] = true;									// 닫힘완료 시 DCS1 고장
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DCS1_FAULT] = false;								// 눌리면 DCS1 고장 복귀
			}

			if(!mip_Input.di1_DCS2)														// 눌리지 않으면 DCS2 고장(Open Circuit)
			{
				error_list_flag[DCU_DCS2_FAULT] = true;									// 닫힘완료 시 DCS2 고장
				error_list_flag[DCU_ERROR] = true;
			}
			else
			{
				error_list_flag[DCU_DCS2_FAULT] = false;								// 눌리면 DCS2 고장 복귀
			}
		}
		#else
		{
			if((!mip_Input.di0_DLS1) && (!mip_Input.di1_DLS2))								//둘다 풀린 경우에는 f07 발생 이후 dls 고장을 현시 한다.
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
			else if((mip_Input.di0_DLS1) && (!mip_Input.di1_DLS2))						// DLS 1은 눌렸으나 DLS2는 눌리지 않은경우
			{
				error_list_time[DCU_DLS1_FAULT] = osKernelSysTick();					// DLS 1은 정상이므로 측정 시간을 갱신 한다.
				Erase_fault(DCU_DLS1_FAULT);
				if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS2_FAULT]) > 5000)
				{
					FaultFlag_SET(DCU_DLS2_FAULT);
				}
			}
			else if((!mip_Input.di0_DLS1) && (mip_Input.di1_DLS2))						// DLS 1은 눌리지않았으나 DLS2는 눌린 경우
			{
				error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();
				Erase_fault(DCU_DLS2_FAULT);
				if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000)
				{
					FaultFlag_SET(DCU_DLS1_FAULT);
				}

			}
			else /* 둘다 눌린 경우*/
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
		if((mip_Input.di0_DLS1) && (mip_Input.di1_DLS2))								//둘다 풀린 경우에는 f07 발생 이후 dls 고장을 현시 한다.
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
					debug("# Opend 상태에서 dls 1/2고장 도어 에러 진입 \r\n");
				}
			}
		}
		else if((mip_Input.di0_DLS1) && (!mip_Input.di1_DLS2))						// DLS 1은 눌렸으나 DLS2는 눌리지 않은경우
		{
			error_list_time[DCU_DLS2_FAULT] = osKernelSysTick();					// DLS 2은 정상이므로 측정 시간을 갱신 한다.
			error_list_flag[DCU_DLS2_FAULT] = false;
			if(get_diff_tick(osKernelSysTick(),error_list_time[DCU_DLS1_FAULT]) > 5000)
			{
				FaultFlag_SET(DCU_DLS1_FAULT);
			}
		}
		else if((!mip_Input.di0_DLS1) && (mip_Input.di1_DLS2))						// DLS 1은 눌리지않았으나 DLS2는 눌린 경우
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
					debug("# CLOSED 상태에서 도어 에러 상태 진입 \r\n");
				}
			}
		}
		/*
		 *  DCS 1은 눌렀으나 DCS2가 눌리지 않은경우
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
					debug("# Opened 상태에서 도어 에러 상태 진입 \r\n");
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
				/* DLS 1 & 2가 OFF(false)로 고장이 난 상황에서는 닫힘 완료 판단을 6초뒤에 한다 6초 동안 계속 도어는
				 * 밀기 때문에 엔코더 고장이 아님에도 불구하고 고장을 출력함
				 * 닫힘 완료 부분에서는 엔코더 고장 판별을 하기가 어려우므로 제외 한다.
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
		
			if(PreviousEncoderValue != mmf_Encoder.Position) 						//엔코더 값이 변화 할 경우 (정상 상태)
			{
				MotorMonitoringFlag = false;
				error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
				error_list_time[DCU_MOTOR_ERROR] = osKernelSysTick();
			}
			else if((PreviousEncoderValue == mmf_Encoder.Position) && (m_adc.MotorCurrent < 500u))															// 엔코더 값이 변화 없을 경우 (고장 상태)
			{
				/*
				 *  도어가 멈추면 opening인 상태를 유지 할 것이므로 엔코더 변화가 없는 상태가 오래 지속 되면(opening 상황) 모터 고장으로 간주한다.
				 */
				if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_MOTOR_ERROR]) > 6000))
				{
					FaultFlag_SET(DCU_MOTOR_ERROR);
					mdc_PreDoorState = mdc_DoorState;
					mdc_DoorState = DOOR_ERROR;
				}
				else if((get_diff_tick(osKernelSysTick(),error_list_time[DCU_ENCODER_ERROR]) > 2000) && (MotorMonitoringFlag == false))
				{
					MotorMonitoringFlag = true;									//모터 고장과의 관계가 불분명 하기 때문에 바로 고장을 띄우진 않는다.
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
					debug("# 엔코더 펄스가 일정하므로 속도 제어 불가 slowmode 동작 \r\n");
				}
			}
			else	/* 이외의 상황은 모터-엔코더 고장을 감지하지 않는다 (ex) 장애물이 껴서 엔코더 변화가 없는 경우에 전류값이 높게 나옴)*/
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

	 * 예기치 못한 잠김 풀림 감지 : 1)닫힌 상태에서 DLS1/2가 풀린 경우 고장 동작을 수행 할수 있도록 함
	 *                				2) 도어 상태를 ERROR로 바꾸고 DecisionControl에서 고장 동작을 수행 할 수 있도록 한다.
	 */
#if 0 //jeon_190617 수정
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

			if(error_list_count[DCU_UNEXPECTED_UNLOCK] > 1) 		//300ms후 고장 발령, decisioncontrol에서 고장동작 실시
			{
				error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
				error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();					//f07이 발생한 시점으로 시작한다.
				FaultFlag_SET(DCU_UNEXPECTED_UNLOCK);
				mdc_PreDoorState = DOOR_CLOSED;
				mdc_DoorState = DOOR_ERROR;
			}
		}
		else if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* 열림 닫힘 동작을 반복하고 dls 쳤다가 풀린 경우 일 것임*/
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
		
		if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* 열림 닫힘 동작을 반복하고 dls 쳤다가 풀린 경우 일 것임*/
		{

			Erase_fault(DCU_UNEXPECTED_UNLOCK);
		}
	}
	else if((mip_Input.di0_Isolation == true)||(mip_Input.di1_EED == true))
	{
		Flag_f07 = false;
		error_list_flag[DCU_UNEXPECTED_UNLOCK] = pdFALSE;
		error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
		
		if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* 열림 닫힘 동작을 반복하고 dls 쳤다가 풀린 경우 일 것임*/
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
						
				if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* 열림 닫힘 동작을 반복하고 dls 쳤다가 풀린 경우 일 것임*/
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

				if(error_list_count[DCU_UNEXPECTED_UNLOCK] > 1) 		//300ms후 고장 발령, decisioncontrol에서 고장동작 실시
				{
					error_list_count[DCU_UNEXPECTED_UNLOCK] = 0;
					error_list_time[DCU_UNEXPECTED_UNLOCK] = osKernelSysTick();					//f07이 발생한 시점으로 시작한다.
					FaultFlag_SET(DCU_UNEXPECTED_UNLOCK);
					mdc_PreDoorState = DOOR_CLOSED;
					mdc_DoorState = DOOR_ERROR;
				}
			}
			else if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* 열림 닫힘 동작을 반복하고 dls 쳤다가 풀린 경우 일 것임*/
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
				if(get_diff_tick(osKernelSysTick(), safetytime) > 10000)		// 10초 동안 Safety Loop가 같지 않으면
				{
					FaultFlag_SET(DCU_SAFETYLOOP_FAULT);
				}
			}
			/*
			 * Safety Loop 1/2가 같으면 모든 도어가 Opening or Closing 동작을 정상적으로 완료한 것으로 판단
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
	 * 장애물 에러 감지 : 1)장애물 감지(3회 이상) 하고 열기 시작 할때 고장을 현시한다
	 *                    2)닫힘 완료가 된 경우에는 장애물이 없으므로 고장 삭제
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
	/* rs 485 슬레이브의 경우 - > 485 마스터로부터 절체 비트를 받은 상태
	 * l4 r1의 경우 -> tcms로 부터 절체비트를 수신
	 */
	//m_isMasterSlaveChange = false;
	if(m_isMasterSlaveChange == false)
	{
		/*
		 *  통신 에러가 발생 할 경우 절체를 수행한다.
		 */
		if((g_unDeviceID == DCU_R1_MCAR) || (g_unDeviceID == DCU_R1_TCAR) || (g_unDeviceID == DCU_L4_MCAR) || (g_unDeviceID == DCU_L4_TCAR))
		{
			if(error_list_flag[DCU_MVB_ERROR] == true)	//R1 , L4의 경우에는 MVB 고장만 발생했을때 절체
			{
				m_is485changeflag = false;
			}
		}
		else
		{
			if(error_list_flag[DCU_RS485_ERROR] == true ) // RS485 슬레이브는 485고장이 발생했을때 절체
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
	 * 열림 실패 고장 감지 : Opening 상태가 10초 이상 지속 되면 열림 실패 고장 현시
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
	 * 예를들어 Closed 상태가 10초 이상 지속할경우 시간 갱신을 하지 못한다.
	 * 그러다가 열림 명령을주면 열림실패고장을 띄우게 되고 절체가나오는 현상이발생함
	 * opening 상태가 아닌 경우에만 시간 값을 갱신하도록 수정했음 (기존에는 opend)
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
	 * 로터리 스위치 값이 읽히지 않음
	 * 잘 읽히는 DCU -> 마스터 퓨즈 뽑으면 OO으로 읽히고 다시 퓨즈 꼽으면 잘읽힘
	 * 안 읽히는 DCU -> 퓨즈둘다 꼽은 상태에서 읽으면 Fx 값으로 읽히긴 읽히는데 올리면 f0 f1 f0 f1 f8 지맘대로 출력됨
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
