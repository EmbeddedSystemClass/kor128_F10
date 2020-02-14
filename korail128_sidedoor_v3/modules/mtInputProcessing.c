/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtInputProcessing
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtInputProcessing.c
*********************************************************************/

#include "mtInputProcessing.h"
/* 초기화 후에만 스위치 입력정보를 받기 위한 초기화 정보확인을 위해 인클루드 */
/*## dependency mtDecisionControl */
#include "mtDecisionControl.h"
#include "mMotorOut.h"

extern osThreadId alivecheckTaskHandle;

struct _Input mip_Input = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
struct _Input mip_PreInput = {false,false,false,false,false,false,false,false,false,false};
struct Test_Input mip_Test = {0,0,0,0,0};

uint8_t DLSSwitchOn;	// jeon_190710 DLS완료스위치 : DLS 2개중 1개 이상 그리고 DCS2개중 1개 이상이 눌린 경우 true
uint8_t DCSSwitchOn;	// jeon_190710 DCS완료스위치 : DLS 2개중 1개 이상 그리고 DCS2개중 1개 이상이 눌린 경우 true


void mip_TaskInputProcessing(void const * argument)
{
    /*
     * Local variable definition
     */
    volatile static _Bool PrevButton[13]={0,}, CurrButton[13]={0,};
    volatile static GPIO_PinState TestPin=GPIO_PIN_SET, PreTestPin=GPIO_PIN_SET;							// no input -> pull-up
    volatile static uint32_t ButtonPushCnt[8]={0,}, GPIOInCnt=0;
    volatile static uint32_t PushCnt1=0, PushCnt2=0, PushCnt3=0, PushCnt4=0;
	uint8_t fmc_switch_value=0, fmc_rdo0_value=0, fmc_rdi0_value=0, fmc_rdi1_value=0, i=0, idx;
	uint32_t PreviousWakeTime = osKernelSysTick();
    int32_t cnt2=0, loopcnt=0;
	uint8_t CloseSwitch_cnt = 0;																			//DLS,DCS가 다 눌린 시간을 체크하는 변수 조건을 만족해야 플래그 셋이 된다.
	
	#ifdef DEBUG_SIMULATOR
		mip_Input.di0_DLS1 = true;
		mip_Input.di1_DLS2 = true;
		mip_Input.di1_DCS1 = true;
		mip_Input.di1_DCS2 = true;
		mip_Input.DoorClosedSwitchOn = true;
		
		mip_Input.di0_ZVR = true;	// 열차 정차 중
		mip_Input.di0_Isolation = false;	// 차단스위치 정상
		mip_Input.di1_EED = false;
		mip_Input.di1_EAD = false;
	#endif
	
    for(;;)
    {
        /* Place this task in the blocked state until it is time to run again. */
        //osDelayUntil(&PreviousWakeTime, 25UL);
        osDelayUntil(&PreviousWakeTime, 10UL);
        /*
         * 초기화 전 스위치입력(DLS,DCS)를 통한 닫힘완료 판단을 위해 초기화 전/후 항상 스위치 입력을 체크해야 함
         */
        /*
         * Test Pin 입력 체크
         */
		#if 0
		{
			TestPin = HAL_GPIO_ReadPin(MTEST_GPIO_Port, MTEST_Pin);
			if((PreTestPin != TestPin)&&(TestPin==GPIO_PIN_RESET))
			{
				fmc_switch_value = FMC_MOD_TL_RD;
				/*
				 * 열림/닫힘 스위치 입력
				 */
				if(fmc_switch_value==0xf0)
				{
					if(((m_isMasterSlave == MASTER_DCU)&&(!m_isSlaveRunCommand)) ||	// Master에서 열림명령 항상 입력
						((m_isMasterSlave == SLAVE_DCU)&&(m_isSlaveRunCommand)))		// Slave에서는 SlaveRun에서만 열림명령 입력
					{
						PushCnt1++;
						/*
						 * Input Test Pin OK
						 */
						if((PushCnt1%2)==1)
						{
							debug("## Test Button Push\r\n");
							mip_Input.di0_OpenCloseButton = true;
						}
						else
						{
							debug("## Test Button Release\r\n");
							mip_Input.di0_OpenCloseButton = false;
						}
					}
				}
				/*
				 * 중고장(F01) 발생
				 */
				else if(fmc_switch_value==0xff)
				{
					PushCnt3++;
					if((PushCnt3%2)==1)
					{
						debug("## F01 중고장 발생\r\n");
						error_list_flag[DCU_HARD_FAULT] = true;
						error_list_flag[DCU_ERROR] = true;
					}
					else
					{
						debug("## F01 중고장 발생 해지\r\n");
						error_list_flag[DCU_HARD_FAULT] = false;
					}
				}
				/*
				 * 경고장(F02) 발생
				 */
				else if(fmc_switch_value==0xfe)
				{
					PushCnt4++;
					if((PushCnt4%2)==1)
					{
						debug("## F02 경고장 발생\r\n");
						error_list_flag[DCU_MINOR_FAULT] = true;
						error_list_flag[DCU_ERROR] = true;
					}
					else
					{
						debug("## F02 경고장 발생 해지\r\n");
						error_list_flag[DCU_MINOR_FAULT] = false;
					}
				}
			}
			PreTestPin = TestPin;
		}
		#endif
		
		/*
		 * IO입력
		 */
	    /*
	     * todo : 모든 스위치 입력에 노이즈가 들어오므로 스위치 입력 전부 필터링을 수행해야 함
	     */
		#ifdef DEBUG_SIMULATOR	// 입력 받지 않음
		{
			/*
			 * 조합시험에서 Not Lock(잠기지 않은 상태) 정보 전송을 위해 주석처리
			 */
			#if 0
				if(((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
					mip_Input.DoorClosedSwitchOn = true;
				else
					mip_Input.DoorClosedSwitchOn = false;
			#endif
		}
		/*
		 */
		#else
		{
			/*
			 * 현차 적용 시 반드시 필터링 할 것
			 * 오세준과장 EMC 테스트 시(2018.10.29) 콘솔 끼니까 입력하지도 않았는데 차단스위치가 On으로 입력 되어 Free상태로 되어 도어 움직이지 않았음
			 */
			#if 0	// 신호 필터링 없이 바로 입력
			{
				/*
				 * 입력은 전부 무조껀 True는 입력 있음, false는 입력 없음
				 */
				fmc_rdi0_value = FMC_RDI_EN0;
				mip_Input.di0_OpenCloseButton = !getAbit(fmc_rdi0_value,DI0_OPEN_CLOSE);
				mip_Input.di0_ReOpen		  = !getAbit(fmc_rdi0_value,DI0_REOPEN);
				mip_Input.di0_ZVR			  = !getAbit(fmc_rdi0_value,DI0_ZVR);
				mip_Input.di0_Isolation		  = getAbit(fmc_rdi0_value,DI0_ISOLATION);
				mip_Input.di0_Bypass		  = !getAbit(fmc_rdi0_value,DI0_BYPASS);
				mip_Input.di0_SafetyLoopA	  = getAbit(fmc_rdi0_value,DI0_SAFETYLOOP_A);
				mip_Input.di0_SafetyLoopB	  = getAbit(fmc_rdi0_value,DI0_SAFETYLOOP_B);
				mip_Input.di0_DLS1			  = getAbit(fmc_rdi0_value,DI0_DLS1);
				
				fmc_rdi1_value = FMC_RDI_EN1;
				mip_Input.di1_DCS2			  = getAbit(fmc_rdi1_value,DI1_DCS2);
				mip_Input.di1_DCS1			  = getAbit(fmc_rdi1_value,DI1_DCS1);
				mip_Input.di1_EED			  = !getAbit(fmc_rdi1_value,DI1_EED);
				mip_Input.di1_EAD			  = getAbit(fmc_rdi1_value,DI1_EAD);
				mip_Input.di1_DLS2			  = getAbit(fmc_rdi1_value,DI1_DLS2);
				
				/*
				 * DLS 2개중 1개 이상, DCS2개중 1개 이상 눌려야 닫힘완료로 판단
				 */
				if(((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
					mip_Input.DoorClosedSwitchOn = true;
				else
					mip_Input.DoorClosedSwitchOn = false;
			}
			#elif 1// 신호 필터링
			{
		    	if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
		    	{
		    		loopcnt++;
		    		if(loopcnt>20)
		    		{
		    			loopcnt=0;
//			    			debug("# Master input Task 실행 안함\r\n");
		    		}
		    	}
		    	else if((m_isMasterSlave == SLAVE_DCU) && !m_isSlaveRunCommand)
		    	{
		    		loopcnt++;
		    		if(loopcnt>20)
		    		{
		    			loopcnt=0;
			    			//debug("# Slave input Task 실행 안함\r\n");
		    		}
		    	}
		    	else
		    	{
		    		mip_Input.RotarySwitch = FMC_MOD_TL_RD;
		    		for(idx=0; idx<13; idx++)
					{
						if(idx<8)	CurrButton[idx] = getAbit(FMC_RDI_EN0, idx);
						else		CurrButton[idx] = getAbit(FMC_RDI_EN1, (idx-8));

						/*
						 * 스위치 입력이 같은 상태가 일정시간 유지되면 입력 상태 업데이트, 1(입력있음) 0(입력없음)
						 */
						if(CurrButton[idx]==PrevButton[idx])
						{
							if(ButtonPushCnt[idx]>20)														// 10ms x 20 = 200ms 동안 입력이 유지되어야 입력값 사용
							{
								ButtonPushCnt[idx]=0;
								switch(idx)
								{
									case 0:	 mip_Input.di0_OpenCloseButton	= !CurrButton[idx]; break;
									case 1:	 mip_Input.di0_ReOpen			= !CurrButton[idx]; break;
									case 2:	 mip_Input.di0_ZVR				= !CurrButton[idx]; break;
									case 3:	 mip_Input.di0_Isolation		= CurrButton[idx]; break;
									case 4:	 mip_Input.di0_Bypass			= !CurrButton[idx]; break;
									case 5:	 mip_Input.di0_SafetyLoopA		= !CurrButton[idx]; break;
									case 6:	 mip_Input.di0_SafetyLoopB		= !CurrButton[idx]; break;
									case 7:	 mip_Input.di0_DLS1				= CurrButton[idx]; break;

									case 8:	 mip_Input.di1_DCS2				= CurrButton[idx]; break;
									case 9:	 mip_Input.di1_DCS1				= CurrButton[idx]; break;
									case 10: mip_Input.di1_EED				= !CurrButton[idx]; break;
									case 11: mip_Input.di1_EAD				= CurrButton[idx]; break;
									case 12: mip_Input.di1_DLS2				= CurrButton[idx]; break;
									default: break;
								}

								/*
								 * DLS 2개중 1개 이상, DCS2개중 1개 이상 눌려야 닫힘완료로 판단()한다.

								 */
#if 0 //jeon_190709								
								if(((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
									mip_Input.DoorClosedSwitchOn = true;
								else
									mip_Input.DoorClosedSwitchOn = false;
#else
								
								if((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true))
								{
									DLSSwitchOn = false;
								}
								else if(error_list_flag[DCU_DLS1_FAULT] == true)
								{
									if(mip_Input.di1_DLS2)
									{
										DLSSwitchOn = true;
									}
									else
									{
										DLSSwitchOn = false;
									}
								}
								else if(error_list_flag[DCU_DLS2_FAULT] == true)
								{
									if(mip_Input.di0_DLS1)
									{
										DLSSwitchOn = true;
									}
									else
									{
										DLSSwitchOn = false;
									}
								}	
								else
								{
									if((mip_Input.di0_DLS1) && (mip_Input.di1_DLS2))
									{
										DLSSwitchOn = true;
									}
									else
									{
										DLSSwitchOn = false;
									}
								}

								if((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true))
								{
									DCSSwitchOn = false;
								}
								else if(error_list_flag[DCU_DCS1_FAULT] == true)
								{
									if(mip_Input.di1_DCS2)
									{
										DCSSwitchOn = true;
									}
									else
									{
										DCSSwitchOn = false;
									}
								}
								else if(error_list_flag[DCU_DCS2_FAULT] == true)
								{
									if(mip_Input.di1_DCS1)
									{
										DCSSwitchOn = true;
									}
									else
									{
										DCSSwitchOn = false;
									}
								}	
								else
								{
									if((mip_Input.di1_DCS1) && (mip_Input.di1_DCS2))
									{
										DCSSwitchOn = true;
									}
									else
									{
										DCSSwitchOn = false;
									}
								}

								if((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true))
								{
									if((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true))
									{
										mip_Input.DoorClosedSwitchOn = true;
//										if(mip_Input.DoorClosedSwitchOn == true) mip_Input.DoorClosedSwitchOn = true;
//										else  mip_Input.DoorClosedSwitchOn = false;
									}
									else
									{
										if(error_list_flag[DCU_DCS1_FAULT] == true)
										{
											if(mip_Input.di1_DCS2)
											{
												mip_Input.DoorClosedSwitchOn = true;
											}
											else
											{
												mip_Input.DoorClosedSwitchOn = false;
											}
										}
										else if(error_list_flag[DCU_DCS2_FAULT] == true)
										{
											if(mip_Input.di1_DCS1)
											{
												mip_Input.DoorClosedSwitchOn = true;
											}
											else
											{
												mip_Input.DoorClosedSwitchOn = false;
											}
										}
										else
										{											
											if((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2))
											{
												mip_Input.DoorClosedSwitchOn = true;
											}
											else
											{
												mip_Input.DoorClosedSwitchOn = false;
											}
										}
									}
								}
								else if(error_list_flag[DCU_DLS1_FAULT] == true)
								{
									if((mip_Input.di1_DLS2)&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
									{
										mip_Input.DoorClosedSwitchOn = true;
									}
									else
									{
										mip_Input.DoorClosedSwitchOn = false;
									}
								}
								else if(error_list_flag[DCU_DLS2_FAULT] == true)
								{
									if((mip_Input.di0_DLS1)&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
									{
										mip_Input.DoorClosedSwitchOn = true;
									}
									else
									{
										mip_Input.DoorClosedSwitchOn = false;
									}
								}
								else
								{
									if((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true))
									{
										if((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))
										{
											mip_Input.DoorClosedSwitchOn = true;
										}	
										else
										{
											mip_Input.DoorClosedSwitchOn = false;
										}
									}
									else
									{			
										if(((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
										{
											mip_Input.DoorClosedSwitchOn = true;
										}
										else
										{
											mip_Input.DoorClosedSwitchOn = false;
										}
									}	
								}
#endif
							}
							else
							{
								ButtonPushCnt[idx]++;
							}
						}
						else																	// 스위치 입력이 다른 상태가 발생하면
						{
							ButtonPushCnt[idx]=0;												// 처음부터 다시 카운트
						}
						PrevButton[idx] = CurrButton[idx];
					}
		    		/*
		    		 *  todo : Monitoring Task에서 Input Task로 옮겼음 -> 정상동작하는지 확인 필요
		    		 *  절체 불가 고장이 없는 상태에서 절체 버튼을 눌러야 절체를 시작
		    		 *  절체 버튼은 마스터 cpu에서만 동작 한다.
		    		 */
		    		if(m_isMasterSlave == MASTER_DCU)
		    		{
		    			if((HAL_GPIO_ReadPin(MTEST_GPIO_Port,MTEST_Pin) == false) && (error_list_flag[DCU_CANT_SLAVE] == false))
		    			{
		    				m_isPacketSendToSlave = false;										// 마스터 cpu는 slave dcu에게 패킷을 보내지 않게 되고 slave dcu가 제어권을 가져오게 된다.
		    			}
		    		}
		    	}
			}
			#else
			{
		    	if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
		    	{
		    		loopcnt++;
		    		if(loopcnt>20)
		    		{
		    			loopcnt=0;
		    			debug("# Master input Task 실행 안함\r\n");
		    		}
		    	}
		    	else
		    	{
					fmc_rdi0_value = *(unsigned char*)(0x64000002);
					fmc_rdi1_value = *(unsigned char*)(0x64000003);
					//fmc_switch_value = *(unsigned char*)(0x64000004);

					/*
					 * Read from SMC_CPLD_RDI_EN0(0x61000004)   10001011
					 */
					if(mip_Test.OpenButton_Test_flag ==false) mip_Input.di0_OpenCloseButton= (     (~fmc_rdi0_value) & 0x01);	// DI00 : 운전실에 있는 열림/닫힘 스위치
					mip_Input.di0_ReOpen	= (((~fmc_rdi0_value)>>1) & 0x01);	// DI01 : 운전실에 있는 재열림 스위치(열차에서 신호선 끊어 놓는다고 함)
					mip_Input.di0_ZVR= (((~fmc_rdi0_value)>>2) & 0x01);	// DI02 : ZVR
					mip_Input.di0_Isolation = (   (fmc_rdi0_value>>3) & 0x01);	// DI03 : 차단스위치(구성품 설명서에 bypass라고 정의)
					mip_Input.di0_Bypass	= (((~fmc_rdi0_value)>>4) & 0x01);	// DI04 : DODBPS (Door Obstacle Bypass Switch?? 장애물 감지 안하는 신호)
					mip_Input.di0_SafetyLoopA		= (((fmc_rdi0_value)>>5) & 0x01);	// DI05 : 옆문이 닫히면 SafetyLoop1이 연결되어 신호 On이 입력됨
					mip_Input.di0_SafetyLoopB		= (((fmc_rdi0_value)>>6) & 0x01);	// DI06 : 자기 자신의 문이 닫히면 SafetyLoop2가 연결되어 신호 On이 입력됨
					if(mip_Test.DLS1_Test_flag == false) mip_Input.di0_DLS1	= (   (fmc_rdi0_value>>7) & 0x01);	// DI07 : DLS1

					/*
					 * Read from SMC_CPLD_RDI_EN0(0x61000006)   11111
					 */
					if(mip_Test.DCS2_Test_flag == false) mip_Input.di1_DCS2	= (       fmc_rdi1_value  & 0x01);	// DI08 : DCS2
					if(mip_Test.DCS1_Test_flag == false) mip_Input.di1_DCS1	= (   (fmc_rdi1_value>>1) & 0x01);	// DI09 : DCS1
					mip_Input.di1_EED		= (((~fmc_rdi1_value)>>2) & 0x01);	// DI10 : 내부 비상 스위치
					mip_Input.di1_EAD		= (   (fmc_rdi1_value>>3) & 0x01);	// DI11 : 외부 비상 스위치
					if(mip_Test.DLS2_Test_flag == false) mip_Input.di1_DLS2		= (   (fmc_rdi1_value>>4) & 0x01);	// DI12 : DLS2

					#if 0
					/*
					 * DLS 2개중 1개 이상, DCS2개중 1개 이상 눌려야 닫힘완료로 판단
					 */
					if(((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
						mip_Input.DoorClosedSwitchOn = true;
					else
						mip_Input.DoorClosedSwitchOn = false;
					#else
					if(((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
						mip_Input.DoorClosedSwitchOn = true;
					else
						mip_Input.DoorClosedSwitchOn = false;


					#endif
					mip_Input.RotarySwitch = fmc_switch_value;
		    	}
			}
			#endif
		}
		#endif
		
    	/*
    	 * IO입/출력 고장진단 (아직 구현 안됨)
    	 */
    	cnt2++;
    	if(cnt2>10)
    	{
    		cnt2=0;
    		
    		/*
    		 * 2018.11.09 오산공장에서 직접 확인한 내용
    		 */
			#if 0
    		{
				debug("ZVR :%d, O/C :%d, ReO/C: %d, ISO: %d, ByPass : %d, Safe A :%d, Safe B: %d, DLS 1:%d, DLS 2: %d, DCS1 :%d, DCS2 :%d, EED :%d, EAD :%d\r\n",
						mip_Input.di0_ZVR,
						mip_Input.di0_OpenCloseButton,
						mip_Input.di0_ReOpen,
						mip_Input.di0_Isolation,
						mip_Input.di0_Bypass,		// 안먹음
						mip_Input.di0_SafetyLoopA,
						mip_Input.di0_SafetyLoopB,
						mip_Input.di0_DLS1,
						mip_Input.di1_DLS2,
						mip_Input.di1_DCS1,			// 안먹음
						mip_Input.di1_DCS2,			// 안먹음
						mip_Input.di1_EED,			// 안먹음
						mip_Input.di1_EAD);			// 안먹음
			}
    		#endif
    	}
    }
}

void mip_MasterCodeMasterLED(_Bool LED)
{
	if(LED)
		HAL_GPIO_WritePin(MASTER_SLAVE_LED_GPIO_Port, MASTER_SLAVE_LED_Pin, GPIO_PIN_RESET);	// Master LED On
	else
		HAL_GPIO_WritePin(MASTER_SLAVE_LED_GPIO_Port, MASTER_SLAVE_LED_Pin, GPIO_PIN_SET);		// Master LED Off
}

void mip_SleveCodeSlaveRun(_Bool SlaveRun) {
	if(SlaveRun)
	{
		HAL_GPIO_WritePin(S_FND_CTL1_GPIO_Port, S_FND_CTL1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(S_FND_CTL2_GPIO_Port, S_FND_CTL2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(S_FND_CTL3_GPIO_Port, S_FND_CTL3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(S_MRAM_CTL_GPIO_Port, S_MRAM_CTL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(S_RTC_CTL_GPIO_Port, S_RTC_CTL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SMVB_SW_CTL_GPIO_Port, SMVB_SW_CTL_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(S_RelaySwitchingBySlave_GPIO_Port, S_RelaySwitchingBySlave_Pin, GPIO_PIN_RESET);	// Slave로 연결
	}
	else
	{
		HAL_GPIO_WritePin(S_FND_CTL1_GPIO_Port, S_FND_CTL1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_FND_CTL2_GPIO_Port, S_FND_CTL2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_FND_CTL3_GPIO_Port, S_FND_CTL3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_MRAM_CTL_GPIO_Port, S_MRAM_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_RTC_CTL_GPIO_Port, S_RTC_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SMVB_SW_CTL_GPIO_Port, SMVB_SW_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_RelaySwitchingBySlave_GPIO_Port, S_RelaySwitchingBySlave_Pin, GPIO_PIN_SET);	// Master로 연결
	}
}

void mip_EmergencyLamp(_Bool EmergencyLamp)
{
	if(EmergencyLamp)
	{
		//*((uint8_t*)(0x64000020)) = 0xff;
		//(*(unsigned char *)0x64000020) = (uint8_t)0xff;
		FMC_OUTEN0 =(uint8_t)0xff;
	}
	else
	{
		FMC_OUTEN0 =(uint8_t)0xfe;
	}
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtInputProcessing.c
*********************************************************************/
