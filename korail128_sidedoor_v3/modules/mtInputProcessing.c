/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtInputProcessing
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtInputProcessing.c
*********************************************************************/

#include "mtInputProcessing.h"
/* �ʱ�ȭ �Ŀ��� ����ġ �Է������� �ޱ� ���� �ʱ�ȭ ����Ȯ���� ���� ��Ŭ��� */
/*## dependency mtDecisionControl */
#include "mtDecisionControl.h"
#include "mMotorOut.h"

extern osThreadId alivecheckTaskHandle;

struct _Input mip_Input = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
struct _Input mip_PreInput = {false,false,false,false,false,false,false,false,false,false};
struct Test_Input mip_Test = {0,0,0,0,0};

uint8_t DLSSwitchOn;	// jeon_190710 DLS�Ϸὺ��ġ : DLS 2���� 1�� �̻� �׸��� DCS2���� 1�� �̻��� ���� ��� true
uint8_t DCSSwitchOn;	// jeon_190710 DCS�Ϸὺ��ġ : DLS 2���� 1�� �̻� �׸��� DCS2���� 1�� �̻��� ���� ��� true


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
	uint8_t CloseSwitch_cnt = 0;																			//DLS,DCS�� �� ���� �ð��� üũ�ϴ� ���� ������ �����ؾ� �÷��� ���� �ȴ�.
	
	#ifdef DEBUG_SIMULATOR
		mip_Input.di0_DLS1 = true;
		mip_Input.di1_DLS2 = true;
		mip_Input.di1_DCS1 = true;
		mip_Input.di1_DCS2 = true;
		mip_Input.DoorClosedSwitchOn = true;
		
		mip_Input.di0_ZVR = true;	// ���� ���� ��
		mip_Input.di0_Isolation = false;	// ���ܽ���ġ ����
		mip_Input.di1_EED = false;
		mip_Input.di1_EAD = false;
	#endif
	
    for(;;)
    {
        /* Place this task in the blocked state until it is time to run again. */
        //osDelayUntil(&PreviousWakeTime, 25UL);
        osDelayUntil(&PreviousWakeTime, 10UL);
        /*
         * �ʱ�ȭ �� ����ġ�Է�(DLS,DCS)�� ���� �����Ϸ� �Ǵ��� ���� �ʱ�ȭ ��/�� �׻� ����ġ �Է��� üũ�ؾ� ��
         */
        /*
         * Test Pin �Է� üũ
         */
		#if 0
		{
			TestPin = HAL_GPIO_ReadPin(MTEST_GPIO_Port, MTEST_Pin);
			if((PreTestPin != TestPin)&&(TestPin==GPIO_PIN_RESET))
			{
				fmc_switch_value = FMC_MOD_TL_RD;
				/*
				 * ����/���� ����ġ �Է�
				 */
				if(fmc_switch_value==0xf0)
				{
					if(((m_isMasterSlave == MASTER_DCU)&&(!m_isSlaveRunCommand)) ||	// Master���� ������� �׻� �Է�
						((m_isMasterSlave == SLAVE_DCU)&&(m_isSlaveRunCommand)))		// Slave������ SlaveRun������ ������� �Է�
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
				 * �߰���(F01) �߻�
				 */
				else if(fmc_switch_value==0xff)
				{
					PushCnt3++;
					if((PushCnt3%2)==1)
					{
						debug("## F01 �߰��� �߻�\r\n");
						error_list_flag[DCU_HARD_FAULT] = true;
						error_list_flag[DCU_ERROR] = true;
					}
					else
					{
						debug("## F01 �߰��� �߻� ����\r\n");
						error_list_flag[DCU_HARD_FAULT] = false;
					}
				}
				/*
				 * �����(F02) �߻�
				 */
				else if(fmc_switch_value==0xfe)
				{
					PushCnt4++;
					if((PushCnt4%2)==1)
					{
						debug("## F02 ����� �߻�\r\n");
						error_list_flag[DCU_MINOR_FAULT] = true;
						error_list_flag[DCU_ERROR] = true;
					}
					else
					{
						debug("## F02 ����� �߻� ����\r\n");
						error_list_flag[DCU_MINOR_FAULT] = false;
					}
				}
			}
			PreTestPin = TestPin;
		}
		#endif
		
		/*
		 * IO�Է�
		 */
	    /*
	     * todo : ��� ����ġ �Է¿� ����� �����Ƿ� ����ġ �Է� ���� ���͸��� �����ؾ� ��
	     */
		#ifdef DEBUG_SIMULATOR	// �Է� ���� ����
		{
			/*
			 * ���ս��迡�� Not Lock(����� ���� ����) ���� ������ ���� �ּ�ó��
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
			 * ���� ���� �� �ݵ�� ���͸� �� ��
			 * �����ذ��� EMC �׽�Ʈ ��(2018.10.29) �ܼ� ���ϱ� �Է������� �ʾҴµ� ���ܽ���ġ�� On���� �Է� �Ǿ� Free���·� �Ǿ� ���� �������� �ʾ���
			 */
			#if 0	// ��ȣ ���͸� ���� �ٷ� �Է�
			{
				/*
				 * �Է��� ���� ������ True�� �Է� ����, false�� �Է� ����
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
				 * DLS 2���� 1�� �̻�, DCS2���� 1�� �̻� ������ �����Ϸ�� �Ǵ�
				 */
				if(((mip_Input.di0_DLS1)||(mip_Input.di1_DLS2))&&((mip_Input.di1_DCS1)||(mip_Input.di1_DCS2)))
					mip_Input.DoorClosedSwitchOn = true;
				else
					mip_Input.DoorClosedSwitchOn = false;
			}
			#elif 1// ��ȣ ���͸�
			{
		    	if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
		    	{
		    		loopcnt++;
		    		if(loopcnt>20)
		    		{
		    			loopcnt=0;
//			    			debug("# Master input Task ���� ����\r\n");
		    		}
		    	}
		    	else if((m_isMasterSlave == SLAVE_DCU) && !m_isSlaveRunCommand)
		    	{
		    		loopcnt++;
		    		if(loopcnt>20)
		    		{
		    			loopcnt=0;
			    			//debug("# Slave input Task ���� ����\r\n");
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
						 * ����ġ �Է��� ���� ���°� �����ð� �����Ǹ� �Է� ���� ������Ʈ, 1(�Է�����) 0(�Է¾���)
						 */
						if(CurrButton[idx]==PrevButton[idx])
						{
							if(ButtonPushCnt[idx]>20)														// 10ms x 20 = 200ms ���� �Է��� �����Ǿ�� �Է°� ���
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
								 * DLS 2���� 1�� �̻�, DCS2���� 1�� �̻� ������ �����Ϸ�� �Ǵ�()�Ѵ�.

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
						else																	// ����ġ �Է��� �ٸ� ���°� �߻��ϸ�
						{
							ButtonPushCnt[idx]=0;												// ó������ �ٽ� ī��Ʈ
						}
						PrevButton[idx] = CurrButton[idx];
					}
		    		/*
		    		 *  todo : Monitoring Task���� Input Task�� �Ű��� -> �������ϴ��� Ȯ�� �ʿ�
		    		 *  ��ü �Ұ� ������ ���� ���¿��� ��ü ��ư�� ������ ��ü�� ����
		    		 *  ��ü ��ư�� ������ cpu������ ���� �Ѵ�.
		    		 */
		    		if(m_isMasterSlave == MASTER_DCU)
		    		{
		    			if((HAL_GPIO_ReadPin(MTEST_GPIO_Port,MTEST_Pin) == false) && (error_list_flag[DCU_CANT_SLAVE] == false))
		    			{
		    				m_isPacketSendToSlave = false;										// ������ cpu�� slave dcu���� ��Ŷ�� ������ �ʰ� �ǰ� slave dcu�� ������� �������� �ȴ�.
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
		    			debug("# Master input Task ���� ����\r\n");
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
					if(mip_Test.OpenButton_Test_flag ==false) mip_Input.di0_OpenCloseButton= (     (~fmc_rdi0_value) & 0x01);	// DI00 : �����ǿ� �ִ� ����/���� ����ġ
					mip_Input.di0_ReOpen	= (((~fmc_rdi0_value)>>1) & 0x01);	// DI01 : �����ǿ� �ִ� �翭�� ����ġ(�������� ��ȣ�� ���� ���´ٰ� ��)
					mip_Input.di0_ZVR= (((~fmc_rdi0_value)>>2) & 0x01);	// DI02 : ZVR
					mip_Input.di0_Isolation = (   (fmc_rdi0_value>>3) & 0x01);	// DI03 : ���ܽ���ġ(����ǰ ������ bypass��� ����)
					mip_Input.di0_Bypass	= (((~fmc_rdi0_value)>>4) & 0x01);	// DI04 : DODBPS (Door Obstacle Bypass Switch?? ��ֹ� ���� ���ϴ� ��ȣ)
					mip_Input.di0_SafetyLoopA		= (((fmc_rdi0_value)>>5) & 0x01);	// DI05 : ������ ������ SafetyLoop1�� ����Ǿ� ��ȣ On�� �Էµ�
					mip_Input.di0_SafetyLoopB		= (((fmc_rdi0_value)>>6) & 0x01);	// DI06 : �ڱ� �ڽ��� ���� ������ SafetyLoop2�� ����Ǿ� ��ȣ On�� �Էµ�
					if(mip_Test.DLS1_Test_flag == false) mip_Input.di0_DLS1	= (   (fmc_rdi0_value>>7) & 0x01);	// DI07 : DLS1

					/*
					 * Read from SMC_CPLD_RDI_EN0(0x61000006)   11111
					 */
					if(mip_Test.DCS2_Test_flag == false) mip_Input.di1_DCS2	= (       fmc_rdi1_value  & 0x01);	// DI08 : DCS2
					if(mip_Test.DCS1_Test_flag == false) mip_Input.di1_DCS1	= (   (fmc_rdi1_value>>1) & 0x01);	// DI09 : DCS1
					mip_Input.di1_EED		= (((~fmc_rdi1_value)>>2) & 0x01);	// DI10 : ���� ��� ����ġ
					mip_Input.di1_EAD		= (   (fmc_rdi1_value>>3) & 0x01);	// DI11 : �ܺ� ��� ����ġ
					if(mip_Test.DLS2_Test_flag == false) mip_Input.di1_DLS2		= (   (fmc_rdi1_value>>4) & 0x01);	// DI12 : DLS2

					#if 0
					/*
					 * DLS 2���� 1�� �̻�, DCS2���� 1�� �̻� ������ �����Ϸ�� �Ǵ�
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
    	 * IO��/��� �������� (���� ���� �ȵ�)
    	 */
    	cnt2++;
    	if(cnt2>10)
    	{
    		cnt2=0;
    		
    		/*
    		 * 2018.11.09 ������忡�� ���� Ȯ���� ����
    		 */
			#if 0
    		{
				debug("ZVR :%d, O/C :%d, ReO/C: %d, ISO: %d, ByPass : %d, Safe A :%d, Safe B: %d, DLS 1:%d, DLS 2: %d, DCS1 :%d, DCS2 :%d, EED :%d, EAD :%d\r\n",
						mip_Input.di0_ZVR,
						mip_Input.di0_OpenCloseButton,
						mip_Input.di0_ReOpen,
						mip_Input.di0_Isolation,
						mip_Input.di0_Bypass,		// �ȸ���
						mip_Input.di0_SafetyLoopA,
						mip_Input.di0_SafetyLoopB,
						mip_Input.di0_DLS1,
						mip_Input.di1_DLS2,
						mip_Input.di1_DCS1,			// �ȸ���
						mip_Input.di1_DCS2,			// �ȸ���
						mip_Input.di1_EED,			// �ȸ���
						mip_Input.di1_EAD);			// �ȸ���
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
		HAL_GPIO_WritePin(S_RelaySwitchingBySlave_GPIO_Port, S_RelaySwitchingBySlave_Pin, GPIO_PIN_RESET);	// Slave�� ����
	}
	else
	{
		HAL_GPIO_WritePin(S_FND_CTL1_GPIO_Port, S_FND_CTL1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_FND_CTL2_GPIO_Port, S_FND_CTL2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_FND_CTL3_GPIO_Port, S_FND_CTL3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_MRAM_CTL_GPIO_Port, S_MRAM_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_RTC_CTL_GPIO_Port, S_RTC_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SMVB_SW_CTL_GPIO_Port, SMVB_SW_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_RelaySwitchingBySlave_GPIO_Port, S_RelaySwitchingBySlave_Pin, GPIO_PIN_SET);	// Master�� ����
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
