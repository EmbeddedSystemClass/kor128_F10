/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtObstacleDetect
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtObstacleDetect.c
*********************************************************************/

#include "mtObstacleDetect.h"
#include "mtDecisionControl.h"
#include "mtInputProcessing.h"
#include "mtMotorFeedback.h"
#include "cmsis_os.h"


struct _detect mod_Detect = {false, false, 0 , 0};
extern _Bool mdc_FlagSlowMode;
volatile static int32_t ConstObsVelocity=10, ConstObsDetectCurr1=300, ConstObsDetectCurr23=160; //jeon_190514 orig: ConstObsDetectCurr23=150;
volatile static uint8_t ConstObstacleCurr1Cnt=5, ConstObstacleCurr23Cnt=20;
static void DetectObstacle(void);

void mod_TaskObstacleDetect(void const * argument)
{
    /*
     * Local variable definition
     */
	uint32_t PreviousWakeTime = osKernelSysTick();
    uint32_t Delay1msCnt = 0;
    uint8_t  ObsDetectCnt = 0;
	if(m_isMasterSlave == MASTER_DCU)
	{
	    debug("2. [Master Start] Obstacle Detect Task\r\n");
	}
	else // SLAVE_DCU
	{
    	while(m_isTaskExecution == false)
    	{
    		osDelay(200);
    	}
	    debug("2. [Slave Start] Obstacle Detect Task\r\n");
	    PreviousWakeTime = osKernelSysTick();
	}
    
    for(;;)
    {
		/* Place this task in the blocked state until it is time to run again. */
		osDelayUntil(&PreviousWakeTime, 10UL);
		/*
		 * todo : MotorFeedback Task���� 10ms���� ADC��ȯ�Լ� ȣ���ϴµ�...
		 *        ���⼭ ���� �ߺ��ؼ� ȣ���� �ʿ� ���� �� ����
		 *        ���������� Ȯ���� ����� �ϰ� �� �����ϸ� ���⼭ ADC��ȯ ȣ���ϴ°� ������ ��
		 */
		//mmf_ConversionADC();

		/*
		 * Closing State���� ���ڴ� ����� ������� ��� ��ֹ��� ���� �� ����Ƚ�� ����, �����Ϸ� �÷��� ��
		 */
		if((mdc_DoorState==DOOR_CLOSING))
		{
			DetectObstacle();
#if 0 //jeon_190624
			if(error_list_flag[DCU_ENCODER_ERROR] == true)
			{
				if(get_diff_tick(osKernelSysTick() , mdm_time.ClosingStart) > 5000)
				{
					if(m_adc.MotorCurrent>150u)
					{
						mmf_EndDecision.ClosedByCurrent = true;
						debug("# �����Ϸ� : ������ ���� �Ϸ� �÷��� �� \r\n");
					}
				}
			}
			else
			{
				if(mdc_FlagSlowMode == true)
				{
					if(get_diff_tick(osKernelSysTick() , mdm_time.ClosingStart) > 5000)
					{
						if(m_adc.MotorCurrent>150u)
						{
							mmf_EndDecision.ClosedByCurrent = true;
							debug("# �����Ϸ� : ������ ���� �Ϸ� �÷��� �� \r\n");
						}
					}
				}
			}
#else
			if(mip_Input.DoorClosedSwitchOn)									// DLS,DCS�� ������ �ʴ� ��������
			{
				mmf_EndDecision.ClosedByPosition = true;

			}
#endif
			/*
			 * DLS, DCS�� �����̹Ƿ� ���ÿ� �� �� ����߻� ��Ȳ�� ������� ����
			 */
			/*
			 * DLS, DCS �� �� ������ ���� ���¿��� �������� �� ��ֹ� ���� 
			 */
		}
		/*
		 * Opening State���� �����Ϸ� �÷��� ��
		 */
		else if(mdc_DoorState == DOOR_OPENING)
		{
			/*
			 *  �ʱ� Opening �ÿ� ������ ���� �ö󰡹Ƿ� ���� ��ɺο����� 3�ʵڿ� ������ ���� �Ϸ� �÷��� ��
			 */
			if(mdc_isInitComplete == true)
			{
				if(error_list_flag[DCU_ENCODER_ERROR] == true)
				{
					if((get_diff_tick(osKernelSysTick(),mdm_time.OpeningStart) > 5000) && (m_adc.MotorCurrent > 500u))
					{
						ObsDetectCnt++;
						if(ObsDetectCnt > 20) // �� 0.4��
						{
							ObsDetectCnt = 0;
							mmf_EndDecision.OpenedByCurrent = true;
							debug("# ���� ������ ���� �����Ϸ� �÷��� �� \r\n");
						}
					}
					else ObsDetectCnt = 0;

				}
				else
				{
//					if((get_diff_tick(osKernelSysTick(),mdm_time.OpeningStart) > 3000) && (m_adc.MotorCurrent > 300u))
					if((get_diff_tick(osKernelSysTick(),mdm_time.OpeningStart) > 3000) && (m_adc.MotorCurrent > 500u)) //jeon_190924
					{
						if(mmf_Encoder.Position > 3000)
						{
							ObsDetectCnt++;
							if(ObsDetectCnt > 20) // �� 0.4��
							{
								ObsDetectCnt = 0;
								mmf_EndDecision.OpenedByCurrent = true;
								debug("# ���� ������ ���� �����Ϸ� �÷��� �� \r\n");
							}
						}
#if 1 //jeon_190709 DLS #1,2 error test
						else if(((!mip_Input.di0_DLS1)&&(!mip_Input.di1_DLS2))||((!mip_Input.di1_DCS1)&&(!mip_Input.di1_DCS2)))
						{
							ObsDetectCnt++;
							if(ObsDetectCnt > 20) // �� 0.4��
							{
								ObsDetectCnt = 0;
								mmf_EndDecision.OpenedByCurrent = true;
								debug("# ���� ������ ���� �����Ϸ� �÷��� �� \r\n");
							}
						}
#endif
						else
						{
							if(mmf_PIDControl == PID_CONTROL_NONE)
							{
								ObsDetectCnt++;
								if(ObsDetectCnt > 20) // �� 0.4��
								{
									ObsDetectCnt = 0;
									mmf_EndDecision.OpenedByCurrent = true;
									debug("# ���� ������ ���� �����Ϸ� �÷��� �� \r\n");
								}
							}
						}
					}
					else ObsDetectCnt = 0;
				}

			}
			else
			{
				if((get_diff_tick(osKernelSysTick(),mdm_time.OpeningStart) > 3000) && (m_adc.MotorCurrent > 500u))
				{
					ObsDetectCnt++;
					if(ObsDetectCnt > 20) // �� 0.4��
					{
						ObsDetectCnt = 0;
						mmf_EndDecision.OpenedByCurrent = true;
						debug("# ���� ������ ���� �����Ϸ� �÷��� �� \r\n");
					}
				}
				else ObsDetectCnt = 0;
			}
		}
		else	// ���� ���� : opened , closed , error,
		{
		    ObsDetectCnt = 0;
		}
		Delay1msCnt++;
		if(Delay1msCnt>10)
		{
			Delay1msCnt=0;
//			debug("ObsVelocity(%d), ObsDetectCurr1(%d), ObsDetectCurr23(%d)\r\n",
//							ConstObsVelocity,
//							ConstObsDetectCurr1,
//							ConstObsDetectCurr23);
		}
    }
}

static void DetectObstacle(void)
{
    volatile static uint8_t ObstacleCurrCnt=0;
    volatile static int32_t prevPosition=0, Velocity=0;
    volatile static uint32_t slow_close_chk_time=0;
    volatile static uint32_t slow_close_chk_count=0;
    
	/*
	 * Encoder Mode
	 */
	if(!error_list_flag[DCU_ENCODER_ERROR])
	{
		//=====================================
		/*
		 * ���� �һ����/���μ� �� ��ֹ� ���� ����
		 *  �ʱ� �⵿���� ������ ������ư Ŭ�� �� 1�� ���ĺ��� ��ֹ� ����
		 * 
		 *	1ȸ ��ֹ� ����
		 *		- SlowMode�� �ƴϰ�,�ӵ���ȭ�� 3���� ���� ��((�������ڴ�-���翣�ڴ�)>3)
		 *		  120ms�̻� ������ 300���� ũ�� ��ֹ� ����
		 *	2ȸ ��ֹ� ����
		 *		- SlowMode�� �ƴϰ�, �ӵ���ȭ�� ���� ��((�������ڴ�-���翣�ڴ�)==0),
		 *		  36ms�̻� ������ 700���� ũ�� ��ֹ� ����
		 *	3ȸ ��ֹ� ����
		 *		- SlowMode�� �ƴϰ�, �ӵ���ȭ�� ���� ��((�������ڴ�-���翣�ڴ�)==0), 
		 *		  36ms�̻� ������ 700���� ũ�� ��ֹ� ����
		 */
		//=====================================
		
		/*
		 * ���ڴ� 10mm���� ū �Ÿ����� ���� ���� �� ��ֹ� ����
		 * 0.18[mm]:1[pulse] = 10[mm]:y[pulse]
		 * y[pulse] = 10/0.18 = 55[pulse]
		 * ��ֹ� ����(?) ���� �κ����� ��ֹ� ������� ��ֹ� ���� �ؾ��� �����Ҳ��� >55�κ��� �����ϸ� �ɵ�
		 */
		/*
		 * ���� �ʱ� �⵿������ ���� ��ֹ� ���ν� ������ ���� ������� ���� �� 1�� ���ĺ��� ��ֹ� ���� ����
		 */
#if 1 ///jeon_190710 ��ֹ� ���� ��ƾ ����
		if(mdc_FlagSlowMode) //slow mode
		{
			ConstObstacleCurr1Cnt=5;
			ConstObstacleCurr23Cnt=20; 
			ConstObsVelocity=2;        
			ConstObsDetectCurr1=500;  
			ConstObsDetectCurr23=500;
			
			if(get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 1000) //jeon_190822 500->1000
			{
				Velocity = prevPosition - mmf_Encoder.Position;
				
				if(mip_Input.DoorClosedSwitchOn)
				{
					// �����Ϸ� ����
					slow_close_chk_count = 0;
					ObstacleCurrCnt = 0;
					mmf_EndDecision.ClosedByPosition = true;
				}
				else if(mmf_Encoder.Position > 4000)		
				{
					if(DLSSwitchOn)
					{
						ObstacleCurrCnt = 0;
						slow_close_chk_count = 0;
						mmf_EndDecision.ClosedByPosition = true;
					}
					else
					{
						if(slow_close_chk_count == 0)
						{
							slow_close_chk_time = osKernelSysTick();
							slow_close_chk_count++;
						}
						else
						{
							if((get_diff_tick(osKernelSysTick(),slow_close_chk_time) > 500) && (Velocity<1))
							{
								if(m_adc.MotorCurrent>300)			// ���� ������ 300 �̻��� ���°�
								{
									ObstacleCurrCnt++;
									if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
									{
										mmf_PIDControl = PID_CONTROL_NONE;
										ObstacleCurrCnt = 0;
										mod_Detect.ObstacleDetectCnt++;
										mdc_isOpeningByObstacle = true;
										
										slow_close_chk_count = 0;
									}
								}
								else
								{
									ObstacleCurrCnt = 0;
								}
								
							}
							else
							{
								slow_close_chk_count++;
							}	
						}
					}
				}	
//				else if((mmf_Encoder.Position < 5) && (mip_Input.DoorClosedSwitchOn == true)) //jeon_190924
				else if(mmf_Encoder.Position < 5) 
				{
					slow_close_chk_count = 0;
					
					if(m_adc.MotorCurrent>500)			// ���� ������ 300 �̻��� ���°� jeon_190924 orig:300
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
						{
							mmf_EndDecision.ClosedByCurrent = true;
							mmf_EndDecision.ClosedByPosition = true;
							ObstacleCurrCnt = 0;
							mdc_PreDoorState = DOOR_CLOSING;
							mdc_DoorState = DOOR_CLOSED;
						}
					}
				}
				else
				{
					slow_close_chk_count = 0;
					
					if(Velocity<2) 
					{
						if(mod_Detect.ObstacleDetectCnt < 1)					// ��ֹ� ó�� ���� �õ�
						{
							if(m_adc.MotorCurrent>500)			// ���� ������ 300 �̻��� ���°� jeon_190924 orig:300
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
								{
									mmf_PIDControl = PID_CONTROL_NONE;
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
							else
							{
								ObstacleCurrCnt = 0;
							}
						}														// ��ֹ� 1ȸ ���� ����
						else if(mod_Detect.ObstacleDetectCnt == 1)				// ��ֹ� 2ȸ ���� �õ�
						{
							if(m_adc.MotorCurrent>500)			// ���� ������ 150 �̻��� ���°�
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
								{
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
						}														// ��ֹ� 2ȸ ���� ���ĺ���
						else if(mod_Detect.ObstacleDetectCnt >= 2)				// ��ֹ� 3ȸ ���� �õ�
						{
							if(m_adc.MotorCurrent>500)			// ���� ������ 150 �̻��� ���°�
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
								{
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
						}
					}
				}
			} 
			else
			{
				if(mip_Input.DoorClosedSwitchOn)
				{
					// �����Ϸ� ����
					mmf_EndDecision.ClosedByPosition = true;
				}
				else if(DLSSwitchOn) 			// ���� ������ 150 �̻��� ���°�
				{
					if(m_adc.MotorCurrent>300)			// ���� ������ 300 �̻��� ���°�
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
						{
							ObstacleCurrCnt = 0;
							mmf_EndDecision.ClosedByCurrent = true;
						}
					}
				}
#if 0 //jeon_190710				
				else if(mip_Input.DCSSwitchOn)			// ���� ������ 150 �̻��� ���°�
				{
					if(m_adc.MotorCurrent>400)			// ���� ������ 300 �̻��� ���°�
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
						{
							ObstacleCurrCnt = 0;
							mmf_EndDecision.ClosedByCurrent = true;
						}
					}
				}	
#endif				
				else if(mmf_Encoder.Position < 5)
				{
					if(m_adc.MotorCurrent>400)			// ���� ������ 300 �̻��� ���°�
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
						{
							ObstacleCurrCnt = 0;
							mmf_EndDecision.ClosedByCurrent = true;
							mdc_PreDoorState = DOOR_CLOSING;
							mdc_DoorState = DOOR_CLOSED;
						}
					}
				}
				ObstacleCurrCnt = 0;
			}
		}
		else //normal mode
		{
			ConstObstacleCurr1Cnt=5;
			ConstObstacleCurr23Cnt=20;
			ConstObsVelocity=4;
			ConstObsDetectCurr1=300;
			ConstObsDetectCurr23=150;
			
			if((get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 1000) && (mmf_Encoder.Position < 5)) //jeon_190822 500->1000
			{
				Velocity = prevPosition - mmf_Encoder.Position;
				
				if(mip_Input.DoorClosedSwitchOn)
				{
					// �����Ϸ� ����
					ObstacleCurrCnt = 0;
					mmf_EndDecision.ClosedByPosition = true;
				}	
				else if(((DCSSwitchOn)||(DLSSwitchOn)) && (m_adc.MotorCurrent>400))			// ���� ������ 150 �̻��� ���°�
				{
					ObstacleCurrCnt++;
					if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
					{
						ObstacleCurrCnt = 0;
						mmf_EndDecision.ClosedByCurrent = true;
					}
				}
				else if(mmf_Encoder.Position < 5)
				{
					if(m_adc.MotorCurrent>300)			// ���� ������ 300 �̻��� ���°�
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
						{
							mmf_EndDecision.ClosedByCurrent = true;
							mmf_EndDecision.ClosedByPosition = true;
							ObstacleCurrCnt = 0;
							mdc_PreDoorState = DOOR_CLOSING;
							mdc_DoorState = DOOR_CLOSED;
						}
					}
				}
				else
				{
					if(Velocity<2) 
					{
						if(mod_Detect.ObstacleDetectCnt < 1)					// ��ֹ� ó�� ���� �õ�
						{
							if(m_adc.MotorCurrent>300)			// ���� ������ 300 �̻��� ���°�
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms���� �����Ǹ�
								{
									mmf_PIDControl = PID_CONTROL_NONE;
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
							else
							{
								ObstacleCurrCnt = 0;
							}
						}														// ��ֹ� 1ȸ ���� ����
						else if(mod_Detect.ObstacleDetectCnt == 1)				// ��ֹ� 2ȸ ���� �õ�
						{
							if(m_adc.MotorCurrent>500)			// ���� ������ 150 �̻��� ���°�
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
								{
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
						}														// ��ֹ� 2ȸ ���� ���ĺ���
						else if(mod_Detect.ObstacleDetectCnt >= 2)				// ��ֹ� 3ȸ ���� �õ�
						{
							if(m_adc.MotorCurrent>500)			// ���� ������ 150 �̻��� ���°�
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
								{
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
						}
					}					
				}
			}
//			else if((get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 500) && (!mip_Input.DCSSwitchOn))
			else if(get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 1000) //jeon_190822 500->1000
			{
				Velocity = prevPosition - mmf_Encoder.Position;
				
				if(Velocity<ConstObsVelocity) 
				{
					if(mod_Detect.ObstacleDetectCnt < 1)					// ��ֹ� ó�� ���� �õ�
					{
						if(m_adc.MotorCurrent>ConstObsDetectCurr1)			// ���� ������ 300 �̻��� ���°�
						{
							ObstacleCurrCnt++;
							if(ObstacleCurrCnt>ConstObstacleCurr1Cnt-1)		// 50ms���� �����Ǹ�
							{
								mmf_PIDControl = PID_CONTROL_NONE;
								ObstacleCurrCnt = 0;
								mod_Detect.ObstacleDetectCnt++;
								mdc_isOpeningByObstacle = true;
							}
						}
						else
						{
							ObstacleCurrCnt = 0;
						}
					}														// ��ֹ� 1ȸ ���� ����
					else if(mod_Detect.ObstacleDetectCnt == 1)				// ��ֹ� 2ȸ ���� �õ�
					{
						if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// ���� ������ 150 �̻��� ���°�
						{
							ObstacleCurrCnt++;
							if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
							{
								ObstacleCurrCnt = 0;
								mod_Detect.ObstacleDetectCnt++;
								mdc_isOpeningByObstacle = true;
							}
						}
					}														// ��ֹ� 2ȸ ���� ���ĺ���
					else if(mod_Detect.ObstacleDetectCnt >= 2)				// ��ֹ� 3ȸ ���� �õ�
					{
						if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// ���� ������ 150 �̻��� ���°�
						{
							ObstacleCurrCnt++;
							if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
							{
								ObstacleCurrCnt = 0;
								mod_Detect.ObstacleDetectCnt++;
								mdc_isOpeningByObstacle = true;
							}
						}
					}
				}
			} 
			else
			{
				if(mip_Input.DoorClosedSwitchOn)
				{
					// �����Ϸ� ����
					ObstacleCurrCnt = 0;
					mmf_EndDecision.ClosedByPosition = true;
				}	
				else if(((DCSSwitchOn)||(DLSSwitchOn)) && (m_adc.MotorCurrent>400))			// ���� ������ 150 �̻��� ���°�
				{
					ObstacleCurrCnt++;
					if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
					{
						ObstacleCurrCnt = 0;
						mmf_EndDecision.ClosedByCurrent = true;
					}
				}
				ObstacleCurrCnt = 0;
			}
		}
			
			
		prevPosition = mmf_Encoder.Position;
#else
		if( (get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 1000) &&
			(mmf_Encoder.Position>DCU_OBS_POSITION) && 
//			(!mip_Input.DoorClosedSwitchOn) && //jeon_190709 �߰�
			(mmf_Encoder.Position<9000)) //&& jeon_190709 orig;2000
		{
			/*
			 * Slow Mode�� �̵��� �� ��ֹ� ���� ����
			 */
			if(mdc_FlagSlowMode)
			{
				ConstObstacleCurr1Cnt=5;
				ConstObstacleCurr23Cnt=10; //orig:10
				ConstObsVelocity=5;        //orig:5
				ConstObsDetectCurr1=400;  //orig:400
				ConstObsDetectCurr23=150;
			}
			/*
			 * Normal Mode�� �̵��� �� ��ֹ� ���� ����
			 */
			else
			{
				ConstObstacleCurr1Cnt=5;
				ConstObstacleCurr23Cnt=20;
				//ConstObsVelocity=10;
				ConstObsVelocity=4;
				ConstObsDetectCurr1=300;
				ConstObsDetectCurr23=150;
			}
			
			/*
			 * �ӵ��� �����ӵ� �����̸鼭
			 * (���ڴ��� ������ ���� ������ ��ٸ��� �����ϴٰ� 1ȸ�� ��ֹ� �������� �ʹ� ������
			 *  ��ֹ��� �ɷ� ���� �ӵ����Ϸ� ���ҵǾ��� �� ������ ���� ��ֹ� ���� �Ǵ�) 
			 */
			Velocity = prevPosition - mmf_Encoder.Position;
			if(Velocity<ConstObsVelocity) 
//			if((Velocity < ConstObsVelocity) && (mmf_Encoder.Position < 3500)) //jeon_190514
			{
				if(mod_Detect.ObstacleDetectCnt < 1)					// ��ֹ� ó�� ���� �õ�
				{
					if(m_adc.MotorCurrent>ConstObsDetectCurr1)			// ���� ������ 300 �̻��� ���°�
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr1Cnt-1)		// 50ms���� �����Ǹ�
						{
							mmf_PIDControl = PID_CONTROL_NONE;
							ObstacleCurrCnt = 0;
							mod_Detect.ObstacleDetectCnt++;
							mdc_isOpeningByObstacle = true;
						}
					}
					else
					{
						ObstacleCurrCnt = 0;
					}
				}														// ��ֹ� 1ȸ ���� ����
				else if(mod_Detect.ObstacleDetectCnt == 1)				// ��ֹ� 2ȸ ���� �õ�
				{
					if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// ���� ������ 150 �̻��� ���°�
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
						{
							ObstacleCurrCnt = 0;
							mod_Detect.ObstacleDetectCnt++;
							mdc_isOpeningByObstacle = true;
						}
					}
				}														// ��ֹ� 2ȸ ���� ���ĺ���
				else if(mod_Detect.ObstacleDetectCnt >= 2)				// ��ֹ� 3ȸ ���� �õ�
				{
					if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// ���� ������ 150 �̻��� ���°�
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms���� �����Ǹ�
						{
							ObstacleCurrCnt = 0;
							mod_Detect.ObstacleDetectCnt++;
							mdc_isOpeningByObstacle = true;
						}
					}
				}
			}
		}
		/*
		 * ���ڴ� 10mm���� ���� �Ÿ����� ������ �������� �����Ϸ� ����
		 */
		else
		{
			if(error_list_flag[DCU_ENCODER_ERROR] == false)
			{
				if(mdc_isFirstClosed == true)
				{
					if((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true))
					{
						if((mmf_Encoder.Position >= 0) && (mmf_Encoder.Position <=DCU_OBS_POSITION))
						{
							// �����Ϸ� ����
							if(m_adc.MotorCurrent>150u)
							{
								mmf_EndDecision.ClosedByCurrent = true;
								//debug("# �����Ϸ� : ������ ���� �Ϸ� �÷��� �� \r\n");
							}
						}
					}
					else
					{
						if((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true))
						{
							if((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true))
							{
								if((mmf_Encoder.Position >= 0) && (mmf_Encoder.Position <=DCU_OBS_POSITION))
								{
									// �����Ϸ� ����
									if(m_adc.MotorCurrent>150u)
									{
										mmf_EndDecision.ClosedByCurrent = true;
										//debug("# �����Ϸ� : ������ ���� �Ϸ� �÷��� �� \r\n");
									}
								}
							}
							else if(error_list_flag[DCU_DLS1_FAULT] == true)
							{
								if(mip_Input.di1_DLS2)
								{
									mmf_EndDecision.ClosedByPosition = true;
								}
							}
							else if(error_list_flag[DCU_DLS2_FAULT] == true)
							{
								if(mip_Input.di0_DLS1)
								{
									mmf_EndDecision.ClosedByPosition = true;
								}
							}
							else
							{
								if((mip_Input.di1_DLS2) || (mip_Input.di0_DLS1))
								{
									mmf_EndDecision.ClosedByPosition = true;
								}
							}
						}
						else if(mip_Input.DoorClosedSwitchOn)
						{
							// �����Ϸ� ����
							mmf_EndDecision.ClosedByPosition = true;
						}
						else if((mmf_Encoder.Position >= 0) && (mmf_Encoder.Position <=DCU_OBS_POSITION))
						{
							// �����Ϸ� ����
							if(m_adc.MotorCurrent>150u)
							{
								mmf_EndDecision.ClosedByCurrent = true;
								//debug("# �����Ϸ� : ������ ���� �Ϸ� �÷��� �� \r\n");
							}
						}
					}
				}
				else //�ʱ�ȭ �����
				{
					if(mip_Input.DoorClosedSwitchOn)
					{
						// �����Ϸ� ����
						mmf_EndDecision.ClosedByPosition = true;
					}
					else if((mmf_Encoder.Position >= 0) && (mmf_Encoder.Position <=DCU_OBS_POSITION))
					{
						// �����Ϸ� ����
						if(m_adc.MotorCurrent>150u)
						{
							mmf_EndDecision.ClosedByCurrent = true;
							//debug("# �����Ϸ� : ������ ���� �Ϸ� �÷��� �� \r\n");
						}
					}
				}
			}
			else
			{
				if(get_diff_tick(osKernelSysTick() , mdm_time.ClosingStart) > 5000)
				{
					if(m_adc.MotorCurrent>150u)
					{
						mmf_EndDecision.ClosedByCurrent = true;
						//debug("# �����Ϸ� : ������ ���� �Ϸ� �÷��� �� \r\n");
					}
				}
			}
		}
		prevPosition = mmf_Encoder.Position;
#endif		
	}
	/*
	 * B_emf Mode
	 */
	else
	{
//        	if((mmf_Bemf.Position>(float)50) && (mmf_Bemf.Position<(float)2000))// B_emfPos 50 ~ 2000��������
		if(!mip_Input.DoorClosedSwitchOn)									// DLS,DCS�� ������ �ʴ� ��������
		{
			if(m_adc.MotorCurrent>500u)										// �������� 500��ŭ �����
			{
				ObstacleCurrCnt++;
				if(ObstacleCurrCnt>3u)											// 300ms���� �����Ǹ�
				{
					ObstacleCurrCnt = 0;
					mod_Detect.ObstacleDetectCnt++;
				}
			}
			else
			{
				// No Action -> �������� ����
			}
		}
		else
		{
			/*
			 * bemf ���۶� ���� �Ϸ� �Ǵ��� ���������� 3�� �Ŀ� ������ �����Ϸ� �Ǵ��� ����
			 */
			if((m_adc.MotorCurrent>300u) && (get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > (mdm_time.CloseConfigtime+3000))) 									
			{
				ObstacleCurrCnt++;
				if(ObstacleCurrCnt>3u)											// 300ms���� �����Ǹ�
				{
					ObstacleCurrCnt = 0;
					mmf_EndDecision.ClosedByCurrent = true;
				}
			}
		}

	}
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtObstacleDetect.c
*********************************************************************/
