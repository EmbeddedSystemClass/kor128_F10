/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtObstacleDetect
//!	Generated Date	: 토, 1, 7 2017  
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
		 * todo : MotorFeedback Task에서 10ms마다 ADC변환함수 호출하는데...
		 *        여기서 굳이 중복해서 호출할 필요 없을 것 같음
		 *        재진씨한테 확인해 보라고 하고 잘 동작하면 여기서 ADC변환 호출하는거 삭제할 것
		 */
		//mmf_ConversionADC();

		/*
		 * Closing State에서 엔코더 고장과 상관없이 모두 장애물을 감지 시 감지횟수 설정, 닫힘완료 플래그 셋
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
						debug("# 닫힘완료 : 전류로 닫힘 완료 플래그 셋 \r\n");
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
							debug("# 닫힘완료 : 전류로 닫힘 완료 플래그 셋 \r\n");
						}
					}
				}
			}
#else
			if(mip_Input.DoorClosedSwitchOn)									// DLS,DCS가 눌리지 않는 구간에서
			{
				mmf_EndDecision.ClosedByPosition = true;

			}
#endif
			/*
			 * DLS, DCS는 병렬이므로 동시에 둘 다 고장발생 상황은 고려하지 않음
			 */
			/*
			 * DLS, DCS 둘 다 눌리지 않은 상태에서 전류증가 시 장애물 감지 
			 */
		}
		/*
		 * Opening State에서 열림완료 플래그 셋
		 */
		else if(mdc_DoorState == DOOR_OPENING)
		{
			/*
			 *  초기 Opening 시에 전류가 많이 올라가므로 열림 명령부여한지 3초뒤에 전류로 열림 완료 플래그 셋
			 */
			if(mdc_isInitComplete == true)
			{
				if(error_list_flag[DCU_ENCODER_ERROR] == true)
				{
					if((get_diff_tick(osKernelSysTick(),mdm_time.OpeningStart) > 5000) && (m_adc.MotorCurrent > 500u))
					{
						ObsDetectCnt++;
						if(ObsDetectCnt > 20) // 약 0.4초
						{
							ObsDetectCnt = 0;
							mmf_EndDecision.OpenedByCurrent = true;
							debug("# 전류 값으로 인한 열림완료 플래그 셋 \r\n");
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
							if(ObsDetectCnt > 20) // 약 0.4초
							{
								ObsDetectCnt = 0;
								mmf_EndDecision.OpenedByCurrent = true;
								debug("# 전류 값으로 인한 열림완료 플래그 셋 \r\n");
							}
						}
#if 1 //jeon_190709 DLS #1,2 error test
						else if(((!mip_Input.di0_DLS1)&&(!mip_Input.di1_DLS2))||((!mip_Input.di1_DCS1)&&(!mip_Input.di1_DCS2)))
						{
							ObsDetectCnt++;
							if(ObsDetectCnt > 20) // 약 0.4초
							{
								ObsDetectCnt = 0;
								mmf_EndDecision.OpenedByCurrent = true;
								debug("# 전류 값으로 인한 열림완료 플래그 셋 \r\n");
							}
						}
#endif
						else
						{
							if(mmf_PIDControl == PID_CONTROL_NONE)
							{
								ObsDetectCnt++;
								if(ObsDetectCnt > 20) // 약 0.4초
								{
									ObsDetectCnt = 0;
									mmf_EndDecision.OpenedByCurrent = true;
									debug("# 전류 값으로 인한 열림완료 플래그 셋 \r\n");
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
					if(ObsDetectCnt > 20) // 약 0.4초
					{
						ObsDetectCnt = 0;
						mmf_EndDecision.OpenedByCurrent = true;
						debug("# 전류 값으로 인한 열림완료 플래그 셋 \r\n");
					}
				}
				else ObsDetectCnt = 0;
			}
		}
		else	// 도어 상태 : opened , closed , error,
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
		 * 기존 소사원시/수인선 꺼 장애물 감지 조건
		 *  초기 기동전류 때문에 열림버튼 클릭 후 1초 이후부터 장애물 감지
		 * 
		 *	1회 장애물 감지
		 *		- SlowMode가 아니고,속도변화가 3보다 작을 때((이전엔코더-현재엔코더)>3)
		 *		  120ms이상 전류가 300보다 크면 장애물 감지
		 *	2회 장애물 감지
		 *		- SlowMode가 아니고, 속도변화가 없을 때((이전엔코더-현재엔코더)==0),
		 *		  36ms이상 전류가 700보다 크면 장애물 감지
		 *	3회 장애물 감지
		 *		- SlowMode가 아니고, 속도변화가 없을 때((이전엔코더-현재엔코더)==0), 
		 *		  36ms이상 전류가 700보다 크면 장애물 감지
		 */
		//=====================================
		
		/*
		 * 엔코더 10mm보다 큰 거리에서 전류 증가 시 장애물 감지
		 * 0.18[mm]:1[pulse] = 10[mm]:y[pulse]
		 * y[pulse] = 10/0.18 = 55[pulse]
		 * 장애물 막대(?) 얇은 부분으로 장애물 대었을때 장애물 감지 해야함 수정할꺼면 >55부분을 수정하면 될듯
		 */
		/*
		 * 모터 초기 기동전류로 인한 장애물 오인식 방지를 위해 닫힘명령 수신 후 1초 이후부터 장애물 감지 수행
		 */
#if 1 ///jeon_190710 장애물 감지 루틴 수정
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
					// 닫힘완료 수행
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
								if(m_adc.MotorCurrent>300)			// 모터 전류가 300 이상인 상태가
								{
									ObstacleCurrCnt++;
									if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
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
					
					if(m_adc.MotorCurrent>500)			// 모터 전류가 300 이상인 상태가 jeon_190924 orig:300
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
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
						if(mod_Detect.ObstacleDetectCnt < 1)					// 장애물 처음 감지 시도
						{
							if(m_adc.MotorCurrent>500)			// 모터 전류가 300 이상인 상태가 jeon_190924 orig:300
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
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
						}														// 장애물 1회 감지 이후
						else if(mod_Detect.ObstacleDetectCnt == 1)				// 장애물 2회 감지 시도
						{
							if(m_adc.MotorCurrent>500)			// 모터 전류가 150 이상인 상태가
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
								{
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
						}														// 장애물 2회 감지 이후부터
						else if(mod_Detect.ObstacleDetectCnt >= 2)				// 장애물 3회 감지 시도
						{
							if(m_adc.MotorCurrent>500)			// 모터 전류가 150 이상인 상태가
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
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
					// 닫힘완료 수행
					mmf_EndDecision.ClosedByPosition = true;
				}
				else if(DLSSwitchOn) 			// 모터 전류가 150 이상인 상태가
				{
					if(m_adc.MotorCurrent>300)			// 모터 전류가 300 이상인 상태가
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
						{
							ObstacleCurrCnt = 0;
							mmf_EndDecision.ClosedByCurrent = true;
						}
					}
				}
#if 0 //jeon_190710				
				else if(mip_Input.DCSSwitchOn)			// 모터 전류가 150 이상인 상태가
				{
					if(m_adc.MotorCurrent>400)			// 모터 전류가 300 이상인 상태가
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
						{
							ObstacleCurrCnt = 0;
							mmf_EndDecision.ClosedByCurrent = true;
						}
					}
				}	
#endif				
				else if(mmf_Encoder.Position < 5)
				{
					if(m_adc.MotorCurrent>400)			// 모터 전류가 300 이상인 상태가
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
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
					// 닫힘완료 수행
					ObstacleCurrCnt = 0;
					mmf_EndDecision.ClosedByPosition = true;
				}	
				else if(((DCSSwitchOn)||(DLSSwitchOn)) && (m_adc.MotorCurrent>400))			// 모터 전류가 150 이상인 상태가
				{
					ObstacleCurrCnt++;
					if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
					{
						ObstacleCurrCnt = 0;
						mmf_EndDecision.ClosedByCurrent = true;
					}
				}
				else if(mmf_Encoder.Position < 5)
				{
					if(m_adc.MotorCurrent>300)			// 모터 전류가 300 이상인 상태가
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
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
						if(mod_Detect.ObstacleDetectCnt < 1)					// 장애물 처음 감지 시도
						{
							if(m_adc.MotorCurrent>300)			// 모터 전류가 300 이상인 상태가
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt-1)		// 50ms동안 유지되면
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
						}														// 장애물 1회 감지 이후
						else if(mod_Detect.ObstacleDetectCnt == 1)				// 장애물 2회 감지 시도
						{
							if(m_adc.MotorCurrent>500)			// 모터 전류가 150 이상인 상태가
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
								{
									ObstacleCurrCnt = 0;
									mod_Detect.ObstacleDetectCnt++;
									mdc_isOpeningByObstacle = true;
								}
							}
						}														// 장애물 2회 감지 이후부터
						else if(mod_Detect.ObstacleDetectCnt >= 2)				// 장애물 3회 감지 시도
						{
							if(m_adc.MotorCurrent>500)			// 모터 전류가 150 이상인 상태가
							{
								ObstacleCurrCnt++;
								if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
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
					if(mod_Detect.ObstacleDetectCnt < 1)					// 장애물 처음 감지 시도
					{
						if(m_adc.MotorCurrent>ConstObsDetectCurr1)			// 모터 전류가 300 이상인 상태가
						{
							ObstacleCurrCnt++;
							if(ObstacleCurrCnt>ConstObstacleCurr1Cnt-1)		// 50ms동안 유지되면
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
					}														// 장애물 1회 감지 이후
					else if(mod_Detect.ObstacleDetectCnt == 1)				// 장애물 2회 감지 시도
					{
						if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// 모터 전류가 150 이상인 상태가
						{
							ObstacleCurrCnt++;
							if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
							{
								ObstacleCurrCnt = 0;
								mod_Detect.ObstacleDetectCnt++;
								mdc_isOpeningByObstacle = true;
							}
						}
					}														// 장애물 2회 감지 이후부터
					else if(mod_Detect.ObstacleDetectCnt >= 2)				// 장애물 3회 감지 시도
					{
						if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// 모터 전류가 150 이상인 상태가
						{
							ObstacleCurrCnt++;
							if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
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
					// 닫힘완료 수행
					ObstacleCurrCnt = 0;
					mmf_EndDecision.ClosedByPosition = true;
				}	
				else if(((DCSSwitchOn)||(DLSSwitchOn)) && (m_adc.MotorCurrent>400))			// 모터 전류가 150 이상인 상태가
				{
					ObstacleCurrCnt++;
					if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
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
//			(!mip_Input.DoorClosedSwitchOn) && //jeon_190709 추가
			(mmf_Encoder.Position<9000)) //&& jeon_190709 orig;2000
		{
			/*
			 * Slow Mode로 이동할 때 장애물 감지 기준
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
			 * Normal Mode로 이동할 때 장애물 감지 기준
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
			 * 속도가 일정속도 이하이면서
			 * (엔코더가 변하지 않을 때까지 기다리면 감속하다가 1회때 장애물 감지력이 너무 쎄져서
			 *  장애물이 걸려 일정 속도이하로 감소되었을 때 전류를 보고 장애물 감지 판단) 
			 */
			Velocity = prevPosition - mmf_Encoder.Position;
			if(Velocity<ConstObsVelocity) 
//			if((Velocity < ConstObsVelocity) && (mmf_Encoder.Position < 3500)) //jeon_190514
			{
				if(mod_Detect.ObstacleDetectCnt < 1)					// 장애물 처음 감지 시도
				{
					if(m_adc.MotorCurrent>ConstObsDetectCurr1)			// 모터 전류가 300 이상인 상태가
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr1Cnt-1)		// 50ms동안 유지되면
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
				}														// 장애물 1회 감지 이후
				else if(mod_Detect.ObstacleDetectCnt == 1)				// 장애물 2회 감지 시도
				{
					if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// 모터 전류가 150 이상인 상태가
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
						{
							ObstacleCurrCnt = 0;
							mod_Detect.ObstacleDetectCnt++;
							mdc_isOpeningByObstacle = true;
						}
					}
				}														// 장애물 2회 감지 이후부터
				else if(mod_Detect.ObstacleDetectCnt >= 2)				// 장애물 3회 감지 시도
				{
					if(m_adc.MotorCurrent>ConstObsDetectCurr23)			// 모터 전류가 150 이상인 상태가
					{
						ObstacleCurrCnt++;
						if(ObstacleCurrCnt>ConstObstacleCurr23Cnt)		// 200ms동안 유지되면
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
		 * 엔코더 10mm보다 작은 거리에서 전류를 증가시켜 닫힘완료 수행
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
							// 닫힘완료 수행
							if(m_adc.MotorCurrent>150u)
							{
								mmf_EndDecision.ClosedByCurrent = true;
								//debug("# 닫힘완료 : 전류로 닫힘 완료 플래그 셋 \r\n");
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
									// 닫힘완료 수행
									if(m_adc.MotorCurrent>150u)
									{
										mmf_EndDecision.ClosedByCurrent = true;
										//debug("# 닫힘완료 : 전류로 닫힘 완료 플래그 셋 \r\n");
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
							// 닫힘완료 수행
							mmf_EndDecision.ClosedByPosition = true;
						}
						else if((mmf_Encoder.Position >= 0) && (mmf_Encoder.Position <=DCU_OBS_POSITION))
						{
							// 닫힘완료 수행
							if(m_adc.MotorCurrent>150u)
							{
								mmf_EndDecision.ClosedByCurrent = true;
								//debug("# 닫힘완료 : 전류로 닫힘 완료 플래그 셋 \r\n");
							}
						}
					}
				}
				else //초기화 수행시
				{
					if(mip_Input.DoorClosedSwitchOn)
					{
						// 닫힘완료 수행
						mmf_EndDecision.ClosedByPosition = true;
					}
					else if((mmf_Encoder.Position >= 0) && (mmf_Encoder.Position <=DCU_OBS_POSITION))
					{
						// 닫힘완료 수행
						if(m_adc.MotorCurrent>150u)
						{
							mmf_EndDecision.ClosedByCurrent = true;
							//debug("# 닫힘완료 : 전류로 닫힘 완료 플래그 셋 \r\n");
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
						//debug("# 닫힘완료 : 전류로 닫힘 완료 플래그 셋 \r\n");
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
//        	if((mmf_Bemf.Position>(float)50) && (mmf_Bemf.Position<(float)2000))// B_emfPos 50 ~ 2000구간에서
		if(!mip_Input.DoorClosedSwitchOn)									// DLS,DCS가 눌리지 않는 구간에서
		{
			if(m_adc.MotorCurrent>500u)										// 모터전류 500만큼 상승이
			{
				ObstacleCurrCnt++;
				if(ObstacleCurrCnt>3u)											// 300ms동안 유지되면
				{
					ObstacleCurrCnt = 0;
					mod_Detect.ObstacleDetectCnt++;
				}
			}
			else
			{
				// No Action -> 닫힌상태 유지
			}
		}
		else
		{
			/*
			 * bemf 동작때 닫힘 완료 판단을 설정값보다 3초 후에 전류로 닫힘완료 판단을 수행
			 */
			if((m_adc.MotorCurrent>300u) && (get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > (mdm_time.CloseConfigtime+3000))) 									
			{
				ObstacleCurrCnt++;
				if(ObstacleCurrCnt>3u)											// 300ms동안 유지되면
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
