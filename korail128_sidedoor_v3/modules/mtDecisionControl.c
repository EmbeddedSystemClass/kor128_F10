/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtDecisionControl
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtDecisionControl.c
*********************************************************************/

#include "mtDecisionControl.h"
#include "mMotorOut.h"
#include "mtInputProcessing.h"
#include "mtMotorFeedback.h"
#include "mtObstacleDetect.h"
#include "mLibrary.h"
#include "mtMonitoring.h"


#if 0
#define PWM_SLOW			300
#define PWM_CLOSING			500
#define PWM_OPENING			500
#define PWM_BRAKE			100
#else
#define PWM_SLOW			50
#define PWM_CLOSING			100
#define PWM_OPENING			100
#define PWM_BRAKE			200
#endif

_DoorState mdc_DoorState = 0;
_DoorState mdc_PreDoorState = 0;
_DoorState mdc_InitDoorState = 0;
_Bool mdc_FlagSlowMode=false;
_Bool mdc_isInitComplete=false;									// 초기화 완료 시 true
_Bool mdc_isOpeningByObstacle=false;							// 장애감지 시 true, 장애감지 3회 시 false로 설정되어 엔코더로 완전열림 판단수행
_Bool mdc_FlagTestDODBPS=false;
_Bool mdc_isFirstClosed=false;									// Decision Task에서 Closing->Closed가 되는 순간이 잠깐이기 때문에 따로 플래그를 사용해서 Monitoring Task와 동기화 시킴
_Bool mdc_isFirstOpened=false;							// Decision Task에서 Opening->Opened가 되는 순간이 잠깐이기 때문에 따로 플래그를 사용해서 Monitoring Task와 동기화 시킴 
_Bool FlagDODBPS=false;

static void InitClosing(void);
void StateSetToOpenedAtOpening(_DoorDistance Mode);
void StateSetToOpeningAtClosed(_DoorDistance Mode);
void StateSetToClosedAtClosing(_DoorDistance Mode);
void StateSetToClosingAtOpened(_DoorDistance Mode);
static void DecisionOnClosed(void);
static void DecisionOnOpening(void);
static void DecisionOnOpened(void);
static void DecisionOnClosing(void);
static void DecisionOnError(void);
static void DecisionOnObstacle(void);
void FaultAction_F07(void);
void FaultAction_F10(void);
void FaultAction_F04_F08(uint8_t preDoorState);
void FaultAction_F05_F06(uint8_t preDoorState);

void ConfigDcuParameters(void);

uint32_t dodbps_power=0;
int32_t motor_voltage=0;
volatile uint32_t F10_OpenCount = 0;

void mdc_TaskDecisionControl(void const * argument)
{
	uint32_t PreviousWakeTime = osKernelSysTick();
	volatile static uint32_t loopcnt=0, dispcnt=0, dispErrcnt=0;
	_Bool prevOpenCloseButton=false;
	volatile static uint32_t zvr_CloseTime = 0;


    /*
     * 초기화 동작
     */
	#if 1
    {
    	/*
		 * 초기화 수행 전 Master/Slave DCU 판단
		 */
		if(m_isMasterSlave == MASTER_DCU)
		{
			debug("4. [Master Start]  Decision Control Task\r\n");
			osDelay(500);																		// Input Task가 먼저 수행되어 입력값을 알아야 로직 flow가 결정되므로 지연 후 시작
			mmo_DoorFree();																		// 처음 초기화 전 모터 프리
			mmo_ConnectHbridgeGND(true);														// 이거 연결(true로 셋) 안하면 모터 구동 안됨
			#ifndef DEBUG_SIMULATOR
				mdc_isFirstClosed = false;
				mdc_isInitComplete = false;
				InitClosing();																	// Power On 시 무조껀 Master DCU에 의해 초기화
			#endif
		}
		else // SLAVE_DCU
		{
			/*
			 * Slave는 절체될 때까지 무한 대기
			 */
			while(!m_isSlaveRunCommand)
			{
				osDelay(20);
				mdc_DoorState = mdc_InitDoorState;
				loopcnt++;
				if(loopcnt>250)																	// 5s
				{
					loopcnt=0;
					debug("slave cpu wait to run...\r\n");
				}
				mdc_DoorState = DOOR_INIT;
			}
			PreviousWakeTime = osKernelSysTick();
			loopcnt=0;
			/*
			 * 제어권이 넘어오면 mram 제어권도 가지기 때문에 mram_write_enable 함수를 호출시켜
			 * mram 에 데이터를 쓰고 읽을 수 있게 한다.
			 */

			mip_SleveCodeSlaveRun(1);
			m_isTaskExecution = true;
			//osDelay(500);
			mram_write_enable();																// 제어권이 넘어오면 mram write 허가
			ConfigDcuParameters();
			/*
			 * 190311 :DecisionControl과 Monitoring 태스크의 주기?가 맞지 않아 장애물 감지 기준값 설정이 안된상태에서
			 * 모니터링 태스크가 실행되면 절체 후 장애물 감지 고장이 뜨는 현상 수정 19.03.11
			 */
			mip_EmergencyLamp(0);
			mip_MasterCodeMasterLED(1);
			FaultFlag_SET(DCU_SLAVE_RUN);
																		//190311 : decisioncontrol 이외의 모든 태스크 수행
			mmo_DoorFree();																		// 처음 초기화 전 모터 프리
			mmo_ConnectHbridgeGND(true);														// 이거 연결(true로 셋) 안하면 모터 구동 안됨
			osDelay(200);
			/*
			 * 190312 열린상태에서 절체 될경우 도어의 위치는 알 수 있기 때문에 관련 값들을 초기화 한다
			 */
			if(mdc_DoorState == DOOR_INIT)
			{
				InitClosing();
			}
			else if(mdc_DoorState == DOOR_OPENED)
			{
				mmf_Encoder.Position = m_OpeningEncoderPulse; //위치를 알수 있으므로 값 초기화
				mdc_isInitComplete = true;
				mdc_PreDoorState = DOOR_OPENING;
				mdc_DoorState = DOOR_OPENED;
			}
			else if(mdc_DoorState == DOOR_CLOSED)
			{
				mmf_Encoder.Position = 0;
				mdc_isInitComplete = true;
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSED;
			}
			else if((mdc_DoorState == DOOR_OPENING) || (mdc_DoorState == DOOR_CLOSING))
			{
				mdc_isInitComplete = false;
//				if(mip_Input.di0_OpenCloseButton == true)
//				{
//					mmo_DoorOpening(DCU_SLOWMODE_POWER);
//					mdc_FlagSlowMode = true;
//					mdc_PreDoorState = DOOR_OPENING;
//					mdc_DoorState = DOOR_OPENING;
//				}
//				else
//				{
//					mmo_DoorClosing(DCU_SLOWMODE_POWER);
//					mdc_FlagSlowMode = true;
//					mdc_PreDoorState = DOOR_CLOSING;
//					mdc_DoorState = DOOR_CLOSING;
//				}
				InitClosing();
			}
			else if(mdc_DoorState == DOOR_ERROR)
			{
				InitClosing(); //고장 발생등으로 절체할 경우
			}
			/*
			 * 190311 :DecisionControl과 Monitoring 태스크의 주기?가 맞지 않아 장애물 감지 기준값 설정이 안된상태에서
			 * 모니터링 태스크가 실행되면 절체 후 장애물 감지 고장이 뜨는 현상 수정 19.03.11
			 */

			debug("4. [Slave Start]  Decision Control Task\r\n");													// 이거 연결(true로 셋) 안하면 모터 구동 안됨
		}
    }
	#else
    {
    	/*
    	 * Master DCU는 초기화 전 닫힘완료 전까지 열림명령에 따라 도어를 열고 닫아야 함.
    	 */
		if(m_isMasterSlave == MASTER_DCU)
		{
			debug("4. [Master Start]  Decision Control Task\r\n");
			osDelay(500);																		// Input Task가 먼저 수행되어 입력값을 알아야 로직 flow가 결정되므로 지연 후 시작
			mmo_DoorFree();																		// 처음 초기화 전 모터 프리
			mmo_ConnectHbridgeGND(true);														// 이거 연결(true로 셋) 안하면 모터 구동 안됨
			
			while(!mdc_DoorState)																// 초기 DoorState = DOOR_INIT
			{
				if(mip_Input.di0_OpenCloseButton &&												// 열림버튼이 눌리면
				   (prevOpenCloseButton != mip_Input.di0_OpenCloseButton))
				{
					mmo_DoorFree();
					mmo_DoorOpening(DCU_SLOWMODE_POWER);														// Slow Mode로 Opening
				}
				else if(!mip_Input.di0_OpenCloseButton &&										// 열림버튼이 풀리면
						(prevOpenCloseButton != mip_Input.di0_OpenCloseButton))
				{
					mmo_DoorFree();
					mmo_DoorClosing(DCU_SLOWMODE_POWER);														// Slow Mode로 Closing
				}
				else
				{
					if(mip_Input.DoorClosedSwitchOn)											// DLS가 눌리면
					{
						InitClosing();															// Closed 초기화 완료
					}
					else if(mmf_EndDecision.OpenedByCurrent)									// Opened
					{
						mmo_DoorFree();
					}
				}
				prevOpenCloseButton = mip_Input.di0_OpenCloseButton;
			}
		}
		else // SLAVE_DCU
		{
			
		}
    }
	#endif

	/*
	 * 초기화 완료 후 동작
	 */
    for(;;)
    {
    	/*
    	 * Place this task in the blocked state until it is time to run again.
    	 */
        osDelayUntil(&PreviousWakeTime, 20UL); //jeon_190717 task 주기 20ms => 25ms
        
 /*       
        if(mdc_PreDoorState != mdc_DoorState)
        {
			printf("Door Pre_State : %d, Door State : %d \r\n", mdc_PreDoorState, mdc_DoorState);
			printf("OBS : %d, Position : %d \r\n", mod_Detect.ObstacleDetectCnt, mmf_Encoder.Position);
		}
*/
    	/*
    	 * Data Input (세마포어로 공유해서 전역변수 바로 사용하는 방법 고려해 볼 것)
    	 */
    	#if 0
    	{
    		/*
    		 * InputProcessing 모듈로부터 입력 스위치 정보 가져오기
    		 */
    		// mip_Input;
    		/*
    		 * MotorFeedback 모듈로부터 열림/닫힘 완료 정보 가져오기
    		 */
    		// mmf_EndDecision;
    		/*
    		 * ObstacleDetect 모듈로부터 장애물 감지 정보 가져오기
    		 */
    		// mod_Detect;
    		/*
    		 * DataManagement 모듈로부터 에러정보 가져오기
    		 */
    		// mdm_ErrorFlag;
    	}
    	#endif

    	/*
    	 * Master에서 Slave로부터 패킷 수신으로 Slave DCU Run인 상태를 확인하였으면 Master 정지
    	 */
    	if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
    	{
    		loopcnt++;
    		if(loopcnt>20)
    		{
    			mmo_DoorFree();
    			mmo_ConnectHbridgeGND(false);			//절체된 경우에는 모터를 꺼버린다
        		mdc_DoorState = DOOR_ISOLATION;
        		mdc_PreDoorState = DOOR_ISOLATION;
        		mmf_PIDControl = PID_CONTROL_NONE;
    			loopcnt=0;
    			debug("# Master Logic Task 실행 안함\r\n");
    		}
    	}
    	/*
    	 * 차단 시 Change State to DOOR_ISOLATION
    	 */
    	else if(mip_Input.di0_Isolation == true)										// 돌리면 High 계속 유지, 복귀하면 Low 계속 유지
    	{
			//debug("# 차단스위치 돌려 -> 도어프리\r\n");
    		mdc_DoorState = DOOR_ISOLATION;
    		mdc_PreDoorState = DOOR_ISOLATION;
			mmf_PIDControl = PID_CONTROL_NONE;											// PID제어를 먼저 중지 시키고
			mmo_DoorFree();																// 차단 시 모터를 Free로 놔야 함
			mram_Event_save(DCU_ISO_EVENT);
			zvr_CloseTime = osKernelSysTick();
    	}
		/*
		 * 열차 이동 중 DLS가 풀리면 닫힐 때까지 닫힘동작 수행
		 */
    	else if(!mip_Input.di0_ZVR)														// 이동중
    	{
    		mmf_PIDControl = PID_CONTROL_NONE;
 
    		if(mdc_PreDoorState == DOOR_ISOLATION)
    		{
    			if(mip_Input.DoorClosedSwitchOn == true)
    			{
    				mdc_PreDoorState = DOOR_CLOSING;
    				mdc_DoorState = DOOR_CLOSED;
    				mmo_DoorBrake();
    			}
    			else
    			{
    				mdc_DoorState = DOOR_CLOSING;
    				
    				mmo_DoorClosing(DCU_SLOWMODE_POWER);
    			}
    		}
    		else
    		{
    			if(mip_Input.DoorClosedSwitchOn) //(mip_Input.di0_DLS1 || mip_Input.di1_DLS2)
    			{
           			/* 기관실에 도어상태를 closed로 바꾸어서 정보를 전달해야함*/
    					mmo_DoorBrake();
    					
 	    				mdc_PreDoorState = DOOR_CLOSING;
        				mdc_DoorState = DOOR_CLOSED;
        				
        				if((mip_Input.di0_DLS1) && (error_list_flag[DCU_DLS1_FAULT] == true)) Erase_fault(DCU_DLS1_FAULT);
        				if((mip_Input.di1_DLS2) && (error_list_flag[DCU_DLS2_FAULT] == true)) Erase_fault(DCU_DLS2_FAULT);
        				if((mip_Input.di1_DCS1) && (error_list_flag[DCU_DCS1_FAULT] == true)) Erase_fault(DCU_DCS1_FAULT);
        				if((mip_Input.di1_DCS2) && (error_list_flag[DCU_DCS2_FAULT] == true)) Erase_fault(DCU_DCS2_FAULT);
    			}
    			else if(mip_Input.di1_DCS1 || mip_Input.di1_DCS2)
    			{
    				if(get_diff_tick(osKernelSysTick(),zvr_CloseTime) > 300000) //orig:600000, 300000
    				{
        				mdc_PreDoorState = DOOR_CLOSING;
        				mdc_DoorState = DOOR_CLOSED;
        				mmo_DoorBrake();
     				}
    				else
    				{
             			if(mdc_DoorState == DOOR_OPENING)
            			{
            				mmo_DoorFree(); 
            				HAL_Delay(10); //동작 변경 시 10ms 딜레이 진행
            			}
             			
            				mdc_DoorState = DOOR_CLOSING;
            				mmo_DoorClosing(DCU_SLOWMODE_POWER);
    				}
    			}
    			else
    			{
    				zvr_CloseTime = osKernelSysTick();
        			if(mdc_DoorState == DOOR_OPENING)
        			{
        				mmo_DoorFree(); 
        				HAL_Delay(10); //동작 변경 시 10ms 딜레이 진행
        			}
        			
						mdc_DoorState = DOOR_CLOSING;
						mmo_DoorClosing(DCU_SLOWMODE_POWER);
    			}
     		}
    	}
    	else if(mip_Input.di1_EAD == true)
    	{
    		/*
    		 * 외부 비상핸들은 복귀했을때 free상태를 유지한다
    		 */
    		mdc_DoorState = DOOR_ISOLATION;
    		mdc_PreDoorState = DOOR_EAD;
    		mmf_PIDControl = PID_CONTROL_NONE;
    		mmo_DoorFree();
    		mram_Event_save(DCU_EAD_EVENT);
    		zvr_CloseTime = osKernelSysTick();
    	}
    	else if(mip_Input.di1_EED == true)
    	{
    		mdc_DoorState = DOOR_ISOLATION;
    		mdc_PreDoorState = DOOR_ISOLATION;
    		mmf_PIDControl = PID_CONTROL_NONE;
    		mmo_DoorFree();
    		mram_Event_save(DCU_EED_EVENT);
    		zvr_CloseTime = osKernelSysTick();
    	}
#if 0  	//jeon_190709 orig:0
    	else if(((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true)) && ((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true)))
		{
        		mmo_DoorBrake();
				mdc_DoorState = DOOR_ISOLATION;
				mdc_PreDoorState = DOOR_ISOLATION;
				mmf_PIDControl = PID_CONTROL_NONE;											// PID제어를 먼저 중지 시키고
				mmo_DoorFree();																// 차단 시 모터를 Free로 놔야 함
		}
#endif
    	/*
    	 * Change State to OPENED/OPENING/CLOSING/CLOSED
    	 */
    	else
    	{
    		zvr_CloseTime = osKernelSysTick();
    		/*
    		 * 외부 비상핸들을 복귀 시키면 free 상태로 동작을 멈추고 대기하나
    		 * 차단스위치 or 내부 비상핸들이 입력되고 복귀하면 초기화 동작을 수행한다.
    		 */
    		if(mdc_PreDoorState == DOOR_EAD)
    		{
    			if(mip_Input.di0_Isolation == true)
    			{
    	    		mdc_DoorState = DOOR_ISOLATION;
    	    		mdc_PreDoorState = DOOR_ISOLATION;
    	    		debug("# 이전 입력이 외부 비상핸들, 차단스위치 입력으로 정상상태 복귀 \r\n");
    			}
    			else if(mip_Input.di1_EED == true)
    			{
    	    		mdc_DoorState = DOOR_ISOLATION;
    	    		mdc_PreDoorState = DOOR_ISOLATION;
    	    		debug("# 이전 입력이 외부 비상핸들, 내부비상핸들 입력으로 정상상태 복귀 \r\n");
    			}
    			else if(mip_Input.di1_EAD == false)
    			{
    			    		/*
    			    		 * 외부 비상핸들은 복귀했을때 free상태를 유지한다
    			    		 */
    				if(mip_Input.DoorClosedSwitchOn)
    				{
    						mdc_DoorState = DOOR_ISOLATION;
    						mdc_PreDoorState = DOOR_ISOLATION;
    				}		
    			}
    			    	
    		}
    		/*
    		 * 차단에서 복귀(엔코더 위치를 알 수 없으므로) 시 SlowMode로 Opening/Closing 수행
    		 */
    	    else if(mdc_PreDoorState == DOOR_ISOLATION)
    		{
    			/*
     			 * 열림버튼이 눌려져 있으면 Opening 수행해서 전류로 멈춰야 함
    			 */

				if(mip_Input.di0_OpenCloseButton)
    			{
    				debug("# 차단스위치 복귀 -> 열림\r\n");

    				if(mip_Input.DoorClosedSwitchOn == true)
    				{
    					InitClosing(); //jeon_190924
    				}
    				else
    				{
						mdc_isInitComplete = false;
						mdc_FlagSlowMode = true;
						mmo_DoorOpening(DCU_SLOWMODE_POWER);
						mdc_FlagSlowMode = true;
						mdc_DoorState = DOOR_OPENING;
						mdc_PreDoorState = DOOR_OPENING;
    				}	
    			}
    			/*
    			 * 열림버튼이 떼져 있으면 Closing 수행해서 전류로 멈춰야 함
    			 */
    			else
    			{
    				debug("# 차단스위치 복귀 -> 닫힘(초기화)\r\n");
    				InitClosing();											// Closing 상태에서 닫힘스위치가 눌린 것을 보고 닫힘완료로 판단 수행
     			}
     		}
    		/*
    		 * Normal Closed/Opening/Opened/Closing Operation
    		 */
    		else
    		{
    			/*
    			 * DLS/DCS, 엔코더, 전류를 통한 닫힘완료 및 열림완료 판단
    			 * 
    			 * 1. 열림완료(DLS,DCS를 보지 않음)
    			 * 		1차 - 엔코더를 통한 열림완료
    			 * 		2차 - 엔코더를 통한 열림완료 실패 시 전류를 통한 열림완료
    			 * 		
    			 * 2. 닫힘완료(DLS를 통해 장애물감지를 할 것인지 닫힘완료를 할 것인지 결정)
    			 * 		* DLS1 or DLS2가 둘 중 하나 눌린 경우
    			 * 		1차 - 엔코더를 통한 닫힘완료
    			 * 		2차 - 엔코더를 통한 닫힘완료 실패 시 전류를 통한 닫힘완료
    			 * 		
    			 * 		* DLS1 and DLS2 둘 다 눌리지 않은 경우
    			 * 		1차 - 엔코더 10mm보다 큰 거리에서 전류 증가 시 장애물 감지
    			 * 		2차 - 엔코더 10mm보다 작은 거리에서 전류를 증가시켜 닫힘완료 수행
    			 */
    			switch(mdc_DoorState)
    			{
    				case DOOR_CLOSED:
            			/*
            			 * Closed -> 버튼, 강제열림 입력 -> Opening or Closing
            			 */
        				DecisionOnClosed();
    					break;
    				case DOOR_OPENING:
            			/*
            			 * Opening -> 엔코더/전류를 통한 End 판단 -> Opened
            			 */
        				DecisionOnOpening();
    					break;
    				case DOOR_OPENED:
            			/*
            			 * Opened -> 시간경과 -> Closing
            			 */
        				DecisionOnOpened();
    					break;
    				case DOOR_CLOSING:
            			/*
            			 * Closing -> 버튼, 엔코더/전류/포토센서 센싱 -> Opening or Closed
            			 */
    					DecisionOnClosing();
    					break;
    				case DOOR_OBSTACLE:
    					DecisionOnObstacle();
    					break;
    				case DOOR_ERROR:
    					DecisionOnError();
    					break;
    				default:
    					break;
    			}
    		}
    	}
		
		//#ifdef DEBUG_SIMULATOR
		#if 0
			#if 0
				dispcnt++;
				if(dispcnt>2)
				{
					dispcnt=0;
					if((mdc_DoorState==DOOR_OPENING)||(mdc_DoorState==DOOR_CLOSING))
						debug("# Pos:%d/%d\r\n", (int)mmf_Encoder.Position, (int)m_OpeningEncoderPulse);
				}
			#else
				dispcnt++;
				if(dispcnt>50)
				{
					dispcnt=0;
					debug("# Door State: ");
					switch(mdc_DoorState)
					{
						case DOOR_INIT:		debug("Init"); break;
						case DOOR_OPENING:	debug("Opening"); break;
						case DOOR_OPENED:	debug("Opened"); break;
						case DOOR_CLOSING:	debug("Closing"); break;
						case DOOR_CLOSED:	debug("Closed"); break;
						case DOOR_OBSTACLE:	debug("Obstacle"); break;
						case DOOR_ISOLATION:debug("Isoltation"); break;
						case DOOR_ERROR:	debug("Error"); break;
						default: break;
					}
					debug("\r\n");
				}
			#endif
		#endif
    }
}

static void InitClosing(void)
{
	/*
	 * 최초 부팅 직후, 차단에서 정상으로 복귀 시 초기화 수행(Vref=200으로 저속제어, PID제어 미수행)
	 */
	if(mdc_isFirstClosed == false) mdc_isInitComplete = false;
	mmf_Encoder.MaxOpenPosition = m_OpeningEncoderPulse;						// 입력 task 우선순위가 높기 때문에 m_OpeningEncoderPulse 값이 먼저 생성 됨
	
	/*
	 * Slave (Master가 살짝 움직인 엔코더 값때문에 Slave는 리셋을 시켜줘야 함)
	 *  - Master에 의해 모터가 살짝 움직여서 0값에서 더 회전하여 9998 값이 되고
	 *    시뮬레이터로 닫힘완료 판단을(엔코더 값이 100보다 작은 경우) 못하고, 
	 *    엔코더 값이 9998에서 100보다 작을 때까지 회전한 다음 닫힘완료를 판단함
	 *  - Slave 실행 대기 중일 때 초기화 단계에서 엔코더 값을 리셋시켜줘야 함.
	 *
	 * Master
	 *  - Closing(100)을 해도 바로 닫힘완료 판단을 수행하나, PWM출력이 남아 있어서 모터가 살짝 움직임
	 *  - 어쨌꺼나 Master는 닫힘완료 판단을 하고, Closed 상태에서 주기적으로 엔코더 값을 리셋시킴
	 */
	if(m_isMasterSlave == SLAVE_DCU)
	{
		if(mip_Input.DoorClosedSwitchOn == false)
//		if((mip_Input.di0_DLS1 == false) && (mip_Input.di1_DLS2 == false)) //jeon_190709
		{
			mmf_PIDControl = PID_CONTROL_NONE;
			mmo_DoorClosing(DCU_SLOWMODE_POWER);														// Closing 함수가 계속 호출되면 Disable(N채널 둘 다 Braking 이 잠깐 On됨) -> PWM설정 -> Enable이 계속 반복
			mdc_FlagSlowMode = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState = DOOR_CLOSING;
			mdm_time.ClosingStart = osKernelSysTick();						//초기화 동작에도 닫힘 시간을 적용해서 dls dcs 고장을 판별해야함
			mdm_time.ClosingPrintf = 1;
		}
		else
		{
			mdc_isInitComplete = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState 	 = DOOR_CLOSED;
			printf("슬레이브 차단 스위치 닫힘 복귀 닫힘 완료 \r\n");
		}
	}
	/*
	 *  초기화 동작 시 dcs ,dls 눌러져 있으면 닫힘 완료
	 */
	else if(m_isMasterSlave == MASTER_DCU)
	{
		if(mip_Input.DoorClosedSwitchOn == false)
		{
			mmf_PIDControl = PID_CONTROL_NONE;
			mmo_DoorClosing(DCU_SLOWMODE_POWER);														// Closing 함수가 계속 호출되면 Disable(N채널 둘 다 Braking 이 잠깐 On됨) -> PWM설정 -> Enable이 계속 반복
			mdc_FlagSlowMode = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState = DOOR_CLOSING;
			mdm_time.ClosingStart = osKernelSysTick();						//초기화 동작에도 닫힘 시간을 적용해서 dls dcs 고장을 판별해야함
			mdm_time.ClosingPrintf = 1;
		}
		else
		{
			/* ex) 전원을 껐다 켰을 경우 dls 둘중 하나가 눌려 있지 않은경우 미는 동작을 수행 할 수 있도록 함 decision_onclosed에서 100ms 이후에 brake함*/
			if((mip_Input.di0_DLS1 == false) && (mip_Input.di1_DLS2 == true)) mmo_DoorClosing(DCU_SLOWMODE_POWER);
			else if((mip_Input.di0_DLS1 == true) && (mip_Input.di1_DLS2 == false)) mmo_DoorClosing(DCU_SLOWMODE_POWER);
			mdc_isInitComplete = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState 	 = DOOR_CLOSING; //jeon_190924 orig : DOOR_CLOSED;
			printf("차단 스위치 닫힘 복귀 닫힘 완료 \r\n");
		}
	}
}

void StateSetToOpeningAtClosed(_DoorDistance Mode)
{
	int32_t CurrEncPos=0;
	  /*
	#define OPEN_ACCEL_SECTiON			500				// 580 - 650mm opening
	#define OPEN_CONST_SECTiON			2000			// 2000 - 650mm opening
	#define OPEN_DEACE_SECTiON			3580			// 3580 - 650mm opening

	#define CLOSE_ACCEL_SECTiON			3000			// 3580 - 650mm closing
	#define CLOSE_CONST_SECTiON			1580			// 1580 - 650mm closing
	#define CLOSE_DEACE_SECTiON			0				// 0 - 650mm closing
	 
	    */ 

	if(Mode==DOOR_POSITION_CLOSED_BY_END)
	{
		debug("# 닫힌상태에서 열림버튼을 눌러 -> 열림\r\n");
		#if 0
			OpeningSection.Acceleration = (m_OpeningEncoderPulse*2)/10;		// Closed에서 Opening 하는 경우 가속구간설정
			OpeningSection.Constant = (m_OpeningEncoderPulse*4)/10;			// Closed에서 Opening 하는 경우 등속구간설정
			OpeningSection.Deceleration = m_OpeningEncoderPulse;			// Closed에서 Opening 하는 경우 감속구간설정
		#else
			OpeningSection.Acceleration = OPEN_ACCEL_SECTiON;		// 2s(500)
			OpeningSection.Constant = OPEN_CONST_SECTiON;			// 2s(1250)
			OpeningSection.Deceleration = OPEN_DEACE_SECTiON;		// 2s(3580)
/*JEON_190530
			OpeningSection.Acceleration = (m_OpeningEncoderPulse*3)/10;		// Closed에서 Opening 하는 경우 가속구간설정
			OpeningSection.Constant = (m_OpeningEncoderPulse*5)/10;			// Closed에서 Opening 하는 경우 등속구간설정
			OpeningSection.Deceleration = m_OpeningEncoderPulse;			// Closed에서 Opening 하는 경우 감속구간설정
*/
		#endif
		mdc_DoorState = DOOR_OPENING;
		mmf_PIDControl = PID_CONTROL_OPENING;
	}
	else if(Mode==DOOR_POSITION_CLOSED_BY_BUTTON)
	{
		/*
		 * 열림거리 재설정 - 끝까지 열림
		 */
		CurrEncPos = mmf_Encoder.MaxOpenPosition - mmf_Encoder.PreStopPosition;			// Closing 도중 버튼을 눌러 Opening 하는 경우
		OpeningSection.Acceleration = ((CurrEncPos*3)/10)+mmf_Encoder.PreStopPosition;	// Closing 도중 버튼을 눌러 Opening 하는 경우 가속구간설정
		OpeningSection.Constant = ((CurrEncPos*5)/10)+mmf_Encoder.PreStopPosition;		// Closing 도중 버튼을 눌러 Opening 하는 경우 등속구간설정
		OpeningSection.Deceleration = CurrEncPos+mmf_Encoder.PreStopPosition;			// Closing 도중 버튼을 눌러 Opening 하는 경우 감속구간설정
		mmf_Encoder.RefVelocity = 0;													// 0부터 가속/등속/감속이 되개 Closing 시 설정된 RefVelocity 값을 0으로 리셋
		
		/*
		 * Closed -> Opening by OpenClose Button Push or Obstacle
		 */
		mdc_PreDoorState = DOOR_CLOSING;
		mdc_DoorState = DOOR_OPENING;
		mmf_PIDControl = PID_CONTROL_OPENING;
	}
}

void StateSetToClosedAtClosing(_DoorDistance Mode)
{
	int32_t PrePosition=0, MatchCnt=0;
	
	mmf_PIDControl = PID_CONTROL_NONE;												// PID제어를 먼저 중지 시키고
	mmf_NextMotorDirSelect=PID_CONTROL_OPENING;
	setPIDPWMOut();
	mmo_DoorFree();																	// 닫힘 완료 시 모터를 Free로 놔야 함
	if(Mode==DOOR_POSITION_CLOSED_BY_END)												// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
	{
		osDelay(1);
		mmo_DoorBrake();
	}
	else if(Mode==DOOR_POSITION_CLOSED_BY_BUTTON)
	{
		/*
		 * 이 조건이 없으면 장애물 3회 감지 후 완전열릴 때 따닥하고 잠깐 멈췄다 열림
		 *  : 장애물 3회째 감지 시 바로 열려야 하는데 500ms 지연으로 잠깐 멈췄다 열림
		 *  : 장애물 감지가 없을 때만 Closing 도중 버튼으로 Openging 할 때 부드럽게 열기 위해 시간지영 추가
		 */
		if(mod_Detect.ObstacleDetectCnt==0)
		{
			/*
			 * Closing 하다 Free후 바로 Opening 하면 역기전력으로 다이오드 나가므로 서서히 줄여야 함.
			 * : Closing 하다가 Free로 놓을 때 바로 Free로 놓으면 
			 *   PWM2에 걸리는 전류가 역으로 PWM1 FET로 흘러  PWM1에 있는 다이오드 나감.
			 *   따라서 Free로 놓을 때 PWM을 갑자기 0으로 놓으면 안되고, 서서히 줄이다가 0이되면 Free로 놔야 함
			 */
			while(MatchCnt<100)
			{
				if(PrePosition==mmf_Encoder.Position)	MatchCnt++;
				else									MatchCnt=0;
				
				PrePosition = mmf_Encoder.Position;
				osDelay(1);
			}
		}
	}
	else if(Mode==DOOR_POSITION_CLOSED_BY_OBSTACLE)
	{
		/*
		 * 장애물 감지 시 지연주면 장애물 감지로 열릴때 한번에 안열리고 이것 때문에 따닥 열림
		 */
		//osDelay(300);																// 장애물 감지 시에는 지연 없이 바로 Opening 수행
	}
	
	mmf_Encoder.PreStopPosition = mmf_Encoder.Position;
	mmf_Encoder.tmpClosedPosition = mmf_Encoder.Position;
	mmf_Bemf.RefPosition = mmf_Bemf.Position;
	
	if(mdc_DoorState==DOOR_CLOSED)
	{
		mmf_Encoder.MaxClosePosition = mmf_Encoder.Position;
		mmo_MotorPositionReset();
		mmf_Encoder.Position = 0;													// Free 후 모터가 더 움직일 수 있으므로 완전히 멈춘다음 엔코더 리
		mmf_Bemf.Position=(float)0;
		mod_Detect.ObstacleDetectCnt = 0;											// 닫힘 완료하면 장애물감지 횟수 리셋
		mmf_EndDecision.ClosedByPosition = false;
		mmf_EndDecision.ClosedByCurrent = false;
	}
}

/*
 * SRS-DCU-FR-001
 */
static void DecisionOnClosed(void)
{
	volatile static uint32_t loopcnt=0;
	
	mmf_EndDecision.OpenedByCurrent = false;
	mmf_EndDecision.OpenedByPosition = false;
	/*
	 * Initialization at Closed
	 */
	if((mdc_PreDoorState==DOOR_CLOSING)&&(mdc_DoorState==DOOR_CLOSED))
	{
		osDelay(100);											//닫힘완료 판단이 되면 100ms 밀고 나서 멈추는 동작 (닫힌 상태에서 전원을 껐다 키면 DLS 고장을 띄우는 경우가 있음)
		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_END);
		mdc_PreDoorState=DOOR_CLOSED;
		mdc_isFirstClosed = true;
		mdc_isInitComplete = true;
		FlagDODBPS=false; //jeon_190514
		/*
		 * 닫힘(Closing)시간 계산
		 */
		mdm_time.Closed = osKernelSysTick();								
		mdm_time.Closing = mdm_time.Closed - mdm_time.ClosingStart;
		debug("## 닫힘완료: 닫힘시간(%d), 닫힘위치(%d)\r\n",
				(int)mdm_time.Closing, (int)mmf_Encoder.tmpClosedPosition);
	}
	/*
	 * Closed -> Opening by OpenClose Button Push
	 */
	/*
	 * Closed -> Closed after OpenClose Button Release										// 닫힘완료가 되면 초기화 완료로 SlowMode 고려할 필요 없음
	 */
	else
	{
		mip_Input.CommandClose = false;
		/*
		 * Closed -> Opening by OpenClose Button Push
		 */
		if(mip_Input.CommandOpen == true)
		{
			if(mip_Input.CommandTestMode == true)
			{
				loopcnt=0;
				mmo_MotorPositionReset();
				mmf_Encoder.Position = 0;
				mdc_DoorState = DOOR_OPENING;
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorOpening(DCU_SLOWMODE_POWER);
				mdc_FlagSlowMode = true;
				/*
				 * 열림(Opening) 시작시간 저장
				 */
				mdm_time.OpeningStart = osKernelSysTick();										// Closed 상태에서 열림버튼을 눌러 Opening 시작
				mdm_time.OpeningPrintf = 1;
			}
			else
			{
				StateSetToOpeningAtClosed(DOOR_POSITION_CLOSED_BY_END);
				/*
				 * 열림(Opening) 시작시간 저장
				 */
				mdm_time.OpeningStart = osKernelSysTick();										// Closed 상태에서 열림버튼을 눌러 Opening 시작
				mdm_time.OpeningPrintf = 1;
			}
		}
		else
		{
			if(mip_Input.di0_OpenCloseButton)
			{
				StateSetToOpeningAtClosed(DOOR_POSITION_CLOSED_BY_END);

				/*
				 * 열림(Opening) 시작시간 저장
				 */
				mdm_time.OpeningStart = osKernelSysTick();										// Closed 상태에서 열림버튼을 눌러 Opening 시작
				mdm_time.OpeningPrintf = 1;
				if(debugprint.Data_Print_flag == true)
				{
					printf("Closed -> Opening Start \r\n");
					printf("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
					printf("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
				}
			}
			else
			{
				/*
				 * Closed(brake)에서 도어가 움직이면 엔코더 값이 틀어지므로 주기적으로 엔코더 값 리셋
				 */
				mdc_isFirstClosed = true;
				mdc_isInitComplete = true;
				mod_Detect.ObstacleDetectCnt = 0;
				mdc_FlagSlowMode = false;
				loopcnt++;
				if(loopcnt>2)
				{ 
					mmo_MotorPositionReset();
					mmf_Bemf.Position=(float)0;
					mmf_Encoder.Position = 0;													// Free 후 모터가 더 움직일 수 있으므로 완전히 멈춘다음 엔코더 리
				}
				else if(loopcnt>50)															//1초후
				{
					loopcnt=0;
					Erase_fault(DCU_UNEXPECTED_UNLOCK);
				}
			}
		}
	}
}

static void DecisionOnOpening(void)
{
	int32_t CurrEncPos=0;
	
	/*
	 * Opening -> Closing by OpenClose button Release
	 */
	#if 1
	{
		/*
		 * 열림중 열림버튼을 풀어 닫힘 (장애물 미감지 시에만 동작)
		 * 장애물 감지 조건이 없으면 열림버튼을 푼 상태에서 장애물 감지로 Opening 상태가 되면바로 닫힘동작을 수행함 
		 */
		if(mip_Input.CommandTestMode || mip_Input.CommandOpen)
		{
			//No Action
		}
		else if(!mip_Input.di0_OpenCloseButton && (mod_Detect.ObstacleDetectCnt==0) && (!mip_Input.di0_ReOpen))								// jjkim 190104 : reopen 버튼을 누르고있으면 계속 opening 상태를 유지 해야함
		{
			debug("# 열림중:열림버튼을 풀어 -> 열림 중 닫힘\r\n");
			mdm_time.ClosingStart = osKernelSysTick();
			if(mdc_isInitComplete == true)
			{
				/*
				 * Opening -> Opened
				 */
				StateSetToOpenedAtOpening(DOOR_POSITION_OPENED_BY_BUTTON);	// 열림 중 열림버튼을 풀어 중간지점에서 Opened
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_BUTTON);
				return;														// 열림 중 닫힘 시 열림완료(Opening->Opened) 판단 수행할 필요 없음
			}
			else
			{
				mdc_DoorState = DOOR_CLOSING;
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorClosing(DCU_SLOWMODE_POWER);
			}
		}
	}
	#endif
	
    /*
     * Opening -> Opened or Opening
     */
	#if 1
	{
		/*
		 * 자동 닫힘완료 스위치 Off
		 */
		#ifdef DEBUG_SIMULATOR
			/*
			 * 닫힘스위치가 눌린 상태에서 엔코더가 0->50 만큼 이동하면 시뮬레이터로 닫힘스위치를 풀어줌
			 */
			if((mip_Input.DoorClosedSwitchOn) && (mmf_Encoder.Position>100))
			{
				mip_Input.di0_DLS1 = false;
				mip_Input.di1_DLS2 = false;
				mip_Input.di1_DCS1 = false;
				mip_Input.di1_DCS2 = false;
				mip_Input.DoorClosedSwitchOn = false;
				debug("# 시뮬레이션모드에서 강제로 닫힘스위치 Release\r\n");
			}
		#endif
		
		/*
		 * Opening -> Opened by Encoder, Current, 장애감지
		 */
		#if 1
		{
			/*
			 * 엔코더로 멈추는 경우 -> 제일 먼저 엔코더로 멈춤을 시도해야 함
			 */
			if(mmf_EndDecision.OpenedByPosition)
			{
				if(mdc_isInitComplete == true)
				{
					if(mod_Detect.ObstacleDetectCnt > mod_Detect.ObstacleConfigValue)
					{
						debug("# 열림중:장애감지 -> 열림완료 (%d)\r\n", mod_Detect.ObstacleDetectCnt);
					}
					else
					{
						debug("# 열림중:엔코더로 -> 열림완료 판단\r\n");
					}
					mdc_PreDoorState = DOOR_OPENING;
					mdc_DoorState = DOOR_OPENED;
					
					if(debugprint.Data_Print_flag == true)
					{
						mdm_time.OpeningPrintf = false;
						printf("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
						printf("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
					}
				}
				else
				{
					debug("# 1\r\n");
					// 초기화 전에는 엔코더값을 모르므로 엔코더로 열림완료 판단 불가 -> No Action
				}
			}
			/*
			 * 전류로 멈추는 경우 -> 초기화와 상관없이 전류로 닫힘완료가 판단되면 무조껀 항상 정지시켜야 함
			 */
			else if(mmf_EndDecision.OpenedByCurrent)
			{
				mmf_EndDecision.OpenedByCurrent = false;
				debug("# 열림중:전류로 -> 열림완료 판단\r\n");
				/*
				 * [중요] B_emf모드는 초기화 전/후 동일하게 정지상태만 만족하면 바로 멈춰야 함
				 * 
				 * 모터 정지 상태에서도 B_emf전압은 계속 측정 됨
				 * -> B_emf를 통한 속도감소 발생, 위치되 계속 감소함
				 * -> 속도0이 일정시간 유지되는 조건으로 Closed 상태를 만들 수 없음
				 * -> 계속 Closing 방향으로 FET를 Turn On 시키고 있음
				 * 
				 * -> 닫힘완료가 안된 상태에서 Open명령으로 Opening 동작을 수행하면
				 * -> FET 쇼트가 발생해 MCU Reset되는 현상이 발생되는 것으로 추정됨
				 */
				mdc_PreDoorState = DOOR_OPENING;
				mdc_DoorState = DOOR_OPENED;
				
				if(debugprint.Data_Print_flag == true)
				{
					mdm_time.OpeningPrintf = false;
					printf("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
					printf("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
				}
			}
			/*
			 * Opening -> Opening
			 */
			else
			{
				/*Opening 동작 계속 수행*/

				if((debugprint.Data_Print_flag == true) && (mmf_Encoder.Position >=1500))
				{
					if(mdm_time.OpeningPrintf == true)
					{
						mdm_time.OpeningPrintf = false;
						debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
						debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
					}
				}
			}
		}
		#endif
	}
	#endif
}

void StateSetToClosingAtOpened(_DoorDistance Mode)
{
	int32_t CurrEncPos=0;
	  /*
	#define OPEN_ACCEL_SECTiON			500				// 580 - 650mm opening
	#define OPEN_CONST_SECTiON			2000			// 2000 - 650mm opening
	#define OPEN_DEACE_SECTiON			3580			// 3580 - 650mm opening

	#define CLOSE_ACCEL_SECTiON			3000			// 3580 - 650mm closing
	#define CLOSE_CONST_SECTiON			1580			// 1580 - 650mm closing
	#define CLOSE_DEACE_SECTiON			0				// 0 - 650mm closing
	 
	    */ 

	if(Mode==DOOR_POSITION_OPENED_BY_END)
	{
		if(ClosePowerValue == 0) //2500
		{
			ClosingSection.Acceleration = CLOSE_ACCEL_SECTiON;		// 2.5s(3000)
			ClosingSection.Constant = CLOSE_CONST_SECTiON;			// 2.5s(2250)
			ClosingSection.Deceleration = CLOSE_DEACE_SECTiON;		// 2.5s(0)
		}
		else if(ClosePowerValue == 1) //3000
		{
			ClosingSection.Acceleration = CLOSE_ACCEL_SECTiON;		// 3s(3000)
			ClosingSection.Constant = (CLOSE_CONST_SECTiON-200);	// 3s(2050)
			ClosingSection.Deceleration = CLOSE_DEACE_SECTiON;		// 3s(0)

		}
		else if(ClosePowerValue == 2) //3500
		{
			ClosingSection.Acceleration = CLOSE_ACCEL_SECTiON;		// 3.5s(3000)
			ClosingSection.Constant = (CLOSE_CONST_SECTiON-800);	// 3.5s(1250)
			ClosingSection.Deceleration = CLOSE_DEACE_SECTiON;		// 3.5s(0)

		}
		else if(ClosePowerValue == 3) //4000
		{
			ClosingSection.Acceleration = CLOSE_ACCEL_SECTiON;		// 4s(3000)
			ClosingSection.Constant = (CLOSE_CONST_SECTiON-1200);	// 4s(1050)
			ClosingSection.Deceleration = CLOSE_DEACE_SECTiON;		// 4s(0)

		}
		else if(ClosePowerValue == 4) //4500
		{
			ClosingSection.Acceleration = CLOSE_ACCEL_SECTiON;		// 4.5s(3000)
			ClosingSection.Constant = (CLOSE_CONST_SECTiON-2100);	// 4.5s(150)
			ClosingSection.Deceleration = CLOSE_DEACE_SECTiON;		// 4.5s(0)

		}
		else if(ClosePowerValue == 5) //5000
		{
			ClosingSection.Acceleration = CLOSE_ACCEL_SECTiON;		// 5s(3000)
			ClosingSection.Constant = (CLOSE_CONST_SECTiON-2100);	// 5s(150)
			ClosingSection.Deceleration = CLOSE_DEACE_SECTiON;		// 5s(0)

		}
		else
		{
			ClosingSection.Acceleration = CLOSE_ACCEL_SECTiON;		// 2.5s(3000)
			ClosingSection.Constant = CLOSE_CONST_SECTiON;			// 2.5s(2450)
			ClosingSection.Deceleration = CLOSE_DEACE_SECTiON;		// 2.5s(0)

		}
/*JEON_190530
		ClosingSection.Acceleration = (m_OpeningEncoderPulse*8)/10;		// Opened에서 Closing 하는 경우 가속구간 설정
		ClosingSection.Constant = (m_OpeningEncoderPulse*6)/10;			// Opened에서 Closing 하는 경우 등속구간 설정
		ClosingSection.Deceleration = 0;								// Opened에서 Closing 하는 경우 감속구간 설정
*/
		mdc_DoorState = DOOR_CLOSING;
		mdc_PreDoorState = DOOR_CLOSING;
		mmf_PIDControl = PID_CONTROL_CLOSING;
	}
	else if(Mode==DOOR_POSITION_OPENED_BY_BUTTON)
	{
		CurrEncPos = mmf_Encoder.Position;
		ClosingSection.Acceleration = (CurrEncPos*7)/10;			// Opening 도중 버튼을 풀어 Closing 하는 경우 가속구간 설정
		ClosingSection.Constant = (CurrEncPos*5)/10;				// Opening 도중 버튼을 풀어 Closing 하는 경우 등속구간 설정
		ClosingSection.Deceleration = 0;							// Opening 도중 버튼을 풀어 Closing 하는 경우 감속구간 설정
		mmf_Encoder.RefVelocity = 0;								// 다시 PWM 0부터 가속/등속/감속이 되개 Opening 시 설정된 RefVelocity 값을 0으로 리셋
		mdc_PreDoorState = DOOR_OPENING;
		mdc_DoorState = DOOR_CLOSING;
		mmf_PIDControl = PID_CONTROL_CLOSING;
	}
	else if(Mode==DOOR_POSITION_OPENED_BY_OBSTACLE)
	{
		CurrEncPos = mmf_Encoder.PreStopPosition + 200;
		ClosingSection.Acceleration = (CurrEncPos*7)/10;
		ClosingSection.Constant = (CurrEncPos*5)/10;
		ClosingSection.Deceleration = 0;
		mdc_DoorState = DOOR_CLOSING;
		mdc_PreDoorState = DOOR_CLOSING;
		mmf_PIDControl = PID_CONTROL_CLOSING;
	}
}

void StateSetToOpenedAtOpening(_DoorDistance Mode)
{
	int32_t PrePosition=0, MatchCnt=0;
	
	mmf_PIDControl = PID_CONTROL_NONE;												// PID제어를 먼저 중지 시키고
	mmf_NextMotorDirSelect=PID_CONTROL_CLOSING;
	setPIDPWMOut();
	mmo_DoorFree();																	// 모터를 Free로 놔야 함
	if(Mode==DOOR_POSITION_OPENED_BY_END)												// 끝까지 와서 열렸으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
	{
		osDelay(1);
		mmo_DoorBrake();
	}
	else if(Mode==DOOR_POSITION_OPENED_BY_BUTTON)
	{
		/*
		 * Free 후 도어가 정지할 때까지 기다린 후 Opening 동작을 수행해야 함
		 */
		while(MatchCnt<100)
		{
			if(PrePosition==mmf_Encoder.Position)	MatchCnt++;
			else									MatchCnt=0;
			
			PrePosition = mmf_Encoder.Position;
			osDelay(1);
		}
	}
	
	/*
	 * MiddleOpened(장애물로열림완료)가 아닌 FullOpened(엔코더/전류로 열림완료) 상태에서만 최대거리 저장
	 */
	/*
	 * 장애물 없이 완전열림
	 */
	if(mod_Detect.ObstacleDetectCnt==0)
	{
		mmf_Encoder.MaxOpenPosition = mmf_Encoder.Position;
	}
	/*
	 * 장애물 3회 감지 시 완전열림
	 */
	else if(mod_Detect.ObstacleDetectCnt > mod_Detect.ObstacleConfigValue)
	{
		mmf_Encoder.MaxOpenPosition = mmf_Encoder.Position;
	}
}

static void DecisionOnOpened(void)
{
	volatile static _Bool PreOpenCloseButton = false;
	volatile static _Bool PreReopenButton = false;
	uint32_t CurrTime = 0;
	volatile static uint8_t loopcnt = 0;
	/*
	 * Initialization at Opened
	 */
	if((mdc_PreDoorState==DOOR_OPENING)&&(mdc_DoorState==DOOR_OPENED))
	{
		StateSetToOpenedAtOpening(DOOR_POSITION_OPENED_BY_END);
		mdc_PreDoorState=DOOR_OPENED;
		mdc_isFirstOpened = true;
		
		/*
		 * 열림(Opening)시간 계산
		 */
		mdm_time.Opened = osKernelSysTick();
		mdm_time.Opening = mdm_time.Opened - mdm_time.OpeningStart;
		debug("## 열림완료: 열림시간(%d), 열림위치(%d)\r\n",
					(int)mdm_time.Opening, (int)mmf_Encoder.Position);
		if(mmf_EndDecision.OpenedByPosition == true) mmf_EndDecision.OpenedByPosition = false;
		#ifdef DEBUG_SIMULATOR	// Opening -> Opened: 모터 전류값 0으로 리셋
			/*
			 * 시뮬레이션에서 Open 완료되어 정지시키면 전류값이 0으로 떨어져야 함.
			 */
			m_adc.MotorCurrent = 0;
		#endif
	}
	/*
	 * Opened -> Closing by OpenClose Button Release, 장애감지
	 */
	/*
	 * Opened -> Opened after OpenClose Button Push, 장애감지
	 */
	else
	{
		/*
		 * Opened -> Closing : 완전 열린상태에서 열림버튼을 풀어 닫힘동작 수행
		 */
		if(mip_Input.CommandOpen == true)
		{
			if(mip_Input.CommandClose == true)
			{
				mip_Input.CommandOpen = false; // 값 초기화
				if(mip_Input.CommandTestMode == true)
				{
					mdc_DoorState = DOOR_CLOSING;
					mmf_PIDControl = PID_CONTROL_NONE;
					mmo_DoorClosing(DCU_SLOWMODE_POWER);
					mdm_time.ClosingStart = osKernelSysTick();
					mdm_time.ClosingPrintf = 1u;
				}
				else
				{
					StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
					mdm_time.ClosingStart = osKernelSysTick();
					mdm_time.ClosingPrintf = 1u;
				}
			}
		}
		else if(!mip_Input.di0_OpenCloseButton && !mip_Input.di0_ReOpen && (PreReopenButton != mip_Input.di0_ReOpen))
		{
			/*
			 *  closing 도중 reopen 버튼을 눌러서 open 완료가 된 경우에는 reopen 버튼을 뗄 경우 닫힘 동작을 수행해야한다.
			 */
			if(mdc_isInitComplete == true)
			{
				debug("# 열림후 ReOpen버튼을 풀어 -> 닫힘 (초기화 후)\r\n");
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
			}
			else
			{
				debug("# 열림후 ReOpen버튼을 풀어 -> 닫힘 (초기화 전)\r\n");
				mdc_DoorState = DOOR_CLOSING;
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorClosing(DCU_SLOWMODE_POWER);
			}
			/*
			 * 닫힘(Closing) 시작시간 저장
			 */
			mdm_time.ClosingStart = osKernelSysTick();
			mdm_time.ClosingPrintf = 1u;
			if(debugprint.Data_Print_flag == true)
			{
				printf("Opened -> Closing Start \r\n");
				printf("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
				printf("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
			}
		}
		/*
		 * Opened -> Closing : 장애물3회 감지로 완전 열린상태에서 열림버튼 또는 재열림(ReOpen)버튼으로 닫힘동작 수행
		 * 장애물 3회 감지하고 닫힐 때 다시 감지하고 닫힐 때  장애물 감지하면 이동작 안함
		 */
		else if(mod_Detect.ObstacleDetectCnt >= mod_Detect.ObstacleConfigValue)
		{
			if(!mip_Input.di0_OpenCloseButton && PreOpenCloseButton)				// 열림버튼을 눌렀다 뗀 경우
			{
				debug("# 장애물 감지로 완전열림후 열림버튼을 풀어 -> 닫힘 (초기화 후)\r\n");
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
				if(debugprint.Data_Print_flag == true)
				{
					debug("Opened -> Closing Start \r\n");
					debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
					debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
				}
			}
			else if(mip_Input.di0_ReOpen)
			{
				debug("# 장애물 감지로 완전열림후 재열림 버튼을 눌러 -> 닫힘 (초기화 후)\r\n");
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
				if(debugprint.Data_Print_flag == true)
				{
					debug("Opened -> Closing Start \r\n");
					debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
					debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
				}
				#ifdef DEBUG_SIMULATOR	// Opened -> Closing: 재열림버튼 입력 해지
					mip_Input.di0_ReOpen = false;
					debug("# 시뮬레이터로 Reopen 입력 해지\r\n");
				#endif
			}
			/*
			 * 닫힘(Closing) 시작시간 저장
			 */
			mdm_time.ClosingStart = osKernelSysTick();
			mdm_time.ClosingPrintf = 1u;
		}
		else if(!mip_Input.di0_OpenCloseButton && (mod_Detect.ObstacleDetectCnt < mod_Detect.ObstacleConfigValue) && !mip_Input.di0_ReOpen)
		{
			if(mdc_isInitComplete == true)
			{
				debug("# 열림후 열림버튼을 풀어 -> 닫힘 (초기화 후)\r\n");
//				printf("OBS : %d, Position : %d \r\n", mod_Detect.ObstacleDetectCnt, mmf_Encoder.Position);
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
			}
			else
			{
				debug("# 열림후 열림버튼을 풀어 -> 닫힘 (초기화 전)\r\n");
				mdc_DoorState = DOOR_CLOSING;
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorClosing(DCU_SLOWMODE_POWER);
			}
			/*
			 * 닫힘(Closing) 시작시간 저장
			 */
			mdm_time.ClosingStart = osKernelSysTick();
			mdm_time.ClosingPrintf = 1u;
			if(debugprint.Data_Print_flag == true)
			{
				debug("Opened -> Closing Start \r\n");
				debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
				debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
			}
		}
		/*
		 * Opened -> Opened after OpenClose Button Push, 장애감지
		 */
		else
		{


			// No Action
			#ifdef DEBUG_SIMULATOR	// Opened -> Opened: 재열림버튼 입력 해지
				if(mip_Input.di0_ReOpen)
				{
					mip_Input.di0_ReOpen = false;
					debug("# 시뮬레이터로 Reopen 입력 해지\r\n");
				}
			#endif
		}
	}
	mmf_EndDecision.ClosedByCurrent = false;
	mmf_EndDecision.ClosedByPosition = false;
	PreOpenCloseButton = mip_Input.di0_OpenCloseButton;
	PreReopenButton    = mip_Input.di0_ReOpen;
}


static void DecisionOnClosing(void)
{
//jeon_190514	volatile static _Bool FlagDODBPS=false, FlagClosingByDODBPS=false, FlagFreeByDODBPS=false;
	volatile static _Bool FlagReClosingByDODBPS=false, FlagClosingByDODBPS=false, FlagFreeByDODBPS=false;
	volatile static uint32_t dodbpsClosingStartTime=0, dodbpsClosingTime=0;
	
    /*
	 * Closing -> Opening by OpenClose Button Push or Obstacle
	 */
	#if 1
    {
		/*
		 * 닫힘중에 유지보수프로그램의 열림 버튼을 누르면 열림동작을 수행하지 않음
		 */
		if((mip_Input.CommandTestMode == true) || (mip_Input.CommandClose == true))
		{
			/*
			 * 190311 : 유지보수프로그램으로 장애물 감지 기능 추가
			 */
			if(mdc_isOpeningByObstacle)
			{
				/*
				 * DODBPS 입력이 없을 때 장애감지로 열림
				 */
				if(!mip_Input.di0_Bypass)
				{
					debug("# 닫힘중:장애물 감지 1-> 열림\r\n");
//					printf("OBS : %d, Position : %d \r\n", mod_Detect.ObstacleDetectCnt, mmf_Encoder.Position);
					FlagDODBPS = false;												// 만약 dod닫힘중 dodbps 버튼을 풀경우 해당 플래그는 초기화 시켜야 한다.
					FlagClosingByDODBPS = false;
					mdc_PreDoorState = DOOR_CLOSING;
					mdc_DoorState = DOOR_OBSTACLE;
					StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_OBSTACLE);	// 장애물 감지 후 다시 열기 위해 중간지점에서 Closed
					mdm_time.OpeningStart = osKernelSysTick();						// Closing 상태에서 장애물 감지로 Opening 시작
					return;															// 닫힘 중 열림 시 닫힘완료 판단 수행할 필요 없음
				}
				/*
				 * DODBPS 입력이 있을 때 DODBPS Closing 설정
				 */
				else
				{
					mdc_isOpeningByObstacle = false;
//jeon_190514					mod_Detect.ObstacleDetectCnt = 0;								//dodbps동작에서는 장애물 감지 값은 필요 없음
					mod_Detect.ObstacleDetectCnt = 1;								//dodbps동작에서는 장애물 감지 값은 필요 없음
					if(FlagDODBPS == false)
					{
						dodbpsClosingStartTime = osKernelSysTick();
						FlagDODBPS = true;												// 닫힘완료(DLS,DCS누름)가 되면 클리어
						FlagClosingByDODBPS = true;
						mdc_FlagTestDODBPS = true;
					}
				}
			}
			else
			{
				// No Action
				if((debugprint.Data_Print_flag == true) && (mmf_Encoder.Position <=1500))
				{
					if(mdm_time.ClosingPrintf == true)
					{
						mdm_time.ClosingPrintf = false;
						debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
						debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
					}
				}
			}
		}
		else
		{
			if((mip_Input.di0_OpenCloseButton) || (mip_Input.di0_ReOpen))
			{
				mdm_time.OpeningStart = osKernelSysTick();					// Closing 상태에서 열림버튼을 눌러 Opening 시작
				/*
				 *  dodbps 감지중 열림 명령 후에 다시 닫힘 명령을 주면 dodbps 동작을 실행하고 쾅 닫힘.
				 *  열림 명령이 들어오면 값들을 초기화 시킨다.
				 */
				FlagDODBPS = false;
				mdc_isOpeningByObstacle = false;
				mod_Detect.ObstacleDetectCnt = 0;
				
				if (mip_Input.di0_OpenCloseButton)
				{
					debug("# 닫힘중:열림버튼을 눌러 -> 열림\r\n");
				}
				else
				{
					debug("# 닫힘 중 Reopen 버튼을 눌러 - > 열림 \r\n");
				}

				if(mdc_isInitComplete == true)
				{
					/*
					 * Closing -> Closed
					 */
					StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);	// 닫힘도중 열림버튼을 눌러 다시 열기위해 중간지점에서 Closed
					StateSetToOpeningAtClosed(DOOR_POSITION_CLOSED_BY_BUTTON);
					return;																			// 닫힘 중 열림 시 닫힘완료 판단 수행할 필요 없음
				}
				else
				{
					mdc_DoorState = DOOR_OPENING;
					mmf_PIDControl = PID_CONTROL_NONE;
					mmo_DoorOpening(DCU_SLOWMODE_POWER);
				}
			}
			/*
			 * 닫힘중 - 장애물감지
			 * DODBPS 닫힘 동작 중 장애물 감지 시 dodbpsClosingStartTime 이 계속 갱신 됨 왜냐 하면
			 * 장애물감지 로직에서 mdc_isOpeningByObstacle을 셋 시킴
			 */
			else if(mdc_isOpeningByObstacle)
			{
				/*
				 * DODBPS 입력이 없을 때 장애감지로 열림
				 */
				if(!mip_Input.di0_Bypass)
				{
					debug("# 닫힘중:장애물 감지 2-> 열림\r\n");
					FlagDODBPS = false;												// 만약 dod닫힘중 dodbps 버튼을 풀경우 해당 플래그는 초기화 시켜야 한다.
					FlagClosingByDODBPS = false;
					mdc_PreDoorState = DOOR_CLOSING;
					mdc_DoorState = DOOR_OBSTACLE;
					StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_OBSTACLE);	// 장애물 감지 후 다시 열기 위해 중간지점에서 Closed
					mdm_time.OpeningStart = osKernelSysTick();						// Closing 상태에서 장애물 감지로 Opening 시작
					return;															// 닫힘 중 열림 시 닫힘완료 판단 수행할 필요 없음
				}
				/*
				 * DODBPS 입력이 있을 때 DODBPS Closing 설정
				 */
				else
				{
					debug("# DODBPS 닫힘중:장애물 감지\r\n");
					mdc_isOpeningByObstacle = false;
					mod_Detect.ObstacleDetectCnt = 0;								//dodbps동작에서는 장애물 감지 값은 필요 없음
					if(FlagDODBPS == false)
					{
						dodbpsClosingStartTime = osKernelSysTick();
						FlagDODBPS = true;												// 닫힘완료(DLS,DCS누름)가 되면 클리어
						FlagClosingByDODBPS = true;
						mdc_FlagTestDODBPS = true;
					}
					else
					{
						if(FlagReClosingByDODBPS == true)
						{
							dodbpsClosingStartTime = osKernelSysTick();
							FlagClosingByDODBPS = true;
							mdc_FlagTestDODBPS = true;
						}	
					}
				}
			}
			else
			{
				// No Action
				if((debugprint.Data_Print_flag == true) && (mmf_Encoder.Position <=1500))
				{
					if(mdm_time.ClosingPrintf == true)
					{
						mdm_time.ClosingPrintf = false;
						printf("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
						printf("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
					}
				}
			}
		}
    }
	#endif
	
	/*
	 * Closing -> Closed or Closing
	 */
	#if 1
	{
		/*
		 * 자동 닫힘완료 스위치 On
		 */
		#ifdef DEBUG_SIMULATOR
			/*
			 * 닫힘스위치가 눌리지 않은 상태에서 엔코더가 3000->50 만큼 이동하면 시뮬레이터로 닫힘스위치를 눌러 줌
			 * 
			 * 위치를 조건에 추가하지 않으면 Closing 도중 열림버튼을 눌러 Opening 할 때 엔코더 오류 발생
			 * 	1. Closing에서 열림버튼 Push
			 * 	2. Closed 상태가 되어 닫힘스위치 true로 설정
			 * 	3. Opening->Opened 시 닫힘스위치가 true인 상태에서 열림완료를 판단했으므로 엔코더 오류 발생
			 */
			if((!mip_Input.DoorClosedSwitchOn) && (mmf_Encoder.Position<100) && !mdc_FlagTestDODBPS)
			{
				mip_Input.di0_DLS1 = true;
				mip_Input.di1_DLS2 = true;
				mip_Input.di1_DCS1 = true;
				mip_Input.di1_DCS2 = true;
				mip_Input.DoorClosedSwitchOn = true;
				debug("# 시뮬레이션모드에서 강제로 닫힘스위치 Push\r\n");
			}
		#endif
		
		/*
		 * Closing -> Closed by Encoder or Current
		 */
		#if 1
		{
			/*
			 * DLS1 or DLS2가 둘 중 하나 눌리면 닫힘완료 판단 (병렬이므로 무조껀 둘 중 하나가 눌려야 함)
			 */
			if(mip_Input.DoorClosedSwitchOn)
			{
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSED;
				
				FlagDODBPS = false; //jeon_190514

				/*
				 * 엔코더로 멈추는 경우 -> 제일 먼저 엔코더로 멈춤을 시도해야 함
				 */
#if 0 //jeon_190618					
				if(mmf_EndDecision.ClosedByPosition)
				{
					mmf_EndDecision.ClosedByPosition = false;
					
					mdc_PreDoorState = DOOR_CLOSING;
					mdc_DoorState = DOOR_CLOSED;
					
					FlagDODBPS = false; //jeon_190514
					if((mdc_isInitComplete == true) || (mdc_isFirstClosed == true))
					{
						/*
						 * 닫힘조건이 만족되어도 PID제어 때문에 속도를 갑자기 0으로 만들면 안되고, 일정시간 엔코더변화가 없으면 닫힘완료 판단
						 */
						debug("# 닫힘중:엔코더로 -> 닫힘완료 판단\r\n");
						/*
						 * 초기화 후 Closed 조건이 만족되고, 일정시간 엔코더변화가 없으면 닫힘완료 판단
						 */
						mdc_PreDoorState = DOOR_CLOSING;
						mdc_DoorState = DOOR_CLOSED;
						
						FlagDODBPS = false; //jeon_190514
						
						if(debugprint.Data_Print_flag == true)
						{
							debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
							debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
						}
					}
					else
					{
						// 초기화 전에는 엔코더값을 모르므로 엔코더로 닫힘완료 판단 불가 -> No Action
					}
				}
				/*
				 * 전류로 멈추는 경우 -> 초기화와 상관없이 전류로 닫힘완료가 판단되면 무조껀 항상 정지시켜야 함
				 */
				else if(mmf_EndDecision.ClosedByCurrent)
				{
					mmf_EndDecision.ClosedByCurrent = false;
					debug("# 닫힘중:전류로 -> 닫힘완료 판단\r\n");
					/*
					 * [중요] B_emf모드는 초기화 전/후 동일하게 정지상태만 만족하면 바로 멈춰야 함
					 * 
					 * 모터 정지 상태에서도 B_emf전압은 계속 측정 됨
					 * -> B_emf를 통한 속도감소 발생, 위치되 계속 감소함
					 * -> 속도0이 일정시간 유지되는 조건으로 Closed 상태를 만들 수 없음
					 * -> 계속 Closing 방향으로 FET를 Turn On 시키고 있음
					 * 
					 * -> 닫힘완료가 안된 상태에서 Open명령으로 Opening 동작을 수행하면
					 * -> FET 쇼트가 발생해 MCU Reset되는 현상이 발생되는 것으로 추정됨
					 */
					mdc_PreDoorState = DOOR_CLOSING;
					mdc_DoorState = DOOR_CLOSED;
					
					FlagDODBPS = false; //jeon_190514
					
					if(debugprint.Data_Print_flag == true)
					{
						mdm_time.OpeningPrintf = false;
						debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
						debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
					}
				}
#endif

			}
			/*
			 * DLS1 or DLS2가 둘 다 눌리지 않으면 Closing -> Closing
			 */
			else
			{
				/*
				 * DLS 2개가 눌리지 않으면 닫힘 시간 5초뒤 , 엔코더 위치, 전류값 판단으로 닫힘완료를 수행한다.
				 */
				if( ((mip_Input.di0_DLS1 == true) && (mip_Input.di1_DLS2 == true)) &&
					((mip_Input.di1_DCS1 == false) || (mip_Input.di1_DCS2 == false)))
				{
					if(get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 6000) //orig:5000, jeon_190709
					{
						if((mmf_EndDecision.ClosedByCurrent == true) || (mmf_EndDecision.ClosedByPosition == true))
						{
							mdc_PreDoorState = DOOR_CLOSING;
							mdc_DoorState = DOOR_CLOSED;
							if(mmf_EndDecision.ClosedByCurrent == true)
							{
								mmf_EndDecision.ClosedByCurrent = false;
								debug("DCS 입력 확인 불가  전류 값으로 닫힘완료 확인 \r\n");
							}
							if(mmf_EndDecision.ClosedByPosition == true)
							{
								mmf_EndDecision.ClosedByPosition = false;
								debug("DCS 입력 확인 불가  위치 값으로 닫힘완료 확인 \r\n");
							}
							if(debugprint.Data_Print_flag == true)
							{
								mdm_time.OpeningPrintf = false;
								debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
								debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
							}
						}
						else
						{
							// No Action
						}
					}
					else
					{
						// No Action
					}
				}
#if 1 //jeon_190624 orig:1
				else if( ((mip_Input.di0_DLS1 == false) && (mip_Input.di1_DLS2 == false)) &&
					((mip_Input.di1_DCS1 == true) || (mip_Input.di1_DCS2 == true)))
				{
					if(get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 6000) //orig:5000, jeon_190709					//korail128량 구조상 dcs가 좀더 뒤 쪽에 있기 때문에  6초뒤에 닫힘 완료 판단을 수행
					{
						if((mmf_EndDecision.ClosedByCurrent == true) || (mmf_EndDecision.ClosedByPosition == true))
						{
							mdc_PreDoorState = DOOR_CLOSING;
							mdc_DoorState = DOOR_CLOSED;
							if(mmf_EndDecision.ClosedByCurrent == true)
							{
								mmf_EndDecision.ClosedByCurrent = false;
								debug("DLS 입력 확인 불가  전류 값으로 닫힘완료 확인 \r\n");
							}
							if(mmf_EndDecision.ClosedByPosition == true)
							{
								mmf_EndDecision.ClosedByPosition = false;
								debug("DlS 입력 확인 불가  위치 값으로 닫힘완료 확인 \r\n");
							}
							if(debugprint.Data_Print_flag == true)
							{
								mdm_time.OpeningPrintf = false;
								debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
								debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
							}
						}
						else
						{
							// No Action
						}
					}
					else
					{
						// No Action
					}
				}
#endif
				/*
				 * DODBPS Closing : 8초 Closing, 3초 Free 반복
				 * 장애물 감지 력(200N~300N) 이 안맞을 경우 mmo_DoorClosing(200); 을 수정하면 될 듯
				 */
				if(FlagDODBPS)
				{
					/*
					 * dodbps 누른 상태에서 장애물 감지시 엔코더 에러가 뜨는데. dodbps동작중에는 모터-엔코더 고장을 감지하지않도록 수정
					 */
					error_list_time[DCU_MOTOR_ERROR] =osKernelSysTick();
					error_list_time[DCU_ENCODER_ERROR] = osKernelSysTick();
					if(FlagClosingByDODBPS)
					{
		    			if(FlagReClosingByDODBPS == false)
		    			{
		    			    
		    				//jeon_190516 mmo_DoorClosing(DCU_DODBPS_POWER);	
		    				if((motor_voltage >= 1900) && (motor_voltage < 2300))
		    				{
		    					mmo_DoorClosing(DCU_DODBPS_POWER+250);
		    				}
		    				else if(motor_voltage >= 3600)
		    				{
		    					mmo_DoorClosing(DCU_DODBPS_POWER-100);
		    				}
		    				else
		    				{
		    					mmo_DoorClosing(DCU_DODBPS_POWER);
		    				}
		    			}
		    			else
		    			{
		    				mmo_DoorClosing(DCU_DODBPS_POWER-70);	
		    			}

		    			FlagClosingByDODBPS = false;
						FlagReClosingByDODBPS = false;
		    			mmf_PIDControl = PID_CONTROL_NONE;											// PID제어를 먼저 중지 시키고
//jeon_190514						mmo_DoorFree();
						debug("# dodbps closing 동작 실시 : %d\r\n", mod_Detect.ObstacleDetectCnt);
					}
					else if(FlagFreeByDODBPS)
					{
						FlagFreeByDODBPS = false;
		    			mmf_PIDControl = PID_CONTROL_NONE;											// PID제어를 먼저 중지 시키고
						mmo_DoorClosing(0);
						mmo_DoorFree();
						debug("# dodbps free 동작 실시 \r\n");
					}
					else
					{
						/*
						 * 8~11s Free
						 */
						/*
						 * DODBPS Closing : 8초 Closing, 3초 Free 반복
						 * todo <<jjkim_181219>> : DODBPS 동작 확인 [3]
						 * comment : DODBPS 동작 확인 [1]의 내용대로 dodbpsClosingStartTime이 계속 갱신 되는데 시간 경과에 따른 동작이 안될 것 같음
						 */
						dodbpsClosingTime = osKernelSysTick();
						if(get_diff_tick(dodbpsClosingTime,dodbpsClosingStartTime) > DCU_DODBPS_CLOSINGTIME)              //free 동작 후 다시 Closing 동작 실시
						{
//							FlagClosingByDODBPS = true;
							FlagReClosingByDODBPS = true;
							dodbpsClosingStartTime = osKernelSysTick();
							
							mmf_PIDControl = PID_CONTROL_NONE;
							mmo_DoorClosing(DCU_SLOWMODE_POWER);
						}
						else if(get_diff_tick(dodbpsClosingTime,dodbpsClosingStartTime) > DCU_DODBPS_FREETIME)			//8초뒤 free동작 실시
						{
							FlagFreeByDODBPS = true;
							
//							mod_Detect.ObstacleDetectCnt = 0; //jeon_190514
//							m_CurrentPWMDuty = 200; //jeon_190514
						}
					}
				}
				/*
				 * Normal Closing 동작 계속 수행
				 */
				else
				{
					// No Action
					if((debugprint.Data_Print_flag == true) && (mmf_Encoder.Position <= 1500))
					{
						if(mdm_time.ClosingPrintf == true)
						{
							debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
							debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
						}
					}
				}
			}
		}
		#endif
	}
	#endif
}

/*
 *  도어 고장이 난 상태
 */
static void DecisionOnError(void)
{
	/*
	 *  고장 나기 이전의 도어상태를 보고 고장 동작을 결정 한다.
	 */
	switch (mdc_PreDoorState)
	{
		case DOOR_CLOSED:
			if(error_list_flag[DCU_HARD_FAULT] == true)		mmo_DoorFree();
			/*
			 *  DCS가 OFF로 고장이 난 상태
			 *   : Closed 상태에서 DCS가 눌려야 되는데 눌리지 않은 상태로 DCS고장
			 */
		    else if(error_list_flag[DCU_DCS1_FAULT] && error_list_flag[DCU_DCS2_FAULT])		FaultAction_F05_F06(mdc_PreDoorState);
			else if((error_list_flag[DCU_DLS1_FAULT]) && (error_list_flag[DCU_DLS2_FAULT]))	FaultAction_F04_F08(mdc_PreDoorState);
			else if(error_list_flag[DCU_UNEXPECTED_UNLOCK])	FaultAction_F07();
			break;
		case DOOR_CLOSING:
			if(error_list_flag[DCU_HARD_FAULT] == true)		mmo_DoorFree();
				else if(error_list_flag[DCU_MOTOR_ERROR])	mmo_DoorFree();
			break;
		case DOOR_OPENING:
			if(error_list_flag[DCU_HARD_FAULT] == true)		mmo_DoorFree();
			else if(error_list_flag[DCU_MOTOR_ERROR])		mmo_DoorFree();
			else if(error_list_flag[DCU_OPEN_FAULT])	    FaultAction_F10();
			break;
		case DOOR_OPENED:
			if(error_list_flag[DCU_HARD_FAULT] == true)		mmo_DoorFree();
			else if(error_list_flag[DCU_DCS1_FAULT] && error_list_flag[DCU_DCS2_FAULT])		FaultAction_F05_F06(mdc_PreDoorState);
			else if(error_list_flag[DCU_DLS2_FAULT] && error_list_flag[DCU_DLS1_FAULT])		FaultAction_F04_F08(mdc_PreDoorState);
			break;
		default:
			break;
	}
}

static void DecisionOnObstacle(void)
{
	enum obs_state {OBS_NONE,OBS_OPENING,OBS_OPEN};
	volatile static uint8_t ObstacleState = 0;
	uint32_t curr_time = 0;
	int32_t CurrEncPos = 0;

	/*
	 *  열림 명령이나 dodbps 신호, 장애물 감지 3회 이상인 경우
	 */
	if((mip_Input.di0_OpenCloseButton) || (mod_Detect.ObstacleDetectCnt >= mod_Detect.ObstacleConfigValue))
	{
		if(mod_Detect.ObstacleDetectCnt >= mod_Detect.ObstacleConfigValue)
		{
			debug("장애물 %d회 감지로 완전 열림 수행 \r\n",mod_Detect.ObstacleDetectCnt);
		}
		else
		{
			debug("열림 버튼 입력으로 열림 동작 수행 장애물 카운트 값 초기화 \r\n");
			mod_Detect.ObstacleDetectCnt = 0;
		}
		ObstacleState = OBS_NONE;
		mdc_DoorState = DOOR_OPENING;
		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
		
		CurrEncPos = mmf_Encoder.MaxOpenPosition - mmf_Encoder.PreStopPosition;			// Closing 도중 장애물 감지로 Opening 하는 경우
		OpeningSection.Acceleration = ((CurrEncPos*2)/10)+mmf_Encoder.PreStopPosition;	// Closing 도중 장애물 감지로 Opening 하는 경우 가속구간설정
		OpeningSection.Constant = ((CurrEncPos*4)/10)+mmf_Encoder.PreStopPosition;		// Closing 도중 장애물 감지로 Opening 하는 경우 등속구간설정
		OpeningSection.Deceleration = CurrEncPos+mmf_Encoder.PreStopPosition;			// Closing 도중 장애물 감지로 Opening 하는 경우 감속구간설정
		mmf_Encoder.RefVelocity = 0;
#if 0 //jeon_190624		
		mmf_PIDControl = PID_CONTROL_OPENING;
#endif
		mdc_isOpeningByObstacle = false;
#if 1 //jeon_190624		
		mmf_PIDControl = PID_CONTROL_NONE;
		mmo_DoorOpening(DCU_SLOWMODE_POWER);
#endif

		if(mip_Input.di0_OpenCloseButton) mod_Detect.ObstacleDetectCnt = 0;
	}
	else if(mip_Input.di0_Bypass)
	{
		ObstacleState = OBS_NONE;
		mdc_PreDoorState = DOOR_CLOSING;
		mdc_DoorState = DOOR_CLOSING;
	}
#if 0 //jeon_190618		
	else if(mip_Input.DoorClosedSwitchOn)
	{
		mmf_EndDecision.ClosedByPosition = true;
		
		mdc_PreDoorState = DOOR_CLOSING;
		mdc_DoorState = DOOR_CLOSED;
		mod_Detect.ObstacleDetectCnt = 0;
		ObstacleState = OBS_NONE;
	}
#endif		
	else
	{
		if(ObstacleState == OBS_NONE)
		{
#if 0 //jeon_191002			
			mmf_PIDControl = PID_CONTROL_NONE;
//jeon_190514			mmo_DoorOpening(DCU_SLOWMODE_POWER-100u);
			if(DLSSwitchOn == true) mmo_DoorOpening(DCU_SLOWMODE_POWER);
			else mmo_DoorOpening(DCU_SLOWMODE_POWER-100u);
			
			ObstacleState = OBS_OPENING;
			motor_voltage = m_adc.PowerVoltage;
#endif			
			if(mip_Input.DoorClosedSwitchOn == true)
			{
/* 차량 초기 조성 시 정렬이 안됐을 경우 - 시험용*/				
//jeon_191002				mmo_DoorOpening(DCU_SLOWMODE_POWER);

/* 차량 초기 조성 후 정렬이 된 경우 */				
				mmo_DoorBrake();
				
				mdc_isOpeningByObstacle = false;
				ObstacleState = OBS_NONE;
				
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSED;
			}
			else
			{
				mmf_PIDControl = PID_CONTROL_NONE;
				
				mmo_DoorOpening(DCU_SLOWMODE_POWER-100u);
				
				ObstacleState = OBS_OPENING;
				motor_voltage = m_adc.PowerVoltage;
			}

		}
		else if(ObstacleState == OBS_OPENING)
		{

			/*
			 * todo : 정남 공장가서 확인 후, 열림 폭이 부족하다면 +200 부분을 수정 하면 됨
			 */
#if 0 //jeon_190709
			if((mmf_Encoder.Position > (mmf_Encoder.PreStopPosition+138)) &&
			   (mmf_Encoder.Position < 9000))
			{
				debug("# 장애 %d회 감지 -> 장애물 열림완료 판단\r\n", mod_Detect.ObstacleDetectCnt);
				ObstacleState = OBS_OPEN;
				mmo_DoorBrake();
				/*
				 * todo : 재진씨 한테 확인하라고 해서 osKernelSysTick()로 해서 정상동작하면 HAL_GetTick() 사용하지 말기
				 */
				m_ObstacleOpenedTime = HAL_GetTick();
			}
#else
			if(mmf_Encoder.Position > 4600)
			{
				if((mmf_Encoder.Position > (mmf_Encoder.PreStopPosition+138)) &&
							   (mmf_Encoder.Position < 9000))
				{
					debug("# 장애 %d회 감지 -> 장애물 열림완료 판단\r\n", mod_Detect.ObstacleDetectCnt);
					ObstacleState = OBS_OPEN;
					mmo_DoorBrake();
					/*
					 * todo : 재진씨 한테 확인하라고 해서 osKernelSysTick()로 해서 정상동작하면 HAL_GetTick() 사용하지 말기
					 */
					m_ObstacleOpenedTime = HAL_GetTick();
				}
				else if(mip_Input.DoorClosedSwitchOn == true) //jeon_191002
				{
					mmo_DoorBrake();
					
					mdc_isOpeningByObstacle = false;
					ObstacleState = OBS_NONE;
					
					mdc_PreDoorState = DOOR_CLOSING;
					mdc_DoorState = DOOR_CLOSED;
				}

			}
#endif
			else if(mmf_Encoder.Position > 3600)
			{
				debug("# 장애물 열림완료 판단\r\n");
				ObstacleState = OBS_OPEN;
				mmo_DoorBrake();
			}
			else
			{
				if((mmf_Encoder.Position > (mmf_Encoder.PreStopPosition+138)) &&
											   (mmf_Encoder.Position < 9000))
				{
					debug("# 장애 %d회 감지 -> 장애물 열림완료 판단\r\n", mod_Detect.ObstacleDetectCnt);
					ObstacleState = OBS_OPEN;
					mmo_DoorBrake();
					/*
					 * todo : 재진씨 한테 확인하라고 해서 osKernelSysTick()로 해서 정상동작하면 HAL_GetTick() 사용하지 말기
					 */
					m_ObstacleOpenedTime = HAL_GetTick();
				}
			}
		}
		else if(ObstacleState == OBS_OPEN)
		{
			/*
			 * todo : 재진씨 한테 확인하라고 해서 osKernelSysTick()로 해서 정상동작하면 HAL_GetTick() 사용하지 말기
			 */
			curr_time = HAL_GetTick();
			if(get_diff_tick(curr_time,m_ObstacleOpenedTime) > 1000)
			{
				debug("# 1초후 닫힘 동작 Obs -> closing \r\n");
				mdc_isOpeningByObstacle = false;
				ObstacleState = OBS_NONE;
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSING;
//jeon_190514				mmo_DoorClosing(DCU_SLOWMODE_POWER-20);


				if((motor_voltage >= 1900) && (motor_voltage < 2300))
				{
					mmo_DoorClosing(DCU_OBSMODE_POWER+150);
					debug("Low Voltage OBS \r\n");
				}
				else if(motor_voltage >= 3600)
				{
					mmo_DoorClosing(DCU_OBSMODE_POWER-100);
					debug("High Voltage OBS \r\n");
				}
				else
				{
					mmo_DoorClosing(DCU_OBSMODE_POWER-30);
					debug("Normal Voltage OBS \r\n");
				}	
//				mmo_DoorClosing(DCU_OBSMODE_POWER); //jeon_190514
				mdm_time.ClosingStart =  osKernelSysTick();
			}
		}
	}
}

void ConfigDcuParameters(void)
{
	/*
	 * dcu 메모리맵 참조.
	 */
	uint16_t openConfig_value =0;
	uint16_t CloseConfig_value=0;
	uint16_t ObstaclConfig_value=0;
	uint16_t temp_value =0;

	openConfig_value    |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x005F);					//95번지
	temp_value 		     = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0060)<<8);					//96번지
	openConfig_value    |= temp_value;
	CloseConfig_value   |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0061);					//97번지
	temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0062)<<8);					//98번지
	CloseConfig_value   |= temp_value;
	ObstaclConfig_value |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0063);				//99번지
	temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0064)<<8);					//100번지
	ObstaclConfig_value |= temp_value;

	/*
	 * 만약 mram의 값을 읽었는데 그이상의 값이 나온다면 설계사양서의 내용대로 설정값을 지정한다.
	 */
	if((openConfig_value > 5000) || (CloseConfig_value > 5000) || (ObstaclConfig_value >=11))
	{
		openConfig_value = 2500u;
		CloseConfig_value= 3000u;
		ObstaclConfig_value =3u;
		mram_byte_write(MRAM_DATA_ADDR+0x005F,(uint8_t)(openConfig_value & 0x00FF));
		mram_byte_write(MRAM_DATA_ADDR+0x0060,(uint8_t)((openConfig_value & 0xFF00)>>8));
		mram_byte_write(MRAM_DATA_ADDR+0x0061,(uint8_t)(CloseConfig_value & 0x00FF));
		mram_byte_write(MRAM_DATA_ADDR+0x0062,(uint8_t)((CloseConfig_value & 0xFF00)>>8));
		mram_byte_write(MRAM_DATA_ADDR+0x0063,(uint8_t)(ObstaclConfig_value & 0x00FF));
		mram_byte_write(MRAM_DATA_ADDR+0x0064,(uint8_t)((ObstaclConfig_value & 0xFF00)>>8));
	}
	else if((openConfig_value == 0) || (CloseConfig_value == 0) || (ObstaclConfig_value ==0))
	{
		openConfig_value = 2500u;
		CloseConfig_value= 3000u;
		ObstaclConfig_value =3u;
		mram_byte_write(MRAM_DATA_ADDR+0x005F,(uint8_t)(openConfig_value & 0x00FF));
		mram_byte_write(MRAM_DATA_ADDR+0x0060,(uint8_t)((openConfig_value & 0xFF00)>>8));
		mram_byte_write(MRAM_DATA_ADDR+0x0061,(uint8_t)(CloseConfig_value & 0x00FF));
		mram_byte_write(MRAM_DATA_ADDR+0x0062,(uint8_t)((CloseConfig_value & 0xFF00)>>8));
		mram_byte_write(MRAM_DATA_ADDR+0x0063,(uint8_t)(ObstaclConfig_value & 0x00FF));
		mram_byte_write(MRAM_DATA_ADDR+0x0064,(uint8_t)((ObstaclConfig_value & 0xFF00)>>8));
	}
	mod_Detect.ObstacleConfigValue = ObstaclConfig_value;
	mdm_time.OpenConfigtime = (uint32_t)openConfig_value;
	mdm_time.CloseConfigtime =(uint32_t)CloseConfig_value;
	debug("열림시간 설정 값 : %d ms 닫힘 시간 설정 값 : %d ms 장애물 감지 설정 값 : %d \r\n",openConfig_value,CloseConfig_value,ObstaclConfig_value);
	switch(mdm_time.OpenConfigtime)
	{
		case Ms_2000: OpenPowerValue = 0; break;
		case Ms_2500: OpenPowerValue = 1; break;
		case Ms_3000: OpenPowerValue = 2; break;
		case Ms_3500: OpenPowerValue = 3; break;
		case Ms_4000: OpenPowerValue = 4; break;
		default: OpenPowerValue = 1;break;
	}
	switch(mdm_time.CloseConfigtime)
	{
		case Ms_2500: ClosePowerValue = 0; break;
		case Ms_3000: ClosePowerValue = 1; break;
		case Ms_3500: ClosePowerValue = 2; break;
		case Ms_4000: ClosePowerValue = 3; break;
		case Ms_4500: ClosePowerValue = 4; break;
		case Ms_5000: ClosePowerValue = 5; break;
		default: ClosePowerValue = 1;break;
	}
	
#if 0 //jeon_190708 default open 2.5s, colse 3s obs 3로 고정
	openConfig_value = 2500u;
	CloseConfig_value= 3000u;
	ObstaclConfig_value =3u;
	
	OpenPowerValue = 1;
	ClosePowerValue = 1;
#endif
}

void FaultAction_F07(void)
{
	volatile static _Bool FlagClosingByUnexpectedUnlock = false;
	volatile static _Bool FlagBrakeByUnexpectedUnlock = false;
	volatile static uint32_t UnExpectedUnlockTime = 0;
	volatile static int32_t before_encoder = 0;
	volatile static uint32_t F07_OpenTime = 0;
	volatile static uint32_t F07_ClosingTime = 0;
	volatile static uint32_t F07_OpeningTime = 0;
#if 0 //jeon_190617 수정	
	if((mip_Input.di0_DLS1) || (mip_Input.di1_DLS2))
	{
		mmo_DoorClosing(0);
		mmo_DoorFree();
		osDelay(1);																	// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
		mmo_DoorBrake();
		mdc_PreDoorState = DOOR_CLOSING;
		mdc_DoorState = DOOR_CLOSED;
		m_F07Statement = F07_BRAKE;
		/*
		 * todo : delay 쓰지 말고 count해서 지연하는 방법 적용 고려해 볼 것
		 */
		osDelay(100);
		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
	}
	else if(mip_Input.di0_OpenCloseButton == true)  //true : open, false : close
	{
		mdc_PreDoorState = DOOR_CLOSING;
		mdc_DoorState = DOOR_CLOSED;
		m_F07Statement = F07_BRAKE;
		mmo_DoorClosing(0);
		mmo_DoorFree();
		osDelay(1);																	// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
		mmo_DoorBrake();
		osDelay(100);
		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
		debug("F07 고장 동작에서 열림명령으로 CLOSED로 복귀 \r\n");
	}
	else
	{
		if(error_list_flag[DCU_ENCODER_ERROR] == false)
		{
			if(m_F07Statement == F07_BRAKE)
			{
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorOpening(DCU_SLOWMODE_POWER);
				before_encoder = mmf_Encoder.Position;
				m_F07Statement = F07_OPENING;
			}
			else if(m_F07Statement == F07_OPENING)
			{
				if(mmf_Encoder.Position > (before_encoder + 200))
				{
					m_F07Statement = F07_OPEN;
					mmo_DoorBrake();
					F07_OpenTime = osKernelSysTick();
				}
			}
			else if(m_F07Statement == F07_OPEN)
			{
				if(get_diff_tick(osKernelSysTick(),F07_OpenTime) > 1000)
				{
					m_F07Statement = F07_CLOSING;
					F07_ClosingTime = osKernelSysTick();
					mmo_DoorClosing(DCU_SLOWMODE_POWER);
				}
			}
			else //closing
			{
				if(get_diff_tick(osKernelSysTick(),F07_ClosingTime) > 1000)
				{
					m_F07Statement = F07_BRAKE;
					mmo_DoorBrake();
					osDelay(100);
				}
			}
		}
		else  //엔코더 고장일 경우
		{
			if(m_F07Statement == F07_BRAKE)
			{
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorOpening(DCU_SLOWMODE_POWER);
				before_encoder = mmf_Encoder.Position;
				F07_OpeningTime = osKernelSysTick();
				m_F07Statement = F07_OPENING;
			}
			else if(m_F07Statement == F07_OPENING)
			{
				if(get_diff_tick(osKernelSysTick(),F07_OpeningTime) > 500)
				{
					m_F07Statement = F07_OPEN;
					mmo_DoorBrake();
					F07_OpenTime = osKernelSysTick();
				}

			}
			else if(m_F07Statement == F07_OPEN)
			{
				if(get_diff_tick(osKernelSysTick(),F07_OpenTime) > 1000)
				{
					m_F07Statement = F07_CLOSING;
					F07_ClosingTime = osKernelSysTick();
					mmo_DoorClosing(DCU_SLOWMODE_POWER);
				}
			}
			else //closing
			{
				if(get_diff_tick(osKernelSysTick(),F07_ClosingTime) > 1000)
				{
					m_F07Statement = F07_BRAKE;
					mmo_DoorBrake();
					osDelay(100);
				}
			}
		}

	}
#else	
	if(mip_Input.di0_OpenCloseButton == true)  //true : open, false : close
	{
		mdc_PreDoorState = DOOR_CLOSING;
		mdc_DoorState = DOOR_CLOSED;
		m_F07Statement = F07_BRAKE;
		mmo_DoorClosing(0);
		mmo_DoorFree();
		osDelay(1);																	// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
		mmo_DoorBrake();
//		osDelay(100);
//		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
//		debug("F07 고장 동작에서 열림명령으로 CLOSED로 복귀 \r\n");
		
		if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* 열림 닫힘 동작을 반복하고 dls 쳤다가 풀린 경우 일 것임*/
		{

			Erase_fault(DCU_UNEXPECTED_UNLOCK);
		}
	}
	else
	{
		if((mip_Input.di0_DLS1) || (mip_Input.di1_DLS2))
		{
			mmo_DoorClosing(0);
			mmo_DoorFree();
			osDelay(1);																	// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
			mmo_DoorBrake();
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState = DOOR_CLOSED;
			m_F07Statement = F07_BRAKE;
			/*
			 * todo : delay 쓰지 말고 count해서 지연하는 방법 적용 고려해 볼 것
			 */
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
		}
		else
		{
			if(!mip_Input.di0_Bypass) //jeon_190708 F07은 DODBPS가 설정되지 않았을 때만 동작함
			{
				if(error_list_flag[DCU_ENCODER_ERROR] == false)
				{
					if(((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true)) && ((error_list_flag[DCU_DCS1_FAULT] == true) || (error_list_flag[DCU_DCS2_FAULT] == true)))
					{
						mmo_DoorClosing(0);
						mmo_DoorFree();
					}
					else if(((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true)) && ((error_list_flag[DCU_DLS1_FAULT] == true) || (error_list_flag[DCU_DLS2_FAULT] == true)))
					{
						mmo_DoorClosing(0);
						mmo_DoorFree();
					}
					else
					{
						if(m_F07Statement == F07_BRAKE)
						{
							mmf_PIDControl = PID_CONTROL_NONE;
							mmo_DoorOpening(DCU_SLOWMODE_POWER);
							before_encoder = mmf_Encoder.Position;
							m_F07Statement = F07_OPENING;
						}
						else if(m_F07Statement == F07_OPENING)
						{
							if(mmf_Encoder.Position > (before_encoder + 200))
							{
								m_F07Statement = F07_OPEN;
								mmo_DoorBrake();
								F07_OpenTime = osKernelSysTick();
							}
						}
						else if(m_F07Statement == F07_OPEN)
						{
							if(get_diff_tick(osKernelSysTick(),F07_OpenTime) > 1000)
							{
								m_F07Statement = F07_CLOSING;
								F07_ClosingTime = osKernelSysTick();
								mmo_DoorClosing(DCU_SLOWMODE_POWER);
							}
						}
						else //closing
						{
							if(get_diff_tick(osKernelSysTick(),F07_ClosingTime) > 1000)
							{
								m_F07Statement = F07_BRAKE;
								mmo_DoorBrake();
								osDelay(100);
							}
						}
					}	
				}
				else  //엔코더 고장일 경우
				{
					if(((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true)) && ((error_list_flag[DCU_DCS1_FAULT] == true) || (error_list_flag[DCU_DCS2_FAULT] == true)))
					{
						mmo_DoorClosing(0);
						mmo_DoorFree();
					}
					else if(((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true)) && ((error_list_flag[DCU_DLS1_FAULT] == true) || (error_list_flag[DCU_DLS2_FAULT] == true)))
					{
						mmo_DoorClosing(0);
						mmo_DoorFree();
					}
					else
					{
						if(m_F07Statement == F07_BRAKE)
						{
							mmf_PIDControl = PID_CONTROL_NONE;
							mmo_DoorOpening(DCU_SLOWMODE_POWER);
							before_encoder = mmf_Encoder.Position;
							F07_OpeningTime = osKernelSysTick();
							m_F07Statement = F07_OPENING;
						}
						else if(m_F07Statement == F07_OPENING)
						{
							if(get_diff_tick(osKernelSysTick(),F07_OpeningTime) > 500)
							{
								m_F07Statement = F07_OPEN;
								mmo_DoorBrake();
								F07_OpenTime = osKernelSysTick();
							}
		
						}
						else if(m_F07Statement == F07_OPEN)
						{
							if(get_diff_tick(osKernelSysTick(),F07_OpenTime) > 1000)
							{
								m_F07Statement = F07_CLOSING;
								F07_ClosingTime = osKernelSysTick();
								mmo_DoorClosing(DCU_SLOWMODE_POWER);
							}
						}
						else //closing
						{
							if(get_diff_tick(osKernelSysTick(),F07_ClosingTime) > 1000)
							{
								m_F07Statement = F07_BRAKE;
								mmo_DoorBrake();
								osDelay(100);
							}
						}
					}	
				}
			}	
			else //jeon_190708 F07은 DODBPS가 설정되지 않았을 때만 동작함
			{
				m_F07Statement = F07_CLOSING;
				F07_ClosingTime = osKernelSysTick();
				mmo_DoorClosing(DCU_SLOWMODE_POWER);
			}
		}
	}
#endif
}

void FaultAction_F10(void)
{
	volatile static uint32_t F10_OpenTime = 0;
	volatile static uint32_t F10_ClosingTime = 0;
	volatile static uint32_t F10_OpeningTime = 0;
	//volatile static uint32_t F10_OpenCount = 0;    // wjjang_200212 전역변수로 이관
	
	if(((error_list_flag[DCU_DLS1_FAULT] == true) && (error_list_flag[DCU_DLS2_FAULT] == true)) && ((error_list_flag[DCU_DCS1_FAULT] == true) || (error_list_flag[DCU_DCS2_FAULT] == true)))
	{
		mmo_DoorClosing(0);
		mmo_DoorFree();
	}
	else if(((error_list_flag[DCU_DCS1_FAULT] == true) && (error_list_flag[DCU_DCS2_FAULT] == true)) && ((error_list_flag[DCU_DLS1_FAULT] == true) || (error_list_flag[DCU_DLS2_FAULT] == true)))
	{
		mmo_DoorClosing(0);
		mmo_DoorFree();
	}
	else
	{
	
		if(mip_Input.di0_OpenCloseButton == true)  //true : open, false : close
		{
			osDelay(500);			//wjjang 200211 orig:1000
			F10_OpenCount++;
			
			if(F10_OpenCount > 2) //jeon_190709 orig:2
			{
				mdc_PreDoorState = DOOR_OPENING;
				mdc_DoorState = DOOR_ERROR;
				
			}
			else
			{
				Erase_fault(DCU_OPEN_FAULT);
				
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSED;
			}
		}
		else
		{
			F10_OpenCount = 0;
			if((mip_Input.di0_DLS1) || (mip_Input.di1_DLS2))
			{
				mmo_DoorClosing(0);
				mmo_DoorFree();
				osDelay(1);																	// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
				mmo_DoorBrake();
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSED;
			}
			else
			{
				mmo_DoorClosing(0);
				mmo_DoorFree();
				osDelay(1);																	// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
				mmo_DoorBrake();
				mdc_PreDoorState = DOOR_OPENING;
				mdc_DoorState = DOOR_OPENED;
			}
		}
	}	
}

void FaultAction_F04_F08(uint8_t PreDoorState)
{
	volatile static _Bool BrakeAction_flag = false;
	volatile static uint32_t f0408closing_time = 0;
	volatile static uint32_t f0408brake_time = 0;

	m_F07Statement = F07_BRAKE;			//초기화하지않으면 닫힘먼저 시작함 오동작하기시작한다
	if(PreDoorState == DOOR_CLOSED)
	{
		if(mip_Input.di0_DLS1 || mip_Input.di1_DLS2)
		{
			m_F0408Statement = F0408_BRAKE;
			mdc_PreDoorState = DOOR_CLOSED;
			mdc_DoorState = DOOR_CLOSED;
			BrakeAction_flag = false;
			error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
			debug("# dls 고장 복귀 ERROR - > CLOSE \r\n");
		}
		else if(mip_Input.di0_OpenCloseButton)
		{
			m_F0408Statement = F0408_BRAKE;
			mdc_PreDoorState = DOOR_CLOSED;
			mdc_DoorState = DOOR_CLOSED;
			BrakeAction_flag = false;
			error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
			debug("# 열림 명령 DLS 고장 복귀 error -> close\r\n");
		}
		#if 0
		else if((mip_Input.di1_DCS1 == false) || (mip_Input.di1_DCS2 == false))
		{
			mmo_DoorClosing(DCU_SLOWMODE_POWER);
		}
		else if(mip_Input.di1_DCS1 || mip_Input.di1_DCS2)
		{
			if(error_list_flag[DCU_ENCODER_ERROR] == false)
			{
				if(mmf_Encoder.Position < DCU_OBS_POSITION)
				{
					if(BrakeAction_flag == false)
					{
						BrakeAction_flag = true;
						mmo_DoorFree();
						osDelay(100);																	// 끝까지 와서 닫혔으면 전류가 거의 흐르지 않으므로 바로 Brake 수행
						mmo_DoorBrake();
					}
				}

			}

		}
		#else
		else if((mip_Input.di0_DLS1 == false) || (mip_Input.di1_DLS2 == false)) //jeon_190617 orig:||
		{ 
			if(m_F0408Statement == F0408_BRAKE)
			{
				if(get_diff_tick(osKernelSysTick() , f0408brake_time) > 500)
				{
					mmo_DoorClosing(DCU_SLOWMODE_POWER);
					m_F0408Statement = F0408_CLOSING;
					f0408closing_time = osKernelSysTick();
				}
			}
			else if(m_F0408Statement == F0408_CLOSING)
			{
				if(get_diff_tick(osKernelSysTick(),f0408closing_time) > 1000)
				{
					f0408brake_time = osKernelSysTick();
					m_F0408Statement = F0408_BRAKE;
					mmo_DoorBrake();
					mdc_PreDoorState = DOOR_CLOSED;
					mdc_DoorState = DOOR_CLOSED;
				}
			}
		}
		#endif
	}
	else if(PreDoorState == DOOR_OPENED)
	{
		BrakeAction_flag = false;
		if(!mip_Input.di0_DLS1 || !mip_Input.di1_DLS2)
		{
			mdc_PreDoorState = DOOR_OPENED;
			mdc_DoorState = DOOR_OPENED;
			debug("# dls 고장 복귀 ERROR - > OPENED \r\n");
		}
		else if(!mip_Input.di0_OpenCloseButton)
		{
			mdc_PreDoorState = DOOR_OPENED;
			mdc_DoorState = DOOR_OPENED;
			debug("# 닫힘 명령 DLS 고장 복귀 error -> Open\r\n");
		}
	}
}

void FaultAction_F05_F06(uint8_t preDoorState)
{
	if(preDoorState == DOOR_CLOSED)
	{
		//DCS가 ON이 되면 상태 복귀
		if(mip_Input.di1_DCS1 || mip_Input.di1_DCS2)
		{
			mdc_PreDoorState = DOOR_CLOSED;
			mdc_DoorState = DOOR_CLOSED;
			error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
			if(mip_Input.di1_DCS1 == true) error_list_flag[DCU_DCS1_FAULT] = false;
			if(mip_Input.di1_DCS2 == true) error_list_flag[DCU_DCS2_FAULT] = false;
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
			debug("# dcs 고장 복귀 ERROR - > CLOSE \r\n");
		}
		else if(mip_Input.di0_OpenCloseButton)
		{
			mdc_PreDoorState = DOOR_CLOSED;
			mdc_DoorState = DOOR_CLOSED;
			error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
			debug("# 열림 명령 DCS 고장 복귀 error -> close\r\n");
		}
	}
	else if(preDoorState == DOOR_OPENED)
	{
		//DCS가 OFF가 되면 상태 복귀
		if(!mip_Input.di1_DCS2 || !mip_Input.di1_DCS1)
		{
			mdc_PreDoorState = DOOR_OPENED;
			mdc_DoorState = DOOR_OPENED;
			if(mip_Input.di1_DCS1 == false) error_list_flag[DCU_DCS1_FAULT] = false;
			if(mip_Input.di1_DCS2 == false) error_list_flag[DCU_DCS2_FAULT] = false;
			debug("# 열림 명령 DCS 고장 복귀 error -> close\r\n");
		}
		else if(!mip_Input.di0_OpenCloseButton)
		{
			mdc_PreDoorState = DOOR_OPENED;
			mdc_DoorState = DOOR_OPENED;
			debug("# dcs 고장 상태에서 열림 명령 open 으로 복귀 \r\n ");

		}
	}
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtDecisionControl.c
*********************************************************************/
