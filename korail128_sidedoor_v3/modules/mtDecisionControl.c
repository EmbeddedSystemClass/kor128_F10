/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtDecisionControl
//!	Generated Date	: ��, 1, 7 2017  
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
_Bool mdc_isInitComplete=false;									// �ʱ�ȭ �Ϸ� �� true
_Bool mdc_isOpeningByObstacle=false;							// ��ְ��� �� true, ��ְ��� 3ȸ �� false�� �����Ǿ� ���ڴ��� �������� �Ǵܼ���
_Bool mdc_FlagTestDODBPS=false;
_Bool mdc_isFirstClosed=false;									// Decision Task���� Closing->Closed�� �Ǵ� ������ ����̱� ������ ���� �÷��׸� ����ؼ� Monitoring Task�� ����ȭ ��Ŵ
_Bool mdc_isFirstOpened=false;							// Decision Task���� Opening->Opened�� �Ǵ� ������ ����̱� ������ ���� �÷��׸� ����ؼ� Monitoring Task�� ����ȭ ��Ŵ 
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
     * �ʱ�ȭ ����
     */
	#if 1
    {
    	/*
		 * �ʱ�ȭ ���� �� Master/Slave DCU �Ǵ�
		 */
		if(m_isMasterSlave == MASTER_DCU)
		{
			debug("4. [Master Start]  Decision Control Task\r\n");
			osDelay(500);																		// Input Task�� ���� ����Ǿ� �Է°��� �˾ƾ� ���� flow�� �����ǹǷ� ���� �� ����
			mmo_DoorFree();																		// ó�� �ʱ�ȭ �� ���� ����
			mmo_ConnectHbridgeGND(true);														// �̰� ����(true�� ��) ���ϸ� ���� ���� �ȵ�
			#ifndef DEBUG_SIMULATOR
				mdc_isFirstClosed = false;
				mdc_isInitComplete = false;
				InitClosing();																	// Power On �� ������ Master DCU�� ���� �ʱ�ȭ
			#endif
		}
		else // SLAVE_DCU
		{
			/*
			 * Slave�� ��ü�� ������ ���� ���
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
			 * ������� �Ѿ���� mram ����ǵ� ������ ������ mram_write_enable �Լ��� ȣ�����
			 * mram �� �����͸� ���� ���� �� �ְ� �Ѵ�.
			 */

			mip_SleveCodeSlaveRun(1);
			m_isTaskExecution = true;
			//osDelay(500);
			mram_write_enable();																// ������� �Ѿ���� mram write �㰡
			ConfigDcuParameters();
			/*
			 * 190311 :DecisionControl�� Monitoring �½�ũ�� �ֱ�?�� ���� �ʾ� ��ֹ� ���� ���ذ� ������ �ȵȻ��¿���
			 * ����͸� �½�ũ�� ����Ǹ� ��ü �� ��ֹ� ���� ������ �ߴ� ���� ���� 19.03.11
			 */
			mip_EmergencyLamp(0);
			mip_MasterCodeMasterLED(1);
			FaultFlag_SET(DCU_SLAVE_RUN);
																		//190311 : decisioncontrol �̿��� ��� �½�ũ ����
			mmo_DoorFree();																		// ó�� �ʱ�ȭ �� ���� ����
			mmo_ConnectHbridgeGND(true);														// �̰� ����(true�� ��) ���ϸ� ���� ���� �ȵ�
			osDelay(200);
			/*
			 * 190312 �������¿��� ��ü �ɰ�� ������ ��ġ�� �� �� �ֱ� ������ ���� ������ �ʱ�ȭ �Ѵ�
			 */
			if(mdc_DoorState == DOOR_INIT)
			{
				InitClosing();
			}
			else if(mdc_DoorState == DOOR_OPENED)
			{
				mmf_Encoder.Position = m_OpeningEncoderPulse; //��ġ�� �˼� �����Ƿ� �� �ʱ�ȭ
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
				InitClosing(); //���� �߻������� ��ü�� ���
			}
			/*
			 * 190311 :DecisionControl�� Monitoring �½�ũ�� �ֱ�?�� ���� �ʾ� ��ֹ� ���� ���ذ� ������ �ȵȻ��¿���
			 * ����͸� �½�ũ�� ����Ǹ� ��ü �� ��ֹ� ���� ������ �ߴ� ���� ���� 19.03.11
			 */

			debug("4. [Slave Start]  Decision Control Task\r\n");													// �̰� ����(true�� ��) ���ϸ� ���� ���� �ȵ�
		}
    }
	#else
    {
    	/*
    	 * Master DCU�� �ʱ�ȭ �� �����Ϸ� ������ ������ɿ� ���� ��� ���� �ݾƾ� ��.
    	 */
		if(m_isMasterSlave == MASTER_DCU)
		{
			debug("4. [Master Start]  Decision Control Task\r\n");
			osDelay(500);																		// Input Task�� ���� ����Ǿ� �Է°��� �˾ƾ� ���� flow�� �����ǹǷ� ���� �� ����
			mmo_DoorFree();																		// ó�� �ʱ�ȭ �� ���� ����
			mmo_ConnectHbridgeGND(true);														// �̰� ����(true�� ��) ���ϸ� ���� ���� �ȵ�
			
			while(!mdc_DoorState)																// �ʱ� DoorState = DOOR_INIT
			{
				if(mip_Input.di0_OpenCloseButton &&												// ������ư�� ������
				   (prevOpenCloseButton != mip_Input.di0_OpenCloseButton))
				{
					mmo_DoorFree();
					mmo_DoorOpening(DCU_SLOWMODE_POWER);														// Slow Mode�� Opening
				}
				else if(!mip_Input.di0_OpenCloseButton &&										// ������ư�� Ǯ����
						(prevOpenCloseButton != mip_Input.di0_OpenCloseButton))
				{
					mmo_DoorFree();
					mmo_DoorClosing(DCU_SLOWMODE_POWER);														// Slow Mode�� Closing
				}
				else
				{
					if(mip_Input.DoorClosedSwitchOn)											// DLS�� ������
					{
						InitClosing();															// Closed �ʱ�ȭ �Ϸ�
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
	 * �ʱ�ȭ �Ϸ� �� ����
	 */
    for(;;)
    {
    	/*
    	 * Place this task in the blocked state until it is time to run again.
    	 */
        osDelayUntil(&PreviousWakeTime, 20UL); //jeon_190717 task �ֱ� 20ms => 25ms
        
 /*       
        if(mdc_PreDoorState != mdc_DoorState)
        {
			printf("Door Pre_State : %d, Door State : %d \r\n", mdc_PreDoorState, mdc_DoorState);
			printf("OBS : %d, Position : %d \r\n", mod_Detect.ObstacleDetectCnt, mmf_Encoder.Position);
		}
*/
    	/*
    	 * Data Input (��������� �����ؼ� �������� �ٷ� ����ϴ� ��� ����� �� ��)
    	 */
    	#if 0
    	{
    		/*
    		 * InputProcessing ���κ��� �Է� ����ġ ���� ��������
    		 */
    		// mip_Input;
    		/*
    		 * MotorFeedback ���κ��� ����/���� �Ϸ� ���� ��������
    		 */
    		// mmf_EndDecision;
    		/*
    		 * ObstacleDetect ���κ��� ��ֹ� ���� ���� ��������
    		 */
    		// mod_Detect;
    		/*
    		 * DataManagement ���κ��� �������� ��������
    		 */
    		// mdm_ErrorFlag;
    	}
    	#endif

    	/*
    	 * Master���� Slave�κ��� ��Ŷ �������� Slave DCU Run�� ���¸� Ȯ���Ͽ����� Master ����
    	 */
    	if((m_isMasterSlave == MASTER_DCU) && m_isSlaveRunCommand)
    	{
    		loopcnt++;
    		if(loopcnt>20)
    		{
    			mmo_DoorFree();
    			mmo_ConnectHbridgeGND(false);			//��ü�� ��쿡�� ���͸� ��������
        		mdc_DoorState = DOOR_ISOLATION;
        		mdc_PreDoorState = DOOR_ISOLATION;
        		mmf_PIDControl = PID_CONTROL_NONE;
    			loopcnt=0;
    			debug("# Master Logic Task ���� ����\r\n");
    		}
    	}
    	/*
    	 * ���� �� Change State to DOOR_ISOLATION
    	 */
    	else if(mip_Input.di0_Isolation == true)										// ������ High ��� ����, �����ϸ� Low ��� ����
    	{
			//debug("# ���ܽ���ġ ���� -> ��������\r\n");
    		mdc_DoorState = DOOR_ISOLATION;
    		mdc_PreDoorState = DOOR_ISOLATION;
			mmf_PIDControl = PID_CONTROL_NONE;											// PID��� ���� ���� ��Ű��
			mmo_DoorFree();																// ���� �� ���͸� Free�� ���� ��
			mram_Event_save(DCU_ISO_EVENT);
			zvr_CloseTime = osKernelSysTick();
    	}
		/*
		 * ���� �̵� �� DLS�� Ǯ���� ���� ������ �������� ����
		 */
    	else if(!mip_Input.di0_ZVR)														// �̵���
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
           			/* ����ǿ� ������¸� closed�� �ٲپ ������ �����ؾ���*/
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
            				HAL_Delay(10); //���� ���� �� 10ms ������ ����
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
        				HAL_Delay(10); //���� ���� �� 10ms ������ ����
        			}
        			
						mdc_DoorState = DOOR_CLOSING;
						mmo_DoorClosing(DCU_SLOWMODE_POWER);
    			}
     		}
    	}
    	else if(mip_Input.di1_EAD == true)
    	{
    		/*
    		 * �ܺ� ����ڵ��� ���������� free���¸� �����Ѵ�
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
				mmf_PIDControl = PID_CONTROL_NONE;											// PID��� ���� ���� ��Ű��
				mmo_DoorFree();																// ���� �� ���͸� Free�� ���� ��
		}
#endif
    	/*
    	 * Change State to OPENED/OPENING/CLOSING/CLOSED
    	 */
    	else
    	{
    		zvr_CloseTime = osKernelSysTick();
    		/*
    		 * �ܺ� ����ڵ��� ���� ��Ű�� free ���·� ������ ���߰� ����ϳ�
    		 * ���ܽ���ġ or ���� ����ڵ��� �Էµǰ� �����ϸ� �ʱ�ȭ ������ �����Ѵ�.
    		 */
    		if(mdc_PreDoorState == DOOR_EAD)
    		{
    			if(mip_Input.di0_Isolation == true)
    			{
    	    		mdc_DoorState = DOOR_ISOLATION;
    	    		mdc_PreDoorState = DOOR_ISOLATION;
    	    		debug("# ���� �Է��� �ܺ� ����ڵ�, ���ܽ���ġ �Է����� ������� ���� \r\n");
    			}
    			else if(mip_Input.di1_EED == true)
    			{
    	    		mdc_DoorState = DOOR_ISOLATION;
    	    		mdc_PreDoorState = DOOR_ISOLATION;
    	    		debug("# ���� �Է��� �ܺ� ����ڵ�, ���κ���ڵ� �Է����� ������� ���� \r\n");
    			}
    			else if(mip_Input.di1_EAD == false)
    			{
    			    		/*
    			    		 * �ܺ� ����ڵ��� ���������� free���¸� �����Ѵ�
    			    		 */
    				if(mip_Input.DoorClosedSwitchOn)
    				{
    						mdc_DoorState = DOOR_ISOLATION;
    						mdc_PreDoorState = DOOR_ISOLATION;
    				}		
    			}
    			    	
    		}
    		/*
    		 * ���ܿ��� ����(���ڴ� ��ġ�� �� �� �����Ƿ�) �� SlowMode�� Opening/Closing ����
    		 */
    	    else if(mdc_PreDoorState == DOOR_ISOLATION)
    		{
    			/*
     			 * ������ư�� ������ ������ Opening �����ؼ� ������ ����� ��
    			 */

				if(mip_Input.di0_OpenCloseButton)
    			{
    				debug("# ���ܽ���ġ ���� -> ����\r\n");

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
    			 * ������ư�� ���� ������ Closing �����ؼ� ������ ����� ��
    			 */
    			else
    			{
    				debug("# ���ܽ���ġ ���� -> ����(�ʱ�ȭ)\r\n");
    				InitClosing();											// Closing ���¿��� ��������ġ�� ���� ���� ���� �����Ϸ�� �Ǵ� ����
     			}
     		}
    		/*
    		 * Normal Closed/Opening/Opened/Closing Operation
    		 */
    		else
    		{
    			/*
    			 * DLS/DCS, ���ڴ�, ������ ���� �����Ϸ� �� �����Ϸ� �Ǵ�
    			 * 
    			 * 1. �����Ϸ�(DLS,DCS�� ���� ����)
    			 * 		1�� - ���ڴ��� ���� �����Ϸ�
    			 * 		2�� - ���ڴ��� ���� �����Ϸ� ���� �� ������ ���� �����Ϸ�
    			 * 		
    			 * 2. �����Ϸ�(DLS�� ���� ��ֹ������� �� ������ �����ϷḦ �� ������ ����)
    			 * 		* DLS1 or DLS2�� �� �� �ϳ� ���� ���
    			 * 		1�� - ���ڴ��� ���� �����Ϸ�
    			 * 		2�� - ���ڴ��� ���� �����Ϸ� ���� �� ������ ���� �����Ϸ�
    			 * 		
    			 * 		* DLS1 and DLS2 �� �� ������ ���� ���
    			 * 		1�� - ���ڴ� 10mm���� ū �Ÿ����� ���� ���� �� ��ֹ� ����
    			 * 		2�� - ���ڴ� 10mm���� ���� �Ÿ����� ������ �������� �����Ϸ� ����
    			 */
    			switch(mdc_DoorState)
    			{
    				case DOOR_CLOSED:
            			/*
            			 * Closed -> ��ư, �������� �Է� -> Opening or Closing
            			 */
        				DecisionOnClosed();
    					break;
    				case DOOR_OPENING:
            			/*
            			 * Opening -> ���ڴ�/������ ���� End �Ǵ� -> Opened
            			 */
        				DecisionOnOpening();
    					break;
    				case DOOR_OPENED:
            			/*
            			 * Opened -> �ð���� -> Closing
            			 */
        				DecisionOnOpened();
    					break;
    				case DOOR_CLOSING:
            			/*
            			 * Closing -> ��ư, ���ڴ�/����/���伾�� ���� -> Opening or Closed
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
	 * ���� ���� ����, ���ܿ��� �������� ���� �� �ʱ�ȭ ����(Vref=200���� ��������, PID���� �̼���)
	 */
	if(mdc_isFirstClosed == false) mdc_isInitComplete = false;
	mmf_Encoder.MaxOpenPosition = m_OpeningEncoderPulse;						// �Է� task �켱������ ���� ������ m_OpeningEncoderPulse ���� ���� ���� ��
	
	/*
	 * Slave (Master�� ��¦ ������ ���ڴ� �������� Slave�� ������ ������� ��)
	 *  - Master�� ���� ���Ͱ� ��¦ �������� 0������ �� ȸ���Ͽ� 9998 ���� �ǰ�
	 *    �ùķ����ͷ� �����Ϸ� �Ǵ���(���ڴ� ���� 100���� ���� ���) ���ϰ�, 
	 *    ���ڴ� ���� 9998���� 100���� ���� ������ ȸ���� ���� �����ϷḦ �Ǵ���
	 *  - Slave ���� ��� ���� �� �ʱ�ȭ �ܰ迡�� ���ڴ� ���� ���½������ ��.
	 *
	 * Master
	 *  - Closing(100)�� �ص� �ٷ� �����Ϸ� �Ǵ��� �����ϳ�, PWM����� ���� �־ ���Ͱ� ��¦ ������
	 *  - ��·���� Master�� �����Ϸ� �Ǵ��� �ϰ�, Closed ���¿��� �ֱ������� ���ڴ� ���� ���½�Ŵ
	 */
	if(m_isMasterSlave == SLAVE_DCU)
	{
		if(mip_Input.DoorClosedSwitchOn == false)
//		if((mip_Input.di0_DLS1 == false) && (mip_Input.di1_DLS2 == false)) //jeon_190709
		{
			mmf_PIDControl = PID_CONTROL_NONE;
			mmo_DoorClosing(DCU_SLOWMODE_POWER);														// Closing �Լ��� ��� ȣ��Ǹ� Disable(Nä�� �� �� Braking �� ��� On��) -> PWM���� -> Enable�� ��� �ݺ�
			mdc_FlagSlowMode = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState = DOOR_CLOSING;
			mdm_time.ClosingStart = osKernelSysTick();						//�ʱ�ȭ ���ۿ��� ���� �ð��� �����ؼ� dls dcs ������ �Ǻ��ؾ���
			mdm_time.ClosingPrintf = 1;
		}
		else
		{
			mdc_isInitComplete = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState 	 = DOOR_CLOSED;
			printf("�����̺� ���� ����ġ ���� ���� ���� �Ϸ� \r\n");
		}
	}
	/*
	 *  �ʱ�ȭ ���� �� dcs ,dls ������ ������ ���� �Ϸ�
	 */
	else if(m_isMasterSlave == MASTER_DCU)
	{
		if(mip_Input.DoorClosedSwitchOn == false)
		{
			mmf_PIDControl = PID_CONTROL_NONE;
			mmo_DoorClosing(DCU_SLOWMODE_POWER);														// Closing �Լ��� ��� ȣ��Ǹ� Disable(Nä�� �� �� Braking �� ��� On��) -> PWM���� -> Enable�� ��� �ݺ�
			mdc_FlagSlowMode = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState = DOOR_CLOSING;
			mdm_time.ClosingStart = osKernelSysTick();						//�ʱ�ȭ ���ۿ��� ���� �ð��� �����ؼ� dls dcs ������ �Ǻ��ؾ���
			mdm_time.ClosingPrintf = 1;
		}
		else
		{
			/* ex) ������ ���� ���� ��� dls ���� �ϳ��� ���� ���� ������� �̴� ������ ���� �� �� �ֵ��� �� decision_onclosed���� 100ms ���Ŀ� brake��*/
			if((mip_Input.di0_DLS1 == false) && (mip_Input.di1_DLS2 == true)) mmo_DoorClosing(DCU_SLOWMODE_POWER);
			else if((mip_Input.di0_DLS1 == true) && (mip_Input.di1_DLS2 == false)) mmo_DoorClosing(DCU_SLOWMODE_POWER);
			mdc_isInitComplete = true;
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState 	 = DOOR_CLOSING; //jeon_190924 orig : DOOR_CLOSED;
			printf("���� ����ġ ���� ���� ���� �Ϸ� \r\n");
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
		debug("# �������¿��� ������ư�� ���� -> ����\r\n");
		#if 0
			OpeningSection.Acceleration = (m_OpeningEncoderPulse*2)/10;		// Closed���� Opening �ϴ� ��� ���ӱ�������
			OpeningSection.Constant = (m_OpeningEncoderPulse*4)/10;			// Closed���� Opening �ϴ� ��� ��ӱ�������
			OpeningSection.Deceleration = m_OpeningEncoderPulse;			// Closed���� Opening �ϴ� ��� ���ӱ�������
		#else
			OpeningSection.Acceleration = OPEN_ACCEL_SECTiON;		// 2s(500)
			OpeningSection.Constant = OPEN_CONST_SECTiON;			// 2s(1250)
			OpeningSection.Deceleration = OPEN_DEACE_SECTiON;		// 2s(3580)
/*JEON_190530
			OpeningSection.Acceleration = (m_OpeningEncoderPulse*3)/10;		// Closed���� Opening �ϴ� ��� ���ӱ�������
			OpeningSection.Constant = (m_OpeningEncoderPulse*5)/10;			// Closed���� Opening �ϴ� ��� ��ӱ�������
			OpeningSection.Deceleration = m_OpeningEncoderPulse;			// Closed���� Opening �ϴ� ��� ���ӱ�������
*/
		#endif
		mdc_DoorState = DOOR_OPENING;
		mmf_PIDControl = PID_CONTROL_OPENING;
	}
	else if(Mode==DOOR_POSITION_CLOSED_BY_BUTTON)
	{
		/*
		 * �����Ÿ� �缳�� - ������ ����
		 */
		CurrEncPos = mmf_Encoder.MaxOpenPosition - mmf_Encoder.PreStopPosition;			// Closing ���� ��ư�� ���� Opening �ϴ� ���
		OpeningSection.Acceleration = ((CurrEncPos*3)/10)+mmf_Encoder.PreStopPosition;	// Closing ���� ��ư�� ���� Opening �ϴ� ��� ���ӱ�������
		OpeningSection.Constant = ((CurrEncPos*5)/10)+mmf_Encoder.PreStopPosition;		// Closing ���� ��ư�� ���� Opening �ϴ� ��� ��ӱ�������
		OpeningSection.Deceleration = CurrEncPos+mmf_Encoder.PreStopPosition;			// Closing ���� ��ư�� ���� Opening �ϴ� ��� ���ӱ�������
		mmf_Encoder.RefVelocity = 0;													// 0���� ����/���/������ �ǰ� Closing �� ������ RefVelocity ���� 0���� ����
		
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
	
	mmf_PIDControl = PID_CONTROL_NONE;												// PID��� ���� ���� ��Ű��
	mmf_NextMotorDirSelect=PID_CONTROL_OPENING;
	setPIDPWMOut();
	mmo_DoorFree();																	// ���� �Ϸ� �� ���͸� Free�� ���� ��
	if(Mode==DOOR_POSITION_CLOSED_BY_END)												// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
	{
		osDelay(1);
		mmo_DoorBrake();
	}
	else if(Mode==DOOR_POSITION_CLOSED_BY_BUTTON)
	{
		/*
		 * �� ������ ������ ��ֹ� 3ȸ ���� �� �������� �� �����ϰ� ��� ����� ����
		 *  : ��ֹ� 3ȸ° ���� �� �ٷ� ������ �ϴµ� 500ms �������� ��� ����� ����
		 *  : ��ֹ� ������ ���� ���� Closing ���� ��ư���� Openging �� �� �ε巴�� ���� ���� �ð����� �߰�
		 */
		if(mod_Detect.ObstacleDetectCnt==0)
		{
			/*
			 * Closing �ϴ� Free�� �ٷ� Opening �ϸ� ������������ ���̿��� �����Ƿ� ������ �ٿ��� ��.
			 * : Closing �ϴٰ� Free�� ���� �� �ٷ� Free�� ������ 
			 *   PWM2�� �ɸ��� ������ ������ PWM1 FET�� �귯  PWM1�� �ִ� ���̿��� ����.
			 *   ���� Free�� ���� �� PWM�� ���ڱ� 0���� ������ �ȵǰ�, ������ ���̴ٰ� 0�̵Ǹ� Free�� ���� ��
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
		 * ��ֹ� ���� �� �����ָ� ��ֹ� ������ ������ �ѹ��� �ȿ����� �̰� ������ ���� ����
		 */
		//osDelay(300);																// ��ֹ� ���� �ÿ��� ���� ���� �ٷ� Opening ����
	}
	
	mmf_Encoder.PreStopPosition = mmf_Encoder.Position;
	mmf_Encoder.tmpClosedPosition = mmf_Encoder.Position;
	mmf_Bemf.RefPosition = mmf_Bemf.Position;
	
	if(mdc_DoorState==DOOR_CLOSED)
	{
		mmf_Encoder.MaxClosePosition = mmf_Encoder.Position;
		mmo_MotorPositionReset();
		mmf_Encoder.Position = 0;													// Free �� ���Ͱ� �� ������ �� �����Ƿ� ������ ������� ���ڴ� ��
		mmf_Bemf.Position=(float)0;
		mod_Detect.ObstacleDetectCnt = 0;											// ���� �Ϸ��ϸ� ��ֹ����� Ƚ�� ����
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
		osDelay(100);											//�����Ϸ� �Ǵ��� �Ǹ� 100ms �а� ���� ���ߴ� ���� (���� ���¿��� ������ ���� Ű�� DLS ������ ���� ��찡 ����)
		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_END);
		mdc_PreDoorState=DOOR_CLOSED;
		mdc_isFirstClosed = true;
		mdc_isInitComplete = true;
		FlagDODBPS=false; //jeon_190514
		/*
		 * ����(Closing)�ð� ���
		 */
		mdm_time.Closed = osKernelSysTick();								
		mdm_time.Closing = mdm_time.Closed - mdm_time.ClosingStart;
		debug("## �����Ϸ�: �����ð�(%d), ������ġ(%d)\r\n",
				(int)mdm_time.Closing, (int)mmf_Encoder.tmpClosedPosition);
	}
	/*
	 * Closed -> Opening by OpenClose Button Push
	 */
	/*
	 * Closed -> Closed after OpenClose Button Release										// �����Ϸᰡ �Ǹ� �ʱ�ȭ �Ϸ�� SlowMode ����� �ʿ� ����
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
				 * ����(Opening) ���۽ð� ����
				 */
				mdm_time.OpeningStart = osKernelSysTick();										// Closed ���¿��� ������ư�� ���� Opening ����
				mdm_time.OpeningPrintf = 1;
			}
			else
			{
				StateSetToOpeningAtClosed(DOOR_POSITION_CLOSED_BY_END);
				/*
				 * ����(Opening) ���۽ð� ����
				 */
				mdm_time.OpeningStart = osKernelSysTick();										// Closed ���¿��� ������ư�� ���� Opening ����
				mdm_time.OpeningPrintf = 1;
			}
		}
		else
		{
			if(mip_Input.di0_OpenCloseButton)
			{
				StateSetToOpeningAtClosed(DOOR_POSITION_CLOSED_BY_END);

				/*
				 * ����(Opening) ���۽ð� ����
				 */
				mdm_time.OpeningStart = osKernelSysTick();										// Closed ���¿��� ������ư�� ���� Opening ����
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
				 * Closed(brake)���� ��� �����̸� ���ڴ� ���� Ʋ�����Ƿ� �ֱ������� ���ڴ� �� ����
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
					mmf_Encoder.Position = 0;													// Free �� ���Ͱ� �� ������ �� �����Ƿ� ������ ������� ���ڴ� ��
				}
				else if(loopcnt>50)															//1����
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
		 * ������ ������ư�� Ǯ�� ���� (��ֹ� �̰��� �ÿ��� ����)
		 * ��ֹ� ���� ������ ������ ������ư�� Ǭ ���¿��� ��ֹ� ������ Opening ���°� �Ǹ�ٷ� ���������� ������ 
		 */
		if(mip_Input.CommandTestMode || mip_Input.CommandOpen)
		{
			//No Action
		}
		else if(!mip_Input.di0_OpenCloseButton && (mod_Detect.ObstacleDetectCnt==0) && (!mip_Input.di0_ReOpen))								// jjkim 190104 : reopen ��ư�� ������������ ��� opening ���¸� ���� �ؾ���
		{
			debug("# ������:������ư�� Ǯ�� -> ���� �� ����\r\n");
			mdm_time.ClosingStart = osKernelSysTick();
			if(mdc_isInitComplete == true)
			{
				/*
				 * Opening -> Opened
				 */
				StateSetToOpenedAtOpening(DOOR_POSITION_OPENED_BY_BUTTON);	// ���� �� ������ư�� Ǯ�� �߰��������� Opened
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_BUTTON);
				return;														// ���� �� ���� �� �����Ϸ�(Opening->Opened) �Ǵ� ������ �ʿ� ����
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
		 * �ڵ� �����Ϸ� ����ġ Off
		 */
		#ifdef DEBUG_SIMULATOR
			/*
			 * ��������ġ�� ���� ���¿��� ���ڴ��� 0->50 ��ŭ �̵��ϸ� �ùķ����ͷ� ��������ġ�� Ǯ����
			 */
			if((mip_Input.DoorClosedSwitchOn) && (mmf_Encoder.Position>100))
			{
				mip_Input.di0_DLS1 = false;
				mip_Input.di1_DLS2 = false;
				mip_Input.di1_DCS1 = false;
				mip_Input.di1_DCS2 = false;
				mip_Input.DoorClosedSwitchOn = false;
				debug("# �ùķ��̼Ǹ�忡�� ������ ��������ġ Release\r\n");
			}
		#endif
		
		/*
		 * Opening -> Opened by Encoder, Current, ��ְ���
		 */
		#if 1
		{
			/*
			 * ���ڴ��� ���ߴ� ��� -> ���� ���� ���ڴ��� ������ �õ��ؾ� ��
			 */
			if(mmf_EndDecision.OpenedByPosition)
			{
				if(mdc_isInitComplete == true)
				{
					if(mod_Detect.ObstacleDetectCnt > mod_Detect.ObstacleConfigValue)
					{
						debug("# ������:��ְ��� -> �����Ϸ� (%d)\r\n", mod_Detect.ObstacleDetectCnt);
					}
					else
					{
						debug("# ������:���ڴ��� -> �����Ϸ� �Ǵ�\r\n");
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
					// �ʱ�ȭ ������ ���ڴ����� �𸣹Ƿ� ���ڴ��� �����Ϸ� �Ǵ� �Ұ� -> No Action
				}
			}
			/*
			 * ������ ���ߴ� ��� -> �ʱ�ȭ�� ������� ������ �����Ϸᰡ �ǴܵǸ� ������ �׻� �������Ѿ� ��
			 */
			else if(mmf_EndDecision.OpenedByCurrent)
			{
				mmf_EndDecision.OpenedByCurrent = false;
				debug("# ������:������ -> �����Ϸ� �Ǵ�\r\n");
				/*
				 * [�߿�] B_emf���� �ʱ�ȭ ��/�� �����ϰ� �������¸� �����ϸ� �ٷ� ����� ��
				 * 
				 * ���� ���� ���¿����� B_emf������ ��� ���� ��
				 * -> B_emf�� ���� �ӵ����� �߻�, ��ġ�� ��� ������
				 * -> �ӵ�0�� �����ð� �����Ǵ� �������� Closed ���¸� ���� �� ����
				 * -> ��� Closing �������� FET�� Turn On ��Ű�� ����
				 * 
				 * -> �����Ϸᰡ �ȵ� ���¿��� Open������� Opening ������ �����ϸ�
				 * -> FET ��Ʈ�� �߻��� MCU Reset�Ǵ� ������ �߻��Ǵ� ������ ������
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
				/*Opening ���� ��� ����*/

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
		ClosingSection.Acceleration = (m_OpeningEncoderPulse*8)/10;		// Opened���� Closing �ϴ� ��� ���ӱ��� ����
		ClosingSection.Constant = (m_OpeningEncoderPulse*6)/10;			// Opened���� Closing �ϴ� ��� ��ӱ��� ����
		ClosingSection.Deceleration = 0;								// Opened���� Closing �ϴ� ��� ���ӱ��� ����
*/
		mdc_DoorState = DOOR_CLOSING;
		mdc_PreDoorState = DOOR_CLOSING;
		mmf_PIDControl = PID_CONTROL_CLOSING;
	}
	else if(Mode==DOOR_POSITION_OPENED_BY_BUTTON)
	{
		CurrEncPos = mmf_Encoder.Position;
		ClosingSection.Acceleration = (CurrEncPos*7)/10;			// Opening ���� ��ư�� Ǯ�� Closing �ϴ� ��� ���ӱ��� ����
		ClosingSection.Constant = (CurrEncPos*5)/10;				// Opening ���� ��ư�� Ǯ�� Closing �ϴ� ��� ��ӱ��� ����
		ClosingSection.Deceleration = 0;							// Opening ���� ��ư�� Ǯ�� Closing �ϴ� ��� ���ӱ��� ����
		mmf_Encoder.RefVelocity = 0;								// �ٽ� PWM 0���� ����/���/������ �ǰ� Opening �� ������ RefVelocity ���� 0���� ����
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
	
	mmf_PIDControl = PID_CONTROL_NONE;												// PID��� ���� ���� ��Ű��
	mmf_NextMotorDirSelect=PID_CONTROL_CLOSING;
	setPIDPWMOut();
	mmo_DoorFree();																	// ���͸� Free�� ���� ��
	if(Mode==DOOR_POSITION_OPENED_BY_END)												// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
	{
		osDelay(1);
		mmo_DoorBrake();
	}
	else if(Mode==DOOR_POSITION_OPENED_BY_BUTTON)
	{
		/*
		 * Free �� ��� ������ ������ ��ٸ� �� Opening ������ �����ؾ� ��
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
	 * MiddleOpened(��ֹ��ο����Ϸ�)�� �ƴ� FullOpened(���ڴ�/������ �����Ϸ�) ���¿����� �ִ�Ÿ� ����
	 */
	/*
	 * ��ֹ� ���� ��������
	 */
	if(mod_Detect.ObstacleDetectCnt==0)
	{
		mmf_Encoder.MaxOpenPosition = mmf_Encoder.Position;
	}
	/*
	 * ��ֹ� 3ȸ ���� �� ��������
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
		 * ����(Opening)�ð� ���
		 */
		mdm_time.Opened = osKernelSysTick();
		mdm_time.Opening = mdm_time.Opened - mdm_time.OpeningStart;
		debug("## �����Ϸ�: �����ð�(%d), ������ġ(%d)\r\n",
					(int)mdm_time.Opening, (int)mmf_Encoder.Position);
		if(mmf_EndDecision.OpenedByPosition == true) mmf_EndDecision.OpenedByPosition = false;
		#ifdef DEBUG_SIMULATOR	// Opening -> Opened: ���� ������ 0���� ����
			/*
			 * �ùķ��̼ǿ��� Open �Ϸ�Ǿ� ������Ű�� �������� 0���� �������� ��.
			 */
			m_adc.MotorCurrent = 0;
		#endif
	}
	/*
	 * Opened -> Closing by OpenClose Button Release, ��ְ���
	 */
	/*
	 * Opened -> Opened after OpenClose Button Push, ��ְ���
	 */
	else
	{
		/*
		 * Opened -> Closing : ���� �������¿��� ������ư�� Ǯ�� �������� ����
		 */
		if(mip_Input.CommandOpen == true)
		{
			if(mip_Input.CommandClose == true)
			{
				mip_Input.CommandOpen = false; // �� �ʱ�ȭ
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
			 *  closing ���� reopen ��ư�� ������ open �Ϸᰡ �� ��쿡�� reopen ��ư�� �� ��� ���� ������ �����ؾ��Ѵ�.
			 */
			if(mdc_isInitComplete == true)
			{
				debug("# ������ ReOpen��ư�� Ǯ�� -> ���� (�ʱ�ȭ ��)\r\n");
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
			}
			else
			{
				debug("# ������ ReOpen��ư�� Ǯ�� -> ���� (�ʱ�ȭ ��)\r\n");
				mdc_DoorState = DOOR_CLOSING;
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorClosing(DCU_SLOWMODE_POWER);
			}
			/*
			 * ����(Closing) ���۽ð� ����
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
		 * Opened -> Closing : ��ֹ�3ȸ ������ ���� �������¿��� ������ư �Ǵ� �翭��(ReOpen)��ư���� �������� ����
		 * ��ֹ� 3ȸ �����ϰ� ���� �� �ٽ� �����ϰ� ���� ��  ��ֹ� �����ϸ� �̵��� ����
		 */
		else if(mod_Detect.ObstacleDetectCnt >= mod_Detect.ObstacleConfigValue)
		{
			if(!mip_Input.di0_OpenCloseButton && PreOpenCloseButton)				// ������ư�� ������ �� ���
			{
				debug("# ��ֹ� ������ ���������� ������ư�� Ǯ�� -> ���� (�ʱ�ȭ ��)\r\n");
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
				debug("# ��ֹ� ������ ���������� �翭�� ��ư�� ���� -> ���� (�ʱ�ȭ ��)\r\n");
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
				if(debugprint.Data_Print_flag == true)
				{
					debug("Opened -> Closing Start \r\n");
					debug("Current\t Encoder\t Close/Open\t DLS1\t DLS2\t DCS1\t DCS2\t\r\n");
					debug("[%d]\t[%d]\t\t[%d]\t\t[%d]\t[%d]\t[%d]\t[%d]\r\n", m_adc.MotorCurrent, mmf_Encoder.Position, mip_Input.di0_OpenCloseButton, mip_Input.di0_DLS1, mip_Input.di1_DLS2, mip_Input.di1_DCS1, mip_Input.di1_DCS2);
				}
				#ifdef DEBUG_SIMULATOR	// Opened -> Closing: �翭����ư �Է� ����
					mip_Input.di0_ReOpen = false;
					debug("# �ùķ����ͷ� Reopen �Է� ����\r\n");
				#endif
			}
			/*
			 * ����(Closing) ���۽ð� ����
			 */
			mdm_time.ClosingStart = osKernelSysTick();
			mdm_time.ClosingPrintf = 1u;
		}
		else if(!mip_Input.di0_OpenCloseButton && (mod_Detect.ObstacleDetectCnt < mod_Detect.ObstacleConfigValue) && !mip_Input.di0_ReOpen)
		{
			if(mdc_isInitComplete == true)
			{
				debug("# ������ ������ư�� Ǯ�� -> ���� (�ʱ�ȭ ��)\r\n");
//				printf("OBS : %d, Position : %d \r\n", mod_Detect.ObstacleDetectCnt, mmf_Encoder.Position);
				StateSetToClosingAtOpened(DOOR_POSITION_OPENED_BY_END);
			}
			else
			{
				debug("# ������ ������ư�� Ǯ�� -> ���� (�ʱ�ȭ ��)\r\n");
				mdc_DoorState = DOOR_CLOSING;
				mmf_PIDControl = PID_CONTROL_NONE;
				mmo_DoorClosing(DCU_SLOWMODE_POWER);
			}
			/*
			 * ����(Closing) ���۽ð� ����
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
		 * Opened -> Opened after OpenClose Button Push, ��ְ���
		 */
		else
		{


			// No Action
			#ifdef DEBUG_SIMULATOR	// Opened -> Opened: �翭����ư �Է� ����
				if(mip_Input.di0_ReOpen)
				{
					mip_Input.di0_ReOpen = false;
					debug("# �ùķ����ͷ� Reopen �Է� ����\r\n");
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
		 * �����߿� �����������α׷��� ���� ��ư�� ������ ���������� �������� ����
		 */
		if((mip_Input.CommandTestMode == true) || (mip_Input.CommandClose == true))
		{
			/*
			 * 190311 : �����������α׷����� ��ֹ� ���� ��� �߰�
			 */
			if(mdc_isOpeningByObstacle)
			{
				/*
				 * DODBPS �Է��� ���� �� ��ְ����� ����
				 */
				if(!mip_Input.di0_Bypass)
				{
					debug("# ������:��ֹ� ���� 1-> ����\r\n");
//					printf("OBS : %d, Position : %d \r\n", mod_Detect.ObstacleDetectCnt, mmf_Encoder.Position);
					FlagDODBPS = false;												// ���� dod������ dodbps ��ư�� Ǯ��� �ش� �÷��״� �ʱ�ȭ ���Ѿ� �Ѵ�.
					FlagClosingByDODBPS = false;
					mdc_PreDoorState = DOOR_CLOSING;
					mdc_DoorState = DOOR_OBSTACLE;
					StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_OBSTACLE);	// ��ֹ� ���� �� �ٽ� ���� ���� �߰��������� Closed
					mdm_time.OpeningStart = osKernelSysTick();						// Closing ���¿��� ��ֹ� ������ Opening ����
					return;															// ���� �� ���� �� �����Ϸ� �Ǵ� ������ �ʿ� ����
				}
				/*
				 * DODBPS �Է��� ���� �� DODBPS Closing ����
				 */
				else
				{
					mdc_isOpeningByObstacle = false;
//jeon_190514					mod_Detect.ObstacleDetectCnt = 0;								//dodbps���ۿ����� ��ֹ� ���� ���� �ʿ� ����
					mod_Detect.ObstacleDetectCnt = 1;								//dodbps���ۿ����� ��ֹ� ���� ���� �ʿ� ����
					if(FlagDODBPS == false)
					{
						dodbpsClosingStartTime = osKernelSysTick();
						FlagDODBPS = true;												// �����Ϸ�(DLS,DCS����)�� �Ǹ� Ŭ����
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
				mdm_time.OpeningStart = osKernelSysTick();					// Closing ���¿��� ������ư�� ���� Opening ����
				/*
				 *  dodbps ������ ���� ��� �Ŀ� �ٽ� ���� ����� �ָ� dodbps ������ �����ϰ� �� ����.
				 *  ���� ����� ������ ������ �ʱ�ȭ ��Ų��.
				 */
				FlagDODBPS = false;
				mdc_isOpeningByObstacle = false;
				mod_Detect.ObstacleDetectCnt = 0;
				
				if (mip_Input.di0_OpenCloseButton)
				{
					debug("# ������:������ư�� ���� -> ����\r\n");
				}
				else
				{
					debug("# ���� �� Reopen ��ư�� ���� - > ���� \r\n");
				}

				if(mdc_isInitComplete == true)
				{
					/*
					 * Closing -> Closed
					 */
					StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);	// �������� ������ư�� ���� �ٽ� �������� �߰��������� Closed
					StateSetToOpeningAtClosed(DOOR_POSITION_CLOSED_BY_BUTTON);
					return;																			// ���� �� ���� �� �����Ϸ� �Ǵ� ������ �ʿ� ����
				}
				else
				{
					mdc_DoorState = DOOR_OPENING;
					mmf_PIDControl = PID_CONTROL_NONE;
					mmo_DoorOpening(DCU_SLOWMODE_POWER);
				}
			}
			/*
			 * ������ - ��ֹ�����
			 * DODBPS ���� ���� �� ��ֹ� ���� �� dodbpsClosingStartTime �� ��� ���� �� �ֳ� �ϸ�
			 * ��ֹ����� �������� mdc_isOpeningByObstacle�� �� ��Ŵ
			 */
			else if(mdc_isOpeningByObstacle)
			{
				/*
				 * DODBPS �Է��� ���� �� ��ְ����� ����
				 */
				if(!mip_Input.di0_Bypass)
				{
					debug("# ������:��ֹ� ���� 2-> ����\r\n");
					FlagDODBPS = false;												// ���� dod������ dodbps ��ư�� Ǯ��� �ش� �÷��״� �ʱ�ȭ ���Ѿ� �Ѵ�.
					FlagClosingByDODBPS = false;
					mdc_PreDoorState = DOOR_CLOSING;
					mdc_DoorState = DOOR_OBSTACLE;
					StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_OBSTACLE);	// ��ֹ� ���� �� �ٽ� ���� ���� �߰��������� Closed
					mdm_time.OpeningStart = osKernelSysTick();						// Closing ���¿��� ��ֹ� ������ Opening ����
					return;															// ���� �� ���� �� �����Ϸ� �Ǵ� ������ �ʿ� ����
				}
				/*
				 * DODBPS �Է��� ���� �� DODBPS Closing ����
				 */
				else
				{
					debug("# DODBPS ������:��ֹ� ����\r\n");
					mdc_isOpeningByObstacle = false;
					mod_Detect.ObstacleDetectCnt = 0;								//dodbps���ۿ����� ��ֹ� ���� ���� �ʿ� ����
					if(FlagDODBPS == false)
					{
						dodbpsClosingStartTime = osKernelSysTick();
						FlagDODBPS = true;												// �����Ϸ�(DLS,DCS����)�� �Ǹ� Ŭ����
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
		 * �ڵ� �����Ϸ� ����ġ On
		 */
		#ifdef DEBUG_SIMULATOR
			/*
			 * ��������ġ�� ������ ���� ���¿��� ���ڴ��� 3000->50 ��ŭ �̵��ϸ� �ùķ����ͷ� ��������ġ�� ���� ��
			 * 
			 * ��ġ�� ���ǿ� �߰����� ������ Closing ���� ������ư�� ���� Opening �� �� ���ڴ� ���� �߻�
			 * 	1. Closing���� ������ư Push
			 * 	2. Closed ���°� �Ǿ� ��������ġ true�� ����
			 * 	3. Opening->Opened �� ��������ġ�� true�� ���¿��� �����ϷḦ �Ǵ������Ƿ� ���ڴ� ���� �߻�
			 */
			if((!mip_Input.DoorClosedSwitchOn) && (mmf_Encoder.Position<100) && !mdc_FlagTestDODBPS)
			{
				mip_Input.di0_DLS1 = true;
				mip_Input.di1_DLS2 = true;
				mip_Input.di1_DCS1 = true;
				mip_Input.di1_DCS2 = true;
				mip_Input.DoorClosedSwitchOn = true;
				debug("# �ùķ��̼Ǹ�忡�� ������ ��������ġ Push\r\n");
			}
		#endif
		
		/*
		 * Closing -> Closed by Encoder or Current
		 */
		#if 1
		{
			/*
			 * DLS1 or DLS2�� �� �� �ϳ� ������ �����Ϸ� �Ǵ� (�����̹Ƿ� ������ �� �� �ϳ��� ������ ��)
			 */
			if(mip_Input.DoorClosedSwitchOn)
			{
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSED;
				
				FlagDODBPS = false; //jeon_190514

				/*
				 * ���ڴ��� ���ߴ� ��� -> ���� ���� ���ڴ��� ������ �õ��ؾ� ��
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
						 * ���������� �����Ǿ PID���� ������ �ӵ��� ���ڱ� 0���� ����� �ȵǰ�, �����ð� ���ڴ���ȭ�� ������ �����Ϸ� �Ǵ�
						 */
						debug("# ������:���ڴ��� -> �����Ϸ� �Ǵ�\r\n");
						/*
						 * �ʱ�ȭ �� Closed ������ �����ǰ�, �����ð� ���ڴ���ȭ�� ������ �����Ϸ� �Ǵ�
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
						// �ʱ�ȭ ������ ���ڴ����� �𸣹Ƿ� ���ڴ��� �����Ϸ� �Ǵ� �Ұ� -> No Action
					}
				}
				/*
				 * ������ ���ߴ� ��� -> �ʱ�ȭ�� ������� ������ �����Ϸᰡ �ǴܵǸ� ������ �׻� �������Ѿ� ��
				 */
				else if(mmf_EndDecision.ClosedByCurrent)
				{
					mmf_EndDecision.ClosedByCurrent = false;
					debug("# ������:������ -> �����Ϸ� �Ǵ�\r\n");
					/*
					 * [�߿�] B_emf���� �ʱ�ȭ ��/�� �����ϰ� �������¸� �����ϸ� �ٷ� ����� ��
					 * 
					 * ���� ���� ���¿����� B_emf������ ��� ���� ��
					 * -> B_emf�� ���� �ӵ����� �߻�, ��ġ�� ��� ������
					 * -> �ӵ�0�� �����ð� �����Ǵ� �������� Closed ���¸� ���� �� ����
					 * -> ��� Closing �������� FET�� Turn On ��Ű�� ����
					 * 
					 * -> �����Ϸᰡ �ȵ� ���¿��� Open������� Opening ������ �����ϸ�
					 * -> FET ��Ʈ�� �߻��� MCU Reset�Ǵ� ������ �߻��Ǵ� ������ ������
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
			 * DLS1 or DLS2�� �� �� ������ ������ Closing -> Closing
			 */
			else
			{
				/*
				 * DLS 2���� ������ ������ ���� �ð� 5�ʵ� , ���ڴ� ��ġ, ������ �Ǵ����� �����ϷḦ �����Ѵ�.
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
								debug("DCS �Է� Ȯ�� �Ұ�  ���� ������ �����Ϸ� Ȯ�� \r\n");
							}
							if(mmf_EndDecision.ClosedByPosition == true)
							{
								mmf_EndDecision.ClosedByPosition = false;
								debug("DCS �Է� Ȯ�� �Ұ�  ��ġ ������ �����Ϸ� Ȯ�� \r\n");
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
					if(get_diff_tick(osKernelSysTick(),mdm_time.ClosingStart) > 6000) //orig:5000, jeon_190709					//korail128�� ������ dcs�� ���� �� �ʿ� �ֱ� ������  6�ʵڿ� ���� �Ϸ� �Ǵ��� ����
					{
						if((mmf_EndDecision.ClosedByCurrent == true) || (mmf_EndDecision.ClosedByPosition == true))
						{
							mdc_PreDoorState = DOOR_CLOSING;
							mdc_DoorState = DOOR_CLOSED;
							if(mmf_EndDecision.ClosedByCurrent == true)
							{
								mmf_EndDecision.ClosedByCurrent = false;
								debug("DLS �Է� Ȯ�� �Ұ�  ���� ������ �����Ϸ� Ȯ�� \r\n");
							}
							if(mmf_EndDecision.ClosedByPosition == true)
							{
								mmf_EndDecision.ClosedByPosition = false;
								debug("DlS �Է� Ȯ�� �Ұ�  ��ġ ������ �����Ϸ� Ȯ�� \r\n");
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
				 * DODBPS Closing : 8�� Closing, 3�� Free �ݺ�
				 * ��ֹ� ���� ��(200N~300N) �� �ȸ��� ��� mmo_DoorClosing(200); �� �����ϸ� �� ��
				 */
				if(FlagDODBPS)
				{
					/*
					 * dodbps ���� ���¿��� ��ֹ� ������ ���ڴ� ������ �ߴµ�. dodbps�����߿��� ����-���ڴ� ������ ���������ʵ��� ����
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
		    			mmf_PIDControl = PID_CONTROL_NONE;											// PID��� ���� ���� ��Ű��
//jeon_190514						mmo_DoorFree();
						debug("# dodbps closing ���� �ǽ� : %d\r\n", mod_Detect.ObstacleDetectCnt);
					}
					else if(FlagFreeByDODBPS)
					{
						FlagFreeByDODBPS = false;
		    			mmf_PIDControl = PID_CONTROL_NONE;											// PID��� ���� ���� ��Ű��
						mmo_DoorClosing(0);
						mmo_DoorFree();
						debug("# dodbps free ���� �ǽ� \r\n");
					}
					else
					{
						/*
						 * 8~11s Free
						 */
						/*
						 * DODBPS Closing : 8�� Closing, 3�� Free �ݺ�
						 * todo <<jjkim_181219>> : DODBPS ���� Ȯ�� [3]
						 * comment : DODBPS ���� Ȯ�� [1]�� ������ dodbpsClosingStartTime�� ��� ���� �Ǵµ� �ð� ����� ���� ������ �ȵ� �� ����
						 */
						dodbpsClosingTime = osKernelSysTick();
						if(get_diff_tick(dodbpsClosingTime,dodbpsClosingStartTime) > DCU_DODBPS_CLOSINGTIME)              //free ���� �� �ٽ� Closing ���� �ǽ�
						{
//							FlagClosingByDODBPS = true;
							FlagReClosingByDODBPS = true;
							dodbpsClosingStartTime = osKernelSysTick();
							
							mmf_PIDControl = PID_CONTROL_NONE;
							mmo_DoorClosing(DCU_SLOWMODE_POWER);
						}
						else if(get_diff_tick(dodbpsClosingTime,dodbpsClosingStartTime) > DCU_DODBPS_FREETIME)			//8�ʵ� free���� �ǽ�
						{
							FlagFreeByDODBPS = true;
							
//							mod_Detect.ObstacleDetectCnt = 0; //jeon_190514
//							m_CurrentPWMDuty = 200; //jeon_190514
						}
					}
				}
				/*
				 * Normal Closing ���� ��� ����
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
 *  ���� ������ �� ����
 */
static void DecisionOnError(void)
{
	/*
	 *  ���� ���� ������ ������¸� ���� ���� ������ ���� �Ѵ�.
	 */
	switch (mdc_PreDoorState)
	{
		case DOOR_CLOSED:
			if(error_list_flag[DCU_HARD_FAULT] == true)		mmo_DoorFree();
			/*
			 *  DCS�� OFF�� ������ �� ����
			 *   : Closed ���¿��� DCS�� ������ �Ǵµ� ������ ���� ���·� DCS����
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
	 *  ���� ����̳� dodbps ��ȣ, ��ֹ� ���� 3ȸ �̻��� ���
	 */
	if((mip_Input.di0_OpenCloseButton) || (mod_Detect.ObstacleDetectCnt >= mod_Detect.ObstacleConfigValue))
	{
		if(mod_Detect.ObstacleDetectCnt >= mod_Detect.ObstacleConfigValue)
		{
			debug("��ֹ� %dȸ ������ ���� ���� ���� \r\n",mod_Detect.ObstacleDetectCnt);
		}
		else
		{
			debug("���� ��ư �Է����� ���� ���� ���� ��ֹ� ī��Ʈ �� �ʱ�ȭ \r\n");
			mod_Detect.ObstacleDetectCnt = 0;
		}
		ObstacleState = OBS_NONE;
		mdc_DoorState = DOOR_OPENING;
		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
		
		CurrEncPos = mmf_Encoder.MaxOpenPosition - mmf_Encoder.PreStopPosition;			// Closing ���� ��ֹ� ������ Opening �ϴ� ���
		OpeningSection.Acceleration = ((CurrEncPos*2)/10)+mmf_Encoder.PreStopPosition;	// Closing ���� ��ֹ� ������ Opening �ϴ� ��� ���ӱ�������
		OpeningSection.Constant = ((CurrEncPos*4)/10)+mmf_Encoder.PreStopPosition;		// Closing ���� ��ֹ� ������ Opening �ϴ� ��� ��ӱ�������
		OpeningSection.Deceleration = CurrEncPos+mmf_Encoder.PreStopPosition;			// Closing ���� ��ֹ� ������ Opening �ϴ� ��� ���ӱ�������
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
/* ���� �ʱ� ���� �� ������ �ȵ��� ��� - �����*/				
//jeon_191002				mmo_DoorOpening(DCU_SLOWMODE_POWER);

/* ���� �ʱ� ���� �� ������ �� ��� */				
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
			 * todo : ���� ���尡�� Ȯ�� ��, ���� ���� �����ϴٸ� +200 �κ��� ���� �ϸ� ��
			 */
#if 0 //jeon_190709
			if((mmf_Encoder.Position > (mmf_Encoder.PreStopPosition+138)) &&
			   (mmf_Encoder.Position < 9000))
			{
				debug("# ��� %dȸ ���� -> ��ֹ� �����Ϸ� �Ǵ�\r\n", mod_Detect.ObstacleDetectCnt);
				ObstacleState = OBS_OPEN;
				mmo_DoorBrake();
				/*
				 * todo : ������ ���� Ȯ���϶�� �ؼ� osKernelSysTick()�� �ؼ� �������ϸ� HAL_GetTick() ������� ����
				 */
				m_ObstacleOpenedTime = HAL_GetTick();
			}
#else
			if(mmf_Encoder.Position > 4600)
			{
				if((mmf_Encoder.Position > (mmf_Encoder.PreStopPosition+138)) &&
							   (mmf_Encoder.Position < 9000))
				{
					debug("# ��� %dȸ ���� -> ��ֹ� �����Ϸ� �Ǵ�\r\n", mod_Detect.ObstacleDetectCnt);
					ObstacleState = OBS_OPEN;
					mmo_DoorBrake();
					/*
					 * todo : ������ ���� Ȯ���϶�� �ؼ� osKernelSysTick()�� �ؼ� �������ϸ� HAL_GetTick() ������� ����
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
				debug("# ��ֹ� �����Ϸ� �Ǵ�\r\n");
				ObstacleState = OBS_OPEN;
				mmo_DoorBrake();
			}
			else
			{
				if((mmf_Encoder.Position > (mmf_Encoder.PreStopPosition+138)) &&
											   (mmf_Encoder.Position < 9000))
				{
					debug("# ��� %dȸ ���� -> ��ֹ� �����Ϸ� �Ǵ�\r\n", mod_Detect.ObstacleDetectCnt);
					ObstacleState = OBS_OPEN;
					mmo_DoorBrake();
					/*
					 * todo : ������ ���� Ȯ���϶�� �ؼ� osKernelSysTick()�� �ؼ� �������ϸ� HAL_GetTick() ������� ����
					 */
					m_ObstacleOpenedTime = HAL_GetTick();
				}
			}
		}
		else if(ObstacleState == OBS_OPEN)
		{
			/*
			 * todo : ������ ���� Ȯ���϶�� �ؼ� osKernelSysTick()�� �ؼ� �������ϸ� HAL_GetTick() ������� ����
			 */
			curr_time = HAL_GetTick();
			if(get_diff_tick(curr_time,m_ObstacleOpenedTime) > 1000)
			{
				debug("# 1���� ���� ���� Obs -> closing \r\n");
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
	 * dcu �޸𸮸� ����.
	 */
	uint16_t openConfig_value =0;
	uint16_t CloseConfig_value=0;
	uint16_t ObstaclConfig_value=0;
	uint16_t temp_value =0;

	openConfig_value    |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x005F);					//95����
	temp_value 		     = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0060)<<8);					//96����
	openConfig_value    |= temp_value;
	CloseConfig_value   |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0061);					//97����
	temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0062)<<8);					//98����
	CloseConfig_value   |= temp_value;
	ObstaclConfig_value |= (uint16_t)mram_byte_read(MRAM_DATA_ADDR+0x0063);				//99����
	temp_value           = (uint16_t)(mram_byte_read(MRAM_DATA_ADDR+0x0064)<<8);					//100����
	ObstaclConfig_value |= temp_value;

	/*
	 * ���� mram�� ���� �о��µ� ���̻��� ���� ���´ٸ� �����缭�� ������ �������� �����Ѵ�.
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
	debug("�����ð� ���� �� : %d ms ���� �ð� ���� �� : %d ms ��ֹ� ���� ���� �� : %d \r\n",openConfig_value,CloseConfig_value,ObstaclConfig_value);
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
	
#if 0 //jeon_190708 default open 2.5s, colse 3s obs 3�� ����
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
#if 0 //jeon_190617 ����	
	if((mip_Input.di0_DLS1) || (mip_Input.di1_DLS2))
	{
		mmo_DoorClosing(0);
		mmo_DoorFree();
		osDelay(1);																	// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
		mmo_DoorBrake();
		mdc_PreDoorState = DOOR_CLOSING;
		mdc_DoorState = DOOR_CLOSED;
		m_F07Statement = F07_BRAKE;
		/*
		 * todo : delay ���� ���� count�ؼ� �����ϴ� ��� ���� ����� �� ��
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
		osDelay(1);																	// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
		mmo_DoorBrake();
		osDelay(100);
		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
		debug("F07 ���� ���ۿ��� ����������� CLOSED�� ���� \r\n");
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
		else  //���ڴ� ������ ���
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
		osDelay(1);																	// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
		mmo_DoorBrake();
//		osDelay(100);
//		StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
//		debug("F07 ���� ���ۿ��� ����������� CLOSED�� ���� \r\n");
		
		if(error_list_flag[DCU_UNEXPECTED_UNLOCK] == true) /* ���� ���� ������ �ݺ��ϰ� dls �ƴٰ� Ǯ�� ��� �� ����*/
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
			osDelay(1);																	// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
			mmo_DoorBrake();
			mdc_PreDoorState = DOOR_CLOSING;
			mdc_DoorState = DOOR_CLOSED;
			m_F07Statement = F07_BRAKE;
			/*
			 * todo : delay ���� ���� count�ؼ� �����ϴ� ��� ���� ����� �� ��
			 */
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
		}
		else
		{
			if(!mip_Input.di0_Bypass) //jeon_190708 F07�� DODBPS�� �������� �ʾ��� ���� ������
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
				else  //���ڴ� ������ ���
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
			else //jeon_190708 F07�� DODBPS�� �������� �ʾ��� ���� ������
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
	//volatile static uint32_t F10_OpenCount = 0;    // wjjang_200212 ���������� �̰�
	
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
				osDelay(1);																	// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
				mmo_DoorBrake();
				mdc_PreDoorState = DOOR_CLOSING;
				mdc_DoorState = DOOR_CLOSED;
			}
			else
			{
				mmo_DoorClosing(0);
				mmo_DoorFree();
				osDelay(1);																	// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
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

	m_F07Statement = F07_BRAKE;			//�ʱ�ȭ���������� �������� ������ �������ϱ�����Ѵ�
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
			debug("# dls ���� ���� ERROR - > CLOSE \r\n");
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
			debug("# ���� ��� DLS ���� ���� error -> close\r\n");
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
						osDelay(100);																	// ������ �ͼ� �������� ������ ���� �帣�� �����Ƿ� �ٷ� Brake ����
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
			debug("# dls ���� ���� ERROR - > OPENED \r\n");
		}
		else if(!mip_Input.di0_OpenCloseButton)
		{
			mdc_PreDoorState = DOOR_OPENED;
			mdc_DoorState = DOOR_OPENED;
			debug("# ���� ��� DLS ���� ���� error -> Open\r\n");
		}
	}
}

void FaultAction_F05_F06(uint8_t preDoorState)
{
	if(preDoorState == DOOR_CLOSED)
	{
		//DCS�� ON�� �Ǹ� ���� ����
		if(mip_Input.di1_DCS1 || mip_Input.di1_DCS2)
		{
			mdc_PreDoorState = DOOR_CLOSED;
			mdc_DoorState = DOOR_CLOSED;
			error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
			if(mip_Input.di1_DCS1 == true) error_list_flag[DCU_DCS1_FAULT] = false;
			if(mip_Input.di1_DCS2 == true) error_list_flag[DCU_DCS2_FAULT] = false;
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
			debug("# dcs ���� ���� ERROR - > CLOSE \r\n");
		}
		else if(mip_Input.di0_OpenCloseButton)
		{
			mdc_PreDoorState = DOOR_CLOSED;
			mdc_DoorState = DOOR_CLOSED;
			error_list_flag[DCU_UNEXPECTED_UNLOCK] = false;
			osDelay(100);
			StateSetToClosedAtClosing(DOOR_POSITION_CLOSED_BY_BUTTON);
			debug("# ���� ��� DCS ���� ���� error -> close\r\n");
		}
	}
	else if(preDoorState == DOOR_OPENED)
	{
		//DCS�� OFF�� �Ǹ� ���� ����
		if(!mip_Input.di1_DCS2 || !mip_Input.di1_DCS1)
		{
			mdc_PreDoorState = DOOR_OPENED;
			mdc_DoorState = DOOR_OPENED;
			if(mip_Input.di1_DCS1 == false) error_list_flag[DCU_DCS1_FAULT] = false;
			if(mip_Input.di1_DCS2 == false) error_list_flag[DCU_DCS2_FAULT] = false;
			debug("# ���� ��� DCS ���� ���� error -> close\r\n");
		}
		else if(!mip_Input.di0_OpenCloseButton)
		{
			mdc_PreDoorState = DOOR_OPENED;
			mdc_DoorState = DOOR_OPENED;
			debug("# dcs ���� ���¿��� ���� ��� open ���� ���� \r\n ");

		}
	}
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtDecisionControl.c
*********************************************************************/
