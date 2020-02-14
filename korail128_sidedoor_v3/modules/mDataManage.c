/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mDataManage
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mDataManage.c
*********************************************************************/

#include "mDataManage.h"

_Bool error_list_flag[17] = {false,false,false,false,false, \
							 false,false,false,false,false, \
							 false,false,false,false,false, \
							 false,false};

int16_t Current_Velocity_value = 0;
int16_t Ref_Velocity_value = 0;


uint8_t error_list_count[17]={0,0,0,0,0,
							  0,0,0,0,0,
							  0,0,0,0,0,
							  0,0};
uint32_t error_list_time[17]={0,0,0,0,0,
							  0,0,0,0,0,
							  0,0,0,0,0,
							  0,0};
uint32_t error_485list_time[6] = {0,0,0,0,0,0};
uint16_t error_485_flag[6] = {false,false,false,false,false,false};
xSemaphoreHandle g_semaphoreBEMF = NULL;

struct _door_time mdm_time = {0,0,0,0,0,0,0,0,0,0};
struct _dcu_id_t dcuID ={0,0};
int32_t m_CurrentPWMDuty = 0;
int32_t OpenPowerValue = 1;								//유지보수프로그램으로 파라메터 설정할때 열림 시간 조절해주는 변수
int32_t ClosePowerValue =1;								//유지보수프로그램으로 파라메터 설정할때 닫힘 시간 조절해주는 변수

_Bool m_ErrorFlag = false;

uint32_t m_ObstacleOpenedTime = 0;

int32_t m_OpeningEncoderPulse = 0;

struct _adc m_adc = {0,0,0,0,0};

struct _adcsamples m_adcsamples = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},0,0};

uint32_t m_afec0_data[4];

_MasterSlave m_isMasterSlave = MASTER_DCU;
_CarDefineIDConfig m_CardefineID = Define_NONE;			// M Car - T Car 구분에 따라 동작하기 위한 열거형 타입
_RtcSetDefine m_RtcDefine = SET_NONE;
_F07Statement m_F07Statement = F07_BRAKE;
_F0408Statement m_F0408Statement = F0408_BRAKE;

uint32_t g_unDeviceID=0;								// DCU_L1_MCAR, DCU_R1_MCAR, DCU_L1_TCAR, DCU_R1_TCAR
uint8_t m_isFaultCount = 0;
_Bool m_isSlaveRunCommand = false;						// 절체 Flag
_Bool m_isPacketSendToSlave = true;						// 절체 테스트 Flag
_Bool m_is485changeflag = true;
_Bool m_isMasterSlaveChange = true;
_Bool m_isTaskExecution = false;						// 190311 수정 내역: Decision에서 값 초기화가 완료되면 셋시켜서 다른 태스크들 실행 시킨다.
_Bool m_isTestEncoderflag = false;
RTCTIME rtc_time ={0,0,0,0,0,0,0};						// rs485 , mvb 데이터만 저장하는 변수
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mDataManage.c
*********************************************************************/
