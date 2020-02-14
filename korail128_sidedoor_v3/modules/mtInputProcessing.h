/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtInputProcessing
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtInputProcessing.h
*********************************************************************/

#ifndef mtInputProcessing_H
#define mtInputProcessing_H

#include <oxf/Ric.h>
#include "mLibrary.h"

#ifdef HW_DEPENDENT_CODE
	#if 0
		struct _Input {
			_Bool Open3s;		/*## attribute Open3s */
			_Bool di0_OpenCloseButton;		/*## attribute Open3s */
			_Bool Open3m;		/*## attribute Open3m */
			_Bool Isolation;		/*## attribute Isolation */
			_Bool Photo1;		/*## attribute Photo1 */
			_Bool Photo2;		/*## attribute Photo2 */
			_Bool DCS;		/*## attribute DCS */
			_Bool Shutdown;		/*## attribute Shutdown */
			_Bool TestButton;		/*## attribute TestButton */
		};
	#else
		struct _Input {
			uint8_t di0_OpenCloseButton;	// DI00 : 운전실에 있는 열림/닫힘 스위치, True(스위치 누름-Open), False(누르지 않음-Close)
			uint8_t di0_ReOpen;			// DI01 : 운전실에 있는 재열림 스위치(열차에서 신호선 끊어 놓는다고 함)
			uint8_t di0_ZVR;				// DI02 : True(정차신호 수신-정자중), False(정차신호 미수신-이동중)
			uint8_t di0_Isolation;		// DI03 : 차단스위치(구성품 설명서에 bypass라고 정의), True(차단), False(복귀)
			uint8_t di0_Bypass;			// DI04 : DODBPS (Door Obstacle Bypass Switch 장애물 감지 안하는 신호), True(스위치 누름, 장애물 감지 미수행), False(누르지 않음, 장애물 감지 수행)
			uint8_t di0_SafetyLoopA;		// DI05 : 옆문이 닫히면 SafetyLoop1이 연결되어 신호 On이 입력됨
			uint8_t di0_SafetyLoopB;		// DI06 : 자기 자신의 문이 닫히면 SafetyLoop2가 연결되어 신호 On이 입력됨
			uint8_t di0_DLS1;				// DI07 : 오른쪽위치, DLS1, True(스위치 누름-닫힘), False(누르지 않음-열림)
			
			uint8_t di1_DCS2;				// DI08 : 왼쪽위치, DCS2, True(스위치 누름-닫힘), False(누르지 않음-열림)
			uint8_t di1_DCS1;				// DI09 : 오른쪽위치, DCS1, True(스위치 누름-닫힘), False(누르지 않음-열림)
			uint8_t di1_EED;				// DI10 : 글자 없음, 내부 비상 스위치, True(차단), False(복귀-Normal State)
			uint8_t di1_EAD;				// DI11 : 글자 있는거, 외부 비상 스위치, True(차단), False(복귀-Normal State)
			uint8_t di1_DLS2;				// DI12 : 왼쪽위치, DLS2, True(스위치 누름-닫힘), False(누르지 않음-열림)
			
			uint8_t DoorClosedSwitchOn;	// 닫힘완료스위치 : DLS 2개중 1개 이상 그리고 DCS2개중 1개 이상이 눌린 경우 true
			uint8_t Shutdown;				/*## attribute Shutdown */
			uint8_t CommandOpen;
			uint8_t CommandClose;
			uint8_t CommandTestMode;
			uint8_t RotarySwitch;
		};
		struct Test_Input{
			uint8_t DLS1_Test_flag;			// off 고장 on 고장을 재현하기위한 플래그 해당 플래그가 셋이 되면 input 태스크는 스위치 입력을 받지 않는다.
			uint8_t DLS2_Test_flag;
			uint8_t DCS1_Test_flag;
			uint8_t DCS2_Test_flag;
			uint8_t OpenButton_Test_flag;
		};
	#endif
#endif

extern struct Test_Input mip_Test;
extern struct _Input mip_Input;
extern struct _Input mip_PreInput;
void mip_TaskInputProcessing(void const * argument);
void mip_MasterCodeMasterLED(_Bool LED);
void mip_SleveCodeSlaveRun(_Bool SlaveRun);
void mip_EmergencyLamp(_Bool EmergencyLamp);


#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtInputProcessing.h
*********************************************************************/
