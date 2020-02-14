/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtInputProcessing
//!	Generated Date	: ��, 1, 7 2017  
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
			uint8_t di0_OpenCloseButton;	// DI00 : �����ǿ� �ִ� ����/���� ����ġ, True(����ġ ����-Open), False(������ ����-Close)
			uint8_t di0_ReOpen;			// DI01 : �����ǿ� �ִ� �翭�� ����ġ(�������� ��ȣ�� ���� ���´ٰ� ��)
			uint8_t di0_ZVR;				// DI02 : True(������ȣ ����-������), False(������ȣ �̼���-�̵���)
			uint8_t di0_Isolation;		// DI03 : ���ܽ���ġ(����ǰ ������ bypass��� ����), True(����), False(����)
			uint8_t di0_Bypass;			// DI04 : DODBPS (Door Obstacle Bypass Switch ��ֹ� ���� ���ϴ� ��ȣ), True(����ġ ����, ��ֹ� ���� �̼���), False(������ ����, ��ֹ� ���� ����)
			uint8_t di0_SafetyLoopA;		// DI05 : ������ ������ SafetyLoop1�� ����Ǿ� ��ȣ On�� �Էµ�
			uint8_t di0_SafetyLoopB;		// DI06 : �ڱ� �ڽ��� ���� ������ SafetyLoop2�� ����Ǿ� ��ȣ On�� �Էµ�
			uint8_t di0_DLS1;				// DI07 : ��������ġ, DLS1, True(����ġ ����-����), False(������ ����-����)
			
			uint8_t di1_DCS2;				// DI08 : ������ġ, DCS2, True(����ġ ����-����), False(������ ����-����)
			uint8_t di1_DCS1;				// DI09 : ��������ġ, DCS1, True(����ġ ����-����), False(������ ����-����)
			uint8_t di1_EED;				// DI10 : ���� ����, ���� ��� ����ġ, True(����), False(����-Normal State)
			uint8_t di1_EAD;				// DI11 : ���� �ִ°�, �ܺ� ��� ����ġ, True(����), False(����-Normal State)
			uint8_t di1_DLS2;				// DI12 : ������ġ, DLS2, True(����ġ ����-����), False(������ ����-����)
			
			uint8_t DoorClosedSwitchOn;	// �����Ϸὺ��ġ : DLS 2���� 1�� �̻� �׸��� DCS2���� 1�� �̻��� ���� ��� true
			uint8_t Shutdown;				/*## attribute Shutdown */
			uint8_t CommandOpen;
			uint8_t CommandClose;
			uint8_t CommandTestMode;
			uint8_t RotarySwitch;
		};
		struct Test_Input{
			uint8_t DLS1_Test_flag;			// off ���� on ������ �����ϱ����� �÷��� �ش� �÷��װ� ���� �Ǹ� input �½�ũ�� ����ġ �Է��� ���� �ʴ´�.
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
