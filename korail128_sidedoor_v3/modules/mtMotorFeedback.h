/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtMotorFeedback
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.h
*********************************************************************/

#ifndef mtMotorFeedback_H
#define mtMotorFeedback_H

#include <oxf/Ric.h>
#include "mDataManage.h"
#include "mLibrary.h"

struct _FlagEnd {
    _Bool OpenedByPosition;						// ���ڴ��� ���� ����/�����Ϸ� �Ǵ��� Encoder��� PID����� �����Ÿ� �Ѿ�� �ٷ� ����
    _Bool ClosedByPosition;						// (���ڴ��� ��ġ�� ���� �ӵ� �������� �� PWM����� ����ǹǷ�, ���ڴ��δ� ��Ȯ�ؾ� �ϹǷ� PID����� �ٷ� ����)
    _Bool OpenedByCurrent;						// ������ ���� ����/�����Ϸ� �Ǵ��� Bemf��� PID����� �Ǵ� SlowMode(PID���� �̼���)���� �� �� �����ؾ� ��
    _Bool ClosedByCurrent;						// (������ ����/�����Ϸ� �Ǵ��� Obstacle Detect Task���� ����
};

struct _Encoder {
    int32_t Direction;		/*## attribute Direction */
    int32_t MaxOpenPosition;		/*## attribute RefPosition */
    int32_t MaxClosePosition;		/*## attribute RefPosition */
    int32_t RefVelocity;		/*## attribute RefVelocity */
    int32_t PreStopPosition;		/*## attribute Position */
    int32_t Position;		// 1[pulse]�߻� �� 0.18[mm]�̵� /*## attribute Position */
    int32_t tmpClosedPosition;		/*## attribute Position */	// only ������ġ ��¿�
    int32_t Velocity;		/*## attribute Velocity */
    int32_t PrePosition;
	#ifdef EMC_TEST
    int32_t InitPosition;		/*## attribute Velocity */
    int32_t InitVelocity;		/*## attribute Velocity */
	#endif
};

typedef struct _VelocitySection {
	int32_t Acceleration;		/*## attribute CaptureRa */
	int32_t Constant;		/*## attribute CaptureRb */
	int32_t Deceleration;		/*## attribute CaptureRa */
}VelocitySection;

#if 0 //jeon_190715
struct _Bemf {
    float RefPosition;		/*## attribute RefPosition */
    float RefVelocity;		/*## attribute RefVelocity */
    float Position;		/*## attribute Position */
    float Velocity;		/*## attribute Velocity */
};
#else
struct _Bemf {
	int32_t RefPosition;		/*## attribute RefPosition */
	int32_t RefVelocity;		/*## attribute RefVelocity */
	int32_t Position;		/*## attribute Position */
	int32_t Velocity;		/*## attribute Velocity */
};
#endif

#define TEST_BEMF
//#define LPF									// �̻��ϰ� ������ -> ����� �ؾ� ��

extern struct _Bemf mmf_Bemf;
extern struct _Encoder mmf_Encoder;
extern VelocitySection OpeningSection;
extern VelocitySection ClosingSection;
extern struct _FlagEnd mmf_EndDecision;
extern _pidControlState mmf_NextMotorDirSelect;
extern _pidControlState mmf_PIDControl;

void mmf_ConversionADC(void);
void mmf_TaskMotorFeedback(void const * argument);
void mmf_isrBemf_PositionCalculate(void);
/* ��������(mmf_PIDControl == None)������ ȣ����� ���� */
void mmf_isrBemf_VelocityControl(void);
void mmf_isrEncoder_PositionCalculate(void);
/* TC0_CH0�� Interrupt Service Routine���� 10ms���� ȣ��Ǿ� 100ms���� �����Ǵ� Vref�� 10ms���� PID���� ���� */
void mmf_isrEncoder_VelocityControl(void);
void mmf_vTimerCallback_VrefGenerator(void);
void setPIDPWMOut(void);
static void DiagnosisInMotorFeedback(void);


#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.h
*********************************************************************/
