/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtMotorFeedback
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.h
*********************************************************************/

#ifndef mtMotorFeedback_H
#define mtMotorFeedback_H

#include <oxf/Ric.h>
#include "mDataManage.h"
#include "mLibrary.h"

struct _FlagEnd {
    _Bool OpenedByPosition;						// 엔코더를 통한 열림/닫힘완료 판단은 Encoder모드 PID제어에서 열림거리 넘어가면 바로 정지
    _Bool ClosedByPosition;						// (엔코더로 위치에 따라 속도 프로파일 및 PWM출력이 변경되므로, 엔코더로는 정확해야 하므로 PID제어에서 바로 정지)
    _Bool OpenedByCurrent;						// 전류를 통한 열림/닫힘완료 판단은 Bemf모드 PID제어에서 또는 SlowMode(PID제어 미수행)에서 둘 다 정지해야 함
    _Bool ClosedByCurrent;						// (전류로 열림/닫힘완료 판단은 Obstacle Detect Task에서 수행
};

struct _Encoder {
    int32_t Direction;		/*## attribute Direction */
    int32_t MaxOpenPosition;		/*## attribute RefPosition */
    int32_t MaxClosePosition;		/*## attribute RefPosition */
    int32_t RefVelocity;		/*## attribute RefVelocity */
    int32_t PreStopPosition;		/*## attribute Position */
    int32_t Position;		// 1[pulse]발생 시 0.18[mm]이동 /*## attribute Position */
    int32_t tmpClosedPosition;		/*## attribute Position */	// only 닫힘위치 출력용
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
//#define LPF									// 이상하게 동작함 -> 디버깅 해야 함

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
/* 정지상태(mmf_PIDControl == None)에서는 호출되지 않음 */
void mmf_isrBemf_VelocityControl(void);
void mmf_isrEncoder_PositionCalculate(void);
/* TC0_CH0의 Interrupt Service Routine에서 10ms마다 호출되어 100ms마다 생성되는 Vref를 10ms마다 PID제어 수행 */
void mmf_isrEncoder_VelocityControl(void);
void mmf_vTimerCallback_VrefGenerator(void);
void setPIDPWMOut(void);
static void DiagnosisInMotorFeedback(void);


#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.h
*********************************************************************/
