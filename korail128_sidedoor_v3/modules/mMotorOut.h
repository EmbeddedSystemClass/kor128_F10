/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mMotorOut
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mMotorOut.h
*********************************************************************/

#ifndef mMotorOut_H
#define mMotorOut_H

#include <oxf/Ric.h>
#include "mDataManage.h"
#include "mLibrary.h"

/* 
 * ------------------------------------
 * EMU250
 * * 모터사양 - 엔코더 펄스
 *    - 모터 기어비 1:1
 *    - 1회전 당 엔코더 100펄스 발생			(실제로는 200펄스가 발생하는데 A상,B상을 이용해 방향판단하도록 구현해서 반으로 줄어들음)
 *    - 모터 1회전 당 이동(피치)거리 : 36mm
 *    - 1펄스 당 0.36mm 이동
 *
 * * 타입별 도어 열림폭
 *    - 타입A 열림폭 : 830mm -> 약 2300펄스
 *    - 타입B 열림폭 : 900mm -> 약 2500펄스
 * 
 * ------------------------------------
 * korail128
 * * 모터사양 - 엔코더 펄스
 *    - 모터 기어비 1:1
 *    - 1회전 당 엔코더 200펄스 발생
 *    - 모터 1회전 당 이동(피치)거리 : 18mm
 *    - 1펄스 당 0.18mm 이동
 *    
 *    - 3000[pulse]:X[mm] = 1[pulse]:0.18[mm]
 *      X[mm] = 3000*0.18 = 540[mm]
 */

extern _pwmCh g_CurrentPWM_Ch;

extern struct pwm_channel_t g_pchFET_ctla_pwmh[4];

/* true : H-bridge와 GND 연결 */
/* false : H-bridge와 GND 끊어짐 */
/*## operation mmo_ConnectHbridgeGND(_Bool) */
void mmo_ConnectHbridgeGND(_Bool connect);

/* 4개 FET On, */
/* N채널 FET2, FET4 Min PWM으로 둘 다 출력해서 잡고 있음 */
/*## operation mmo_DoorBrake() */
void mmo_DoorBrake(void);

/*## operation mmo_DoorClosing(uint32_t) */
void mmo_DoorClosing(uint32_t pwmSetValue);

/* 4개 FET 모두 Off */
/*## operation mmo_DoorFree() */
void mmo_DoorFree(void);

void mmo_DoorOpening(uint32_t pwmSetValue);

void mmo_MotorDisable(void);

void mmo_MotorPositionReset(void);

void mmo_MotorPWMStart(void);

void mmo_MotorPWMStop(void);

void mmo_DoorOpeningNchFET(uint32_t pwmValue);

void mmo_DoorClosingNchFET(uint32_t pwmValue);
uint32_t mmo_getEncoderCountValue(void);

#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mMotorOut.h
*********************************************************************/
