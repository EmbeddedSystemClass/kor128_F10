/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mMotorOut
//!	Generated Date	: ��, 1, 7 2017  
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
 * * ���ͻ�� - ���ڴ� �޽�
 *    - ���� ���� 1:1
 *    - 1ȸ�� �� ���ڴ� 100�޽� �߻�			(�����δ� 200�޽��� �߻��ϴµ� A��,B���� �̿��� �����Ǵ��ϵ��� �����ؼ� ������ �پ����)
 *    - ���� 1ȸ�� �� �̵�(��ġ)�Ÿ� : 36mm
 *    - 1�޽� �� 0.36mm �̵�
 *
 * * Ÿ�Ժ� ���� ������
 *    - Ÿ��A ������ : 830mm -> �� 2300�޽�
 *    - Ÿ��B ������ : 900mm -> �� 2500�޽�
 * 
 * ------------------------------------
 * korail128
 * * ���ͻ�� - ���ڴ� �޽�
 *    - ���� ���� 1:1
 *    - 1ȸ�� �� ���ڴ� 200�޽� �߻�
 *    - ���� 1ȸ�� �� �̵�(��ġ)�Ÿ� : 18mm
 *    - 1�޽� �� 0.18mm �̵�
 *    
 *    - 3000[pulse]:X[mm] = 1[pulse]:0.18[mm]
 *      X[mm] = 3000*0.18 = 540[mm]
 */

extern _pwmCh g_CurrentPWM_Ch;

extern struct pwm_channel_t g_pchFET_ctla_pwmh[4];

/* true : H-bridge�� GND ���� */
/* false : H-bridge�� GND ������ */
/*## operation mmo_ConnectHbridgeGND(_Bool) */
void mmo_ConnectHbridgeGND(_Bool connect);

/* 4�� FET On, */
/* Nä�� FET2, FET4 Min PWM���� �� �� ����ؼ� ��� ���� */
/*## operation mmo_DoorBrake() */
void mmo_DoorBrake(void);

/*## operation mmo_DoorClosing(uint32_t) */
void mmo_DoorClosing(uint32_t pwmSetValue);

/* 4�� FET ��� Off */
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
