/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mDataManage
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mDataManage.h
*********************************************************************/

#ifndef mDataManage_H
#define mDataManage_H

#include <oxf/Ric.h>
#include "mLibrary.h"

/*
 * Revision History
 * 
 * -------------------
 * v0.1.0 (2018.09.10)
 * �⺻ ����Ʈ���� ���� emu250 enddoor �÷������� Ȯ��
 * Open/Close ������ Ȯ�� (���ķ����� ���� ���� Closing �� Reset �� -> HW���װ���)
 * 
 * -------------------
 * v0.1.1 (2018.10.26, korail128_sidedoor_svn_v61)
 * �⺻ ���� korail128 sidedoor �÷������� Ȯ��
 * Open/Close ������ Ȯ��
 * ���� �� ������ư�� Ǯ�� ��������, ���� �� ������ư�� ���� �������� Ȯ��
 * ��ֹ� 3ȸ ���� �� �������� ���� Ȯ��
 * ���ܽ���ġ ���� �� Free, ���� �� Closing ���� Ȯ�� (����: 2ȸ° ���� ���� �� ��� �а� ����)
 */
#define VERSION_MAJOR				(1)				// orig:1
#define VERSION_MINOR				(0)				// orig:0
#define VERSION_SECOND				(2)				// orig:2

#define RSWDT_PERIOD        		1000			// 1000ms
/*
 * 1�޽� �� ���� �̵��Ÿ�
 *	0.18[mm]:1[pulse] = 10[mm]:y[pulse]
 *	y[pulse] = 10/0.18 = 55[pulse]
 * 
 */
#define TYPE_A						2277			// 831mm(max:2308 pulse) opening  
#define TYPE_B						2500			// 911mm(max 2530 pulse) opening
#define TYPE_KORAIL_SIDEDOOR		3580			// 3580 - 650mm opening

#define OPEN_ACCEL_SECTiON			500				// 2s(500) - 650mm opening
#define OPEN_FAST_ACCEL_SECTiON		20				// jeon_190822 10 => 20
#define OPEN_CONST_SECTiON			1250			// 2s(1250) - 650mm opening
#define OPEN_SLOW_CONST_SECTiON		2900
#define OPEN_DEACE_SECTiON			3580			// 2s(3580) - 650mm opening

#define CLOSE_ACCEL_SECTiON			3000			// 2.5s(3000) - 650mm closing
#define CLOSE_CONST_SECTiON			2250			// 2.5s(2250) - 650mm closing
#define CLOSE_SLOW_CONST_SECTiON	900			//3s 1000
#define CLOSE_DEACE_SECTiON			0				// 2.5s(0) - 650mm closing

#define Motor_Ke					0.1				// Ke = 110V/1000rpm 
#define Motor_Vmax					110				// �������� = 110V 
#define Motor_Imax					2.1				// �������� = 2.1A 

#define PWM_FREQUENCY				10000			// PWM ���ļ�
#define PWM_MAX						700				// 100V PWM �ִ� ���
#define PWM_MIN						200				// 100V PWM �ּ� ���
#define PERIOD_VALUE				1000			// PWM Duty ��� 0~1000

#define SPI_INSTRUCTION_DATA		0x00			// 0b.0000.0000 : Process Data �� �аų� ���� ���� ��� (F code 3, ������Ʈ ����)
#define SPI_INSTRUCTION_LA_PCS		0x20			// 0b.0010.0000 : LA PCS �� �аų� ���� ���� ��� (���� ��Ʈ)
#define SPI_INSTRUCTION_CTR_REG		0x60			// 0b.0110.0000 : Control Register �� �аų� ���� ���� ��� (Src/Snk ��Ʈ����)
#define SPI_INSTRUCTION_MSG0		0x80			// 0b.1000.0000 : Priority 0 �� Event �� Message Data �� �аų� ���� ���� ��ɾ�
#define SPI_INSTRUCTION_MSG1		0xA0			// 0b.1010.0000 : Priority 1 �� Event �� Message Data �� �аų� ���� ���� ��ɾ�
#define SPI_INSTRUCTION_MEMORY		0xC0			// 0b.1100.0000 : ��ü Memory �� �����͸� �аų� ���� ���� ���, �����ʹ� �ּ� 2B - �ִ� 32B ������ 2B ������ �����Ϳ��� �Ѵ�.

#define SPI_INSTRUCTION_READ		0x00			// �б� ����
#define SPI_INSTRUCTION_WRITE		0x08			// ���� ����

#define MVB_SLAVE_SOURCE			0x08			// ���� 4bit �� ù��° bit
#define MVB_SLAVE_SINK				0x04			// ���� 4bit �� �ι�° bit

#define MVB_FCODE0_2BYTE			0x00			// 2byte data length	0b.0000.0000
#define MVB_FCODE1_4BYTE			0x10			// 4byte data length	0b.0001.0000
#define MVB_FCODE2_8BYTE			0x20			// 8byte data length	0b.0010.0000
#define MVB_FCODE3_16BYTE			0x30			// 16byte data length	0b.0011.0000
#define MVB_FCODE4_32BYTE			0x40			// 32byte data length	0b.0100.0000

#define DCU1_MVB					0xf1			// ���͸� ����ġ 1�� ���� �� ���� 0xf1��
#define DCU2_MVB					0xf2
#define DCU3_MVB					0xf3
#define DCU4_MVB					0xf4

#define DCU_ALL_MVB_SINK_COMMON		0x2C			// DCU1_MVB_SINK_SDR, DCU2_MVB_SINK_SDR, DCU3_MVB_SINK_SDR, DCU4_MVB_SINK_SDR
#define DCU_ALL_MVB_SINK_SDR		0xD0			// DCU1_MVB_SINK_SDR, DCU2_MVB_SINK_SDR, DCU3_MVB_SINK_SDR, DCU4_MVB_SINK_SDR

#define DCU_L1_M_MVB_SOURSE_SD		0xD4
#define DCU_M_MVB_SINK_TDR			0xF8			// DCU1_MVB_SINK_TDR, DCU2_MVB_SINK_TDR
#define DCU_L1_M_MVB_SOURSE_TD		0xDC

#define DCU_R1_M_MVB_SOURSE_SD		0xD8
#define DCU_R1_M_MVB_SOURSE_TD		0xE0

#define DCU_L1_T_MVB_SOURSE_SD		0xE4
#define DCU_T_MVB_SINK_TDR			0xFC			// DCU3_MVB_SINK_TDR, DCU4_MVB_SINK_TDR
#define DCU_L1_T_MVB_SOURSE_TD		0xEC

#define DCU_R1_T_MVB_SOURSE_SD		0xE8
#define DCU_R1_T_MVB_SOURSE_TD		0xF0

#if 0
/*
 * �һ����/���μ� �ڵ��� �Է� ���� (M_Car, T_Car ���� ����)
 */
#define DCU_L1				0x01
#define DCU_L2				0x02
#define DCU_L3				0x03
#define	DCU_L4				0x04
#define DCU_R1				0x11
#define DCU_R2				0x12
#define DCU_R3				0x13
#define	DCU_R4				0x14
#else
/*
 * Korail128 Test�� DCU����
 */
#define DCU_L1_MCAR			0x01
#define DCU_L2_MCAR			0x02
#define DCU_L3_MCAR			0x03
#define DCU_L4_MCAR			0x04
#define	DCU_R4_MCAR			0x05
#define DCU_R3_MCAR			0x06
#define	DCU_R2_MCAR			0x07
#define DCU_R1_MCAR			0x08

#define DCU_L1_TCAR			0x11
#define DCU_L2_TCAR			0x12
#define DCU_L3_TCAR			0x13
#define DCU_L4_TCAR			0x14
#define	DCU_R4_TCAR			0x15
#define DCU_R3_TCAR			0x16
#define	DCU_R2_TCAR			0x17
#define DCU_R1_TCAR			0x18
#endif

/* �����ڵ�, �����ڵ� ����Ʈ */
/*## type _ErrorCodeList */
#if 0
/*
 * EMU250
 */
typedef enum _ErrorCodeList_t {
    DCU_ERROR, /* 					F00 - DCU���� */
    DCU_SELF_TEST_ERROR, /* 		F01 - �ڰ����ܿ��� */
    DCU_MOTOR_ERROR, /* 			F02 - ���Ͱ��� */
    DCU_ENCODER_ERROR, /* 			F03 - ���ڴ����� */
    DCU_DCS_OPEN_CIRCUIT_ERROR, /* 	F04_1 - DCS Open Circuit ���� */
    DCU_DCS_SHORT_CIRCUIT_ERROR, /* F04_2 - DCS Short Circuit ���� */
    DCU_OBSTACLE_15, /* 			F05 - ��ֹ� 15ȸ ���� (16ȸ���� ����) */
//    DCU_HIGH_VOLTAGE_ERROR, /* 	F06 - 130V�̻� �������� ����  - ���� ��û���� ������*/
//    DCU_LOW_VOLTAGE_ERROR, /* 	F07 - 65V���� �������� ����  - ���� ��û���� ������ */
    DCU_CLOSE_ERROR, /* 			F08 - ��������(�������۽ð��ʰ�) */
    DCU_OPEN_ERROR, /* 				F09 - ��������(�������۽ð��ʰ�) */
    DCU_OPEN_SWITCH_ERROR, /* 		F10 - ��������ġ ���� */
} _ErrorCodeList;
#else


/* dcu �Ķ��Ÿ ��(�ӵ�,�� ��)*/

#if 1
#define DCU_DODBPS_POWER 			420 //jeon_190624 orig:420						//������ ���� ���� uint32_t
#define DCU_DODBPS_FREETIME			8000u
#define DCU_DODBPS_CLOSINGTIME		(DCU_DODBPS_FREETIME + 1000u)
#define DCU_SLOWMODE_POWER 			200u //jeon_190514 orig:250u				//������ ���� ���� uint32_t
#define DCU_OBSMODE_POWER 			300u //jeon_190619 orig:230u				//������ ���� ���� uint32_t 
#define DCU_OBS_POSITION			0u  //jeon_190514 orig:3u
#endif
#define __DEBUG
#ifdef __DEBUG
	#define debug(...) {printf(__VA_ARGS__);}
	#define trace(...) {printf("file:%s, function:%s, line:%d -> ", __FILE__, __FUNCTION__, __LINE__);printf(__VA_ARGS__);}
#else
   #define debug(...)
#endif
/*
 * korail128
 */
typedef enum _ErrorCodeList_t {
	/*
	 * �����÷���
	 * - ����
	 *   : ������ �߻��� ������ �ش������ �Բ� �׻� �����÷��׵� 1�� ������
	 *   : FND Display Task���� F01~F16 ������ ��ĵ�ؼ� true�� ������ 0���̸� �����÷��״� �ڵ����� 0���� Ŭ���� ��
	 *     -> ���� Ư�� ������ ���ŵǸ� �ش� ������ Ŭ���� �ϸ� �ǰ�, ��ü ������ ���� ��쿡�� �����÷��װ� �ڵ����� Ŭ���� ��
	 */
	DCU_ERROR = 0u,
	
	/*
	 * F01 �߰���
	 * -���� ����
	 *   : �������� ���� �ڰ� �׽�Ʈ�� ����
	 *   : Closed ���¿���(Closing -> Closed ��)�� �ڰ��׽�Ʈ ����
	 *   : �ڰ��׽�Ʈ ���� ���°� Opening���� �ٲ�� �ڰ��׽�Ʈ �ߴ�
	 * -����߻� ����
	 *   : ������� �Ұ� -> ���������� ���� 1�� ��ġ��Ŵ, �ѹ� �߻��ϸ� Clear���� ����
	 * -���庹�� ����
	 *   : ���� �� ���庹�� ����
	 */
	DCU_HARD_FAULT =1u,
	
	/*
	 * F02 �����
	 * -���� ����
	 *   : ���� �һ���ÿ��� RS485 ��� �Ұ� �ÿ��� True�� ���� �ǰ�, ��� ���� �� �ڵ����� �ش�  Clear
	 * -����߻� ����
	 *   : ������� �Ұ� -> ���������� ���� 2�� ��ġ��Ŵ
	 * -���庹�� ����
	 *   : ���� �� ���庹�� ����
	 */
	DCU_MINOR_FAULT=2u,
	
	/*
	 * F03 ���� ȸ�� ����
	 * -����߻� ���� (Ȯ���Ϸ��� ���װ� ������ �־�� ��)
	 *   : ����(Opening) �Ǵ� ����(Closing) ���¿��� ���ڴ� �޽��� ��ȭ���� �ʰ�, ���Ϳ� ������ �帣�� ���� ��� ���Ͱ���
	 *     ��, ���� ���(Opening, Closing���)�� ���´µ� ���Ͱ� �ƿ� �������� �ʴ� ���
	 * -���庹�� ����
	 *   : ���� �� ���庹�� ����, ���� �Ѿ����� �ش�  Clear
	 */
	DCU_MOTOR_ERROR=3u,
	
	/*
	 * F04 DLS1����
	 * -����߻� ���� (DLS���� �� ���ڴ�,������ ���� Opened/Closed �Ǵ� ����)
	 *   : ����(Opened) ���¿��� DLS1�� ���� ������ ���� - Short Circuit
	 *   : ����(Closed) ���¿��� DLS1�� ������ ������ ���� - Open Circuit
	 * -���庹�� ����
	 *   : ����(Opened) ���¿��� DLS1�� ������ ������ DLS1 ��������
	 *   : ����(Closed) ���¿��� DLS1�� ������ DLS1 ��������
	 */
	DCU_DLS1_FAULT=4u,
	
	/*
	 * F05 DCS1����
	 * -����߻� ���� (DLS���� �� ���ڴ�,������ ���� Opened/Closed �Ǵ� ����)
	 *   : ����(Opened) ���¿��� DCS1�� ���� ��찡 3ȸ �����Ǹ� DCS1���� - Short Circuit
	 *   : ����(Closed) ���¿��� DCS1�� ���� ���� �ʴ� ��찡 3ȸ �����Ǹ� DCS1���� - Open Circuit
	 * -���庹�� ����
	 *   : ����(Opened) ���¿��� DCS1�� ������ ������ DLS1 ��������
	 *   : ����(Closed) ���¿��� DCS1�� ������ DLS1 ��������
	 */
	DCU_DCS1_FAULT=5u,
	
	/*
	 * F06 DCS2����
	 * - DCS1�� ����
	 */
	DCU_DCS2_FAULT=6u,
	
	/*
	 * F07 ����ġ ���� ��� Ǯ��
	 * -����߻� ����
	 *   : ��������(Closed)���� DLS�� ��¦ ��� DLS�� �� �� ������ ������ ����ġ ���� ���Ǯ��(F07) �߻�
	 * -���庹�� ����
	 *   : ��������(Closed)���� DLS�� �� �� ������ F07 ����
	 */
	DCU_UNEXPECTED_UNLOCK=7u,
	
	/*
	 * F08 DLS2����
	 * - DLS1�� ����
	 */
	DCU_DLS2_FAULT=8u,
	
	/*
	 * F09 ������ ��ְ��� ���� (��ֹ� 3ȸ ����)
	 * -����߻� ����
	 *   : ��ֹ� 3ȸ ���� �� ��������
	 * -���庹�� ����
	 *   : ����(Closed)���°� �Ǹ� ���� ����
	 */
	DCU_OBSTACLE=9u,
	
	/*
	 * F10 ���� ���� ���� ����
	 * -����߻� ����
	 *   : DLS�� Off�ǰ� 2s�̳��� DCS�� Off ���� �ʴ� ��찡 3ȸ �����Ǹ� ��������(F11) �߻�
	 * -���庹�� ����
	 *   : DLS�� Off�ǰ� 2s�̳��� DCS�� Off �Ǹ� ��������(F11) ����
	 */
	DCU_OPEN_FAULT=10u,
	
	/*
	 * F11 ���߰� ���� ��
	 * -���� ����
	 *   : 
	 * -����߻� ����
	 *   : 
	 * -���庹�� ����
	 *   : 
	 */
	DCU_SLAVE_RUN=11u,
	
	/*
	 * F12 ���߰� ��ȯ �Ұ�
	 * -���� ����
	 *   : Slave�κ��� ��Ŷ�� �������� ���ϸ� Slave(���߰�) �������� �Ǵ�
	 * -����߻� ����
	 *   : Slave ǻ�� ���� ��
	 *   : ��� �簳 �� �ش�  Clear
	 * -���庹�� ����
	 *   : 
	 */
	DCU_CANT_SLAVE=12u,
	
	/*
	 * F13 ���ڴ� ����
	 * -����߻� ���� (Ȯ���Ϸ��� ���װ� ������ �־�� ��)
	 *   : �����Ϸ� �� ���ڴ� �޽��� ������ ������ ���ų� ���� ������ ��찡 3ȸ �����Ǹ� ���ڴ����� �߻�
	 *     - ���ڴ� �޽��� ������ ������ ���� �߻��� ���ڴ��� ���� �����ϷḦ �Ǵ��� ��� ���ڴ�����
	 *     - ���ڴ� �޽��� ������ ������ ���� �߻��� ������ ���� �����ϷḦ �Ǵ��� ��� ���ڴ�����
	 *   : Opening �� ���׷� ���ڴ� �� ������ ������ ���߰� F13
	 * -���庹�� ����
	 *   : �����Ϸ� �� ���ڴ� �޽��� ������ ������ ������ ���ڴ����� ����
	 */
	DCU_ENCODER_ERROR=13u,
	
	/*
	 * F14 Safety Loop �̻�
	 * -����߻� ����
	 *   : ���׷� Ȯ��
	 * -���庹�� ����
	 *   : 
	 */
	DCU_SAFETYLOOP_FAULT=14u,
	
	/*
	 * F15 MVB��� ����
	 * -���� ����
	 *   : TCMS�� MVB�� ��� ���� �� ����
	 * -����߻� ����
	 *   : ���� �ð� �������� ��Ŷ �̼��� �� ��Ű���
	 * -���庹�� ����
	 *   : ��� �簳 �� ���� ����
	 */
	DCU_MVB_ERROR=15u,
	
	/*
	 * F16 RS-485��� ����
	 * -���� ����
	 *   : 
	 * -����߻� ����
	 *   : Master DCU�� Slave DCU �� RS485 ��� ���� �� ����
	 * -���庹�� ����
	 *   : ��� �簳 �� ���� ����
	 */
	DCU_RS485_ERROR=16u,
	
	/*
	 * F17 ��� ���� -> ����
	 * -����߻� ����
	 *   : DCS�� On�ǰ�, 2s �̳��� DLS�� On ���� �ʴ� ��찡 3ȸ �����Ǹ� ������(F17) �߻�
	 *   : ���������� �պ��� �ϹǷ� ��ü �� F17 �ٽ� �߻��ϸ� ��������θ� F17 Clear �Ǿ�� ��.
	 * -���庹�� ����
	 *   : DCS�� On�ǰ�, 2s �̳��� DLS�� On�Ǹ� ������(F17) ����
	 */
	DCU_LOCK_FAULT=17u
} _ErrorCodeList;


typedef enum _RS485ErrorList_t {
	DCU_RS485_ERROR_L1 = 0U,
	DCU_RS485_ERROR_L2 = 1U,
	DCU_RS485_ERROR_L3 = 2U,
	DCU_RS485_ERROR_R2 = 3U,
	DCU_RS485_ERROR_R3 = 4U,
	DCU_RS485_ERROR_R4 = 5U
}_RS485ErrorList;


typedef enum _EventCodeList_t {
	DCU_ISO_EVENT = 20,
	DCU_EAD_EVENT = 21,					//�ܺ� ����ڵ�
	DCU_EED_EVENT = 22,					//���� ����ڵ�
	DCU_EVNET_NONE =23
}_EventCodeList;

typedef enum _StateCodeList_t {
	S01_SPARE=1,
	S02_SPARE,
	S03_SPARE,
	S04_DCU_OK,
	S05_DNC,
	S06_DI,
	S07_DFO,
	S08_FP,
	S09_ODS,
	S10_EED,
	S11_EAD,
	S12_OPENPB,
	S13_CLOSEPB,
	S14_REOPENPB,
	S15_SPARE,
	S16_SPARE
} _StateCodeList;
#endif

/* ������� */
/*## type _DoorState */
typedef enum _DoorState_t {
    DOOR_INIT, /*   		=0 */
    DOOR_OPENING, /*   	=1 */
    DOOR_OPENED, /*   	=2 */
    DOOR_CLOSING, /*   	=3 */
    DOOR_CLOSED, /*   	=4 */
    DOOR_OBSTACLE, /*   	=5 */
    DOOR_ISOLATION, /* 		=6 */
    DOOR_ERROR, /*   		=7 */
	DOOR_EAD
} _DoorState;

typedef enum _DoorDistance_t {
    DOOR_POSITION_CLOSED_BY_END,			// ���������� ��ü �̵��Ÿ���ŭ �̵��ϰ� �����Ϸ� -> ��ü �Ÿ����� ����(Closing) ����/���/���� �Ÿ� ����
    DOOR_POSITION_OPENED_BY_END,			// ���������� ��ü �̵��Ÿ���ŭ �̵��ϰ� �����Ϸ� -> ��ü �Ÿ����� ����(Closing) ����/���/���� �Ÿ� ����
    DOOR_POSITION_CLOSED_BY_BUTTON,		// ���� �� ������ư�� ���� �ٽ� ���� ���� �����Ϸ�
    DOOR_POSITION_CLOSED_BY_OBSTACLE,	// ���� �� ��ֹ������� �ٽ� ���� ���� �����Ϸ�
    DOOR_POSITION_OPENED_BY_BUTTON,		// ���� �� ������ư�� Ǯ�� �ٽ� �ݱ����� �����Ϸ�
    DOOR_POSITION_OPENED_BY_OBSTACLE,	// ���� �� ��ֹ������� �� ��¦ �̵��ϰ� �ٽ� �ݱ����� �����Ϸ�
} _DoorDistance;

/* Opening ��	- Afec0_Ch6(Supply Voltage), Afec0_Ch5(Bemf Voltage) */
/* Closing ��	- Afec0_Ch5(Supply Voltage), Afec0_Ch6(Bemf Voltage) */
/*## type _AfecChValue */
typedef enum _AfecChValue_t {
    CH4_MOTOR_CURR, /* PB0(AFEC0_AD4) - �������� */
    CH5_BEMF_R, /* PB1(AFEC0_AD5) - BEMF����(��) */
    CH6_BEMF_F, /* PB2(AFEC0_AD6) - BEMF����(��) */
    CH8_3_3V /* PB8(AFEC0_AD8) - 3.3V Monitor */
} _AfecChValue;

typedef enum _pwmCh_t {
    PWM_CH_NO_SELECTE,
    PWM_CH1_OPENING_FORWARD,
    PWM_CH3_CLOSING_REVERSE = 3
} _pwmCh;

typedef enum _pidControlState_t {
    PID_CONTROL_NONE,
    PID_CONTROL_OPENING,
    PID_CONTROL_CLOSING
} _pidControlState;

typedef enum _rdien0_t {
    DI0_OPEN_CLOSE,
    DI0_REOPEN,
    DI0_ZVR,
    DI0_ISOLATION,
    DI0_BYPASS,
    DI0_SAFETYLOOP_A,
    DI0_SAFETYLOOP_B,
    DI0_DLS1
} _rdien0;

typedef enum _rdien1_t {
    DI1_DCS2,
    DI1_DCS1,
    DI1_EED,
    DI1_EAD,
    DI1_DLS2,
    DI1_RESERVED1,
    DI1_RESERVED2,
    DI1_RESERVED3
} _rdien1;

typedef enum _MasterSlave_t {
	SLAVE_DCU,
	MASTER_DCU
} _MasterSlave;
typedef enum _F07Statement_t{
	F07_BRAKE,
	F07_OPEN ,
	F07_CLOSING,
	F07_OPENING
}_F07Statement;

typedef enum _F0408Statement_t{
	F0408_BRAKE,
	F0408_CLOSING,
}_F0408Statement;

typedef enum _OpenCloseConfig_time {
	Ms_2000 = 2000,
	Ms_2500 = 2500,
	Ms_3000 = 3000,
	Ms_3500 = 3500,
	Ms_4000 = 4000,
	Ms_4500 = 4500,
	Ms_5000 = 5000,
} _OpenCloseConfig;
typedef enum _CarDefineID{
	Define_MCAR = 0x18,
	Define_TCAR = 0x19,
	Define_NONE = 0x20
} _CarDefineIDConfig;

typedef enum _RtcSetdefine{
	SET_OK = 32,
	SET_NONE = 33
}_RtcSetDefine;

struct _adc {
    int32_t BemfVoltage;	/* = PowerVoltage(�����ΰ�����) - MotorVoltage(���Ϳ� �ɸ��� ����) */		/*## attribute BemfVoltage */
    int32_t MCUVoltage;		/* afec0_CH8 MCU 3.3V */		/*## attribute MCUVoltage */
    int32_t MotorCurrent;	/* afec0_CH4 Motor Current */		/*## attribute MotorCurrent */
    int32_t PowerVoltage;	/* Opening �� Afec0_Ch6, Closing �� Afec0_Ch5 */		/*## attribute PowerVoltage */
    int32_t MotorVoltage;	/* Opening �� Afec0_Ch5, Closing �� Afec0_Ch6 */		/*## attribute MotorVoltage */
};

struct _adcsamples {
    int32_t MotorCurrent[5];		/*## attribute MotorCurrent */
    int32_t PowerVoltage[5];		/*## attribute PowerVoltage */
    int32_t MotorVoltage[5];		/*## attribute MotorVoltage */
    uint32_t Afec0Ch5;		/*## attribute Afec0Ch5 */
    uint32_t Afec0Ch6;		/*## attribute Afec0Ch6 */
};

struct pwm_channel_t {
    uint32_t channel;		/*## attribute channel */
    uint32_t ul_prescaler;		/*## attribute ul_prescaler */
    uint32_t alignment;		/*## attribute alignment */
    uint32_t polarity;		/*## attribute polarity */
    uint32_t ul_duty;		/*## attribute ul_duty */
    uint32_t ul_period;		/*## attribute ul_period */
    uint32_t b_pwmh_output_inverted;		/*## attribute b_pwmh_output_inverted */
    uint32_t us_deadtime_pwmh;		/*## attribute us_deadtime_pwmh */
};

struct _dcu_id_t {
	uint8_t McarTcarPin;
	uint8_t CodingPin;
};

typedef struct _struct_RTCTIME {
	uint8_t ucSeconds;
	uint8_t ucMinutes;
	uint8_t ucHours;
	uint8_t ucDay;
	uint8_t ucDate;
	uint8_t ucMonth;
	uint8_t ucYears;
} RTCTIME;

struct _door_time {
	uint32_t OpeningStart;
	uint32_t OpeningPrintf;
	uint32_t Opened;
	uint32_t Opening;
	uint32_t ClosingStart;
	uint32_t ClosingPrintf;
	uint32_t Closed;
	uint32_t Closing;
	uint32_t OpenConfigtime;
	uint32_t CloseConfigtime;
};
extern int16_t Current_Velocity_value;
extern int16_t Ref_Velocity_value;
extern _Bool error_list_flag[17];
extern uint8_t error_list_count[17];
extern uint32_t error_list_time[17];
extern uint32_t error_485list_time[6];
extern uint16_t error_485_flag[6];
extern xSemaphoreHandle g_semaphoreBEMF;

extern int32_t m_CurrentPWMDuty;
extern int32_t OpenPowerValue;
extern int32_t ClosePowerValue;

extern _Bool m_ErrorFlag;

extern struct _door_time mdm_time;

extern uint32_t m_ObstacleOpenedTime;

extern int32_t m_OpeningEncoderPulse;

extern struct _adc m_adc;

extern struct _adcsamples m_adcsamples;

extern uint32_t m_afec0_data[4];
extern _RtcSetDefine m_RtcDefine;
extern _MasterSlave m_isMasterSlave;
extern _CarDefineIDConfig m_CardefineID;
extern _F07Statement m_F07Statement;
extern _F0408Statement m_F0408Statement;
extern uint32_t g_unDeviceID;
extern uint8_t m_isFaultCount;
extern _Bool m_isSlaveRunCommand;
extern _Bool m_isPacketSendToSlave;
extern _Bool m_is485changeflag;
extern _Bool m_isMasterSlaveChange;
extern _Bool m_isTaskExecution;
extern _Bool m_isTestEncoderflag;
extern uint8_t DLSSwitchOn;	// jeon_190710 DLS�Ϸὺ��ġ : DLS 2���� 1�� �̻� �׸��� DCS2���� 1�� �̻��� ���� ��� true
extern uint8_t DCSSwitchOn;	// jeon_190710 DCS�Ϸὺ��ġ : DLS 2���� 1�� �̻� �׸��� DCS2���� 1�� �̻��� ���� ��� true

RTCTIME rtc_get_time(void);
extern RTCTIME rtc_time;

extern int32_t motor_voltage;

#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mDataManage.h
*********************************************************************/
