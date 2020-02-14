/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mDataManage
//!	Generated Date	: 토, 1, 7 2017  
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
 * 기본 소프트웨어 구조 emu250 enddoor 플랫폼에서 확인
 * Open/Close 정상동작 확인 (에뮬레이터 끼고 열면 Closing 시 Reset 됨 -> HW버그같음)
 * 
 * -------------------
 * v0.1.1 (2018.10.26, korail128_sidedoor_svn_v61)
 * 기본 동작 korail128 sidedoor 플랫폼에서 확인
 * Open/Close 정상동작 확인
 * 열림 중 열림버튼을 풀어 닫힘동작, 닫힘 중 열림버튼을 눌러 열림동작 확인
 * 장애물 3회 감지 후 완전열림 동작 확인
 * 차단스위치 차단 시 Free, 복귀 시 Closing 동작 확인 (버그: 2회째 차단 복귀 시 계속 밀고 있음)
 */
#define VERSION_MAJOR				(1)				// orig:1
#define VERSION_MINOR				(0)				// orig:0
#define VERSION_SECOND				(2)				// orig:2

#define RSWDT_PERIOD        		1000			// 1000ms
/*
 * 1펄스 당 도어 이동거리
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
#define Motor_Vmax					110				// 정격전압 = 110V 
#define Motor_Imax					2.1				// 정격전류 = 2.1A 

#define PWM_FREQUENCY				10000			// PWM 주파수
#define PWM_MAX						700				// 100V PWM 최대 출력
#define PWM_MIN						200				// 100V PWM 최소 출력
#define PERIOD_VALUE				1000			// PWM Duty 출력 0~1000

#define SPI_INSTRUCTION_DATA		0x00			// 0b.0000.0000 : Process Data 를 읽거나 쓰기 위한 명령 (F code 3, 지정포트 전용)
#define SPI_INSTRUCTION_LA_PCS		0x20			// 0b.0010.0000 : LA PCS 를 읽거나 쓰기 위한 명령 (지정 포트)
#define SPI_INSTRUCTION_CTR_REG		0x60			// 0b.0110.0000 : Control Register 를 읽거나 쓰기 위한 명령 (Src/Snk 포트지정)
#define SPI_INSTRUCTION_MSG0		0x80			// 0b.1000.0000 : Priority 0 의 Event 에 Message Data 를 읽거나 쓰기 위한 명령어
#define SPI_INSTRUCTION_MSG1		0xA0			// 0b.1010.0000 : Priority 1 의 Event 에 Message Data 를 읽거나 쓰기 위한 명령어
#define SPI_INSTRUCTION_MEMORY		0xC0			// 0b.1100.0000 : 전체 Memory 에 데이터를 읽거나 쓰기 위한 명령, 데이터는 최소 2B - 최대 32B 사이의 2B 단위의 데이터여야 한다.

#define SPI_INSTRUCTION_READ		0x00			// 읽기 설정
#define SPI_INSTRUCTION_WRITE		0x08			// 쓰기 설정

#define MVB_SLAVE_SOURCE			0x08			// 하위 4bit 중 첫번째 bit
#define MVB_SLAVE_SINK				0x04			// 하위 4bit 중 두번째 bit

#define MVB_FCODE0_2BYTE			0x00			// 2byte data length	0b.0000.0000
#define MVB_FCODE1_4BYTE			0x10			// 4byte data length	0b.0001.0000
#define MVB_FCODE2_8BYTE			0x20			// 8byte data length	0b.0010.0000
#define MVB_FCODE3_16BYTE			0x30			// 16byte data length	0b.0011.0000
#define MVB_FCODE4_32BYTE			0x40			// 32byte data length	0b.0100.0000

#define DCU1_MVB					0xf1			// 로터리 스위치 1에 있을 대 값이 0xf1임
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
 * 소사원시/수인선 코딩핀 입력 정보 (M_Car, T_Car 구분 없음)
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
 * Korail128 Test용 DCU정보
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

/* 오류코드, 고장코드 리스트 */
/*## type _ErrorCodeList */
#if 0
/*
 * EMU250
 */
typedef enum _ErrorCodeList_t {
    DCU_ERROR, /* 					F00 - DCU에러 */
    DCU_SELF_TEST_ERROR, /* 		F01 - 자가진단에러 */
    DCU_MOTOR_ERROR, /* 			F02 - 모터고장 */
    DCU_ENCODER_ERROR, /* 			F03 - 엔코더고장 */
    DCU_DCS_OPEN_CIRCUIT_ERROR, /* 	F04_1 - DCS Open Circuit 고장 */
    DCU_DCS_SHORT_CIRCUIT_ERROR, /* F04_2 - DCS Short Circuit 고장 */
    DCU_OBSTACLE_15, /* 			F05 - 장애물 15회 감지 (16회부터 오류) */
//    DCU_HIGH_VOLTAGE_ERROR, /* 	F06 - 130V이상 공급전압 감지  - 고객사 요청으로 삭제됨*/
//    DCU_LOW_VOLTAGE_ERROR, /* 	F07 - 65V이하 공급전압 감지  - 고객사 요청으로 삭제됨 */
    DCU_CLOSE_ERROR, /* 			F08 - 닫힘실패(닫힘동작시간초과) */
    DCU_OPEN_ERROR, /* 				F09 - 열림실패(열림동작시간초과) */
    DCU_OPEN_SWITCH_ERROR, /* 		F10 - 열림스위치 에러 */
} _ErrorCodeList;
#else


/* dcu 파라메타 값(속도,힘 등)*/

#if 1
#define DCU_DODBPS_POWER 			420 //jeon_190624 orig:420						//변수에 들어가는 값은 uint32_t
#define DCU_DODBPS_FREETIME			8000u
#define DCU_DODBPS_CLOSINGTIME		(DCU_DODBPS_FREETIME + 1000u)
#define DCU_SLOWMODE_POWER 			200u //jeon_190514 orig:250u				//변수에 들어가는 값은 uint32_t
#define DCU_OBSMODE_POWER 			300u //jeon_190619 orig:230u				//변수에 들어가는 값은 uint32_t 
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
	 * 오류플래그
	 * - 설명
	 *   : 오류가 발생될 때마다 해당오류와 함께 항상 오류플래그도 1로 설정됨
	 *   : FND Display Task에서 F01~F16 에러를 스캔해서 true인 에러가 0개이면 오류플래그는 자동으로 0으로 클리어 됨
	 *     -> 만약 특정 오류가 제거되면 해당 오류만 클리어 하면 되고, 전체 오류가 없을 경우에만 오류플래그가 자동으로 클리어 됨
	 */
	DCU_ERROR = 0u,
	
	/*
	 * F01 중고장
	 * -고장 설명
	 *   : 전원투입 직후 자가 테스트를 수행
	 *   : Closed 상태에서(Closing -> Closed 시)만 자가테스트 수행
	 *   : 자가테스트 도중 상태가 Opening으로 바뀌면 자가테스트 중단
	 * -고장발생 조건
	 *   : 고장모의 불가 -> 가변저항을 돌려 1에 위치시킴, 한번 발생하면 Clear되지 않음
	 * -고장복귀 조건
	 *   : 동작 중 고장복귀 없음
	 */
	DCU_HARD_FAULT =1u,
	
	/*
	 * F02 경고장
	 * -고장 설명
	 *   : 이전 소사원시에서 RS485 통신 불가 시에만 True로 설정 되고, 통신 복귀 시 자동으로 해당  Clear
	 * -고장발생 조건
	 *   : 고장모의 불가 -> 가변저항을 돌려 2에 위치시킴
	 * -고장복귀 조건
	 *   : 동작 중 고장복귀 없음
	 */
	DCU_MINOR_FAULT=2u,
	
	/*
	 * F03 모터 회로 고장
	 * -고장발생 조건 (확인하려면 지그가 무조껀 있어야 함)
	 *   : 열림(Opening) 또는 닫힘(Closing) 상태에서 엔코더 펄스가 변화하지 않고, 모터에 전류가 흐르지 않을 경우 모터고장
	 *     즉, 열림 명령(Opening, Closing출력)을 보냈는데 모터가 아예 움직이지 않는 경우
	 * -고장복귀 조건
	 *   : 동작 중 고장복귀 없음, 껏다 켜야지만 해당  Clear
	 */
	DCU_MOTOR_ERROR=3u,
	
	/*
	 * F04 DLS1고장
	 * -고장발생 조건 (DLS고장 시 엔코더,전류를 통한 Opened/Closed 판단 수행)
	 *   : 열린(Opened) 상태에서 DLS1이 눌려 있으면 고장 - Short Circuit
	 *   : 닫힌(Closed) 상태에서 DLS1이 눌리지 않으면 고장 - Open Circuit
	 * -고장복귀 조건
	 *   : 열린(Opened) 상태에서 DLS1이 눌리지 않으면 DLS1 고장해지
	 *   : 닫힌(Closed) 상태에서 DLS1이 눌리면 DLS1 고장해지
	 */
	DCU_DLS1_FAULT=4u,
	
	/*
	 * F05 DCS1고장
	 * -고장발생 조건 (DLS고장 시 엔코더,전류를 통한 Opened/Closed 판단 수행)
	 *   : 열린(Opened) 상태에서 DCS1이 눌린 경우가 3회 누적되면 DCS1고장 - Short Circuit
	 *   : 닫힌(Closed) 상태에서 DCS1이 눌려 있지 않는 경우가 3회 누적되면 DCS1고장 - Open Circuit
	 * -고장복귀 조건
	 *   : 열린(Opened) 상태에서 DCS1이 눌리지 않으면 DLS1 고장해지
	 *   : 닫힌(Closed) 상태에서 DCS1이 눌리면 DLS1 고장해지
	 */
	DCU_DCS1_FAULT=5u,
	
	/*
	 * F06 DCS2고장
	 * - DCS1과 같음
	 */
	DCU_DCS2_FAULT=6u,
	
	/*
	 * F07 예기치 못한 잠김 풀림
	 * -고장발생 조건
	 *   : 닫힌상태(Closed)에서 DLS를 살짝 당겨 DLS가 둘 다 눌리지 않으면 예기치 못한 잠김풀리(F07) 발생
	 * -고장복귀 조건
	 *   : 닫힌상태(Closed)에서 DLS가 둘 다 눌리면 F07 해지
	 */
	DCU_UNEXPECTED_UNLOCK=7u,
	
	/*
	 * F08 DLS2고장
	 * - DLS1과 같음
	 */
	DCU_DLS2_FAULT=8u,
	
	/*
	 * F09 닫힘중 장애감지 고장 (장애물 3회 감지)
	 * -고장발생 조건
	 *   : 장애물 3회 감지 시 완전열림
	 * -고장복귀 조건
	 *   : 닫힌(Closed)상태가 되면 고장 복귀
	 */
	DCU_OBSTACLE=9u,
	
	/*
	 * F10 도어 열림 실패 고장
	 * -고장발생 조건
	 *   : DLS가 Off되고 2s이내에 DCS가 Off 되지 않는 경우가 3회 누적되면 열림실패(F11) 발생
	 * -고장복귀 조건
	 *   : DLS가 Off되고 2s이내에 DCS가 Off 되면 열림실패(F11) 해지
	 */
	DCU_OPEN_FAULT=10u,
	
	/*
	 * F11 이중계 동작 중
	 * -고장 설명
	 *   : 
	 * -고장발생 조건
	 *   : 
	 * -고장복귀 조건
	 *   : 
	 */
	DCU_SLAVE_RUN=11u,
	
	/*
	 * F12 이중계 전환 불가
	 * -고장 설명
	 *   : Slave로부터 패킷을 수신하지 못하면 Slave(이중계) 고장으로 판단
	 * -고장발생 조건
	 *   : Slave 퓨즈 빼면 됨
	 *   : 통신 재개 시 해당  Clear
	 * -고장복귀 조건
	 *   : 
	 */
	DCU_CANT_SLAVE=12u,
	
	/*
	 * F13 엔코더 고장
	 * -고장발생 조건 (확인하려면 지그가 무조껀 있어야 함)
	 *   : 열림완료 시 엔코더 펄스가 정해진 값보다 적거나 많이 나오는 경우가 3회 누적되면 엔코더고장 발생
	 *     - 엔코더 펄스가 정해진 값보다 많이 발생해 엔코더를 통해 열림완료를 판단한 경우 엔코더고장
	 *     - 엔코더 펄스가 정해진 값보다 적게 발생해 전류를 통해 열림완료를 판단한 경우 엔코더고장
	 *   : Opening 시 지그로 엔코더 선 끊으면 전류로 멈추고 F13
	 * -고장복귀 조건
	 *   : 열림완료 시 엔코더 펄스가 정해진 값으로 나오면 엔코더고장 해지
	 */
	DCU_ENCODER_ERROR=13u,
	
	/*
	 * F14 Safety Loop 이상
	 * -고장발생 조건
	 *   : 지그로 확인
	 * -고장복귀 조건
	 *   : 
	 */
	DCU_SAFETYLOOP_FAULT=14u,
	
	/*
	 * F15 MVB통신 고장
	 * -고장 설명
	 *   : TCMS와 MVB간 통신 실패 시 고장
	 * -고장발생 조건
	 *   : 일정 시간 간격으로 패킷 미수신 시 통신고장
	 * -고장복귀 조건
	 *   : 통신 재개 시 고장 복귀
	 */
	DCU_MVB_ERROR=15u,
	
	/*
	 * F16 RS-485통신 고장
	 * -고장 설명
	 *   : 
	 * -고장발생 조건
	 *   : Master DCU와 Slave DCU 간 RS485 통신 실패 시 고장
	 * -고장복귀 조건
	 *   : 통신 재개 시 고장 복귀
	 */
	DCU_RS485_ERROR=16u,
	
	/*
	 * F17 잠김 실패 -> 없음
	 * -고장발생 조건
	 *   : DCS가 On되고, 2s 이내에 DLS가 On 되지 않는 경우가 3회 누적되면 잠김실패(F17) 발생
	 *   : 인위적으로 손봐야 하므로 절체 후 F17 다시 발생하면 재부팅으로만 F17 Clear 되어야 함.
	 * -고장복귀 조건
	 *   : DCS가 On되고, 2s 이내에 DLS가 On되면 잠김실패(F17) 해지
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
	DCU_EAD_EVENT = 21,					//외부 비상핸들
	DCU_EED_EVENT = 22,					//내부 비상핸들
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

/* 도어상태 */
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
    DOOR_POSITION_CLOSED_BY_END,			// 정상적으로 전체 이동거리만큼 이동하고 닫힘완료 -> 전체 거리에서 닫힘(Closing) 가속/등속/감속 거리 설정
    DOOR_POSITION_OPENED_BY_END,			// 정상적으로 전체 이동거리만큼 이동하고 열림완료 -> 전체 거리에서 닫힘(Closing) 가속/등속/감속 거리 설정
    DOOR_POSITION_CLOSED_BY_BUTTON,		// 닫힘 중 열림버튼을 눌러 다시 열기 위해 닫힘완료
    DOOR_POSITION_CLOSED_BY_OBSTACLE,	// 닫힘 중 장애물감지로 다시 열기 위해 닫힘완료
    DOOR_POSITION_OPENED_BY_BUTTON,		// 열림 중 열림버튼을 풀어 다시 닫기위해 열림완료
    DOOR_POSITION_OPENED_BY_OBSTACLE,	// 열림 중 장애물감지로 시 살짝 이동하고 다시 닫기위해 열림완료
} _DoorDistance;

/* Opening 시	- Afec0_Ch6(Supply Voltage), Afec0_Ch5(Bemf Voltage) */
/* Closing 시	- Afec0_Ch5(Supply Voltage), Afec0_Ch6(Bemf Voltage) */
/*## type _AfecChValue */
typedef enum _AfecChValue_t {
    CH4_MOTOR_CURR, /* PB0(AFEC0_AD4) - 모터전류 */
    CH5_BEMF_R, /* PB1(AFEC0_AD5) - BEMF전압(정) */
    CH6_BEMF_F, /* PB2(AFEC0_AD6) - BEMF전압(역) */
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
    int32_t BemfVoltage;	/* = PowerVoltage(모터인가전압) - MotorVoltage(모터에 걸리는 전압) */		/*## attribute BemfVoltage */
    int32_t MCUVoltage;		/* afec0_CH8 MCU 3.3V */		/*## attribute MCUVoltage */
    int32_t MotorCurrent;	/* afec0_CH4 Motor Current */		/*## attribute MotorCurrent */
    int32_t PowerVoltage;	/* Opening 시 Afec0_Ch6, Closing 시 Afec0_Ch5 */		/*## attribute PowerVoltage */
    int32_t MotorVoltage;	/* Opening 시 Afec0_Ch5, Closing 시 Afec0_Ch6 */		/*## attribute MotorVoltage */
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
extern uint8_t DLSSwitchOn;	// jeon_190710 DLS완료스위치 : DLS 2개중 1개 이상 그리고 DCS2개중 1개 이상이 눌린 경우 true
extern uint8_t DCSSwitchOn;	// jeon_190710 DCS완료스위치 : DLS 2개중 1개 이상 그리고 DCS2개중 1개 이상이 눌린 경우 true

RTCTIME rtc_get_time(void);
extern RTCTIME rtc_time;

extern int32_t motor_voltage;

#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mDataManage.h
*********************************************************************/
