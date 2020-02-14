/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mLibrary
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mLibrary.h
*********************************************************************/

#ifndef mLibrary_H
#define mLibrary_H

#include "main.h"
#include "stm32f7xx_hal.h"
#include "stdio.h"
#include <oxf/Ric.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "mDataManage.h"
#include "cmsis_os.h"
#include "mtInputProcessing.h"

/* 모든 모듈이 include하므로 특정 변수를 선언하면 multiple definition으로 컴파일 에러 발생하므로 */
/* 구조체와 같은 데이터 형 타입이나 공통으로 호출되는 라이브러리 함수만 정의되어야 한다. */

#define high	1
#define low		0
#define true	1
#define false	0

#define BIT_SET(p,n) ((p) |= (1 << (n)))
#define BIT_CLEAR(p,n) ((p) &= (~(1) << (n)))

void print_float(uint32_t ch, float temp);
_Bool getAbit(uint8_t data, uint8_t bit);
uint32_t get_diff_tick(uint32_t curr_tick, uint32_t start_tick);
void mram_write_enable();
void mram_write_diable();
void mram_byte_write(uint32_t addr, uint8_t data);
void mram_block_write(void);
uint8_t mram_byte_read(uint32_t addr);
void mram_erase(uint32_t addr, uint32_t size);
void mram_erase_Fault_data(void);
void Erase_fault(uint8_t error_code);
#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mLibrary.h
*********************************************************************/
