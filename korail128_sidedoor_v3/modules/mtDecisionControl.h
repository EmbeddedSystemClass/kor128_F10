/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtDecisionControl
//!	Generated Date	: ≈‰, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtDecisionControl.h
*********************************************************************/

#ifndef mtDecisionControl_H
#define mtDecisionControl_H

#include <oxf/Ric.h>
#include "mDataManage.h"


extern _DoorState mdc_DoorState;
extern _DoorState mdc_PreDoorState;
extern _DoorState mdc_InitDoorState;
extern _Bool mdc_isInitComplete;
extern _Bool mdc_isOpeningByObstacle;
extern _Bool mdc_FlagTestDODBPS;
extern _Bool mdc_isFirstClosed;
extern _Bool mdc_isFirstOpened;

volatile uint32_t F10_OpenCount;


void mdc_TaskDecisionControl(void const * argument);

#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtDecisionControl.h
*********************************************************************/
