/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtObstacleDetect
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtObstacleDetect.h
*********************************************************************/

#ifndef mtObstacleDetect_H
#define mtObstacleDetect_H

#include <oxf/Ric.h>
#include "mLibrary.h"

struct _detect {
    _Bool passenger;/* True(���న��), False(None) */		/*## attribute passenger */
    _Bool isopening;/* True(��ְ�������), False(None) */		/*## attribute isopening */
    int8_t ObstacleDetectCnt;
    uint16_t ObstacleConfigValue;
};

extern struct _detect mod_Detect;

void mod_TaskObstacleDetect(void const * argument);


#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtObstacleDetect.h
*********************************************************************/
