/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtMonitoring
//!	Generated Date	: ≈‰, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMonitoring.h
*********************************************************************/

#ifndef mtMonitoring_H
#define mtMonitoring_H

#include <oxf/Ric.h>
#include "mLibrary.h"

#define ADC_12_BIT		4095UL
#define ADC_VOLT_REF	3300

struct adcResults {
    float value;		/*## attribute value */
    _Bool finished;		/*## attribute finished */
};
struct DebugPrint{
	uint8_t Data_Print_flag;
};
typedef enum fndsel_t {
    FND_0,		// 0
    FND_1,		// 1
    FND_2,		// 2
    FND_3,		// 3
    FND_4,		// 4
    FND_5,		// 5
    FND_6,		// 6
    FND_7,		// 7
    FND_8,		// 8
    FND_9,		// 9
    FND_F,		// 10
    FND_a,		// 11
    FND_b,		// 12
    FND_V,		// 13
    FND_E,		// 14
    FND_r,		// 15
    FND_R,		// 16
    FND_L,		// 17
    FND_M,		// 18
    FND_T,		// 19
    FND_NONE	// 20
} fndsel;
extern uint8_t m_FND[21];
extern struct _dcu_id_t dcuID;
extern struct adcResults g_afec0_ch4;
extern struct adcResults g_afec0_ch5;
extern struct adcResults g_afec0_ch6;
extern struct adcResults g_afec0_ch8;
extern struct DebugPrint debugprint;
void DisplayMonitoringInfo(uint32_t info);
void md_TaskDisplayFND(void const * argument);
void mm_TaskMonitoring(void const * argument);
void mm_SaveCurrentDcuData(void);
void mm_ReadFaultDcuData(void);
void mm_SaveFaultDcuData(uint8_t ErrorCode);
static void DiagnosisInMonitoring(void);
void DisplaySWVersionToFND(uint8_t major, uint8_t minor, uint8_t second);
static void SupplyVoltageMonitor(void);
static void DiagnosisInDecisionControl(uint8_t doorstate, _Bool dcs);
void FaultFlag_SET(_ErrorCodeList error_code);
void mram_Event_save(_EventCodeList event_code);
void mm_DLSFault_Dectetion(void);
void mm_DCSFault_Dectetion(void);
void mm_Mo_EnFault_Dectetion(void);
void mm_F07fault_Dectection(void);
void mm_SafetyFault_Dectection(void);
void mm_ObsFault_Dectection(void);
void mm_OpenFault_Detection(void);
void mm_HardFault_Detection(void);
void mm_485_Change_Detection(void);
#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMonitoring.h
*********************************************************************/
