/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtCommunication
//!	Generated Date	: ≈‰, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtCommunication.h
*********************************************************************/

#ifndef mtCommunication_H
#define mtCommunication_H

#include <oxf/Ric.h>
#include "mLibrary.h"


typedef uint32_t (*callback_function)(uint8_t* string, uint8_t ucNumOfParameters);

void callback_version(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_gpio(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_state(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_motor_pwm(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_dcu(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_door_open(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_fault(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_door_free(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_reset(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_mram(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_obs(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_debug(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_Switch(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_DCUID(int8_t* string, uint8_t ucNumOfParameters);
void callback_test_RTCSET(int8_t* string, uint8_t ucNumOfParameters);
void mc_TaskCLICommand(void const * argument);
void mc_TaskMVB(void const * argument);
void mac_TaskAliveCheck(void const * argument);
void mac_TaskRS485(void const * argument);
uint8_t Statement_DCU(uint8_t byte_address);
uint8_t FalutStatement_DCU(uint8_t byte_address);
uint8_t *Return_DCU_R_Value(uint8_t dcu_id);
uint8_t *Return_DCU_L_Value(uint8_t dcu_id);
void SlaveRunning_Check(osEvent Pevent);
void StateData_Send(uint8_t arr[]);
void MemHandling_data(uint8_t arr[],uint16_t size);
void Comm_Fault_tx(uint16_t block_num);
void mvbSourceConfigurationRead(uint8_t Fcode, uint8_t PortNumber);
void mvbSourceConfiguration(uint8_t Fcode, uint8_t PortNumber);
void mvbSourceWrite(uint8_t PortNumber);
void mvbSinkConfiguration(uint8_t Fcode, uint8_t PortNumber);
void mvbSinkRead(uint8_t Fcode, uint8_t PortNumber);

#endif
/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtCommunication.h
*********************************************************************/
