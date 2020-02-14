/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mMotorOut
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mMotorOut.c
*********************************************************************/

#include "mMotorOut.h"
#include "mtMotorFeedback.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

_pwmCh g_CurrentPWM_Ch = 0;
struct pwm_channel_t g_pchFET_ctla_pwmh[4];
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

static void MotorEnable(void);

/* true : H-bridge�� GND ���� */
/* false : H-bridge�� GND ������ */
void mmo_ConnectHbridgeGND(_Bool connect) {
    if(connect == true)
    {
		#ifdef HW_DEPENDENT_CODE
			HAL_GPIO_WritePin(MMPWR_CTL_GPIO_Port, MMPWR_CTL_Pin, GPIO_PIN_SET); 	// Connect - ����
		#endif
    }
    else
    {
		#ifdef HW_DEPENDENT_CODE
			HAL_GPIO_WritePin(MMPWR_CTL_GPIO_Port, MMPWR_CTL_Pin, GPIO_PIN_RESET);	// Disconnect - ��������
		#endif
    }
}

/* 4�� FET On, */
/* Nä�� FET2, FET4 Min PWM���� �� �� ����ؼ� ��� ���� */
void mmo_DoorBrake(void) {
    /*
     * #EDCU-SW-NFR-P-02
     */
    g_CurrentPWM_Ch = PWM_CH_NO_SELECTE;								// PWM Update�� �׸��ϰ� �ϰ�, PWM ����� Disable ���Ѿ� ��
//    mmo_MotorDisable();
    
	#ifdef HW_DEPENDENT_CODE
    {
		/*
		 * Motor Brake FET��� ����
		 * 	                             VCC
		 * 	                              |
		 * 	                              |
		 *							|-----|-----|  
		 * 	Low->���е�� ����		  \               /   Low->���е�� ����
		 * 	(Pä��PWM1 FET Off)	---\  		     /--- (Pä��PWM3 FET Off)
		 * 	TIM12_ch2              A\--(Motor)--/B	  TIM5_ch2
		 *							|           |
		 *						    |           |	
		 *  Low->�����ؼ�high		----|           |---- Low->�����ؼ�high
		 * 	(Nä��PWM2 FET On)		|-----|-----|     (Nä��PWM4 FET Off)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 */
		#if 0
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);				// PWM1 - Pä�� FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);					// PWM4 - Nä�� FET Off
			
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3 - Pä�� FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);					// PWM2 - Nä�� FET Off
		#else
			/*
			 * Nä�� Brake
			 */
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);					// PWM1(Active High) - Pä�� FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1000);				// PWM4(Active Low)  - Nä�� FET On : PWM 1000 -> Low�� 100% ��� -> ���� High�Ǽ� Nch FET Turn ON
			
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3(Active High) - Pä�� FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1000);				// PWM2(Active Low)  - Nä�� FET On : PWM 1000 -> Low�� 100% ��� -> ���� High�Ǽ� Nch FET Turn ON
		#endif
    }
	#endif
}

void mmo_DoorClosing(uint32_t pwmSetValue) {
    mmo_MotorDisable();
    
	#ifdef HW_DEPENDENT_CODE
	{
		/*
		 * Reverse (Closing)
		 * 	                             VCC
		 * 	                              |
		 * 	                              |
		 *							|-----|-----|  
		 * 	Low->���е�� ����		  \             |     High->���е��
		 * 	(Pä��PWM1 FET Off)	---\  		    |---- (Pä��PWM3 FET On)
		 * 	TIM12_ch2              A\--(Motor)--|B	  TIM5_ch2
		 *							|           |
		 *						    |             /	
		 *  PWM					----|            /--- Low->�����ؼ�high
		 * 	(Nä��PWM2 FET PWM���)	|-----|-----/     (Nä��PWM4 FET Off)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 */
    	/*
    	 * TIM5_ch1 PWM2��� �� A��ġ ADC��� High�� �� ������ Low�� ������ ������ ��ٷ��� ��.
    	 */
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1000);				// PWM3 - Pä�� FET On
		if(pwmSetValue>PWM_MAX)	pwmSetValue = PWM_MAX;
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwmSetValue);		// PWM2 - Nä�� FET PWM
	
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);				// PWM1 - Pä�� FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);					// PWM4 - Nä�� FET Off
	}
	#endif
    MotorEnable();
    g_CurrentPWM_Ch = PWM_CH3_CLOSING_REVERSE;							// �ݵ�� PWM��¼����� ��ȿ�� ������ Vref Generator���� �� ���� ������ �� �ֵ��� �ؾ� ��.
}

/* 4�� FET ��� Off */
void mmo_DoorFree(void) {
    g_CurrentPWM_Ch = PWM_CH_NO_SELECTE;								// PWM Update�� �׸��ϰ� �ϰ�, PWM ����� Disable ���Ѿ� ��
	#ifdef HW_DEPENDENT_CODE
    {
		/*
		 * Motor Free FET��� ����
		 * 	                             VCC
		 * 	                              |
		 * 	                              |
		 *							|-----|-----|  
		 * 	Low->���е�� ����		  \               /   Low->���е�� ����
		 * 	(Pä��PWM1 FET Off)	---\  		     /--- (Pä��PWM3 FET Off)
		 * 	TIM12_ch2              A\--(Motor)--/B	  TIM5_ch2
		 *							|           |
		 *						  \               /	
		 *  Low->�����ؼ�high		---\             /--- Low->�����ؼ�high
		 * 	(Nä��PWM2 FET Off)		\-----|-----/     (Nä��PWM4 FET Off)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 * 
		 */
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);					// PWM1(Active High) - Pä�� FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);					// PWM4(Active Low)  - Nä�� FET Off : PWM 0 -> Low�� 0% ��� == High 100% -> ���� High�Ǽ� Nch FET Turn ON
		
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3(Active High) - Pä�� FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);					// PWM2(Active Low)  - Nä�� FET Off : PWM 0 -> Low�� 0% ��� == High 100% -> ���� High�Ǽ� Nch FET Turn ON
	}
    #endif
    MotorEnable();
}

void mmo_DoorOpening(uint32_t pwmSetValue) {
    mmo_MotorDisable();
	#ifdef HW_DEPENDENT_CODE
	{
		/*
		 * Forward (Opening)
		 * 	                             VCC
		 * 	                              |
		 * 	                              |
		 *							|-----|-----|  
		 * 	High->���е��		    |             /   Low->���е�� ����
		 * 	(Pä��PWM1 FET On)	----| 		     /--- (Pä��PWM3 FET Off)
		 * 	TIM12_ch2              A|--(Motor)--/B	  TIM5_ch2
		 *							|           |
		 *						  \             |  	
		 *  Low->�����ؼ�high		---\            |---- PWM
		 * 	(Nä��PWM2 FET Off)	    \-----|-----|     (Nä��PWM4 FET PWM���)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 */
    	/*
    	 * TIM5_ch3 PWM4��� �� B��ġ ADC��� High�� �� ������ Low�� ������ ������ ��ٷ��� ��.
    	 */
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1000);				// PWM1 - Pä�� FET On
		if(pwmSetValue>PWM_MAX)	pwmSetValue = PWM_MAX;
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwmSetValue);		// PWM4 - Nä�� FET PWM
		
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3 - Pä�� FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);					// PWM2 - Nä�� FET Off (�ؼ��� Low -> duty 0�� all High, duty 20%�� 80%�� High, 20%�� Low)
	}
	#endif
    MotorEnable();
    g_CurrentPWM_Ch = PWM_CH1_OPENING_FORWARD;							// �ݵ�� PWM��¼����� ��ȿ�� ������ Vref Generator���� �� ���� ������ �� �ֵ��� �ؾ� ��. 
}

void mmo_MotorDisable(void) {
	#ifdef HW_DEPENDENT_CODE
		HAL_GPIO_WritePin(MShutDown_GPIO_Port, MShutDown_Pin, GPIO_PIN_SET);		// ���������� AND Gate�� Low����� ������ PWM����� H-Bridge�� ������ ����
	#endif
}

void mmo_MotorPositionReset(void)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

void mmo_MotorPWMStart(void)
{
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);									// PWM1
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);									// PWM2
	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);									// PWM3
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);									// PWM4	
}

void mmo_MotorPWMStop(void)
{
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);									// PWM1
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);									// PWM2
	
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);									// PWM3
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);									// PWM4
}

void mmo_DoorOpeningNchFET(uint32_t pwmValue)
{
//jeon_190812	if(pwmValue>PWM_MAX)	pwmValue = PWM_MAX;
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwmValue);				// PWM4 - Nä�� FET PWM
}

void mmo_DoorClosingNchFET(uint32_t pwmValue)
{
	if(pwmValue>PWM_MAX)	pwmValue = PWM_MAX;
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwmValue);				// PWM2 - Nä�� FET PWM
}

uint32_t mmo_getEncoderCountValue(void)
{
	uint32_t Encoder_Value =0;
	Encoder_Value = __HAL_TIM_GET_COUNTER(&htim3);
	if(Encoder_Value > 9000) Encoder_Value = 0;  //9000�̻� �Ѵ� ���� ������ ����÷ο� (CLOSING ��Ȳ������ �߻�)
	return Encoder_Value;
}

static void MotorEnable(void) {
	#ifdef HW_DEPENDENT_CODE
		HAL_GPIO_WritePin(MShutDown_GPIO_Port, MShutDown_Pin, GPIO_PIN_RESET); // Low ��� -> AND Gate true �Է� -> PWM�� H-Bridge�� ��� ��
	#endif
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mMotorOut.c
*********************************************************************/
