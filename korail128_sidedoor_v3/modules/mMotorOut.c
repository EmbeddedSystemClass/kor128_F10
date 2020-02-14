/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mMotorOut
//!	Generated Date	: 토, 1, 7 2017  
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

/* true : H-bridge와 GND 연결 */
/* false : H-bridge와 GND 끊어짐 */
void mmo_ConnectHbridgeGND(_Bool connect) {
    if(connect == true)
    {
		#ifdef HW_DEPENDENT_CODE
			HAL_GPIO_WritePin(MMPWR_CTL_GPIO_Port, MMPWR_CTL_Pin, GPIO_PIN_SET); 	// Connect - 연결
		#endif
    }
    else
    {
		#ifdef HW_DEPENDENT_CODE
			HAL_GPIO_WritePin(MMPWR_CTL_GPIO_Port, MMPWR_CTL_Pin, GPIO_PIN_RESET);	// Disconnect - 연결해지
		#endif
    }
}

/* 4개 FET On, */
/* N채널 FET2, FET4 Min PWM으로 둘 다 출력해서 잡고 있음 */
void mmo_DoorBrake(void) {
    /*
     * #EDCU-SW-NFR-P-02
     */
    g_CurrentPWM_Ch = PWM_CH_NO_SELECTE;								// PWM Update를 그만하게 하고, PWM 출력을 Disable 시켜야 함
//    mmo_MotorDisable();
    
	#ifdef HW_DEPENDENT_CODE
    {
		/*
		 * Motor Brake FET출력 파형
		 * 	                             VCC
		 * 	                              |
		 * 	                              |
		 *							|-----|-----|  
		 * 	Low->전압드랍 없음		  \               /   Low->전압드랍 없음
		 * 	(P채널PWM1 FET Off)	---\  		     /--- (P채널PWM3 FET Off)
		 * 	TIM12_ch2              A\--(Motor)--/B	  TIM5_ch2
		 *							|           |
		 *						    |           |	
		 *  Low->반전해서high		----|           |---- Low->반전해서high
		 * 	(N채널PWM2 FET On)		|-----|-----|     (N채널PWM4 FET Off)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 */
		#if 0
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);				// PWM1 - P채널 FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);					// PWM4 - N채널 FET Off
			
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3 - P채널 FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);					// PWM2 - N채널 FET Off
		#else
			/*
			 * N채널 Brake
			 */
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);					// PWM1(Active High) - P채널 FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1000);				// PWM4(Active Low)  - N채널 FET On : PWM 1000 -> Low가 100% 출력 -> 반전 High되서 Nch FET Turn ON
			
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3(Active High) - P채널 FET Off
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1000);				// PWM2(Active Low)  - N채널 FET On : PWM 1000 -> Low가 100% 출력 -> 반전 High되서 Nch FET Turn ON
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
		 * 	Low->전압드랍 없음		  \             |     High->전압드랍
		 * 	(P채널PWM1 FET Off)	---\  		    |---- (P채널PWM3 FET On)
		 * 	TIM12_ch2              A\--(Motor)--|B	  TIM5_ch2
		 *							|           |
		 *						    |             /	
		 *  PWM					----|            /--- Low->반전해서high
		 * 	(N채널PWM2 FET PWM출력)	|-----|-----/     (N채널PWM4 FET Off)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 */
    	/*
    	 * TIM5_ch1 PWM2출력 전 A위치 ADC결과 High가 떠 있으면 Low로 떨어질 때까지 기다려야 함.
    	 */
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1000);				// PWM3 - P채널 FET On
		if(pwmSetValue>PWM_MAX)	pwmSetValue = PWM_MAX;
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwmSetValue);		// PWM2 - N채널 FET PWM
	
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);				// PWM1 - P채널 FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);					// PWM4 - N채널 FET Off
	}
	#endif
    MotorEnable();
    g_CurrentPWM_Ch = PWM_CH3_CLOSING_REVERSE;							// 반드시 PWM출력설정을 완효한 다음에 Vref Generator에서 이 값을 참조할 수 있도록 해야 함.
}

/* 4개 FET 모두 Off */
void mmo_DoorFree(void) {
    g_CurrentPWM_Ch = PWM_CH_NO_SELECTE;								// PWM Update를 그만하게 하고, PWM 출력을 Disable 시켜야 함
	#ifdef HW_DEPENDENT_CODE
    {
		/*
		 * Motor Free FET출력 파형
		 * 	                             VCC
		 * 	                              |
		 * 	                              |
		 *							|-----|-----|  
		 * 	Low->전압드랍 없음		  \               /   Low->전압드랍 없음
		 * 	(P채널PWM1 FET Off)	---\  		     /--- (P채널PWM3 FET Off)
		 * 	TIM12_ch2              A\--(Motor)--/B	  TIM5_ch2
		 *							|           |
		 *						  \               /	
		 *  Low->반전해서high		---\             /--- Low->반전해서high
		 * 	(N채널PWM2 FET Off)		\-----|-----/     (N채널PWM4 FET Off)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 * 
		 */
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 0);					// PWM1(Active High) - P채널 FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);					// PWM4(Active Low)  - N채널 FET Off : PWM 0 -> Low가 0% 출력 == High 100% -> 반전 High되서 Nch FET Turn ON
		
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3(Active High) - P채널 FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);					// PWM2(Active Low)  - N채널 FET Off : PWM 0 -> Low가 0% 출력 == High 100% -> 반전 High되서 Nch FET Turn ON
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
		 * 	High->전압드랍		    |             /   Low->전압드랍 없음
		 * 	(P채널PWM1 FET On)	----| 		     /--- (P채널PWM3 FET Off)
		 * 	TIM12_ch2              A|--(Motor)--/B	  TIM5_ch2
		 *							|           |
		 *						  \             |  	
		 *  Low->반전해서high		---\            |---- PWM
		 * 	(N채널PWM2 FET Off)	    \-----|-----|     (N채널PWM4 FET PWM출력)
		 * 	TIM5_ch1					  |           TIM5_ch3
		 * 	                             ===
		 * 	                             GND
		 */
    	/*
    	 * TIM5_ch3 PWM4출력 전 B위치 ADC결과 High가 떠 있으면 Low로 떨어질 때까지 기다려야 함.
    	 */
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1000);				// PWM1 - P채널 FET On
		if(pwmSetValue>PWM_MAX)	pwmSetValue = PWM_MAX;
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwmSetValue);		// PWM4 - N채널 FET PWM
		
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);					// PWM3 - P채널 FET Off
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);					// PWM2 - N채널 FET Off (극성이 Low -> duty 0은 all High, duty 20%는 80%는 High, 20%는 Low)
	}
	#endif
    MotorEnable();
    g_CurrentPWM_Ch = PWM_CH1_OPENING_FORWARD;							// 반드시 PWM출력설정을 완효한 다음에 Vref Generator에서 이 값을 참조할 수 있도록 해야 함. 
}

void mmo_MotorDisable(void) {
	#ifdef HW_DEPENDENT_CODE
		HAL_GPIO_WritePin(MShutDown_GPIO_Port, MShutDown_Pin, GPIO_PIN_SET);		// 최종적으로 AND Gate에 Low출력이 나가서 PWM출력이 H-Bridge로 나가지 않음
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
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwmValue);				// PWM4 - N채널 FET PWM
}

void mmo_DoorClosingNchFET(uint32_t pwmValue)
{
	if(pwmValue>PWM_MAX)	pwmValue = PWM_MAX;
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwmValue);				// PWM2 - N채널 FET PWM
}

uint32_t mmo_getEncoderCountValue(void)
{
	uint32_t Encoder_Value =0;
	Encoder_Value = __HAL_TIM_GET_COUNTER(&htim3);
	if(Encoder_Value > 9000) Encoder_Value = 0;  //9000이상 넘는 경우는 무조건 언더플로우 (CLOSING 상황에서만 발생)
	return Encoder_Value;
}

static void MotorEnable(void) {
	#ifdef HW_DEPENDENT_CODE
		HAL_GPIO_WritePin(MShutDown_GPIO_Port, MShutDown_Pin, GPIO_PIN_RESET); // Low 출력 -> AND Gate true 입력 -> PWM이 H-Bridge로 출력 됨
	#endif
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mMotorOut.c
*********************************************************************/
