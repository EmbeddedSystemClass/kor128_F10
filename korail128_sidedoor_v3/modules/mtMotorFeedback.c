/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtMotorFeedback
//!	Generated Date	: 토, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.c
*********************************************************************/

#include "mtMotorFeedback.h"
/* 도어 상태에 따라 엔코더펄스 +/- 카운트를 위해 인클루드 */
/*## dependency mtDecisionControl */
#include "mtDecisionControl.h"
/* DCS 고장진단 판단을 위해 인클루드 */
/*## dependency mtInputProcessing */
#include "mtInputProcessing.h"
/*## dependency mtObstacleDetect */
#include "mtObstacleDetect.h"
#include "mMotorOut.h"


#ifdef EMC_TEST
struct _Encoder mmf_Encoder = {0,0,0,0,0,0,200,200};
#else
struct _Encoder mmf_Encoder = {0,0,0,0,0,0,0,0,0};
#endif
VelocitySection OpeningSection = {0,0,0};
VelocitySection ClosingSection = {0,0,0};
struct _FlagEnd mmf_EndDecision = {false, false, false, false};
struct _Bemf mmf_Bemf = {0,0,0,0};
_pidControlState mmf_NextMotorDirSelect = 0;
_pidControlState mmf_PIDControl = 0;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
static float PID_Velocity;
volatile static int32_t motorOpenPower=7;																// max 100까지 하면 PWM 800이상 출력 됨
volatile static int32_t motorClosePower=3;

uint8_t bemf_task_flag=false;
#if 1
void mmf_ConversionADC(void) {
    /*
     * ADC 변환 시작
     */

	#ifdef HW_DEPENDENT_CODE
	/*
	 * todo : ADC 정상적으로 되는지 확인해 봐야 함
	 */
	if(HAL_ADC_Start(&hadc1) == HAL_OK)
	{
		/*
		 * MBEMF_F_ADC_IN - PF10, ADC3_IN8(ADC_CHANNEL_8)
		 */
		if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)	m_adc.MotorVoltage = HAL_ADC_GetValue(&hadc1);
		/*
		 * M_MOTOR_CURRENT - PA5, ADC1_IN5(ADC_CHANNEL_5)
		 */
		if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)	m_adc.MotorCurrent = HAL_ADC_GetValue(&hadc1);
		/*
		 * MBEMF_R_ADC_IN - PB1, ADC1_IN9(ADC_CHANNEL_9)
		 */
		if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)	m_adc.PowerVoltage = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	
	if(m_adc.MotorVoltage > m_adc.PowerVoltage)
	{
		motor_voltage = m_adc.MotorVoltage;
	}
	else if(m_adc.MotorVoltage < m_adc.PowerVoltage)
	{
		motor_voltage = m_adc.PowerVoltage;
	}
	else
	{
		motor_voltage = m_adc.MotorVoltage;
	}
	
	
	/*
	 * M3_3V_MONITOR - PA4, ADC1_IN4(ADC_CHANNEL_4)
	 *  - 3.3V : 4095 = 2.6V : y
	 *  - y = (4095*2.6)/3.3 = about 3226
	 */
	if(HAL_ADC_Start(&hadc3) == HAL_OK)
	{
		if(HAL_ADC_PollForConversion(&hadc3, 100)==HAL_OK)	m_adc.MCUVoltage = HAL_ADC_GetValue(&hadc3);
		HAL_ADC_Stop(&hadc3);
	}
    #endif
}
#else
void mmf_ConversionADC(void)
{
	/*
	 * ADC Conversion Sequence : ADC12_IN4 -> ADC12_IN5 -> ADC12_IN9 -> ADC3_IN8
	 * 
	 * 	FP10(ADC3_IN8)	+- 3.3V Monitor
	 * 					|
	 * 	PB1	(ADC12_IN9)	+- Bemf_R_ADC_IN
	 * 					|
	 * 	PA5	(ADC12_IN5)	+- Motor_Current
	 * 					|
	 * 	PA4	(ADC12_IN4)	+- Bemf_F_ADC_IN
	 */
	/*
	 * 모터인가전압	= 모터에걸리는전압 + Back_emf전압
	 *                PWM입력으로 전압이 계속 바뀜
	 * 
	 * PowerVoltage = MotorVoltage  + V_bemf
	 * 
	 * 	M3_3V_MONITOR - PA4, ADC1_IN4
	 *  - 3.3V : 4095 = 2.6V : y
	 *  - y = (4095*2.6)/3.3 = about 3226
	 */
	#ifdef DEBUG_SIMULATOR
		volatile static int16_t currEncPulse=0, prevEncPulse=0;												// 1ms irs호출 시마다 현재 엔코더 펄스 업데이트
		currEncPulse = mmo_getEncoderCountValue();
		
		if((mmf_PIDControl==PID_CONTROL_OPENING)||
		   (currEncPulse>prevEncPulse))						// Free에서 손으로 돌릴 때 엔코더 펄스 변화를 보고 자동으로 ADC수행
		{
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN4(ADC_CHANNEL_4)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorCurrent = HAL_ADC_GetValue(&hadc1);	// ADC12_IN5(ADC_CHANNEL_5)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.PowerVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN9(ADC_CHANNEL_9)
			
			HAL_ADC_Start(&hadc3);
			if(HAL_ADC_PollForConversion(&hadc3, 1000)==HAL_OK)	m_adc.MCUVoltage = HAL_ADC_GetValue(&hadc3);	// ADC3_IN8(ADC_CHANNEL_8)
		}
		else if((mmf_PIDControl==PID_CONTROL_CLOSING)||
				(currEncPulse<prevEncPulse))				// Free에서 손으로 돌릴 때 엔코더 펄스 변화를 보고 자동으로 ADC수행
		{
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.PowerVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN4(ADC_CHANNEL_4)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorCurrent = HAL_ADC_GetValue(&hadc1);	// ADC12_IN5(ADC_CHANNEL_5)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN9(ADC_CHANNEL_9)
			
			HAL_ADC_Start(&hadc3);
			if(HAL_ADC_PollForConversion(&hadc3, 1000)==HAL_OK)	m_adc.MCUVoltage = HAL_ADC_GetValue(&hadc3);	// ADC3_IN8(ADC_CHANNEL_8)
		}
		prevEncPulse = currEncPulse;
	#else
		if(mmf_PIDControl==PID_CONTROL_OPENING)
		{
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN4(ADC_CHANNEL_4)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorCurrent = HAL_ADC_GetValue(&hadc1);	// ADC12_IN5(ADC_CHANNEL_5)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.PowerVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN9(ADC_CHANNEL_9)
			
			HAL_ADC_Start(&hadc3);
			if(HAL_ADC_PollForConversion(&hadc3, 1000)==HAL_OK)	m_adc.MCUVoltage = HAL_ADC_GetValue(&hadc3);	// ADC3_IN8(ADC_CHANNEL_8)
		}
		else if(mmf_PIDControl==PID_CONTROL_CLOSING)
		{
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.PowerVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN4(ADC_CHANNEL_4)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorCurrent = HAL_ADC_GetValue(&hadc1);	// ADC12_IN5(ADC_CHANNEL_5)
			HAL_ADC_Start(&hadc1);
			if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN9(ADC_CHANNEL_9)
			
			HAL_ADC_Start(&hadc3);
			if(HAL_ADC_PollForConversion(&hadc3, 1000)==HAL_OK)	m_adc.MCUVoltage = HAL_ADC_GetValue(&hadc3);	// ADC3_IN8(ADC_CHANNEL_8)
		}
	#endif
}
#endif

void mmf_TaskMotorFeedback(void const * argument)
{
	uint32_t PreviousWakeTime = osKernelSysTick();

    /*
     * Local variable definition
     */
    #ifdef LPF
    	volatile static float CurrMotorCurrent=0, CurrPowerVoltage=0, CurrMotorVoltage=0;	// 현재 추정값
    	volatile static float PrevMotorCurrent=0, PrevPowerVoltage=0, PrevMotorVoltage=0;	// 이전 추정값
    	volatile static float alpha=0.9;													// 필터계수
    #else
    	int32_t SumSample=0;
    	int8_t i;
    	double tmp=0;
    #endif
    /*
     * B_emf ADC를 위한 타이머 설정
     */
#if 0 //jeon_190715
    vSemaphoreCreateBinary(g_semaphoreBEMF);
#else
    g_semaphoreBEMF = xSemaphoreCreateBinary();
#endif
    if(g_semaphoreBEMF != NULL)
    {
    	debug("## The semaphore was created successfully\r\n");
    	xSemaphoreTake(g_semaphoreBEMF,20);
    }
    else
    	debug("## The semaphore creation was failed\r\n");
    
    debug("1. Start MotorFeedback Task\r\n");
    
    for(;;)
    {
  		mmf_ConversionADC();					//여기서 선언해주어야 엔코더 고장났을때에도 전류값 측정해서 열림완료 ,닫힘완료 판단을 할수있

#if 0 //jeon_190822_Temp display  		
  		printf("\r\nm_adc.MotorVoltage(ADC3_IN8) : %d \r\n", m_adc.MotorVoltage);
  		printf("m_adc.PowerVoltage(ADC1_IN5) : %d \r\n", m_adc.PowerVoltage);
  		printf("m_adc.MotorCurrent(ADC1_IN9) : %d \r\n", m_adc.MotorCurrent);
#endif  		
  		
    	/*
    	 * Encoder Mode에서 ADC
    	 */
    	if(!error_list_flag[DCU_ENCODER_ERROR])
    	{
    		osDelayUntil (&PreviousWakeTime, 10UL);

    		
    		/*
    		 * Moving Average Filter
    		 */
//    		for(i=0; i<5; i++)
//    			{SumSample += m_adcsamples.PowerVoltage[i];}
//    		m_adc.PowerVoltage = SumSample/5;
//    		SumSample=0;
//
//    		m_adc.MotorVoltage = SumSample/5;
//    		SumSample=0;
//
//    		m_adc.MotorCurrent = SumSample/5;
//    		SumSample=0;


    	}
    	/*
    	 * B_emf Mode에서 ADC
    	 */
    	else
    	{
    		/*
    		 * todo: binary세마포어 두번 get해서 중지하는건 아닌지 확인 할 것
    		 *       Block Time을 무한대로 하는 방법 고려해 볼 것
    		 */
//			if(xSemaphoreTake(g_semaphoreBEMF, portMAX_DELAY) == pdTRUE)					// TC0_CH0이 세마포어 Give하면 시작
        	if(xSemaphoreTake(g_semaphoreBEMF, 1) == pdTRUE)					// TC0_CH0이 세마포어 Give하면 시작
        	{
        		bemf_task_flag = true;
        		#ifdef LPF
        		{
        			/*
        			 * Recursive LowPass Filter Equation
        			 * 현재추정값 = alpha*이전추정값 + (1-alpha)*현재입력값
        			 */
        			CurrMotorCurrent = alpha*PrevMotorCurrent + (1-alpha)*(float)m_adcsamples.MotorCurrent[0];
        			CurrPowerVoltage = alpha*PrevPowerVoltage + (1-alpha)*(float)m_adcsamples.PowerVoltage[0];
        			CurrMotorVoltage = alpha*PrevMotorVoltage + (1-alpha)*(float)m_adcsamples.MotorVoltage[0];
        			
        			m_adc.MotorCurrent = CurrMotorCurrent;
        			PrevMotorCurrent = CurrMotorCurrent;
        			m_adc.PowerVoltage = CurrPowerVoltage;
        			PrevPowerVoltage = CurrPowerVoltage;
        			m_adc.MotorVoltage = CurrMotorVoltage;
        			PrevMotorVoltage = CurrMotorVoltage;
        		}
        		#else
        		{
        			/*
        			 * Moving Average Filter
        			 */
        			for(i=0; i<5; i++)
        				{SumSample += m_adcsamples.PowerVoltage[i];}
        			m_adc.PowerVoltage = SumSample/5;
        			SumSample=0;
        			
        			for(i=0; i<5; i++)
        				{SumSample += m_adcsamples.MotorVoltage[i];}
        			m_adc.MotorVoltage = SumSample/5;
        			SumSample=0;
        			
        			for(i=0; i<5; i++)
        				{SumSample += m_adcsamples.MotorCurrent[i];}
        			m_adc.MotorCurrent = SumSample/5;
        			SumSample=0;
        		}
        		#endif
        		
        		/*
        		 * B_emf Voltage 계산
        		 */
        		if(mmf_PIDControl==PID_CONTROL_NONE)
        		{
        			m_adc.BemfVoltage = 0;
        		}
        		else
        		{
        			mmf_ConversionADC();
        			m_adc.BemfVoltage = m_adc.PowerVoltage - m_adc.MotorVoltage;
        			
         			/*
         			 * 속도/위치 계산
         			 */
            		if(mmf_PIDControl==PID_CONTROL_OPENING)
            		{
            			/*
            			 * 속도계산
						 * 속도가 100[pulse/100ms]일 경우 
						 * - 1000[pulse/s]의 속도가 되고, 1초당 1000pulse가 발생함
						 *   (최종 이동해야 할 거리는 2500 pulse)
            			 */
 //jeon_190715           			mmf_Bemf.Velocity = (float)m_adc.BemfVoltage*(float)0.73;
            			
                	    /*
                	     * 위치계산
                	     * 속도가 400[pulse/s]인 경우
                	     * 1초에 400[pulse]
                	     * 0.1s에 40[pulse]
                	     * 400pulse/s * 0.1s = 40[pulse]
                	     */
 //jeon_190715           			mmf_Bemf.Position += mmf_Bemf.Velocity*(float)0.1;
            			
//            			mmf_Bemf.Velocity = m_adc.BemfVoltage * 8;
//            			mmf_Bemf.Position += mmf_Bemf.Velocity/20;
            			
/*            			
						printf("B-Mode[O], BemfVoltage : %d \r\n", m_adc.BemfVoltage);
						printf("B-Mode[O], PowerVoltage : %d \r\n", m_adc.PowerVoltage);
						printf("B-Mode[O], MotorVoltage : %d \r\n", m_adc.MotorVoltage);
						printf("B-Mode[O], Velocity : %d \r\n", mmf_Bemf.Velocity);
						printf("B-Mode[O], Position : %d \r\n", mmf_Bemf.Position);
*/
            		}
            		else if(mmf_PIDControl==PID_CONTROL_CLOSING)
            		{
            			/*
            			 * 속도계산
            			 */
//jeon_190715            			mmf_Bemf.Velocity = (float)m_adc.BemfVoltage*(float)-0.73;
            			
            			/*
            			 * 위치계산
            			 */
//jeon_190715            			mmf_Bemf.Position += mmf_Bemf.Velocity*(float)0.1;			// 속도 부호가 (-)이므로 그냥 더하면 위치값이 빼짐
            			
//            			mmf_Bemf.Velocity = m_adc.BemfVoltage * 8;
//            			mmf_Bemf.Position += mmf_Bemf.Velocity/20;
            			
/*            									
						printf("B-Mode[C], BemfVoltage : %d \r\n", m_adc.BemfVoltage);
						printf("B-Mode[C], PowerVoltage : %d \r\n", m_adc.PowerVoltage);
						printf("B-Mode[C], MotorVoltage : %d \r\n", m_adc.MotorVoltage);
						printf("B-Mode[C], Velocity : %d \r\n", mmf_Bemf.Velocity);
						printf("B-Mode[C], Position : %d \r\n", mmf_Bemf.Position);
*/
            		}
            		else
            		{
            			// No Action
            		}
        		}
        		
        		/*
        		 * Bemf전압 측정을 위해 Stop한 Timer 다시 시작
        		 */
        		
        		/*
        		 * 100ms 지연 후 다시 Bemf 전압 ADC 시작
        		 */
        		
        		bemf_task_flag = false;
                osDelayUntil (&PreviousWakeTime, 10UL);
        	}
        	else
        	{
        		osDelayUntil (&PreviousWakeTime, 10UL);
        	}
    	}
    }
}

void mmf_isrBemf_PositionCalculate(void)
{
    volatile static int8_t tick1msCnt=0;
    /*
     * todo:Bemf 수행 구간에 인터럽트 수행 금지해야 함.
     */
    // cpu_irq_disable();
    // __disable_irq();
    // cpu_irq_enable();
    // __enable_irq();
    
    /*
     * 정지상태에서는 B_emf 계산 및 제어가 수행되지 않음
     */
    //  if(mmf_PIDControl>0)
 //   if(((mmf_PIDControl==PID_CONTROL_OPENING) || (mmf_PIDControl==PID_CONTROL_CLOSING)) && (bemf_task_flag == false))
    if((mmf_PIDControl==PID_CONTROL_OPENING) || (mmf_PIDControl==PID_CONTROL_CLOSING))
    {
    	tick1msCnt++;

    	if(tick1msCnt>10)																// tick1msCnt == 101
    	{
    		tick1msCnt = 0;
    		mmf_isrBemf_VelocityControl();
    		xSemaphoreGiveFromISR(g_semaphoreBEMF,pdFALSE);								// BemfTask에서 V_bemf를 통한 속도/위치 계산 <- 0.001s ~ 0.079s 동안 충분히 수행완료 됨
    	}
    	else if(tick1msCnt==96)															// tick1msCnt == 80
    	{
#if 0 //jeon_190715, 쿵쿵소리...
    		mmo_DoorFree();
#endif    		
    	}
    	else																			// 1 < tick1msCnt < 79
    	{
 //   		printf("BEMF Mode 0 \r\n");
    		// No Action
    	}
    }
    else /* mmf_PIDControl == PID_CONTROL_NONE */
    {
 //   	printf("BEMF Mode exception \r\n");
    	tick1msCnt = 0;
    }
}

/* 정지상태(mmf_PIDControl == None)에서는 호출되지 않음 */
#if 0
void mmf_isrBemf_VelocityControl(void)
{
    volatile static float V_Pgain=0.1, V_Igain=0, V_Dgain=0;
    volatile static float SumVerror=0, PrevVerror=0, PrevPID_Velocity=0;
    volatile static uint32_t Cnt_10ms=0;
    volatile static int32_t PrevMotorVelocity=0;
    float Verror=0;
    
    /*
     * 100ms 간격으로 V_ref 생성
     */
    #if 1
    {
    	if(mmf_PIDControl==PID_CONTROL_OPENING)
    	{
    		if(mmf_Bemf.Position<(float)100)		{mmf_Bemf.RefVelocity = (float)400;}
    		else if(mmf_Bemf.Position<(float)200)	{mmf_Bemf.RefVelocity = (float)800;}
    		else if(mmf_Bemf.Position<(float)2000)	{mmf_Bemf.RefVelocity = (float)1200;}
    		else /* mmf_Bemf.Position>=2000 */		{mmf_Bemf.RefVelocity = (float)0;}
    	}
    	else if(mmf_PIDControl==PID_CONTROL_CLOSING)
    	{
    		if(mmf_Bemf.Position>(float)2400)		{mmf_Bemf.RefVelocity = (float)-400;}
    		else if(mmf_Bemf.Position>(float)2300)	{mmf_Bemf.RefVelocity = (float)-800;}
    		else if(mmf_Bemf.Position>(float)500)	{mmf_Bemf.RefVelocity = (float)-1200;}
    		else /* mmf_Bemf.Position<=500 */		{mmf_Bemf.RefVelocity = (float)0;}
    	}
    	else
    	{
    		Verror=0;
    		PrevVerror=0;
    		SumVerror=0;
    		PID_Velocity = (float)0;
    		PrevPID_Velocity = (float)0;
    		m_CurrentPWMDuty=0;
    	}
    }
    #endif
    
    /*
     * 이동(Opening or Closing) 시 10ms 간격으로 PID 속도제어 수행
     */
    #if 1
    {
    //	if(mmf_PIDControl>0)
    	if((mmf_PIDControl==PID_CONTROL_OPENING) || (mmf_PIDControl==PID_CONTROL_CLOSING))
    	{
    		/*
    		 * 속도계산
    		 * 속도가 100[pulse/100ms]일 경우 
    		 * - 1000[pulse/s]의 속도가 되고, 1초당 1000pulse가 발생함
    		 *   (최종 이동해야 할 거리는 2500 pulse)
    		 */
    		/*
    		 ** BemfTask에서 B_emf전압을 속도로 환산하여 mmf_MotorVelocity를 업데이트 수행
    		 */
    		
    		/*
    		 * #EDCU-SW-NFR-P-01
    		 */
    		/*
    		 * #EDCU-SW-NFR-P-02
    		 */
    		/*
    		 * Velocity PID제어
    		 *  -> 첨부터 MotorRefVelocity값 바꾸지 말고, Vref가 0인 상태에서 Run 1~2회 수행 후 값 바꿀 것
    		 *     (첩부터 MotorRefVelocity값 바꾸면 PWM은 출력되는데 모터가 돌지를 않네, 왜그런지 나중에 확인해 봐야 함)
    		 */
    		Verror = mmf_Bemf.RefVelocity - mmf_Bemf.Velocity;
    		SumVerror += Verror;
    		PID_Velocity +=	(
    							(Verror*V_Pgain) + 													// P게인
    							(SumVerror*V_Igain) +												// I게인
    							((Verror-PrevVerror)*V_Dgain)										// D게인
    						);
    		PrevVerror = Verror;
    		
    		/*
    		 * 속도가 저속에서는 정밀하지 않아서 속도 기준으로 방향을 결정하지 않음
    		 * -> 처음 시작 순간 FET Switching을 수행하지 못해서 오동작
    		 * -> DoorOpening(0), DoorClosing(0) 호출 못해서 같은 방향으로 회건하거나, 아예 움직이지 않거나
    		 */
    		/*
    		 * Opening PWM 출력
    		 */
    		if(mmf_PIDControl==PID_CONTROL_OPENING)
    		{
    			/*
    			 * 처음 Opening으로 H-Bridge 스위칭
    			 * (PWM출력 Update마다 DoorOpening()/DoorClosing() 호출하면 스위칭 노이즈 발생)
    			 */
    			if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
     			{
    				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
    				/*
    				 * Max2000[pulse/s] 속도로 Closing 하다가 지연없이 바로 MAx 2000[pulse/s] 속도로 
    				 * Opening하면 역기전력이 흘러 Reset 되는 거 같음.
    				 * 안전하게 Free로 놓은 다음 20ms 딜레이 주고 H-Bridge Switching
    				 */
    //    			mmo_DoorBrake();
    				mmo_DoorFree();
    				mmo_DoorOpening(0);
    			}
    			
      			/*
    			 * todo emu: 이거 수행하면 2~3회는 Closing 잘 되다가
    			 *       도어는 움직이지 않고 속도가 계산되어, 이후 주구장창 계속 Closing PWM출력
    			 *       버그 잡을 것
    			 */
    //    		if(mmf_Bemf.Position>m_OpeningEncoderPulse)									// 다 열리면
    //    		{
    //    			mmo_DoorFree();
    //     			mmo_DoorBrake();														// 강제로 정지(Opened), cdshim_20180904
    //    		}
    //    		else
    			{
        			if(PID_Velocity>(float)800)	{PID_Velocity = (float)800;}
        			m_CurrentPWMDuty = (int32_t)PID_Velocity;
					mmo_DoorOpeningNchFET(m_CurrentPWMDuty);
    			}
    		}
    		/*
    		 * Closing PWM 출력
    		 */
    		else if(mmf_PIDControl==PID_CONTROL_CLOSING)
    		{
    			/*
    			 * 처음 Closing으로 H-Bridge 스위칭
    			 */
    			if(mmf_NextMotorDirSelect==PID_CONTROL_CLOSING)
    			{
    				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
    //    			mmo_DoorBrake();
    				mmo_DoorFree();
    				mmo_DoorClosing(0);
    			}
    			
      			/*
    			 * todo emu: 이거 수행하면 2~3회는 Closing 잘 되다가
    			 *       도어는 움직이지 않고 속도가 계산되어, 이후 주구장창 계속 Closing PWM출력
    			 *       버그 잡을 것
    			 */
    //     		if(mmf_Bemf.Position<30)												// 다 닫히면(0위치까지 가면 속도가 매우 느리고, 정지 시에는 B_emf가 발생하지 않으므로 위치가 변하지 않음
    //     		{
    //     			mmo_DoorFree();
    //     			mmo_DoorBrake();													//  강제로 정지(Closed), cdshim_20180904
    //     			mmf_Bemf.Position=0;
    //     		}
    //     		else
    			{
        			if(PID_Velocity<(float)-800)	{PID_Velocity = (float)-800;}
        			m_CurrentPWMDuty = (int32_t)PID_Velocity*-1;
					mmo_DoorClosingNchFET(m_CurrentPWMDuty);
    			}
    		}
    		else // PID_Velocity==0
    		{
    			// No Action
    		}
    		PrevPID_Velocity = PID_Velocity;
    	}
    	else
    	{
    		Verror=0;
    		PrevVerror=0;
    		SumVerror=0;
    		PID_Velocity = (float)0;
    		PrevPID_Velocity = (float)0;
    		m_CurrentPWMDuty=0;
    	}
    }
    #endif
}
#elif 0
void mmf_isrBemf_VelocityControl(void)
{
    volatile static float V_Pgain=0.1;
    volatile static int32_t PID_10msCnt=0, PID_100msCnt=0;
    volatile static uint8_t StartClosing = 0, StartOpening=0;
    volatile static int16_t currEncPulse=0, prevEncPulse=0;												// 1ms irs호출 시마다 현재 엔코더 펄스 업데이트
    float Verror=0;
    
	if(mmf_PIDControl==PID_CONTROL_OPENING)
	{
		/*
		 * 100ms 간격으로 V_ref 생성
		 */
		if(PID_100msCnt > 99)
		{
			PID_100msCnt=0;
			StartOpening=1;
			if(mmf_Bemf.Position<(float)100)		{mmf_Bemf.RefVelocity = (float)40;}
			else if(mmf_Bemf.Position<(float)200)	{mmf_Bemf.RefVelocity = (float)80;}
			else if(mmf_Bemf.Position<(float)2000)	{mmf_Bemf.RefVelocity = (float)120;}
			else /* mmf_Bemf.Position>=2000 */		{mmf_Bemf.RefVelocity = (float)0;}
			
			printf("%d, %d, %d, %d, %d\r\n",
					(int)mmf_Bemf.Position,
					mmf_Bemf.RefVelocity, mmf_Bemf.Velocity,
					m_adc.MotorCurrent,
					m_CurrentPWMDuty);
		}
		else
		{
			PID_100msCnt++;
		}
		
	    /*
	     * Opening 시 10ms 간격으로 PID 속도제어 수행
	     */
		if(PID_10msCnt > 10)
		{
			/*
			 * 속도계산
			 * 속도가 100[pulse/100ms]일 경우 
			 * - 1000[pulse/s]의 속도가 되고, 1초당 1000pulse가 발생함
			 *   (최종 이동해야 할 거리는 2500 pulse)
			 */
			/*
			 ** BemfTask에서 B_emf전압을 속도로 환산하여 mmf_MotorVelocity를 업데이트 수행
			 */
			Verror = mmf_Bemf.RefVelocity - mmf_Bemf.Velocity;
			PID_Velocity +=	(Verror*V_Pgain);
			
			/*
			 * 속도가 저속에서는 정밀하지 않아서 속도 기준으로 방향을 결정하지 않음
			 * -> 처음 시작 순간 FET Switching을 수행하지 못해서 오동작
			 * -> DoorOpening(0), DoorClosing(0) 호출 못해서 같은 방향으로 회건하거나, 아예 움직이지 않거나
			 */
			/*
			 * 처음 Opening으로 H-Bridge 스위칭
			 * (PWM출력 Update마다 DoorOpening()/DoorClosing() 호출하면 스위칭 노이즈 발생)
			 */
			if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
			{
				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
				/*
				 * Max2000[pulse/s] 속도로 Closing 하다가 지연없이 바로 MAx 2000[pulse/s] 속도로 
				 * Opening하면 역기전력이 흘러 Reset 되는 거 같음.
				 * 안전하게 Free로 놓은 다음 20ms 딜레이 주고 H-Bridge Switching
				 */
				mmo_DoorFree();
				mmo_DoorOpening(0);
			}
	    	else
			{
				if(StartOpening)
				{
					if(mmf_Bemf.Position>m_OpeningEncoderPulse)									// 다 열리면
					{
						mmo_DoorFree();
						StartOpening=0;
						prevEncPulse = currEncPulse;
					}
					else
					{
						if(PID_Velocity>(float)800)	{PID_Velocity = (float)800;}
						m_CurrentPWMDuty = (int32_t)PID_Velocity;
						mmo_DoorOpeningNchFET(m_CurrentPWMDuty);
					}
				}
			}
		}
		else
		{
			PID_10msCnt++;
		}
	}
	else if(mmf_PIDControl==PID_CONTROL_CLOSING)
	{
		/*
		 * 100ms 간격으로 V_ref 생성
		 */
		if(PID_100msCnt > 99)
		{
			PID_100msCnt=0;
			StartOpening=1;
			if(mmf_Bemf.Position>(float)2400)		{mmf_Bemf.RefVelocity = (float)-400;}
			else if(mmf_Bemf.Position>(float)2300)	{mmf_Bemf.RefVelocity = (float)-800;}
			else if(mmf_Bemf.Position>(float)500)	{mmf_Bemf.RefVelocity = (float)-1200;}
			else /* mmf_Bemf.Position<=500 */		{mmf_Bemf.RefVelocity = (float)0;}
			
			printf("%d, %d, %d, %d, %d\r\n",
					(int)mmf_Bemf.Position,
					mmf_Bemf.RefVelocity, mmf_Bemf.Velocity,
					m_adc.MotorCurrent,
					m_CurrentPWMDuty);
		}
		else
		{
			PID_100msCnt++;
		}
		
	    /*
	     * Closing 시 10ms 간격으로 PID 속도제어 수행
	     */
		if(PID_10msCnt > 10)
		{
			Verror = mmf_Bemf.RefVelocity - mmf_Bemf.Velocity;
			PID_Velocity +=	(Verror*V_Pgain);
			
			/*
			 * 처음 Closing으로 H-Bridge 스위칭
			 */
			if(mmf_NextMotorDirSelect==PID_CONTROL_CLOSING)
			{
				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
				mmo_DoorFree();
				mmo_DoorClosing(0);
			}
	    	else
			{
				if(StartClosing)
				{
					if(mmf_Bemf.Position<30)												// 다 닫히면(0위치까지 가면 속도가 매우 느리고, 정지 시에는 B_emf가 발생하지 않으므로 위치가 변하지 않음
					{
						mmo_DoorFree();
						StartClosing=0;
						prevEncPulse = currEncPulse;
						mmf_Bemf.Position=0;
					}
					else
					{
						if(PID_Velocity<(float)-800)	{PID_Velocity = (float)-800;}
						m_CurrentPWMDuty = (int32_t)PID_Velocity*-1;
						mmo_DoorClosingNchFET(m_CurrentPWMDuty);
					}
				}
			}
		}
		else
		{
			PID_10msCnt++;
		}
	}
	else
	{
		Verror=0;
		PID_Velocity = (float)0;
		m_CurrentPWMDuty=0;
	}
}
#else
void mmf_isrBemf_VelocityControl(void)
{
    volatile static float V_Pgain=0.1;
    float Verror=0;
//    printf("BEMF Mode - velocity \r\n");
	if(mmf_PIDControl==PID_CONTROL_OPENING)
	{
#if 1 //jeon_190710
		if((mip_Input.di0_DLS1 == true) || (mip_Input.di1_DLS2 == true))
		{
			m_CurrentPWMDuty += 70;
			if(m_CurrentPWMDuty < 200) m_CurrentPWMDuty = 200;
			else if(m_CurrentPWMDuty > (PWM_MIN+400)) m_CurrentPWMDuty = (PWM_MIN+400);
		}
		else if((osKernelSysTick() - mdm_time.OpeningStart) < 500)	
		{
			m_CurrentPWMDuty = 550;
		}
		else if((osKernelSysTick() - mdm_time.OpeningStart) < 1800)	
		{
			m_CurrentPWMDuty = 550;
		}
		else if((osKernelSysTick() - mdm_time.OpeningStart) < 4000)	
		{
			m_CurrentPWMDuty = 200;
		}
		else if((osKernelSysTick() - mdm_time.OpeningStart) < 6000)	
		{
			m_CurrentPWMDuty = 200;
		}
		else
		{
			m_CurrentPWMDuty = 0;
		}
		
		mmo_DoorOpening(m_CurrentPWMDuty);
			
#else
		if(((mip_Input.di0_DLS1 == true) || (mip_Input.di1_DLS2 == true)) && (mmf_Bemf.Position < OPEN_FAST_ACCEL_SECTiON))
		{
			m_CurrentPWMDuty += 10;
			if(m_CurrentPWMDuty < 200) m_CurrentPWMDuty = 200;
		}		
		else if((mmf_Bemf.Position < OPEN_FAST_ACCEL_SECTiON) && (mmf_Bemf.Position<9000))	
		{
			m_CurrentPWMDuty = 200;
		}
		if((mmf_Bemf.Position < OPEN_SLOW_CONST_SECTiON) && (mmf_Bemf.Position<9000))	
		{
			
			if(OpenPowerValue == 0) //2000
			{
				m_CurrentPWMDuty = 660; //2s(60)
			}
			else if(OpenPowerValue == 1) //2500
			{
				m_CurrentPWMDuty = 550; //jeon_190715 orig: 2.5s(550)
			}
			else if(OpenPowerValue == 2) //3000
			{
				m_CurrentPWMDuty = 460; //3s(40)
			}
			else if(OpenPowerValue == 3) //3500
			{
				m_CurrentPWMDuty = 370; //3.5s(25)
			}
			else if(OpenPowerValue == 4) //4000
			{
				m_CurrentPWMDuty = 300; //4s(18)
			}
			else //2500
			{
				m_CurrentPWMDuty = 550; //jeon_190715 orig: 2.5s(550)
			}
		}
		else if((mmf_Bemf.Position >= OPEN_SLOW_CONST_SECTiON) && (mmf_Bemf.Position<9000))
		{
			m_CurrentPWMDuty = 200;
		}
		else/* currEncPulse>9000 */							m_CurrentPWMDuty=0;
		
		Verror = mmf_Bemf.RefVelocity - mmf_Bemf.Velocity;
		PID_Velocity +=	(Verror*V_Pgain);
		
		if(PID_Velocity>(float)800)	{PID_Velocity = (float)800;}
		else if(PID_Velocity<(float)0)	{PID_Velocity = (float)0;}

//		printf("O-duty:%d\r\n", m_CurrentPWMDuty);
		mmo_DoorOpeningNchFET(m_CurrentPWMDuty);
//		mmo_DoorOpening(m_CurrentPWMDuty);
#endif
	}
	else if(mmf_PIDControl==PID_CONTROL_CLOSING)
	{
#if 1 //jeon_190710
		if(mip_Input.DoorClosedSwitchOn)
		{
			m_CurrentPWMDuty=0;
		}
		else if((osKernelSysTick() - mdm_time.ClosingStart) < 1800)
		{
			m_CurrentPWMDuty = 450;
		}
		else if((osKernelSysTick() - mdm_time.ClosingStart) < 4000)
		{
			m_CurrentPWMDuty = 200;
		}
		else if((osKernelSysTick() - mdm_time.ClosingStart) < 8000)
		{
			m_CurrentPWMDuty = 200;
		}
		else
		{
			m_CurrentPWMDuty = 0;
		}
		
		mmo_DoorClosing(m_CurrentPWMDuty);

#else
		if((mip_Input.DoorClosedSwitchOn) && (mmf_Bemf.Position < 10))
		{
			m_CurrentPWMDuty=0;
		}
		else if((mmf_Bemf.Position >= CLOSE_ACCEL_SECTiON) && (mmf_Bemf.Position<9000))	
		{
//jeon_190624							m_CurrentPWMDuty = 550;
			if(ClosePowerValue == 0) //2500
			{
				m_CurrentPWMDuty = 640; //20190618 44 => 40, 2.5s(44)
			}
			else if(ClosePowerValue == 1) //3000
			{
				m_CurrentPWMDuty = 550; //3s(36)
			}
			else if(ClosePowerValue == 2) //3500
			{
				m_CurrentPWMDuty = 460; //3.5s(18)
			}
			else if(ClosePowerValue == 3) //4000
			{
				m_CurrentPWMDuty = 370; //4s(12)
			}
			else if(ClosePowerValue == 4) //4500
			{
				m_CurrentPWMDuty = 300; //4.5s(9)
			}
			else if(ClosePowerValue == 5) //5000
			{
				m_CurrentPWMDuty = 250; //5s(9)
			}
			else //3000
			{
				m_CurrentPWMDuty = 550; //3s(36)
			}
		}
		if((mmf_Bemf.Position >= CLOSE_SLOW_CONST_SECTiON) && (mmf_Bemf.Position<9000))	
		{
//jeon_190624							m_CurrentPWMDuty = 550;
			if(ClosePowerValue == 0) //2500
			{
				m_CurrentPWMDuty = 640; //20190618 44 => 40, 2.5s(44)
			}
			else if(ClosePowerValue == 1) //3000
			{
				m_CurrentPWMDuty = 550; //3s(36)
			}
			else if(ClosePowerValue == 2) //3500
			{
				m_CurrentPWMDuty = 460; //3.5s(18)
			}
			else if(ClosePowerValue == 3) //4000
			{
				m_CurrentPWMDuty = 370; //4s(12)
			}
			else if(ClosePowerValue == 4) //4500
			{
				m_CurrentPWMDuty = 300; //4.5s(9)
			}
			else if(ClosePowerValue == 5) //5000
			{
				m_CurrentPWMDuty = 250; //5s(9)
			}
			else //3000
			{
				m_CurrentPWMDuty = 550; //3s(36)
			}
		}
		else if((mmf_Bemf.Position<CLOSE_SLOW_CONST_SECTiON) && (mmf_Bemf.Position<9000))
		{
			m_CurrentPWMDuty = 200;
		}
		else/* currEncPulse>9000 */							m_CurrentPWMDuty=0;
		
//		printf("C-duty:%d\r\n", m_CurrentPWMDuty);
		mmo_DoorClosingNchFET(m_CurrentPWMDuty);
//		mmo_DoorClosing(m_CurrentPWMDuty);

#endif
	}
	else
	{
		Verror=0;
		PID_Velocity = (float)0;
		m_CurrentPWMDuty=0;
	}
}
#endif

void mmf_isrEncoder_PositionCalculate(void)
{
    /*
     * Called by B상(TC0_TIOB) Capture Input Interrupt
     */
	#ifdef HW_DEPENDENT_CODE
    #endif
}

/* TC0_CH0의 Interrupt Service Routine에서 10ms마다 호출되어 100ms마다 생성되는 Vref를 10ms마다 PID제어 수행 */
#if 0
void mmf_isrEncoder_VelocityControl(void)
{
    volatile static float V_Pgain=1, V_Igain=0, V_Dgain=0;
    volatile static float SumVerror=0, PrevVerror=0, PrevPID_Velocity=0;
    volatile static float Verror=0;
    volatile static int32_t Vref_1msCnt=0, PID_1msCnt=0;
    volatile static int32_t currEncPulse=0, prevEncPulse=0;		// 1ms irs호출 시마다 현재 엔코더 펄스 업데이트
    
    #if 0
    /*
     * 주파수로부터 속도구하는 방법 ??? (이해 안됨)
     * https://www.portescap.com/node/2032
     * 모터 속도는 다음 공식을 사용하여 카운트 주파수로부터 파생될 수 있습니다. N = f x 60 / CPR 여기서:
     * N = 모터 속도(분당 회전)
     * f = 인코더 카운트 주파수(Hz)
     * CPR = 회전당 카운트 수(회전당 라인 수/펄스의 4배)
     * 
     * capture input을 통한 펄스 주파수를 이용해서 속도를 측정하는 방법
     * “DSP 특별부록-2. MOTOR회전 속도 측정 방법 구현.pdf 파일” 확인
     */
    //	uint16_t frequence, dutycycle;
    //	frequence = (sysclk_get_peripheral_bus_hz(TC0) /
    //				 divisors[TC_CAPTURE_TIMER_SELECTION]) /
    //				 mmf_Encoder.CaptureRb;
    //	dutycycle = (mmf_Encoder.CaptureRb - mmf_Encoder.CaptureRa) * 100 /
    //				mmf_Encoder.CaptureRb;
    //	debug("2.MotorFeedback frequency: %d Hz, Duty: %d%% \r\n",
    //			frequence, dutycycle);
    #endif
    
    currEncPulse = mmf_Encoder.Position;	// 1ms마다 현재 엔코더펄스 업데이트
    
    /*
     * 100ms 간격으로 V_ref 생성
     */
    #if 1
    {
    	if(mmf_PIDControl==PID_CONTROL_OPENING)
    	{
    		/*
    		 * Opening 시 위치에 따라 사다리꼴 속도제어 (시간 범위 안에 들어오도록 튜닝 필요)
    		 */
    		Vref_1msCnt++;
    		if(Vref_1msCnt>100)
    		{
    			Vref_1msCnt=0;
    			
    			#if 1
    				mmf_Encoder.RefVelocity=50;
    			#elif 0
    				if(currEncPulse<500)				{mmf_Encoder.RefVelocity+=4;}	// 0~500
    				else if(currEncPulse<1000)			{mmf_Encoder.RefVelocity+=8;}	// 500~1000
    				else if(currEncPulse<1500)			{mmf_Encoder.RefVelocity+=0;}	// 1000~1500
    				else if(currEncPulse<2000)			{mmf_Encoder.RefVelocity-=8;}	// 1500~2000
    				else if(currEncPulse<2500)			{mmf_Encoder.RefVelocity-=4;}	// 2000~2500
    				else								mmf_Encoder.RefVelocity=0;		// 2500 이상
    			#else // 이걸로 확인
    				if(currEncPulse<500)				{mmf_Encoder.RefVelocity+=2;}	// cdshim_20180904
    				else if(currEncPulse<1000)			{mmf_Encoder.RefVelocity+=0;}
    				else if(currEncPulse<2500)
    				{
    					if(mmf_Encoder.RefVelocity>2)	mmf_Encoder.RefVelocity-=4;
    					else							mmf_Encoder.RefVelocity =2;		// 2500위치에 도달할 때까지 PWM출력은 나가야 함
    				}
    				else/* currEncPulse>2500 */			mmf_Encoder.RefVelocity=0;
    			#endif
    		}
    	}
    	else if(mmf_PIDControl==PID_CONTROL_CLOSING)
    	{
    		/*
    		 * Closing 시 위치에 따라 사다리꼴 속도제어 (시간 범위 안에 들어오도록 튜닝 필요)
    		 */
    		Vref_1msCnt++;
    		if(Vref_1msCnt>100)
    		{
    			Vref_1msCnt=0;
    			#if 1
    				mmf_Encoder.RefVelocity=-50;
    			#elif 0
    				if(currEncPulse>2000)			{mmf_Encoder.RefVelocity-=4;}		// 2000~2500
    				else if(currEncPulse>1500)		{mmf_Encoder.RefVelocity-=8;}		// 1500~2000
    				else if(currEncPulse>1000)		{mmf_Encoder.RefVelocity-=0;}		// 1000~1500
    				else if(currEncPulse>500)		{mmf_Encoder.RefVelocity+=8;}		// 500~1000
    				else if(currEncPulse>0)			{mmf_Encoder.RefVelocity+=4;}		// 0~500
    				else							mmf_Encoder.RefVelocity=0;			// 0 이하
    			#else // 이걸로 확인
    				if(currEncPulse>2000)				{mmf_Encoder.RefVelocity-=1;}	
    				else if(currEncPulse>1500)			{mmf_Encoder.RefVelocity-=0;}
    				else if(currEncPulse>0)
    				{
    					if(mmf_Encoder.RefVelocity<-2)	mmf_Encoder.RefVelocity+=1;
    					else							mmf_Encoder.RefVelocity =-1;	// 0위치에 도달할 때까지 PWM출력은 나가야 함
    				}
    				else/* currEncPulse<0 */			mmf_Encoder.RefVelocity=0;
    			#endif
    		}
    	}
    	else
    	{
    		/*
    		 * Opening 시 30s가 안지났는데 900mm만큼 다 이동해서 Opened 상태가 되면
    		 * Vref_1msCnt 값은 30이상까지 카운트를 못해서 0으로 Reset되지 않고,
    		 * 3s후 Closing 상태로 변경되면 이전 카운트 값부터 다시 시작하게 되므로
    		 * 
    		 * Opening/Closing PID Control을 수행하지 않으면 Decision모듈에서 Brake한 경우이므로
    		 * Vref_1msCnt 값을 0으로 Reset 해준다.
    		 */
    		Vref_1msCnt=0;
    		Verror=0;
    		PrevVerror = 0;
    		SumVerror = 0;
    		PID_Velocity = (float)0;
    		PrevPID_Velocity = (float)0;
    		m_CurrentPWMDuty = 0;
    		mmf_Encoder.RefVelocity=0;
    		mmf_Encoder.Velocity=0;
    	}
    }
    #endif
    
    /*
     * 이동(Opening or Closing) 시 10ms 간격으로 PID 속도제어 수행
     */
    #if 1
    {
//    	if(mmf_PIDControl>0)
    	if((mmf_PIDControl==PID_CONTROL_OPENING) || (mmf_PIDControl==PID_CONTROL_CLOSING))
    	{
    		PID_1msCnt++;
			if(PID_1msCnt>10)
        	{
    			PID_1msCnt=0;
        		/*
        		 * 속도계산
        		 * 속도가 100[pulse/100ms]일 경우 
        		 * - 1000[pulse/s]의 속도가 되고, 1초당 1000pulse가 발생함
        		 *   (최종 이동해야 할 거리는 2500 pulse)
        		 */
//            	mmf_Encoder.Velocity = (currEncPulse - prevEncPulse)*100;					// 속도 = 엔코더변화량/10ms = 엔코더변화량/0.01s = 100 x 엔코더변화량/s
        		mmf_Encoder.Velocity = (currEncPulse - prevEncPulse);
    			prevEncPulse = currEncPulse;													// 10ms마다 이전 엔코더 펄스 업데이트
    			
        		/*
        		 * Velocity PID제어
        		 *  -> 첨부터 MotorRefVelocity값 바꾸지 말고, Vref가 0인 상태에서 Run 1~2회 수행 후 값 바꿀 것
        		 *     (첨부터 MotorRefVelocity값 바꾸면 PWM은 출력되는데 모터가 돌지를 않네, 왜그런지 나중에 확인해 봐야 함)
        		 */
        		Verror = (float)mmf_Encoder.RefVelocity - (float)mmf_Encoder.Velocity;
        		SumVerror += Verror;
            	PID_Velocity +=	(
            						(Verror*V_Pgain) + 											// P게인
            						(SumVerror*V_Igain) +										// I게인
            						((Verror-PrevVerror)*V_Dgain)								// D게인
            					);
        		PrevPID_Velocity = PID_Velocity;
        		PrevVerror = Verror;
        		
        		/*
        		 * Opening PWM 출력
        		 */
//            	if(PID_Velocity>(float)0)
    			if(mmf_PIDControl==PID_CONTROL_OPENING)
        		{
        			/*
        			 * 처음 Opening으로 H-Bridge 스위칭
        			 * (PWM출력 Update마다 DoorOpening()/DoorClosing() 호출하면 스위칭 노이즈 발생)
        			 */
//            		if(PrevPID_Velocity<=(float)0)
//    				if(PrevPID_Velocity==(float)0)
    				if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
        			{
        				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
        				/*
        				 * Max2000[pulse/s] 속도로 Closing 하다가 지연없이 바로 MAx 2000[pulse/s] 속도로 
        				 * Opening하면 역기전력이 흘러 Reset 되는 거 같음.
        				 * 안전하게 Free로 놓은 다음 20ms 딜레이 주고 H-Bridge Switching
        				 */
    					//mmo_DoorBrake();
        				mmo_DoorFree();
        				mmo_DoorOpening(0);
        			}
        			
        			if(currEncPulse>m_OpeningEncoderPulse)										// 다 열리면
        			{
        				mmo_DoorFree();
         				mmo_DoorBrake();														// 강제로 정지(Opened), cdshim_20180904
        			}
        			else
        			{
            			if(PID_Velocity>(float)900)	{PID_Velocity = (float)900;}
            			m_CurrentPWMDuty = (int32_t)PID_Velocity;
            			mmo_DoorOpeningNchFET(m_CurrentPWMDuty);
        			}
        		}
        		/*
        		 * Closing PWM 출력
        		 */
//            	else if(PID_Velocity<(float)0)
    			else if(mmf_PIDControl==PID_CONTROL_CLOSING)
        		{
        			/*
        			 * 처음 Closing으로 H-Bridge 스위칭
        			 */
//            		if(PrevPID_Velocity>=(float)0)
//    				if(PrevPID_Velocity==(float)0)
    				if(mmf_NextMotorDirSelect==PID_CONTROL_CLOSING)
        			{
        				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
    					//mmo_DoorBrake();
        				mmo_DoorFree();
        				mmo_DoorClosing(0);
        			}
        			
        			if(currEncPulse<0)															// 다 닫히면
        			{
        				mmo_DoorFree();
         				mmo_DoorBrake();														//  강제로 정지(Closed), cdshim_20180904
        			}
        			else
        			{
            			if(PID_Velocity<(float)-900)	{PID_Velocity = (float)-900;}
            			m_CurrentPWMDuty = (int32_t)PID_Velocity*-1;
    					/*
    					 * mmo_DoorClosing()가 호출되면 g_CurrentPWM_Ch==PWM_CH3_CLOSING_REVERSE이어야 하나
    					 * PWM_CH_NO_SELECTE인 상태에서 이 로직을 타는 경우가 있음
    					 * -> 2500위치에서 Vref를 -2씩 계속 빼서 PWM출력을 보내야 하나, 채널이 선택 안되어 PWM출력은 안나가고,
    					 *    Vref만 -2씩 계속 빼져서 -2000 이상 계속 빼짐
    					 *    어차피 Opening은 1이고, Closing은 3이니까, 그냥 상수값으로 채널 고정시킴
    					 * 
    					 * -> 이렇게 했는데도 같은현상 반복 됨
    					 *    속도로 보고 Closing함수를 호출 할 경우 호출되지 않는 경우가 발생하는 거 같음
    					 */
            			mmo_DoorClosingNchFET(m_CurrentPWMDuty);
        			}
        		}
        		else // PID_Velocity==0
        		{
        			// No Action
        		}
        	}
    	}
    	else
    	{
    		Verror=0;
    		PrevVerror = 0;
    		SumVerror = 0;
    		PID_Velocity = (float)0;
    		PrevPID_Velocity = (float)0;
    		m_CurrentPWMDuty = 0;
    		mmf_Encoder.RefVelocity=0;
    		mmf_Encoder.Velocity=0;
    	}
    }
    #endif
}
#else
void mmf_isrEncoder_VelocityControl(void)
{
    volatile static float Vo_Pgain=2, Vc_Pgain=3.5;
    volatile static float Verror=0;
    volatile static int32_t PID_10msCnt=0;
    volatile static int16_t currEncPulse=0, prevEncPulse=0;												// 1ms irs호출 시마다 현재 엔코더 펄스 업데이트
    volatile static uint8_t StartClosing = 0, StartOpening=0;
    volatile static int32_t OpeningAccBeforDLS=0, OpeningAccAfterDLS=0;

    volatile static int32_t Open_Curr_Vel_time=0, Close_Curr_Vel_time=0;
    volatile static int32_t Open_Past_Vel_time=0, Close_Past_Vel_time=0;
	uint16_t powervoltage=0;

    volatile static int32_t closing_pwm_ind_Cnt=0;

    extern volatile uint32_t F10_OpenCount;

     #if 1
    {
		if(mmf_PIDControl==PID_CONTROL_OPENING)
		{
		    /*
		     * 100ms마다 V_ref 생성 -> 10ms마다 제어 수행
		     */
			if(PID_10msCnt > 9)
			{
				PID_10msCnt=0;
				StartOpening=1;
				if(currEncPulse<OpeningSection.Acceleration)
				{
					#if 0
						if(mmf_Encoder.RefVelocity<5)	mmf_Encoder.RefVelocity+=40;
						else							mmf_Encoder.RefVelocity+=motorOpenPower;
					#else
						if(currEncPulse<OpeningSection.Acceleration)
						{
							OpeningAccBeforDLS += 35; //30
							OpeningAccAfterDLS += motorOpenPower+3; //0
							
							/*
							 * Opening 시에만 닫힌상태에서 DLS가 풀릴때까지 Kick Start
							 */
							if(currEncPulse<300)
							{
								mmf_Encoder.RefVelocity = OpeningAccBeforDLS;
							}
							/*
							 * DLS가 풀린 이후부터는 가속구간까지 정상적으로 가속시작
							 */
							else
							{
//jeon_190531								mmf_Encoder.RefVelocity = OpeningAccAfterDLS;
								if(OpenPowerValue == 0) //2000
								{
									mmf_Encoder.RefVelocity = 65; //2s(60)
								}
								else if(OpenPowerValue == 1) //2500
								{
									mmf_Encoder.RefVelocity = 54; //2.5s(54)
								}
								else if(OpenPowerValue == 2) //3000
								{
									mmf_Encoder.RefVelocity = 40; //3s(60)
								}
								else if(OpenPowerValue == 3) //3500
								{
									mmf_Encoder.RefVelocity = 25; //3.5s(25)
								}
								else if(OpenPowerValue == 4) //4000
								{
									mmf_Encoder.RefVelocity = 18; //4s(18)
								}
								else //2500
								{
									mmf_Encoder.RefVelocity = 54; //2.5s(54)
								}

							}
						}
					#endif
				}
				else if(currEncPulse<OpeningSection.Constant)		
				{
//jeon_190530					mmf_Encoder.RefVelocity+=0;
					
					if(OpenPowerValue == 0) //2000
					{
						mmf_Encoder.RefVelocity = 65; //2s(60)
					}
					else if(OpenPowerValue == 1) //2500
					{
						mmf_Encoder.RefVelocity = 54; //2.5s(54)
					}
					else if(OpenPowerValue == 2) //3000
					{
						mmf_Encoder.RefVelocity = 40; //3s(40)
					}
					else if(OpenPowerValue == 3) //3500
					{
						mmf_Encoder.RefVelocity = 25; //3.5s(25)
					}
					else if(OpenPowerValue == 4) //4000
					{
						mmf_Encoder.RefVelocity = 18; //4s(18)
					}
					else //2500
					{
						mmf_Encoder.RefVelocity = 54; //2.5s(54)
					}
				}
				else if(currEncPulse<OpeningSection.Deceleration)
				{
					#if 0
						if(mmf_Encoder.RefVelocity>10)					mmf_Encoder.RefVelocity -=13;
						else											mmf_Encoder.RefVelocity =10;			// 2500위치에 도달할 때까지 PWM출력은 나가야 함
					#else
						if(mmf_Encoder.RefVelocity>7)					mmf_Encoder.RefVelocity -=6;  //jeon_190530 : 10, 10
						else											mmf_Encoder.RefVelocity =7;			// 2500위치에 도달할 때까지 PWM출력은 나가야 함
					#endif
				}
				else/* currEncPulse>3000 */							mmf_Encoder.RefVelocity=0;
//				printf("%d, %d, %d, %d\r\n",
//						(int)mmf_Encoder.Position,
//						mmf_Encoder.RefVelocity, mmf_Encoder.Velocity,
//						m_CurrentPWMDuty);
//				printf("%d, %d, %d, %d, %d\r\n",
//						(int)mmf_Encoder.Position,
//						mmf_Encoder.RefVelocity, mmf_Encoder.Velocity,
//						m_adc.MotorCurrent,
//						m_CurrentPWMDuty);
			}
			else
			{
				PID_10msCnt++;
			}
			
			/*
			 * 10ms마다 제어 : 처음 Opening 시 방향 설정
			 */
			if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
			{
				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
				mmo_DoorFree();
				mmo_DoorOpening(0);
			}
			/*
			 * 10ms마다 제어 : Opening PWM Update
			 */
			else
			{
				if(StartOpening)
				{
					/*
					 * 타아밍을 고려한 열림완료/닫힘완료 판단
					 * 고려사항1
					 * Closing에서 Closed가 되어 엔코더를 리셋해도
					 * 모터가 조금만(2펄스) 더 돌면 엔코더 값이 9998 값이 나옴
					 * 따라서 Closed 상태에서 엔코더 값이 9998이 되고 이때 열림동작을 수행하면
					 * 바로 열림완료로 판단하므로 엔코더 값이 9000보다 작은 조건을 추가함
					 *
					 * 고려사항2
					 * 여기서 플래그를 true로 하고, 판단모듈에서 OpenedByPosition를 false로 해도
					 * PID주기가 1ms로 실행되기 때문에 이 부분이 다시 실행되어 다시 OpenedByPosition를 true로 셋함
					 * 이후에 판단모듈에서 연림완료가 되고, 닫힘동작을 수행해도 OpenedByPosition는 여전히 true로 있고
					 * 닫힘 완료 후 다시 열림동작을 수행하면 OpenedByPosition가 true기 때문에 바로 열림완료가 수행됨
					 * 따라서 열림완료 이후에 OpenedByPosition를 false로 클리어 해야 함. -> 닫힘완료도 마찬가지
					 */
					if((mmf_Encoder.Position>m_OpeningEncoderPulse)&&(mmf_Encoder.Position<9000))						// 다 열리면
					{
						mmo_DoorFree();
						mmf_EndDecision.OpenedByPosition = true;
						//debug("# Opening State: 엔코더 열림완료 판단\r\n");
						StartOpening=0;
						prevEncPulse = currEncPulse;
					}
					else
					{
						mmf_Encoder.Position = mmo_getEncoderCountValue();
					    currEncPulse = mmf_Encoder.Position;
					    
					    Open_Curr_Vel_time = osKernelSysTick();
					    
//jeon_190530						mmf_Encoder.Velocity = (currEncPulse - prevEncPulse);
						
					    if(Open_Curr_Vel_time > Open_Past_Vel_time)
					    {
					    	mmf_Encoder.Velocity = (((currEncPulse - prevEncPulse)*10)/(Open_Curr_Vel_time - Open_Past_Vel_time));
					    }
					    else
					    {
					    	mmf_Encoder.Velocity = (currEncPulse - prevEncPulse);
					    }
						Current_Velocity_value = (int16_t)(((currEncPulse - prevEncPulse)*1000)/(5*(Open_Curr_Vel_time - Open_Past_Vel_time)));

						Open_Past_Vel_time = Open_Curr_Vel_time;
					    
						prevEncPulse = currEncPulse;													// 10ms마다 이전 엔코더 펄스 업데이트
						Verror = (float)mmf_Encoder.RefVelocity - (float)mmf_Encoder.Velocity;
//jeon_190621						m_CurrentPWMDuty += (Verror*Vo_Pgain);
						
// jeon_190514 : voltage lever check - pwm limit	
						
#if 1 //jeon_190621					
/*
#define OPEN_ACCEL_SECTiON			500				// 2s(500) - 650mm opening
#define OPEN_CONST_SECTiON			1250			// 2s(1250) - 650mm opening
#define OPEN_SLOW_CONST_SECTiON		2500
##define OPEN_DEACE_SECTiON			3580			// 2s(3580) - 650mm opening
*/
						if(((mip_Input.di0_DLS1 == true) || (mip_Input.di1_DLS2 == true)) && (currEncPulse < OPEN_FAST_ACCEL_SECTiON))
						{
#if 0 //jeon_190829 original
							m_CurrentPWMDuty += 30;
							if(m_CurrentPWMDuty < PWM_MIN) m_CurrentPWMDuty = PWM_MIN;
							else if(m_CurrentPWMDuty > (PWM_MIN+400)) m_CurrentPWMDuty = (PWM_MIN+400); //jeon_190822 (PWM_MIN+200) => (PWM_MAX) or (PWM_MIN+300) 
#else						// wjjang_200214 F10 Open 3회 및 PWM Duty 조정

							if(F10_OpenCount < 1)
							{
								if(closing_pwm_ind_Cnt == 0) m_CurrentPWMDuty = 200;
								else 						 m_CurrentPWMDuty += 30; //jeon_191204 orig:70
								if(m_CurrentPWMDuty < (PWM_MIN)) m_CurrentPWMDuty = (PWM_MIN);
								else if(m_CurrentPWMDuty > (PWM_MIN+400)) m_CurrentPWMDuty = (PWM_MIN+400); //jeon_191204 orig:(PWM_MIN+400)
							}
							
							else if(F10_OpenCount == 1)
							{
								if(closing_pwm_ind_Cnt == 0) m_CurrentPWMDuty = 300;
								else 						 m_CurrentPWMDuty += 50; //jeon_191204 orig:70
								if(m_CurrentPWMDuty < (PWM_MIN+100)) m_CurrentPWMDuty = (PWM_MIN+100);
								else if(m_CurrentPWMDuty > (PWM_MIN+500)) m_CurrentPWMDuty = (PWM_MIN+500); //jeon_191204 orig:(PWM_MIN+400)
							}

							else if(F10_OpenCount > 1)
							{
								if(closing_pwm_ind_Cnt == 0) m_CurrentPWMDuty = 400;
								else 						 m_CurrentPWMDuty += 70; //jeon_191204 orig:70
								if(m_CurrentPWMDuty < (PWM_MIN+200)) m_CurrentPWMDuty = (PWM_MIN+200);
								else if(m_CurrentPWMDuty > (PWM_MIN+600)) m_CurrentPWMDuty = (PWM_MIN+600); //jeon_191204 orig:(PWM_MIN+400)
							}

							closing_pwm_ind_Cnt++;

/*							if(closing_pwm_ind_Cnt == 0) m_CurrentPWMDuty = 100;
							else 						 m_CurrentPWMDuty += 70;
							
							closing_pwm_ind_Cnt++;
							
							if(m_CurrentPWMDuty < 100) m_CurrentPWMDuty = 100;
							else if(m_CurrentPWMDuty > (PWM_MIN+400)) m_CurrentPWMDuty = (PWM_MIN+400); //jeon_190822 (PWM_MIN+200) => (PWM_MAX) or (PWM_MIN+300) 
*/
#endif
						}
						else if((currEncPulse < OPEN_FAST_ACCEL_SECTiON) && (currEncPulse<9000))	
						{
							m_CurrentPWMDuty = PWM_MIN;
						}
						else if ((currEncPulse < (OPEN_ACCEL_SECTiON-400)) && (currEncPulse<9000))	
						{
							closing_pwm_ind_Cnt = 0;
							
							switch(mdm_time.OpenConfigtime)
							{
								case 2000: 
									m_CurrentPWMDuty = 700; 
									break;
								case 2200: 
									m_CurrentPWMDuty = 660; 
									break;
								case 2400: 
									m_CurrentPWMDuty = 620; 
									break;
								case 2600: 
									m_CurrentPWMDuty = 580; 
									break;
								case 2800: 
									m_CurrentPWMDuty = 550; 
									break;
								case 3000: 
									m_CurrentPWMDuty = 520; 
									break;
								case 3200: 
									m_CurrentPWMDuty = 490; 
									break;
								case 3400: 
									m_CurrentPWMDuty = 450; 
									break;
								case 3600: 
									m_CurrentPWMDuty = 410; 
									break;
								case 3800: 
									m_CurrentPWMDuty = 370; 
									break;
								case 4000: 
									m_CurrentPWMDuty = 330; 
									break;
								default: 
									m_CurrentPWMDuty = 580; 
									break;
							}
							
							if((motor_voltage >= 1900) && (motor_voltage < 2300)) /* DC 70V */
							{
								m_CurrentPWMDuty += 100;
							}
						}
						else if((currEncPulse < OPEN_SLOW_CONST_SECTiON) && (currEncPulse<9000))	
						{
							closing_pwm_ind_Cnt = 0;
#if 1 //jeon_190812							
							if(mdc_DoorState == DOOR_CLOSING) powervoltage =(uint16_t)(m_adc.MotorVoltage / 31);
							else if(mdc_DoorState == DOOR_OPENING) powervoltage =(uint16_t)(m_adc.PowerVoltage / 31);
							else powervoltage = 100;

							switch(mdm_time.OpenConfigtime)
							{
								case 2000: 
									Ref_Velocity_value = 520;
									break;
								case 2200: 
									Ref_Velocity_value = 470;
									break;
								case 2400: 
									Ref_Velocity_value = 420;
									break;
								case 2600: 
									Ref_Velocity_value = 370;
									break;
								case 2800: 
									Ref_Velocity_value = 340;
									break;
								case 3000: 
									Ref_Velocity_value = 310;
									break;
								case 3200: 
									Ref_Velocity_value = 290;
									break;
								case 3400: 
									Ref_Velocity_value = 270;
									break;
								case 3600: 
									Ref_Velocity_value = 250;
									break;
								case 3800: 
									Ref_Velocity_value = 230;
									break;
								case 4000: 
									Ref_Velocity_value = 210;
									break;
								default: 
									Ref_Velocity_value = 370;
									break;
							}
							
							if(Current_Velocity_value < (Ref_Velocity_value-10)) m_CurrentPWMDuty += (Ref_Velocity_value - Current_Velocity_value);
							else if(Current_Velocity_value > (Ref_Velocity_value+10)) m_CurrentPWMDuty -= (Current_Velocity_value - Ref_Velocity_value);

							if((motor_voltage >= 1900) && (motor_voltage < 2300)) /* DC 70V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN+400)) 
								{
									m_CurrentPWMDuty = (PWM_MIN+400);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX+300)) 
								{
									m_CurrentPWMDuty = (PWM_MAX+300);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 2300) && (motor_voltage < 2600)) /* DC 80V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN+300)) 
								{
									m_CurrentPWMDuty = (PWM_MIN+300);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX+200)) 
								{
									m_CurrentPWMDuty = (PWM_MAX+200);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 2600) && (motor_voltage < 2900)) /* DC 90V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN+150)) 
								{
									m_CurrentPWMDuty = (PWM_MIN+150);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX+100)) 
								{
									m_CurrentPWMDuty = (PWM_MAX+100);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 2900) && (motor_voltage < 3300)) /* DC 100V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN)) 
								{
									m_CurrentPWMDuty = (PWM_MIN);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX)) 
								{
									m_CurrentPWMDuty = (PWM_MAX);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 3300) && (motor_voltage < 3600)) /* DC 110V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN-70)) 
								{
									m_CurrentPWMDuty = (PWM_MIN-70);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX-170)) 
								{
									m_CurrentPWMDuty = (PWM_MAX-170);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if(motor_voltage >= 3600)
							{
								if(m_CurrentPWMDuty < (PWM_MIN-140)) 
								{
									m_CurrentPWMDuty = (PWM_MIN-140);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX-240)) 
								{
									m_CurrentPWMDuty = (PWM_MAX-240);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else
							{
								if(m_CurrentPWMDuty>600)		m_CurrentPWMDuty = 600;
								else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
								
							}
							
#endif
						}
						else if((currEncPulse >= OPEN_SLOW_CONST_SECTiON) && (currEncPulse<9000))
						{
							closing_pwm_ind_Cnt = 0;
							
							m_CurrentPWMDuty = PWM_MIN;
						}
						else/* currEncPulse>9000 */		
						{
							closing_pwm_ind_Cnt = 0;
							
							m_CurrentPWMDuty=0;
						}
#endif

						
//	jeon_190514				
//						if(m_CurrentPWMDuty>600)		m_CurrentPWMDuty = 600;
//						else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
						mmo_DoorOpeningNchFET(m_CurrentPWMDuty);
//			 	 		printf("\r\n O : currEncPulse : %d, Velocity : %d, m_CurrentPWMDuty : %d \r\n", currEncPulse, mmf_Encoder.Velocity, m_CurrentPWMDuty);
//			 	 		printf("\r\n O[%d] : currEncPulse : %d, Velocity : %d, m_CurrentPWMDuty : %d, Power_Voltage : %d \r\n", mdm_time.OpenConfigtime, currEncPulse, Current_Velocity_value, m_CurrentPWMDuty, powervoltage);
					}
				}
			}
		}
		else if(mmf_PIDControl==PID_CONTROL_CLOSING)
		{
			
			/*#define CLOSE_ACCEL_SECTiON			3000			// 2.5s(3000) - 650mm closing
			  #define CLOSE_CONST_SECTiON			2250			// 2.5s(2250) - 650mm closing
			  #define CLOSE_DEACE_SECTiON			0				// 2.5s(0) - 650mm closing
			 */
			
			if(PID_10msCnt>9)
			{
				PID_10msCnt=0;
				StartClosing =1;

				
				if((currEncPulse>= CLOSE_SLOW_CONST_SECTiON) && (currEncPulse<9000))	
				{
					if(mmf_Encoder.RefVelocity>40)					mmf_Encoder.RefVelocity-=1;  
					else if(mmf_Encoder.RefVelocity<40)					mmf_Encoder.RefVelocity+=1;  
					else											mmf_Encoder.RefVelocity =40;
				}
				else if((currEncPulse<CLOSE_SLOW_CONST_SECTiON) && (currEncPulse<9000))
				{
					if(mmf_Encoder.RefVelocity>15)					mmf_Encoder.RefVelocity-=1;  
					else if(mmf_Encoder.RefVelocity<15)					mmf_Encoder.RefVelocity+=1; 
					else											mmf_Encoder.RefVelocity =15;	
				}
				else/* currEncPulse>9000 */							mmf_Encoder.RefVelocity=0;
				

			}
			else
			{
				PID_10msCnt++;
			}
			
			/*
			 * 처음 Closing 시 방향 설정
			 */
			if(mmf_NextMotorDirSelect==PID_CONTROL_CLOSING)
			{
				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
				mmo_DoorFree();
				mmo_DoorClosing(0);
			}
			/*
			 * Closing PWM Update
			 */
			else
			{
				if(StartClosing)
				{
					mmf_Encoder.Position = mmo_getEncoderCountValue();
					/*
					 * 다 닫히면 0에서 더 닫혀서 underflow 발생하면 9950과 같은 값이 나올 수 있음
					 */

					if(mip_Input.DoorClosedSwitchOn)
					{
						mmf_EndDecision.ClosedByPosition = true;
						prevEncPulse = currEncPulse;
						StartClosing=0;
					}

					else
					{
					    currEncPulse = mmf_Encoder.Position;

					    Close_Curr_Vel_time = osKernelSysTick();
											    
					    if(Close_Curr_Vel_time > Close_Past_Vel_time)
					    {
					    	mmf_Encoder.Velocity = (((prevEncPulse - currEncPulse)*10)/(Close_Curr_Vel_time - Close_Past_Vel_time));
					    }
					    else
					    {
					    	mmf_Encoder.Velocity = (prevEncPulse - currEncPulse);
					    }
						Current_Velocity_value = (int16_t)(((prevEncPulse - currEncPulse)*1000)/(5*(Close_Curr_Vel_time - Close_Past_Vel_time)));

					    Close_Past_Vel_time = Close_Curr_Vel_time;

						prevEncPulse = currEncPulse;													// 10ms마다 이전 엔코더 펄스 업데이트
						Verror = (float)mmf_Encoder.RefVelocity - (float)mmf_Encoder.Velocity;
//jeon_190621						m_CurrentPWMDuty += (Verror*Vc_Pgain);
						
#if 1 //jeon_190621			
						if((mip_Input.DoorClosedSwitchOn) && (currEncPulse < 10))
						{
							m_CurrentPWMDuty=0;
						}
						else if((currEncPulse >= CLOSE_ACCEL_SECTiON) && (currEncPulse<9000))	
						{
#if 1 //jeon_190621			
							switch(mdm_time.CloseConfigtime)
							{
								case 2000: m_CurrentPWMDuty = 750; break;
								case 2200: m_CurrentPWMDuty = 720; break;
								case 2400: m_CurrentPWMDuty = 690; break;
								case 2600: m_CurrentPWMDuty = 660; break;
								case 2800: m_CurrentPWMDuty = 630; break;
								case 3000: m_CurrentPWMDuty = 600; break;
								case 3200: m_CurrentPWMDuty = 570; break;
								case 3400: m_CurrentPWMDuty = 540; break;
								case 3600: m_CurrentPWMDuty = 510; break;
								case 3800: m_CurrentPWMDuty = 480; break;
								case 4000: m_CurrentPWMDuty = 450; break;
								case 4200: m_CurrentPWMDuty = 420; break;
								case 4400: m_CurrentPWMDuty = 390; break;
								case 4600: m_CurrentPWMDuty = 360; break;
								case 4800: m_CurrentPWMDuty = 330; break;
								case 5000: m_CurrentPWMDuty = 300; break;
								default: m_CurrentPWMDuty = 600; break;
							}
							
							if((motor_voltage >= 1900) && (motor_voltage < 2300)) /* DC 70V */
							{
								m_CurrentPWMDuty += 100;
							}
#else
							switch(mdm_time.CloseConfigtime)
							{
								case 2000: 
									Ref_Velocity_value = 490;
									break;
								case 2200: 
									Ref_Velocity_value = 460;
									break;
								case 2400: 
									Ref_Velocity_value = 430; 
									break;
								case 2600: 
									Ref_Velocity_value = 400; 
									break;
								case 2800: 
									Ref_Velocity_value = 380; 
									break;
								case 3000: 
									Ref_Velocity_value = 360; 
									break;
								case 3200: 
									Ref_Velocity_value = 340; 
									break;
								case 3400: 
									Ref_Velocity_value = 310; 
									break;
								case 3600: 
									Ref_Velocity_value = 280;
									break;
								case 3800: 
									Ref_Velocity_value = 240;
									break;
								case 4000: 
									Ref_Velocity_value = 230;
									break;
								case 4200: 
									Ref_Velocity_value = 220; 
									break;
								case 4400: 
									Ref_Velocity_value = 210; 
									break;
								case 4600: 
									Ref_Velocity_value = 200; 
									break;
								case 4800: 
									Ref_Velocity_value = 190;
									break;
								case 5000: 
									Ref_Velocity_value = 180;
									break;
								default: 
									Ref_Velocity_value = 360; 
									break;
							}
							
							if(Current_Velocity_value < (Ref_Velocity_value-10)) m_CurrentPWMDuty += (Ref_Velocity_value - Current_Velocity_value);
							else if(Current_Velocity_value > (Ref_Velocity_value+10)) m_CurrentPWMDuty -= (Current_Velocity_value - Ref_Velocity_value);

							if(m_CurrentPWMDuty < (PWM_MIN)) 
							{
								m_CurrentPWMDuty = (PWM_MIN);
							}	
							else if(m_CurrentPWMDuty > (PWM_MAX)) 
							{
								m_CurrentPWMDuty = (PWM_MAX);
							}
							else
							{
								m_CurrentPWMDuty += 0;
							}

#endif
						}
						else if((currEncPulse >= CLOSE_SLOW_CONST_SECTiON) && (currEncPulse<9000))	
						{
#if 1 //jeon_190717
							switch(mdm_time.CloseConfigtime)
							{
								case 2000: 
									Ref_Velocity_value = 490;
									break;
								case 2200: 
									Ref_Velocity_value = 460;
									break;
								case 2400: 
									Ref_Velocity_value = 430; 
									break;
								case 2600: 
									Ref_Velocity_value = 400; 
									break;
								case 2800: 
									Ref_Velocity_value = 380; 
									break;
								case 3000: 
									Ref_Velocity_value = 360; 
									break;
								case 3200: 
									Ref_Velocity_value = 340; 
									break;
								case 3400: 
									Ref_Velocity_value = 310; 
									break;
								case 3600: 
									Ref_Velocity_value = 280;
									break;
								case 3800: 
									Ref_Velocity_value = 240;
									break;
								case 4000: 
									Ref_Velocity_value = 230;
									break;
								case 4200: 
									Ref_Velocity_value = 220; 
									break;
								case 4400: 
									Ref_Velocity_value = 210; 
									break;
								case 4600: 
									Ref_Velocity_value = 200; 
									break;
								case 4800: 
									Ref_Velocity_value = 190;
									break;
								case 5000: 
									Ref_Velocity_value = 180;
									break;
								default: 
									Ref_Velocity_value = 360; 
									break;
							}
							
							if(Current_Velocity_value < (Ref_Velocity_value-10)) m_CurrentPWMDuty += (Ref_Velocity_value - Current_Velocity_value);
							else if(Current_Velocity_value > (Ref_Velocity_value+10)) m_CurrentPWMDuty -= (Current_Velocity_value - Ref_Velocity_value);

							if((motor_voltage >= 1900) && (motor_voltage < 2300)) /* DC 70V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN+300)) 
								{
									m_CurrentPWMDuty = (PWM_MIN+300);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX+300)) 
								{
									m_CurrentPWMDuty = (PWM_MAX+300);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 2300) && (motor_voltage < 2600)) /* DC 80V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN+250)) 
								{
									m_CurrentPWMDuty = (PWM_MIN+250);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX+250)) 
								{
									m_CurrentPWMDuty = (PWM_MAX+250);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 2600) && (motor_voltage < 2900)) /* DC 90V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN+150)) 
								{
									m_CurrentPWMDuty = (PWM_MIN+150);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX+150)) 
								{
									m_CurrentPWMDuty = (PWM_MAX+150);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 2900) && (motor_voltage < 3300)) /* DC 100V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN)) 
								{
									m_CurrentPWMDuty = (PWM_MIN);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX)) 
								{
									m_CurrentPWMDuty = (PWM_MAX);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if((motor_voltage >= 3300) && (motor_voltage < 3600)) /* DC 110V */
							{
								if(m_CurrentPWMDuty < (PWM_MIN-70)) 
								{
									m_CurrentPWMDuty = (PWM_MIN-70);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX-170)) 
								{
									m_CurrentPWMDuty = (PWM_MAX-170);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else if(motor_voltage >= 3600)
							{
								if(m_CurrentPWMDuty < (PWM_MIN-140)) 
								{
									m_CurrentPWMDuty = (PWM_MIN-140);
								}	
								else if(m_CurrentPWMDuty > (PWM_MAX-240)) 
								{
									m_CurrentPWMDuty = (PWM_MAX-240);
								}
								else
								{
									m_CurrentPWMDuty += 0;
								}
							}
							else
							{
								if(m_CurrentPWMDuty>600)		m_CurrentPWMDuty = 600;
								else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
								
							}
							
#else
//jeon_190624							m_CurrentPWMDuty = 550;
							if(ClosePowerValue == 0) //2500
							{
								m_CurrentPWMDuty = 640; //20190618 44 => 40, 2.5s(44)
							}
							else if(ClosePowerValue == 1) //3000
							{
								m_CurrentPWMDuty = 600; //3s(36) 550
							}
							else if(ClosePowerValue == 2) //3500
							{
								m_CurrentPWMDuty = 460; //3.5s(18)
							}
							else if(ClosePowerValue == 3) //4000
							{
								m_CurrentPWMDuty = 370; //4s(12)
							}
							else if(ClosePowerValue == 4) //4500
							{
								m_CurrentPWMDuty = 300; //4.5s(9)
							}
							else if(ClosePowerValue == 5) //5000
							{
								m_CurrentPWMDuty = 250; //5s(9)
							}
							else //3000
							{
								m_CurrentPWMDuty = 600; //3s(36) 550
							}
#endif
						}
						else if((currEncPulse<CLOSE_SLOW_CONST_SECTiON) && (currEncPulse<9000))
						{
							m_CurrentPWMDuty = PWM_MIN;
							if((motor_voltage >= 1900) && (motor_voltage < 2300)) /* DC 70V */
							{
								m_CurrentPWMDuty = (PWM_MIN+200);
							}
							else if((motor_voltage >= 2300) && (motor_voltage < 2600)) /* DC 80V */
							{
								m_CurrentPWMDuty = (PWM_MIN+150);
							}
							else if((motor_voltage >= 2600) && (motor_voltage < 2900)) /* DC 90V */
							{
								m_CurrentPWMDuty = (PWM_MIN+70);
							}
							else if((motor_voltage >= 2900) && (motor_voltage < 3300)) /* DC 100V */
							{
								m_CurrentPWMDuty = (PWM_MIN+0);
							}
							else if((motor_voltage >= 3300) && (motor_voltage < 3600)) /* DC 110V */
							{
								m_CurrentPWMDuty = (PWM_MIN-20);
							}
							else if(motor_voltage >= 3600)
							{
								m_CurrentPWMDuty = (PWM_MIN-40);
							}
							else
							{
								m_CurrentPWMDuty = (PWM_MIN+0);
							}
						}
						else/* currEncPulse>9000 */							m_CurrentPWMDuty=0;
#endif

// jeon_190514 : voltage lever check - pwm limit						
				
#if 0 //jeon_190624_TEMP
						if((motor_voltage >= 1900) && (motor_voltage < 2300)) /* DC 70V */
						{
							if(m_CurrentPWMDuty>950)		m_CurrentPWMDuty = 950;
							else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
						}
						else if((motor_voltage >= 2300) && (motor_voltage < 2600)) /* DC 80V */
						{
							if(m_CurrentPWMDuty>800)		m_CurrentPWMDuty = 800;
							else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
						}
						else if((motor_voltage >= 2600) && (motor_voltage < 2900)) /* DC 90V */
						{
							if(m_CurrentPWMDuty>700)		m_CurrentPWMDuty = 700;
							else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
						}
						else if((motor_voltage >= 2900) && (motor_voltage < 3300)) /* DC 100V */
						{
							if(m_CurrentPWMDuty>550)		m_CurrentPWMDuty = 550; //jeon_190618 600=>550
							else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
						}
						else if((motor_voltage >= 3300) && (motor_voltage < 3600)) /* DC 110V */
						{
							if(m_CurrentPWMDuty>520)		m_CurrentPWMDuty = 520;
							else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
						}
						else if(motor_voltage >= 3600)
						{
							if(m_CurrentPWMDuty>430)		m_CurrentPWMDuty = 430;
							else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
						}
						else
						{
							if(m_CurrentPWMDuty>600)		m_CurrentPWMDuty = 600;
							else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;
							
						}
#endif
						
						
//	jeon_190514				
//						if(m_CurrentPWMDuty>700)	m_CurrentPWMDuty = 700;
//						else if(m_CurrentPWMDuty<0)		m_CurrentPWMDuty = 0;

//test_jeon_190621						m_CurrentPWMDuty = 200;
						mmo_DoorClosingNchFET(m_CurrentPWMDuty);
//			 	 		printf("\r\n C : Enc : %d, Vel : %d, PWM : %d \r\n", currEncPulse, Current_Velocity_value, m_CurrentPWMDuty);
			 	 		
//			 	 		printf("\r\n C : currEncPulse : %d, Velocity : %d, m_CurrentPWMDuty : %d \r\n", currEncPulse, mmf_Encoder.Velocity, m_CurrentPWMDuty);
					}
				}
			}
		}
		else
		{
			/*
			 * todo : 초기화 상태에서 PID제어를 안할 때 닫힘완료 판단을 위해 추가
			 *        ISR에서 계속 수행하면 안되므로 딴데서 수행해야 함.
			 */
			mmf_Encoder.Position = mmo_getEncoderCountValue();
			if((mdc_DoorState == DOOR_CLOSED) ||(mdc_DoorState == DOOR_OPENED))
			{
//				currEncPulse = 0;
//				prevEncPulse = 0;
				prevEncPulse = mmf_Encoder.Position;
				m_CurrentPWMDuty = 0;
				mmf_Encoder.RefVelocity=0;
				mmf_Encoder.Velocity=0;
				PID_10msCnt =0;
				OpeningAccBeforDLS = 0;
				OpeningAccAfterDLS = 0;
			}
			if(mdc_isFirstClosed == true)
			{
				if(error_list_flag[DCU_ENCODER_ERROR] == false)
				{
#if 0 //jeon_190618
					if((mmf_Encoder.Position < DCU_OBS_POSITION) || (mmf_Encoder.Position>9000))
					{
						//mmo_DoorFree();
						mmf_EndDecision.ClosedByPosition = true;
						prevEncPulse = currEncPulse;
						StartClosing=0;

					}
#else
					if(mip_Input.DoorClosedSwitchOn)
					{
						mmf_EndDecision.ClosedByPosition = true;
						prevEncPulse = currEncPulse;
						StartClosing=0;
					}
#endif
				}
			}
		}
    }
	#else
		if(mmf_PIDControl==PID_CONTROL_OPENING)
		{
			/*
			 * Position < 2500 인 구간에서만 Opening 동작 수행
			 */
			if(currEncPulse<m_OpeningEncoderPulse)				mmf_Encoder.RefVelocity = 50;
			else														mmf_Encoder.RefVelocity = 0;
		}
		else if(mmf_PIDControl==PID_CONTROL_CLOSING)
		{
			/*
			 * 0 < Position < 3000인 구간에서만 closing 동작 수행
			 */
			if((currEncPulse>0) && (currEncPulse<3000))	mmf_Encoder.RefVelocity = -50;
			/*
			 * Position < 0 인 구간에서는 엔코더 0에서 Closing을 더하면 underflow 발생해서 9985으로 값이 갑자기 증가함 
			 */
			else														mmf_Encoder.RefVelocity = 0;
		}
		else
		{
			m_CurrentPWMDuty = 0;
			mmf_Encoder.RefVelocity=0;
		}
	#endif
}
#endif

void mmf_vTimerCallback_VrefGenerator(void)
{
    /*
     * PID제어 미수행 시 속도에 따라 바로 PWM출력 수행
     */
    #if 0
    {
        volatile static uint32_t Cnt_100ms=0, CurrTime=0, PwmOut=0;
        volatile static uint32_t OpenStartTime=0, CloseStartTime=0;
        volatile static uint32_t WeightingFactor_O=99, WeightingFactor_C=101;											// 마찰, 모터응답속도 등의 오차요인을 고려하여 출력을 더 높여서 맞췄음
    
		CurrTime = osKernelSysTick();
		
		if(g_CurrentPWM_Ch==PWM_CH1_OPENING_FORWARD)
		{
			if(OpenStartTime == 0)																				// Opening 시 처음 한번만 저장
			{
				Cnt_100ms = 0;
				OpenStartTime = CurrTime;
			}
			if(CurrTime<(OpenStartTime+500))		Cnt_100ms++;												// 시작 ~ 0.5s	: 가속
			else if(CurrTime<(OpenStartTime+2500))	{}															// 0.5s ~ 3		: 등속
			else if(CurrTime<(OpenStartTime+3000))	{if(Cnt_100ms>0) Cnt_100ms--;}								// 3s ~ 3.5s	: 감속
			else									{Cnt_100ms=0;}												// Set PWM_Out to zero
			
			/*
			 * 계산식 "PwmOut = Cnt_100ms*93" 유도
			 * 
			 * 1. PWM과 Velocity(pulse/s)와의 관계식
			 *    -> 874pwm 출력을 보내면 1870pulse/s의 속도가 나옴
			 *    -> 874:1870 = pwm:pulse/s
			 *              874
			 *    -> PWM = ------ x pulse/s
			 *              1870
			 *              
			 * 2. 93만큼 PWM을 증가/감소 시켜 사다리꼴 속도 프로파일 생성
			 *    [pulse/s]               [PWM]
			 *  1000 _|_____________________|_ 467
			 *   800 _|____/|         |\____|_ 373
			 *   600 _|___/ |         | \___|_ 280
			 *   400 _|__/  |         |  \__|_ 186
			 *   200 _|_/   |         |   \_|_  93
			 *	   0 _|/    |         |    \|_   0
			 *	   ---|-----|---------|-----|------ [s]
			 *	      |     |         |     |
			 *        |0.5s | 2s 등속 | 0.5s|
			 *        |가속 |         | 감속|
			 *        
			 * 사다리꼴 속도 프로파일에서 면적은 이동거리 S(pulse)가 되고,
			 * 3s동안 이동해야 하는 거리(pulse)는 TYPE_B이므로 2500[pulse]만큼 이동해야 하므로
			 * -> S(pulse) = (1/2*V_ref*0.5)*2 + V_ref*2 = 2500
			 *               2.5*V_ref = 2500  
			 *    따라서
			 *    V_ref = 1000[pulse/s]가 된다.
			 * 
			 * 실제확인 결과 피드백 제어를 수행하지 않으므로 
			 * 마찰, 모터응답속도 등의 오차요인을 고려하여 PWM출력을 조정해야 함
			 */
			PwmOut = Cnt_100ms*WeightingFactor_O;
			pwm_channel_update_duty(PWM,&g_pchFET_ctla_pwmh[g_CurrentPWM_Ch],PwmOut);
			m_CurrentPWMDuty = (int32_t)PwmOut;
		}
		else if(g_CurrentPWM_Ch==PWM_CH3_CLOSING_REVERSE)
		{
			if(CloseStartTime==0)
			{
				Cnt_100ms = 0;
				CloseStartTime = CurrTime;
			}
			
			if(CurrTime<(CloseStartTime+500))		Cnt_100ms++;												// 시작 ~ 0.5s	: 가속
			else if(CurrTime<(CloseStartTime+2500))	{}															// 0.5s ~ 3		: 등속
			else if(CurrTime<(CloseStartTime+3000))	{if(Cnt_100ms>0) Cnt_100ms--;}								// 3s ~ 3.5s	: 감속
			else									{Cnt_100ms=0;}	// Set PWM_Out to zero
			
			/*
			 * 계산식 PwmOut = Cnt_100ms*94
			 */
			PwmOut = Cnt_100ms*WeightingFactor_C;
			pwm_channel_update_duty(PWM,&g_pchFET_ctla_pwmh[g_CurrentPWM_Ch],PwmOut);            	
			m_CurrentPWMDuty = (int32_t)PwmOut;
		}
		else
		{
			// No PWM Channel Selected -> PWM Update 하면 안됨
			m_CurrentPWMDuty = 0;
		}
    }
    /*
     * PID제어 수행 시 속도제어 목표값(Vref)만을 생성하고 PID함수에서 PWM출력 수행
     */
    #elif 0
    {
        volatile static uint32_t CurrTime=0, OpenStartTime=0, CloseStartTime=0;
        volatile static int32_t Cnt_100ms=0;
        
		CurrTime = osKernelSysTick();
		
		if(g_CurrentPWM_Ch==PWM_CH1_OPENING_FORWARD)
		{
			if(OpenStartTime == 0)																				// Opening 시 처음 한번만 저장
			{
				Cnt_100ms = 0;
				OpenStartTime = CurrTime;
			}
			if(CurrTime<(OpenStartTime+500))		Cnt_100ms++;												// 0.0s ~ 0.5s	: 가속
			else if(CurrTime<(OpenStartTime+2500))	{}															// 0.5s ~ 2.5s	: 등속
			else if(CurrTime<(OpenStartTime+3000))	{if(Cnt_100ms>0) Cnt_100ms--;}								// 2.5s ~ 3.0s	: 감속
			else									{Cnt_100ms=0;}
			
			/*
			 * Opening 시 사다리꼴 속도 프로파일 생성 (3s동안 2500[pulse] 이동)
			 *    [pulse/s]
			 *  1000 _|     ___________
			 *   800 _|____/|         |\
			 *   600 _|___/ |         | \
			 *   400 _|__/  |         |  \
			 *   200 _|_/   |         |   \
			 *	   0 _|/    |         |    \
			 *	   ---|-----|---------|-----|------ [s]
			 *	      |     |         |     |
			 *        |0.5s | 2s 등속  | 0.5s|
			 *        | 가속 |         | 감속 |
			 *        
			 * 사다리꼴 속도 프로파일에서 면적은 이동거리 S(pulse)가 되고,
			 * 3s동안 이동해야 하는 거리(pulse)는 TYPE_B이므로 2500[pulse]만큼 이동해야 하므로
			 * -> S(pulse) = (1/2*V_ref*0.5)*2 + V_ref*2 = 2500
			 *               2.5*V_ref = 2500
			 *                   V_ref = 1000
			 *    따라서
			 *    V_ref = 1000[pulse/s]가 되고, 0.5s 동안 100ms마다 200[pulse/s] 씩 속도를 가/감속 한다.
			 */
//       			mmf_Encoder.RefVelocity = Cnt_100ms*200;
		}
		else if(g_CurrentPWM_Ch==PWM_CH3_CLOSING_REVERSE)
		{
			if(CloseStartTime==0)
			{
				Cnt_100ms = 0;
				CloseStartTime = CurrTime;
			}
			
			if(CurrTime<(CloseStartTime+1000))		Cnt_100ms--;												// 0.0s ~ 1.0s	: 가속
			else if(CurrTime<(CloseStartTime+2500))	{}															// 1.0s ~ 4.0s	: 등속
			else if(CurrTime<(CloseStartTime+3000))	{if(Cnt_100ms<0) Cnt_100ms++;}								// 4.0s ~ 5.0s	: 감속
			else									{Cnt_100ms=0;}
			
			/*
			 * Closing 시 사다리꼴 속도 프로파일 생성 (5s동안 2500[pulse] 이동)
			 *    [pulse/s]
			 *        |
			 *	   ---|-----|---------|-----|------ [s]
			 *	   0 _|\    |         |    /|
			 *  -125 _|_\   |         |   / |
			 *  -250 _|__\  |         |  /  |
			 *  -375 _|___\ |         | /   |
			 *  -500 _|____\|         |/    |
			 *  -625 _|     -----------     |
			 *	      |     |         |     |
			 *        | 1s  | 3s 등속  |  1s |
			 *        |가속  |         | 감속 |
			 * 
			 * 사다리꼴 속도 프로파일에서 면적은 이동거리 S(pulse)가 되고,
			 * 5s동안 이동해야 하는 거리(pulse)는 TYPE_B이므로 -2500[pulse]만큼 이동해야 하므로
			 * -> S(pulse) = (1/2*V_ref*1)*2 + V_ref*3 = -2500
			 *                 4*V_ref = -2500
			 *                   V_ref = -625
			 *    따라서
			 *    V_ref = 625[pulse/s]가 되고, 1s 동안 625[pulse/s]를 100ms마다 속도 가/감속을 한다.
			 */
//    			mmf_Encoder.RefVelocity = Cnt_100ms*62;
		}
		else
		{
			// No PWM Channel Selected
			mmf_Encoder.RefVelocity = 0;
		}
    }
    #else
    #endif
}

void setPIDPWMOut(void)
{
    PID_Velocity=(float)0;
}

static void DiagnosisInMotorFeedback(void)
{
    /*
     * 닫힘 스위치(DCS) 고장 진단
     */
    if((mdc_DoorState == DOOR_CLOSED) && (mdc_PreDoorState==DOOR_CLOSING))		// 도어가 닫히는 시점에
    {
    	if((mip_Input.di1_DCS1==false) || (mip_Input.di1_DCS2==false))			// DCS가 눌려있지 않으면 DCS 오류
    	{
    		
    	}
    	else																	// DCS가 눌려있으면 정상
    	{
    		
    	}
    }
    else if((mdc_DoorState == DOOR_OPENING) && (mdc_PreDoorState==DOOR_CLOSED))	// 도어가 열리는 시점에
    {
    	if((mip_Input.di1_DCS1==true) || (mip_Input.di1_DCS2==true))			// DCS가 눌려있으면 DCS 오류
    	{
    		
    	}
    	else																	// DCS가 눌려있지 않으면 정상
    	{
    		
    	}
    }
    else
    {
    	// No Check
    }
    
    /*
     * 모터 고장 진단
     */
    
    /*
     * 엔코더 고장 진단
     */
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.c
*********************************************************************/
