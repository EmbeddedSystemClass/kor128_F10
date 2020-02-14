/*********************************************************************
	Rhapsody in C	: 8.1.5 
	Login		: sung
	Component	: DefaultBuild 
	Configuration 	: DefaultConfig
	Model Element	: mtMotorFeedback
//!	Generated Date	: ��, 1, 7 2017  
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.c
*********************************************************************/

#include "mtMotorFeedback.h"
/* ���� ���¿� ���� ���ڴ��޽� +/- ī��Ʈ�� ���� ��Ŭ��� */
/*## dependency mtDecisionControl */
#include "mtDecisionControl.h"
/* DCS �������� �Ǵ��� ���� ��Ŭ��� */
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
volatile static int32_t motorOpenPower=7;																// max 100���� �ϸ� PWM 800�̻� ��� ��
volatile static int32_t motorClosePower=3;

uint8_t bemf_task_flag=false;
#if 1
void mmf_ConversionADC(void) {
    /*
     * ADC ��ȯ ����
     */

	#ifdef HW_DEPENDENT_CODE
	/*
	 * todo : ADC ���������� �Ǵ��� Ȯ���� ���� ��
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
	 * �����ΰ�����	= ���Ϳ��ɸ������� + Back_emf����
	 *                PWM�Է����� ������ ��� �ٲ�
	 * 
	 * PowerVoltage = MotorVoltage  + V_bemf
	 * 
	 * 	M3_3V_MONITOR - PA4, ADC1_IN4
	 *  - 3.3V : 4095 = 2.6V : y
	 *  - y = (4095*2.6)/3.3 = about 3226
	 */
	#ifdef DEBUG_SIMULATOR
		volatile static int16_t currEncPulse=0, prevEncPulse=0;												// 1ms irsȣ�� �ø��� ���� ���ڴ� �޽� ������Ʈ
		currEncPulse = mmo_getEncoderCountValue();
		
		if((mmf_PIDControl==PID_CONTROL_OPENING)||
		   (currEncPulse>prevEncPulse))						// Free���� ������ ���� �� ���ڴ� �޽� ��ȭ�� ���� �ڵ����� ADC����
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
				(currEncPulse<prevEncPulse))				// Free���� ������ ���� �� ���ڴ� �޽� ��ȭ�� ���� �ڵ����� ADC����
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
    	volatile static float CurrMotorCurrent=0, CurrPowerVoltage=0, CurrMotorVoltage=0;	// ���� ������
    	volatile static float PrevMotorCurrent=0, PrevPowerVoltage=0, PrevMotorVoltage=0;	// ���� ������
    	volatile static float alpha=0.9;													// ���Ͱ��
    #else
    	int32_t SumSample=0;
    	int8_t i;
    	double tmp=0;
    #endif
    /*
     * B_emf ADC�� ���� Ÿ�̸� ����
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
  		mmf_ConversionADC();					//���⼭ �������־�� ���ڴ� ���峵�������� ������ �����ؼ� �����Ϸ� ,�����Ϸ� �Ǵ��� �Ҽ���

#if 0 //jeon_190822_Temp display  		
  		printf("\r\nm_adc.MotorVoltage(ADC3_IN8) : %d \r\n", m_adc.MotorVoltage);
  		printf("m_adc.PowerVoltage(ADC1_IN5) : %d \r\n", m_adc.PowerVoltage);
  		printf("m_adc.MotorCurrent(ADC1_IN9) : %d \r\n", m_adc.MotorCurrent);
#endif  		
  		
    	/*
    	 * Encoder Mode���� ADC
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
    	 * B_emf Mode���� ADC
    	 */
    	else
    	{
    		/*
    		 * todo: binary�������� �ι� get�ؼ� �����ϴ°� �ƴ��� Ȯ�� �� ��
    		 *       Block Time�� ���Ѵ�� �ϴ� ��� ����� �� ��
    		 */
//			if(xSemaphoreTake(g_semaphoreBEMF, portMAX_DELAY) == pdTRUE)					// TC0_CH0�� �������� Give�ϸ� ����
        	if(xSemaphoreTake(g_semaphoreBEMF, 1) == pdTRUE)					// TC0_CH0�� �������� Give�ϸ� ����
        	{
        		bemf_task_flag = true;
        		#ifdef LPF
        		{
        			/*
        			 * Recursive LowPass Filter Equation
        			 * ���������� = alpha*���������� + (1-alpha)*�����Է°�
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
        		 * B_emf Voltage ���
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
         			 * �ӵ�/��ġ ���
         			 */
            		if(mmf_PIDControl==PID_CONTROL_OPENING)
            		{
            			/*
            			 * �ӵ����
						 * �ӵ��� 100[pulse/100ms]�� ��� 
						 * - 1000[pulse/s]�� �ӵ��� �ǰ�, 1�ʴ� 1000pulse�� �߻���
						 *   (���� �̵��ؾ� �� �Ÿ��� 2500 pulse)
            			 */
 //jeon_190715           			mmf_Bemf.Velocity = (float)m_adc.BemfVoltage*(float)0.73;
            			
                	    /*
                	     * ��ġ���
                	     * �ӵ��� 400[pulse/s]�� ���
                	     * 1�ʿ� 400[pulse]
                	     * 0.1s�� 40[pulse]
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
            			 * �ӵ����
            			 */
//jeon_190715            			mmf_Bemf.Velocity = (float)m_adc.BemfVoltage*(float)-0.73;
            			
            			/*
            			 * ��ġ���
            			 */
//jeon_190715            			mmf_Bemf.Position += mmf_Bemf.Velocity*(float)0.1;			// �ӵ� ��ȣ�� (-)�̹Ƿ� �׳� ���ϸ� ��ġ���� ����
            			
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
        		 * Bemf���� ������ ���� Stop�� Timer �ٽ� ����
        		 */
        		
        		/*
        		 * 100ms ���� �� �ٽ� Bemf ���� ADC ����
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
     * todo:Bemf ���� ������ ���ͷ�Ʈ ���� �����ؾ� ��.
     */
    // cpu_irq_disable();
    // __disable_irq();
    // cpu_irq_enable();
    // __enable_irq();
    
    /*
     * �������¿����� B_emf ��� �� ��� ������� ����
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
    		xSemaphoreGiveFromISR(g_semaphoreBEMF,pdFALSE);								// BemfTask���� V_bemf�� ���� �ӵ�/��ġ ��� <- 0.001s ~ 0.079s ���� ����� ����Ϸ� ��
    	}
    	else if(tick1msCnt==96)															// tick1msCnt == 80
    	{
#if 0 //jeon_190715, �����Ҹ�...
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

/* ��������(mmf_PIDControl == None)������ ȣ����� ���� */
#if 0
void mmf_isrBemf_VelocityControl(void)
{
    volatile static float V_Pgain=0.1, V_Igain=0, V_Dgain=0;
    volatile static float SumVerror=0, PrevVerror=0, PrevPID_Velocity=0;
    volatile static uint32_t Cnt_10ms=0;
    volatile static int32_t PrevMotorVelocity=0;
    float Verror=0;
    
    /*
     * 100ms �������� V_ref ����
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
     * �̵�(Opening or Closing) �� 10ms �������� PID �ӵ����� ����
     */
    #if 1
    {
    //	if(mmf_PIDControl>0)
    	if((mmf_PIDControl==PID_CONTROL_OPENING) || (mmf_PIDControl==PID_CONTROL_CLOSING))
    	{
    		/*
    		 * �ӵ����
    		 * �ӵ��� 100[pulse/100ms]�� ��� 
    		 * - 1000[pulse/s]�� �ӵ��� �ǰ�, 1�ʴ� 1000pulse�� �߻���
    		 *   (���� �̵��ؾ� �� �Ÿ��� 2500 pulse)
    		 */
    		/*
    		 ** BemfTask���� B_emf������ �ӵ��� ȯ���Ͽ� mmf_MotorVelocity�� ������Ʈ ����
    		 */
    		
    		/*
    		 * #EDCU-SW-NFR-P-01
    		 */
    		/*
    		 * #EDCU-SW-NFR-P-02
    		 */
    		/*
    		 * Velocity PID����
    		 *  -> ÷���� MotorRefVelocity�� �ٲ��� ����, Vref�� 0�� ���¿��� Run 1~2ȸ ���� �� �� �ٲ� ��
    		 *     (ø���� MotorRefVelocity�� �ٲٸ� PWM�� ��µǴµ� ���Ͱ� ������ �ʳ�, �ֱ׷��� ���߿� Ȯ���� ���� ��)
    		 */
    		Verror = mmf_Bemf.RefVelocity - mmf_Bemf.Velocity;
    		SumVerror += Verror;
    		PID_Velocity +=	(
    							(Verror*V_Pgain) + 													// P����
    							(SumVerror*V_Igain) +												// I����
    							((Verror-PrevVerror)*V_Dgain)										// D����
    						);
    		PrevVerror = Verror;
    		
    		/*
    		 * �ӵ��� ���ӿ����� �������� �ʾƼ� �ӵ� �������� ������ �������� ����
    		 * -> ó�� ���� ���� FET Switching�� �������� ���ؼ� ������
    		 * -> DoorOpening(0), DoorClosing(0) ȣ�� ���ؼ� ���� �������� ȸ���ϰų�, �ƿ� �������� �ʰų�
    		 */
    		/*
    		 * Opening PWM ���
    		 */
    		if(mmf_PIDControl==PID_CONTROL_OPENING)
    		{
    			/*
    			 * ó�� Opening���� H-Bridge ����Ī
    			 * (PWM��� Update���� DoorOpening()/DoorClosing() ȣ���ϸ� ����Ī ������ �߻�)
    			 */
    			if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
     			{
    				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
    				/*
    				 * Max2000[pulse/s] �ӵ��� Closing �ϴٰ� �������� �ٷ� MAx 2000[pulse/s] �ӵ��� 
    				 * Opening�ϸ� ���������� �귯 Reset �Ǵ� �� ����.
    				 * �����ϰ� Free�� ���� ���� 20ms ������ �ְ� H-Bridge Switching
    				 */
    //    			mmo_DoorBrake();
    				mmo_DoorFree();
    				mmo_DoorOpening(0);
    			}
    			
      			/*
    			 * todo emu: �̰� �����ϸ� 2~3ȸ�� Closing �� �Ǵٰ�
    			 *       ����� �������� �ʰ� �ӵ��� ���Ǿ�, ���� �ֱ���â ��� Closing PWM���
    			 *       ���� ���� ��
    			 */
    //    		if(mmf_Bemf.Position>m_OpeningEncoderPulse)									// �� ������
    //    		{
    //    			mmo_DoorFree();
    //     			mmo_DoorBrake();														// ������ ����(Opened), cdshim_20180904
    //    		}
    //    		else
    			{
        			if(PID_Velocity>(float)800)	{PID_Velocity = (float)800;}
        			m_CurrentPWMDuty = (int32_t)PID_Velocity;
					mmo_DoorOpeningNchFET(m_CurrentPWMDuty);
    			}
    		}
    		/*
    		 * Closing PWM ���
    		 */
    		else if(mmf_PIDControl==PID_CONTROL_CLOSING)
    		{
    			/*
    			 * ó�� Closing���� H-Bridge ����Ī
    			 */
    			if(mmf_NextMotorDirSelect==PID_CONTROL_CLOSING)
    			{
    				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
    //    			mmo_DoorBrake();
    				mmo_DoorFree();
    				mmo_DoorClosing(0);
    			}
    			
      			/*
    			 * todo emu: �̰� �����ϸ� 2~3ȸ�� Closing �� �Ǵٰ�
    			 *       ����� �������� �ʰ� �ӵ��� ���Ǿ�, ���� �ֱ���â ��� Closing PWM���
    			 *       ���� ���� ��
    			 */
    //     		if(mmf_Bemf.Position<30)												// �� ������(0��ġ���� ���� �ӵ��� �ſ� ������, ���� �ÿ��� B_emf�� �߻����� �����Ƿ� ��ġ�� ������ ����
    //     		{
    //     			mmo_DoorFree();
    //     			mmo_DoorBrake();													//  ������ ����(Closed), cdshim_20180904
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
    volatile static int16_t currEncPulse=0, prevEncPulse=0;												// 1ms irsȣ�� �ø��� ���� ���ڴ� �޽� ������Ʈ
    float Verror=0;
    
	if(mmf_PIDControl==PID_CONTROL_OPENING)
	{
		/*
		 * 100ms �������� V_ref ����
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
	     * Opening �� 10ms �������� PID �ӵ����� ����
	     */
		if(PID_10msCnt > 10)
		{
			/*
			 * �ӵ����
			 * �ӵ��� 100[pulse/100ms]�� ��� 
			 * - 1000[pulse/s]�� �ӵ��� �ǰ�, 1�ʴ� 1000pulse�� �߻���
			 *   (���� �̵��ؾ� �� �Ÿ��� 2500 pulse)
			 */
			/*
			 ** BemfTask���� B_emf������ �ӵ��� ȯ���Ͽ� mmf_MotorVelocity�� ������Ʈ ����
			 */
			Verror = mmf_Bemf.RefVelocity - mmf_Bemf.Velocity;
			PID_Velocity +=	(Verror*V_Pgain);
			
			/*
			 * �ӵ��� ���ӿ����� �������� �ʾƼ� �ӵ� �������� ������ �������� ����
			 * -> ó�� ���� ���� FET Switching�� �������� ���ؼ� ������
			 * -> DoorOpening(0), DoorClosing(0) ȣ�� ���ؼ� ���� �������� ȸ���ϰų�, �ƿ� �������� �ʰų�
			 */
			/*
			 * ó�� Opening���� H-Bridge ����Ī
			 * (PWM��� Update���� DoorOpening()/DoorClosing() ȣ���ϸ� ����Ī ������ �߻�)
			 */
			if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
			{
				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
				/*
				 * Max2000[pulse/s] �ӵ��� Closing �ϴٰ� �������� �ٷ� MAx 2000[pulse/s] �ӵ��� 
				 * Opening�ϸ� ���������� �귯 Reset �Ǵ� �� ����.
				 * �����ϰ� Free�� ���� ���� 20ms ������ �ְ� H-Bridge Switching
				 */
				mmo_DoorFree();
				mmo_DoorOpening(0);
			}
	    	else
			{
				if(StartOpening)
				{
					if(mmf_Bemf.Position>m_OpeningEncoderPulse)									// �� ������
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
		 * 100ms �������� V_ref ����
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
	     * Closing �� 10ms �������� PID �ӵ����� ����
	     */
		if(PID_10msCnt > 10)
		{
			Verror = mmf_Bemf.RefVelocity - mmf_Bemf.Velocity;
			PID_Velocity +=	(Verror*V_Pgain);
			
			/*
			 * ó�� Closing���� H-Bridge ����Ī
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
					if(mmf_Bemf.Position<30)												// �� ������(0��ġ���� ���� �ӵ��� �ſ� ������, ���� �ÿ��� B_emf�� �߻����� �����Ƿ� ��ġ�� ������ ����
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
     * Called by B��(TC0_TIOB) Capture Input Interrupt
     */
	#ifdef HW_DEPENDENT_CODE
    #endif
}

/* TC0_CH0�� Interrupt Service Routine���� 10ms���� ȣ��Ǿ� 100ms���� �����Ǵ� Vref�� 10ms���� PID���� ���� */
#if 0
void mmf_isrEncoder_VelocityControl(void)
{
    volatile static float V_Pgain=1, V_Igain=0, V_Dgain=0;
    volatile static float SumVerror=0, PrevVerror=0, PrevPID_Velocity=0;
    volatile static float Verror=0;
    volatile static int32_t Vref_1msCnt=0, PID_1msCnt=0;
    volatile static int32_t currEncPulse=0, prevEncPulse=0;		// 1ms irsȣ�� �ø��� ���� ���ڴ� �޽� ������Ʈ
    
    #if 0
    /*
     * ���ļ��κ��� �ӵ����ϴ� ��� ??? (���� �ȵ�)
     * https://www.portescap.com/node/2032
     * ���� �ӵ��� ���� ������ ����Ͽ� ī��Ʈ ���ļ��κ��� �Ļ��� �� �ֽ��ϴ�. N = f x 60 / CPR ���⼭:
     * N = ���� �ӵ�(�д� ȸ��)
     * f = ���ڴ� ī��Ʈ ���ļ�(Hz)
     * CPR = ȸ���� ī��Ʈ ��(ȸ���� ���� ��/�޽��� 4��)
     * 
     * capture input�� ���� �޽� ���ļ��� �̿��ؼ� �ӵ��� �����ϴ� ���
     * ��DSP Ư���η�-2. MOTORȸ�� �ӵ� ���� ��� ����.pdf ���ϡ� Ȯ��
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
    
    currEncPulse = mmf_Encoder.Position;	// 1ms���� ���� ���ڴ��޽� ������Ʈ
    
    /*
     * 100ms �������� V_ref ����
     */
    #if 1
    {
    	if(mmf_PIDControl==PID_CONTROL_OPENING)
    	{
    		/*
    		 * Opening �� ��ġ�� ���� ��ٸ��� �ӵ����� (�ð� ���� �ȿ� �������� Ʃ�� �ʿ�)
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
    				else								mmf_Encoder.RefVelocity=0;		// 2500 �̻�
    			#else // �̰ɷ� Ȯ��
    				if(currEncPulse<500)				{mmf_Encoder.RefVelocity+=2;}	// cdshim_20180904
    				else if(currEncPulse<1000)			{mmf_Encoder.RefVelocity+=0;}
    				else if(currEncPulse<2500)
    				{
    					if(mmf_Encoder.RefVelocity>2)	mmf_Encoder.RefVelocity-=4;
    					else							mmf_Encoder.RefVelocity =2;		// 2500��ġ�� ������ ������ PWM����� ������ ��
    				}
    				else/* currEncPulse>2500 */			mmf_Encoder.RefVelocity=0;
    			#endif
    		}
    	}
    	else if(mmf_PIDControl==PID_CONTROL_CLOSING)
    	{
    		/*
    		 * Closing �� ��ġ�� ���� ��ٸ��� �ӵ����� (�ð� ���� �ȿ� �������� Ʃ�� �ʿ�)
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
    				else							mmf_Encoder.RefVelocity=0;			// 0 ����
    			#else // �̰ɷ� Ȯ��
    				if(currEncPulse>2000)				{mmf_Encoder.RefVelocity-=1;}	
    				else if(currEncPulse>1500)			{mmf_Encoder.RefVelocity-=0;}
    				else if(currEncPulse>0)
    				{
    					if(mmf_Encoder.RefVelocity<-2)	mmf_Encoder.RefVelocity+=1;
    					else							mmf_Encoder.RefVelocity =-1;	// 0��ġ�� ������ ������ PWM����� ������ ��
    				}
    				else/* currEncPulse<0 */			mmf_Encoder.RefVelocity=0;
    			#endif
    		}
    	}
    	else
    	{
    		/*
    		 * Opening �� 30s�� �������µ� 900mm��ŭ �� �̵��ؼ� Opened ���°� �Ǹ�
    		 * Vref_1msCnt ���� 30�̻���� ī��Ʈ�� ���ؼ� 0���� Reset���� �ʰ�,
    		 * 3s�� Closing ���·� ����Ǹ� ���� ī��Ʈ ������ �ٽ� �����ϰ� �ǹǷ�
    		 * 
    		 * Opening/Closing PID Control�� �������� ������ Decision��⿡�� Brake�� ����̹Ƿ�
    		 * Vref_1msCnt ���� 0���� Reset ���ش�.
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
     * �̵�(Opening or Closing) �� 10ms �������� PID �ӵ����� ����
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
        		 * �ӵ����
        		 * �ӵ��� 100[pulse/100ms]�� ��� 
        		 * - 1000[pulse/s]�� �ӵ��� �ǰ�, 1�ʴ� 1000pulse�� �߻���
        		 *   (���� �̵��ؾ� �� �Ÿ��� 2500 pulse)
        		 */
//            	mmf_Encoder.Velocity = (currEncPulse - prevEncPulse)*100;					// �ӵ� = ���ڴ���ȭ��/10ms = ���ڴ���ȭ��/0.01s = 100 x ���ڴ���ȭ��/s
        		mmf_Encoder.Velocity = (currEncPulse - prevEncPulse);
    			prevEncPulse = currEncPulse;													// 10ms���� ���� ���ڴ� �޽� ������Ʈ
    			
        		/*
        		 * Velocity PID����
        		 *  -> ÷���� MotorRefVelocity�� �ٲ��� ����, Vref�� 0�� ���¿��� Run 1~2ȸ ���� �� �� �ٲ� ��
        		 *     (÷���� MotorRefVelocity�� �ٲٸ� PWM�� ��µǴµ� ���Ͱ� ������ �ʳ�, �ֱ׷��� ���߿� Ȯ���� ���� ��)
        		 */
        		Verror = (float)mmf_Encoder.RefVelocity - (float)mmf_Encoder.Velocity;
        		SumVerror += Verror;
            	PID_Velocity +=	(
            						(Verror*V_Pgain) + 											// P����
            						(SumVerror*V_Igain) +										// I����
            						((Verror-PrevVerror)*V_Dgain)								// D����
            					);
        		PrevPID_Velocity = PID_Velocity;
        		PrevVerror = Verror;
        		
        		/*
        		 * Opening PWM ���
        		 */
//            	if(PID_Velocity>(float)0)
    			if(mmf_PIDControl==PID_CONTROL_OPENING)
        		{
        			/*
        			 * ó�� Opening���� H-Bridge ����Ī
        			 * (PWM��� Update���� DoorOpening()/DoorClosing() ȣ���ϸ� ����Ī ������ �߻�)
        			 */
//            		if(PrevPID_Velocity<=(float)0)
//    				if(PrevPID_Velocity==(float)0)
    				if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
        			{
        				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
        				/*
        				 * Max2000[pulse/s] �ӵ��� Closing �ϴٰ� �������� �ٷ� MAx 2000[pulse/s] �ӵ��� 
        				 * Opening�ϸ� ���������� �귯 Reset �Ǵ� �� ����.
        				 * �����ϰ� Free�� ���� ���� 20ms ������ �ְ� H-Bridge Switching
        				 */
    					//mmo_DoorBrake();
        				mmo_DoorFree();
        				mmo_DoorOpening(0);
        			}
        			
        			if(currEncPulse>m_OpeningEncoderPulse)										// �� ������
        			{
        				mmo_DoorFree();
         				mmo_DoorBrake();														// ������ ����(Opened), cdshim_20180904
        			}
        			else
        			{
            			if(PID_Velocity>(float)900)	{PID_Velocity = (float)900;}
            			m_CurrentPWMDuty = (int32_t)PID_Velocity;
            			mmo_DoorOpeningNchFET(m_CurrentPWMDuty);
        			}
        		}
        		/*
        		 * Closing PWM ���
        		 */
//            	else if(PID_Velocity<(float)0)
    			else if(mmf_PIDControl==PID_CONTROL_CLOSING)
        		{
        			/*
        			 * ó�� Closing���� H-Bridge ����Ī
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
        			
        			if(currEncPulse<0)															// �� ������
        			{
        				mmo_DoorFree();
         				mmo_DoorBrake();														//  ������ ����(Closed), cdshim_20180904
        			}
        			else
        			{
            			if(PID_Velocity<(float)-900)	{PID_Velocity = (float)-900;}
            			m_CurrentPWMDuty = (int32_t)PID_Velocity*-1;
    					/*
    					 * mmo_DoorClosing()�� ȣ��Ǹ� g_CurrentPWM_Ch==PWM_CH3_CLOSING_REVERSE�̾�� �ϳ�
    					 * PWM_CH_NO_SELECTE�� ���¿��� �� ������ Ÿ�� ��찡 ����
    					 * -> 2500��ġ���� Vref�� -2�� ��� ���� PWM����� ������ �ϳ�, ä���� ���� �ȵǾ� PWM����� �ȳ�����,
    					 *    Vref�� -2�� ��� ������ -2000 �̻� ��� ����
    					 *    ������ Opening�� 1�̰�, Closing�� 3�̴ϱ�, �׳� ��������� ä�� ������Ŵ
    					 * 
    					 * -> �̷��� �ߴµ��� �������� �ݺ� ��
    					 *    �ӵ��� ���� Closing�Լ��� ȣ�� �� ��� ȣ����� �ʴ� ��찡 �߻��ϴ� �� ����
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
    volatile static int16_t currEncPulse=0, prevEncPulse=0;												// 1ms irsȣ�� �ø��� ���� ���ڴ� �޽� ������Ʈ
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
		     * 100ms���� V_ref ���� -> 10ms���� ���� ����
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
							 * Opening �ÿ��� �������¿��� DLS�� Ǯ�������� Kick Start
							 */
							if(currEncPulse<300)
							{
								mmf_Encoder.RefVelocity = OpeningAccBeforDLS;
							}
							/*
							 * DLS�� Ǯ�� ���ĺ��ʹ� ���ӱ������� ���������� ���ӽ���
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
						else											mmf_Encoder.RefVelocity =10;			// 2500��ġ�� ������ ������ PWM����� ������ ��
					#else
						if(mmf_Encoder.RefVelocity>7)					mmf_Encoder.RefVelocity -=6;  //jeon_190530 : 10, 10
						else											mmf_Encoder.RefVelocity =7;			// 2500��ġ�� ������ ������ PWM����� ������ ��
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
			 * 10ms���� ���� : ó�� Opening �� ���� ����
			 */
			if(mmf_NextMotorDirSelect==PID_CONTROL_OPENING)
			{
				mmf_NextMotorDirSelect=PID_CONTROL_NONE;
				mmo_DoorFree();
				mmo_DoorOpening(0);
			}
			/*
			 * 10ms���� ���� : Opening PWM Update
			 */
			else
			{
				if(StartOpening)
				{
					/*
					 * Ÿ�ƹ��� ����� �����Ϸ�/�����Ϸ� �Ǵ�
					 * �������1
					 * Closing���� Closed�� �Ǿ� ���ڴ��� �����ص�
					 * ���Ͱ� ���ݸ�(2�޽�) �� ���� ���ڴ� ���� 9998 ���� ����
					 * ���� Closed ���¿��� ���ڴ� ���� 9998�� �ǰ� �̶� ���������� �����ϸ�
					 * �ٷ� �����Ϸ�� �Ǵ��ϹǷ� ���ڴ� ���� 9000���� ���� ������ �߰���
					 *
					 * �������2
					 * ���⼭ �÷��׸� true�� �ϰ�, �Ǵܸ�⿡�� OpenedByPosition�� false�� �ص�
					 * PID�ֱⰡ 1ms�� ����Ǳ� ������ �� �κ��� �ٽ� ����Ǿ� �ٽ� OpenedByPosition�� true�� ����
					 * ���Ŀ� �Ǵܸ�⿡�� �����Ϸᰡ �ǰ�, ���������� �����ص� OpenedByPosition�� ������ true�� �ְ�
					 * ���� �Ϸ� �� �ٽ� ���������� �����ϸ� OpenedByPosition�� true�� ������ �ٷ� �����Ϸᰡ �����
					 * ���� �����Ϸ� ���Ŀ� OpenedByPosition�� false�� Ŭ���� �ؾ� ��. -> �����Ϸᵵ ��������
					 */
					if((mmf_Encoder.Position>m_OpeningEncoderPulse)&&(mmf_Encoder.Position<9000))						// �� ������
					{
						mmo_DoorFree();
						mmf_EndDecision.OpenedByPosition = true;
						//debug("# Opening State: ���ڴ� �����Ϸ� �Ǵ�\r\n");
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
					    
						prevEncPulse = currEncPulse;													// 10ms���� ���� ���ڴ� �޽� ������Ʈ
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
#else						// wjjang_200214 F10 Open 3ȸ �� PWM Duty ����

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
			 * ó�� Closing �� ���� ����
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
					 * �� ������ 0���� �� ������ underflow �߻��ϸ� 9950�� ���� ���� ���� �� ����
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

						prevEncPulse = currEncPulse;													// 10ms���� ���� ���ڴ� �޽� ������Ʈ
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
			 * todo : �ʱ�ȭ ���¿��� PID��� ���� �� �����Ϸ� �Ǵ��� ���� �߰�
			 *        ISR���� ��� �����ϸ� �ȵǹǷ� ������ �����ؾ� ��.
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
			 * Position < 2500 �� ���������� Opening ���� ����
			 */
			if(currEncPulse<m_OpeningEncoderPulse)				mmf_Encoder.RefVelocity = 50;
			else														mmf_Encoder.RefVelocity = 0;
		}
		else if(mmf_PIDControl==PID_CONTROL_CLOSING)
		{
			/*
			 * 0 < Position < 3000�� ���������� closing ���� ����
			 */
			if((currEncPulse>0) && (currEncPulse<3000))	mmf_Encoder.RefVelocity = -50;
			/*
			 * Position < 0 �� ���������� ���ڴ� 0���� Closing�� ���ϸ� underflow �߻��ؼ� 9985���� ���� ���ڱ� ������ 
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
     * PID���� �̼��� �� �ӵ��� ���� �ٷ� PWM��� ����
     */
    #if 0
    {
        volatile static uint32_t Cnt_100ms=0, CurrTime=0, PwmOut=0;
        volatile static uint32_t OpenStartTime=0, CloseStartTime=0;
        volatile static uint32_t WeightingFactor_O=99, WeightingFactor_C=101;											// ����, ��������ӵ� ���� ���������� ����Ͽ� ����� �� ������ ������
    
		CurrTime = osKernelSysTick();
		
		if(g_CurrentPWM_Ch==PWM_CH1_OPENING_FORWARD)
		{
			if(OpenStartTime == 0)																				// Opening �� ó�� �ѹ��� ����
			{
				Cnt_100ms = 0;
				OpenStartTime = CurrTime;
			}
			if(CurrTime<(OpenStartTime+500))		Cnt_100ms++;												// ���� ~ 0.5s	: ����
			else if(CurrTime<(OpenStartTime+2500))	{}															// 0.5s ~ 3		: ���
			else if(CurrTime<(OpenStartTime+3000))	{if(Cnt_100ms>0) Cnt_100ms--;}								// 3s ~ 3.5s	: ����
			else									{Cnt_100ms=0;}												// Set PWM_Out to zero
			
			/*
			 * ���� "PwmOut = Cnt_100ms*93" ����
			 * 
			 * 1. PWM�� Velocity(pulse/s)���� �����
			 *    -> 874pwm ����� ������ 1870pulse/s�� �ӵ��� ����
			 *    -> 874:1870 = pwm:pulse/s
			 *              874
			 *    -> PWM = ------ x pulse/s
			 *              1870
			 *              
			 * 2. 93��ŭ PWM�� ����/���� ���� ��ٸ��� �ӵ� �������� ����
			 *    [pulse/s]               [PWM]
			 *  1000 _|_____________________|_ 467
			 *   800 _|____/|         |\____|_ 373
			 *   600 _|___/ |         | \___|_ 280
			 *   400 _|__/  |         |  \__|_ 186
			 *   200 _|_/   |         |   \_|_  93
			 *	   0 _|/    |         |    \|_   0
			 *	   ---|-----|---------|-----|------ [s]
			 *	      |     |         |     |
			 *        |0.5s | 2s ��� | 0.5s|
			 *        |���� |         | ����|
			 *        
			 * ��ٸ��� �ӵ� �������Ͽ��� ������ �̵��Ÿ� S(pulse)�� �ǰ�,
			 * 3s���� �̵��ؾ� �ϴ� �Ÿ�(pulse)�� TYPE_B�̹Ƿ� 2500[pulse]��ŭ �̵��ؾ� �ϹǷ�
			 * -> S(pulse) = (1/2*V_ref*0.5)*2 + V_ref*2 = 2500
			 *               2.5*V_ref = 2500  
			 *    ����
			 *    V_ref = 1000[pulse/s]�� �ȴ�.
			 * 
			 * ����Ȯ�� ��� �ǵ�� ��� �������� �����Ƿ� 
			 * ����, ��������ӵ� ���� ���������� ����Ͽ� PWM����� �����ؾ� ��
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
			
			if(CurrTime<(CloseStartTime+500))		Cnt_100ms++;												// ���� ~ 0.5s	: ����
			else if(CurrTime<(CloseStartTime+2500))	{}															// 0.5s ~ 3		: ���
			else if(CurrTime<(CloseStartTime+3000))	{if(Cnt_100ms>0) Cnt_100ms--;}								// 3s ~ 3.5s	: ����
			else									{Cnt_100ms=0;}	// Set PWM_Out to zero
			
			/*
			 * ���� PwmOut = Cnt_100ms*94
			 */
			PwmOut = Cnt_100ms*WeightingFactor_C;
			pwm_channel_update_duty(PWM,&g_pchFET_ctla_pwmh[g_CurrentPWM_Ch],PwmOut);            	
			m_CurrentPWMDuty = (int32_t)PwmOut;
		}
		else
		{
			// No PWM Channel Selected -> PWM Update �ϸ� �ȵ�
			m_CurrentPWMDuty = 0;
		}
    }
    /*
     * PID���� ���� �� �ӵ����� ��ǥ��(Vref)���� �����ϰ� PID�Լ����� PWM��� ����
     */
    #elif 0
    {
        volatile static uint32_t CurrTime=0, OpenStartTime=0, CloseStartTime=0;
        volatile static int32_t Cnt_100ms=0;
        
		CurrTime = osKernelSysTick();
		
		if(g_CurrentPWM_Ch==PWM_CH1_OPENING_FORWARD)
		{
			if(OpenStartTime == 0)																				// Opening �� ó�� �ѹ��� ����
			{
				Cnt_100ms = 0;
				OpenStartTime = CurrTime;
			}
			if(CurrTime<(OpenStartTime+500))		Cnt_100ms++;												// 0.0s ~ 0.5s	: ����
			else if(CurrTime<(OpenStartTime+2500))	{}															// 0.5s ~ 2.5s	: ���
			else if(CurrTime<(OpenStartTime+3000))	{if(Cnt_100ms>0) Cnt_100ms--;}								// 2.5s ~ 3.0s	: ����
			else									{Cnt_100ms=0;}
			
			/*
			 * Opening �� ��ٸ��� �ӵ� �������� ���� (3s���� 2500[pulse] �̵�)
			 *    [pulse/s]
			 *  1000 _|     ___________
			 *   800 _|____/|         |\
			 *   600 _|___/ |         | \
			 *   400 _|__/  |         |  \
			 *   200 _|_/   |         |   \
			 *	   0 _|/    |         |    \
			 *	   ---|-----|---------|-----|------ [s]
			 *	      |     |         |     |
			 *        |0.5s | 2s ���  | 0.5s|
			 *        | ���� |         | ���� |
			 *        
			 * ��ٸ��� �ӵ� �������Ͽ��� ������ �̵��Ÿ� S(pulse)�� �ǰ�,
			 * 3s���� �̵��ؾ� �ϴ� �Ÿ�(pulse)�� TYPE_B�̹Ƿ� 2500[pulse]��ŭ �̵��ؾ� �ϹǷ�
			 * -> S(pulse) = (1/2*V_ref*0.5)*2 + V_ref*2 = 2500
			 *               2.5*V_ref = 2500
			 *                   V_ref = 1000
			 *    ����
			 *    V_ref = 1000[pulse/s]�� �ǰ�, 0.5s ���� 100ms���� 200[pulse/s] �� �ӵ��� ��/���� �Ѵ�.
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
			
			if(CurrTime<(CloseStartTime+1000))		Cnt_100ms--;												// 0.0s ~ 1.0s	: ����
			else if(CurrTime<(CloseStartTime+2500))	{}															// 1.0s ~ 4.0s	: ���
			else if(CurrTime<(CloseStartTime+3000))	{if(Cnt_100ms<0) Cnt_100ms++;}								// 4.0s ~ 5.0s	: ����
			else									{Cnt_100ms=0;}
			
			/*
			 * Closing �� ��ٸ��� �ӵ� �������� ���� (5s���� 2500[pulse] �̵�)
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
			 *        | 1s  | 3s ���  |  1s |
			 *        |����  |         | ���� |
			 * 
			 * ��ٸ��� �ӵ� �������Ͽ��� ������ �̵��Ÿ� S(pulse)�� �ǰ�,
			 * 5s���� �̵��ؾ� �ϴ� �Ÿ�(pulse)�� TYPE_B�̹Ƿ� -2500[pulse]��ŭ �̵��ؾ� �ϹǷ�
			 * -> S(pulse) = (1/2*V_ref*1)*2 + V_ref*3 = -2500
			 *                 4*V_ref = -2500
			 *                   V_ref = -625
			 *    ����
			 *    V_ref = 625[pulse/s]�� �ǰ�, 1s ���� 625[pulse/s]�� 100ms���� �ӵ� ��/������ �Ѵ�.
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
     * ���� ����ġ(DCS) ���� ����
     */
    if((mdc_DoorState == DOOR_CLOSED) && (mdc_PreDoorState==DOOR_CLOSING))		// ��� ������ ������
    {
    	if((mip_Input.di1_DCS1==false) || (mip_Input.di1_DCS2==false))			// DCS�� �������� ������ DCS ����
    	{
    		
    	}
    	else																	// DCS�� ���������� ����
    	{
    		
    	}
    }
    else if((mdc_DoorState == DOOR_OPENING) && (mdc_PreDoorState==DOOR_CLOSED))	// ��� ������ ������
    {
    	if((mip_Input.di1_DCS1==true) || (mip_Input.di1_DCS2==true))			// DCS�� ���������� DCS ����
    	{
    		
    	}
    	else																	// DCS�� �������� ������ ����
    	{
    		
    	}
    }
    else
    {
    	// No Check
    }
    
    /*
     * ���� ���� ����
     */
    
    /*
     * ���ڴ� ���� ����
     */
}

/*********************************************************************
	File Path	: E:/6.Korail128_Sidedoor/3.Software/3.Source/korail128_sidedoor/modules/mtMotorFeedback.c
*********************************************************************/
