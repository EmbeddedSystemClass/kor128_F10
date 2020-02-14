
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "mMotorOut.h"
#include "mtInputProcessing.h"
#include "mtMotorFeedback.h"
#include "mtObstacleDetect.h"
#include "mtDecisionControl.h"
#include "mtMonitoring.h"
#include "mtCommunication.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

NOR_HandleTypeDef hnor1;
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId ioTaskHandle;
osThreadId motorTaskHandle;
osThreadId commTaskHandle;

osThreadId inputTaskHandle;
osThreadId motorfeedbackTaskHandle;
osThreadId obstackedetectTaskHandle;
osThreadId decisioncontrolTaskHandle;
osThreadId monitoringTaskHandle;
osThreadId communicationTaskHandle;
osThreadId mvbTaskHandle;
osThreadId alivecheckTaskHandle;
osThreadId displayfndTaskHandle;
osThreadId rs485TaskHandle;
volatile uint8_t UartReady = 0;
uint8_t aUartRx1Buffer[11];
uint8_t aUartRx3Buffer[1];
uint8_t aUartRx5Buffer[8];
/* The queue used to hold received characters. */
osMessageQId hRx1Queue;
osMessageQId hRx3Queue;
osMessageQId hRx5Queue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_FMC_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI5_Init(void);
static void MX_SPI5_Init2(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
void StartDefaultTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
ssize_t _write(int fd __attribute__((unused)), const char *buf __attribute__((unused)), size_t nbyte __attribute__((unused)))
{
	HAL_UART_Transmit(&huart3, (uint8_t *)(buf), 1, 100);
	return 1;
}
int _read(int file, char *data, int len)
{
	char rxdata=0;
    int bytes_read;
    
    for(/* empty */; len > 0; --len)
    {
//    	HAL_UART_Receive(&huart3, (uint8_t *)(data), 1, 0xFFFF);
    	HAL_UART_Receive(&huart3, (uint8_t *)(data), 1, 500);
    	rxdata = (uint8_t *)(data);
    	
    	HAL_UART_Transmit(&huart3,&rxdata,1,10);
    	
    	if(*data == 0x0D)
    	{
    		HAL_UART_Transmit(&huart3,(uint8_t *)"\n",1,10);
    	}
		else if(*data == 0x00)	//null
		{
			break;
		}
    	data++;
    }
    return bytes_read;
}
void StartIOTask(void const * argument);
void StartMotorTask(void const * argument);
void StartCOMMTask(void const * argument);
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
extern void mmf_TaskMotorFeedback(void const * argument);
extern void mod_TaskObstacleDetect(void const * argument);
extern void mip_TaskInputProcessing(void const * argument);
extern void mdc_TaskDecisionControl(void const * argument);
extern void mm_TaskMonitoring(void const * argument);
extern void mc_TaskCLICommand(void const * argument);
extern void mc_TaskMVB(void const * argument);
extern void mac_TaskAliveCheck(void const * argument);
uint8_t Get_DCUID(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void SupportExternalDevicePower(void)
{
	uint8_t ExtDevice_PowerSupply_Feedback=0;
	
//	#ifndef DEBUG_SIMULATOR
	{
		/*
		 * 외부장치 전원 공급 체크
		 */
//		ExtDevice_PowerSupply_Feedback = FMC_RDO_EN0;
		ExtDevice_PowerSupply_Feedback = *(unsigned char*)(0x64000000);

		printf("## 전원피드백 : %02X\r\n", ExtDevice_PowerSupply_Feedback);
		
		if(!(ExtDevice_PowerSupply_Feedback&0x04))
			printf("## Disconnect 전원공급 Fail\r\n");
		else if(!(ExtDevice_PowerSupply_Feedback&0x08))
			printf("## DCS2 전원공급 Fail\r\n");
		else if(!(ExtDevice_PowerSupply_Feedback&0x10))
			printf("## DCS1 전원공급 Fail\r\n");
		else if(!(ExtDevice_PowerSupply_Feedback&0x20))
			printf("## EAD-EED 전원공급 Fail\r\n");
		else if(!(ExtDevice_PowerSupply_Feedback&0x40))
			printf("## 차단스위치 전원공급 Fail\r\n");
		else
		{
			/*
			 * 차단스위치, EAD-EED, DCS1, DCS2 Power On, 비상등 Power Off
			 */
			printf("## 외부장치 전원공급 OK\r\n");
			return;
		}
		
		/*
		 * 외부장치 전원을 정상적으로 공급하지 못하면 외부 입력값 오인식으로 오동작 발생할 수 있음
		 */
//		while(1)
//		{
//			FMC_SEG_DISPLAY1 = m_FND[FND_E];
//			FMC_SEG_DISPLAY2 = m_FND[FND_r];
//			FMC_SEG_DISPLAY3 = m_FND[FND_r];
//			osDelay(1000);
//			FMC_SEG_DISPLAY1 = 0x0;
//			FMC_SEG_DISPLAY2 = 0x0;
//			FMC_SEG_DISPLAY3 = 0x0;
//			osDelay(1000);
//		}
	}
//	#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	#if 1
	float a=0, b=0, c=0, d=0;
	uint8_t i, DeviceID=0xff, prevDeviceID=0xff, idMatchCnt=0, idUnMatchCnt=0;
	RTCTIME init_rtctime;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_FMC_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	/*
	* 디버그 모드에서 타이머 인터럽트 중지
	*/
	__HAL_DBGMCU_FREEZE_TIM1();
	__HAL_DBGMCU_FREEZE_TIM3();
	__HAL_DBGMCU_FREEZE_TIM5();
	__HAL_DBGMCU_FREEZE_TIM6();

	/*=======================*/
	/*
	* =	System Description	=
	* 
	* 	1. TIM1
	* 		- FreeRTOS Tick 인터럽트
	* 		- APB2(200MHz) 클럭 사용
	* 	2. TIM3
	* 		- 엔코더 Read
	* 		- APB1(100MHz) 클럭 사용
	* 	3. TIM5
	* 		- PWM출력
	* 		    : TIM_OCPOLARITY_LOW 설정   -> Active Low(PWM 30%를 주면 Low가 30%, High가 70% 출력)
	* 		    : TIM_OCPOLARITY_HIGH 설정 -> Active High(PWM 30%를 주면 High가 30%, Low가 70% 출력)
	* 		- APB1(100MHz) 클럭 사용
	* 		- 16bit Prescaler
	* 			: 100MHz(100,000,000)	0.01us
	* 			: 5분주 -> 0.01us x 5 = 1카운트 0.05us (20kHz) 	// 10kHz로 모터 돌리면 스위칭 노이즈 발생
	* 		- 1000카운트 -> 50us (count_period)
	* 		- pwm_duty = pulse/count_period x 100
	* 	4. TIM6
	* 		- 100ms Overflow Interrupt
	* 		- APB1(100MHz) 클럭 사용
	* 		- 16bit Prescaler
	* 			: 100MHz(100,000,000)	0.01us
	* 			: 10,000분주	-> 0.01us x 10,000 = 1카운트 100us(0.1ms)
	* 		- 10,000카운트	-->    1s마다 인터럽트 발생
	* 		- 1,000카운트	--> 100ms마다 인터럽트 발생
	* 		- 100카운트		-->  10ms마다 인터럽트 발생	<-- 이걸로 설정
	* 		
	* 	5. UART1
	* 		- DCU간 통신을 위한 RS485 인터페이스
	* 		- 38400bps, Asynchronous, High Polarity
	* 		- Hardware Flow Control(RTS - DE) Enable
	* 	6. UART3
	* 		- 디버그 콘솔(RS232)
	* 	7. UART5
	* 		- DCU내부 Master/Slave간 alive 체크를 위한 RS232 인터페이스
	* 		
	* 	8. SPI2
	* 		- SPI Master MRAM 인터페이스
	* 		- APB1 클럭소스, 25MHz Bit/s로 설정
	* 	9. SPI5
	* 		- TCMS와 통신을 위한 SPI Master MVBc 인터페이스
	* 		- Hardware /CS
	* 		- APB2 클럭소스, 50MHz Bit/s로 설정
	* 		
	* 	10. ADC
	* 		- Rank1 - ADC1_IN4_PA4: M3_3V_MONITOR
	* 		- Rank2 - ADC1_IN5_PA5: MADC1(모터전류)
	* 		- Rank3 - ADC1_IN9_PB1: MBEMF_R_ADC_IN
	* 		- Rank1 - ADC3_IN8_PF10: MBEMF_F_ADC_IN
	* 		
	* System IO Description
	* 	1. GPIO Input
	* 		PI11(MTEST_Pin) : 절체 스위치(Reset 버튼 옆에 있음)
	* 		
	* 	2. GPIO Output
	* 		PB5(MMPWR_CTL_Pin) : 디폴트 Low출력 -> N채널 FET Off -> H-Bridge GND 연결 끊어짐
	* 		PH13(MASTER_SLAVE_LED_Pin) : RESET(LED On), SET(LED Off)
	* 		PH15(MShutDown_Pin) : 디폴트 High출력 설정함 -> AND Gate false입력 -> PWM4개가 H-Bridge로 출력 안됨
	* 		PI5(S_RelaySwitchingBySlave_Pin) : RESET(절체), SET(절체미수행)
	*/
	/*=======================*/
	a = 1.25f;
	b = 2.81f;
	c = a+b;				// int형 변수일 경우 덧셈 결과는 1+2=3
	d = a*b;				// int형 변수일 경우 곱샘 결과는 1*2=2
	printf("-- 코레일128량(Korail128) based on RTOS!(%.2f, %.2f) --\r\n", c, d);
	printf("-- Compiled: %s %s --\r\n", __DATE__, __TIME__);
	
//	HAL_Delay(5000);					//jeon_191007 5000ms 지연

	
	/*
	 * 외부 디바이스 전원 공급 (한번만 출력하면 잘 안되는 것 같음..)
	 */
	FMC_OUTEN0 = 0xfe;				// 출력이 한번에 잘 안되는 것 같아서 
	HAL_Delay(100);					// 지연하고
	FMC_OUTEN0 = 0xfe;				// 3번 출력 다시 설정
	HAL_Delay(100);
	FMC_OUTEN0 = 0xfe;
	HAL_Delay(100);
//	SupportExternalDevicePower();	// 제대로 안읽힘
	
	/*
	 * Master/Slave 동작을 위한 절체 수행
	 */
	if(HAL_GPIO_ReadPin(SET_MASTER_SLAVE_GPIO_Port, SET_MASTER_SLAVE_Pin) == 1)
	{
		printf("-- Master DCU\r\n");
		m_isMasterSlave = MASTER_DCU;
		HAL_GPIO_WritePin(GPIOH, MASTER_SLAVE_LED_Pin, GPIO_PIN_RESET);								// Master LED On
		mip_SleveCodeSlaveRun(0);

		HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);

	}
	else
	{
		printf("-- Slave DCU\r\n");
		m_isMasterSlave = SLAVE_DCU;
		HAL_GPIO_WritePin(MASTER_SLAVE_LED_GPIO_Port, MASTER_SLAVE_LED_Pin, GPIO_PIN_SET);			// Slave LED Off
		HAL_GPIO_WritePin(S_FND_CTL1_GPIO_Port, S_FND_CTL1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_FND_CTL2_GPIO_Port, S_FND_CTL2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_FND_CTL3_GPIO_Port, S_FND_CTL3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_MRAM_CTL_GPIO_Port, S_MRAM_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_RTC_CTL_GPIO_Port, S_RTC_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SMVB_SW_CTL_GPIO_Port, SMVB_SW_CTL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(S_RelaySwitchingBySlave_GPIO_Port, S_RelaySwitchingBySlave_Pin, GPIO_PIN_SET);	// Master로 연결

		HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);

	}
	#if 1
	DeviceID = Get_DCUID();
	if(DeviceID != 0xff)	g_unDeviceID = DeviceID;
	else					m_CardefineID = Define_NONE;
	#else
	m_CardefineID = Define_MCAR;
	g_unDeviceID = DCU_L1_MCAR;
	//g_unDeviceID = DCU_L4_TCAR;
	#endif
//	m_OpeningEncoderPulse = TYPE_B;						// 전체 도어 이동 거리
//	m_OpeningEncoderPulse = 3600;
//	m_OpeningEncoderPulse = 3530;
	m_OpeningEncoderPulse = 3580;

#if 1
{
	/*
	 * 명령어 기준 MVB Configuration
	 * 1. SDR(Status Data Request)
	 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
	 * 		DCU_R1_MCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
	 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
	 * 		DCU_R1_TCAR Sink    <--(Fcode 2, Port 0xD0)--  TCMS Source
	 * 
	 * 2. SD(Status Data)
	 * 		DCU_L1_MCAR Source  --(Fcode 4, Port 0xD4)-->  TCMS Sink
	 * 		DCU_R1_MCAR Source  --(Fcode 4, Port 0xD8)-->  TCMS Sink
	 * 		DCU_L1_TCAR Source  --(Fcode 4, Port 0xE4)-->  TCMS Sink
	 * 		DCU_R1_TCAR Source  --(Fcode 4, Port 0xE8)-->  TCMS Sink
	 * 
	 * 3. TDR(Trace Data Request)
	 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xF8)--  TCMS Source
	 * 		DCU_R1_MCAR Sink    <--(Fcode 2, Port 0xF8)--  TCMS Source
	 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xFC)--  TCMS Source
	 * 		DCU_R1_TCAR Sink    <--(Fcode 2, Port 0xFC)--  TCMS Source
	 * 
	 * 4. TD(Trace Data)
	 * 		DCU_L1_MCAR Source  --(Fcode 4, Port 0xDC)-->  TCMS Sink
	 * 		DCU_R1_MCAR Source  --(Fcode 4, Port 0xE0)-->  TCMS Sink
	 * 		DCU_L1_TCAR Source  --(Fcode 4, Port 0xEC)-->  TCMS Sink
	 * 		DCU_R1_TCAR Source  --(Fcode 4, Port 0xF0)-->  TCMS Sink
	 */
	/*
	 * DCU 기준 MVB Configuration
	 * 1. DCU1_MVB
	 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
	 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xD4, SD )-->  TCMS Sink
	 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xF8, TDR)--  TCMS Source
	 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xDC, TD )-->  TCMS Sink
	 * 					
	 * 2. DCU2_MVB
	 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
	 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xD8, SD )-->  TCMS Sink
	 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xF8, TDR)--  TCMS Source
	 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xE0, TD )-->  TCMS Sink
	 * 
	 * 3. DCU3_MVB
	 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
	 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xE4, SD )-->  TCMS Sink
	 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xFC, TDR)--  TCMS Source
	 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xEC, TD )-->  TCMS Sink
	 * 		
	 * 4. DCU4_MVB
	 * 		DCU_L1_MCAR Sink    <--(Fcode 2, Port 0xD0, SDR)--  TCMS Source
	 * 		DCU_R1_MCAR Source   --(Fcode 4, Port 0xE8, SD )-->  TCMS Sink
	 * 		DCU_L1_TCAR Sink    <--(Fcode 2, Port 0xFC, TDR)--  TCMS Source
	 * 		DCU_R1_TCAR Source   --(Fcode 4, Port 0xF0, TD )-->  TCMS Sink
	 */
	switch(g_unDeviceID)
	{
		case DCU_R1_MCAR:
			mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
			mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD8);							// DCU R1 M --> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD8
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
			mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE0);							// DCU R1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xE0
			break;
		case DCU_L4_MCAR:
			mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
			mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xD4);							// DCU L1 M	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xD4
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xF8);								// DCU M    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xF8
			mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xDC);							// DCU L1 M	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xDC
			break;
		case DCU_R1_TCAR:
			mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
    		mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE8);							// DCU R1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE8
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
			mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xF0);							// DCU R1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xF0
			break;
		case DCU_L4_TCAR:
			mvbSinkConfiguration(MVB_FCODE3_16BYTE, 0x2C);								// DCU ALL  <-- MVB <-- (COMMON) <-- TCMS : Fcode 3, Port Number 0x2C
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xD0);								// DCU ALL  <-- MVB <--  (SDR)   <-- TCMS : Fcode 2, Port Number 0xD0
			mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xE4);							// DCU L1 T	--> MVB -->  (SD)    --> TCMS : Fcode 4, Port Number 0xE4
			mvbSinkConfiguration(MVB_FCODE2_8BYTE, 0xFC);								// DCU T    <-- MVB <--  (TDR)   <-- TCMS : Fcode 2, Port Number 0xFC
			mvbSourceConfiguration(MVB_FCODE4_32BYTE, 0xEC);							// DCU L1 T	--> MVB -->  (TD)    --> TCMS : Fcode 4, Port Number 0xEC
			break;
		default: debug("No MVB Source Configuration\r\n"); break;
	}
}
#endif

	
	if(HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK){Error_Handler();}						// Encoder 기능 시작
	if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)					{Error_Handler();}						// Timer Interrupt 기능 시작
	
	/* Create the queues used to hold Rx/Tx characters. */
	osMessageQDef(hRx1Queue, 500, uint8_t);																// QueueSize(1byte), QueueType(uint8_t)
	hRx1Queue = osMessageCreate(osMessageQ(hRx1Queue), NULL);
	if((hRx1Queue != 0))
	{
		if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11) != HAL_OK)	{Error_Handler();}	// 11byte를 수신하면 인터럽트 발생
	}
	else	{Error_Handler();}
	  
//	osMessageQDef(hRx3Queue, 1, uint8_t);																// QueueSize(1byte), QueueType(uint8_t)
//	hRx3Queue = osMessageCreate(osMessageQ(hRx3Queue), NULL);
//	if((hRx3Queue != 0))
//	{
//		if(HAL_UART_Receive_IT(&huart3, (uint8_t *)aUartRx3Buffer, 1) != HAL_OK)	{Error_Handler();}
//	}
//	else	{Error_Handler();}
	
	osMessageQDef(hRx5Queue, 100, uint8_t);																// QueueSize(100byte), QueueType(uint8_t)
	hRx5Queue = osMessageCreate(osMessageQ(hRx5Queue), NULL);
	if((hRx5Queue != 0))
	{
		if(HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8) != HAL_OK)	{Error_Handler();}	// 8byte를 수신하면 인터럽트 발생
	}
	else	{Error_Handler();}
	
	/*
	* PWM출력 100%일 때 PWM주기(100us)마다 100ns 만큼 low로 떨어졌다 올라옴
	*/
	/*
	 * todo: 타이머 인터럽트 Nested 안되게 설정할 것 (확인할 것)
	 * 중첩되게 하면 스택 사용량 등의 문제가 될 수 있기 때문에....
	 */
	if(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)		{Error_Handler();}						// PWM2
	if(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)		{Error_Handler();}						// PWM3
	if(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK)		{Error_Handler();}						// PWM4
	if(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK)		{Error_Handler();}						// PWM1
	#endif

	if(m_isMasterSlave == MASTER_DCU)
	{
		/*
		 * 부팅할때 슬레이브는 제어권이 없으므로 ff가나옴 절대 설정값을 정하면 안되고
		 * 제어권이 넘어왔을때 설정 값들을 지정하도록 변경
		 */
		mram_write_enable();
		printf("## Enable writes to MRAM\r\n");
		printf("고장 개수 : %d \r\n",mram_byte_read(MRAM_FAULT_ADDR));
		ConfigDcuParameters();
		init_rtctime = rtc_get_time();

		printf("RTC DATA : %d 년 %d월 %d일 %d시 %d분 %d 초 \r\n",init_rtctime.ucYears,
				init_rtctime.ucMonth,
				init_rtctime.ucDate,
				init_rtctime.ucHours,
				init_rtctime.ucMinutes,
				init_rtctime.ucSeconds);
	}
	else
	{
		mram_write_diable();
	}
	HAL_Delay(300);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /*
   *  todo <<jjkim_190108>> : (1) - dodbps 감지 후 힘이 약함
   *  						  (2) - f07 발생 시 동작 확인
   *  						  (3) - 열림 닫힘 시간 조절 확인
   *						  (4) - 장애물 감지 3회 확인 ( 시간 조절 해야함)
   *						  (5) - ead eed 동작 확인
   *						  (6) - 고장 발생 시, 빨간색 등 뜨지 않음
   *						  (7) - dls 1/2 고장 발생 시, 도어 에러로 있지 않음
   *						  (8) - dls dcs 다 눌린채로 전원을 켰을 때 열림동작하면 무조건 initflag = false ?? 그 동작 실시함 정상적으로 닫고 난 후에 정상 동작함
   *						  (9) - 닫힘 동작시 엔코더 거리로 플래그 셋 하는거 확인 완료
   *						  (10) - 열림 동작 중 가속구간에서 닫힘 명령시 쾅 닫힘
   *						  (11) - reopen 동작 구현 안되있는 듯
   *						  (12) - 전류로 열림완료 불가
   */
	/* add threads, ... */
	#if 0	// Basic Test
	{
		osThreadDef(ioTask, StartIOTask, osPriorityNormal, 0, 128);
		ioTaskHandle = osThreadCreate(osThread(ioTask), NULL);
		
		//osThreadDef(motorTask, StartMotorTask, osPriorityNormal, 0, 128);
		//motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);
		
		//osThreadDef(commTask, StartCOMMTask, osPriorityNormal, 0, 128);
		//commTaskHandle = osThreadCreate(osThread(commTask), NULL);
	}
	#else	// korail128 operation
	{
		/*
		 * 전원 안정화를 위한 딜레이
		 * - 전원 불안정 시 노이즈 유입으로 인한 오동작 요인 제거를 위한 지연
		 * - vTaskDelay(300)으로 실행하면 아예 실행이 안되네...
		 *   -> 실행할 태스크가 없는데 Context Switching이 발생해서 그런가???
		 */
		
		/*
		 * Master-Slave Alive Check task - Task를 이용하지 말고, Application Timer를 이용해서 사용해야 함??
		 *
		 * Task는 우선순위에 따라 실행되는 개념이고,
		 * Timer는 주기적으로 실행되는 개념이므로,
		 * TimerInterrupt보다는 정확하지는 않으나, 러프하게 주기적으로 Master-Slave 간 통신을 수행해야 함!
		 */
		/*
		 * todo: alive check 우선순위를 Normal에서 Low로 설정해서 문제 없으면 Low로 수정할 것
		 * Decision이랑 우선순위가 같으면 안됨.
		 */
		osThreadDef(alivecheckTask, mac_TaskAliveCheck, osPriorityAboveNormal, 0, 512);							// 6, hRx5Queue
		alivecheckTaskHandle = osThreadCreate(osThread(alivecheckTask), NULL);
		/*
		 * Class Diagram Object
		 */
		/*
		 * Priority based Preemptive scheduling (우선순위가 높은 태스크가 항상 프로세서를 점유)
		 * - 확인결과 InputProcessing_Task와 Monitoring_Task의 우선순위를 같게 할 경우
		 *   Monitoring 결과를 printf 하는 도중 입력값을 printf하는 경우가 발생할 수 있음
		 * - 실행주기가 빠른 태스크의 우선순위를 높게 책정
		 */

		/*
		 * MotorFeedback task
		 */
		osThreadDef(motorfeedbackTask, mmf_TaskMotorFeedback, osPriorityRealtime, 0, 128);					// 1
		motorfeedbackTaskHandle = osThreadCreate(osThread(motorfeedbackTask), NULL);
		
		/*
		 * ObstacleDetection task
		 */
		osThreadDef(obstackedetectTask, mod_TaskObstacleDetect, osPriorityRealtime, 0, 128);					// 2 - 20ms
		obstackedetectTaskHandle = osThreadCreate(osThread(obstackedetectTask), NULL);
		
		/*
		 * Input task
		 */
		osThreadDef(inputTask, mip_TaskInputProcessing, osPriorityHigh, 0, 128);						// 3 - 25ms
		inputTaskHandle = osThreadCreate(osThread(inputTask), NULL);
		
		/*
		 * DecisionControl task
		 */
		osThreadDef(decisioncontrolTask, mdc_TaskDecisionControl, osPriorityHigh, 0, 1024);				// 4 - 100ms
		decisioncontrolTaskHandle = osThreadCreate(osThread(decisioncontrolTask), NULL);

		/*
		 * Monitoring task
		 */
		osThreadDef(monitoringTask, mm_TaskMonitoring, osPriorityAboveNormal, 0, 256);						// 5
		monitoringTaskHandle = osThreadCreate(osThread(monitoringTask), NULL);
		
		/*
		 * MVB task
		 */
		osThreadDef(mvbTask, mc_TaskMVB, osPriorityAboveNormal, 0, 512);											// 6
		mvbTaskHandle = osThreadCreate(osThread(mvbTask), NULL);
		
		/*
		 *  RS485 TASK
		 */
		osThreadDef(rs485Task, mac_TaskRS485, osPriorityAboveNormal, 0, 1024);										// 6, hRx1Queue
		rs485TaskHandle = osThreadCreate(osThread(rs485Task), NULL);

		
		/*
		 *  CLI TASK
		 */
		osThreadDef(communicationTask, mc_TaskCLICommand, osPriorityBelowNormal, 0, 256);							// 6, hRx3Queue
		communicationTaskHandle = osThreadCreate(osThread(communicationTask), NULL);
		
		/*
		 * FND Display task
		 */
		osThreadDef(displayfndTask, md_TaskDisplayFND, osPriorityBelowNormal, 0, 256);								// 6
		displayfndTaskHandle = osThreadCreate(osThread(displayfndTask), NULL);
	}
	#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	FMC_SEG_DISPLAY1 = m_FND[FND_F];
	FMC_SEG_DISPLAY2 = m_FND[FND_F];
	FMC_SEG_DISPLAY3 = m_FND[FND_F];
	while (1)
	{
		printf("## FreeRTOS Start Fail\r\n");
		osDelay(1000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C0EAFF;
  hi2c1.Init.OwnAddress1 = 208;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;//SPI_POLARITY_HIGH
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;//SPI_BAUDRATEPRESCALER_2
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW; //SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE; //SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; //SPI_BAUDRATEPRESCALER_64;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;//SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)  //motor pwm control
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9; //jeon_190716 orig:5-16Khz, 올리면 느려지나?, 9:10khz, 95: 1khz인듯?, 9를 넘어가면 고주파음이 들림.
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000; //jeon_190716 orig:1000, 올리면 느려지나?
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the NOR2 memory initialization sequence
  */
  hnor1.Instance = FMC_NORSRAM_DEVICE;
  hnor1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hnor2.Init */
  hnor1.Init.NSBank = FMC_NORSRAM_BANK2;
  hnor1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hnor1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;//FMC_MEMORY_TYPE_NOR;
  hnor1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hnor1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hnor1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hnor1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hnor1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hnor1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hnor1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hnor1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hnor1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hnor1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hnor1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hnor1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;//15;
  Timing.AddressHoldTime = 15;//15;
  Timing.DataSetupTime = 255;//255;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 15;//16;
  Timing.DataLatency = 15;//15;
  Timing.AccessMode = FMC_ACCESS_MODE_A;//FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_NOR_Init(&hnor1, &Timing, NULL) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, MDoor_set_out1_Pin|MASTER_SLAVE_LED_Pin|MShutDown_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, S_FND_CTL1_Pin|S_FND_CTL2_Pin|SMVB_SW_CTL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S_FND_CTL3_Pin|S_MRAM_CTL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S_RTC_CTL_GPIO_Port, S_RTC_CTL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, MDoor_set_out2_Pin|S_RelaySwitchingBySlave_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MMPWR_CTL_GPIO_Port, MMPWR_CTL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MMVB_MONITOR2_GPIO_Port, MMVB_MONITOR2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MMVB_MONITOR1_GPIO_Port, MMVB_MONITOR1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MSPI_CS2_GPIO_Port, MSPI_CS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MDoor_set_in3_Pin MDoor_set_in4_Pin MDoor_set_in1_Pin MDoor_set_in2_Pin */
  GPIO_InitStruct.Pin = MDoor_set_in3_Pin|MDoor_set_in4_Pin|MDoor_set_in1_Pin|MDoor_set_in2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : MTEST_Pin */
  GPIO_InitStruct.Pin = MTEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MTEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MSPI_CS1_Pin */
  GPIO_InitStruct.Pin = MSPI_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MSPI_CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SET_MASTER_SLAVE_Pin */
  GPIO_InitStruct.Pin = SET_MASTER_SLAVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SET_MASTER_SLAVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MMVB_MONITOR2_Pin */
  GPIO_InitStruct.Pin = MMVB_MONITOR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MMVB_MONITOR2_GPIO_Port, &GPIO_InitStruct);
  
   /*Configure GPIO pin : MMVB_MONITOR1_Pin */
  GPIO_InitStruct.Pin = MMVB_MONITOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MMVB_MONITOR1_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : MDoor_set_out1_Pin MASTER_SLAVE_LED_Pin MShutDown_Pin */
  GPIO_InitStruct.Pin = MDoor_set_out1_Pin|MASTER_SLAVE_LED_Pin|MShutDown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : S_FND_CTL1_Pin S_FND_CTL2_Pin SMVB_SW_CTL_Pin */
  GPIO_InitStruct.Pin = S_FND_CTL1_Pin|S_FND_CTL2_Pin|SMVB_SW_CTL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : S_FND_CTL3_Pin S_MRAM_CTL_Pin */
  GPIO_InitStruct.Pin = S_FND_CTL3_Pin|S_MRAM_CTL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : S_RTC_CTL_Pin */
  GPIO_InitStruct.Pin = S_RTC_CTL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S_RTC_CTL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MDoor_set_out2_Pin S_RelaySwitchingBySlave_Pin */
  GPIO_InitStruct.Pin = MDoor_set_out2_Pin|S_RelaySwitchingBySlave_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : MMPWR_CTL_Pin */
  GPIO_InitStruct.Pin = MMPWR_CTL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MMPWR_CTL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MSPI_CS2_Pin */
  GPIO_InitStruct.Pin = MSPI_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MSPI_CS2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 * ISR
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	if(GPIO_Pin == MMVB_MONITOR1_Pin)
//	{
//	}
//	else if(GPIO_Pin == MMVB_MONITOR2_Pin)
//	{
//	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle ->Instance == USART1)													// DCU끼리 RS485
	{
		osMessagePut(hRx1Queue, 0x88, 0);
		HAL_UART_Receive_IT(&huart1, (uint8_t *)aUartRx1Buffer, 11);
	}
//	else if(UartHandle->Instance == USART3)												// 디버그 콘솔(RS232)
//		osMessagePut(hRx3Queue, 0x88, 0);
	else if(UartHandle->Instance == UART5)// DCU내부 Master/Slave간 alive
	{
		osMessagePut(hRx5Queue, 0x88, 0);
		HAL_UART_Receive_IT(&huart5, (uint8_t *)aUartRx5Buffer, 8);
	}
}
/*
 * 에러 상황 발생시 호출 되는 함수
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(huart);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_ErrorCallback can be implemented in the user file
   */
}
uint8_t Get_DCUID(void)
{
	uint8_t return_value=0;
	
	dcuID.CodingPin = (((HAL_GPIO_ReadPin(MDoor_set_in4_GPIO_Port, MDoor_set_in4_Pin))<<3) +
					  ((HAL_GPIO_ReadPin(MDoor_set_in3_GPIO_Port, MDoor_set_in3_Pin))<<2) + 
					  ((HAL_GPIO_ReadPin(MDoor_set_in2_GPIO_Port, MDoor_set_in2_Pin))<<1) +
					  ((HAL_GPIO_ReadPin(MDoor_set_in1_GPIO_Port, MDoor_set_in1_Pin))<<0) ) & 0x0f;

	/* a2 b1 커넥터가 쇼트되어있으면 1, 그렇지않은경우 0을 출력 */
	dcuID.McarTcarPin =!getAbit(FMC_RDI_EN1,6);
	printf("car id : %d \r\n",dcuID.McarTcarPin);
	//printf("Rotery Switch value : %02x \r\n",dcuID.RotterySwtich);
	if((dcuID.CodingPin != 0x00) && (dcuID.CodingPin != 0x0f))
	{
		/* 커넥터 a2 b1이 연결되어 있지 않으면 m카*/
		if(dcuID.McarTcarPin == 0x00)
		{
			if(dcuID.CodingPin == 0x01)      {return_value = DCU_R1_MCAR; printf("# DCU_R1_MCAR\r\n");}		// R1
			else if(dcuID.CodingPin == 0x06) {return_value = DCU_R2_MCAR; printf("# DCU_R2_MCAR\r\n");}		// R2
			else if(dcuID.CodingPin == 0x0A) {return_value = DCU_R3_MCAR; printf("# DCU_R3_MCAR\r\n");}		// R3
			else if(dcuID.CodingPin == 0x02) {return_value = DCU_R4_MCAR; printf("# DCU_R4_MCAR\r\n");}		// R4
			else if(dcuID.CodingPin == 0x04) {return_value = DCU_L1_MCAR; printf("# DCU_L1_MCAR\r\n");}		// L1
			else if(dcuID.CodingPin == 0x03) {return_value = DCU_L2_MCAR; printf("# DCU_L2_MCAR\r\n");}		// L2
			else if(dcuID.CodingPin == 0x05) {return_value = DCU_L3_MCAR; printf("# DCU_L3_MCAR\r\n");}		// L3
			else if(dcuID.CodingPin == 0x09) {return_value = DCU_L4_MCAR; printf("# DCU_L4_MCAR\r\n");}		// L4
			else							 {return_value = 0xff;}
			m_CardefineID = Define_MCAR;
		}
		/* 커넥터 a2 b1이 연결되어 있으면 t카*/
		else if(dcuID.McarTcarPin == 0x01)
		{
			if(dcuID.CodingPin == 0x01)      {return_value = DCU_R1_TCAR; printf("# DCU_R1_TCAR\r\n");}		// R1
			else if(dcuID.CodingPin == 0x06) {return_value = DCU_R2_TCAR; printf("# DCU_R2_TCAR\r\n");}		// R2
			else if(dcuID.CodingPin == 0x0A) {return_value = DCU_R3_TCAR; printf("# DCU_R3_TCAR\r\n");}		// R3
			else if(dcuID.CodingPin == 0x02) {return_value = DCU_R4_TCAR; printf("# DCU_R4_TCAR\r\n");}		// R4
			else if(dcuID.CodingPin == 0x04) {return_value = DCU_L1_TCAR; printf("# DCU_L1_TCAR\r\n");}		// L1
			else if(dcuID.CodingPin == 0x03) {return_value = DCU_L2_TCAR; printf("# DCU_L2_TCAR\r\n");}		// L2
			else if(dcuID.CodingPin == 0x05) {return_value = DCU_L3_TCAR; printf("# DCU_L3_TCAR\r\n");}		// L3
			else if(dcuID.CodingPin == 0x09) {return_value = DCU_L4_TCAR; printf("# DCU_L4_TCAR\r\n");}		// L4
			else							 {return_value = 0xff;}
			m_CardefineID = Define_TCAR;
		}
		else
		{
			if(dcuID.CodingPin == 0x01)      {return_value = DCU_R1_TCAR; printf("# DCU_R1\r\n");}			// R1
			else if(dcuID.CodingPin == 0x06) {return_value = DCU_R2_TCAR; printf("# DCU_R2\r\n");}			// R2
			else if(dcuID.CodingPin == 0x0A) {return_value = DCU_R3_TCAR; printf("# DCU_R3\r\n");}			// R3
			else if(dcuID.CodingPin == 0x02) {return_value = DCU_R4_TCAR; printf("# DCU_R4\r\n");}			// R4
			else if(dcuID.CodingPin == 0x04) {return_value = DCU_L1_TCAR; printf("# DCU_L1\r\n");}			// L1
			else if(dcuID.CodingPin == 0x03) {return_value = DCU_L2_TCAR; printf("# DCU_L2\r\n");}			// L2
			else if(dcuID.CodingPin == 0x05) {return_value = DCU_L3_TCAR; printf("# DCU_L3\r\n");}			// L3
			else if(dcuID.CodingPin == 0x09) {return_value = DCU_L4_TCAR; printf("# DCU_L4\r\n");}			// L4
			else							 {printf("# DCU_ID None\r\n");}
			return_value =0xff;
			m_CardefineID = Define_NONE;
		}
	}
	else
	{
		return_value = 0xff;
		m_CardefineID = Define_NONE;
	}
	#if 0
	else
	{
		dcuID.RotterySwtich = *(unsigned char*)(0x64000004);
		dcuID.RotterySwtich = *(unsigned char*)(0x64000004);
		dcuID.RotterySwtich = *(unsigned char*)(0x64000004);
		printf("DCU ID:");
		if((dcuID.RotterySwtich & 0x0f) == 0x01)      {return_value = DCU_L1_MCAR; printf("DCU_L1_MCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x02) {return_value = DCU_L2_MCAR; printf("DCU_L2_MCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x03) {return_value = DCU_L3_MCAR; printf("DCU_L3_MCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x04) {return_value = DCU_L4_MCAR; printf("DCU_L4_MCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x05) {return_value = DCU_R4_MCAR; printf("DCU_R4_MCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x06) {return_value = DCU_R3_MCAR; printf("DCU_R3_MCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x07) {return_value = DCU_R2_MCAR; printf("DCU_R2_MCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x08) {return_value = DCU_R1_MCAR; printf("DCU_R1_MCAR, ");}
		
		else if((dcuID.RotterySwtich & 0x0f) == 0x11) {return_value = DCU_L1_TCAR; printf("DCU_L1_TCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x12) {return_value = DCU_L2_TCAR; printf("DCU_L2_TCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x13) {return_value = DCU_L3_TCAR; printf("DCU_L3_TCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x14) {return_value = DCU_L4_TCAR; printf("DCU_L4_TCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x15) {return_value = DCU_R4_TCAR; printf("DCU_R4_TCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x16) {return_value = DCU_R3_TCAR; printf("DCU_R3_TCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x17) {return_value = DCU_R2_TCAR; printf("DCU_R2_TCAR, ");}
		else if((dcuID.RotterySwtich & 0x0f) == 0x18) {return_value = DCU_R1_TCAR; printf("DCU_R1_TCAR, ");}
		else return_value = 0xff;
		printf("코딩핀 신호 인식 불가, 로터리 스위치 값(%02x)\r\n", return_value);
	}
	#endif
	return return_value;
}

/*
 * Normal Function
 */
void StartIOTask(void const * argument)
{
	uint32_t adcresult[4] = {0,}, loopcnt=0;
	uint32_t EncoderCounter[3]={0,}, ADCCnt=0, GPIOOutCnt=0, GPIOInCnt=0, ENCCnt=0;
	uint8_t Direction = 0;
	uint8_t fmc_switch_value=0, fmc_rdo0_value=0, fmc_rdi0_value=0, fmc_rdi1_value=0;
	uint8_t prev_fmc_switch_value=0, i=0;

	printf("## Temp Task\r\n");
	
	for(;;)
	{
		/*
		 * GPIO
		 */
		#if 0
		{
			GPIOOutCnt++;
			if(GPIOOutCnt>1000)
			{
				GPIOOutCnt=0;
				HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_13);
				i ^= 0xff;
				
				FMC_SEG_DISPLAY1 = i;
				FMC_SEG_DISPLAY2 = i;
				FMC_SEG_DISPLAY3 = i;
			}
			GPIOInCnt++;
			if(GPIOInCnt>100)
			{
				GPIOInCnt=0;
				fmc_switch_value = FMC_MOD_TL_RD;
				fmc_rdo0_value = FMC_RDO_EN0;
				fmc_rdi0_value = FMC_RDI_EN0;
				fmc_rdi1_value = FMC_RDI_EN1;
				#if 0
					if(prev_fmc_switch_value != fmc_switch_value)							// 가변저항값이 변할 때만 입력 출력
					{
						printf("## SW:(0x%x), RDO0:(0x%x), RDI0:(0x%x), RDI1:(0x%x)\r\n",
								fmc_switch_value,fmc_rdo0_value,fmc_rdi0_value,fmc_rdi1_value);
					}
					prev_fmc_switch_value = fmc_switch_value;
				#else
					printf("## SW:(0x%x), RDO0:(0x%x), RDI0:(0x%x), RDI1:(0x%x)\r\n",
							fmc_switch_value,fmc_rdo0_value,fmc_rdi0_value,fmc_rdi1_value);
				#endif
			}
			osDelay(1);
		}
		#endif
		
		/*
		 * ADC
		 */
		#if 1
		{
			loopcnt++;
			if(loopcnt>20)
			{
				loopcnt=0;
				#if 0
					if(HAL_ADC_Start(&hadc1) != HAL_OK) {Error_Handler();}
						/*
						 * PA4, ADC1_IN4(ADC_CHANNEL_4) - M3_3V_MONITOR
						 *  - 3.3V : 4095 = 2.6V : y
						 *  - y = (4095*2.6)/3.3 = about 3226
						 */
						if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)
							adcresult[0] = HAL_ADC_GetValue(&hadc1);
						/*
						 * PA5, ADC1_IN5(ADC_CHANNEL_5) - M_MOTOR_CURRENT
						 */
						if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)
							adcresult[1] = HAL_ADC_GetValue(&hadc1);
						/*
						 * PB1, ADC1_IN9(ADC_CHANNEL_9) - MBEMF_R_ADC_IN
						 */
						if(HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)
							adcresult[2] = HAL_ADC_GetValue(&hadc1);
					HAL_ADC_Stop(&hadc1);
					
					/*
					 * PF10, ADC3_IN8(ADC_CHANNEL_8) - MBEMF_F_ADC_IN
					 */
					if(HAL_ADC_Start(&hadc3) != HAL_OK) {Error_Handler();}
						if(HAL_ADC_PollForConversion(&hadc3, 100)==HAL_OK)
							adcresult[3] = HAL_ADC_GetValue(&hadc3);
					HAL_ADC_Stop(&hadc3);
				#else
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
					 * PowerVoltage = MotorVoltage  + V_bemf
					 * 
					 * 	M3_3V_MONITOR - PA4, ADC1_IN4
					 *  - 3.3V : 4095 = 2.6V : y
					 *  - y = (4095*2.6)/3.3 = about 3226
					 */
					if(g_CurrentPWM_Ch == PWM_CH1_OPENING_FORWARD)
					{
						HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN4(ADC_CHANNEL_4)
						HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorCurrent = HAL_ADC_GetValue(&hadc1);	// ADC12_IN5(ADC_CHANNEL_5)
						HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.PowerVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN9(ADC_CHANNEL_9)
						
						HAL_ADC_Start(&hadc3);
						if(HAL_ADC_PollForConversion(&hadc3, 1000)==HAL_OK)	m_adc.MCUVoltage = HAL_ADC_GetValue(&hadc3);	// ADC3_IN8(ADC_CHANNEL_8)
						
						printf("# %d\t%d\t%d\t%d\r\n",m_adc.MotorVoltage,m_adc.MotorCurrent,m_adc.PowerVoltage,m_adc.MCUVoltage);
					}
					else if(g_CurrentPWM_Ch == PWM_CH3_CLOSING_REVERSE)
					{
						HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.PowerVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN4(ADC_CHANNEL_4)
						HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorCurrent = HAL_ADC_GetValue(&hadc1);	// ADC12_IN5(ADC_CHANNEL_5)
						HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1, 1000)==HAL_OK)	m_adc.MotorVoltage = HAL_ADC_GetValue(&hadc1);	// ADC12_IN9(ADC_CHANNEL_9)
						
						HAL_ADC_Start(&hadc3);
						if(HAL_ADC_PollForConversion(&hadc3, 1000)==HAL_OK)	m_adc.MCUVoltage = HAL_ADC_GetValue(&hadc3);	// ADC3_IN8(ADC_CHANNEL_8)
						
						printf("# %d\t%d\t%d\t%d\r\n",m_adc.PowerVoltage,m_adc.MotorCurrent,m_adc.MotorVoltage,m_adc.MCUVoltage);
					}
					printf("## Temp Task\r\n");

				#endif
			}
			osDelay(10);
		}
		#endif
		
		/*
		 * Encoder Pulse Count
		 */
		#if 0
			#if 0											// 모터가 움직일 때만 펄스 출력 1씩 증가/감소
			{
				EncoderCounter[2]=EncoderCounter[1];
				EncoderCounter[1]=EncoderCounter[0];
				EncoderCounter[0]=TIM3->CNT;
				Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
				
				if(EncoderCounter[2]==EncoderCounter[1] &&
				   EncoderCounter[1]!=EncoderCounter[0]    )
				{
					if(Direction==0)
						printf("# Direction=CW, Count=%d\r\n", (uint16_t)EncoderCounter[0]);
					else
						printf("# Direction=CCW, Count=%d\r\n", (uint16_t)EncoderCounter[0]);
				}
			}
			#else											// 1초 간격으로 펄스 출력 (1초동안 움직인 거리(펄스)를 출력) -> 하드웨어적으로 발생한 펄스가 저장 됨
			{
				ENCCnt++;
				if(ENCCnt>1000)
				{
					ENCCnt=0;
					EncoderCounter[2]=EncoderCounter[1];
					EncoderCounter[1]=EncoderCounter[0];
					EncoderCounter[0]=TIM3->CNT;
					Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	
					printf("# Direction=%d, Count=%d\r\n", Direction, (uint16_t)EncoderCounter[0]);
				}
			}
			#endif
			osDelay(1);
		#endif
	}
}

void StartMotorTask(void const * argument)
{
	uint8_t motor_state=0;
	uint32_t printcnt=0, loopcnt1=0, loopcnt2=0;
	
	printf("## Start Motor Task\r\n");
	osDelay(100);
	mmo_ConnectHbridgeGND(true);
	
	for(;;)
	{
		/*
		 * 모터 구동 테스트
		 */
		if(mip_Input.di0_OpenCloseButton)
		{
			if(motor_state==0)
			{
				mmo_DoorOpening(300);
				loopcnt2++;
				if(loopcnt2>3000)
				{
					loopcnt2=0;
					motor_state = 1;
				}
			}
			else if(motor_state==1)
			{
				mmo_DoorFree();
				loopcnt2++;
				if(loopcnt2>1000)
				{
					loopcnt2=0;
					motor_state = 2;
				}
			}
			else if(motor_state==2)
			{
				mmo_DoorClosing(300);
				loopcnt2++;
				if(loopcnt2>3000)
				{
					loopcnt2=0;
					motor_state = 3;
				}
			}
			else if(motor_state==3)
			{
				mmo_DoorFree();
				__HAL_TIM_SET_COUNTER(&htim3, 0);
				loopcnt2++;
				if(loopcnt2>1000)
				{
					loopcnt2=0;
					motor_state = 0;
				}
			}
			loopcnt1++;
			if(loopcnt1>1000)
			{
				loopcnt1=0;
				printf("## State %d\r\n", motor_state);
			}
		}
		else
		{
			mmo_DoorFree();
		}
		
		osDelay(1);
	}
}

static uint32_t BCDtoUINT32(uint8_t ucBCD)
{
	uint32_t unData = 0;

	unData+=((ucBCD>>4)*10);
	unData+=(ucBCD&0x0F);

	return unData;
}

static uint32_t calculate_week(uint32_t ul_year, uint32_t ul_month, uint32_t ul_day)
{
	uint32_t ul_week;

	if(ul_month == 1 || ul_month == 2)
	{
		ul_month += 12;
		--ul_year;
	}
	ul_week = (ul_day + 2 * ul_month + 3 * (ul_month + 1) / 5 + ul_year + ul_year / 4 - ul_year / 100 + ul_year / 400) % 7;
	++ul_week;
	return ul_week;
}

void rtc_set_time(uint8_t Year,uint8_t Month,uint8_t Date,uint8_t Hour,uint8_t Minute,uint8_t Second)
{
	uint32_t tmpYear=0, tmpMonth=0, tmpDate=0;
	uint8_t aTxByte=0;
	uint32_t rtcRegister=0;
	uint32_t i2c_error_code=0;
	
	for(rtcRegister=0; rtcRegister<7; rtcRegister++)
	{
		switch(rtcRegister)
		{
			case 0:
				aTxByte = (((Second/10)<<4) & 0x70) + ((Second%10) & 0x0f);
				break;
			case 1:
				aTxByte = (((Minute/10)<<4) & 0x70) + ((Minute%10) & 0x0f);
				break;
			case 2:
				aTxByte = (((Hour/10)<<4) & 0x30) + ((Hour%10) & 0x0f);
				break;
			case 3:
				tmpYear += BCDtoUINT32(Year);
				tmpMonth += BCDtoUINT32(Month);
				tmpDate += BCDtoUINT32(Date);
				aTxByte = calculate_week(tmpYear,tmpMonth,tmpDate);			// 년/월/일 계산해서 요일 산출
				break;
			case 4:
				aTxByte = (((Date/10)<<4) & 0x30) + ((Date%10) & 0x0f);
				break;
			case 5:
				aTxByte = (((Month/10)<<4) & 0x10) + ((Month%10) & 0x0f);
				break;
			case 6:
				aTxByte = (((Year/10)<<4) & 0xf0) + ((Year%10) & 0x0f);
				break;
		}
		if(HAL_I2C_Mem_Write(&hi2c1, (uint16_t)SLAVE_ADDRESS, (uint16_t)rtcRegister, I2C_MEMADD_SIZE_8BIT, &aTxByte, 1, 5000) != HAL_OK)
		{

			i2c_error_code = HAL_I2C_GetError(&hi2c1);
			printf("## I2C MRAM Write Error: %d\r\n", i2c_error_code);
			i2c_error_code = 0;

		}
	}
}

RTCTIME rtc_get_time(void)
{
	RTCTIME time;
	uint8_t aRxByte=0, tmp_data;
	uint32_t rtcRegister=0;
	uint32_t i2c_state_code=0, i2c_error_code=0;

	for(rtcRegister=0; rtcRegister<7; rtcRegister++)
	{
		if(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)SLAVE_ADDRESS, (uint16_t)rtcRegister, I2C_MEMADD_SIZE_8BIT, &aRxByte, 1, 5000) != HAL_OK)
		{
			i2c_error_code = HAL_I2C_GetError(&hi2c1);
			printf("## I2C MRAM Read Error: %d\r\n", i2c_error_code);
			i2c_error_code = 0;
		}
		else
		{
			switch(rtcRegister)
			{
				case 0:
					tmp_data = (aRxByte & 0x7F);
					time.ucSeconds = ((tmp_data>>4)*10 + (tmp_data & 0x0f));
					break;
				case 1:
					tmp_data = (aRxByte & 0x7F);
					time.ucMinutes = ((tmp_data>>4)*10 + (tmp_data & 0x0f));
					break;
				case 2:
					tmp_data = (aRxByte & 0x3F);
					time.ucHours = ((tmp_data>>4)*10 + (tmp_data & 0x0f));
					break;
				case 3:
					time.ucDay = (aRxByte & 0x07);
					break;
				case 4:
					tmp_data = (aRxByte & 0x3F);
					time.ucDate = ((tmp_data>>4)*10 + (tmp_data & 0x0f));
					break;
				case 5:
					tmp_data = (aRxByte & 0x1F);
					time.ucMonth = ((tmp_data>>4)*10 + (tmp_data & 0x0f));
					break;
				case 6:
					tmp_data = (aRxByte & 0xfF);
					time.ucYears = ((tmp_data>>4)*10 + (tmp_data & 0x0f));
					break;
				default:
					break;
			}
			while(i2c_state_code != HAL_I2C_STATE_READY)			// 0x20
			{
				i2c_state_code = HAL_I2C_GetState(&hi2c1);
			}
			i2c_state_code = 0;
		}
	}
	return time;
}

void StartCOMMTask(void const * argument)
{
	#if 0	// console
	{
		osEvent event;
		
		printf("UART3 console test\r\n");
		for(;;)
		{
			/*
			 * uart3(console debug) 테스트
			 */
			event = osMessageGet(hRx3Queue, 0);				// usart3 수신 인터럽트 발생 시 큐에 저장한 1byte를 가져 옴
			if(event.status == osEventMessage)
			{
				#if 0
					if(HAL_UART_Transmit(&huart3, (uint8_t *)&event.value.p, 1, 2000)!= HAL_OK)
						printf("UART echo back Error\r\n");
					else
						printf("UART echo back OK\r\n");
				#else
					printf("(%c)",(uint8_t)event.value.p);
					/*
					 * compiler dependent
					 * fflush를 해야 fflush를 호출해야 실제 물리적으로 데이터를 write 수행
					 */
					fflush(stdout);
				#endif
			}
			else
			{
				HAL_UART_Receive_IT(&huart3, (uint8_t *)aUartRx3Buffer, 1);			// 수신인터럽트를 다시 활성화 시켜야 인터럽트로 다시 데이터 수신 가능
			}
			osDelay(1);
		}
	}
	#elif 0	// rs485
	{
		uint32_t loopcnt1=0;
		uint8_t aUartTx_Buffer[] = "UART_TEST_";
		printf("## Start RS485 Task~\r\n");
		
		for(;;)
		{
			loopcnt1++;
			if(loopcnt1>1000)
			{
				loopcnt1=0;
				/*
				 *uart1(RS485) - TCMS 통신
				 */
				if(HAL_UART_Transmit(&huart1, (uint8_t*)aUartTx_Buffer, 10, 2000)!= HAL_OK)
					printf("## RS485 Transmit Error(%d)\r\n");
				else
					printf("## RS485 Transmit OK\r\n");
			}
			osDelay(1);
		}
	}
	#elif 1	// rtc
	{
		uint32_t loopcnt1=0;
		RTCTIME rtctime;
		rtc_set_time(18,10,2,18,05,30);
		
		for(;;)
		{
			loopcnt1++;
			if(loopcnt1>1000)
			{
				loopcnt1=0;
				/*
				 * I2C Read, 0x68 = 1101000
				 * 
				 * Address,RD,ACK,Data
				 * 1101000,1 ,0  ,
				 */
				rtctime = rtc_get_time();
				printf("## RTC %d/%d/%02d, %d, %d:%d:%d\r\n",
					rtctime.ucYears, rtctime.ucMonth, rtctime.ucDate, rtctime.ucDay, 
					rtctime.ucHours, rtctime.ucMinutes, rtctime.ucSeconds);
			}
			osDelay(1);
		}
	}
	#elif 0	// MVB
	{
		uint32_t loopcnt2=0;
		HAL_StatusTypeDef retTXval=5, retRXval=5;
		uint8_t aRxBuffer[32]={0};
		uint8_t aTxBuffer[35]={0};												// 3byte 패킷 헤더, 최대 32byte data length
		uint8_t num=0;
		uint8_t Fcode=MVB_FCODE4_32BYTE, PortNumber=0, ByteDataLength=0;			// 현차 Fcode 4
		
		/*
		 * Fcode에 따른 데이터 Size 결정
		 */
		switch(Fcode)
		{
			case MVB_FCODE0_2BYTE: ByteDataLength = 2; break;
			case MVB_FCODE1_4BYTE: ByteDataLength = 4; break;
			case MVB_FCODE2_8BYTE: ByteDataLength = 8; break;
			case MVB_FCODE3_16BYTE: ByteDataLength = 16; break;
			case MVB_FCODE4_32BYTE: ByteDataLength = 32; break;
			default: break;
		}
		
		for(;;)
		{
			/*
			 * SPI5 - MVB 통신
			 * 
			 * 1. DU설정
			 *    - Master Frame 설정 (사용되는 Fcode/PortAddr 전부 설정)
			 *    - 소스포트 설정 (Fcode[2]/PortAddr[0xD0])
			 *    - 싱크포트 설정 (안함)
			 *    - 소스포트 선택항 자동쓰기 -> DU의 소스(Fcode[2]/PortAddr[0xD0])에서 버스에 8byte 데이터를 띄움
			 *    
			 * 2. DCU설정
			 *    - 싱크포트 설정 (Fcode[2]/PortAddr[0xD0])
			 */
			#if 1
			{
				loopcnt2++;
				/*
				 * 1ms마다 MVB통신은 이뤄지나, 설정된 사양에 따라 data가 update 됨 (사양서에서는 전부 T4=512ms)
				 */
				if(loopcnt2>511)
				{
					loopcnt2=0;
					
					#if 0
					{
						/*
						 * 1. MVB Source 설정 후 Write : TCMS <- MVB(Source)
						 */
						/*
						 * 1.1 MVB Source Configuration : F-Code(4), Port Number(0x20) <- 실제 TCMS와 통신 시 설정
						 */
						#if 1
						{
							PortNumber = 0x20;														// TCMS(Master) <-- DCU(Slave)
							
							aTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;			// 메모리 명령어, Write
							aTxBuffer[1] = 0x20;													// LA PCS Offset Address (0x2000~0x27FF)
							aTxBuffer[2] = PortNumber;
							aTxBuffer[3] = Fcode|MVB_SLAVE_SOURCE;									// F_Code(상위4bit), Source
							aTxBuffer[4] = 0x00;
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
								retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)aTxBuffer, 5, 5000);	// 1패킷 끊어서 보내고
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
							
							switch(retTXval)
							{
								case HAL_OK:
									printf("SPI Master TX1 OK\r\n");
									break;
								case HAL_TIMEOUT:
									printf("SPI Master TX1 Timeout\r\n");
									break;
								case HAL_ERROR:
									printf("SPI Master TX1 Error\r\n");
									break;
								default:
									break;
							}
							osDelay(100);
						}
						#endif
						
						/*
						 * 1.2 MVB Source Write : 메모리 영역에 Write
						 */
						#if 1
						{
							aTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;			// 메모리 명령어, Write
							aTxBuffer[1] = 0x00;													// LA Data Offset (0x0000~0x1FFFF)
							aTxBuffer[2] = PortNumber<<2;
							for(uint8_t i=0; i<ByteDataLength; i++)
								aTxBuffer[i+3] = num++;
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
								retTXval = HAL_SPI_Transmit(&hspi5, &aTxBuffer, ByteDataLength+3, 5000);			// 다시 데이터를 전송
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
							switch(retTXval)
							{
								case HAL_OK:
									printf("SPI Master TX2 OK\r\n");
									break;
								case HAL_TIMEOUT:
									printf("SPI Master TX2 Timeout\r\n");
									break;
								case HAL_ERROR:
									printf("SPI Master TX2 Error\r\n");
									break;
								default:
									break;
							}
						}
						#endif
					}
					#else
					{
						/*
						 * 2. MVB Sink 설정 후 Read : TCMS -> MVB(Sink)
						 */
						/*
						 * 2.1 MVB Sink Configuration : F-Code(4), Port Number(0x20) <- 실제 TCMS와 통신 시 설정
						 */
						#if 1
						{
							PortNumber = 0x10;														// TCMS(Master) --> DCU(Slave)
							
							aTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_WRITE;			// 메모리 명령어, Write
							aTxBuffer[1] = 0x20;													// LA PCS Offset Address (0x2000~0x27FF)
							aTxBuffer[2] = PortNumber;												// PortNumber: 2 (MVB 테스트 할때 쓰는 포트번호)
							aTxBuffer[3] = Fcode|MVB_SLAVE_SINK;									// F_Code(상위4bit), Sink
							aTxBuffer[4] = 0x00;
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
								retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)aTxBuffer, 5, 5000);
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
							
							switch(retTXval)
							{
								case HAL_OK:
									printf("SPI Master TX1(for sink configuration) OK\r\n");
									break;
								case HAL_TIMEOUT:
									printf("SPI Master TX1-1 Timeout\r\n");
									break;
								case HAL_ERROR:
									printf("SPI Master TX1-1 Error\r\n");
									break;
								default:
									break;
							}
							osDelay(100);
						}
						#endif
						
						/*
						 * 2.2 Sink data read
						 */
						#if 1
						{
							/*
							 ** 5 byte 송신 후
							 */
							aTxBuffer[0] = SPI_INSTRUCTION_MEMORY|SPI_INSTRUCTION_READ;				// 메모리 명령어, Read
							aTxBuffer[1] = 0x00;													// LA Data Offset (0x0000~0x1FFFF)
							aTxBuffer[2] = PortNumber<<2;
							aTxBuffer[3] = 0x00;													// 더미데이터는 무조껀 2byte를 전송해야 함
							aTxBuffer[4] = 0x00;													// 더미 데이터를 전송해야 클럭이 MVB로 공급되어 이후 동작이 수행됨
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_RESET);
							retTXval = HAL_SPI_Transmit(&hspi5, (uint8_t*)aTxBuffer, 5, 5000);
							
							switch(retTXval)
							{
								case HAL_OK:
									printf("SPI Master TX1-2 OK\r\n");
									break;
								case HAL_TIMEOUT:
									printf("SPI Master TX1-2 Timeout\r\n");
									break;
								case HAL_ERROR:
									printf("SPI Master TX1-2 Error\r\n");
									break;
								default:
									break;
							}
							osDelay(100);
							
							/*
							 ** Fcode에 해당하는 data length만큼 데이터 수신
							 */
							retRXval = HAL_SPI_Receive(&hspi5, (uint8_t*)aRxBuffer, ByteDataLength, 1000);
							HAL_GPIO_WritePin(MSPI_CS1_GPIO_Port, MSPI_CS1_Pin, GPIO_PIN_SET);
							switch(retRXval)
							{
								case HAL_OK:
									printf("SPI Slave RX OK: ");
									for(num=0; num<32; num++)
									{
										printf("%02X ", aRxBuffer[num]);
									}
									printf("\r\n");
									break;
								case HAL_TIMEOUT:
									printf("SPI Slave RX Timeout\r\n");
									break;
								case HAL_ERROR:
									printf("SPI Slave RX Error\r\n");
									break;
								default:
									break;
							}
						}
						#endif
					}
					#endif
				}
			}
			#endif
			osDelay(1);
		}
	}
	#elif 1	// MRAM
	{
		uint32_t loopcnt1=0;
		uint8_t retval=0;
		
		/*
		 * SPI2 - MRAM Control
		 */
		retval = mram_byte_read(0x1FFFF);
		if(retval==0xAA)	printf("## Init MRAM Read(0xAA) OK\r\n");
		else				printf("## Init MRAM Read(0xAA) Fail\r\n");
		
		for(;;)
		{
			loopcnt1++;
			if(loopcnt1>1000)
			{
				loopcnt1=0;
				/*
				 * MRAM 0x1FFFF번지 Write
				 */
				mram_byte_write(0x1FFFF, 0xAA);										// MRAM 0x1FFFF번지에 0xAA Write
				/*
				 * MRAM 0x1FFFF번지 Read
				 */
				printf("## Read data: %02X\r\n", mram_byte_read(0x1FFFF));
			}
			osDelay(1);
		}
	}
	#else
	{
		uint32_t loopcnt1=0;
		printf("## No Action\r\n");
		for(;;)
		{
			loopcnt1++;
			if(loopcnt1>1000)
			{
				loopcnt1=0;
			}
			osDelay(1);
		}
	}
	#endif
}
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	uint32_t PreviousWakeTime = osKernelSysTick();
	uint8_t DeviceID=0xff, loopcnt=0;
					
	/* Infinite loop */
	for(;;)
	{
		osDelayUntil(&PreviousWakeTime, 200UL);
		
		/*
		 * 5s마다 ID체크
		 */
		loopcnt++;
		if(loopcnt>25)
		{
			printf(" mvb flag %d \r\n",m_isMasterSlaveChange);
			loopcnt=0;
			#ifndef DEBUG_SIMULATOR
				DeviceID = Get_DCUID();
				if(DeviceID != 0xff)
				{
					g_unDeviceID = DeviceID;
					//printf("DCU ID default : %d \r\n",g_unDeviceID);
				}
				else
				{
					//printf("dcu id fail : %d \r\n",DeviceID);
					m_CardefineID = Define_NONE;
				}

			#else
				DeviceID = DCU_L1_MCAR; //printf("직접설정, DCU0 ID: DCU L1 MCAR\r\n");
//				DeviceID = DCU_L2_MCAR; printf("직접설정, DCU1 ID: DCU L2 MCAR\r\n");
//				DeviceID = DCU_L3_MCAR; printf("직접설정, DCU2 ID: DCU L3 MCAR\r\n");
//				DeviceID = DCU_L4_MCAR; printf("직접설정, DCU3 ID: DCU L4 MCAR\r\n");

//				DeviceID = DCU_R4_MCAR; printf("직접설정, DCU4 ID: DCU R4 MCAR\r\n");
//				DeviceID = DCU_R3_MCAR; printf("직접설정, DCU5 ID: DCU R3 MCAR\r\n");
//				DeviceID = DCU_R2_MCAR; printf("직접설정, DCU6 ID: DCU R2 MCAR\r\n");
//				DeviceID = DCU_R1_MCAR; printf("직접설정, DCU7 ID	: DCU R1 MCAR\r\n");

//				DeviceID = DCU_L1_TCAR; printf("직접설정, DCU8 ID: DCU L1 TCAR\r\n");
//				DeviceID = DCU_R1_TCAR; printf("직접설정, DCUF ID: DCU R1 TCAR\r\n");
			#endif
		}
		
		/*
		 * todo : IDLE Task에서 이 동작을 수행하는게 맞는지 다시한번 생각해 볼 것
		 */
		//HAL_IWDG_Refresh(&hiwdg);
		
		/*
		 * UART1(RS485 Master/Slave통신) 에러 플래그 클리어
		 */
		if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_ORE))
		{
			printf("overrun 485\r\n");
			__HAL_UART_CLEAR_OREFLAG(&huart1);							// 오버런 플래그를 클리어하고
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);				// 인터럽트를 재 활성화 한다. 데이터는 깨지게 되지만 어차피 유지보수프로그램에서 재요청함
			//__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

		}
		else if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_NE))
		{
			printf("noise error 485 \r\n");
			__HAL_UART_CLEAR_NEFLAG(&huart1);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		}
		else if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_FE))
		{
			printf("frame error 485 \r\n");
			__HAL_UART_CLEAR_FEFLAG(&huart1);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		}
		/*
		* UART3(RS232 디버그 콘솔) 에러 플래그 클리어
		*/
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_ORE))
		{
			printf("overrun cli\r\n");
			__HAL_UART_CLEAR_OREFLAG(&huart3);							// 오버런 플래그를 클리어하고
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);				// 인터럽트를 재 활성화 한다. 데이터는 깨지게 되지만 어차피 유지보수프로그램에서 재요청함
			//__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
		}
		else if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_NE))
		{
			printf("noise error cli\r\n");
			__HAL_UART_CLEAR_NEFLAG(&huart3);
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
		}
		else if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_FE))
		{
			printf("frame error cli\r\n");
			__HAL_UART_CLEAR_FEFLAG(&huart3);
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
		}
		/*
		* UART5(RS232 Master/Slave alive Check) 에러 플래그 클리어
		*/
		if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_ORE))
		{
			printf("overrun m-s\r\n");
			__HAL_UART_CLEAR_OREFLAG(&huart5);							// 오버런 플래그를 클리어하고
			__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);				// 인터럽트를 재 활성화 한다. 데이터는 깨지게 되지만 어차피 유지보수프로그램에서 재요청함
			//__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
		}
		else if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_NE))
		{
			printf("noise error m-s\r\n");
			__HAL_UART_CLEAR_NEFLAG(&huart5);
			__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
		}
		else if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_FE))
		{
			printf("frame error m-s\r\n");
			__HAL_UART_CLEAR_FEFLAG(&huart5);
			__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
		}
	}
	/* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
//  volatile static uint32_t TmrCnt=0; 

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) {
	  /*
	   * 10ms마다 인터럽트 발생
	   */
//	  if(TmrCnt>100)	{TmrCnt=0;printf("# TmrCnt6\r\n");}
//	  else				{TmrCnt++;}

	  /*
	   * 이동 중 엔코더 고장 발생 시 우선 전류로 멈추고, Closed 상태가 된 다음부터 Bemf Mode로 제어를 수행해야 함.
	   */
//	  if(!error_list_flag[DCU_ENCODER_ERROR])	mmf_isrEncoder_VelocityControl();
//	  else /* DCU_ENCODER_ERROR */				mmf_isrBemf_PositionCalculate();	// F03
	  if(!error_list_flag[DCU_ENCODER_ERROR]) mmf_isrEncoder_VelocityControl();
#if 0 //jeoN_190710	  orig:1
	  else if((m_isTestEncoderflag == true) && (error_list_flag[DCU_ENCODER_ERROR] == true)) mmf_isrEncoder_VelocityControl();
#endif	  
	  else  mmf_isrBemf_PositionCalculate(); //jeon_190715 mmf_isrBemf_PositionCalculate();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	/*
	 * todo : 에러 발생 시 파일 및 line 위치정보 MRAM에 저장
	 */
  printf("# Error at %s %d --\r\n\r\n", file, line);
  while(1)
  {
	  HAL_GPIO_TogglePin(MASTER_SLAVE_LED_GPIO_Port, MASTER_SLAVE_LED_Pin);
	  HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
