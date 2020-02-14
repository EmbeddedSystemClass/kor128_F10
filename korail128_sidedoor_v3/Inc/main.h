/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MLA19_Pin GPIO_PIN_3
#define MLA19_GPIO_Port GPIOE
#define MLA20_Pin GPIO_PIN_4
#define MLA20_GPIO_Port GPIOE
#define MLA21_Pin GPIO_PIN_5
#define MLA21_GPIO_Port GPIOE
#define MLA22_Pin GPIO_PIN_6
#define MLA22_GPIO_Port GPIOE
#define MDoor_set_in3_Pin GPIO_PIN_9
#define MDoor_set_in3_GPIO_Port GPIOI
#define MDoor_set_in4_Pin GPIO_PIN_10
#define MDoor_set_in4_GPIO_Port GPIOI
#define MTEST_Pin GPIO_PIN_11
#define MTEST_GPIO_Port GPIOI
#define MLA0_Pin GPIO_PIN_0
#define MLA0_GPIO_Port GPIOF
#define MLA1_Pin GPIO_PIN_1
#define MLA1_GPIO_Port GPIOF
#define MLA2_Pin GPIO_PIN_2
#define MLA2_GPIO_Port GPIOF
#define MLA3_Pin GPIO_PIN_3
#define MLA3_GPIO_Port GPIOF
#define MLA4_Pin GPIO_PIN_4
#define MLA4_GPIO_Port GPIOF
#define MLA5_Pin GPIO_PIN_5
#define MLA5_GPIO_Port GPIOF
#define MSPI_CS1_Pin GPIO_PIN_6
#define MSPI_CS1_GPIO_Port GPIOF
#define MSPI_SCK1_Pin GPIO_PIN_7
#define MSPI_SCK1_GPIO_Port GPIOF
#define MSPI_MISO1_Pin GPIO_PIN_8
#define MSPI_MISO1_GPIO_Port GPIOF
#define MSPI_MOSI1_Pin GPIO_PIN_9
#define MSPI_MOSI1_GPIO_Port GPIOF
#define M3_3V_MONITOR_Pin GPIO_PIN_10
#define M3_3V_MONITOR_GPIO_Port GPIOF
#define SET_MASTER_SLAVE_Pin GPIO_PIN_0
#define SET_MASTER_SLAVE_GPIO_Port GPIOC
#define M_SDCKE0_Pin GPIO_PIN_2
#define M_SDCKE0_GPIO_Port GPIOH
#define M_SDNE0_Pin GPIO_PIN_3
#define M_SDNE0_GPIO_Port GPIOH
#define M_SDNWE_Pin GPIO_PIN_5
#define M_SDNWE_GPIO_Port GPIOH
#define MBEMF_F_ADC_IN_Pin GPIO_PIN_4
#define MBEMF_F_ADC_IN_GPIO_Port GPIOA
#define MADC1_Pin GPIO_PIN_5
#define MADC1_GPIO_Port GPIOA
#define MMVB_MONITOR2_Pin GPIO_PIN_6
#define MMVB_MONITOR2_GPIO_Port GPIOA
#define MMVB_MONITOR1_Pin GPIO_PIN_0
#define MMVB_MONITOR1_GPIO_Port GPIOB
#define MBEMF_R_ADC_IN_Pin GPIO_PIN_1
#define MBEMF_R_ADC_IN_GPIO_Port GPIOB
#define MLA6_Pin GPIO_PIN_12
#define MLA6_GPIO_Port GPIOF
#define MLA7_Pin GPIO_PIN_13
#define MLA7_GPIO_Port GPIOF
#define MLA8_Pin GPIO_PIN_14
#define MLA8_GPIO_Port GPIOF
#define MLA9_Pin GPIO_PIN_15
#define MLA9_GPIO_Port GPIOF
#define MLA10_Pin GPIO_PIN_0
#define MLA10_GPIO_Port GPIOG
#define MLA11_Pin GPIO_PIN_1
#define MLA11_GPIO_Port GPIOG
#define MLD4_Pin GPIO_PIN_7
#define MLD4_GPIO_Port GPIOE
#define MLD5_Pin GPIO_PIN_8
#define MLD5_GPIO_Port GPIOE
#define MLD6_Pin GPIO_PIN_9
#define MLD6_GPIO_Port GPIOE
#define MLD7_Pin GPIO_PIN_10
#define MLD7_GPIO_Port GPIOE
#define MDoor_set_out1_Pin GPIO_PIN_9
#define MDoor_set_out1_GPIO_Port GPIOH
#define MPWM_2_Pin GPIO_PIN_10
#define MPWM_2_GPIO_Port GPIOH
#define MPWM_3_Pin GPIO_PIN_11
#define MPWM_3_GPIO_Port GPIOH
#define MPWM_4_Pin GPIO_PIN_12
#define MPWM_4_GPIO_Port GPIOH
#define MLA16_Pin GPIO_PIN_11
#define MLA16_GPIO_Port GPIOD
#define MLA17_Pin GPIO_PIN_12
#define MLA17_GPIO_Port GPIOD
#define MLA18_Pin GPIO_PIN_13
#define MLA18_GPIO_Port GPIOD
#define MLD0_Pin GPIO_PIN_14
#define MLD0_GPIO_Port GPIOD
#define MLD1_Pin GPIO_PIN_15
#define MLD1_GPIO_Port GPIOD
#define MLA12_Pin GPIO_PIN_2
#define MLA12_GPIO_Port GPIOG
#define MLA13_Pin GPIO_PIN_3
#define MLA13_GPIO_Port GPIOG
#define MLA14_Pin GPIO_PIN_4
#define MLA14_GPIO_Port GPIOG
#define MLA15_Pin GPIO_PIN_5
#define MLA15_GPIO_Port GPIOG
#define S_FND_CTL1_Pin GPIO_PIN_6
#define S_FND_CTL1_GPIO_Port GPIOG
#define S_FND_CTL2_Pin GPIO_PIN_7
#define S_FND_CTL2_GPIO_Port GPIOG
#define MMOT_ENCA_Pin GPIO_PIN_6
#define MMOT_ENCA_GPIO_Port GPIOC
#define MMOT_ENCB_Pin GPIO_PIN_7
#define MMOT_ENCB_GPIO_Port GPIOC
#define S_FND_CTL3_Pin GPIO_PIN_8
#define S_FND_CTL3_GPIO_Port GPIOC
#define S_MRAM_CTL_Pin GPIO_PIN_9
#define S_MRAM_CTL_GPIO_Port GPIOC
#define MRS485_CH1_TX_Pin GPIO_PIN_9
#define MRS485_CH1_TX_GPIO_Port GPIOA
#define MRS485_CH1_RX_Pin GPIO_PIN_10
#define MRS485_CH1_RX_GPIO_Port GPIOA
#define S_RTC_CTL_Pin GPIO_PIN_11
#define S_RTC_CTL_GPIO_Port GPIOA
#define MRS485_CH1_DE_Pin GPIO_PIN_12
#define MRS485_CH1_DE_GPIO_Port GPIOA
#define MASTER_SLAVE_LED_Pin GPIO_PIN_13
#define MASTER_SLAVE_LED_GPIO_Port GPIOH
#define MShutDown_Pin GPIO_PIN_15
#define MShutDown_GPIO_Port GPIOH
#define MPWM_1_Pin GPIO_PIN_0
#define MPWM_1_GPIO_Port GPIOI
#define MDoor_set_out2_Pin GPIO_PIN_1
#define MDoor_set_out2_GPIO_Port GPIOI
#define MSPI_MISO2_Pin GPIO_PIN_2
#define MSPI_MISO2_GPIO_Port GPIOI
#define MSPI_MOSI2_Pin GPIO_PIN_3
#define MSPI_MOSI2_GPIO_Port GPIOI
#define MDBGU_TXD3_Pin GPIO_PIN_10
#define MDBGU_TXD3_GPIO_Port GPIOC
#define MDBGU_RXD3_Pin GPIO_PIN_11
#define MDBGU_RXD3_GPIO_Port GPIOC
#define ALIVE_TXD5_Pin GPIO_PIN_12
#define ALIVE_TXD5_GPIO_Port GPIOC
#define MLD2_Pin GPIO_PIN_0
#define MLD2_GPIO_Port GPIOD
#define MLD3_Pin GPIO_PIN_1
#define MLD3_GPIO_Port GPIOD
#define ALIVE_RXD5_Pin GPIO_PIN_2
#define ALIVE_RXD5_GPIO_Port GPIOD
#define MSPI_SCK2_Pin GPIO_PIN_3
#define MSPI_SCK2_GPIO_Port GPIOD
#define M_NOE_Pin GPIO_PIN_4
#define M_NOE_GPIO_Port GPIOD
#define M_NWE_Pin GPIO_PIN_5
#define M_NWE_GPIO_Port GPIOD
#define M_NWAIT_Pin GPIO_PIN_6
#define M_NWAIT_GPIO_Port GPIOD
#define SMVB_SW_CTL_Pin GPIO_PIN_10
#define SMVB_SW_CTL_GPIO_Port GPIOG
#define MMPWR_CTL_Pin GPIO_PIN_5
#define MMPWR_CTL_GPIO_Port GPIOB
#define SRTC_SCL_Pin GPIO_PIN_6
#define SRTC_SCL_GPIO_Port GPIOB
#define SRTC_SDA_Pin GPIO_PIN_7
#define SRTC_SDA_GPIO_Port GPIOB
#define MSPI_CS2_Pin GPIO_PIN_9
#define MSPI_CS2_GPIO_Port GPIOB
#define S_RelaySwitchingBySlave_Pin GPIO_PIN_5
#define S_RelaySwitchingBySlave_GPIO_Port GPIOI
#define MDoor_set_in1_Pin GPIO_PIN_6
#define MDoor_set_in1_GPIO_Port GPIOI
#define MDoor_set_in2_Pin GPIO_PIN_7
#define MDoor_set_in2_GPIO_Port GPIOI

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
/*
 * MDO_CTL1~6 출력 Read 버퍼 /CS
 */
#define FMC_RDO_EN0 		(*(volatile unsigned char *)0x64000000)

/*
 * 사용 안함
 */
#define FMC_RDO_EN1 		(*(volatile unsigned char *)0x64000001)

/*
 * DI_0(FMC_RDI_EN0) - 입력 1~8 Read 버퍼 /CS
 * ------------------------------------------------------------------------------------------------------
 * | [7]  |    [6]     |     [5]     |      [4]       |   [3]     |     [2]    |   [1]  |      [0]	    |
 * |------|------------|-------------|----------------|-----------|------------|--------|---------------|
 * |1:DLS1|SafetyLoopB |SafetyLoopA  |1:Bypass(DODBPS)|1:차단스위치  |1:ZVR       |1:Off   |1:Close버튼누름	|
 * |0:Off |            |             |0:Off           |0:Off      |0:5km/h over|0:Reopen|0:Open버튼 누름	|
 * ------------------------------------------------------------------------------------------------------
 */
#define FMC_RDI_EN0 		(*(volatile unsigned char *)0x64000002)

/*
 * DI_1(FMC_RDI_EN1) - 입력 9~15 Read 버퍼 /CS
 * -------------------------------------------------------------------------------------------
 * | [7]  |    [6]     |     [5]     |   [4]    |   [3]     |     [2]    |   [1]  |   [0]    |
 * |------|------------|-------------|----------|-----------|------------|--------|----------|
 * |      |            |             |1:DLS2    |1:EAD(외부)	|1:EED(내부)	 |1:DCS1  |1:DCS2    |
 * |      |            |             |0:Off     |0:Off      |0:Off       |0:Off   |0:Off     |
 * -------------------------------------------------------------------------------------------
 */
#define FMC_RDI_EN1 		(*(volatile unsigned char *)0x64000003)

/*
 * 로터리 스위치 입력 /CS
 */
#define FMC_MOD_TL_RD 		(*(volatile unsigned char *)0x64000004)

/*
 * DO_0(FMC_OUTEN0) - MDO_CTL1~6 출력 -> 전원공급, 0111.1001 = 0x79 (모두 전원 공급)
 * -------------------------------------------------------------------------------------------
 * | [7]  |    [6]     |     [5]     |   [4]    |   [3]     |     [2]    |   [1]  |   [0]    |
 * |------|------------|-------------|----------|-----------|------------|--------|----------|
 * |      |차단스위치전원  |EAD-EED Power|DCS1 Power|DCS2 Power	| Disconnect |Reserved|비상등         |
 * -------------------------------------------------------------------------------------------
 */

#define FMC_OUTEN0 			(*(volatile unsigned char *)0x64000020)
/*
 * 사용 안함
 */
#define FMC_OUTEN1 			(*(volatile unsigned char *)0x64000010)

/*
 * FND Sel1 /CS
 */
#define FMC_SEG_DISPLAY1 	(*(volatile unsigned char *)0x64000030)

/*
 * FND Sel2 /CS
 */
#define FMC_SEG_DISPLAY2 	(*(volatile unsigned char *)0x64000008)

/*
 * FND Sel3 /CS
 */
#define FMC_SEG_DISPLAY3 	(*(volatile unsigned char *)0x64000028)

/*
 * SPI Mode Commands for MR10Q010
 */
#define CMD_WREN	0x06	// Write Enable
#define CMD_WRDI	0x04	// Write Disable
#define CMD_RDSR	0x05	// Read Status Register
#define CMD_WRSR	0x01	// Write Status Register
#define CMD_READ	0x03	// Read Data Bytes
#define CMD_WRITE	0x02	// Write Data Bytes
#define CMD_SLEEP	0xB9	// Enter Sleep Mode
#define CMD_WAKE	0xAB	// Exit Sleep Mode

#define MRAM_DATA_ADDR 0x00
#define MRAM_FAULT_ADDR 0x100

// I2C
#define SLAVE_ADDRESS				0xD0		// 0x68 << 1 = 0xD0
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
