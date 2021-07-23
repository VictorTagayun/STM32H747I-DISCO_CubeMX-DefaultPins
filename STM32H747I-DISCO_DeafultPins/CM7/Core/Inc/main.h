/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32h747i_discovery_qspi.h"
#include "stm32h747i_discovery_sd.h"
#include "stm32h747i_discovery_lcd.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FMC_D28_Pin GPIO_PIN_6
#define FMC_D28_GPIO_Port GPIOI
#define FMC_NBL3_Pin GPIO_PIN_5
#define FMC_NBL3_GPIO_Port GPIOI
#define FMC_NBL2_Pin GPIO_PIN_4
#define FMC_NBL2_GPIO_Port GPIOI
#define JOY_RIGHT_Pin GPIO_PIN_5
#define JOY_RIGHT_GPIO_Port GPIOK
#define QSPI_BK2_IO2_Pin GPIO_PIN_9
#define QSPI_BK2_IO2_GPIO_Port GPIOG
#define SDIO1_D2_Pin GPIO_PIN_10
#define SDIO1_D2_GPIO_Port GPIOC
#define FMC_D25_Pin GPIO_PIN_1
#define FMC_D25_GPIO_Port GPIOI
#define FMC_D24_Pin GPIO_PIN_0
#define FMC_D24_GPIO_Port GPIOI
#define FMC_D29_Pin GPIO_PIN_7
#define FMC_D29_GPIO_Port GPIOI
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define HDMI_CEC_Pin GPIO_PIN_6
#define HDMI_CEC_GPIO_Port GPIOB
#define JOY_LEFT_Pin GPIO_PIN_4
#define JOY_LEFT_GPIO_Port GPIOK
#define ETH_TX_EN_Pin GPIO_PIN_11
#define ETH_TX_EN_GPIO_Port GPIOG
#define Audio_INT_Pin GPIO_PIN_15
#define Audio_INT_GPIO_Port GPIOJ
#define SDIO1_D3_Pin GPIO_PIN_11
#define SDIO1_D3_GPIO_Port GPIOC
#define FMC_D26_Pin GPIO_PIN_2
#define FMC_D26_GPIO_Port GPIOI
#define FMC_D23_Pin GPIO_PIN_15
#define FMC_D23_GPIO_Port GPIOH
#define FMC_D22_Pin GPIO_PIN_14
#define FMC_D22_GPIO_Port GPIOH
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define JOY_UP_Pin GPIO_PIN_6
#define JOY_UP_GPIO_Port GPIOK
#define JOY_DOWN_Pin GPIO_PIN_3
#define JOY_DOWN_GPIO_Port GPIOK
#define ETH_TXD1_Pin GPIO_PIN_12
#define ETH_TXD1_GPIO_Port GPIOG
#define SPDIF_RX0_Pin GPIO_PIN_7
#define SPDIF_RX0_GPIO_Port GPIOD
#define SDIO1_CK_Pin GPIO_PIN_12
#define SDIO1_CK_GPIO_Port GPIOC
#define FMC_D27_Pin GPIO_PIN_3
#define FMC_D27_GPIO_Port GPIOI
#define SAI1_SCK_A_Pin GPIO_PIN_5
#define SAI1_SCK_A_GPIO_Port GPIOE
#define SAI1_FS_A_Pin GPIO_PIN_4
#define SAI1_FS_A_GPIO_Port GPIOE
#define SAI1_SD_B_Pin GPIO_PIN_3
#define SAI1_SD_B_GPIO_Port GPIOE
#define FMC_SDCAS_Pin GPIO_PIN_15
#define FMC_SDCAS_GPIO_Port GPIOG
#define TOUCH_INT_Pin GPIO_PIN_7
#define TOUCH_INT_GPIO_Port GPIOK
#define QSPI_BK2_IO3_Pin GPIO_PIN_14
#define QSPI_BK2_IO3_GPIO_Port GPIOG
#define ETH_TXD0_Pin GPIO_PIN_13
#define ETH_TXD0_GPIO_Port GPIOG
#define BL_CTRL_Pin GPIO_PIN_12
#define BL_CTRL_GPIO_Port GPIOJ
#define SDIO1_CMD_Pin GPIO_PIN_2
#define SDIO1_CMD_GPIO_Port GPIOD
#define FMC_D2_Pin GPIO_PIN_0
#define FMC_D2_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA
#define FMC_D21_Pin GPIO_PIN_13
#define FMC_D21_GPIO_Port GPIOH
#define FMC_D30_Pin GPIO_PIN_9
#define FMC_D30_GPIO_Port GPIOI
#define B2_Pin GPIO_PIN_13
#define B2_GPIO_Port GPIOC
#define uSD_Detect_Pin GPIO_PIN_8
#define uSD_Detect_GPIO_Port GPIOI
#define SAI1_SD_A_Pin GPIO_PIN_6
#define SAI1_SD_A_GPIO_Port GPIOE
#define PMOD_12_Pin GPIO_PIN_13
#define PMOD_12_GPIO_Port GPIOJ
#define FMC_D3_Pin GPIO_PIN_1
#define FMC_D3_GPIO_Port GPIOD
#define SDIO1_D0_Pin GPIO_PIN_8
#define SDIO1_D0_GPIO_Port GPIOC
#define SDIO1_D1_Pin GPIO_PIN_9
#define SDIO1_D1_GPIO_Port GPIOC
#define CEC_CK_MCO1_Pin GPIO_PIN_8
#define CEC_CK_MCO1_GPIO_Port GPIOA
#define SPI2_SCK_Pin GPIO_PIN_12
#define SPI2_SCK_GPIO_Port GPIOA
#define PMOD_1_Pin GPIO_PIN_11
#define PMOD_1_GPIO_Port GPIOA
#define FMC_D31_Pin GPIO_PIN_10
#define FMC_D31_GPIO_Port GPIOI
#define FMC_SDCLK_Pin GPIO_PIN_8
#define FMC_SDCLK_GPIO_Port GPIOG
#define SAI1_MCLK_A_Pin GPIO_PIN_7
#define SAI1_MCLK_A_GPIO_Port GPIOG
#define FMC_A2_Pin GPIO_PIN_2
#define FMC_A2_GPIO_Port GPIOF
#define FMC_A1_Pin GPIO_PIN_1
#define FMC_A1_GPIO_Port GPIOF
#define FMC_A0_Pin GPIO_PIN_0
#define FMC_A0_GPIO_Port GPIOF
#define QSPI_BK1_NCS_Pin GPIO_PIN_6
#define QSPI_BK1_NCS_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOI
#define FMC_A3_Pin GPIO_PIN_3
#define FMC_A3_GPIO_Port GPIOF
#define FMC_BA0_Pin GPIO_PIN_4
#define FMC_BA0_GPIO_Port GPIOG
#define DSI_Reset_Pin GPIO_PIN_3
#define DSI_Reset_GPIO_Port GPIOG
#define FMC_A12_Pin GPIO_PIN_2
#define FMC_A12_GPIO_Port GPIOG
#define JOY_SEL_Pin GPIO_PIN_2
#define JOY_SEL_GPIO_Port GPIOK
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define FMC_A5_Pin GPIO_PIN_5
#define FMC_A5_GPIO_Port GPIOF
#define FMC_A4_Pin GPIO_PIN_4
#define FMC_A4_GPIO_Port GPIOF
#define ARD_D13_Pin GPIO_PIN_0
#define ARD_D13_GPIO_Port GPIOK
#define ARD_D10_Pin GPIO_PIN_1
#define ARD_D10_GPIO_Port GPIOK
#define QSPI_BK1_IO3_Pin GPIO_PIN_6
#define QSPI_BK1_IO3_GPIO_Port GPIOF
#define QSPI_BK1_IO2_Pin GPIO_PIN_7
#define QSPI_BK1_IO2_GPIO_Port GPIOF
#define PMOD_14_ARD_D3_Pin GPIO_PIN_8
#define PMOD_14_ARD_D3_GPIO_Port GPIOF
#define ARD_D12_Pin GPIO_PIN_11
#define ARD_D12_GPIO_Port GPIOJ
#define ARD_A1_Pin GPIO_PIN_10
#define ARD_A1_GPIO_Port GPIOF
#define QSPI_BK1_IO1_Pin GPIO_PIN_9
#define QSPI_BK1_IO1_GPIO_Port GPIOF
#define ARD_D11_Pin GPIO_PIN_10
#define ARD_D11_GPIO_Port GPIOJ
#define ETH_MDC_SAI4_D1_Pin GPIO_PIN_1
#define ETH_MDC_SAI4_D1_GPIO_Port GPIOC
#define PMOD_3_Pin GPIO_PIN_2
#define PMOD_3_GPIO_Port GPIOC
#define PMOD_2_Pin GPIO_PIN_3
#define PMOD_2_GPIO_Port GPIOC
#define ARD_D0_Pin GPIO_PIN_9
#define ARD_D0_GPIO_Port GPIOJ
#define QSPI_BK2_IO0_Pin GPIO_PIN_2
#define QSPI_BK2_IO0_GPIO_Port GPIOH
#define ETH_MDIO_Pin GPIO_PIN_2
#define ETH_MDIO_GPIO_Port GPIOA
#define ETH_REF_CLK_Pin GPIO_PIN_1
#define ETH_REF_CLK_GPIO_Port GPIOA
#define ARD_D7_Pin GPIO_PIN_0
#define ARD_D7_GPIO_Port GPIOJ
#define FMC_D7_Pin GPIO_PIN_10
#define FMC_D7_GPIO_Port GPIOE
#define ARD_D1_Pin GPIO_PIN_8
#define ARD_D1_GPIO_Port GPIOJ
#define ARD_D6_Pin GPIO_PIN_7
#define ARD_D6_GPIO_Port GPIOJ
#define ARD_D9_Pin GPIO_PIN_6
#define ARD_D9_GPIO_Port GPIOJ
#define QSPI_BK2_IO1_Pin GPIO_PIN_3
#define QSPI_BK2_IO1_GPIO_Port GPIOH
#define FMC_SDNWE_Pin GPIO_PIN_5
#define FMC_SDNWE_GPIO_Port GPIOH
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOI
#define OTG_HS_OverCurrent_Pin GPIO_PIN_1
#define OTG_HS_OverCurrent_GPIO_Port GPIOJ
#define FMC_A7_Pin GPIO_PIN_13
#define FMC_A7_GPIO_Port GPIOF
#define FMC_A8_Pin GPIO_PIN_14
#define FMC_A8_GPIO_Port GPIOF
#define FMC_D6_Pin GPIO_PIN_9
#define FMC_D6_GPIO_Port GPIOE
#define FMC_D8_Pin GPIO_PIN_11
#define FMC_D8_GPIO_Port GPIOE
#define FMC_D18_Pin GPIO_PIN_10
#define FMC_D18_GPIO_Port GPIOH
#define FMC_D19_Pin GPIO_PIN_11
#define FMC_D19_GPIO_Port GPIOH
#define FMC_D1_Pin GPIO_PIN_15
#define FMC_D1_GPIO_Port GPIOD
#define FMC_D0_Pin GPIO_PIN_14
#define FMC_D0_GPIO_Port GPIOD
#define ARD_A4_Pin GPIO_PIN_2
#define ARD_A4_GPIO_Port GPIOC
#define ARD_A5_Pin GPIO_PIN_3
#define ARD_A5_GPIO_Port GPIOC
#define ETH_CRS_DV_Pin GPIO_PIN_7
#define ETH_CRS_DV_GPIO_Port GPIOA
#define QSPI_CLK_Pin GPIO_PIN_2
#define QSPI_CLK_GPIO_Port GPIOB
#define FMC_A6_Pin GPIO_PIN_12
#define FMC_A6_GPIO_Port GPIOF
#define FMC_A9_Pin GPIO_PIN_15
#define FMC_A9_GPIO_Port GPIOF
#define FMC_D9_Pin GPIO_PIN_12
#define FMC_D9_GPIO_Port GPIOE
#define FMC_D12_Pin GPIO_PIN_15
#define FMC_D12_GPIO_Port GPIOE
#define ARD_D8_Pin GPIO_PIN_5
#define ARD_D8_GPIO_Port GPIOJ
#define FMC_D17_Pin GPIO_PIN_9
#define FMC_D17_GPIO_Port GPIOH
#define FMC_D20_Pin GPIO_PIN_12
#define FMC_D20_GPIO_Port GPIOH
#define QSPI_BK1_IO0_Pin GPIO_PIN_11
#define QSPI_BK1_IO0_GPIO_Port GPIOD
#define ARD_A2_Pin GPIO_PIN_0
#define ARD_A2_GPIO_Port GPIOA
#define ARD_A3_Pin GPIO_PIN_1
#define ARD_A3_GPIO_Port GPIOA
#define ETH_RXD0_Pin GPIO_PIN_4
#define ETH_RXD0_GPIO_Port GPIOC
#define FMC_SDRAS_Pin GPIO_PIN_11
#define FMC_SDRAS_GPIO_Port GPIOF
#define FMC_A10_Pin GPIO_PIN_0
#define FMC_A10_GPIO_Port GPIOG
#define FMC_D5_Pin GPIO_PIN_8
#define FMC_D5_GPIO_Port GPIOE
#define FMC_D10_Pin GPIO_PIN_13
#define FMC_D10_GPIO_Port GPIOE
#define FMC_SDNE1_Pin GPIO_PIN_6
#define FMC_SDNE1_GPIO_Port GPIOH
#define FMC_D16_Pin GPIO_PIN_8
#define FMC_D16_GPIO_Port GPIOH
#define PMOD_8_Pin GPIO_PIN_15
#define PMOD_8_GPIO_Port GPIOB
#define FMC_D15_Pin GPIO_PIN_10
#define FMC_D15_GPIO_Port GPIOD
#define FMC_D14_Pin GPIO_PIN_9
#define FMC_D14_GPIO_Port GPIOD
#define ETH_RXD1_Pin GPIO_PIN_5
#define ETH_RXD1_GPIO_Port GPIOC
#define ARD_D2_Pin GPIO_PIN_3
#define ARD_D2_GPIO_Port GPIOJ
#define ARD_D4_Pin GPIO_PIN_4
#define ARD_D4_GPIO_Port GPIOJ
#define FMC_A11_Pin GPIO_PIN_1
#define FMC_A11_GPIO_Port GPIOG
#define FMC_D4_Pin GPIO_PIN_7
#define FMC_D4_GPIO_Port GPIOE
#define FMC_D11_Pin GPIO_PIN_14
#define FMC_D11_GPIO_Port GPIOE
#define FMC_SDCKE1_Pin GPIO_PIN_7
#define FMC_SDCKE1_GPIO_Port GPIOH
#define PMOD_9_Pin GPIO_PIN_14
#define PMOD_9_GPIO_Port GPIOB
#define FMC_D13_Pin GPIO_PIN_8
#define FMC_D13_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define SDRAM_BANK_ADDR                 ((uint32_t)0xD0000000)

/* #define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_8  */
#define SDRAM_MEMORY_WIDTH            FMC_SDRAM_MEM_BUS_WIDTH_16

#define SDCLOCK_PERIOD                   FMC_SDRAM_CLOCK_PERIOD_2
/* #define SDCLOCK_PERIOD                FMC_SDRAM_CLOCK_PERIOD_3 */

#define SDRAM_TIMEOUT                    ((uint32_t)0xFFFF)
#define REFRESH_COUNT                    ((uint32_t)0x0603)   /* SDRAM refresh counter */

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

/* SDRAM write address */
#define SDRAM_WRITE_READ_ADDR         0xD0177000
#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR
#define AUDIO_REC_TOTAL_SIZE         ((uint32_t) 0x0000E000)
#define AUDIO_RECPDM_START_ADDR      (AUDIO_REC_START_ADDR+AUDIO_REC_TOTAL_SIZE)

#define SD_DMA_MODE      0U
#define SD_IT_MODE       1U
#define SD_POLLING_MODE  2U

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
