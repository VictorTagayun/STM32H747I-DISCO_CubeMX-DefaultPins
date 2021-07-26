## Introduction and Purpose

** STM32H747XIH6 is a Dual core Cortex-M7 and Cortex-M4 MCU.  
** The STM32H747I-DISCO Discovery kit is a complete demonstration and development platform for STMicroelectronics STM32H747XIH6 microcontroller, designed to simplify user application development.   
** Use CubeMX to init all peripherals  

## Setup USART1 

	/* USER CODE BEGIN Includes */
	#include <stdio.h>
	
	/* USER CODE BEGIN PFP */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

	/* USER CODE BEGIN 4 */
	PUTCHAR_PROTOTYPE
	{
	  /* Place your implementation of fputc here */
	  /* e.g. write a character to the LPUART1 and Loop until the end of transmission */
	  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF); // hlpuart1 >> huart1

	  return ch;
	}
	
	// Test USART1
	printf("Testing USART1\n");

## FMC / SDRAM Additional Setup after MX init - General Steps to perform the SDRAM exernal memory inialization sequence  
Where did example come from??***   
maybe from Repository\STM32Cube_FW_H7_V1.9.0\Projects\STM32H743I-EVAL\Examples\FMC\FMC_SDRAM\Src\main.c 

Step 1: Configure a clock configuration enable command   
Step 2: Insert 100 us minimum delay   
	Inserted delay is equal to 1 ms due to systick time base unit (ms)   
Step 3: Configure a PALL (precharge all) command    
Step 4 : Configure a Auto-Refresh command   
Step 5: Program the external memory mode register   
Step 6: Set the refresh rate counter    
	Set the device refresh rate    

### Use SDRAM_Initialization_Sequence(&hsdram1, &Command)  

add in main.c  

	SDRAM_Initialization_Sequence(&hsdram1, &Command);  

	
### Use is42s32800g from BSP Driver Component

in main.c add   

	#include "is42s32800g.h"   
	IS42S32800G_SDRAM_Initialization_Sequence();  

## FMC / SDRAM MDMA Setup  

add in main.c  

	SDRAM_MDMA()

add in main.h

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


## FMC / SDRAM Test 

add in main.c   

	Fill_Buffer(sdram_aTxBuffer, BUFFER_SIZE, 0xA244250F);  
	HAL_SDRAM_Write_32b();   
	HAL_SDRAM_Read_32b();   
	Buffercmp();  


## QSPI Setup (temp), but better copy stm32h747i_discovery_qspi.c/h  

### QSPI_demo from BSP

	/*##-1- Configure the QSPI device ##########################################*/
	BSP_QSPI_Init(0,&init);

	/*##-2- Read & check the QSPI info #######################################*/
	BSP_QSPI_GetInfo(0,&pQSPI_Info);

	/* Test the correctness */
	if((pQSPI_Info.FlashSize != 0x8000000) || (pQSPI_Info.EraseSectorSize != 0x2000)  ||
	(pQSPI_Info.ProgPageSize != 0x100)  || (pQSPI_Info.EraseSectorsNumber != 0x4000) ||
	(pQSPI_Info.ProgPagesNumber != 0x80000))

	/*##-3- Erase QSPI memory ################################################*/
	if(BSP_QSPI_EraseBlock(0,WRITE_READ_ADDR,BSP_QSPI_ERASE_8K) != BSP_ERROR_NONE)

	/*##-4- QSPI memory read/write access  #################################*/
	/* Fill the buffer to write */
	Fill_Buffer(qspi_aTxBuffer, BUFFER_SIZE, 0xD20F);
	/* Write data to the QSPI memory */
	if(BSP_QSPI_Write(0,qspi_aTxBuffer, WRITE_READ_ADDR, BUFFER_SIZE) != BSP_ERROR_NONE)
	/* Read back data from the QSPI memory */
	if(BSP_QSPI_Read(0,qspi_aRxBuffer, WRITE_READ_ADDR, BUFFER_SIZE) != BSP_ERROR_NONE)

	/*##-5- Checking data integrity ############################################*/
	if(Buffercmp(qspi_aRxBuffer, qspi_aTxBuffer, BUFFER_SIZE) > 0)

	/*##-6-Memory Mapped Mode ###############################################*/
	if(BSP_QSPI_EnableMemoryMappedMode(0)!=BSP_ERROR_NONE)			


### Functions in QSPI_demo  

BSP_QSPI_Init = below   

    /* Check if instance is already initialized */  
    if(QSPI_Ctx[Instance].IsInitialized == QSPI_ACCESS_NONE)  
	/* Msp QSPI initialization */
	QSPI_MspInit(&hqspi); = same 
	/* STM32 QSPI interface initialization */   
	MX_QSPI_Init = MX_QUADSPI_Init (from CubeIDE)       
	/* QSPI memory reset */   
	QSPI_ResetMemory = no equivalent, can copy the codes from stm32h747i_discovery_qspi.c         
	/* Force Flash enter 4 Byte address mode */   
	MT25TL01G_AutoPollingMemReady = from mt25tl01g.c   
	MT25TL01G_Enter4BytesAddressMode = from mt25tl01g.c   
	/* Configuration of the dummy cycles on QSPI memory side */   
	QSPI_DummyCyclesCfg = no equivalent, can copy the codes from stm32h747i_discovery_qspi.c  
	/* Configure Flash to desired mode */   
	BSP_QSPI_ConfigFlash =  no equivalent, can copy the codes from stm32h747i_discovery_qspi.c 


BSP_QSPI_ConfigFlash(uint32_t Instance, BSP_QSPI_Interface_t Mode, BSP_QSPI_Transfer_t Rate) = below   

	/* Setup MCU transfer rate setting ***************************************************/
	hqspi.Init.SampleShifting = (Rate == BSP_QSPI_STR_TRANSFER) ? QSPI_SAMPLE_SHIFTING_HALFCYCLE : QSPI_SAMPLE_SHIFTING_NONE;
	/* Setup Flash interface ***************************************************/
	switch(QSPI_Ctx[Instance].InterfaceMode)   
		{
		case MT25TL01G_QPI_MODE :               /* 4-4-4 commands */
			if(Mode != MT25TL01G_QPI_MODE)
			{
				if(MT25TL01G_ExitQPIMode(&hqspi) != MT25TL01G_OK)
				{
					ret = BSP_ERROR_COMPONENT_FAILURE;
				}
			}
		break;

		case BSP_QSPI_SPI_MODE :               /* 1-1-1 commands, Power on H/W default setting */
		case BSP_QSPI_SPI_2IO_MODE :           /* 1-2-2 read commands */
		case BSP_QSPI_SPI_4IO_MODE :           /* 1-4-4 read commands */
		default :
			if(Mode == MT25TL01G_QPI_MODE)
			{
				if(MT25TL01G_EnterQPIMode(&hqspi) != MT25TL01G_OK)
				{
					ret = BSP_ERROR_COMPONENT_FAILURE;
				}
			}
		break;
		}
	/* Update QSPI context if all operations are well done */
	if(ret == BSP_ERROR_NONE) 
	{
		/* Update current status parameter *****************************************/
		QSPI_Ctx[Instance].IsInitialized = QSPI_ACCESS_INDIRECT;
		QSPI_Ctx[Instance].InterfaceMode = Mode;
		QSPI_Ctx[Instance].TransferRate  = Rate;
	}


## QSPI Setup copying stm32h747i_discovery_qspi.c/h (more easy)   

see similar guide in [STM32H747I-DISCO_BSP_Study](https://github.com/VictorTagayun/STM32H747I-DISCO_BSP_Study#test-qspi) 

copy file qspi.c & stm32h747i_discovery_qspi.c to CM7\Core\Src   

from BSP example in Repository\STM32Cube_FW_H7_V1.9.0\Projects\STM32H747I-DISCO\Examples\BSP\CM7\Inc, copy "stm32h747i_discovery_conf.h"   

copy to CM7\Core\Inc the ff files below, check from BSP example folder Repository\STM32Cube_FW_H7_V1.9.0\Projects\STM32H747I-DISCO\Examples\BSP  

	mt25tl01g_conf.h
	stm32h747i_discovery_conf.h
	stm32h747i_discovery_errno.h
	stm32h747i_discovery_qspi.h

copy foder mt25tl01g to CM7\Core\Components\

in main.h add

	#include "stm32h747i_discovery_qspi.h"

## SDCARD Setup copying stm32h747i_discovery_sd.c/h (more easy)   

copy file sd.c and stm32h747i_discovery_sd.c in CM7\Core\Src

copy to CM7\Core\Inc the ff files below, check from BSP example folder Repository\STM32Cube_FW_H7_V1.9.0\Projects\STM32H747I-DISCO\Examples\BSP  

	stm32h747i_discovery_sd.h  
	
in stm32h747i_discovery_sd.h comment "#include "stm32h747i_discovery_bus.h""  

in main.h add

	#include "stm32h747i_discovery_sd.h"
	
in stm32h747i_discovery_sd.c/h rename function MX_SDMMC1_SD_Init to BSP_MX_SDMMC1_SD_Init so it wont have conflict with CUBEMX functions


## LCD Setup copying stm32h747i_discovery_sd.c/h (more easy)   

copy file lcd.c and stm32h747i_discovery_lcd.c in CM7\Core\Src

copy to CM7\Core\Inc the ff files below, check from BSP example folder Repository\STM32Cube_FW_H7_V1.9.0\Projects\STM32H747I-DISCO\Examples\BSP  

	stm32h747i_discovery_lcd.h  
	lcd.h
	
in main.h add

	#include "stm32h747i_discovery_lcd.h"	
	
copy foders adv7533 and otm8009a to CM7\Core\Components\

in stm32h747i_discovery_lcd.c/h rename function MX_LTDC_Init to BSP_MX_LTDC_Init so it wont have conflict with CUBEMX functions

in stm32h747i_discovery_lcd.c/h rename function MX_DSIHOST_DSI_Init to BSP_MX_DSIHOST_DSI_Init so it wont have conflict with CUBEMX functions

in stm32h747i_discovery_lcd.c comment #include "stm32h747i_discovery_bus.h" and  #include "stm32h747i_discovery_sdram.h"

copy Repository\STM32Cube_FW_H7_V1.9.0\Utilities\Fonts folder to CM7\Core

copy stm32_lcd.c\h from Repository\STM32Cube_FW_H7_V1.9.0\Utilities\lcd to CM7\Core Src and Inc folder

in main.h add

	#include "stm32_lcd.h"
	
copy stlogo.h from Repository\STM32Cube_FW_H7_V1.9.0\Projects\STM32H747I-DISCO\Examples\BSP\CM7\Inc to CM7\Core\Inc

in lcd.c add

	#include "stlogo.h"

copy fontXX.c to CM7\Core\Src and fonts.h to CM7\Core\Inc

in stm32_lcd.h comment all

	#include "../Fonts/font24.c"  
	#include "../Fonts/font20.c"  
	#include "../Fonts/font16.c"  
	#include "../Fonts/font12.c"  
	#include "../Fonts/font8.c"  
	
in stm32_lcd.h change #include "../Fonts/fonts.h" to #include "fonts.h"

