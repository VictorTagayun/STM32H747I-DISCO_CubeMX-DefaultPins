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

Step 1:  Configure a clock configuration enable command   
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

## FMC / SDRAM Test 

add in main.c   

	Fill_Buffer(sdram_aTxBuffer, BUFFER_SIZE, 0xA244250F);  
	HAL_SDRAM_Write_32b();   
	HAL_SDRAM_Read_32b();   
	Buffercmp();  


## QSPI Setup (temp) better copy stm32h747i_discovery_qspi.c/h  

BSP_QSPI_Init(below){ 
	/* STM32 QSPI interface initialization */   
	MX_QSPI_Init   
	/* QSPI memory reset */   
	QSPI_ResetMemory   
	/* Force Flash enter 4 Byte address mode */   
	MT25TL01G_AutoPollingMemReady    
	MT25TL01G_Enter4BytesAddressMode   
	/* Configuration of the dummy cycles on QSPI memory side */   
	QSPI_DummyCyclesCfg   
	/* Configure Flash to desired mode */   
	BSP_QSPI_ConfigFlash
}  
