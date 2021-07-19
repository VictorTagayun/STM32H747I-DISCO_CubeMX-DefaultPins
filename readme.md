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

## QSPI_demo

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


## QSPI Setup (temp) better copy stm32h747i_discovery_qspi.c/h  

BSP_QSPI_Init = below   

    /* Check if instance is already initialized */  
    if(QSPI_Ctx[Instance].IsInitialized == QSPI_ACCESS_NONE)  
	/* Msp QSPI initialization */
	QSPI_MspInit(&hqspi); = same 
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

