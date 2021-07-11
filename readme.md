## Introduction and Purpose

** STM32H747XIH6 is a Dual core Cortex-M7 and Cortex-M4 MCU.  
** The STM32H747I-DISCO Discovery kit is a complete demonstration and development platform for STMicroelectronics STM32H747XIH6 microcontroller, designed to simplify user application development.   
** Use CubeMX to init all peripherals  

## FMC / SDRAM Additional Setup after MX init (SDRAM_Initialization_Sequence)  

Step 1:  Configure a clock configuration enable command   
Step 2: Insert 100 us minimum delay   
	Inserted delay is equal to 1 ms due to systick time base unit (ms)   
Step 3: Configure a PALL (precharge all) command    
Step 4 : Configure a Auto-Refresh command   
Step 5: Program the external memory mode register   
Step 6: Set the refresh rate counter    
	Set the device refresh rate    
