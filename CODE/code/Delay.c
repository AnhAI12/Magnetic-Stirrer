#include "stm32l4xx.h"                  // Device header

#include "Delay.h"


/* Counts 1ms timeTicks */
//volatile uint32_t usTicks = 0;
 
// delay function: micros >= 1;   

void DelayUs(uint32_t micros){

	micros *= 1;  //SYS CLK is 8 MHz
	

	//substitute 8 cycles for each call of asm code below == //micros /= 8; 

	while(micros--);
}
 
//void DelayUs(uint32_t dlyTicks)
//{
//      uint32_t curTicks;
// 
//      curTicks = usTicks;
//      while ((usTicks - curTicks) < dlyTicks) ;
//}


//void SysTick_Handler(void)
//{
//      /* Increment counter necessary in Delay()*/
//      usTicks++;
//}
