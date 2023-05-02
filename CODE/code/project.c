#include "stm32l4xx.h"                  // Device header
//#include "Delay.h"
//#include "TM1637.h"

/* PA1: analog from POT,			PA3: pwm */

volatile uint16_t duty_pwm, count_tim15, duty_pwm_start;
volatile char flag_btn, flag_start, flag_motor_start, flag_after_start_timing;
volatile uint32_t count=0;
volatile uint32_t data_ADC =0, vel_setup=0;
volatile uint32_t vel_rpm, count_vel_timer, old_count, curr_count, old_cnt, curr_cnt, sample_time;

/*---------MY FUNCTION-------*/
void ADC_Wakeup (void);
void ADC1_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void ADC_Init(void);
void PWM_Init(void);
void TIM6_Init(void);
void EXTI9_5_IRQHandler(void);
void button_Init(void);
void start_program(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void sensor_Init(void);
/*-------------------*/

/*--------- TIM15 Interrupt -------*/
void TIM1_BRK_TIM15_IRQHandler(void)
{
	// clear UIF, need to put it at the beginning of the interrupt function
	TIM15->SR = 0; 
	//code
	//count_tim15++;
	if(flag_start==1)
	{
		count_tim15++;
		if(count_tim15>=15000)		//1.5s
		{
			//enable read ADC
			TIM6->CR1 |= (1<<0);		//timer 6 is used to read adc
			ADC1->CR |= (1<<0);			//enable  ADC
			//enable interrupt button 
			EXTI->IMR1 |=(1<<5);  	//Event request from line 5 is not masked
			//hold
			flag_start=0;
			count_tim15=0;
		}
	}
	
	if(flag_motor_start==1)
	{
		count_tim15++;
		//count time for calculate motor velocity 
		count_vel_timer++;
		//after press button about 2s, decrease velocity of motor, decrease 1 duty per 30ms
		if( count_tim15>=30000 && (((count_tim15-30000)%300)==0) )
		{
			if(duty_pwm_start <=duty_pwm)
			{
				TIM15->CCR2 = duty_pwm;		//motor run with duty being setting
				flag_motor_start=0;
				flag_after_start_timing=1;	//enable flag which allow using ADC to update duty
			}
			else {	
				duty_pwm_start--;								//decrease 1 duty
				TIM15->CCR2 = duty_pwm_start;		//motor run with new duty
			}
		}
	}
	//update duty  
	if(flag_after_start_timing==1)	{
		TIM15->CCR2 = duty_pwm;
		//continous count time(us)
		count_vel_timer++;
	}
	
}

/*-------- ANALOG FUNCTION ------*/
void ADC1_IRQHandler(void)
{
	//kiem tra su kien gi xay ra ngat
	//co phai ready khong
	if((ADC1->ISR & 0x01) == 1)
	{
		//clear co ngat
		ADC1->ISR |= (1<<0);
		count++;
		//Software can start one ADC conversion by setting the ADC_CR_ADSTART bit in the ADC1->CR register
		ADC1->CR |= (1<<2);
	}
	//Software has to wait the completion of ADC conversion by checking whether ADC_CSR_EOC_MST in the ADC123_COMMON->CSR register has been set by the hardware.
	if(((ADC1->ISR & 0x04)>>2) == 1)
	{
		//clear co ngat
		ADC1->ISR |= (1<<2);
		//count++;
		data_ADC = ADC1->DR;
		duty_pwm= data_ADC*100/4096;			//duty_pwm from 0 to 100%
		if(duty_pwm<10) duty_pwm=10;			//duty pwm min is 10%
		//TIM15->CCR2 = duty_pwm;
		//veloctity set up after press button, this variable is used display on led 7seg
		vel_setup= data_ADC*6000/4096;		// velocity max is 6000rpm
		if(vel_setup<600) vel_setup=600;	//vel min is 600rpm
	}
}


void TIM6_DAC_IRQHandler(void){
	// clear UIF, need to put it at the beginning of the interrupt function
	TIM6->SR = 0; 
  // my code  
	count = count + 1;      // optional, to check the timer in debug mode
	//Software can start one ADC conversion by setting the ADC_CR_ADSTART bit in the ADC1->CR register
	ADC1->CR |= (1<<2);
	
}


/*---- INTERRUPT BUTTON AND SENSOR EXTI FUNCTION ------*/
void EXTI9_5_IRQHandler(void)
{
	//check PA5 interrupt flag and clear this flag
	//interrupt when on motor button is pressed 
	if (EXTI->PR1&(1<<5))
	{
		EXTI->PR1 |=(1<<1);	//CLEAR flag
		//my code
		if(flag_btn) 				//when push off motor
		{
			flag_after_start_timing=0;
			flag_btn=0;
			TIM15->CCR2 = 0;		//duty pwm = 0% 
			//reset velocity
			count_vel_timer=0;	//clear count velocity
			vel_rpm=0;
			old_count=0;
			old_cnt=0;
			curr_count = 0;
			curr_cnt = 0;
			GPIOA->ODR &= ~(1<<4);						// turn off green led
			//disable timer15???
		}
		else {							//push on motor
			flag_btn=1;
			flag_motor_start=1;							//motor start
			duty_pwm_start=100;	
			TIM15->CCR2 = duty_pwm_start;		//motor run duty=100%
			GPIOA->ODR |= (1<<4);						// turn on green led
		}
	}
	
	//check PA6 interrupt flag and clear this flag
	// interrupt when sensor Hall HIGH
	if (EXTI->PR1&(1<<6))
	{
		EXTI->PR1 |=(1<<1);	//CLEAR flag
		//my code
		//save current time
		curr_count = count_vel_timer;
		curr_cnt = TIM15->CNT;
		//calculate motor velocity
		//firstly, sample time (us) = current time - old time 
		// one current_count is 100us, one curr_cnt is 1us
		sample_time=(curr_count*100 + curr_cnt*1) - (old_count*100 + old_cnt*1);
		//sample time is time of motor run 1/2 round
		sample_time= sample_time*2;
		//velocity = 1 round / sample time
		vel_rpm= 1*1000000000/sample_time;		//round per second
		vel_rpm= vel_rpm*60;									// rpm
		//after calculator velocity, save time again
		old_count=curr_count;
		old_cnt=curr_cnt;
	}
}


/*------------ MAIN FUNTION ----------- */
int main(void)
{
	//1. Enable RCC PORTA
	RCC->AHB2ENR |= 1;		//
	//config PA1 analog
	GPIOA->MODER &= ~(1<<6);				//PA3 alternate function
	GPIOA->AFR[0] |= (7<<13);				// PA3 AF14
	//configure PA4 output control green led
	GPIOA->MODER &= ~(1<<9);
	GPIOA->ODR &=~(1<<4);			// Turn off green led
	//configure button motor PA5 as EXTI
	button_Init();
	//configgure sensor Hall A3144 PA6 as EXTI
	sensor_Init();
	//3. configure pin PA5 TIM15 as pwm mode
	PWM_Init();
	//4. config PA1 analog ADC from POT
	ADC_Init();
	// Configure timer6
	TIM6_Init();
//	//5. start timer 15
//	TIM15->CR1|=1;
	
//	//6. Enable counter
//	TIM6->CR1 |= (1<<0);	
//	ADC1->CR |= (1<<0);
	
	//start program
	start_program();	
	while(1)
	{
	}
}

/*--------- FUNCTIONS --------*/
void start_program(void)
{
	data_ADC=0;
	flag_btn=0;
	count_tim15=0;
	duty_pwm_start=0;
	flag_motor_start=0;
	flag_after_start_timing=0;
	count_vel_timer=0;	//clear count velocity
	vel_rpm=0;
	old_count=0;
	old_cnt=0;
	curr_count = 0;
	curr_cnt = 0;
	//motor stop
	duty_pwm=0;
	TIM15->CCR2 = duty_pwm;
	//off ADC
	TIM6->CR1 &= ~(1<<0);		//timer 6 is used to read adc
	ADC1->CR &= ~(1<<0);		//disable start ADC
	//disable button 
	EXTI->IMR1&=~(1<<5);  //Event request from line 5 is masked
	//hold 2s
	flag_start=1;
	TIM15->CR1|=1;					//enable TIM15
}

void button_Init(void)
{
	//enable SYSCG register
	RCC->APB2ENR|=(1<<0); 	//SYSCG
	GPIOA->MODER &= ~(3<<10);		//PA5 input
	GPIOA->PUPDR |= 1<<10;			// PA5 Pullup
	// enbale EXTI PA5
	SYSCFG->EXTICR[1] &= ~(7<<4);
	//CONFIG EXTI PA5
	EXTI->PR1 |=(1<<5);		//CLEAR FLAG PIF5
//	//Event request from line 5 is not masked
//	EXTI->IMR1|=(1<<5);		
	//Rising trigger enabled, Rising when button is released
	EXTI->RTSR1|=(1<<5);
	//Falling trigger enabled,Falling when button is pressed, note: be using Pull-up
	EXTI->FTSR1 |= (1<<5);
	//Enable NVIC
	NVIC_ClearPendingIRQ( EXTI9_5_IRQn);
	NVIC_EnableIRQ( EXTI9_5_IRQn);
}

void sensor_Init(void){
	//enable SYSCG register
	//RCC->APB2ENR|=(1<<0); 	//SYSCG
	GPIOA->MODER &= ~(3<<12);		//PA6 input
	// enbale EXTI PA6
	SYSCFG->EXTICR[1] &= ~(7<<8);
	//CONFIG EXTI PA6
	EXTI->PR1 |=(1<<6);		//CLEAR FLAG PIF6
	//Event request from line 6 is not masked
	EXTI->IMR1|=(1<<6);		
	//Rising trigger enabled
	EXTI->RTSR1|=(1<<6);
	//EXTI->FTSR1 |= (1<<5);
	//Enable NVIC
	NVIC_ClearPendingIRQ( EXTI9_5_IRQn);
	NVIC_EnableIRQ( EXTI9_5_IRQn);
}

void TIM6_Init(void)
{
	// Enable clock for GPIOB and TIM6
	RCC->AHB2ENR |= (1<<1);
	RCC->APB1ENR1 |= (1<<4);
	// Set prescaler for APB1 and TIM6
	//RCC->CFGR |= (5<<8);  // APB1 prescale = 1/4 -> PCLK = 1MHz -> timer frequency = 2MHz
	TIM6->PSC = 40;       // PCLK=4MHz -> 1 clock = 10us
//	// Set PB3 as output
//	GPIOB->MODER = 0xFFFFFF7F;
//	GPIOB->ODR &= ~(1UL<<3);
	// Configure TIM6
	TIM6->ARR = 50000;
	TIM6->CNT = 0;
	TIM6->DIER |= (1<<0);
	// Enable NVIC
	__enable_irq();
	NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void ADC_Init(void)
{
	//Enable ADC clock bit RCC_AHB2ENR_ADCEN in the RCC->AHB2ENR register.
	RCC->AHB2ENR |= (1<<13);
	//Disable ADC1 by clearing the ADC_CR_ADEN in the ADC1->CR register. 		(phai la xoa bit chu?)
	ADC1->CR |= (1<<0);	
	//Enable I/O analog switches voltage booster (SYSCFG_CFGR1_BOOSTEN) in the ADC123_COMMON->CCR register.  (la sao ta)
	SYSCFG->CFGR1 |= (1<<8);
	//Set ADC_CCR_VREFEN bit in the ADC123_COMMON->CCR register to enable the conversion of internal channels. This is required to make conversion of internal channels
	ADC1_COMMON ->CCR |= (1<<22);
	//Configure the ADC prescaler to select the frequency of the clock to the ADC (set clock not divided) in ADC123_COMMON->CCR
	
	//Configure ADC_CCR_CKMODE bits in ADC123_COMMON->CCR to set the ADC clock mode as synchronous clock mode (HCLK/1).
	ADC1_COMMON ->CCR |= (1<<16);
	//Configure all ADCs as independent (clear ADC_CCR_DUAL bits) in ADC123_COMMON->CCR
	
	//By default, the ADC is in deep-power-down mode where its supply is internally switched off to reduce the leakage currents. Therefore, software needs to wait up ADC. The ADC_Wakeup() function is provided in the project template.
	ADC_Wakeup();
	
	//Configure RES bits in ADC1->CFGR to set the resolution as 12 bits.
		//ADC1->CFGR 
	//Select right alignment in the ADC1->CFGR register.
	
	//Clear ADC_SQR1_L bits in ADC1->SQR1 to select 1 conversion in the regular channel conversion sequence.
	
	//Specify the channel number 5 as the 1st conversion in regular sequence (ADC1->SQR1)
	//ADC1->SQR1 |= (1<<7)|(1<<8);
	ADC1->SQR1 |= 6<<6;				//PA1 IN6
	//Configure the channel 6 as single-ended (ADC1->DIFSEL)
		//ADC1->DIFSEL 
	//Select ADC sample time in ADC1->SMPR1. The sampling time must be long enough for the input voltage source to charge the embedded capacitor to the input voltage level.

	//Select ADC as discontinuous mode by clearing the ADC_CFGR_CONT bits in ADC1->CFGR
		//ADC1->CFGR |= 
	//Clear ADC_CFGR_EXTEN bits in ADC1->CFGR to select software trigger
	
	//ADC interrupt enable (ADC_IER)
		//ADC ready and  End of regular conversion interrupt enable
		ADC1->IER |= (1<<0)|(1<<2);
		// Enable NVIC
		__enable_irq();
		NVIC_ClearPendingIRQ(ADC1_IRQn);
		NVIC_EnableIRQ(ADC1_IRQn);
	//Enable ADC1 by setting the ADC_CR_ADEN bit in the ADC1->CR register
}


void ADC_Wakeup (void) {
 int wait_time;
 // To start ADC operations, the following sequence should be applied
 // DEEPPWD = 0: ADC not in deep-power down
 // DEEPPWD = 1: ADC in deep-power-down (default reset state)
 if ((ADC1->CR & ADC_CR_DEEPPWD) == ADC_CR_DEEPPWD)
 ADC1->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down mode if still in that state
 // Enable the ADC internal voltage regulator
 // Before performing any operation such as launching a calibration or enabling the ADC,
 // the ADC voltage regulator must first be enabled and the software must wait for the
 // regulator start-up time.
 ADC1->CR |= ADC_CR_ADVREGEN;
 // Wait for ADC voltage regulator start-up time
 // The software must wait for the startup time of the ADC voltage regulator
 // (T_ADCVREG_STUP, i.e. 20 us) before launching a calibration or enabling the ADC.
 wait_time = 20 * (4000000 / 1000000);		//so nay la gif?
 while(wait_time != 0) {
 wait_time--;
 }
}


void PWM_Init(void)
{
	//1. enable RCC TIM15
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	//2. select PWM mode 1 on TIM15_CH2, OCxM (CCMRx)
	TIM15->CCMR1|= (3<<13);					//PWM MODE 1
	//3. Enable OCxPE bit in TIMx_CCMRx
	TIM15->CCMR1 |= 1<<11;
	//4. enable ARPE in CR1
	TIM15->CR1 |= TIM_CR1_ARPE;
	//4.1 set UG
	TIM15->EGR |= 1;		
	//5. enable the CCxE, CCxNE, MOE, OSSI and OSSR bits (TIMx_CCER and TIMx_BDTR registers)
	TIM15->CCER |= 1<<4;			//ENABLE CC2E
	TIM15->BDTR |= ((3<<14)|(3<<10));			//SET MOE AOE, OSSI, OSSR
	//6. Prescale
	TIM15->PSC=4;							//1 clock 1us
	TIM15->CR1 &= ~(1<<3);			//not stop updated 
	TIM15->DIER |= (1<<0);			//enable interrupt
	TIM15->ARR =100;						//max f=10khz, interrupt after 100us
	TIM15->CNT=0;
	int value = TIM15->CNT;
	duty_pwm=0;
	TIM15->CCR2 = duty_pwm;
	//ENABLE NVIC
	__enable_irq();
	NVIC_ClearPendingIRQ(TIM1_BRK_TIM15_IRQn);
	NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
}
