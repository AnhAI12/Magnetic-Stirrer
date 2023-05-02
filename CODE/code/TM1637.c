#include "stm32l4xx.h"                  // Device header

#include "TM1637.h"
#include "Delay.h"


#define CLK_PORT 	GPIOA
#define CLK_PIN	 	5
#define DIO_PORT  	GPIOA
#define DIO_PIN		6

void TM1637Start(void);
void TM1637Stop(void);
void TM1637Ack(void);
void TM1637WriteByte(unsigned char b);
// void DelayUs(unsigned int i);
void ClkHigh(void);
void ClkLow(void);
void DioHigh(void);
void DioLow(void);


// common anode
const char segment[] = {  0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
    0x00
};

void TM1637Init(void)
{
	// configuring CLK and DIO Pin
	CLK_PORT->MODER |= (1<<(2*CLK_PIN));
	CLK_PORT->OSPEEDR |= (3<<(2*CLK_PIN));   // High speed
	DIO_PORT->MODER |= (1<<(2*DIO_PIN));
	DIO_PORT->OSPEEDR |= (3<<(2*DIO_PIN));	//// High speed
	
	TM1637SetBrightness(8);
}

void TM1637Display(int value)
{
	unsigned char digit[4];
	for(int i=0;i<4;i++){
		digit[i] = segment[(value%10)];
		value = value/10;
	}
	// Send data command
	TM1637Start();
	TM1637WriteByte(0x40);
	TM1637Ack();
	TM1637Stop();
	// Send 1st address
	TM1637Start();
	TM1637WriteByte(0xc0);
	TM1637Ack();
	// Send digital value
	for(int i=0;i<4;i++){
		TM1637WriteByte(digit[3-i]);
		TM1637Ack();
	}
	TM1637Stop();
}

void TM1637SetBrightness(char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness
    TM1637Start();
    TM1637WriteByte(0x87 + brightness);
    TM1637Ack();
    TM1637Stop();
}

void TM1637WriteByte(unsigned char b)
{
	// LSB first
	for (int i=0; i<8; i++){
		ClkLow();
		if(b&0x01){
			DioHigh();
		}
		else{
			DioLow();
		}
		DelayUs(3);
		b = b >> 1;
		ClkHigh();
		DelayUs(3);
	}
}

void TM1637Start(void)
{
	ClkHigh();
	DioHigh();
	DelayUs(2);
	DioLow();
}

void TM1637Stop(void)
{
	ClkLow();
	DioLow();
	DelayUs(2);
	ClkHigh();
	DelayUs(2);
	DioHigh();
}

void TM1637Ack(void)
{
	ClkLow();
	DelayUs(5);
	ClkHigh();
	DelayUs(2);
	ClkLow();	
}


void ClkHigh(void)
{
	CLK_PORT->ODR |= (1<<CLK_PIN);	
}

void ClkLow(void)
{
	CLK_PORT->ODR &= ~(1<<CLK_PIN);	
}

void DioHigh(void)
{
	DIO_PORT->ODR |= (1<<DIO_PIN);
}

void DioLow(void)
{
	DIO_PORT->ODR &= ~(1<<DIO_PIN);	
}
