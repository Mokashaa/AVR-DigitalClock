/*
 * project.c
 *
 *  Created on: Jun 18, 2017
 *  Author: Mohamed Okasha
 */

#include "adc.h"
#include "lcd.h"

void Timer0_PWM_Init(uint16 set_duty_cycle);
void INT2_init (void);

uint8 direction = 0x01;


ISR(INT2_vect)
{
	if(direction == 0x01)
	{
		direction =0x02;
	}
	else
	{
		direction = 0x01;
	}
}
int main(void)
{
	uint16 res_value;
	LCD_init(); /* initialize LCD driver */
	ADC_init(); /* initialize ADC driver */
	LCD_clearScreen(); /* clear LCD at the beginning */
	/* display this string "ADC Value = " only once at LCD */
	LCD_displayString("ADC Value = ");
	//Motor Configurations
	DDRB = DDRB | (1<<PB0);
	DDRB = DDRB | (1<<PB1);
	PORTB &= ~ (1<<PB0);
	PORTB &= ~ (1<<PB1);
	INT2_init();
	SREG |= (1<<7);
    while(1)
    {
		PORTB = (PORTB & 0x11111100)|(direction);
		LCD_goToRowColumn(0,12); /* display the number every time at this position */
		res_value = ADC_readChannel(0); /* read channel zero where the potentiometer is connect */
		LCD_intgerToString(res_value); /* display the ADC value on LCD screen */
		Timer0_PWM_Init(((float)res_value*255)/1023);
    }
}

void Timer0_PWM_Init(uint16 set_duty_cycle)
{
	TCNT0 = 0;
	OCR0  = set_duty_cycle;
	DDRB  = DDRB | (1<<PB3);
	TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);
}
void INT2_init (void)
{
	DDRB &= ~ (1<<PB2);
	PORTB |= (1<<PB2);
	GICR  |= (1<<INT2);
	MCUCSR &= ~ (1<<ISC2);
}

