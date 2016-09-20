/*
 * main.c
 * Double stepper motor driver
 * bipolar 0.5 Nm
 *
 * Microcontroller: ATmega8/ATmega88
 *
 *  Created on: 3 jan 2016
 *      Author: Mateusz Staszkow
 */

#include <avr/io.h>
#include <util/delay.h>

//Right motor
//green wire
#define c3 (1<<PD0)
//black wire
#define c4 (1<<PD1)
//red wire
#define c2 (1<<PD2)
//blue wire
#define c1 (1<<PD3)

//Left motor
//green wire
#define l3 (1<<PD4)
//black wire
#define l4 (1<<PD5)
//red wire
#define l2 (1<<PD6)
//blue wire
#define l1 (1<<PD7)

//PORTX: 0 - forward, 1 - backwards
#define PIN_DIRECTION (1<<PC1)
#define DIRECTION (PINC & PIN_DIRECTION)

//Number of pin to measure voltage from
#define ADC_CHANNEL 2

//Offset added to measured value from ADC_CHANNEL. This sum is a time between each step.
#define OFFSET 550

//1 coil, battery safe mode
//right side
#define step1_1cr PORTD &= ~c1; PORTD |= c2;
#define step2_1cr PORTD |= c3; PORTD &= ~c4;
#define step3_1cr PORTD &= ~c2; PORTD |= c1;
#define step4_1cr PORTD |= c4; PORTD &= ~c3;

//left side
#define step1_1cl PORTD &= ~l1; PORTD |= l2;
#define step2_1cl PORTD |= l3; PORTD &= ~l4;
#define step3_1cl PORTD &= ~l2; PORTD |= l1;
#define step4_1cl PORTD |= l4; PORTD &= ~l3;

//2 coils, maximum torque
//right side
#define step1_2cr PORTD &= ~c1; PORTD |= c2; PORTD &= ~c3; PORTD |= c4;
#define step2_2cr PORTD &= ~c1; PORTD |= c2; PORTD |= c3; PORTD &= ~c4;
#define step3_2cr PORTD |= c1; PORTD &= ~c2; PORTD |= c3; PORTD &= ~c4;
#define step4_2cr PORTD |= c1; PORTD &= ~c2; PORTD &= ~c3; PORTD |= c4;

//left side
#define step1_2cl PORTD &= ~l1; PORTD |= l2; PORTD &= ~l3; PORTD |= l4;
#define step2_2cl PORTD &= ~l1; PORTD |= l2; PORTD |= l3; PORTD &= ~l4;
#define step3_2cl PORTD |= l1; PORTD &= ~l2; PORTD |= l3; PORTD &= ~l4;
#define step4_2cl PORTD |= l1; PORTD &= ~l2; PORTD &= ~l3; PORTD |= l4;

//Half step control
//right side
#define step1r PORTD &= ~c1; PORTD |= c2;
#define step2r PORTD &= ~c1; PORTD |= c2; PORTD |= c3; PORTD &= ~c4;
#define step3r PORTD |= c3; PORTD &= ~c4;
#define step4r PORTD &= ~c2; PORTD |= c1; PORTD |= c3; PORTD &= ~c4;
#define step5r PORTD &= ~c2; PORTD |= c1;
#define step6r PORTD &= ~c2; PORTD |= c1; PORTD |= c4; PORTD &= ~c3;
#define step7r PORTD |= c4; PORTD &= ~c3;
#define step8r PORTD &= ~c1; PORTD |= c2; PORTD |= c4; PORTD &= ~c3;

//left side
#define step1l PORTD &= ~l1; PORTD |= l2;
#define step2l PORTD &= ~l1; PORTD |= l2; PORTD |= l3; PORTD &= ~l4;
#define step3l PORTD |= l3; PORTD &= ~l4;
#define step4l PORTD &= ~l2; PORTD |= l1; PORTD |= l3; PORTD &= ~l4;
#define step5l PORTD &= ~l2; PORTD |= l1;
#define step6l PORTD &= ~l2; PORTD |= l1; PORTD |= l4; PORTD &= ~l3;
#define step7l PORTD |= l4; PORTD &= ~l3;
#define step8l PORTD &= ~l1; PORTD |= l2; PORTD |= l4; PORTD &= ~l3;

//void single_coil_control();
//void double_coil_control();
void halfstep_right_forward(uint16_t,uint16_t);
void halfstep_right_backward(uint16_t,uint16_t);
void halfstep_left_forward(uint16_t,uint16_t);
void halfstep_left_backward(uint16_t,uint16_t);

void move_forward(uint16_t,uint16_t,uint16_t);
void move_backwards(uint16_t,uint16_t,uint16_t);

//Time between steps (in microseconds)
volatile uint16_t step_time;
//Step counters
volatile uint16_t cnt_r;
volatile uint16_t cnt_l;

//Changing delay
#define wait_us(time) for(uint16_t i=0;i<(time);i++) _delay_us(1);

uint16_t measure(uint8_t channel);

int main(void)
{
	//ADC initialization
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);

	//Step counters initialization
	cnt_r = 0;
	cnt_l = 0;

	//Ports initialization - right motor
	DDRD |= c1|c2|c3|c4;
	//Ports initialization - left motor
	DDRD |= l1|l2|l3|l4;

	//Pin direction initialization
	DDRC &= ~(PIN_DIRECTION);
	PORTC |= (PIN_DIRECTION);

	//Main loop
	while(1) {

		uint16_t measurement_with_offset = OFFSET+measure(ADC_CHANNEL);

		if(DIRECTION) {
			//move_backwards(cnt_r,cnt_l,(2*measure(2)+476));
			move_backwards(cnt_r,cnt_l,measurement_with_offset);
		} else {
			//move_forward(cnt_r,cnt_l,(2*measure(2)+476));
			move_forward(cnt_r,cnt_l,measurement_with_offset);
		}
	}
}

uint16_t measure(uint8_t channel)
{
	ADMUX = (ADMUX & 0xF8) | channel;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADCW;
}

void halfstep_right_forward(uint16_t position,uint16_t step_time)
{
	switch(position%8)
	{
	case 0:
		step1r;
		break;
	case 1:
		step2r;
		break;
	case 2:
		step3r;
		break;
	case 3:
		step4r;
		break;
	case 4:
		step5r;
		break;
	case 5:
		step6r;
		break;
	case 6:
		step7r;
		break;
	case 7:
		step8r;
		break;
	}

	wait_us(step_time);
	cnt_r++;
	if(cnt_r == 400) cnt_r = 0;
}

void halfstep_left_forward(uint16_t position, uint16_t step_time)
{
	switch(position%8)
	{
	case 0:
		step1l;
		break;
	case 1:
		step2l;
		break;
	case 2:
		step3l;
		break;
	case 3:
		step4l;
		break;
	case 4:
		step5l;
		break;
	case 5:
		step6l;
		break;
	case 6:
		step7l;
		break;
	case 7:
		step8l;
		break;
	}

	wait_us(step_time);
	cnt_l++;
	if(cnt_l == 400) cnt_l = 0;
}

void halfstep_right_backward(uint16_t position, uint16_t step_time)
{
	switch(position%8)
	{
	case 0:
		step1r;
		break;
	case 7:
		step8r;
		break;
	case 6:
		step7r;
		break;
	case 5:
		step6r;
		break;
	case 4:
		step5r;
		break;
	case 3:
		step4r;
		break;
	case 2:
		step3r;
		break;
	case 1:
		step2r;
		break;
	}

	wait_us(step_time);
	cnt_r--;
	if(cnt_r == 0) cnt_r = 400;
}

void halfstep_left_backward(uint16_t position, uint16_t step_time)
{
	switch(position%8)
	{
	case 0:
		step1l;
		break;
	case 7:
		step8l;
		break;
	case 6:
		step7l;
		break;
	case 5:
		step6l;
		break;
	case 4:
		step5l;
		break;
	case 3:
		step4l;
		break;
	case 2:
		step3l;
		break;
	case 1:
		step2l;
		break;
	}

	wait_us(step_time);
	cnt_l--;
	if(cnt_l == 0) cnt_l = 400;
}

void move_forward(uint16_t position_r,uint16_t position_l,uint16_t time)
{
	halfstep_right_forward(position_r,time);
	halfstep_left_backward(position_l,time);
}
void move_backwards(uint16_t position_r,uint16_t position_l, uint16_t time)
{
	halfstep_right_backward(position_r,time);
	halfstep_left_forward(position_l,time);
}
