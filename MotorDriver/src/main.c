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
#include <avr/interrupt.h>
#include <avr/iom8.h>

//Port which pins are connected with motor wires
#define WIRE_PORT PORTD
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

//Another helpful pins definitions
#define RAP (1<<PD1)
#define RAM (1<<PD0)
#define RBP (1<<PD2)
#define RBM (1<<PD3)
#define LAP (1<<PD5)
#define LAM (1<<PD4)
#define LBP (1<<PD6)
#define LBM (1<<PD7)

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
#define step1r PORTD &= ~c1; PORTD |= c2;  PORTD &= ~c3; PORTD &= ~c4;
#define step2r PORTD &= ~c1; PORTD |= c2;  PORTD |= c3; PORTD &= ~c4;
#define step3r PORTD &= ~c1; PORTD &= ~c2; PORTD |= c3; PORTD &= ~c4;
#define step4r PORTD &= ~c2; PORTD |= c1;  PORTD |= c3; PORTD &= ~c4;
#define step5r PORTD &= ~c2; PORTD |= c1;  PORTD &= ~c3; PORTD &= ~c4;
#define step6r PORTD &= ~c2; PORTD |= c1;  PORTD |= c4; PORTD &= ~c3;
#define step7r PORTD &= ~c1; PORTD &= ~c2; PORTD |= c4; PORTD &= ~c3;
#define step8r PORTD &= ~c1; PORTD |= c2;  PORTD |= c4; PORTD &= ~c3;

//left side
#define step1l PORTD &= ~l1; PORTD |= l2;  PORTD &= ~l3; PORTD &= ~l4;
#define step2l PORTD &= ~l1; PORTD |= l2;  PORTD |= l3; PORTD &= ~l4;
#define step3l PORTD &= ~l1; PORTD &= ~l2; PORTD |= l3; PORTD &= ~l4;
#define step4l PORTD &= ~l2; PORTD |= l1;  PORTD |= l3; PORTD &= ~l4;
#define step5l PORTD &= ~l2; PORTD |= l1;  PORTD &= ~l3; PORTD &= ~l4;
#define step6l PORTD &= ~l2; PORTD |= l1;  PORTD |= l4; PORTD &= ~l3;
#define step7l PORTD &= ~l1; PORTD &= ~l2; PORTD |= l4; PORTD &= ~l3;
#define step8l PORTD &= ~l1; PORTD |= l2;  PORTD |= l4; PORTD &= ~l3;

//void single_coil_control();
//void double_coil_control();
void halfstep_right_forward(uint16_t,uint16_t);
void halfstep_right_backward(uint16_t,uint16_t);
void halfstep_left_forward(uint16_t,uint16_t);
void halfstep_left_backward(uint16_t,uint16_t);

void eighthstep_control_right(uint16_t,uint8_t);
void eighthstep_control_left(uint16_t,uint8_t);

void set_pwm_values_right(uint8_t,uint8_t,uint8_t,uint8_t);
void set_pwm_values_left(uint8_t,uint8_t,uint8_t,uint8_t);

void move_forward(uint16_t,uint16_t,uint16_t);
void move_backwards(uint16_t,uint16_t,uint16_t);

//Time between steps (in microseconds)
volatile uint16_t step_time;
//Step counters
volatile uint16_t cnt_r;
volatile uint16_t cnt_l;

//Amounts for PWMs on each wire:
//Right motor
//green wire
volatile uint8_t pwm1_r;
//black wire
volatile uint8_t pwm2_r;
//red wire
volatile uint8_t pwm3_r;
//blue wire
volatile uint8_t pwm4_r;
//Left motor
//green wire
volatile uint8_t pwm1_l;
//black wire
volatile uint8_t pwm2_l;
//red wire
volatile uint8_t pwm3_l;
//blue wire
volatile uint8_t pwm4_l;

//Changing delay
#define wait_us(time) for(uint16_t i=0;i<(time);i++) _delay_us(1);

uint16_t measure(uint8_t channel);

int main(void)
{
	//PWM timer configuration
	TCCR1B |= (1<<WGM12) | (1<<WGM13);	//CTC mode
	TCCR1B |= (1<<CS10);				//Presdaler = 1
	OCR1B = 199;						//Additional frequency division by 200
	TIMSK |= (1<<OCIE1B);				//Compare to OCR1B register enabled

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

		uint16_t measurement_with_offset = (OFFSET+measure(ADC_CHANNEL))/16;

		if(DIRECTION) {
			//move_backwards(cnt_r,cnt_l,(2*measure(2)+476));
			move_backwards(cnt_r,cnt_l,measurement_with_offset);

//			eighthstep_control_right(measurement_with_offset,0);
//			eighthstep_control_left(measurement_with_offset,1);
		} else {
			//move_forward(cnt_r,cnt_l,(2*measure(2)+476));
			move_forward(cnt_r,cnt_l,measurement_with_offset);

//			eighthstep_control_right(measurement_with_offset,1);
//			eighthstep_control_left(measurement_with_offset,0);
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

/*Table of pwm values for duty cycles:
 * Duty cycle | pwm value
 * 		0			0
 * 		20			51
 * 		38			97
 * 		56			143
 * 		71			181
 * 		83			212
 * 		92			235
 * 		98			250
 * 		100			255
 */
void eighthstep_control_right(uint16_t step_time,uint8_t forward)
{
	if(forward && (cnt_r<6400)) cnt_r++;
	else if(forward && (cnt_r==6400)) cnt_r = 1;
	else if((!forward) && (cnt_r>0)) cnt_r--;
	else if((!forward) && (cnt_r==0)) cnt_r = 6399;

	switch(cnt_r%32)
	{
	case 0:
		set_pwm_values_right(0,0,0,255);
		break;
	case 1:
		set_pwm_values_right(0,51,0,250);
		break;
	case 2:
		set_pwm_values_right(0,97,0,235);
		break;
	case 3:
		set_pwm_values_right(0,143,0,212);
		break;
	case 4:
		set_pwm_values_right(0,181,0,181);
		break;
	case 5:
		set_pwm_values_right(0,212,0,143);
		break;
	case 6:
		set_pwm_values_right(0,235,0,97);
		break;
	case 7:
		set_pwm_values_right(0,250,0,51);
		break;
	case 8:
		set_pwm_values_right(0,255,0,0);
		break;
	case 9:
		set_pwm_values_right(0,250,51,0);
		break;
	case 10:
		set_pwm_values_right(0,235,97,0);
		break;
	case 11:
		set_pwm_values_right(0,212,143,0);
		break;
	case 12:
		set_pwm_values_right(0,181,181,0);
		break;
	case 13:
		set_pwm_values_right(0,143,212,0);
		break;
	case 14:
		set_pwm_values_right(0,97,235,0);
		break;
	case 15:
		set_pwm_values_right(0,51,250,0);
		break;
	case 16:
		set_pwm_values_right(0,0,255,0);
		break;
	case 17:
		set_pwm_values_right(51,0,250,0);
		break;
	case 18:
		set_pwm_values_right(97,0,235,0);
		break;
	case 19:
		set_pwm_values_right(143,0,212,0);
		break;
	case 20:
		set_pwm_values_right(181,0,181,0);
		break;
	case 21:
		set_pwm_values_right(212,0,143,0);
		break;
	case 22:
		set_pwm_values_right(235,0,97,0);
		break;
	case 23:
		set_pwm_values_right(250,0,51,0);
		break;
	case 24:
		set_pwm_values_right(255,0,0,0);
		break;
	case 25:
		set_pwm_values_right(250,0,0,51);
		break;
	case 26:
		set_pwm_values_right(235,0,0,97);
		break;
	case 27:
		set_pwm_values_right(212,0,0,143);
		break;
	case 28:
		set_pwm_values_right(181,0,0,181);
		break;
	case 29:
		set_pwm_values_right(143,0,0,212);
		break;
	case 30:
		set_pwm_values_right(97,0,0,235);
		break;
	case 31:
		set_pwm_values_right(51,0,0,250);
		break;
	}

	wait_us(step_time);
}

void eighthstep_control_left(uint16_t step_time,uint8_t forward)
{
	if(forward && (cnt_l<6400)) cnt_l++;
	else if(forward && (cnt_l==6400)) cnt_l = 1;
	else if((!forward) && (cnt_l>0)) cnt_l--;
	else if((!forward) && (cnt_l==0)) cnt_l = 6399;

	switch(cnt_l%32)
	{
	case 0:
		set_pwm_values_left(0,0,0,255);
		break;
	case 1:
		set_pwm_values_left(0,51,0,250);
		break;
	case 2:
		set_pwm_values_left(0,97,0,235);
		break;
	case 3:
		set_pwm_values_left(0,143,0,212);
		break;
	case 4:
		set_pwm_values_left(0,181,0,181);
		break;
	case 5:
		set_pwm_values_left(0,212,0,143);
		break;
	case 6:
		set_pwm_values_left(0,235,0,97);
		break;
	case 7:
		set_pwm_values_left(0,250,0,51);
		break;
	case 8:
		set_pwm_values_left(0,255,0,0);
		break;
	case 9:
		set_pwm_values_left(0,250,51,0);
		break;
	case 10:
		set_pwm_values_left(0,235,97,0);
		break;
	case 11:
		set_pwm_values_left(0,212,143,0);
		break;
	case 12:
		set_pwm_values_left(0,181,181,0);
		break;
	case 13:
		set_pwm_values_left(0,143,212,0);
		break;
	case 14:
		set_pwm_values_left(0,97,235,0);
		break;
	case 15:
		set_pwm_values_left(0,51,250,0);
		break;
	case 16:
		set_pwm_values_left(0,0,255,0);
		break;
	case 17:
		set_pwm_values_left(51,0,250,0);
		break;
	case 18:
		set_pwm_values_left(97,0,235,0);
		break;
	case 19:
		set_pwm_values_left(143,0,212,0);
		break;
	case 20:
		set_pwm_values_left(181,0,181,0);
		break;
	case 21:
		set_pwm_values_left(212,0,143,0);
		break;
	case 22:
		set_pwm_values_left(235,0,97,0);
		break;
	case 23:
		set_pwm_values_left(250,0,51,0);
		break;
	case 24:
		set_pwm_values_left(255,0,0,0);
		break;
	case 25:
		set_pwm_values_left(250,0,0,51);
		break;
	case 26:
		set_pwm_values_left(235,0,0,97);
		break;
	case 27:
		set_pwm_values_left(212,0,0,143);
		break;
	case 28:
		set_pwm_values_left(181,0,0,181);
		break;
	case 29:
		set_pwm_values_left(143,0,0,212);
		break;
	case 30:
		set_pwm_values_left(97,0,0,235);
		break;
	case 31:
		set_pwm_values_left(51,0,0,250);
		break;
	}

	wait_us(step_time);
}

void set_pwm_values_right(uint8_t ram,uint8_t rap,uint8_t rbm,uint8_t rbp)
{
	pwm1_r = ram;
	pwm2_r = rap;
	pwm3_r = rbm;
	pwm4_r = rbp;
}

void set_pwm_values_left(uint8_t lam,uint8_t lap,uint8_t lbm,uint8_t lbp)
{
	pwm1_l = lam;
	pwm2_l = lap;
	pwm3_l = lbm;
	pwm4_l = lbp;
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

//Software PWM configuration
ISR( TIMER1_COMPB_vect )
{
	//PWM counter definition
	static uint8_t cnt;

	//Direct PWM channels output control. See declarations of pwmx_x variables.
	if(cnt>=pwm1_r) WIRE_PORT |= RAM; else WIRE_PORT &= ~RAM;
	if(cnt>=pwm2_r) WIRE_PORT |= RAP; else WIRE_PORT &= ~RAP;
	if(cnt>=pwm3_r) WIRE_PORT |= RBM; else WIRE_PORT &= ~RBM;
	if(cnt>=pwm4_r) WIRE_PORT |= RBP; else WIRE_PORT &= ~RBP;

	if(cnt>=pwm1_l) WIRE_PORT |= LAM; else WIRE_PORT &= ~LAM;
	if(cnt>=pwm2_l) WIRE_PORT |= LAP; else WIRE_PORT &= ~LAP;
	if(cnt>=pwm3_l) WIRE_PORT |= LBM; else WIRE_PORT &= ~LBM;
	if(cnt>=pwm4_l) WIRE_PORT |= LBP; else WIRE_PORT &= ~LBP;

	cnt++;
}
