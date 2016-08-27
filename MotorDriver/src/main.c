/*
 * main.c
 * Sterownik silnika krokowego:
 * 0,5 Nm - bipolarny
 *
 *  Created on: 3 sty 2016
 *      Author: Mateusz Staszkow
 */

#include <avr/io.h>
#include <util/delay.h>
//#include "LCD/lcd44780.h"

//silnik prawy
//zielony
#define c3 (1<<PD0)
//czarny
#define c4 (1<<PD1)
//czerwony
#define c2 (1<<PD2)
//niebieski
#define c1 (1<<PD3)

//silnik lewy
//zielony
#define l3 (1<<PD4)
//czarny
#define l4 (1<<PD5)
//czerwony
#define l2 (1<<PD6)
//niebieski
#define l1 (1<<PD7)

/*
//silnik prawy
//zielony
#define c1 (1<<PD2)
//czarny
#define c2 (1<<PD3)
//czerwony
#define c3 (1<<PD1)
//niebieski
#define c4 (1<<PD0)

//silnik lewy
//zielony
#define l1 (1<<PD6)
//czarny
#define l2 (1<<PD7)
//czerwony
#define l3 (1<<PD5)
//niebieski
#define l4 (1<<PD4)
*/
//0 do przodu, 1 do tylu
#define PIN_KIERUNEK (1<<PC1)
#define KIERUNEK (PINC & PIN_KIERUNEK)

//jedna cewka, jazda ekonomiczna
//prawy
#define krok1_ep PORTD &= ~c1; PORTD |= c2;
#define krok2_ep PORTD |= c3; PORTD &= ~c4;
#define krok3_ep PORTD &= ~c2; PORTD |= c1;
#define krok4_ep PORTD |= c4; PORTD &= ~c3;
//lewy
#define krok1_el PORTD &= ~l1; PORTD |= l2;
#define krok2_el PORTD |= l3; PORTD &= ~l4;
#define krok3_el PORTD &= ~l2; PORTD |= l1;
#define krok4_el PORTD |= l4; PORTD &= ~l3;

//dwie cewki, maksymalny moment
//prawy
#define krok1mp PORTD &= ~c1; PORTD |= c2; PORTD &= ~c3; PORTD |= c4;
#define krok2mp PORTD &= ~c1; PORTD |= c2; PORTD |= c3; PORTD &= ~c4;
#define krok3mp PORTD |= c1; PORTD &= ~c2; PORTD |= c3; PORTD &= ~c4;
#define krok4mp PORTD |= c1; PORTD &= ~c2; PORTD &= ~c3; PORTD |= c4;
//lewy
#define krok1ml PORTD &= ~l1; PORTD |= l2; PORTD &= ~l3; PORTD |= l4;
#define krok2ml PORTD &= ~l1; PORTD |= l2; PORTD |= l3; PORTD &= ~l4;
#define krok3ml PORTD |= l1; PORTD &= ~l2; PORTD |= l3; PORTD &= ~l4;
#define krok4ml PORTD |= l1; PORTD &= ~l2; PORTD &= ~l3; PORTD |= l4;

//dwie cewki, duzy moment, sterowanie polkrokowe
//prawy
#define krok1p PORTD &= ~c1; PORTD |= c2;
#define krok2p PORTD &= ~c1; PORTD |= c2; PORTD |= c3; PORTD &= ~c4;
#define krok3p PORTD |= c3; PORTD &= ~c4;
#define krok4p PORTD &= ~c2; PORTD |= c1; PORTD |= c3; PORTD &= ~c4;
#define krok5p PORTD &= ~c2; PORTD |= c1;
#define krok6p PORTD &= ~c2; PORTD |= c1; PORTD |= c4; PORTD &= ~c3;
#define krok7p PORTD |= c4; PORTD &= ~c3;
#define krok8p PORTD &= ~c1; PORTD |= c2; PORTD |= c4; PORTD &= ~c3;
//lewy
#define krok1l PORTD &= ~l1; PORTD |= l2;
#define krok2l PORTD &= ~l1; PORTD |= l2; PORTD |= l3; PORTD &= ~l4;
#define krok3l PORTD |= l3; PORTD &= ~l4;
#define krok4l PORTD &= ~l2; PORTD |= l1; PORTD |= l3; PORTD &= ~l4;
#define krok5l PORTD &= ~l2; PORTD |= l1;
#define krok6l PORTD &= ~l2; PORTD |= l1; PORTD |= l4; PORTD &= ~l3;
#define krok7l PORTD |= l4; PORTD &= ~l3;
#define krok8l PORTD &= ~l1; PORTD |= l2; PORTD |= l4; PORTD &= ~l3;

//void sterowanie_eko(void);
void polkrokowy_prawy_przod(int,int);
void polkrokowy_prawy_tyl(int,int);
void polkrokowy_prawy_hamuj(int,int);
void polkrokowy_lewy_przod(int,int);
void polkrokowy_lewy_tyl(int,int);
void polkrokowy_lewy_hamuj(int,int);

void pelnokrokowy_prawy_przod(int);
void pelnokrokowy_lewy_przod(int);

void przod(int,int,int);
void tyl(int,int,int);

//czas pomiedzy krokami ( w mikrosekundach)
volatile int czas_t;
volatile int czas_p;

volatile int czas_m;
//wybor sterowania
volatile int sterowanie;
//licznik krokow
volatile int cnt_p;
volatile int cnt_l;

//zmienne opoznienie
#define czekaj(czas) for(int i=0;i<(czas);i++) _delay_us(1);

uint16_t pomiar(uint8_t kanal);

int main(void) {

	//inicjalizacja ADC
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);

	int test=0;
	int i;

	//domyslnie maksymalna szybkosc
	czas_t = 500;
	czas_p = 500;

	//wyzerowanie licznika
	cnt_p = 0;
	cnt_l = 0;

	//ustawienie portow silnik prawy
	DDRD |= c1|c2|c3|c4;
	//ustawienie portow silnik lewy
	DDRD |= l1|l2|l3|l4;
	DDRC &= ~(PIN_KIERUNEK);
	PORTC |= (PIN_KIERUNEK);

	//lcd_init();

	while(1) {
		//if()
		//if(test<500) {
		//polkrokowy_prawy_przod(cnt_p);
		//polkrokowy_lewy_tyl(cnt_l);
		//przod(cnt_p,cnt_l);
		//_delay_ms(2000);
		//tyl(cnt_p,cnt_l);
		//_delay_ms(2000);
			//test++;
		//} else// if(test <1000) {
			//polkrokowy_prawy_hamuj(cnt_p,2000);
			//test++;
		//} else test = 0;



		/*for(i=2000;i>=500;i-=10) {
			przod(cnt_p,cnt_l,i);
		}
		for(i=500;i<=2000;i+=10) {
			przod(cnt_p,cnt_l,i);
		}*/
		if(KIERUNEK) {
			//tyl(cnt_p,cnt_l,(2*pomiar(2)+476));
			tyl(cnt_p,cnt_l,(550+pomiar(2)));
		} else {
			//przod(cnt_p,cnt_l,(2*pomiar(2)+476));
			przod(cnt_p,cnt_l,(550+pomiar(2)));
		}
		//lcd_cls();
		//lcd_int((2*pomiar(1)+476));


		/*
		int i;
		for(i=0;i<20;i++) {
			przod(cnt_p,cnt_l);
		}
		for(i=0;i<20;i++) {
			tyl(cnt_p,cnt_l);
		}*/
	}
}

uint16_t pomiar(uint8_t kanal) {
	ADMUX = (ADMUX & 0xF8) | kanal;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADCW;
}

void pelnokrokowy_prawy_przod(int pozycja) {
	/*switch(pozycja%4)
		{
		case 0:
		if(cnt_p == 200) cnt_p = 0;
		krok1p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		case 1:
		krok2p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		case 2:
		krok3p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		case 3:
		krok4p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		if(cnt_p == 400) {cnt_p = 0;}*/
	krok1mp;
	czekaj(czas_m);
	krok2mp;
	czekaj(czas_m);
	krok3mp;
	czekaj(czas_m);
	krok4mp;
	czekaj(czas_m);
}

void pelnokrokowy_lewy_przod(int pozycja) {
	/*switch(pozycja%4)
		{
		case 0:
		if(cnt_p == 200) cnt_p = 0;
		krok1p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		case 1:
		krok2p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		case 2:
		krok3p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		case 3:
		krok4p;
		PORTC ^= (1<<PC5);
		cnt_p++;
		if(cnt_p == 200) cnt_p = 0;
			czekaj(czas_m);
		if(cnt_p == 400) {cnt_p = 0;}*/
	krok4ml;
	czekaj(czas_m);
	krok3ml;
	czekaj(czas_m);
	krok2ml;
	czekaj(czas_m);
	krok1ml;
	czekaj(czas_m);
}

void polkrokowy_prawy_przod(int pozycja,int czas_m) {
	switch(pozycja%8)
		{
		case 0:
		if(cnt_p == 400) {cnt_p = 0;}
		krok1p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		case 1:
		krok2p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		case 2:
		krok3p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		case 3:
		krok4p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		case 4:
		krok5p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		case 5:
		krok6p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		case 6:
		krok7p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		case 7:
		krok8p;
		cnt_p++;
		if(cnt_p == 400) {cnt_p = 0;}
			czekaj(czas_m);
		}
		if(cnt_p == 400) {cnt_p = 0;}
}

void polkrokowy_lewy_przod(int pozycja, int czas_m) {
	switch(pozycja%8)
		{
		case 0:
		if(cnt_l == 400) {cnt_l = 0;}
		krok1l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		case 1:
		krok2l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		case 2:
		krok3l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		case 3:
		krok4l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		case 4:
		krok5l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		case 5:
		krok6l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		case 6:
		krok7l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		case 7:
		krok8l;
		cnt_l++;
		if(cnt_l == 400) {cnt_l = 0;}
			czekaj(czas_m);
		}
		if(cnt_p == 400) {cnt_p = 0;}
}

void polkrokowy_prawy_tyl(int pozycja, int czas_m) {
	switch(pozycja%8)
			{
			case 0:
			if(cnt_p == 0) {cnt_p = 400;}
			krok1p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			case 7:
			krok8p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			case 6:
			krok7p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			case 5:
			krok6p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			case 4:
			krok5p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			case 3:
			krok4p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			case 2:
			krok3p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			case 1:
			krok2p;
			cnt_p--;
			if(cnt_p == 0) {cnt_p = 400;}
				czekaj(czas_m);
			}
			if(cnt_p == 0) {cnt_p = 400;}
	}

void polkrokowy_lewy_tyl(int pozycja, int czas_m) {
	switch(pozycja%8)
			{
			case 0:
			if(cnt_l == 0) {cnt_l = 400;}
			krok1l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			case 7:
			krok8l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			case 6:
			krok7l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			case 5:
			krok6l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			case 4:
			krok5l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			case 3:
			krok4l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			case 2:
			krok3l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			case 1:
			krok2l;
			cnt_l--;
			if(cnt_l == 0) {cnt_l = 400;}
				czekaj(czas_m);
			}
			if(cnt_l == 0) {cnt_l = 400;}
	}

void polkrokowy_prawy_hamuj(int pozycja, int czas) {
	switch(pozycja%8)
			{
			case 0:
				krok1p;
				czekaj(czas);
				break;
			case 1:
				krok2p;
				czekaj(czas);
				break;
			case 2:
				krok3p;
				czekaj(czas);
				break;
			case 3:
				krok4p;
				czekaj(czas);
				break;
			case 4:
				krok5p;
				czekaj(czas);
				break;
			case 5:
				krok6p;
				czekaj(czas);
				break;
			case 6:
				krok7p;
				czekaj(czas);
				break;
			case 7:
				krok8p;
				czekaj(czas);
				break;
}
}

void polkrokowy_lewy_hamuj(int pozycja, int czas) {
	switch(pozycja%8)
			{
			case 0:
				krok1l;
				czekaj(czas);
				break;
			case 1:
				krok2l;
				czekaj(czas);
				break;
			case 2:
				krok3l;
				czekaj(czas);
				break;
			case 3:
				krok4l;
				czekaj(czas);
				break;
			case 4:
				krok5l;
				czekaj(czas);
				break;
			case 5:
				krok6l;
				czekaj(czas);
				break;
			case 6:
				krok7l;
				czekaj(czas);
				break;
			case 7:
				krok8l;
				czekaj(czas);
				break;
}
}

void przod(int pozycja_p,int pozycja_l,int czas) {
	polkrokowy_prawy_przod(pozycja_p,czas);
	polkrokowy_lewy_tyl(pozycja_l,czas);
}
void tyl(int pozycja_p,int pozycja_l, int czas) {
	polkrokowy_prawy_tyl(pozycja_p,czas);
	polkrokowy_lewy_przod(pozycja_l,czas);
}
