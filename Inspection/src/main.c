#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm

#include "matrix.h"

#include "mpu6050.h"
//
//#define UART_BAUD_RATE 57600
//#include "uart/uart.h"
#include "lcd44780.h"

volatile uint8_t pwm1;
volatile int licznik;

int main(void) {

	//programowy pwm
	DDRC |= (1<<PC0)|(1<<PC1);
	PORTC |= (1<<PC0);
	PORTC |= (1<<PC1);

	TCCR2B |= (1<<WGM21);	// tryb  CTC
	TCCR2B |= (1<<CS20);		// preskaler = 1
	OCR2B = 199;				// dodatkowy podzia³ czêsttotliwoœci przez 200
	TIMSK2 |= (1<<OCIE2B);

	/*TCCR2 |= (1<<WGM21);	// tryb  CTC
	TCCR2 |= (1<<CS20);		// preskaler = 1
	OCR2 = 199;				// dodatkowy podzia³ czêsttotliwoœci przez 200
	TIMSK |= (1<<OCIE2);*/

	#if MPU6050_GETATTITUDE == 0
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    double axg = 0;
    double ayg = 0;
    double azg = 0;
    double gxds = 0;
    double gyds = 0;
    double gzds = 0;
	#endif

	#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
    //long *ptr = 0;
    double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;
	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;
	#endif

	//uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	sei();

	mpu6050_init();
	_delay_ms(50);


	int maxa=0,maxg=0,aax,pom;

	//init mpu6050 dmp processor
	#if MPU6050_GETATTITUDE == 2
	mpu6050_dmpInitialize();
	mpu6050_dmpEnable();
	_delay_ms(10);
	#endif
	DDRD |= (1<<PD0);
	PORTD |= (1<<PD0);
	lcd_init();
	int licz=1,suma=0,_ax,fi,bak;
	int i;
	int ii,jj;
	int sprawdz;

	//pid
			/*float wzmocnienieP=10;		//Wzmocnienie
			float stalaI=0.4;		//Sta³a czasowa ca³kowania
			float stalaD=12;		//Sta³a czasowa ró¿niczkowania
			*/
			float wzmocnienieP=14;		//Wzmocnienie
			float stalaI=0;		//Sta³a czasowa ca³kowania
			float stalaD=0;
			float czas=0.15;	//Czas zmian wielkoœci
			int predkosc=127;		//Prêdkoœæ silników
			int predkosc_k=127;
			int sterowanie,uchyb,uchyb_pop=0;
			int calka=0;
			int lqr;

			//KF
			/*Matrix *x_post = matrix_alloc(4,1);
			for(ii=0;ii<4;ii++) {
				for(jj=0;jj<1;jj++) {
					x_post->matrix_entry[ii][jj]=0;
				}
			}
			Matrix *P_post = matrix_alloc(4,4);
			for(ii=0;ii<4;ii++) {
				for(jj=0;jj<4;jj++) {
					P_post->matrix_entry[ii][jj]=1;
				}
			}
			Matrix *V = matrix_alloc(4,4);
			for(ii=0;ii<4;ii++) {
				for(jj=0;jj<4;jj++) {
					if(ii==jj) V->matrix_entry[ii][ii]=0.8;
					else V->matrix_entry[ii][jj]=0;
				}
			}

			Matrix *W = matrix_alloc(2,2);
			W->matrix_entry[0][0]=0.02;
			W->matrix_entry[0][1]=0;
			W->matrix_entry[1][0]=0;
			W->matrix_entry[1][1]=4;

			//LQR

			Matrix *K_C = matrix_alloc(1,4);
			K_C->matrix_entry[0][0]=80;
			K_C->matrix_entry[0][1]=28.6388;
			K_C->matrix_entry[0][2]=-63.7184;
			K_C->matrix_entry[0][3]=-5.6401;

*/
	for(;;) {
		lcd_cls();
		#if MPU6050_GETATTITUDE == 0
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		#endif

		#if MPU6050_GETATTITUDE == 1
		mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
		mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		_delay_ms(10);
		#endif

		#if MPU6050_GETATTITUDE == 2
		if(mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
			mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
		}
		_delay_ms(10);
		#endif

		#if MPU6050_GETATTITUDE == 0
		//char itmp[10];
		/*ltoa(ax, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(ay, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(az, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gx, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gy, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gz, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		uart_puts("\r\n");*/
		/*lcd_cls();
		lcd_str("a=(");
		lcd_locate(0,3);
		lcd_int(axg);
		lcd_locate(0,6);
		lcd_str(",");
		lcd_locate(0,7);
		lcd_int(ayg);
		lcd_locate(0,10);
		lcd_str(",");
		lcd_locate(0,11);
		lcd_int(azg);
		lcd_locate(0,14);
		lcd_str(")");

		lcd_locate(1,0);
		lcd_str("g=(");
		lcd_locate(1,3);
		lcd_int(gxds);
		lcd_locate(1,6);
		lcd_str(",");
		lcd_locate(1,7);
		lcd_int(gyds);
		lcd_locate(1,10);
		lcd_str(",");
		lcd_locate(1,11);
		lcd_int(gzds);
		lcd_locate(1,14);
		lcd_str(")");
		_delay_ms(100);*/

		/*if(ax<0) ax*=-1;
		if(gx<0) gx*=-1;
		if(ax>maxa) maxa=ax;
		if(gx>maxg) maxg=gx;
		lcd_int(maxa);
		lcd_locate(1,0);
		lcd_int(maxg);
		_delay_ms(100);*/

		/*
		lcd_int(ax);
		lcd_locate(1,0);
		lcd_int(gx);
		_delay_ms(100);*/

		//REGULATOR PROPORCJONALNY
		/*int K=26;
		ax/=100;
		if(ax<0) ax *= -1;
		pom=-K*ax+255;
		if(pom<0) pom = 0;
		pwm1=pom;
		if(ax>=0) PORTC &= ~(1<<PC1);
		else PORTC |= (1<<PC1);
		licz++;*/
		//if((licz%10000)==0)
		//{
			/*lcd_cls();
			lcd_int(pwm1);
			_delay_ms(1000);
		//}
		 *

		*/
//############################33
		ax /= 100;
		suma+=ax;
		licz++;
		if(licz == 20) {
			suma /= 20;
			fi=suma;
			//lcd_cls();
			//lcd_int(fi);
			//_delay_ms(500);

			//
			if(fi>=0) {
				PORTC &= ~(1<<PC1);
			}
			if(fi<0) {
				PORTC |= (1<<PC1);
				fi *= -1;
			}
			//REGULATOR PROPORCJONALNY
			//int K=80;
			//pom=K*fi;
			//if(pom>255) pom = 255;
			//pwm1=pom;


		//PID

		if(fi<0) {
			uchyb = -1*fi;
		} else {
			uchyb = fi;
		}
		calka += stalaI*fi;
		sterowanie = (int)(wzmocnienieP*fi)/* - (int)(stalaD*((fi - uchyb_pop)))*/ + (int)(calka);
		sprawdz = predkosc - sterowanie;
		if(sterowanie>255) sterowanie = 255;
		if(sterowanie<0) sterowanie = 0-sterowanie;
		if(sterowanie<-255) sterowanie = 255;

		//lcd_int(sterowanie);
		//_delay_ms(1000);

		pwm1=sterowanie;
		uchyb_pop=fi;

		suma=0;
		licz=1;

		}

		//LQR
		//Wypelnianie macierzy A
		/*Matrix *A;
		A = matrix_alloc(4,4);

		for(ii=0;ii<4;ii++) {
			for(jj=0;jj<4;jj++) {
				A->matrix_entry[ii][jj]=0;
			}
		}
		A->matrix_entry[0][1]=1; 		A->matrix_entry[2][3]=1;
		A->matrix_entry[1][1]=-0.1192; 	A->matrix_entry[1][2]=6.7359;
		A->matrix_entry[3][1]=-1.2764; 	A->matrix_entry[3][2]=177.1575;

		//Wypelnianie macierzy B
		Matrix *B;
		B = matrix_alloc(4,1);

		for(ii=0;ii<4;ii++) {
			B->matrix_entry[ii][0]=0;
		}
		B->matrix_entry[1][0]=1.1923; 		B->matrix_entry[3][0]=12.7640;

		//Wypelnianie macierzy C
		Matrix *C;
		C = matrix_alloc(2,4);

		for(ii=0;ii<2;ii++) {
			for(jj=0;jj<4;jj++) {
				C->matrix_entry[ii][jj]=0;
			}
		}
		C->matrix_entry[0][0]=1; 		C->matrix_entry[1][2]=1;

		//u=pwm1

		Matrix *x;
		x = matrix_alloc(4,1);
		x->matrix_entry[0][0]=0.000628*licznik; //m #################################
		//x->matrix_entry[1][0]=6.28/(-0.0588*pwm1+20); //m/s
		x->matrix_entry[1][0]=3.1416/(5000*pwm1); //m/s
		x->matrix_entry[2][0]=(ax*9)/1600; //16000 - 90 stopni
		x->matrix_entry[3][0]=gy; //

		Matrix *y = matrix_alloc(2,1);
		y->matrix_entry[0][0]=x->matrix_entry[0][0];
		y->matrix_entry[1][0]=x->matrix_entry[2][0];

		//lcd_cls();
		//lcd_int((int)x->matrix_entry[2][0]);
		//_delay_ms(500);

		//FILTR KALMANA



		Matrix *Ax = matrix_alloc(4,1);
		Matrix *Bu = matrix_alloc(4,1);
		Matrix *x_pri = matrix_alloc(4,1);
		Matrix *AP = matrix_alloc(4,4);
		Matrix *AT = matrix_alloc(4,4);
		Matrix *APAT = matrix_alloc(4,4);
		Matrix *P_pri = matrix_alloc(4,4);
		Matrix *eps = matrix_alloc(2,1);
		Matrix *CX = matrix_alloc(2,1);

		      // x(t+1|t) = Ax(t|t) + Bu(t)
		      Ax=matrix_multiply(A, x_post);
		      Bu->matrix_entry[0][0]=0;
		      Bu->matrix_entry[1][0]=pwm1*B->matrix_entry[1][0];
		      Bu->matrix_entry[2][0]=0;
		      Bu->matrix_entry[3][0]=pwm1*B->matrix_entry[3][0];
		      matrix_add(x_pri, Ax, Bu);

		      // P(t+1|t) = AP(t|t)A^T + V
		      AP=matrix_multiply(A, P_post);
		      AT=matrix_transpose(A);
		      APAT=matrix_multiply(AP, AT);
		      matrix_add(P_pri, APAT, V);

		      // eps(t) = y(t) - Cx(t|t-1)
		      CX=matrix_multiply(C, x_pri);
		      matrix_subtract(eps,y,CX);

		  Matrix *CP = matrix_alloc(2,4);
		  Matrix *CPCT = matrix_alloc(2,2);
		  Matrix *CT = matrix_alloc(4,2);
		  Matrix *S = matrix_alloc(2,2);
		  Matrix *PCT = matrix_alloc(4,2);
		  //Matrix *S1 = matrix_alloc(2,2);
		  Matrix *K = matrix_alloc(4,2);
		  Matrix *Keps = matrix_alloc(4,1);
		  Matrix *KS = matrix_alloc(4,2);
		  Matrix *KT = matrix_alloc(2,4);
		  Matrix *KSKT = matrix_alloc(4,4);

		      // S(t) = CP(t|t-1)C^T + W
		      CP=matrix_multiply(C, P_pri);
		      CT=matrix_transpose(C);
		      CPCT=matrix_multiply(CP, CT);
		      matrix_add(S, CPCT, W);

		      // K(t) = P(t|t-1)C^TS(t)^-1
		      PCT=matrix_multiply(P_pri, CT);
		      matrix_invert(S);
		      K=matrix_multiply(PCT, S); //S^-1

		      // x(t|t) = x(t|t-1) + K(t)eps(t)
		      Keps=matrix_multiply(K, eps);
		      matrix_add(x_post, x_pri, Keps);

		      // P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T
		      matrix_invert(S);
		      KS=matrix_multiply(K, S);//S
		      KT=matrix_transpose(K);
		      KSKT=matrix_multiply(KS, KT);
		      matrix_subtract(P_post, P_pri, KSKT);

		      //LQR

		Matrix *KX = matrix_alloc(1,1);
		KX=matrix_multiply(K, x_post);

		lqr=(int)(KX->matrix_entry[0][0]);
		sprawdz=predkosc + lqr;
		if(sprawdz>255) sprawdz = 255;
		if(sprawdz<0) sprawdz = 0;
		pwm1=sprawdz;


		pwm1 = predkosc
*/

		//############

		//matrix_t *A = make_matrix( 4, 4 );
		//put_entry_matrix( eqs, if1, if1, dx );


		/*lcd_cls();
		lcd_int(fi);
		lcd_locate(0,7);
		lcd_int(pwm1);
		_delay_ms(500);*/


//#################################
		/*for(i=255;i>=0;i--) {
			pwm1=i;
			_delay_ms(30);
			if((i%10)==0) {
				lcd_cls();
				lcd_int(pwm1);
			}
		}
		for(i=0;i<=255;i++) {
			pwm1=i;
			_delay_ms(30);
			if((i%10)==0) {
				lcd_cls();
				lcd_int(pwm1);
			}
		}*/

		#endif

		#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2

		//quaternion
		/*ptr = (long *)(&qw);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&qx);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&qy);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&qz);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);

		//roll pitch yaw
		ptr = (long *)(&roll);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&pitch);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&yaw);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);

		uart_putc('\n');*/
		//lcd_int((int)qw);
		//_delay_ms(1000);
		#endif

	}

}

/*float calka(int e, float t) {
	int suma=0;
	float dt=t/100;

	return suma;
}*/

ISR( TIMER2_COMPB_vect )
{
	static uint8_t cnt; // definicja naszego licznika PWM
	// bezpoœrednie sterowanie wyjœciami kana³ów PWM
	if(cnt>=pwm1) PORTC |= (1<<PC0); else PORTC &= ~(1<<PC0);
	cnt++;	// zwiêkszanie licznika o 1
}
