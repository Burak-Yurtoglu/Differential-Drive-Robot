
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "stm32f407xx.h"
#include "RCC.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif



void GpioConfig(void){

	RCC->AHB1ENR |= (0x1 << 3) | (0x1 << 0) |  (0x1 << 1);// PORT A B D Enable

	// Motor direction
	GPIOA->MODER |= (0x1 << 4) | (0x1 << 6) | (0x1 << 8) | (0x1 << 10) | (0x1 << 12);
	GPIOA->OTYPER = 0;
	GPIOA->OSPEEDR |= (0x2 << 4) | (0x2 << 6) | (0x2 << 8) | (0x2 << 10) | (0x1 << 12);

	// Encoder pin config

	GPIOA->MODER &= ~((0x3 << 0) | (0x3 << 2));
	GPIOA->PUPDR |= (0x1 << 0) | (0x1 << 2);

	//PWM motor control
	GPIOD->MODER |= (0x2 << 24) | (0x2 << 26);
	GPIOD->OTYPER = 0;
	GPIOD->OSPEEDR |= (0x3 << 24) | (0x3 << 26);
	GPIOD->PUPDR = 0;
	GPIOD->AFR[1] |= (0x2 << 16) | (0x2 << 20);


}

void Interrupt_Config(void){

	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************

		1. Enable the SYSCFG/AFIO bit in RCC register
		2. Configure the EXTI configuration Register in the SYSCFG/AFIO
		3. Disable the EXTI Mask using Interrupt Mask Register (IMR)
		4. Configure the Rising Edge / Falling Edge Trigger
		5. Set the Interrupt Priority
		6. Enable the interrupt

		********************************************************/

	//1. Enable the SYSCFG/AFIO bit in RCC register

	RCC->APB2ENR |= (0x1 << 14);

	//2. Configure the EXTI configuration Register in the SYSCFG/AFIO

	SYSCFG->EXTICR[0] = 0;

	//3. Disable the EXTI Mask using Interrupt Mask Register (IMR)

	EXTI->IMR |= (0x1 << 0)|(0x1 << 1);

	//4. Configure the Rising Edge / Falling Edge Trigger

	EXTI->FTSR |= (0x1 << 0) |(0x1 << 1);
	EXTI->RTSR |= 0;


	//5. Set the Interrupt Priority

	NVIC_SetPriority(EXTI0_IRQn , 0 );
	NVIC_EnableIRQ(EXTI0_IRQn);

	NVIC_SetPriority(EXTI1_IRQn , 0);
	NVIC_EnableIRQ(EXTI1_IRQn);
}

void TIM4_Config(void){


	RCC->APB1ENR |= (0x1 << 2);//1. ENABLE the Timer clock


	TIM4->CCMR1 |= (0x6 << 4)|(0x1 << 3); // PWM1 mode selected, Preload EN
	TIM4->CCMR1 |= (0x6 << 12)|(0x1 << 11);
	TIM4->CR1 |= (0x1 << 7); // ARPE set
	TIM4->CCER |= (0x1 << 0) | (0x1 << 4 ); // CC1 (PD12) configured as output
	TIM4->EGR |=  (0x1 << 0); // update flag set

	TIM4->ARR = 1000;
	TIM4->PSC = 84-1;//84Mhz / 84 = 1 Mhz ~~ 1uS delay
	TIM4->CCR1 = 0 ;
	TIM4->CCR2 = 0 ;


	TIM4->CR1 |= (0x1 << 0); // Counter enabled



}


void TIM6_Config(void){
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************

				1. ENABLE the TIM6 CLOCK
				2. Set the prescalar and ARR
				3. ENABLE the Timer, and wait for the update Flag set

				********************************************************/


	//1. ENABLE the Timer clock

	RCC->APB1ENR|= (0x1 << 4);

	//2. Set the prescalar and ARR

	TIM6->PSC = 84-1; //84Mhz / 84 = 1 Mhz ~~ 1uS delay
	TIM6->ARR = 0x2710 ; // MAX ARR value
	TIM6 ->DIER |= (0x1 << 0); // tim6 interrup en
	TIM6->CR1 |= (0x1 << 2);

	NVIC_SetPriority(TIM6_DAC_IRQn , 1 );
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	//3. ENABLE the Timer, and wait for the update Flag set

	TIM6->CR1 |= (0x1 << 0);
	while(!(TIM6->SR & (0x1<<0)));

}
void delay_us(uint16_t us){

	TIM6->CNT = 0;
	while(TIM6->CNT < us);

}

void delay_ms(uint16_t ms){

	for(uint16_t i = 0; i<ms ; ++i){
		delay_us(1000);
	}

}

float Robot_PID_Q(float Ref,float actual);
void Robot_Ref(int32_t X,int32_t Y);
void MotorL(uint16_t PWML);
void MotorR(uint16_t PWMR);
void MotorStop(void);
void MotorStart(void);


uint8_t count=0;
uint8_t N=16;

float Radius = 23;
float Lenght = 150;
float PI = 3.141;

int Enc[2]={0,0};
int Old_Enc[2]={0,0};

float D_center=0;
float Distance[2]={0,0};

int32_t X_axis= 0;
int32_t Y_axis= 0;
int32_t X_Y_Ref[2][8]={
		{300,300,0,0,300,300,0,0},
		{0,300,300,600,600,900,900,0}
};

int point=0;


float Q=0;
float Q_ref=0;
float D_ref=0;

float error[3]={0,0,0};
float error_D = 0;
float error_I = 0;
int Ts=2;

float Vmot[2]={0,0};
float V=1000;
float W=0;

size_t flag=0;
int main(void)
{
	SysClockConfig();
	TIM6_Config();
	TIM4_Config();
	GpioConfig();
	Interrupt_Config();
	MotorStop();
    /* Loop forever */
	for(;;){

		Robot_Ref(X_Y_Ref[0][point],X_Y_Ref[1][point]);

	    if(count >= Ts){

	    	/* Eski encoder değeri ile yeni değerin
	    	farkı bulunurak bir zaman periyodundaki dönüş
			hesplanır*/
	    	Enc[0] = Enc[0] - Old_Enc[0];
	    	Old_Enc[0] = Enc[0];

	    	Enc[1] = Enc[1] - Old_Enc[1];
	    	Old_Enc[1] = Enc[1];

	    	// Tekerlerin aldıkları yol
	    	Distance[0]= (2*PI*Radius*Enc[0])/N;
	        Distance[1]= (2*PI*Radius*Enc[1])/N;

	        // Merkezin aldığı yol
	        D_center = (Distance[0]+ Distance[1])/2;

	        // Robotu yeni durumu hesaplanır
	       	Q += (Distance[0]- Distance[1])/(Lenght);
	       	X_axis += D_center * cos(Q);
	       	Y_axis += D_center * sin(Q);

	       	//Açısal dönme oranı PID ile hesaplanır
	       	W=Robot_PID_Q(Q_ref, Q);


	       	Vmot[0]= 8*(2*V+W*Lenght)/(2*Radius);
	       	Vmot[1]= 8*(2*V-W*Lenght)/(2*Radius);

	       	if(Vmot[0] > 1000)Vmot[0]=1000;
	       	if(Vmot[1] > 1000)Vmot[1]=1000;



	       	if(D_ref <= 10){
	       		point+=1;
	       		if(point >= 8){
	       			MotorStop();
	       			flag=1;
	       		}
	       	}
	       	if(!flag){
	       		MotorStart();
	       		MotorL(Vmot[1]);
	       		MotorR(Vmot[0]);
	       	}
	       	count = 0;
	    }
	}}


float Robot_PID_Q(float Ref,float actual){


	float kp=30;
	float ki=0.007;
	float kd=8;

	error[0] = Ref-actual;
	error[0] = atan2(sin(error[0]),cos(error[0]));

	error_D =(error[0] - error[1])/Ts	;
	error_I +=(Ts/2)*(error[0] + 2*error[1] + error[2]);

	error[2]=error[1];
	error[1]=error[0];


	return kp*error[0]+kd*error_D+ki*error_I;

}

void Robot_Ref(int32_t X,int32_t Y){

	Q_ref = atan2((Y-Y_axis),(X-X_axis));
	D_ref =sqrt(pow(Y-Y_axis,2)+pow(X-X_axis,2));

}
void MotorL(uint16_t PWML){

	TIM4->CCR1 = PWML;
	GPIOA->BSRR |= (0x1 << 2);
	GPIOA->BSRR &= ~(0x1 << 3);

}
void MotorR(uint16_t PWMR){

	TIM4->CCR2 = PWMR;

	GPIOA->BSRR |= (0x1 << 4);
	GPIOA->BSRR &= ~(0x1 << 5);


}

void MotorStop(void){
	GPIOA->BSRR |= (0x1 << 22);
}
void MotorStart(void){

	GPIOA->BSRR |= (0x1 << 6);
}



void EXTI0_IRQHandler(void){


	Enc[1]++;
	EXTI->PR |=(0x1 << 0);

}

void EXTI1_IRQHandler(void){



	Enc[0]++;
	EXTI->PR |=(0x1 << 1);

}


void TIM6_DAC_IRQHandler(void){

	count++;
	TIM6->SR &= ~(0x1 << 0);
}
