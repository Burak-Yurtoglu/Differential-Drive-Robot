/*
 * RCC.h
 *
 *  Created on: Aug 27, 2021
 *      Author: burak
 */

#ifndef RCC_H_
#define RCC_H_

void SysClockConfig(void){
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************

	1. ENABLE HSE and wait for the HSE to become Ready
	2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	3. Configure the FLASH PREFETCH and the LATENCY Related Settings
	4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	5. Configure the MAIN PLL
	6. Enable the PLL and wait for it to become ready
	7. Select the Clock Source and wait for it to be set

	 ********************************************************/
	#define PLL_N 336
	#define PLL_M 8
	#define PLL_P 0 //PLLP2
	#define PLL_Q 7

	RCC->CR |= (0x1 << 16);		    //ENABLE HSE
	while(!(RCC->CR & 0x1 << 17));	//wait for the HSE to become Ready

	RCC->APB1ENR |= (0x1 << 28);		//Set the POWER ENABLE CLOCK
	PWR->CR |= (0x1 << 14);				//Set the VOLTAGE REGULATOR

	FLASH->ACR |= (0x5 << 0) | (0x1 << 8) | (0x1 << 9) | (0x1 << 10) ; //Configure the FLASH PREFETCH and the LATENCY Related Settings


	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;	// AHB PRE
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;	// APB1 PRE
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;	// APB2 PRE


	RCC->PLLCFGR |= (0x1 << 4) | (21 << 10) | (PLL_P << 16) | (PLL_Q << 24) | (RCC_PLLCFGR_PLLSRC_HSE);//Configure the MAIN PLL
	RCC->PLLCFGR &= ~(0x1 << 13);

	RCC->CR |= (0x1 << 24);				 //Enable the PLL
	while(!(RCC -> CR & RCC_CR_PLLRDY)); //wait for it to become ready

	RCC->CFGR |= (0x2 << 0);			//Select the Clock Source
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	//wait for it to be set

}



#endif /* RCC_H_ */
