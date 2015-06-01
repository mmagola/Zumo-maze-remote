#include "zumo_buzzer.h"


void zumo_buzzer_init(void){
	
	//pta12 - buzzer - zworka 328P
	SIM->SCGC5 |=  SIM_SCGC5_PORTA_MASK; 
  PORTA->PCR[12] = PORT_PCR_MUX(1);
	PTA->PDDR |= (1u<<12);
	PTA->PCOR |= (1u<<12);
}


void zb_WRC_start(void){
	
	uint32_t x;
	uint32_t t;
	
	for(x=0; x<500; x++){
		PTA->PTOR |= (1u<<12);
		for(t=0; t<10000; t++){}
	}
	PTA->PCOR |= (1u<<12);
	for(x=0; x<500; x++){
		for(t=0; t<10000; t++){}
	}
	
	for(x=0; x<500; x++){
		PTA->PTOR |= (1u<<12);
		for(t=0; t<10000; t++){}
	}
	PTA->PCOR |= (1u<<12);
	for(x=0; x<500; x++){
		for(t=0; t<10000; t++){}
	}
	
	for(x=0; x<500; x++){
		PTA->PTOR |= (1u<<12);
		for(t=0; t<10000; t++){}
	}
	PTA->PCOR |= (1u<<12);
	for(x=0; x<500; x++){
		for(t=0; t<10000; t++){}
	}
	
	for(x=0; x<1200; x++){
		PTA->PTOR |= (1u<<12);
		for(t=0; t<5000; t++){}
	}
	PTA->PCOR |= (1u<<12);
}

void zb_doubleBeep(void){
	
	uint32_t x;
	uint32_t t;
	
	for(x=0; x<500; x++){
		PTA->PTOR |= (1u<<12);
		for(t=0; t<1500; t++);
	}
	
	PTA->PCOR |= (1u<<12);
	for(x=0; x<500; x++){
		for(t=0; t<1500; t++){}
	}
	
	for(x=0; x<500; x++){
		PTA->PTOR |= (1u<<12);
		for(t=0; t<1500; t++){}
	}
	PTA->PCOR |= (1u<<12);
}
