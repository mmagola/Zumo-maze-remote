#include "zumo_button.h"
#include "MKL46Z4.h"



void zumo_button_init(void){
	
	SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; 					/* Enable clock for port D */
	
	PORTD->PCR[Z_BUTTON] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[Z_BUTTON] |= PORT_PCR_MUX(1);      	/* Pin PTC3 is GPIO */
	
	PORTD->PCR[Z_BUTTON] |=  PORT_PCR_PE_MASK |	PORT_PCR_PS_MASK;
}

uint8_t zumo_button_pressed(){

	if( FPTD->PDIR & (1UL<<Z_BUTTON) ) return 0;
	else return 1;
}
