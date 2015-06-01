#include "motorDriver.h"

#define V_MOD 1023
#define LEFT (1ul<<13)
#define RIGHT (1ul<<9)

volatile uint8_t CH2_CnV_Busy = 0;
volatile uint8_t CH4_CnV_Busy = 0;

void TPM0_IRQHandler(void){
	
	if( TPM0->SC & TPM_SC_TOF_MASK ){
		
		if( CH2_CnV_Busy ) CH2_CnV_Busy = 0;
		if( CH4_CnV_Busy ) CH4_CnV_Busy = 0;
		
		TPM0->SC |= TPM_SC_TOF_MASK;
	}
}

void motorDriverInit(void){

	// CLOCK_SETUP 1
	// 1 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
  //       Reference clock source for MCG module is an external crystal 8MHz
  //       Core clock = 48MHz, BusClock = 24MHz
	
	//
	SIM -> SCGC5 |= SIM_SCGC5_PORTA_MASK
	              | SIM_SCGC5_PORTC_MASK 
	              | SIM_SCGC5_PORTD_MASK;
	
	//
	SIM -> SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	//
	//PORTA ->PCR[6] |= PORT_PCR_MUX(3); // TPM0_CH3 - encoder
	PORTA ->PCR[13] |= PORT_PCR_MUX(1); // PHASE - Left
	PORTC ->PCR[9] |= PORT_PCR_MUX(1); // PHASE - Right
	PORTD ->PCR[2] |= PORT_PCR_MUX(4); // TPM0_CH2 - PWM - Right
	PORTD ->PCR[4] |= PORT_PCR_MUX(4); // TPM0_CH4 - PWM - Left
	//PORTD ->PCR[5] |= PORT_PCR_MUX(4); //TPM0_CH5 - encoder / to tez dioda zielona
	
	// OUTPUT pin
	PTA->PDDR |= (1ul<<13);
	PTC->PDDR |= (1ul<<9);
	
	
	
	////////////////////// PWM /////////////////////////////////
	//select source reference TMP0

	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // ?set 'MCGFLLCLK clock or MCGPLLCLK/2'
	
	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// set "MCGPLLCLK clock with  fixed divide by two"
	
	// set "up-counting"
	TPM0->SC &= ~TPM_SC_CPWMS_MASK; // default set
	
	// divide by 1
	TPM0->SC &= ~TPM_SC_PS_MASK; // the same TPM_SC_PS(0)
	
	// clear counter
	TPM0->CNT = 0x00; 
	
	// set MOD for PWM period equal 1023 ( 10 bit)
	TPM0->MOD = V_MOD;
	
	//////////CHANNEL ENGINE ////////////////////////////////
	//Right engine
	// set TPM0 channel 2 - "Edge-aligned PWM High-true pulses"
	TPM0->CONTROLS[2].CnSC |= TPM_CnSC_MSB_MASK |	
													  TPM_CnSC_ELSB_MASK;
	// Default value for Right engine
	while( CH2_CnV_Busy );
	TPM0->CONTROLS[2].CnV = 0; // STOP
	CH2_CnV_Busy = 1;

	//Left engine
	// set TPM0 channel 4 - "Edge-aligned PWM High-true pulses"
	TPM0->CONTROLS[4].CnSC |= TPM_CnSC_MSB_MASK |	
													  TPM_CnSC_ELSB_MASK;
	// Default value for Left engine
	while( CH4_CnV_Busy );
	TPM0->CONTROLS[4].CnV = 0; // STOP
	CH4_CnV_Busy = 1;
	
	TPM0->SC |= TPM_SC_TOIE_MASK;
	NVIC_ClearPendingIRQ(TPM0_IRQn);				/* Clear NVIC any pending interrupts on PORTC_A */
	NVIC_EnableIRQ(TPM0_IRQn);
	
	
	// enable counter
	TPM0->SC |= TPM_SC_CMOD(1);
}


void driveForwardLeftTrack( uint16_t predkosc ){

	while( CH4_CnV_Busy );
	PTA->PCOR |= LEFT; // clear , set 0 mean forward
	TPM0->CONTROLS[4].CnV = V_MOD * predkosc/100;
	CH4_CnV_Busy = 1;
}

void driveForwardRightTrack( uint16_t predkosc ){
	
	while( CH2_CnV_Busy );
	
	PTC->PCOR |= RIGHT;
	TPM0->CONTROLS[2].CnV = V_MOD * predkosc/100;
	CH2_CnV_Busy = 1;	
}

void driveStopLeft(void){

	while( CH4_CnV_Busy );
	TPM0->CONTROLS[4].CnV = 0; // stop LEFT
	CH4_CnV_Busy = 1;	
}

void driveStopRight(void){

	while( CH2_CnV_Busy );
	TPM0->CONTROLS[2].CnV = 0; // stop RIGHT
	CH2_CnV_Busy = 1;	
}

void driveStop(void){
	
	while( CH2_CnV_Busy || CH4_CnV_Busy );
	TPM0->CONTROLS[2].CnV = 0; // stop RIGHT
	TPM0->CONTROLS[4].CnV = 0; // stop LEFT
	CH2_CnV_Busy = 1;	
	CH4_CnV_Busy = 1;
}


void driveReverseLeftTrack( uint16_t predkosc ){

	while( CH4_CnV_Busy );
	PTA->PSOR |= LEFT; // clear , set 0 mean forward
	TPM0->CONTROLS[4].CnV = V_MOD * predkosc/100;
	CH4_CnV_Busy = 1;
}


void driveReverseRightTrack( uint16_t predkosc ){

	while( CH2_CnV_Busy );
	PTC->PSOR |= RIGHT;
	TPM0->CONTROLS[2].CnV = V_MOD * predkosc/100;
	CH2_CnV_Busy = 1;	
}

void driveLeft( uint16_t predkosc ){
	
	driveReverseLeftTrack( predkosc );
	driveForwardRightTrack( predkosc );
}

void driveRight( uint16_t predkosc ){
	
	driveReverseRightTrack( predkosc );
	driveForwardLeftTrack( predkosc );
}


void driveForward(uint16_t predkosc){
	
	driveForwardLeftTrack(predkosc);
	driveForwardRightTrack(predkosc);
}

void driveReverse(uint16_t predkosc){
	
	driveReverseLeftTrack(predkosc);
	driveReverseRightTrack(predkosc);
}
