/**
	@file		zumo_ledArray.c
	@brief		Library for Freescale KL46Z and Pololu Zumo Reflectance Sensor Array
	@details	Requirements (hardware and software):
						<ul>
							<li> Sensor pinout (from left): PTA4, PTC1, PTD6, PTC2, PTD3,PTA5.
							<li> Disabled NMI. In startup_MKL46Z4.s (line 202) replace FOPT EQU 0xFF  with  FOPT EQU 0xFB
							<li> CLOCK_SETUP  in system_MKL46Z4.c  equals 1
							<li> PEMicro soft in programmer. When there is CMSIS on PTC1 is always 'high' (probably pull-up resistor in programmer is not disabled).
						</ul>
*/

#include "MKL46Z4.h"
#include "zumo_ledArray.h"

// Global variables
volatile la_sensor_t ledArr[6];		/**< Six sensors array */
volatile char la_state;						/**< Encoded status of each sensor (six last bits) */
volatile uint8_t measured = 0;		/**< Counter how many sensors has been readed */
volatile uint8_t cal_flag = 0;		/**< Calibration flag which allow LED array to perfotm self-calibration */
volatile uint8_t valid_data = 0;	/**< Semaphore */

void la_init(void){
	
	la_pins_init();
	la_pins_as_outputs_and_high();
	
	SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK; 	/*Turn on ADC Low Power Timer (LPTMR) registers clock gate*/ 	 
	
	/* Configure LPTMR as timer in 'clear CNR in compare' mode*/ 
	LPTMR0->CSR = (	LPTMR_CSR_TCF_MASK | LPTMR_CSR_TIE_MASK );
	LPTMR0->PSR = ( LPTMR_PSR_PCS( 0 ) | LPTMR_PSR_PBYP_MASK );			/* Set 32kHz MCGIRCLK clock source. No prescaler selected */
	LPTMR0->CMR = LPTMR_CMR_COMPARE( LA_LPTMR_DELAY_CAP_DISCHARGE );

	/* Enable interrupt*/
	NVIC_ClearPendingIRQ(LPTimer_IRQn); 	/* Clear any pending interrupt */
	NVIC_EnableIRQ(LPTimer_IRQn);
		
	LPTMR0->CSR |=  LPTMR_CSR_TEN_MASK;
}

void la_startCal(void){
	cal_flag = 1;
}
void la_stopCal(void){
	cal_flag = 0;
}

void la_pins_init(void){
	
	// 
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
	PORTA->PCR[4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[1] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[2] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[3] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[5] &= ~PORT_PCR_MUX_MASK;

	PORTA->PCR[4] |= PORT_PCR_MUX(1);
	PORTC->PCR[1] |= PORT_PCR_MUX(1);
	PORTD->PCR[6] |= PORT_PCR_MUX(1);
	PORTC->PCR[2] |= PORT_PCR_MUX(1);
	PORTD->PCR[3] |= PORT_PCR_MUX(1);
	PORTA->PCR[5] |= PORT_PCR_MUX(1);	

	PORTA->PCR[4] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTC->PCR[1] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTD->PCR[6] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTC->PCR[2] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTD->PCR[3] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTA->PCR[5] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
		
	NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);				/* Clear NVIC any pending interrupts on PORTC_D */
	NVIC_ClearPendingIRQ(PORTA_IRQn);				/* Clear NVIC any pending interrupts on PORTC_A */
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void la_pins_as_inputs(void){
	
	FPTA->PDDR &= ~( (1ul<<4) | (1ul<<5) );
	FPTC->PDDR &= ~( (1ul<<1) | (1ul<<2) );
	FPTD->PDDR &= ~( (1ul<<3) | (1ul<<6) );
	
	// Enable the interrupts
	PORTA->PCR[4] |= PORT_PCR_IRQC(10);   // Interrupt on falling edge
	PORTC->PCR[1] |= PORT_PCR_IRQC(10);
	PORTD->PCR[6] |= PORT_PCR_IRQC(10);
	PORTC->PCR[2] |= PORT_PCR_IRQC(10);
	PORTD->PCR[3] |= PORT_PCR_IRQC(10);
	PORTA->PCR[5] |= PORT_PCR_IRQC(10);		
}

void la_pins_as_outputs_and_high(void){
	
	// Disable the interrupts
	PORTA->PCR[4] &= ~PORT_PCR_IRQC_MASK;		// left side (top view)
	PORTC->PCR[1] &= ~PORT_PCR_IRQC_MASK;
	PORTD->PCR[6] &= ~PORT_PCR_IRQC_MASK;		
	PORTC->PCR[2] &= ~PORT_PCR_IRQC_MASK;
	PORTD->PCR[3] &= ~PORT_PCR_IRQC_MASK;
	PORTA->PCR[5] &= ~PORT_PCR_IRQC_MASK;		// right side (top view)
		
	FPTA->PDDR |= (1ul<<4) | (1ul<<5);
	FPTC->PDDR |= (1ul<<1) | (1ul<<2);
	FPTD->PDDR |= (1ul<<3) | (1ul<<6);
	
	FPTA->PSOR |= (1ul<<4) | (1ul<<5);
	FPTC->PSOR |= (1ul<<1) | (1ul<<2);
	FPTD->PSOR |= (1ul<<3) | (1ul<<6);
}

void lptimer_reload( uint16_t time ){
	
	LPTMR0->CSR &= ~( LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK );			/* Disable timer to clear timer register*/
	LPTMR0->CMR = LPTMR_CMR_COMPARE( time );	
	LPTMR0->CSR |=  LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK; /* Enable LPTMR timer and interupt */	
}

uint16_t la_getLptmrCNR(void){
	
	// Create a pinter to CNR register (Keil does not allow to write something to this register. In it's opinion it is read-only register)
	uint32_t * point = (uint32_t*)0x4004000Cu;
	*point = 0;		// Write something
	return LPTMR0->CNR;	// Get valid data
}

void la_calibrateMinMax( volatile la_sensor_t * sensor_array ){
	
	uint8_t i;
	for(i=0; i<6; i++){
		if( (sensor_array+i)->value > (sensor_array+i)->max ) (sensor_array+i)->max = (sensor_array+i)->value;
		if( (sensor_array+i)->value < (sensor_array+i)->min ) (sensor_array+i)->min = (sensor_array+i)->value;
	}
}

void la_getPercentageReflectance( int16_t * output_array ){
	
	uint8_t i;
	for(i=0; i<6; i++){
		*(output_array+i) = 100 - (100 * ((ledArr+i)->value - (ledArr+i)->min) / ((ledArr+i)->max - (ledArr+i)->min));
	}
}

char la_calculateSensorState( volatile la_sensor_t * sensor_array ){
	
	char state = 0;
	uint8_t i;
	int16_t temp;
	
	for(i=0; i<6; i++){
		// rotate left
		state <<= 1;
		// calculate reflectance
		temp = 100 - (100 * ((sensor_array+i)->value - (sensor_array+i)->min) / ((sensor_array+i)->max - (sensor_array+i)->min));
		// if reflectance is smaller than 
		if( temp < LA_PERCENTAGE_SWITCHING_LEVEL ) state |= 0x01;
	}
	return state;
}

char la_getSensorState( void ){

	while( valid_data == 0 );	// If data are changing, wait a while.
	return la_state;
}

/**
	@brief	This function changes sensor pins direction. It works after charging sensor capacitors.
*/
void LPTimer_IRQHandler(void){		

	la_pins_as_outputs_and_high();
	LPTMR0->CSR |=  LPTMR_CSR_TCF_MASK;										// Clear interrupt flag
	lptimer_reload( LA_LPTMR_DELAY_CAP_MAX_CHARGE );		// Set maximum discharge time. If everything is ok it is useless.
	la_pins_as_inputs();
}

/**
	@brief	Voltage drop function for two sensors.
	@details	This functions works simply. It decides which sensor has triggered the interrupt. 
						Reads it's time and saves it in array. When voltage on each sensor has dropped it reloads timer and sensor counter
						and calculates state of sensors.
*/
void PORTA_IRQHandler(void){
	
	if( PORTA->PCR[4] & PORT_PCR_ISF_MASK ){

		(ledArr+0)->value = la_getLptmrCNR();					// Read time of discharge
		PORTA->PCR[4] |= PORT_PCR_ISF_MASK;						// Clear interrupt flag
		measured++;																		// Increment counter of readed
	}
	else if( PORTA->PCR[5] & PORT_PCR_ISF_MASK ){
		
		(ledArr+5)->value = la_getLptmrCNR();
		PORTA->PCR[5] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	
	// If each sensor has been readed...
	if( measured == 6 ){
		
		measured = 0;																			// Reset the counter
		la_pins_as_outputs_and_high();										// Discharge capacitors
		lptimer_reload( LA_LPTMR_DELAY_CAP_DISCHARGE );			// Set Discharge time
		
		// If calibration is set
		if( cal_flag == 1 ) la_calibrateMinMax( ledArr );	// calibrate the sensors
		
		valid_data = 0;																		// Set 'semaphore'
		la_state = la_calculateSensorState( ledArr );			// Calculate each sensor status
		valid_data = 1;																		// Release 'semaphore'
	}
}

/**
	@brief	Voltage drop function for four sensors.
	@details	It works like ::PORTA_IRQHandler
*/
void PORTC_PORTD_IRQHandler(void){
	
	if( PORTD->PCR[6] & PORT_PCR_ISF_MASK ){
		
		(ledArr+2)->value = la_getLptmrCNR();
		PORTD->PCR[6] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	else if( PORTC->PCR[2] & PORT_PCR_ISF_MASK ){
		
		(ledArr+3)->value = la_getLptmrCNR();
		PORTC->PCR[2] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	else if( PORTC->PCR[1] & PORT_PCR_ISF_MASK ){
		
		(ledArr+1)->value = la_getLptmrCNR();
		PORTC->PCR[1] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	else if( PORTD->PCR[3] & PORT_PCR_ISF_MASK ){
		
		(ledArr+4)->value = la_getLptmrCNR();
		PORTD->PCR[3] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	
	if( measured == 6 ){
		
		measured = 0;
		la_pins_as_outputs_and_high();
		lptimer_reload( LA_LPTMR_DELAY_CAP_DISCHARGE );
		
		if( cal_flag == 1 ) la_calibrateMinMax( ledArr );
		
		valid_data = 0;
		la_state = la_calculateSensorState( ledArr );
		valid_data = 1;
	}
}
