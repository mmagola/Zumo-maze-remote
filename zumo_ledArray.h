/**
	@file		zumo_ledArray.h
	@brief		Library for Freescale KL46Z and Pololu Zumo Reflectance Sensor Array
	@details	Requirements (hardware and software):
						<ul>
							<li> Sensor pinout (from left): PTA4, PTC1, PTD6, PTC2, PTD3,PTA5.
							<li> Disabled NMI. In startup_MKL46Z4.s (line 202) replace FOPT EQU 0xFF  with  FOPT EQU 0xFB
							<li> CLOCK_SETUP  in system_MKL46Z4.c  equals 1
							<li> PEMicro soft in programmer. When there is CMSIS on PTC1 is always 'high' (probably pull-up resistor in programmer is not disabled).
						</ul>
*/

#ifndef ZUMO_LEDARRAY_H_
#define ZUMO_LEDARRAY_H_

#include "MKL46Z4.h"

/**
  @brief	Number of periods 32kHz square wave enough to discharge capacitors
*/
#define LA_LPTMR_DELAY_CAP_DISCHARGE	5

/**
  @brief Maximum number of periods 32kHz square wave enough to charge capacitors. It is used only if some sensor is broken.
*/
#define LA_LPTMR_DELAY_CAP_MAX_CHARGE 0xffff

/**
  @brief	Threshold between black and white colour
*/
#define LA_PERCENTAGE_SWITCHING_LEVEL 50

/**
  @brief	Buffer structure for ::ledArr
*/
typedef struct{
	uint16_t value;		/**< Current value */
	uint16_t min;			/**< Registered in calibration mode minimum value */
	uint16_t max;			/**< Registered in calibration mode maximum value */
} la_sensor_t;

/**
	@brief	Function prepares LED array pins and LPTMR to work
*/
void la_init(void);

/**
	@brief	Function sets calibration flag.
*/
void la_startCal(void);

/**
	@brief	Function clears calibration flag.
*/
void la_stopCal(void);

/**
	@brief	This function returns status of each sensor.
	@return	Return value is byte with binary coded sensor state (last 6 bits, '1' means dark).
*/
char la_getSensorState(void);

/**
	@brief	This function calculates relative 'surface reflectivity'
	@param	output_array Pointer to destination array.
	@warning	Output array has to have six elements.
*/
void la_getPercentageReflectance( int16_t * output_array );

/**
	@brief	This function prepares LED array pins (multiplexers, pull-up/pull-down resistors, NVIC)
*/
void la_pins_init(void);

/**
	@brief	As the name suggests, function sets pins as inputs and enables interrupts
*/
void la_pins_as_inputs(void);

/**
	@brief	Function sets pins as outputs and disables interrupts
*/
void la_pins_as_outputs_and_high(void);

/**
	@brief	This function resets LPTMR and set time value
	@param	time This value will be writtenn to LPTMR_CMR_COMPARE register
*/
void lptimer_reload( uint16_t time );

/**
	@brief	This function reads CNR register.
	@details	We had to create this function because each reading of this register has to be preceded by writing something to it.
	@return	Return value is number from LPTMR CNR
*/
uint16_t la_getLptmrCNR(void);

/**
	@brief	This function decides which colour is under each sensor (1 means dark/black)
	@param	sensor_array Pointer to buffer structure
	@return	Return value is byte with binary coded sensor state (last 6 bits, '1' means dark).
*/
void la_calibrateMinMax( volatile la_sensor_t * sensor_array );

/**
	@brief	This function decides which colour is under each sensor (1 means dark/black)
	@param	sensor_array Pointer to buffer structure
	@return	Return value is byte with binary coded sensor state (last 6 bits, '1' means dark).
*/
char la_calculateSensorState( volatile la_sensor_t * sensor_array );

#endif
