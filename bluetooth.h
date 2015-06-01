/**
 * @file		bluetooth.h
 * @brief		Bluetooth library for KL46Z and HC-06 module
 */
 
#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_
#include "MKL46Z4.h"

// User settings
/**
	@brief	Defines which UART will be used for communication.	
	@details	You can select any UART module (0-2).
						Default pins:
						<ul>
							<li> UART0 - PTA15(RX)   PTA14(TX)
							<li> UART1 - PTE1 (RX)   PTE0(TX)
							<li> UART2 - PTE17(RX)   PTE16(TX)
						</ul>
	@warning	When KL46Z is connected to ZUMO, UART 1 is useless, 
						because on PTE0 and PTE1 is I2C bus.
*/
#define UART_MODULE 2
/**
	@brief Defines speed of transmission.
	@details	Default communication baud rate in HC-06 is 9600.
						Others are available by changing settings in AT-mode of module.
*/
#define BAUD_RATE 9600
/**
	@brief Defines number of bytes in Rx and Tx buffers
	@details	It can be any number, but too big buffers may cause memory problems.
*/
#define BUFF_SIZE 250		
/**
	@brief When OVERWRITE is set and Rx buffer is full the oldest character will be replaced by receiving data.
	@warning	It can be ONLY 0 or 1!
						OVERWRITE==1 is required when "string" functions (::bt_getStr, ::bt_sendStr) are used.
						Otherwise Tx buffer lock may occur (::bt_getStr uses ::string_count which is incremented by NULL incoming character).
						Functions guarantee the continuity of strings, but use it carefully. 
						Best way is to use still same type of functions (for example ::bt_getChar, ::bt_sendChar / ::bt_getStr, ::bt_sendStr).
*/
#define OVERWRITE 1
/**
	@brief Defines how important UART interrupts will be. It can be 1, 2 or 3.
*/
#define UART_IRQ_PRIORITY 1


// Preprocessor macros
/**
	@brief Preprocessor macro for automatic change of UART module
*/
#define UART(x) XUART(x)
/**
	@brief Preprocessor macro for automatic change of UART module
*/
#define XUART(x) (UART##x)


// Circular buffer structure
/**
  @brief Circular buffer structure for ::RxBuf and ::TxBuf
*/
typedef struct{
	char buf[BUFF_SIZE];					/**< Data */
	uint16_t head;			 					/**< Points to oldest data element */
	uint16_t tail;								/**< Points to next free space */
	uint16_t size;								/**< Quantity of elements in queue */
} UART_BUF_t;


// Global variables
/**
	@brief Circular buffer for received data
*/
extern volatile UART_BUF_t RxBuf;
/**
	@brief Circular buffer for transmitting data
*/
extern volatile UART_BUF_t TxBuf;
/**
	@brief Quantity of strings in Rx buffer
*/
extern volatile int16_t string_count;


// Main user functions
/**
	@brief Initialize the UART module and prepare both buffers
  @param baud_rate is speed of transmission. You can also use macro ::BAUD_RATE.
  @warning Be sure that CLOCK_SETUP in "system_MKL46Z4.c" equals 1.
*/
void bt_init( uint32_t baud_rate );
/**
	@brief Function sends one byte (Really, it adds byte to transmit buffer and starts the transmission).
	@param data Byte to send
	@retval uint8_t
					<ul>
					 <li> 0 = Failure
					 <li> 1 = Success
					</ul>
*/
uint8_t bt_sendChar( const char data );
/**
	@brief Function sends string. It adds characters to Tx buffer like ::bt_sendChar. 
	@param source Pointer to string (name of array that contains characters)
	@retval uint8_t
					<ul>
					 <li> 0 = Failure
					 <li> 1 = Success
					</ul>										
	@warning String has to be ended with a '\0' (NULL) character!
*/
uint8_t bt_sendStr( const char * source );
/**
	@brief	Function that reads a single character from Rx buffer (if there is any).
	@retval	uint8_t
					<ul>
					 <li> 0 = Buffer empty or '\0' character (end of string)
					 <li> other = valid data
					</ul>
	@warning	0 (NULL) and 13 (CR) are reserved. For one-byte commands use another character.
						For more details see interrupt handler.
*/
char bt_getChar( void );
/**
	@brief Function copies string from Rx buffer (if there is any) to destination array.
	@param[out] destination Pointer to character array
	@warning 	Be sure that destination table is big enough to contain incoming string.
						Incoming string has to be ended with a (NULL) or (CR) character.
*/
void bt_getStr( char * destination );


// Other functions
/**
	@brief Function clears data (writes zeros to all structure's fields)
	@param b Pointer to any UART buffer.
*/
void buf_clear( volatile UART_BUF_t * b );
/**
	@brief Function checks whether buffer is empty.
	@param b Pointer to any UART buffer
	@retval uint8_t
					<ul>
					 <li> 0 = False
					 <li> 1 = True
					</ul>
*/
uint8_t buf_empty( const volatile UART_BUF_t * b );
/**
	@brief Function checks whether buffer is full.
	@param b Pointer to any UART buffer
	@retval uint8_t
					<ul>
					 <li> 0 = False
					 <li> 1 = True
					</ul>
*/
uint8_t buf_full( const volatile UART_BUF_t * b );
/**
	@brief Function moving one character to any UART buffer.
	@param c New character
	@param b Pointer to buffer
	@warning Function does not check whether buffer is full.
*/
void to_UART_buffer( const char c, volatile UART_BUF_t * b );
/**
	@brief Function getting one character from any UART buffer.
	@param b Pointer to buffer
	@return Return value is the oldest character in buffer.
	@warning Function does not check whether buffer is empty.
*/
char from_UART_buffer( volatile UART_BUF_t * b );
/**
	@brief Function that replaces the oldest character in buffer.
	@param c New character
	@param b Pointer to buffer where 'c' character will be written
*/
void overwrite_UART_buffer( const char c, volatile UART_BUF_t * b );

#endif
