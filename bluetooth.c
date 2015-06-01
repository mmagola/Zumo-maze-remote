/**
 * @file		bluetooth.c
 * @brief		Bluetooth library for KL46Z and HC-06 module
 */
 
#include "MKL46Z4.h"
#include "bluetooth.h"
#include <string.h>

// Global variables
volatile UART_BUF_t RxBuf;
volatile UART_BUF_t	TxBuf;
volatile int16_t string_count = 0;

/**
	@brief Interrupt handler
	@details	It reacts to byte coming or empty D buffer (in UART module).
						Incoming CR character is converted to NULL 
						(for comfortable usage of terminal e.g. Putty)
*/
#if (UART_MODULE == 0)
void UART0_IRQHandler(void){
	
#elif (UART_MODULE == 1)
void UART1_IRQHandler(void){
	
#elif (UART_MODULE == 2)
void UART2_IRQHandler(void){
#endif	
	
	__disable_irq();
	
	if(UART(UART_MODULE)->S1 & UART_S1_RDRF_MASK){
		
		char c = UART(UART_MODULE)->D;

#if OVERWRITE==1	
		if( c == '\0' || c == '\r'){
			string_count++;
			c = '\0';
		}
#endif
		
		if( !buf_full(&RxBuf) ){
			
#if OVERWRITE==0			
			if( c == '\0' || c == '\r'){
				string_count++;
				c = '\0';
			}
#endif
			
			to_UART_buffer( c, &RxBuf );		
		}
		
#if OVERWRITE==1
		else	overwrite_UART_buffer( c, &RxBuf );
#endif

	}
	
	else if(UART(UART_MODULE)->S1 & UART_S1_TDRE_MASK){
		
		if( buf_empty(&TxBuf) ){
			
			// disable interrupt from transmitter
			UART(UART_MODULE)->C2 &= ~UART_C2_TIE_MASK;
			
			TxBuf.tail = TxBuf.head; // force pointers equalization
			// Sometimes, when transmitter is hardly loaded,
			// tail pointer stops to far. "Size" indicator still works fine.
		}
		// If Tx buffer isn't empty put in UART next character.
		else	UART(UART_MODULE)->D = from_UART_buffer( &TxBuf ); 
	}
	
#if UART_MODULE==0
	NVIC_ClearPendingIRQ(UART0_IRQn);
#elif UART_MODULE==1
	NVIC_ClearPendingIRQ(UART1_IRQn);
#elif UART_MODULE==2
	NVIC_ClearPendingIRQ(UART2_IRQn);
#endif
	
	__enable_irq();
}



void bt_init( uint32_t baud_rate ){

	uint32_t divisor;
	
	
#if UART_MODULE==0
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; 	// PTA14(TX) PTA15(RX) on mux(3) will be used.
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
	PORTA->PCR[14] |= PORT_PCR_MUX(3);
	PORTA->PCR[15] |= PORT_PCR_MUX(3);
	
#elif UART_MODULE==1
	SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;		// 24MHz bus clk
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; 	// PTE0(TX) PTE1(RX) on mux(3) will be used.	
	PORTE->PCR[0] |= PORT_PCR_MUX(3);
	PORTE->PCR[1] |= PORT_PCR_MUX(3);
	
#elif UART_MODULE==2
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;		// 24MHz bus clk
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; 	// PTE16(TX) PTE17(RX) on mux(3) will be used.	
	PORTE->PCR[16] |= PORT_PCR_MUX(3);
	PORTE->PCR[17] |= PORT_PCR_MUX(3);
#endif

	// UART module disable
	UART(UART_MODULE)->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

	// Setting prescaler value
#if UART_MODULE==0
	divisor = (48000000/baud_rate)/16;
#else
	divisor = (24000000/baud_rate)/16;
#endif

	// Cearing prescaler register
	UART(UART_MODULE)->BDH &= ~UART_BDH_SBR_MASK;
	UART(UART_MODULE)->BDL &= ~UART_BDL_SBR_MASK;	
	
	// Writting prescaler value to right register
	UART(UART_MODULE)->BDH |= UART_BDH_SBR(divisor>>8);
	UART(UART_MODULE)->BDL |= UART_BDL_SBR(divisor);
	
	// One stop bit
	UART(UART_MODULE)->BDH &= ~UART_BDH_SBNS_MASK;
	// No parity
	UART(UART_MODULE)->C1 &= ~UART_C1_PE_MASK;
	// 8-bit mode
	UART(UART_MODULE)->C1 &= ~UART_C1_M_MASK;
	
	
	// Interrupt settings
#if UART_MODULE==0
	NVIC_SetPriority(UART0_IRQn, UART_IRQ_PRIORITY);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
	
#elif UART_MODULE==1
	NVIC_SetPriority(UART1_IRQn, UART_IRQ_PRIORITY);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);
	
#elif UART_MODULE==2
	NVIC_SetPriority(UART2_IRQn, UART_IRQ_PRIORITY);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
#endif

	// Interrupts enable
	UART(UART_MODULE)->C2 |= (UART_C2_TIE_MASK | UART_C2_RIE_MASK);
	
	buf_clear(&TxBuf);
	buf_clear(&RxBuf);
	
	// UART module enable
	UART(UART_MODULE)->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
}


uint8_t bt_sendChar( const char data ){
	
	uint8_t exit = 0;		// set failure
	
	if( !buf_full(&TxBuf) ){
		to_UART_buffer( data, &TxBuf );
		exit = 1;			// set success
	}
#if OVERWRITE==1
	else{
		overwrite_UART_buffer( data, &TxBuf );
		exit = 1;			// set success
	}
#endif
	
	if( !(UART(UART_MODULE)->C2 & UART_C2_TIE_MASK) ){
		UART(UART_MODULE)->D = from_UART_buffer( &TxBuf );
		UART(UART_MODULE)->C2 |= UART_C2_TIE_MASK;
	}
	return exit;
}


uint8_t bt_sendStr( const char * source ){
	
	uint16_t i = 0;
	uint16_t len = strlen(source);
	
	if( len != 0 ){
#if OVERWRITE==0		
		// If string length is greater than free space of TxBuf
		if( ((len)+1) > ((BUFF_SIZE)-(TxBuf.size)) ) return 0;	// return failure
#endif	
		
		// Send string byte by byte.
		for(i=0; i<=len; i++)	bt_sendChar( *((source)+i));
		
	}
	return 1;		// return success
}


char bt_getChar( void ){
	
	// If RxBuf isn't empty...
	if( !buf_empty(&RxBuf) ){
		
		char c;
		
		// ... get one character
		c = from_UART_buffer( &RxBuf );
		
		
		if( c == '\0' ){
			string_count--;
			if(string_count < 0) string_count = 0;  // addictional protection
		}
		return c;
	
	}
	else return 0; // If is empty return 0.
}


void bt_getStr( char * destination ){
	
	// If in Rx buffer isn't any string return empty string.
	if( string_count == 0 ) *destination = '\0';
	
	else{
		uint16_t i = 0;
		char c;
		
		// ...if is copy one to user array.
		do{
			c = bt_getChar();
			*((destination)+i) = c;
			i++;
		}
		while( c != '\0');
	}
}


void buf_clear( volatile UART_BUF_t * b ){
	uint16_t i;
	
	for(i=0; i<BUFF_SIZE; i++) b->buf[i] = 0;		
		
	b->head = 0;
	b->tail = 0;	
	b->size = 0;	
}


uint8_t buf_empty( const volatile UART_BUF_t * b ){
	
	return (b->size) == 0;
}


uint8_t buf_full( const volatile UART_BUF_t * b ){
	
	return (b->size) == BUFF_SIZE;
}


void to_UART_buffer( const char c, volatile UART_BUF_t * b ){
	
	b->buf[b->tail++] = c;
	b->tail %= BUFF_SIZE;
	b->size++;
}


char from_UART_buffer( volatile UART_BUF_t * b ){

	char c = b->buf[b->head];
	
	b->buf[b->head++] = 0;
	b->head %= BUFF_SIZE;
	b->size--;
	return c;
}


void overwrite_UART_buffer( const char c, volatile UART_BUF_t * b ){

	if( (b==&RxBuf) && (b->buf[b->tail] == '\0') ){
		string_count--;
		if(string_count < 0) string_count = 0;
	}

	b->head++;
	b->head %= BUFF_SIZE;	
	b->buf[b->tail++] = c;
	b->tail %= BUFF_SIZE;
}
