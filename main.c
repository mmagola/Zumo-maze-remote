/**
	@file	main.c
	@brief	Zumo maze solver main function.
	@details	It uses bluetooth transmitter to send led sensors state, type of node,
						performed reaction and whole track. Please read zumo_ledArray.c/zumo_ledArray.h descryption before you download this code to chip.
*/
#include "MKL46Z4.h"
#include "bluetooth.h"
#include "leds.h"
#include "motorDriver.h"
#include "zumo_button.h"
#include "zumo_buzzer.h"
#include "zumo_ledArray.h"
#include "zumo_maze.h"
#include <stdbool.h>



/**
	@brief	Function sends via Bluetooth (UART2) state of LED sensor. '1' means black.
	@param	state Binary coded sensor state.
*/
uint32_t horner_get_val(const char * str){
	
	uint32_t ile = 0;
	uint16_t i = 1;
	
	while( str[i] != '.' ){
		ile = 10*(ile)+(str[i]-'0');
		i++;
	}
	return ile;
}	
void sendArrayState( char state ){
	int8_t i;
	
	for( i=5; i>=0; i--){
		bt_sendChar( (state & (1<<i)) ? '1' : '0' );
	}
	bt_sendChar( '\r' );
	bt_sendChar( '\n' );
}

/**
	@brief	Function sends via Bluetooth (UART2) command set.
	@param	route Pointer to string.
	@warning	Input string has to be ended with a '\0' (NULL) character.
*/
void sendRoute( char * route ){
	
	while( *route != '\0' ){
		bt_sendChar( *route );
		route++;
	}
	bt_sendChar( '\r' );
}





/*
enum Node_type{
	DEAD_END						= '0',
	FULL_CROSS,						//1
	STRAIGHT_LEFT_CROSS,	//2
	STRAIGHT_RIGHT_CROSS,	//3
	LEFT_RIGHT_CROSS,			//4
	LEFT_TURN,						//5
	RIGHT_TURN,						//6
	MAZE_END							//7
};
*/


/**



	@brief	Zumo maze solver main function.
	@details	This function is built from three parts: calibration, solving the maze, driving to end by orders. 
						It uses bluetooth transmitter to send led sensors state, type of node, performed reaction and whole track.
*/

char tab[BUFF_SIZE];			// If your array is big, define it as global.
	
volatile uint8_t dzielnik_buz = 0;
volatile uint8_t buzzer = 0;
volatile uint8_t buzzer_enable = 0;

bool breakTheLoop( int* mode_flag ){
	
	bt_getStr(tab);
	bool ret = (*mode_flag==1) || (*tab =='r') ;
	if(ret)	*mode_flag = 1;
	
	return ret;
}

bool wirelessButton( void ){
	
	bt_getStr(tab);
	return *tab == 'm';
}


void SysTick_Handler(void) {
	
	if( buzzer_enable ){
		
		dzielnik_buz++;
		
		if( dzielnik_buz > buzzer ){
			PTA->PTOR |= (1u<<12);
			dzielnik_buz = 0;
		}
	}
}	
int main(void){
	
	uint8_t node_type;
	char reaction;
	int mode_flag = 0; //  mode flag 1 - remote   2-maze
	char c;
	
	// Initialize everything
	zumo_button_init();
	ledsInitialize();
	zumo_buzzer_init();
	bt_init( BAUD_RATE );
	motorDriverInit();
	la_init();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 10000);		// Interrupt every 100ms
	

	
	
	
	
	while(1){													// Main loop
		
		
		bt_getStr( tab );
		c = *tab;
		

		
		if(c == 'r') mode_flag = 1;
		if(c == 'm') mode_flag = 2;		

		
		
		if(mode_flag == 2){
			bt_sendStr("\rZumo Maze solver gotowy\rAby skalibrowac nacisnij przycisk...\r");
			// Wait for user reaction
			while( !zumo_button_pressed() && !wirelessButton() );
			_delay_ms( 1000 );

			bt_sendStr("Kalibruje...\r");
			// Calibrate itself
			zm_calibration( 40 );

			bt_sendStr("Kalibracja zakonczona\r");
			
			// infinite loop
			while(1){													
					
				// Turn on orange diode on Zumo. Zumo will look for exit.
				ledGreenOff();
				
				bt_sendStr("Faza 1: Rozpoznanie trasy\rAby kontynuowac nacisnij przycisk...\r");
				
				
				
				// Wait for user reaction
				while( !zumo_button_pressed() && !wirelessButton() ) {if(breakTheLoop(&mode_flag)) break;} 
				_delay_ms( 1000 );
				
				// Prepare both arrays for incoming data
				zm_clearArray( &nodeArr );
				zm_clearArray( &optimizedNodeArr );
				
				// Get to the end of the maze
				do{		
					
					
					if(breakTheLoop(&mode_flag)) break;
					zm_driveToNode( 40 );
					sendArrayState( la_getSensorState() );
					
				
					if(breakTheLoop(&mode_flag)) break;
					node_type =  zm_checkNode( 40 );
					sendArrayState( la_getSensorState() );
					bt_sendChar( node_type );
					bt_sendChar( '\r' );
								
					if(breakTheLoop(&mode_flag)) break;
					reaction = zm_nodeReaction( node_type, 40 );
					bt_sendChar( reaction );
					bt_sendChar( '\r' );
					
				}while( reaction != 'F' );
				
				if(breakTheLoop(&mode_flag)) break;
				// Play some sound
				zb_doubleBeep();
				
				
				bt_sendStr("\r\rFaza 2: Optymalizacja trasy\r");
				// Optimize route	
				zm_routeOptimizer( nodeArr.tab , optimizedNodeArr.tab );
				bt_sendChar( '\r' );
				bt_sendChar( '\r' );
				bt_sendStr("Stara trasa\r");
				sendRoute( nodeArr.tab );
				bt_sendStr("\rNowa trasa\r");
				sendRoute( optimizedNodeArr.tab );
				bt_sendChar( '\r' );
				bt_sendChar( '\r' );
				
				// Turn off orange diode on Zumo. Zumo knows where is end.
				ledGreenOn();
				
				bt_sendStr("Faza 3: Przejazd wg rozkazow\rAby kontynuowac nacisnij przycisk...\r");
				// Wait for user reaction
				
				while( !zumo_button_pressed() && !wirelessButton() ) {if(breakTheLoop(&mode_flag)) break;}
				_delay_ms( 1000 );
					
				// Get to the end without mistakes
				do{
					if(breakTheLoop(&mode_flag)) break;
					zm_driveToNode( 40 );
					sendArrayState( la_getSensorState() );
					
					if(breakTheLoop(&mode_flag)) break;
					node_type = zm_checkNode( 40 );
					sendArrayState( la_getSensorState() );
					bt_sendChar( node_type );
					bt_sendChar( '\r' );
								
					if(breakTheLoop(&mode_flag)) break;
					reaction = zm_strictNodeReaction( &optimizedNodeArr, node_type, 40);
					bt_sendChar( reaction );
					bt_sendChar( '\r' );
					
				}while( reaction != 'F' );
				
				if(breakTheLoop(&mode_flag)) break;
				
				bt_sendStr("\rDojechalem!\r\r");
				// Play some sound
				zb_doubleBeep();

			}
		}
		else if(mode_flag == 1){
			
			switch(c){
				
				case 'q':
					driveStop();
					buzzer_enable = 0;
					break;
				
				case 'w':
					driveForward( horner_get_val(tab) );
					break;
				
				case 's':
					driveReverse( horner_get_val(tab) );
					break;
				
				case 'a':
					driveReverseLeftTrack( horner_get_val(tab) );
					driveForwardRightTrack( horner_get_val(tab) );
					break;
				
				case 'd':
					driveForwardLeftTrack( horner_get_val(tab) );
					driveReverseRightTrack( horner_get_val(tab) );
					break;
				
				case 'b':
					buzzer = horner_get_val(tab)/10;
					buzzer_enable = 1;
					break;
				
				default:
					break;		
			}
		}
		
	}
}
