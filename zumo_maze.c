/**
	@file	zumo_maze.c
	@brief	Maze solver library prepared for Freescale KL46Z board and Pololu Zumo Shield
*/
#include "zumo_maze.h"
#include "MKL46Z4.h"
#include "motorDriver.h"
#include "zumo_ledArray.h"
#include <string.h>

// Global variables
NodeArr_t nodeArr;
NodeArr_t optimizedNodeArr;


void zm_clearArray( NodeArr_t * node_array ){

	uint16_t i;
	for(i=0; i<MAX_NBR_OF_NODES; i++)	*(node_array->tab+i) = 0; // Array reset
	node_array->max_index = 0;
}


void zm_calibration( uint8_t speed ){
	
	// Enable the calibration in LED array
	la_startCal();
	// Start spinning
	driveRight( speed );
	// Wait some time
	_delay_ms( 2000 );
	
	while( la_getSensorState() != 0x0C ); // Stop when the line is under LED array (2 center sensors)
	driveStop();
	// Disable the calibration
	la_stopCal();
}

void zm_driveToNode( uint8_t speed ){
	
	// Coefficients are obtained experimentally
	const int16_t Kp = 15;
	const int16_t Ki = 256;
	const int16_t Kd = 1;
	
	
	// Prepare PID variables 
	int16_t integral = 0;
	int16_t derivative = 0;
	int16_t previous_error = 0;
	int16_t error = 0;
	int16_t output = 0;
	int16_t vleft = 0;
	int16_t vright = 0;
	
	// Drive...
	driveForward(speed);
	
	// until you reach the node.
	while( !NODE ){
		
		// Get error value
		switch( la_getSensorState() ){
			
			case 0x10:	error = -3;	break;	// Line under left side of array
			case 0x18:	error = -2;	break;
			case 0x08:	error = -1;	break;
			case 0x0C:	error = 0;	break;	// Line under center (no error)
			case 0x04:	error = 1;	break;
			case 0x06:	error = 2;	break;
			case 0x02:	error = 3;	break;	// Line under right side
			
			default:		error = 0;	break;			
		}
		
		integral += error;
		derivative = error - previous_error;
		previous_error = error;
		
		// PID output value
		output = error*Kp + integral/Ki + derivative*Kd;
		
		vleft = speed + output;
		vright = speed - output;
		
		// Normalization (motorDriver library has been changed a little and there is not checking the values inside it)
		if( vleft > 100 ) vleft = 100;
		if( vleft < 0 ) vleft = 0;
		if( vright > 100 ) vright = 100;
		if( vright < 0 ) vright = 0;
		
		driveForwardLeftTrack( vleft );
		driveForwardRightTrack( vright );
		
// 		PID controller (source: http://en.wikipedia.org/wiki/PID_controller):	
//		previous_error = 0
//		integral = 0
//		start:
//			error = setpoint - measured_value
//			integral = integral + error*dt
//			derivative = (error - previous_error)/dt
//			output = Kp*error + Ki*integral + Kd*derivative
//			previous_error = error
//			wait(dt)
//			goto start

	}
	// If you are near the node stop the engines.
	driveStop();
}


uint8_t zm_checkNode( uint8_t speed ){

	uint8_t leftAvailable = 0;
	uint8_t rightAvailable = 0;
	uint16_t delay_time = 100;	
	char new_photo;
	
	// White board means there is end of path. So return it.
	if( la_getSensorState() == EMPTY ) return DEAD_END;
	// Special sign (101101) means end of maze.
	else if( la_getSensorState() == FINISH ) return MAZE_END;
	
	// Drive through the intersection until you reach...
	driveForward( speed );
	
	while(	la_getSensorState() != EMPTY 			// ... white board.
			&&	la_getSensorState() != CENTER			// ... line under 2 center sensors
			&&	la_getSensorState() != 0x04				// ... line under one of center sensors
			&&	la_getSensorState() != 0x08 ){		// ... line under one of center sensors
					
		if( (la_getSensorState() & LEFT) == LEFT ) leftAvailable = 1;				// If you see line on the left, set 'left' flag.
		if( (la_getSensorState() & RIGHT) == RIGHT ) rightAvailable = 1;		// If you see line on the right, set 'right' flag.
	}
	
	// Pass the node to get right position to turn.
	_delay_ms( delay_time );
	
	// Make a 'photo'
	new_photo = la_getSensorState();
	// If there is a line under one sensor that means there is a route.
	if( new_photo == 0x04 || new_photo == 0x08 ) new_photo = CENTER;		

	// Stop the engines.
	driveStop();
	
	// Decide which type of node you passed.
	if( new_photo == EMPTY ){
		if( leftAvailable && rightAvailable ) return LEFT_RIGHT_CROSS;
		else if( leftAvailable ) return LEFT_TURN;
		else if( rightAvailable ) return RIGHT_TURN;
	}
	else{    //center
		if( leftAvailable && rightAvailable ) return FULL_CROSS;
		else if( leftAvailable ) return STRAIGHT_LEFT_CROSS;
		else if( rightAvailable ) return STRAIGHT_RIGHT_CROSS;
	}
	
	// 'Fuse'
	return 127;
}

char zm_nodeReaction( uint8_t node_type, uint8_t speed ){
	
	char reaction;
	
	// If you get to the end ...
	if( node_type == MAZE_END ){
		reaction = 'F';													// ... set right reaction character,
		zm_addReaction( reaction, &nodeArr );		// and put it in buffer.
		zm_addReaction( '\0', &nodeArr );		
	}
	// If you can turn left on crossroad ...
	else if( node_type == FULL_CROSS || node_type == LEFT_RIGHT_CROSS || node_type == STRAIGHT_LEFT_CROSS ){
		reaction = 'L';													// ... set right reaction character,
		zm_addReaction( reaction, &nodeArr );		// and put it in buffer.
		driveLeft( speed );											// Start turning left
		while( la_getSensorState() & 0x0C );		// until you get another line (line -> white -> line sequence).
		while( la_getSensorState() != 0x0C );
		driveStop();
	}
	// If you can not do anything...
	else if( node_type == DEAD_END ){
		reaction = 'T';													// ... set right reaction character,
		zm_addReaction( reaction, &nodeArr );		// and put it in buffer.
		driveRight( speed );										// Turn around.
		while( la_getSensorState() != 0x0C );
		driveStop();
	}
	// If there is not road on the left... 
	else if( node_type == STRAIGHT_RIGHT_CROSS ){
		reaction = 'S';													// ... set right reaction character and save it in buffer.
		zm_addReaction( reaction, &nodeArr );	
	}
	// If there is only some turn...
	else if( node_type == LEFT_TURN ){
		reaction = 'l';
		driveLeft( speed );											// Turn in right direction.
		while( la_getSensorState() & 0x0C );
		while( la_getSensorState() != 0x0C );
		driveStop();
	}
	// The same as above.
	else if( node_type == RIGHT_TURN ){
		reaction = 'r';
		driveRight( speed );
		while( la_getSensorState() & 0x0C );
		while( la_getSensorState() != 0x0C );
		driveStop();
	}
	return reaction;
}



void _delay_ms( uint32_t value ){
	uint32_t i;
	for(i=0; i<(6500*value); i++);
}


void zm_addReaction( char reaction, NodeArr_t * node_array ){
	
	// Put character and increment an iterator.
	node_array->tab[node_array->max_index++] = reaction;
};

char zm_getReaction( NodeArr_t * node_array ){
	
	// Get character and increment an interator.
	return node_array->tab[node_array->max_index++];
}




void zm_routeOptimizer( const char * old_route, char * new_route){

	static char temp[ MAX_NBR_OF_NODES ];
  uint8_t compressed = 0;
	
	// Hop will be 1 or 3. It depend on 'T' command in node array.
  uint8_t hop;	
	uint16_t i;
	uint16_t output_index = 0;
  uint16_t route_table_length = strlen(old_route);
	
	// Clear temp array
	//for(i=0; i<route_table_length; i++)	*((char*)temp+i) = 0;

	// Read each command
	for( i=0; i<route_table_length; i += hop ){

		// Check whether next command is not a 'turn' command...
		if( old_route[i+1] != 'T' ){
			*((char*)temp+output_index) = *(old_route+i);		// and write it to temp array.
			hop = 1;
		}
		// If is a 'T' you can change all three commands in one...
		else{

			compressed = 1;
			hop = 3;
			
			// ... using this formula.
			if(  		 *(old_route+i) == 'L' &&  *(old_route+i+2) == 'R'  )	*((char*)temp+output_index) = 'T';
			else if( *(old_route+i) == 'L' &&  *(old_route+i+2) == 'S'  )	*((char*)temp+output_index) = 'R';
			else if( *(old_route+i) == 'R' &&  *(old_route+i+2) == 'L'  )	*((char*)temp+output_index) = 'T';
			else if( *(old_route+i) == 'S' &&  *(old_route+i+2) == 'L'  )	*((char*)temp+output_index) = 'R';
			else if( *(old_route+i) == 'S' &&  *(old_route+i+2) == 'S'  )	*((char*)temp+output_index) = 'T';
			else if( *(old_route+i) == 'L' &&  *(old_route+i+2) == 'L'  )	*((char*)temp+output_index) = 'S';
		}
		// Move cursor (pointer) to the next address.
		output_index++;
	}
	// Add 'ending'
	*((char*)temp+output_index) = '\0';
	
	// If you have optimized something run procedure again.
	if( compressed == 1 )	zm_routeOptimizer( temp, new_route);
	// otherwise copy orders to destination array.
	else{
		for( i=0; i<strlen(temp); i++ )	*((char*)new_route+i) = *(temp+i);
		*((char*)new_route+strlen(temp)) = '\0';
	}
}


char zm_strictNodeReaction( NodeArr_t * node_array, uint8_t node_type, uint8_t speed ){
	
	char reaction;
	
	// If you get to the finish ...
	if( node_type == MAZE_END ){
		reaction = zm_getReaction( &optimizedNodeArr );		// ... get last command ('F').
	}
	// If there is dead end ... 
	else if( node_type == DEAD_END ){
		reaction = zm_getReaction( &optimizedNodeArr );		// ... get next command ('T')
		driveRight( speed );															// and turn around.
		while( la_getSensorState() & 0x0C );
		while( la_getSensorState() != 0x0C );
		driveStop();
	}
	// If node is some turn you do not have to get command.
	else if( node_type == LEFT_TURN ){	
		reaction = 'l';
		driveLeft( speed );
		while( la_getSensorState() & 0x0C );
		while( la_getSensorState() != 0x0C );
		driveStop();
	}
	else if( node_type == RIGHT_TURN ){
		reaction = 'r';
		driveRight( speed );
		while( la_getSensorState() & 0x0C );
		while( la_getSensorState() != 0x0C );
		driveStop();
	}
	
	// If there is crossroad...
	else if(	 node_type == FULL_CROSS 
					|| node_type == LEFT_RIGHT_CROSS
					|| node_type == STRAIGHT_LEFT_CROSS 
					|| node_type == STRAIGHT_RIGHT_CROSS ){
		
		reaction = zm_getReaction( &optimizedNodeArr );		// ... ask for help ...
						
		// ... and follow the order
		switch( reaction ){
			
			
			// turn in right direction
			case 'S':
				break;
			
			case 'L':
				driveLeft( speed );
				while( la_getSensorState() & 0x0C );
				while( la_getSensorState() != 0x0C );
				driveStop();
				break;
			
			case 'R':
				driveRight( speed );
				while( la_getSensorState() & 0x0C );
				while( la_getSensorState() != 0x0C );
				driveStop();
				break;
			
			// If you have to turn around you should check on which number of line you have to stop turning
			case 'T':
				
				if( node_type == LEFT_RIGHT_CROSS || node_type == FULL_CROSS ){
					driveRight( speed );
					// On second line
					while( la_getSensorState() & 0x0C );
					while( la_getSensorState() != 0x0C );
					while( la_getSensorState() & 0x0C );
					while( la_getSensorState() != 0x0C );		
				}
				else if( node_type == STRAIGHT_LEFT_CROSS ){
					driveRight( speed );
					// On first line
					while( la_getSensorState() & 0x0C );
					while( la_getSensorState() != 0x0C );
				}
				else if( node_type == STRAIGHT_RIGHT_CROSS ){
					driveLeft( speed );
					// On first line
					while( la_getSensorState() & 0x0C );
					while( la_getSensorState() != 0x0C );
				}
				driveStop();			
				break;
			
			default:
				break;
		}
	}

	return reaction;
}
