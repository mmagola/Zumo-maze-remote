/**
	@file	zumo_maze.h
	@brief	Maze solver library prepared for Freescale KL46Z board and Pololu Zumo Shield
*/
#ifndef ZUMO_MAZE_H_
#define ZUMO_MAZE_H_
#include "MKL46Z4.h"


/**
	@brief	Defines how many nodes will be in the maze.	In one crossroad can be many nodes (Zumo may reach the same crossroad from different directions).
	@details	Obviously this is only approximation and it should be grater than actual number of nodes.
	@warning	Do not put here to big value because optimization function is based on recursion and memory problems may occur.
*/
#define MAX_NBR_OF_NODES 100


/*!
 * @addtogroup Possible_sensor_array_states Possible sensor array states
 * @{
 */
#define FINISH 			0x2D  	// 101101
#define ALL 				0x3F    // 111111
#define CENTER 			0x0C 		// 001100
#define LEFT 				0x3C    // 111100
#define RIGHT 			0x0F		// 001111
#define EMPTY 			0x00		// 000000
/**
 * @}
 */
 
/**
	@brief	Node definition
*/
#define NODE ((la_getSensorState() == FINISH)||(la_getSensorState() == ALL)||(la_getSensorState() == LEFT)||(la_getSensorState() == RIGHT)||(la_getSensorState() == EMPTY))



// Node buffer structure
/**
  @brief Buffer structure for ::nodeArr and ::optimizedNodeArr
*/
typedef struct{
	char tab[ MAX_NBR_OF_NODES ];		/**< Reaction array */
	uint16_t max_index; 						/**< Index of next element */
} NodeArr_t;


// Global variables
/**
	@brief Buffer for registered actions
*/
extern NodeArr_t nodeArr;
/**
	@brief Buffer for orders
*/
extern NodeArr_t optimizedNodeArr;


/**
	@brief	Enumerated types of nodes which can be spotted.
*/
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


// Main functions
/**
	@brief	Function is checking minimum and maximum value from each light sensor while Zumo is turning around.
	@details	Best way to calibrate Zumo is to put it on the brightest part of line. You are sure that rest of maze will be identified well.
	@param	speed Rotation speed from 0 to 100.
	@warning	Zumo has to be on the line. If you pick up Zumo before it stops calibration, entire process should be restarted.
*/
void		zm_calibration( uint8_t speed );

/**
	@brief Function which allows to follow the line until Zumo will reach the node (crossroad, dead end etc. See -> ::Node_type).
	@details	There is software PID controller in the function. It reads light sensors state and manipulates voltage of engines by PWM.
	@param speed Zumo velocity in range 0-100.
*/
void		zm_driveToNode( uint8_t speed );

/**
	@brief Function checks which type of node is on the road.
	@param speed Zumo velocity in range 0-100.
	@return Return value is the node type enumerated in ::Event_type
*/
uint8_t	zm_checkNode( uint8_t speed );

/**
	@brief	Function performs reaction in node according to left-hand rule (Zumo turns left at intersection when it can turn).
	@details	Available reactions
						<ul>
							<li> F - finish (do nothing)
							<li> L - turn left
							<li> R - turn right
							<li> T - turn around
							<li> S - drive straight (do nothing)
							<li> l - left turn (turn left but do not save this reaction)
							<li> r - right turn (turn right but do not save this reaction)
						</ul>
	@param	node_type Type of node where movement will be done.
	@param	speed Rotation speed.
	@return	Return value is the oldest character in buffer.
	@warning	Function does not check whether buffer is empty.
*/
char		zm_nodeReaction( uint8_t node_type, uint8_t speed );

/**
	@brief	Function creates the shortest path based on previous.
	@details	Function looks for 'T' reaction and right combination of adjacent values.
						Rules
						<ul>
							<li> LTR = T
							<li> LTS = R
							<li> RTL = T
							<li> STL = R
							<li> STS = T
							<li> LTL = S
						</ul>						
						Reactions legend in ::zm_nodeReaction
	@param	old_route Pointer to input node buffer
	@param	new_route Pointer to output (optimized) node buffer.
	@warning	Function uses recursion.
*/
void		zm_routeOptimizer( const char * old_route, char * new_route);

/**
	@brief	Zumo performs reaction in the node according to external command.
	@param	node_array Pointer to buffer where reactions are saved.
	@param	node_type Type of node where movement will be done.
	@param	speed Rotation speed.
	@return	Return value is the performed movement saved as ASCII character. Legend in ::zm_nodeReaction.
*/
char		zm_strictNodeReaction( NodeArr_t * node_array, uint8_t node_type, uint8_t speed );



// Other functions
/**
	@brief	Function puts zeros to buffer.
	@param	node_array Pointer to node buffer structure
*/
void zm_clearArray( NodeArr_t * node_array );

/**
	@brief	Function puts one character (movement code) to node buffer. Maximum number of elements is declared in ::MAX_NBR_OF_NODES.
	@param	reaction ASCII encoded movement.
	@param	node_array Pointer to buffer structure
*/
void zm_addReaction( char reaction, NodeArr_t * node_array );

/**
	@brief	Function getting one character from node buffer.
	@param	node_array Pointer to buffer structure
	@return	Return value is the oldest character in buffer.
	@warning	Function does not check length of node array.
*/
char zm_getReaction( NodeArr_t * node_array );

/**
	@brief Simple delay function based on for loop.
	@param value Time in milliseconds
	@warning Function is not precise.
*/
void _delay_ms( uint32_t value );

#endif
