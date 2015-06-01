#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_

#include "MKL46Z4.h"

// initialize motor driver
void motorDriverInit(void);

void driveStopLeft(void);
void driveStopRight(void);
void driveStop(void);

void driveForwardLeftTrack( uint16_t predkosc );
void driveForwardRightTrack( uint16_t predkosc );
void driveReverseLeftTrack( uint16_t predkosc );
void driveReverseRightTrack( uint16_t predkosc );
void driveForward( uint16_t predkosc ); // predkosc intiger 0 - 100 , nonlimit way
void driveReverse( uint16_t predkosc ); //  predkosc intiger 0 - 100 , nonlimit way
void driveLeft( uint16_t predkosc );
void driveRight( uint16_t predkosc );

#endif
