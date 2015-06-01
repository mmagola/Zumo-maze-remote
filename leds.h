#ifndef LEDS_H_
#define LEDS_H_


void ledsInitialize(void);

void ledsOff (void);
void ledsOn (void);

void ledGreenOff (void);
void ledGreenOn (void);

void ledRedOff(void);
void ledRedOn(void);

void startStopFSM(void);
void ledsService1ms(void);
void nextLedState(void);


unsigned char phaseA(void);
unsigned char phaseB(void);
unsigned char phaseC(void);

void fastSlowFSM(void);

extern unsigned char slowFSM;

void changeDirLED(void);

#endif
