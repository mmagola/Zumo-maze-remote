#ifndef ZUMO_BUTTON_H_
#define ZUMO_BUTTON_H_
#include "MKL46Z4.h"

// pd7 - button
#define Z_BUTTON	7

void zumo_button_init(void);
uint8_t zumo_button_pressed(void);

#endif
