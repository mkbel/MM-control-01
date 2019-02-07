#ifndef _MAIN_H
#define _MAIN_H


#include <inttypes.h>
#include <stdio.h>

void manual_extruder_selector();
void unrecoverable_error(uint16_t leds);
void drive_error(uint16_t leds);
void check_filament_not_present();
void filament_presence_signaler();

extern uint8_t tmc2130_mode;
extern FILE* uart_com;

#endif //_MAIN_H
