// ppm.h

#ifndef _PPM_h
#define _PPM_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

///////////////////////////////////////////////////////////////////////////////
//                         DEFINITIONS AND CONSTANTS                         //
///////////////////////////////////////////////////////////////////////////////

//
// IO pin definitions
//
#define pulsePin (3) // Digital 3, i.e. PD3
#define InputCapturePin (8) // PB0 = ICP1 = Input Capture  - this cannot be changed!
#define PpmOutputPin (9)    // PB1 = OC1A = Output Compare - this cannot be changed!
#define GreenLedPin (13)

//
// Enable/Disable debug PRINTing
//
#define ENABLE_SERIAL_PRINT
//#define DEBUG_TOTAL_PULSE_SET_TIME


#ifdef ENABLE_SERIAL_PRINT
#define PRINT Serial.print
#define PRINTLN Serial.println
#else
#define PRINT(a, ...) // empty
#define PRINTLN(a, ...) // empty
#endif

#define PPM_PULSE_LOW_TIMEOUT (800)             // actual length of the separation pulse is about 400us - 800timer ticks
#define PPM_STREAM_RESET_FULL_TIMEOUT (5000)    // The longer pulse after the normal control pulses, that resets the index to 0 - 2500us = 5000 timer ticks
#define PPM_PULSE_STREAM_TOTAL_TIMEOUT (40000)  // 20000us = 40000 timer ticks
#define PPM_MIDLLE_TIMEOUT (2200) // 0.7..1.5 => middle = 1.1ms pulse, which counts for middle value

#define PPM_VALIDITY_MAX (30) // this is about 1 second of hysteresis
#define COMP_INTERVAL_MS 20 // 20 ms == RC control interval

//
// TBD: should these numbers be in settings?
//
#define NUM_PPM_PULSES_INP_1 (7)   // GRAUPNER MC 12
#define NUM_PPM_PULSES_INP_2 (8) // FATSHARK HEAD TRACKER
#define NUM_PPM_PULSES_OUTPUT (8)








void ppm_init();
void ppm_execute();
void ppm_printUnfiltOutput();
uint16_t ppm_ChValUnFilt(uint8_t ch);

#endif

