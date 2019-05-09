///////////////////////////////////////////////////////////////////////////////
//
//
//                  PPM Mixer
//                  
//                  
//                  (c) Jussi Pajala 2016-2017
//
//
// This Arduino software is intended to mix two ppm streams into one.
//
// Input 1 is an old Graupner MC-12 35MHZ radio, 35MHz radio part disabled and 
//         FrSky 2.4GHz module installed.
// Input 2 is FatShark coggles
//
// Output is connected to FrSky DHT TX Module's ppm input.
//
//
// Input 1 / Graupner Radio is inverted PPM signal, channels 1..7
// Input 2 / Fatshark coggles gives head tracking signal in channels 5..7, 
//           other channes have middle value
// 
// If Input 1 is missing no output is generated
// If Input 2 is missing, Input 1 is used as output (channel 8 value in the 
// middle)
// If both Inputs are present, then output is as Input 1 except Input 
// 2 overrides it in channels 6..8
//
// 
// HW to run this software is Arduino Pro Mini 16MHz 5V.
// Input 1 connected to D8
// Input 2 connected to D3
// Output connected to D9
//
//
// SOFTWARE DESCRIPTION
//
// All functionality is based on Timer1 and change interrupt.
//
// Change interrupt of pin D3 is used to interrupt when Fatshark coggle PPM 
// signal changes value. When the interrupt happens, the interrupt routine
// checks if the signal was going up or down, and records the timer1 value of 
// the event. The main loop does the calculation of signal times etc.
// NOTE: Only one interrupt is used to make sure there are no multiple interupt 
// requests simultaneously, which would ruin the timing.
//
// Input Capture module is used to precisely record PPM stream outputted by the
// Graupned radio in pin D8. The module stores the Timer 1 value when the
// change occurred, and main loop monitors the Input Capture Interrupt flag,
// and stores the time value into variables after it has been noticed. Also, 
// signal times are computed.
//
// Output compare module is used to generate output PPM stream.  
// Main loop monitors the Output Capture Interrupt flag, and gives instructions
// to generate next section of the pulse, i.e. Timer 1 value at the time of
// next event, and whether to set or clear the output pin.
//
// FAIL SAFE
// 
// Monitoring of the input signals has been implemented, to make sure old 
// signal values are not used too long time. 
// 
// For example, a detached wire coming from Graupner radio will disable the PPM
// output totally, to ensure the receiver of the model will go into failsafe
// mode.
// However, removing the Fatshark input will just disable overriding the
// Graupner radio signals of channels 6 and 7.
//
///////////////////////////////////////////////////////////////////////////////
#include "cmd.h"
#include "ppm.h"
#include "settings.h"




///////////////////////////////////////////////////////////////////////////////
//                              LOCAL VARIABLES                              //
///////////////////////////////////////////////////////////////////////////////

// variables for loop/cycle time monitoring
static uint32_t lastLoopT = 0;
static uint16_t largestLoopTus = 0;


Settings settings;
Cmd cmd;

///////////////////////////////////////////////////////////////////////////////
//                           FUNCTION DEFINITIONS                            //
///////////////////////////////////////////////////////////////////////////////

// Arduino Setup function
void setup(void) 
{
	// setup the interrupt to capture pulse length
	pinMode(pulsePin, INPUT);
	pinMode(InputCapturePin, INPUT);
	pinMode(GreenLedPin, OUTPUT);
	pinMode(PpmOutputPin, OUTPUT);
  
	ppm_init();
	cmd.init();

#ifdef ENABLE_SERIAL_PRINT
	Serial.begin(115200);
#endif // #ifdef ENABLE_SERIAL_PRINT

	if(!settings.load())
	{
		Serial.println("Settings load FAILED, using factory settings.");
	}
}

#ifdef ENABLE_SERIAL_PRINT
inline bool debug_PeriodicPrint()
{
  static uint32_t lastPrintMs = 0;
  if(millis()-lastPrintMs > 5000)
  {
    uint8_t i;
    lastPrintMs = millis();

    PRINT("loopt: ");
    PRINTLN(largestLoopTus);
    
    for(i = 0; i < 8; i++)
    {
      PRINT(", ");
//      PRINT(ppmOutputTiming[i]);
    }
    
    PRINTLN();
    return true; // printed something...
  }
  return false; // not printing anything
}
#else
#define debug_PeriodicPrint() false // empty definition, when not PRINTing
#endif // #ifdef ENABLE_SERIAL_PRINT


// Arduino Loop function
void loop(void) 
{
  uint32_t loopT = micros();

	ppm_execute();
	cmd.execute();
//  if(!debug_PeriodicPrint())
  {
    lastLoopT = micros() - loopT;
  
    if(lastLoopT > largestLoopTus)
    {
      largestLoopTus = lastLoopT;
      PRINTLN(lastLoopT);
    }
  }
}
 
 
 

