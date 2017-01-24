//
//
//                  PPM Mixer
//                  
//                  
//                  (c) Jussi Pajala 2016
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

#define NUM_PULSES (12)
#define PPM_PULSE_LOW_TIMEOUT (800)             // actual length of the separation pulse is about 400us - 800timer ticks
#define PPM_STREAM_RESET_FULL_TIMEOUT (5000)    // The longer pulse after the normal control pulses, that resets the index to 0 - 2500us = 5000 timer ticks
#define PPM_PULSE_STREAM_TOTAL_TIMEOUT (40000)  // 20000us = 40000 timer ticks
#define PPM_MIDLLE_TIMEOUT (2200) // 0.7..1.5 => middle = 1.1ms pulse, which counts for middle value

#define PPM_VALIDITY_MAX (30) // this is about 1 second of hysteresis
#define PULSE_FULL_LEN_MIN 2000
#define PULSE_FULL_LEN_MAX 4350
#define COMP_INTERVAL_MS 20 // 20 ms == RC control interval
#define PPM_MAP_INVALID_IDX (0xFF)

#define NUM_PPM_PULSES_GRAUPNER_MC_12 (7)
#define NUM_PPM_PULSES_FATSHARK_HEADTRACK (8)
#define NUM_PPM_PULSES_OUTPUT (8)

///////////////////////////////////////////////////////////////////////////////
//                              LOCAL VARIABLES                              //
///////////////////////////////////////////////////////////////////////////////

// variables for loop/cycle time monitoring
static uint32_t lastLoopT = 0;
static uint16_t largestLoopTus = 0;


// variables related to pulse width measurement by the change interrupt
// we have two sets of the variables, to be able to write into another set in interrupt, while the application is reading the other.
// it would be more convenient to use a structure, and an array, but that would add to memory position calculations in interrupt, which we want to avoid.
// first pulse length variables.
volatile uint16_t p0_start = 0;
volatile uint16_t p0_end = 0;
volatile uint16_t p0_nextStart = 0;
volatile bool    p0_updated = false;
volatile bool   appReading_0 = false;
// the second pulse length variables
volatile uint16_t p1_start = 0;
volatile uint16_t p1_end = 0;
volatile uint16_t p1_nextStart = 0;
volatile bool   p1_updated = false;
volatile bool   appReading_1 = false;


uint16_t calibVal = 0;

// the interrupt routine
void pulseInterruptHandler();

// Change Interrupt pulses
bool i2_validStreamIdx = false;  // was the last stream valid or not
bool i2_validStream = false;  // Is the stream good enough to be used in the output?
uint8_t i2_pulseUpdIdx;       // index of the pulse in the currently received stream
uint8_t i2_PpmReceived = 0;    // how many streams received since last timer overflow

// Input Capture Timer pulses
bool i1_validStreamIdx = false; // was the last stream valid or not
bool i1_validStream = false;  // Is the stream good enough to be used in the output?
uint8_t i1_pulseUpdIdx;      // index of the pulse in the currently received stream
uint8_t i1_PpmReceived = 0;   // how many streams received since last timer overflow


uint16_t ppmOutputTiming[NUM_PPM_PULSES_OUTPUT] = { PPM_MIDLLE_TIMEOUT,PPM_MIDLLE_TIMEOUT,PPM_MIDLLE_TIMEOUT,PPM_MIDLLE_TIMEOUT,PPM_MIDLLE_TIMEOUT,PPM_MIDLLE_TIMEOUT,PPM_MIDLLE_TIMEOUT,PPM_MIDLLE_TIMEOUT,};

const uint8_t ppmMapI2toOut[8] = {0xFF, 0xFF, 0xFF, 0xFF, 5, 6, 7, 0xFF, }; // maps with interrupt captured signals are mapped to output, overriding IC signals
uint8_t ppmMaskI2toTX = 0; // this is updated when the FatShark input is received.


uint8_t i2_PpmValidity = 0;
uint8_t i1_PpmValidity = 0;

static const bool i1_inverted = true;
static const bool invertPpmOutput = false; // set this if the radio needs inverted output.


///////////////////////////////////////////////////////////////////////////////
//                           FUNCTION DEFINITIONS                            //
///////////////////////////////////////////////////////////////////////////////

//
// Validate the given pulse defined by (pt, pf).
// Check if that is the PPM reset pulse.
//
// Depending of the type of pulse (isI1) and it's length the concerning variables are updated.
// I1 pulses are copied to the output array, into the same channel that they came in.
// Also, the I2 pulses of which we care will be put to the output array - channel position depends on the given ppmMap.
//
inline void addPulseToArray(const uint8_t* ppmMap, uint8_t* idx, bool* validStreamIdx, uint16_t pt, uint16_t pf, bool isI1, uint8_t expectedPulsesPerStream)
{
  // is input capture timer pulse, or change interrupt pulse
  
  uint8_t newIdx;
  if(ppmMap == NULL) 
  {
  	// For the Input1 stream, i.e. the graupner radio
  	// do not change the signal index.
    newIdx = *idx;
  }
  else
  {
  	// For the Input2 stream, i.e. the fatshark / headtracker / ...
  	// check the index in output stream from the ppmMap.
  	// Also, check that the idx is small enough.
  	
    if(*idx < expectedPulsesPerStream)
    {
      newIdx = ppmMap[*idx];
    }
    else
    {
      PRINTLN("W1"); // Warning 1: invalid number of I2 pulses received.
      newIdx = PPM_MAP_INVALID_IDX;
    }
  }
  // if the full pulse length exceeds a certain value, we know that this is the "reset" pulse.
  if(pf > PPM_STREAM_RESET_FULL_TIMEOUT)
  {
    if(expectedPulsesPerStream != *idx)
    {
      *validStreamIdx = false; // invalidate the pulses.
      PRINT(*idx);
    }
    else
    {
      *validStreamIdx = true; // validate the pulses
      if(isI1)
      {
        i1_PpmReceived++;
      }
      else
      {
        i2_PpmReceived++;
      }
    }

    *idx = 0;
  }
  else // else this is a new pulse with a valid length
  {
    if(*validStreamIdx)
    {
      bool canCopy = false;
      if(isI1)
      {
        // I1 pulse can be copied, if the I2 pulses are not overriding them.
        if((ppmMaskI2toTX & (1 << newIdx)) == 0)
        {
          canCopy = true;
        }
      }
      else // this is an I2 pulse, which may be overriding an I1 pulse
      {
        // check if the I2 stream is valid
        // and that we care this pulse (new index not PPM_MAP_INVALID_IDX)
        if(i2_validStream  && (newIdx != PPM_MAP_INVALID_IDX))
        {
          canCopy = true;
          // mask the bit that the I2 overrides from the I1 channels, to not allow I1 to re-write it
          ppmMaskI2toTX |= (1 << newIdx);
        }
      }
      if(canCopy)
      {
        if(newIdx < NUM_PPM_PULSES_OUTPUT)
        {
          // copy given pulse time to the output timing array
          ppmOutputTiming[newIdx] = pt; 
        }
        else
        {
          PRINTLN("E3");
        }
      }
    }
    // so, we got a new pulse, increase the PulseIndex
    *idx = *idx + 1;  
  }
}

//
// Add the Change Interrupt based pulse
//
inline void addPulse(uint16_t pt,uint16_t pf)
{
  addPulseToArray(ppmMapI2toOut, &i2_pulseUpdIdx, &i2_validStreamIdx, pt, pf, false, NUM_PPM_PULSES_FATSHARK_HEADTRACK);
}

//
// Add the Input Capture based pulse
// 
inline void add_icPulse(uint16_t pt,uint16_t pf)
{
  addPulseToArray(NULL, &i1_pulseUpdIdx, &i1_validStreamIdx, pt, pf, true, NUM_PPM_PULSES_GRAUPNER_MC_12);
}

//////////////////////////////////////////////////////////////////////////
// CALC_PT_FT 
// a Macro to calculate PT and FT (pulse time, full time) of the measured pulses.
// pt and ft must be defined locally.
//
// upd: bool updated - this will be set to false by the macro
// start: uint32_t pulse start time
// e: uint32_t pulse end time
// ns: uint32_t start time of the next pulse
//////////////////////////////////////////////////////////////////////////
#define CALC_PT_FT(upd, start, e, ns)     upd = false; pt = e - start; ft = ns - start;

///
// Checks if the change interrupt routine has updated some of the variables.
// there are two sets of variables, to ensure we are not reading them
// while interrupt routine is changing them.
// 
void inline checkChangeInterruptPulse(void)
{
  static uint16_t pt; // pulse high-time
  static uint16_t ft; // full time, i.e. total length of the pulse

  appReading_0 = true; // lock the p0_ variables and read them.
  if(p0_updated)
  {
    CALC_PT_FT(p0_updated, p0_start, p0_end, p0_nextStart);
    addPulse(pt,ft);
  }
  appReading_0 = false; // unlcok
  appReading_1 = true; // lock the p1_ variables for reading
  if(p1_updated)
  {
    CALC_PT_FT(p1_updated, p1_start, p1_end, p1_nextStart);
    addPulse(pt,ft);
  }
  appReading_1 = false;
}


//
// The function checks if the Input Capture interrupt flag has been set.
// If it has been set, we know that there is a new value in the IC register.
// We'll read it, store it, and setup for the next interrupt event.
//
// The stored value will be given to addIcPulse function, which will make good use of it.
//
// NOTE: We should execute this function at least in 200us interval, to not miss any edges!
//
inline void checkInputCapture()
{
  static uint16_t ic_start; // start of the current pulse
  static uint16_t ic_rise;  // last seen rising signal
  static uint16_t ic_end;  // last seen rising signal

  bool ic_updated = false;
  
  if(TIFR1 & (1<<ICF1))
  {
    // record the stored time immediately    
    uint16_t t = ICR1;

    // change the waited edge value
    TCCR1B ^= (1<<ICES1);
    TIFR1 |= (1 << ICF1); // clear the interrupt bit, to detect the next

    bool lastEdge = !(bool)(TCCR1B & (1<<ICES1));

    if(i1_inverted) lastEdge = !lastEdge;

    if ( lastEdge == false) // if we were going down
    {
      ic_start = ic_rise; // start of this signal equals the end of the last full cycle
      ic_end = t;  //measure time between down and up
      ic_updated = false; // we'll be done after the end of the cycle is got (the next rise)
    }
    else 
    {
      ic_updated = true;
      ic_rise = t;
    }
  }
  if(ic_updated)
  {
    add_icPulse(ic_end - ic_start, ic_rise-ic_start);
  }
}

//
// Checks Timer1 OC interrupt flag. If it has been set, updates the next 
// interupt time from the preComputed value and sets how the output pin should be set (low or high).
//
// Also, preComputes the next interrupt time, so that after next interrupt
// the time doesn't need to be computed.
// 
inline void checkOutputCompare()
{
  //static bool pulseStarted = false;
  static uint8_t pulseIdx = NUM_PPM_PULSES_OUTPUT;
  static uint16_t nextPulseT = 0;
  static uint16_t oldNextPulseT = 0;
  static uint16_t endPulseLengthLeft = PPM_PULSE_STREAM_TOTAL_TIMEOUT;
  uint16_t timeWhenSet;
  bool pinIsHigh;

  if(!i1_validStream )
  {
    // prevent Timer1 module from setting the ppm output bit
    TCCR1A &= ~((1 << COM1A0) | (1 << COM1A1));
    // if not inverted, set ppm output high.
    // if inverted, put it low.
    digitalWrite(PpmOutputPin, !invertPpmOutput);
    pulseIdx = NUM_PPM_PULSES_OUTPUT; // this will generate the reset pulse first.
    noInterrupts();
    oldNextPulseT = OCR1A ;// fake current moment to be the last signal change moment
    interrupts();

    nextPulseT = oldNextPulseT + 200; 
    endPulseLengthLeft = PPM_PULSE_STREAM_TOTAL_TIMEOUT - (NUM_PPM_PULSES_OUTPUT * (PPM_MIDLLE_TIMEOUT + PPM_PULSE_LOW_TIMEOUT)); // compute length of the reset pulse

    return;
  }
  else
  {
    // just setting the bit is faster than testing it's value
    TCCR1A |= (1 << COM1A1); // A1 high tells that A0 sets the low-high at next compare match
  }
  
  if(TIFR1 & (1 << OCF1A))
  {
    noInterrupts();
    // start a block during which there may not be interrupts.
    {
      OCR1A = nextPulseT; // use the precalculated next pulse time
      TIFR1 |= (1 << OCF1A); // clear the interrupt flag, to detect the next compare match, too
  
      timeWhenSet = TCNT1; // read the timer counter value at the moment when we set the next compare target
      // Check if the current compare match was to start the pulse (set pin high)
      // or start the mid-pulses-low, ie set the pin low.
      if(TCCR1A & (1 << COM1A0))
      {
        // we just raised the pin high
        // tell the OC that next time we want to put it low
        TCCR1A &= ~ (1 << COM1A0);
        pinIsHigh = true;
      }
      else
      {
        // we just set the signal low - next time we must start the signal by raising it
        TCCR1A |= (1 << COM1A0);
        pinIsHigh = false;
      }
    }
    interrupts();

    if(invertPpmOutput) pinIsHigh = !pinIsHigh;

#ifdef DEBUG_TOTAL_PULSE_SET_TIME
    if(pinIsHigh && pulseIdx == 0)
    {
      static uint32_t lastStartUs = 0;
      uint32_t nowUs = micros();
      uint32_t delta = nowUs - lastStartUs;
      if(delta > 20500)
      {
        PRINTLN("ERROR");
        PRINTLN(delta);
        PRINTLN(pulseIdx);
        PRINTLN(pinIsHigh);
        PRINTLN(oldNextPulseT);
        PRINTLN(timeWhenSet);
      }
      lastStartUs = nowUs;
    }
#endif

    oldNextPulseT = nextPulseT;

    if(pinIsHigh)
    {
      // Pre-Compute the next interrupt time - the low time is always constant
      nextPulseT += PPM_PULSE_LOW_TIMEOUT;
    }
    else
    {
      // Pre-Compute the next interrupt time - the low time is always constant
      if(pulseIdx < NUM_PPM_PULSES_OUTPUT)
      {
        uint16_t highTime = ppmOutputTiming[pulseIdx];
        nextPulseT += highTime; // add high time to last pulse change time
        endPulseLengthLeft -= highTime;
        endPulseLengthLeft -= PPM_PULSE_LOW_TIMEOUT; // also take into accout the low time
      }
      else
      {
        // let's generate the longer reset pulse to mark the PPM stream end
        endPulseLengthLeft -= PPM_PULSE_LOW_TIMEOUT; // also leave space for the last low

        // pulse length validation
        if(endPulseLengthLeft > 25000)
        {
          endPulseLengthLeft = 25000;
          PRINT("E1"); // this is shown at startup for no good reason - as we didn't generate any pulses yet.
        }
        else if(endPulseLengthLeft < PPM_STREAM_RESET_FULL_TIMEOUT - PPM_PULSE_LOW_TIMEOUT)
        {
          endPulseLengthLeft = PPM_STREAM_RESET_FULL_TIMEOUT - PPM_PULSE_LOW_TIMEOUT;
          PRINT("E2");
        }
        else
        {
//          PRINTLN(endPulseLengthLeft);
        }
        nextPulseT += endPulseLengthLeft;
        endPulseLengthLeft = PPM_PULSE_STREAM_TOTAL_TIMEOUT;
      }
      

      // advance to next pulse, and check overflow
      pulseIdx ++;
      if(pulseIdx > NUM_PPM_PULSES_OUTPUT)
      {
        // note: for pulseIdx = 0..NUM_PPM_PULSES_OUTPUT-1 we generate normal pulse from ppmOutputTiming array
        //       for pulseIdx = NUM_PPM_PULSES_OUTPUT we generate the longer "reset" signal
        pulseIdx = 0;
      }
    }
//    PRINTLN(nextPulseT);
  }
}

inline void checkInputStatuses()
{
  // check the timer overflow interrupt flag
  // this overflow is happening at 65536/2 us interval => about 33ms.
  // Each PPM sequence should take no more than 20ms.
  // Therefore, we should have got at least one input sequence ready for each channel. 
  // If not, then there is something wrong.
  if(TIFR1 & (1<< TOV1))
  {
    TIFR1 |= (1 << TOV1); // reset the bit

    if(i1_PpmReceived)
    {
      if(i1_PpmValidity < PPM_VALIDITY_MAX)
      {
        i1_PpmValidity++;
        PRINT("P");
        if(i1_PpmValidity == PPM_VALIDITY_MAX)
        {
          PRINTLN("I1 VALIDATED!");
          i1_validStream = true;
        }
      }
      else
      {
        // make sure it is validated?
        i1_validStream = true;
      }
    }
    else
    {
      i1_validStreamIdx = false; // make sure we are not using the first incoming pulses after a glitch, as the first one we receive is probably not at index 0
      // if the validity > 0, decrease it
      if(i1_PpmValidity)
      {
        i1_PpmValidity --;
        PRINT("M");
        if(!i1_PpmValidity)
        {
          // if it become zero, print it.
          PRINTLN("I1 INVALIDATED!");
          i1_validStream = false;
        }
      }
    }
        
    if(i2_PpmReceived)
    {
      if(i2_PpmValidity < PPM_VALIDITY_MAX)
      {
        i2_PpmValidity++;
        PRINT("+");
        if(i2_PpmValidity == PPM_VALIDITY_MAX)
        {
          PRINTLN("I2 VALIDATED!");
          i2_validStream = true;
        }
      }
      else
      {
        // make sure it is validated? 
        i2_validStream = true;
      }
    }
    else // I2 not received
    {
      i2_validStreamIdx = false; // make sure we are not using the first incoming pulses after a glitch, as the first one we receive is probably not at index 0
      // if the validity > 0, decrease it
      if(i2_PpmValidity)
      {
        i2_PpmValidity --;
        PRINT("-");
        if(!i2_PpmValidity) // if it become zero
        {
          uint8_t i;
          for(i = 0; i < 8; i++)
          {
            if(ppmMaskI2toTX & (1 << i))
            {
              uint8_t k = ppmMapI2toOut[i];
              if(k < NUM_PPM_PULSES_OUTPUT) ppmOutputTiming[k] = PPM_MIDLLE_TIMEOUT; // set the servo to the middle, when removing the mapping
            }
          }
          // remove all channel from the mask list
          ppmMaskI2toTX = 0;
  
          PRINTLN("I2 INVALIDATED!");
          i2_validStream = false;
        }
      }
      
    }
    // clear the reception counter.
    i2_PpmReceived = 0;
    i1_PpmReceived = 0;
  }
}

inline void setup_timer1()
{
  OCR1A = 0x3D08;

  // clear all timer control regiusters
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  
  TCCR1A |= (1 << COM1A1); // OC1A == PB1 is used by the output compare - either to set or clear - defined by bit COM1A0
  TCCR1B |= (1 << ICNC1) | (1 << ICES1); // input capture filter + wait for the next positive (raising) edge
  
  // Setup interupt sources of the Timer1 - no interrupts
  TIMSK1 = 0;
  
  // set prescaler to 8 and start the timer
  TCCR1B |= (1 << CS11);
}

// Arduino Setup function
void setup(void) 
{
  // setup the interrupt to capture pulse length
  pinMode(pulsePin, INPUT);
  pinMode(InputCapturePin, INPUT);
  pinMode(GreenLedPin, OUTPUT);
  pinMode(PpmOutputPin, OUTPUT);
  
  // assign change interrupt to INT1, which is 'pulsePin'
  attachInterrupt(1, pulseInterruptHandler, CHANGE);  //interrupt always when pin state changes
  
  
  setup_timer1();

#ifdef ENABLE_SERIAL_PRINT
  Serial.begin(38400);
#endif // #ifdef ENABLE_SERIAL_PRINT
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
      PRINT(ppmOutputTiming[i]);
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

  // read the pulse times recorded in the change interrupt
  checkChangeInterruptPulse();

  checkInputCapture();
  checkOutputCompare(); // this is where we produce the PPM output - the most critical part in a sense
  
  checkInputStatuses();

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

//
// Interrupt handler. Listens for the pin going up or down, calculates pulse total length, and pulse high length.
//
void pulseInterruptHandler()
{
  static uint16_t t_fall = 0;  //Time of front falling
  static uint16_t t_rise = 0;  //Time of front raising
  static bool update_0;
  
  if ( digitalRead( pulsePin ) == LOW )
  {
    t_fall = TCNT1; //get time of pulse going down
    digitalWrite(GreenLedPin, HIGH);

    update_0 = !update_0;// switch to the other set
    if((update_0 && appReading_0) || (!update_0 && appReading_1))
    {
      update_0 = !update_0; // switch back, as app is just reading this one.
    }
    if(update_0)
    {
      p0_start = t_rise; // start
      p0_end = t_fall;  //measure time between down and up
      p0_updated = false; // we'll be done after the end of the cycle is got (the next rise)
    }
    else
    {
      p1_start = t_rise; // start
        p1_end = t_fall;  //measure time between down and up
      p1_updated = false; // we'll be done after the end of the cycle is got (the next rise)
    }
  }
  else // if(digitalRead( pulsePin ) == HIGH)
  {
    uint32_t t = TCNT1;  //get time of pulse going up
    digitalWrite(GreenLedPin, LOW);

    if(update_0)
    {
      p0_nextStart = t;
      p0_updated = true;
    }
    else
    {
      p1_nextStart = t;
      p1_updated = true;
    }
    t_rise = t;
  }
}  
 
 
 

