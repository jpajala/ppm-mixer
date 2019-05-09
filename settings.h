// settings.h

#ifndef _SETTINGS_h
#define _SETTINGS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define SETTINGS_PPM_INPUT_CHANNELS (8)
#define SETTINGS_PPM_OUTPUT_CHANNELS (8)

typedef struct  
{
	uint32_t size;										// size of this structure
	uint8_t channelMap[SETTINGS_PPM_INPUT_CHANNELS];
	uint16_t ppmOuputMin[SETTINGS_PPM_OUTPUT_CHANNELS];
	uint16_t ppmOuputMax[SETTINGS_PPM_OUTPUT_CHANNELS];
	int16_t ppmTrim[SETTINGS_PPM_OUTPUT_CHANNELS];
	uint32_t crc;										// this should be the last item
} MixerSettings;


class Settings
{
public: 
	MixerSettings data;
	Settings();
	Settings(int base, int memSize);
	bool load();
	bool save();
	
	void resetFactorySettings();
protected:
	uint32_t crc();
	uint32_t crc(MixerSettings* s);
	
	bool load(MixerSettings& s);
};

extern Settings settings; // this should be defined in the main .ino file.
#endif

