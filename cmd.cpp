///////////////////////////////////////////////////////////////////////////////
//
//
//                  PPM Mixer
//
//
//                  (c) Jussi Pajala 2016-2017
//
///////////////////////////////////////////////////////////////////////////////

#include "settings.h"
#include "ppm.h"
#include "cmd.h"

#define NUM_PARAMS_MAX (4)

typedef bool (*CmdHandler)(char* params[]);

typedef struct
{
	const char* cmd;
	CmdHandler cmdHandler;
} CmdEntry;

// Command callback functions:
bool saveSettings(char** params);
bool printSettings(char** params);
bool map(char** params);
bool channelMax(char** params);
bool channelMin(char** params);
bool help(char** params);

static const CmdEntry cmdEntries[] = 
{
	{ "save", saveSettings },
	{ "printset", printSettings},
	{ "map", map},
	{ "c+", channelMax}, // channel positive limit (maximum) 
	{ "c-", channelMin}, // channel negative limit (minimum) 
	{ "help", help},
};



Cmd::Cmd()
{
	
}

const char* Cmd::getCurrCmd()
{
	return currCmd;	
}

void Cmd::resetRecvBuf()
{
	memset(recvBuf, 0, sizeof(recvBuf));
	recvBufIdx = 0;
}

void Cmd::init()
{
	resetRecvBuf();
	cmdOngoing = false;
}

void Cmd::execute()
{
	if(cmdOngoing)
	{
		Serial.write("ongoing");
		executeOngoing();
		return;
	}
	int avail = Serial.available();

	while(avail)
	{
		uint8_t c = Serial.read();
		avail--;
		Serial.write(c);
		
		if(recvBufIdx == sizeof(recvBuf))
		{
			resetRecvBuf(); // re-initialize the buffer.
		}
		
		
		recvBuf[recvBufIdx] = c;
		recvBufIdx++;
		
		if(c == '\r' || c == '\n')
		{
			if(recvBufIdx == 1)
			{
				ppm_printUnfiltOutput();
			}
			else
			{
				Serial.println();
				handleCmd();
			}
			resetRecvBuf();
		}
	}
}

void Cmd::executeOngoing()
{
	cmdOngoing = false;
}

void Cmd::handleCmd()
{
	const char* tokens = " \n\r";
	char* params[NUM_PARAMS_MAX] =  {NULL, NULL, NULL, NULL};
	char* tok;
	char* cmd;
	char* savePtr;
	uint8_t numParams = 0;
	uint16_t i;
	
	for(i = 0; i < sizeof(recvBuf); i++)
	{
		if(recvBuf[i] == '\r' || recvBuf[i] == '\n') recvBuf[i] =  ' ';
	}
	
	
	cmd = strtok_r(recvBuf, tokens, &savePtr);
	
	if(!cmd) return;
	
	strncpy(currCmd, cmd, sizeof(currCmd));
	
	
	tok = cmd;
	
	while((tok = strtok_r(NULL, tokens, &savePtr)) && (numParams < sizeof(params)/sizeof(params[0])))
	{
		params[numParams] = tok;
		numParams++;
	}
	
	for(i = 0; i < sizeof(cmdEntries)/sizeof(cmdEntries[0]); i++)
	{
		if(strlen(cmd) == strlen(cmdEntries[i].cmd) && strcasestr(cmdEntries[i].cmd, cmd) != NULL)
		{
			cmdEntries[i].cmdHandler((char**)&params);
		}
	}
}

// Command callback functions:
bool saveSettings(char** params)
{
	(void)params; // remove "unused parameter" warning
	Serial.println("In Save Settings");

	if(settings.save())
	{
		Serial.println("Save Success");
		return true;
	}
	else
	{
		Serial.println("Save FAILED!");
	}
	return false;
}

void printChannelMap()
{
	int i;
	Serial.println("Channel mapping from I2 to output:");
	for(i = 0; i < SETTINGS_PPM_INPUT_CHANNELS;i++)
	{
		Serial.print(i+1);
		Serial.print(": ");
		if(settings.data.channelMap[i] == 0xFF)
		{
			Serial.println("-");
		}
		else
		{
			Serial.println((uint8_t)settings.data.channelMap[i]+1);
		}
	}
}

bool printSettings(char** params)
{
	int i;
	(void)params; // remove "unused parameter" warning
	MixerSettings& data = settings.data;
	
	Serial.print("size: ");
	Serial.println(data.size);

	printChannelMap();	
	Serial.println();
	
	Serial.println("Channel limits:");
	for(i = 0; i < SETTINGS_PPM_INPUT_CHANNELS; i++)
	{
		Serial.print(i+1);
		Serial.print(": [");
		Serial.print(data.ppmOuputMin[i]);
		Serial.print(", ");
		Serial.print(data.ppmOuputMax[i]);
		Serial.println("]");
	}
	Serial.print("CRC: ");
	Serial.println(data.crc);
	return true;
}

/**
 * View or Change mapping of the input channels to output stream.
 * 
 * The mapping tells where input2 aka fatshark signals are mapped in the output stream.
 * Example commands strings:
 * 
 * "map"               print current output mapping
 * "map 6 5"           map input channel 6 to output channel 5
 * "map 7 255"         remove input channel 7 from output PPM stream
 */
bool map(char** params)
{
	unsigned int from, to;

	if(params[0] == NULL || params[1] == NULL)
	{
		Serial.println("USAGE: map [from] [to] (i2idx:outidx)");
		return false;
	}
	from = atoi(params[0]) - 1;
	to = atoi(params[1]) - 1;
	
	if((from < SETTINGS_PPM_INPUT_CHANNELS ) && (to < SETTINGS_PPM_OUTPUT_CHANNELS || to == 0xFE))
	{
		if(to == 0xFE) to++;
		settings.data.channelMap[from] = to;

		printChannelMap();
		
		return true;
	}
	else
	{
		Serial.println("Invalid ch numbers. From = 1..8");
	}
	return false;
}

/**
 * Common function for parsing both min and max channel settings.
 * Parses the input, and saves it to settings structure.
 * Channel index value must be 1..8.
 * Channel min and max value is in milliseconds.
 * 
 * "c-" or "c+"           get the usage help.
 * "c- 4 1000"            set output stream channel 4 minimum value = 1000
 * "c+ 7 2000"            set output stream channel 7 max value = 2000
 * 
 * 
 * @param params user inputted parameters (channel + value)
 * @param min true if this is channel minimum, false if channel maximum value
 */
bool channelMinMax(char** params, bool min)
{
	uint16_t ch = -1;
	uint16_t value = -1;
	bool ok = true;

	if(params[0] == NULL)
	{
		Serial.print("Usage: ");
		Serial.print(cmd.getCurrCmd());
		Serial.println(" [ch] [value] - value optional, current unfiltered used if omitted.");		
		ok = false;
	}
	if(ok)
	{
		ch = atoi(params[0])-1;
		ok = ch < SETTINGS_PPM_OUTPUT_CHANNELS;
		Serial.print("ch: ");Serial.println(ch+1);
	}
	if(ok)
	{
		if(params[1] != NULL)
		{
			value = atoi(params[1]);
			ok = (value > 900) && (value < 3100);
			Serial.println(ok);
		}
		else
		{
			value = ppm_ChValUnFilt(ch);
		}
		Serial.print("value ");Serial.println(value);
	}
	if(ok)
	{
		Serial.print("Channel ");
		Serial.print(ch+1);
		Serial.print(" value ");
		Serial.print(value);
		
		if(min)
		{
			settings.data.ppmOuputMin[ch] = value;
		}		
		else
		{
			settings.data.ppmOuputMax[ch] = value;
		}
	}
	return ok;
}

bool channelMax(char** params) // channel positive limit (maximum)
{
	return channelMinMax(params, false);
}

bool channelMin(char** params) // channel negative limit (minimum)
{
	return channelMinMax(params, true);	
}

// Command callback functions:
bool help(char** params)
{
	uint8_t i;
	(void)params; // remove "unused parameter" warning
	for(i = 0; i < sizeof(cmdEntries)/sizeof(cmdEntries[0]); i++)
	{
		Serial.println(cmdEntries[i].cmd);
	}
	return true;
}
