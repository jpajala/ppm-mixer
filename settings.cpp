#include <EEPROMex.h>
#include <EEPROMVar.h>

// 
// 
// 
#include "settings.h"


static const MixerSettings factorySettings = 
{
	sizeof(MixerSettings), // size;										// size of this structure
	{0xFF, 0xFF, 0xFF, 0xFF, 5, 6, 7, 0xFF, },	// uint8_t channelMap[SETTINGS_PPM_INPUT_CHANNELS];
	{1200, 1200, 1200, 1200,1200, 1200, 1200, 1200},	// uint16_t ppmOuputMin[SETTINGS_PPM_OUTPUT_CHANNELS];
	{3200, 3200, 3200, 3200, 3200, 3200, 3200, 3200},	// uint16_t ppmOuputMax[SETTINGS_PPM_OUTPUT_CHANNELS];
	0	// uint32_t crc;										// this should be the last item
};

Settings::Settings()
	: Settings(0, EEPROMSizeATmega328)
	{}
		
Settings::Settings(int base, int size)
{
	memset(&data, 0, sizeof(MixerSettings));
	EEPROM.setMemPool(base, size);
}

bool Settings::load()
{
	if(!load(data))
	{
		memcpy(&data, &factorySettings, sizeof(data));
		return false;
	}
	return true;
}


bool Settings::save()
{
	MixerSettings tempSettings;
	// first update the size and the CRC value of the structure to be written.
	data.size = sizeof(data);
	data.crc = crc();
	EEPROM.updateBlock(0, data);

	// check saving by loading the settings to a temporary setting and calculating the crc.
	return load(tempSettings);
}
	
void Settings::resetFactorySettings()
{
	
	
}
uint32_t Settings::crc()
{
	return crc(&data);

}
uint32_t Settings::crc(MixerSettings* s)
{
	static const uint32_t crc_table[16] = {
		0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
		0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
		0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
		0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
	};

	uint32_t mycrc = ~0L;
	uint8_t* ptr = (uint8_t*)s;
	uint16_t len = sizeof(*s) - sizeof(s->crc);
	for (uint16_t index = 0 ; index < len  ; ++index) {
		mycrc = crc_table[(mycrc ^ ptr[index]) & 0x0f] ^ (mycrc >> 4);
		mycrc = crc_table[(mycrc ^ (ptr[index] >> 4)) & 0x0f] ^ (mycrc >> 4);
		mycrc = ~mycrc;
	}
	return mycrc;
}

bool Settings::load(MixerSettings& s)
{
	EEPROM.readBlock(0, s);

	return (data.crc == crc(&s));
}
	


