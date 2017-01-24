// cmd.h

#ifndef _CMD_h
#define _CMD_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define CMD_RECV_BUF_SIZE 20

class Cmd
{
public:
	Cmd();
	void init();
	void execute();
	const char* getCurrCmd();
protected:
	char recvBuf[CMD_RECV_BUF_SIZE];
	uint16_t recvBufIdx;
	bool cmdOngoing;
	char currCmd[10];
	
	void resetRecvBuf();
	void executeOngoing();
	void handleCmd();
};

extern Cmd cmd;

#endif

