#include <Bela.h>
#include <libraries/Pipe/Pipe.h>
#include <cmath>

#include "Serial.h"

Serial gSerial;
Pipe gPipe;
unsigned int gSnareDuration;
int gSnareTime = 0;
unsigned int gKickDuration;
int gKickTime = 0;
float gKickPhase = 0;

void serialIo(void* arg) {
	while(!gShouldStop)
	{
		unsigned int maxLen = 128;
		char serialBuffer[maxLen];
		// read from the serial port with a timeout of 100ms
		int ret = gSerial.read(serialBuffer, maxLen, 100);
		if (ret > 0) {
			printf("Received: %.*s\n", ret, serialBuffer);
			for(unsigned int n = 0; n < ret; ++n)
			{
				// when some relevant data is received
				// send a reply on the serial port and
				// pass the data to the audio thread via the pipe
				if('k' == serialBuffer[n])
				{
					gPipe.writeNonRt('k');
					gSerial.write("kick!\n\r");
				} else if('s' == serialBuffer[n])
				{
					gPipe.writeNonRt('s');
					gSerial.write("snare!\n\r");
				}
			}
		}
	}
}

bool setup(BelaContext *context, void *userData) {
	gSerial.setup ("/dev/ttyS4", 115200);
	AuxiliaryTask serialCommsTask = Bela_createAuxiliaryTask(serialIo, 0, "serial-thread", NULL); 
	Bela_scheduleAuxiliaryTask(serialCommsTask);

	gPipe.setup("serialpipe", 1024);
	gKickDuration = 0.2 * context->audioSampleRate;
	gSnareDuration = 0.2 * context->audioSampleRate;

	return true;
}

void render(BelaContext *context, void *userData) {
	char c;
	// check if the serial thread has sent any message
	while(gPipe.readRt(c) > 0)
	{
		// if there is, trigger the start of the respective sound
		if('s' == c)
			gSnareTime = gSnareDuration;
		if('k' == c)
			gKickTime = gKickDuration;
	}
	for(unsigned int n = 0; n < context->audioFrames; ++n)
	{
		// synthesize the snare and kick
		float snareOut = 0;
		float kickOut = 0;
		if(gSnareTime)
		{
			// just a burst of white noise with a decaying envelope
			float noise = 2.f * (rand() / (float)RAND_MAX) - 1.f;
			float env = gSnareTime / (float)gSnareDuration;
			snareOut = 0.4f * noise * env;
			--gSnareTime;
		}
		if(gKickTime)
		{
			// a descending sinewave
			float frequency = map(gKickTime, gKickDuration, 0, 150, 20);
			float env = gKickTime / (float)gKickDuration;
			gKickPhase += 2.f * (float)M_PI * frequency / context->audioSampleRate;
			if(gKickPhase > M_PI)
				gKickPhase -= 2.f * (float)M_PI;
			kickOut = env * sinf(gKickPhase);
			--gKickTime;
		}
		float out = snareOut + kickOut;
		for(unsigned int ch = 0; ch < context->audioOutChannels; ++ch)
		{
			audioWrite(context, n, ch, out);
		}
	}
}

void cleanup(BelaContext *context, void *userData) {}
