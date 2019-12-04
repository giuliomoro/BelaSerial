#pragma once

#include <stddef.h>

class Serial {
public:
	Serial();
	~Serial();

	int setup(const char* device, unsigned int speed);
	int read(char* buf, size_t len, int timeoutMs);
	int write(const char* buf, size_t len = -1);
	void cleanup();

private:
	void setBaudRate (unsigned int speed);
	void setMinCount (int fd, int mcount);
	int setInterfaceAttribs (int fd, int speed);
	unsigned int _speed;
	int _handle;
};
