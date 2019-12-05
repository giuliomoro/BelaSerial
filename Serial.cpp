#include "Serial.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

Serial::Serial() {}
Serial::~Serial() {
	cleanup();
}

int Serial::setup (const char* device, unsigned int speed) {
	_handle = open(device, O_RDWR | O_NOCTTY | O_SYNC);
	if (_handle < 0) {
		fprintf(stderr, "Error opening %s: %s\n", device, strerror(errno));
		return -1;
	}

	setBaudRate(speed);
	if(B0 == _speed) {
		fprintf(stderr, "Error setting BAUD rate for %s: %d speed not supported\n", device, speed);
		return -1;
	}
	setInterfaceAttribs(_handle, _speed);
	setMinCount(_handle, 0); /* set to pure timed read */
	return 0;
}

void Serial::setBaudRate(unsigned int speed) {
	switch(speed) {
		case 50:    	_speed = B50;
		break;
		case 75:    	_speed = B75;
		break;
		case 110:   	_speed = B110;
		break;
		case 134:   	_speed = B134;
		break;
		case 150:   	_speed = B150;
		break;
		case 200:   	_speed = B200;
		break;
		case 300:   	_speed = B300;
		break;
		case 600:   	_speed = B600;
		break;
		case 1200:  	_speed = B1200;
		break;
		case 1800:  	_speed = B1800;
		break;
		case 2400:  	_speed = B2400;
		break;
		case 4800:  	_speed = B4800;
		break;
		case 9600:  	_speed = B9600;
		break;
		case 19200: 	_speed = B19200;
		break;
		case 38400: 	_speed = B38400;
		break;
		case 57600: 	_speed = B57600;
		break;
		case 115200:	_speed = B115200;
		break;
		case 230400:	_speed = B230400;
		break;
		case 0:
		//nobreak
		default:
		_speed = B0;
		break;
	}
}

int Serial::setInterfaceAttribs(int fd, int speed) {
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;      /* 8-bit characters */
	tty.c_cflag &= ~PARENB;  /* no parity bit */
	tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &=
	~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void Serial::setMinCount(int fd, int mcount) {
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5; /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0) {
		printf("Error tcsetattr: %s\n", strerror(errno));
	}
}


int Serial::read(char* buf, size_t len, int timeoutMs) {
	struct pollfd pfd[1];
	pfd[0].fd = _handle;
	pfd[0].events = POLLIN;
	// printf("before poll\n");
	int result = poll(pfd, 1, timeoutMs);
	if (result < 0) {
		fprintf(stderr, "Error polling for serial: %d %s\n", errno,
		strerror(errno));
		return errno;
	} else if (result == 0) {
		// timeout
		return 0;
	} else if (pfd[0].revents & POLLIN) {
		int rdlen = ::read(_handle, buf, len);
		if (rdlen < 0) {
			fprintf(stderr, "Error from read: %d: %s\n", rdlen, strerror(errno));
		}
		return rdlen;
	} else {
		fprintf(stderr, "unknown error while reading serial\n");
		return -1;
	}
}

int Serial::write(const char* buf, size_t len) {
	if(-1 == len)
		len = strlen(buf);
	int ret = ::write(_handle, buf, len);
	if (ret < 0) {
		fprintf(stderr, "write failed: %d %s\n", errno, strerror(errno));
	}
	return ret;
}

void Serial::cleanup() { close(_handle); }

