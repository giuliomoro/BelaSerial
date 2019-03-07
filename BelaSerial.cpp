/*
 * BelaSerial.cpp
 */

#include "BelaSerial.h"

static int _handle;

// void BelaSerial::setPostCallback(void(*postCallback)(void* arg, float*
// buffer, unsigned int length), void* arg);

BelaSerial::BelaSerial() {}
BelaSerial::~BelaSerial() {
	cleanup();
}

bool BelaSerial::setup(const char* device, unsigned int speed) {
  return !init(device, speed);
}

int BelaSerial::init (const char* device, unsigned int speed) {
  printf("Attempting to connect to %s\n", device);
  _handle = open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (_handle < 0) {
    printf("Error opening %s: %s\n", device, strerror(errno));
    return -1;
  } else {
    printf("Successfully opened %s with file descriptor %d\n", device, _handle);
  }

  setBaudRate(speed);
  setInterfaceAttribs(_handle, _speed);
  setMinCount(_handle, 0); /* set to pure timed read */
  return 0;
}

void BelaSerial::setBaudRate (unsigned int speed) {
	switch(speed) {
		case 0:     	_speed = B0;
									break;
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
	}
}

int BelaSerial::setInterfaceAttribs(int fd, int speed) {
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

void BelaSerial::setMinCount(int fd, int mcount) {
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


int BelaSerial::serialRead(char* buf, size_t len, int timeoutMs) {
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
    printf("Timeout\n");
    // timeout
    return 0;
  } else if (pfd[0].revents & POLLIN) {
    // printf("before read\n");
    int rdlen = read(_handle, buf, len);
    // printf("after read\n");
    if (rdlen > 0) {
      if (1 || rdlen != 5) {
        printf("Serial read %d bytes: ", rdlen);
        for (unsigned int n = 0; n < rdlen; ++n) {
          printf("%03d ", buf[n]);
        }
        printf("\n");
      }
    } else if (rdlen < 0) {
      fprintf(stderr, "Error from read: %d: %s\n", rdlen, strerror(errno));
    }
    return rdlen;
  } else {
    fprintf(stderr, "unknown error while reading serial\n");
    return -1;
  }
}

int BelaSerial::serialWrite(const char* buf, size_t len) {
  if (0) {
    printf("Writing %d bytes: ", len);
    for (unsigned int n = 0; n < len; ++n) printf("%d ", buf[n]);
    printf("\n");
  }
  int ret = write(_handle, buf, len);
  if (ret < 0) {
    fprintf(stderr, "write failed: %d %s\n", errno, strerror(errno));
  }
  return ret;
}

void BelaSerial::cleanup() { close(_handle); }

