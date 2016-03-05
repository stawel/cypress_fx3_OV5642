#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <libusb-1.0/libusb.h>

#include "cyusb.h"

static cyusb_handle *h;

static unsigned char buf[128*1024];

static void control_transfer_read(int i2c, int addr)
{
	unsigned short wValue = i2c;
	cyusb_control_transfer(h, 0xC0, 0xBB, wValue, addr, buf, 0, 1000);
}

static void control_transfer_write(int i2c, int addr, int value)
{
	unsigned short wValue = i2c + (value << 8);
	cyusb_control_transfer(h, 0x40, 0xBA, wValue, addr, buf, 0, 1000);
}


int parseInt(char * s)
{
	int x, size;
	if(s[0] == '0' && s[1] == 'x') {
		size = sscanf(s+2, "%x", &x);
	} else {
		size = sscanf(s, "%d", &x);
	}
	if(size != 1) {
		printf("parse error: %s\n",s);
		exit(1);
	}
	return x;
}

int main(int argc, char **argv)
{
	int N;

	N = cyusb_open();
	if ( N < 0 ) {
	   printf("Error in opening library\n");
	   return -1;
	}
	else if ( N == 0 ) {
		printf("No device of interest found\n");
		return 0;
	}
	else  printf("No of devices of interest found = %d\n",N);

	h = cyusb_gethandle(0);
	if ( !h ) {
	   printf("Error obtaining handle\n");
	   return -1;
	}
	else printf("Successfully obtained handle\n");

	if ( argc != 4 &&  argc != 5) {
	   printf("Usage: fx3_i2ctest <R|W> <i2c_addr> <addr> [value]\n");
	   printf("argc = %d\n", argc);
	   return 0;
	}

	int i2c, addr, value = 0;
	i2c = parseInt(argv[2]);
	addr = parseInt(argv[3]);
	if(argc == 5) {
		value = parseInt(argv[4]);
	}

	printf("i2c: %02x addr: %04x value: %02x\n", i2c, addr, value);

	if ( argv[1][0] == 'W' ) {
	      control_transfer_write(i2c, addr, value);
	   printf("Completed writing\n");
	} else {
	   control_transfer_read(i2c, addr);
	   printf("Completed reading\n");
	}
	cyusb_close();
	return 0;
}
