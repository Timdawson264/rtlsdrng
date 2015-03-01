#include <stdint.h>
#include <stdio.h>

//UNIX File
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//IOCTL
#include <sys/ioctl.h>
#include <usb.h>
#include <asm/byteorder.h>
#include "module/rtlsdr.h"

#define CTRL_IN		(USB_TYPE_VENDOR | USB_ENDPOINT_IN )
#define CTRL_OUT	(USB_TYPE_VENDOR | USB_ENDPOINT_OUT)
#define CTRL_TIMEOUT	300
#define BULK_TIMEOUT	0

enum usb_reg {
  USB_SYSCTL		= 0x2000,
  USB_CTRL		= 0x2010,
  USB_STAT		= 0x2014,
  USB_EPA_CFG		= 0x2144,
  USB_EPA_CTL		= 0x2148,
  USB_EPA_MAXPKT		= 0x2158,
  USB_EPA_MAXPKT_2	= 0x215a,
  USB_EPA_FIFO_CFG	= 0x2160,
};

enum sys_reg {
  DEMOD_CTL		= 0x3000,
  GPO			= 0x3001,
  GPI			= 0x3002,
  GPOE			= 0x3003,
  GPD			= 0x3004,
  SYSINTE			= 0x3005,
  SYSINTS			= 0x3006,
  GP_CFG0			= 0x3007,
  GP_CFG1			= 0x3008,
  SYSINTE_1		= 0x3009,
  SYSINTS_1		= 0x300a,
  DEMOD_CTL_1		= 0x300b,
  IR_SUSPEND		= 0x300c,
};

enum blocks {
  DEMODB			= 0,
  USBB			= 1,
  SYSB			= 2,
  TUNB			= 3,
  ROMB			= 4,
  IRB			= 5,
  IICB			= 6,
};

int rtlsdr_control_transfer ( int fd, uint8_t  	bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
                              unsigned char* data, uint16_t wLength, unsigned int  timeout )
{
  struct rtlsdr_ctrltransfer crt_trf = {
        .bRequestType = bmRequestType,
        .bRequest = bRequest,
        .wValue = wValue,
        .wIndex = wIndex,
        .wLength = wLength,
        .timeout = timeout,  /* in milliseconds */
        .data = data,
  };

  return ioctl(fd, RTLSDR_CONTROL, &crt_trf);

}

//TODO: implement rtlsdr_reset(fd)

int rtlsdr_write_reg( int fd, uint8_t block, uint16_t addr, uint16_t val, uint8_t len )
{
  int r;
  unsigned char data[2];

  uint16_t index = ( block << 8 ) | 0x10;

  if ( len == 1 ) {
    data[0] = val & 0xff;
  } else {
    data[0] = val >> 8;
  }

  data[1] = val & 0xff;

  r = rtlsdr_control_transfer( fd, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT );

  if ( r < 0 ) {
    fprintf( stderr, "%s failed with %d\n", __FUNCTION__, r );
  }

  return r;
}


int main( int argc, char** argv )
{
  int rtlsdr0 = open("/dev/rtlsdr0", O_RDWR | O_SYNC);
  
  /* perform a dummy write, if it fails, reset the device */
  if ( rtlsdr_write_reg( rtlsdr0, USBB, USB_SYSCTL, 0x09, 1 ) < 0 ) {
    fprintf( stderr, "Resetting device...\n" );
    //libusb_reset_device(dev->devh);
  }
}
