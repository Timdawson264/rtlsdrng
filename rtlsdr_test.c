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

#define R82XX_IF_FREQ 3570000
#define FIR_LEN 16
#define TWO_POW(n)		((double)(1ULL<<(n)))

/*
 * FIR coefficients.
 *
 * The filter is running at XTal frequency. It is symmetric filter with 32
 * coefficients. Only first 16 coefficients are specified, the other 16
 * use the same values but in reversed order. The first coefficient in
 * the array is the outer one, the last, the last is the inner one.
 * First 8 coefficients are 8 bit signed integers, the next 8 coefficients
 * are 12 bit signed integers. All coefficients have the same weight.
 *
 * Default FIR coefficients used for DAB/FM by the Windows driver,
 * the DVB driver uses different ones
 */
static int fir_default[FIR_LEN] = {
	-54, -36, -41, -40, -32, -14, 14, 53,	/* 8 bit signed */
	101, 156, 215, 273, 327, 372, 404, 421	/* 12 bit signed */
};


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

uint16_t rtlsdr_demod_read_reg(int fd, uint8_t page, uint16_t addr, uint8_t len)
{
	int r;
	unsigned char data[2];

	uint16_t index = page;
	uint16_t reg;
	addr = (addr << 8) | 0x20;

	r = rtlsdr_control_transfer(fd, CTRL_IN, 0, addr, index, data, len, CTRL_TIMEOUT);

	if (r < 0)
		fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

	reg = (data[1] << 8) | data[0];

	return reg;
}

int rtlsdr_demod_write_reg(int fd, uint8_t page, uint16_t addr, uint16_t val, uint8_t len)
{
	int r;
	unsigned char data[2];
	uint16_t index = 0x10 | page;
	addr = (addr << 8) | 0x20;

	if (len == 1)
		data[0] = val & 0xff;
	else
		data[0] = val >> 8;

	data[1] = val & 0xff;

	r = rtlsdr_control_transfer(fd, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

	if (r < 0)
		fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

	rtlsdr_demod_read_reg(fd, 0x0a, 0x01, 1);

	return (r == len) ? 0 : -1;
}

int rtlsdr_set_testmode(int fd, int on)
{
	if (!fd)
		return -1;

	return rtlsdr_demod_write_reg(fd, 0, 0x19, on ? 0x03 : 0x05, 1);
}

int rtlsdr_reset_buffer(int fd)
{
	rtlsdr_write_reg(fd, USBB, USB_EPA_CTL, 0x1002, 2);
	rtlsdr_write_reg(fd, USBB, USB_EPA_CTL, 0x0000, 2);

	return 0;
}

int rtlsdr_set_fir(int fd)
{
	uint8_t fir[20];

	int i;
	/* format: int8_t[8] */
	for (i = 0; i < 8; ++i) {
		const int val = fir_default[i];
		if (val < -128 || val > 127) {
			return -1;
		}
		fir[i] = val;
	}
	/* format: int12_t[8] */
	for (i = 0; i < 8; i += 2) {
		const int val0 = fir_default[8+i];
		const int val1 = fir_default[8+i+1];
		if (val0 < -2048 || val0 > 2047 || val1 < -2048 || val1 > 2047) {
			return -1;
		}
		fir[8+i*3/2] = val0 >> 4;
		fir[8+i*3/2+1] = (val0 << 4) | ((val1 >> 8) & 0x0f);
		fir[8+i*3/2+2] = val1;
	}

	for (i = 0; i < (int)sizeof(fir); i++) {
		if (rtlsdr_demod_write_reg(fd, 1, 0x1c + i, fir[i], 1))
				return -1;
	}

	return 0;
}


void rtlsdr_init_baseband(int fd)
{
	unsigned int i;

	/* initialize USB */
	rtlsdr_write_reg(fd, USBB, USB_SYSCTL, 0x09, 1);
	rtlsdr_write_reg(fd, USBB, USB_EPA_MAXPKT, 0x0002, 2);
	rtlsdr_write_reg(fd, USBB, USB_EPA_CTL, 0x1002, 2);

	/* poweron demod */
	rtlsdr_write_reg(fd, SYSB, DEMOD_CTL_1, 0x22, 1);
	rtlsdr_write_reg(fd, SYSB, DEMOD_CTL, 0xe8, 1);

	/* reset demod (bit 3, soft_rst) */
	rtlsdr_demod_write_reg(fd, 1, 0x01, 0x14, 1);
	rtlsdr_demod_write_reg(fd, 1, 0x01, 0x10, 1);

	/* disable spectrum inversion and adjacent channel rejection */
	rtlsdr_demod_write_reg(fd, 1, 0x15, 0x00, 1);
	rtlsdr_demod_write_reg(fd, 1, 0x16, 0x0000, 2);

	/* clear both DDC shift and IF frequency registers  */
	for (i = 0; i < 6; i++)
		rtlsdr_demod_write_reg(fd, 1, 0x16 + i, 0x00, 1);

	rtlsdr_set_fir(fd);

	/* enable SDR mode, disable DAGC (bit 5) */
	rtlsdr_demod_write_reg(fd, 0, 0x19, 0x05, 1);

	/* init FSM state-holding register */
	rtlsdr_demod_write_reg(fd, 1, 0x93, 0xf0, 1);
	rtlsdr_demod_write_reg(fd, 1, 0x94, 0x0f, 1);

	/* disable AGC (en_dagc, bit 0) (this seems to have no effect) */
	rtlsdr_demod_write_reg(fd, 1, 0x11, 0x00, 1);

	/* disable RF and IF AGC loop */
	rtlsdr_demod_write_reg(fd, 1, 0x04, 0x00, 1);

	/* disable PID filter (enable_PID = 0) */
	rtlsdr_demod_write_reg(fd, 0, 0x61, 0x60, 1);

	/* opt_adc_iq = 0, default ADC_I/ADC_Q datapath */
	rtlsdr_demod_write_reg(fd, 0, 0x06, 0x80, 1);

	/* Enable Zero-IF mode (en_bbin bit), DC cancellation (en_dc_est),
	 * IQ estimation/compensation (en_iq_comp, en_iq_est) */
	rtlsdr_demod_write_reg(fd, 1, 0xb1, 0x1b, 1);

	/* disable 4.096 MHz clock output on pin TP_CK0 */
	rtlsdr_demod_write_reg(fd, 0, 0x0d, 0x83, 1);
}

void rtlsdr_set_i2c_repeater(int fd, int on)
{
	rtlsdr_demod_write_reg(fd, 1, 0x01, on ? 0x18 : 0x10, 1);
}

int rtlsdr_set_if_freq(int fd, uint32_t freq)
{
	uint32_t rtl_xtal;
	int32_t if_freq;
	uint8_t tmp;
	int r;

  rtl_xtal = 28800000;

	/* read corrected clock value */
	//if (rtlsdr_get_xtal_freq(fd, &rtl_xtal, NULL))
	//	return -2;

	if_freq = ((freq * TWO_POW(22)) / rtl_xtal) * (-1);

	tmp = (if_freq >> 16) & 0x3f;
	r = rtlsdr_demod_write_reg(fd, 1, 0x19, tmp, 1);
	tmp = (if_freq >> 8) & 0xff;
	r |= rtlsdr_demod_write_reg(fd, 1, 0x1a, tmp, 1);
	tmp = if_freq & 0xff;
	r |= rtlsdr_demod_write_reg(fd, 1, 0x1b, tmp, 1);

	return r;
}

int rtlsdr_set_sample_rate(int fd, uint32_t samp_rate)
{
	int r = 0;
	uint16_t tmp;
	uint32_t rsamp_ratio, real_rsamp_ratio;
	double real_rate;

	/* check if the rate is supported by the resampler */
	if ((samp_rate <= 225000) || (samp_rate > 3200000) ||
	   ((samp_rate > 300000) && (samp_rate <= 900000))) {
		fprintf(stderr, "Invalid sample rate: %u Hz\n", samp_rate);
		return -1;
	}

	rsamp_ratio = (28800000 * TWO_POW(22)) / samp_rate;
	rsamp_ratio &= 0x0ffffffc;

	real_rsamp_ratio = rsamp_ratio | ((rsamp_ratio & 0x08000000) << 1);
	real_rate = (28800000 * TWO_POW(22)) / real_rsamp_ratio;

	if ( ((double)samp_rate) != real_rate )
		fprintf(stderr, "Exact sample rate is: %f Hz\n", real_rate);
/*
	if (dev->tuner && dev->tuner->set_bw) {
		rtlsdr_set_i2c_repeater(dev, 1);
		dev->tuner->set_bw(fd, (int)real_rate);
		rtlsdr_set_i2c_repeater(dev, 0);
	}
*/
	//dev->rate = (uint32_t)real_rate;

	tmp = (rsamp_ratio >> 16);
	r |= rtlsdr_demod_write_reg(fd, 1, 0x9f, tmp, 2);
	tmp = rsamp_ratio & 0xffff;
	r |= rtlsdr_demod_write_reg(fd, 1, 0xa1, tmp, 2);

	//r |= rtlsdr_set_sample_freq_correction(dev, dev->corr);

	/* reset demod (bit 3, soft_rst) */
	r |= rtlsdr_demod_write_reg(fd, 1, 0x01, 0x14, 1);
	r |= rtlsdr_demod_write_reg(fd, 1, 0x01, 0x10, 1);

	/* recalculate offset frequency if offset tuning is enabled */
	//if (dev->offs_freq)
	//	rtlsdr_set_offset_tuning(dev, 1);

	return r;
}

#define BUFFERSIZE 1024*64
int main( int argc, char** argv )
{
  ssize_t s=1;
  size_t i,loss;
  uint8_t * buffer;//[BUFFERSIZE];
  uint8_t byte,flag;
  
  int rtlsdr0 = open("/dev/rtlsdr0", O_RDWR|O_ASYNC);
  if(rtlsdr0 == -1){
    fprintf(stderr ,"Error opening device\n");
    return 1;
  } 

  buffer = malloc(BUFFERSIZE);
    
  /* perform a dummy write, if it fails, reset the device */
  if ( rtlsdr_write_reg( rtlsdr0, USBB, USB_SYSCTL, 0x09, 1 ) < 0 ) {
    fprintf( stderr, "Resetting device...\n" );
    //libusb_reset_device(dev->devh);
  }
  rtlsdr_init_baseband(rtlsdr0);

  rtlsdr_set_i2c_repeater(rtlsdr0, 1);
  rtlsdr_demod_write_reg(rtlsdr0, 1, 0xb1, 0x1a, 1);
  /* only enable In-phase ADC input */
  rtlsdr_demod_write_reg(rtlsdr0, 0, 0x08, 0x4d, 1);
  /* the R82XX use 3.57 MHz IF for the DVB-T 6 MHz mode, and
   * 4.57 MHz for the 8 MHz mode */
  rtlsdr_set_if_freq(rtlsdr0, R82XX_IF_FREQ);
  /* enable spectrum inversion */
  rtlsdr_demod_write_reg(rtlsdr0, 1, 0x15, 0x01, 1);
  rtlsdr_set_i2c_repeater(rtlsdr0, 0);

  rtlsdr_set_sample_rate(rtlsdr0, 2000000);
  rtlsdr_set_testmode(rtlsdr0, 1);
  rtlsdr_reset_buffer(rtlsdr0);

  loss=0;
  byte=0;
  s=0;
  
  while(s>=0){
    s=read(rtlsdr0, buffer, BUFFERSIZE);
    if(s<0)continue;
    
    for(i=0;i<s;i++){
      fprintf(stdout,"%u ",buffer[i]);
      if(buffer[i]!=0)flag=1;
      
      if(buffer[i]!=byte){
        fprintf(stderr,"DataLoss: %u,%u:%hhu\n",loss, i, buffer[i]);
        loss++;
        byte=buffer[i]; //reset progress
      }
      byte++;
    }
    //if(!flag) fprintf(stderr,"FLAG: %u\n", flag);
  }
}
