#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>

#include "rtl-sdr.h"


int main(int argc, char **argv){
    int r;
    uint32_t devidx;
    rtlsdr_dev_t* rtldev;
 
    r = rtlsdr_open(&rtldev, 0); //Open First Dongle
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", devidx);
		exit(1);
	}
    uint32_t rtl_freq, tuner_freq;
    r = rtlsdr_get_xtal_freq(rtldev, &rtl_freq, &tuner_freq);
	if (r < 0) {
		fprintf(stderr, "Failed to get rtlsdr device #%d xtal freq.\n", devidx);
		exit(1);
	}
    printf("rtl: %u, tuner: %u\n", rtl_freq, tuner_freq);

    

    r = rtlsdr_close(rtldev); //close Dongle
	if (r < 0) {
		fprintf(stderr, "Failed to close rtlsdr device #%d.\n", devidx);
		exit(1);
	}
}
