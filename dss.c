

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
//#include <libftdi1/ftdi.h>
#include <ftdi.h>

#define PIN_TX  0x01 
#define PIX_RX  0x02  
#define PIN_RTS 0x04  
#define PIN_CTS 0x08  
#define PIN_DTR 0x10
#define PIN_DSR 0x20
#define PIN_DCD 0x40
#define PIN_RI  0x80

typedef struct{
  struct ftdi_context * ftdi;
  uint8_t ftdibyte;
  uint8_t  w_clk_pin;
  uint8_t  fq_ud_pin;
  uint8_t  data_pin;
  uint8_t  reset_pin;
  uint8_t   reg[5]; //this is the data that is sent to the dss
}AD9850_t;



void writebit(AD9850_t * dss ,uint8_t pin, uint8_t state){
  char c;
  
  if(state){
    dss->ftdibyte |= pin;
  }else{
    dss->ftdibyte &= ~pin;
  }
  ftdi_write_data(dss->ftdi, &dss->ftdibyte, 1);
  ftdi_read_data(dss->ftdi, &c, 1);
  usleep(1);
}

/* dss clock data */
inline void AD9850_clk(AD9850_t* dss){
  writebit(dss, dss->w_clk_pin, 1);
  writebit(dss, dss->w_clk_pin, 0);
}

/* dss frequancy update */
inline void AD9850_fqud(AD9850_t* dss){
  writebit(dss, dss->fq_ud_pin, 1);
  writebit(dss, dss->fq_ud_pin, 0);
}

/* dss write serial byte */
static void AD9850_write(AD9850_t* dss){ 
  uint8_t i,j;
  AD9850_fqud(dss);
  for(j=0; j<5; j++){
    for(i=0; i<8; i++){
      writebit(dss, dss->data_pin, (dss->reg[j]>>i)&0x01);
      AD9850_clk(dss);
    }
  }
  AD9850_fqud(dss);
}

void AD9850_set_powerdown(AD9850_t* dss, uint8_t powerdown){
  
  dss->reg[4] = ( ( powerdown & 0x1 )<<2) | ( dss->reg[4] & ~(1<<2) );
  AD9850_write(dss); //write out
  
}

void AD9850_set_frequency(AD9850_t* dss, double freq){
  
  //fout = (reg * clkin)/2^32 -- From data sheet
  //freq = (reg * 125)/2^32
  //freq * 2^32 / 125 = reg

  uint32_t* reg = (uint32_t *) dss->reg;
  *reg = lround ( (freq*pow(2,32))/125.0d );
  AD9850_write(dss);	//write out
  
}

void AD9850_correct_frequency(AD9850_t* dss, double ppm){
  uint32_t* reg = (uint32_t *) dss->reg;
  double freq = ((*reg)*125.0d)/pow(2,32);
  freq = (2.0d*freq) - ((freq) * (1.0d + (ppm) / 1000000.0d));
  AD9850_set_frequency(dss, freq);
  
}

void AD9850_set_phase(AD9850_t* dss, uint8_t phase){
  dss->reg[4] = (phase<<3) | ( dss->reg[4] & 0b00000111);
  AD9850_write(dss);	//write out
}

int main(int argc, char ** argv){

  struct ftdi_context ftdic;
  AD9850_t dss;
  memset( &dss, 0, sizeof(AD9850_t) ); //zero
  
  /* config dss */
  dss.w_clk_pin = PIN_TX;
  dss.fq_ud_pin = PIX_RX;
  dss.data_pin = PIN_DTR;
  

  /* Initialize context for subsequent function calls */
  ftdi_init(&ftdic);
  dss.ftdi=&ftdic;
  
  /* Open FTDI device based on FT232R vendor & product IDs */
  if(ftdi_usb_open(&ftdic, 0x0403, 0x6001) < 0) {
      fputs("Can't open device", stderr);
      return 1;
  }
  char c;

  /* Enable bitbang mode */
  ftdi_set_bitmode(&ftdic, dss.w_clk_pin|dss.fq_ud_pin|dss.data_pin , BITMODE_SYNCBB);
  dss.ftdibyte = 0;
  ftdi_write_data(dss.ftdi, &dss.ftdibyte, 1);
  ftdi_read_data(dss.ftdi, &c, 1);
  usleep(100);
  
  //0.001010417 min ppm change
  
  AD9850_set_frequency(&dss, 28.8);
  double ppm;
  while(1){
    printf("ppm error:");
    scanf("%lf", &ppm);
    printf("Correcting by %lfppm\n", ppm);
    AD9850_correct_frequency(&dss, ppm);
    
    uint32_t* reg = (uint32_t *) dss.reg;
    double freq = ((*reg)*125.0d)/pow(2,32);
    printf("New Freq: %.4lfHz\n",freq*1000000);
  }
  return 0;
}
