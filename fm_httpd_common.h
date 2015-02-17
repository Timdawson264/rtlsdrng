#ifndef __FM__HTTPD_H
#define __FM__HTTPD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <pthread.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#ifdef _MSC_VER
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"
#include "convenience/CirDLinkedList.h"

#define DEFAULT_SAMPLE_RATE		24000
#define DEFAULT_BUF_LENGTH		(1 * 16384)
#define MAXIMUM_OVERSAMPLE		16
#define MAXIMUM_BUF_LENGTH		(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN			-100
#define BUFFER_DUMP			4096

/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_broadcast(n, m) pthread_mutex_lock(m); pthread_cond_broadcast(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)

static volatile int do_exit = 0;
static int lcm_post[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};
static int ACTUAL_BUF_LENGTH;

static int *atan_lut = NULL;
static int atan_lut_size = 131072; /* 512 KB */
static int atan_lut_coef = 8;

typedef struct
{
        uint32_t	id;
	uint32_t	freq;
	char*		mod;
	int	        squelch_level;
	int		gain;
}scan_node;

typedef struct controller_state
{
	int      exit_flag;
	pthread_t thread;
	CirLinkList* freqs;
        scan_node *  active_freq;
	int      edge;
	pthread_cond_t hop;
	pthread_mutex_t hop_m;
        pthread_rwlock_t rw;
        
} controller_state;

typedef struct output_state
{
	int      exit_flag;
	pthread_t thread;
	FILE     *file;
	char     *filename;
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int      result_len;
	int      rate;
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
} output_state;

typedef struct demod_state
{
	int      exit_flag;
	pthread_t thread;
	int16_t  lowpassed[MAXIMUM_BUF_LENGTH];
	int      lp_len;
	int16_t  lp_i_hist[10][6];
	int16_t  lp_q_hist[10][6];
	int16_t  result[MAXIMUM_BUF_LENGTH];
	int16_t  droop_i_hist[9];
	int16_t  droop_q_hist[9];
	int      result_len;
	int      rate_in;
	int      rate_out;
	int      rate_out2;
	int      now_r, now_j;
	int      pre_r, pre_j;
	int      prev_index;
	int      downsample;    /* min 1, max 256 */
	int      post_downsample;
	int      output_scale;
	int      squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
	int      downsample_passes;
	int      comp_fir_size;
	int      custom_atan;
	int      deemph, deemph_a;
	int      now_lpr;
	int      prev_lpr_index;
	int      dc_block, dc_avg;
        int      wb_mode;
	void     (*mode_demod)(struct demod_state*);
	pthread_rwlock_t rw;
	pthread_cond_t ready;
	pthread_mutex_t ready_m;
    
} demod_state;

typedef struct dongle_state
{
	int      exit_flag;
	pthread_t thread;
	rtlsdr_dev_t *dev;
	int      dev_index;
	uint32_t freq;
	uint32_t rate;
	int      gain;
	uint16_t buf16[MAXIMUM_BUF_LENGTH];
	uint32_t buf_len;
	int      ppm_error;
	int      offset_tuning;
	int      direct_sampling;
	int      mute;
        
        pthread_rwlock_t rw;

        demod_state *demod_target;
        output_state *output_target;
        controller_state *controller;
        
} dongle_state;


void optimal_settings(int freq, int rate, dongle_state *d);
void sanity_checks(controller_state* controller);
int atan_lut_init(void);
void rotate_90(unsigned char *buf, uint32_t len);

controller_state* controller_init();
output_state* output_init();
demod_state* demod_init();
dongle_state* dongle_init(controller_state* controller, demod_state* demod, output_state* output);

void controller_cleanup(controller_state *s);
void demod_cleanup(demod_state *s);
void dongle_cleanup(dongle_state* s);
void output_cleanup(output_state *s);


void full_demod(demod_state *d);
void fm_demod(demod_state *fm);
void am_demod(demod_state *fm);
void usb_demod(demod_state *fm);
void lsb_demod(demod_state *fm);
void raw_demod(demod_state *fm);
void setdemod(demod_state *demod, char * mod);
output_state* output_init();


#ifdef __cplusplus
}
#endif

#endif /* __FM__HTTPD_H */
