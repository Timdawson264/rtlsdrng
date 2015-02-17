/*
* rtl_httpd.c
*
* Copyright 2015 Tim Dawson <timdawson264@gmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
* MA 02110-1301, USA.
*
*
*/


#include <stdio.h>

#include <string.h>
#include <ctype.h>
#include <stdbool.h>

//http
#include <json-c/json.h>
#include <onion/onion.h>
#include <onion/dict.h>
#include <onion/block.h>
#include <onion/request.h>
#include <onion/response.h>
#include <onion/url.h>
#include <onion/low.h>
#include <onion/log.h>
#include <microhttpd.h>

#include "web_include/web_include.h"

//fft
#include <complex.h>
#include <fftw3.h>

//sdr
#include <math.h>
#include <pthread.h>
#include "rtl-sdr.h"

//output
#include <sox.h>
#include <assert.h>

#include "convenience/convenience.h"

#include "fm_httpd_common.h"

static const char * jsonrpc_parse_err = "{\"jsonrpc\": \"2.0\", \"error\": {\"code\": -32700, \"message\": \"Parse error\"}, \"id\": null}";

static void *controller_thread_fn(void *arg);
static void *demod_thread_fn(void *arg);
static void *dongle_thread_fn(void *arg);
void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx);

/// Same as strcmp, but also checks for NULL values.
bool safe_strcmp( const char* a, const char* b )
{
  if ( !a && !b ) {
    return true;
  }
  if ( !a || !b ) {
    return false;
  }
  return strcmp( a, b ) == 0;
}


typedef struct {
  char serial[255];
  enum rtlsdr_tuner tuner;
  
  //Threads
  output_state* out_state; //Output thread state
  demod_state* dm_state; //Demod thread state
  controller_state* con_state; //Controler thread state
  dongle_state* dong_state; //Dongle Thread state

  /* Sox transcoder */
  uint8_t * sox_buf;
  int64_t sox_len;
  pthread_cond_t sox_ready;
	pthread_mutex_t sox_ready_m;
  pthread_rwlock_t sox_rw;
  
} dongle_t;

CirLinkList* Dongles;

dongle_t* dongle_get_by_serial( char* serial )
{
  //TODO: Thread Saftey
  uint8_t i;
  if( Dongles, serial ) {
    for( i = 0; i < Dongles->size; i++ ) {
      dongle_t* d = CirLinkList_get( Dongles, i );
      if( safe_strcmp( d->serial, serial ) ) {
        return d;
      }
    }
  }
  return NULL;
}

dongle_t* dongle_add( char* serial )
{
  //TODO: Thread Saftey
  dongle_t* d = ( dongle_t* ) malloc( sizeof( dongle_t ) );
  d->tuner = RTLSDR_TUNER_UNKNOWN;
  strncpy( d->serial, serial, sizeof( d->serial ) );

  d->con_state = controller_init();
  d->con_state->freqs = CirLinkList_init();
  d->out_state = output_init();
  d->out_state->http_clients = CirLinkList_init();
  d->dm_state = demod_init();
  d->dong_state = dongle_init(d->con_state, d->dm_state, d->out_state);


  /* quadruple sample_rate to limit to Δθ to ±π/2 */
	d->dm_state->rate_in *= d->dm_state->post_downsample;
  /* set output rate */
  d->out_state->rate = d->dm_state->rate_out;

  ACTUAL_BUF_LENGTH = lcm_post[d->dm_state->post_downsample] * DEFAULT_BUF_LENGTH;

  //Add to list of dongles
  CirLinkList_push( Dongles, d );
  return d;
}

//turn enum into str
char* rlsdr_tuner_str( enum rtlsdr_tuner tuner )
{
  switch( tuner ) {
    case RTLSDR_TUNER_E4000:
      return "E4000";
    case RTLSDR_TUNER_FC0012:
      return "FC0012";
    case RTLSDR_TUNER_FC0013:
      return "FC0013";
    case RTLSDR_TUNER_FC2580:
      return "FC2580";
    case RTLSDR_TUNER_R820T:
      return "R820T";
    case RTLSDR_TUNER_R828D:
      return "R828D";
    default:
      return "UNKNOWN";
  }
}

/* get server state */
struct json_object * rpc_sync(struct json_object * req_obj){

  struct MHD_Response * res;
  
  uint32_t x;
  char buf[256];
  

  uint32_t number_devs = rtlsdr_get_device_count();
  struct json_object* dongles_obj =  json_object_new_array();
  
  for( x = 0; x < number_devs; x++ ) {
    struct json_object* dongle_obj =  json_object_new_object();
    rtlsdr_get_device_usb_strings( x, NULL, NULL, buf ); //get serial
    
    //Check if dongle has being seen
    dongle_t* dongle = dongle_get_by_serial( buf );
    if( !dongle ) {
      dongle = dongle_add( buf ); //add to global list
    }
    
    json_object_object_add ( dongle_obj, "name", json_object_new_string(rtlsdr_get_device_name( x )) );
    json_object_object_add ( dongle_obj, "serial", json_object_new_string(buf) );
    json_object_object_add ( dongle_obj, "tuner", json_object_new_string(rlsdr_tuner_str( dongle->tuner )) );

    
    //TODO: Thraed Safety -dongle
    if( dongle->dong_state->dev ) { //check opened and not lost
      json_object_object_add ( dongle_obj, "state", json_object_new_string("open") );
    } else {
      json_object_object_add ( dongle_obj, "state", json_object_new_string("closed") );
    }

    
    //Add Freq list
    onion_dict* freqs_dict = onion_dict_new();
    struct json_object* freqs_obj = json_object_new_array();
    uint16_t f;
    //TODO: Thread Saftey - controller
    
    for( f = 0; f < dongle->con_state->freqs->size; f++ ) {
      scan_node* sn = CirLinkList_get( dongle->con_state->freqs, f );
      if(!sn){
        fprintf(stderr,"Freqs err sn null; %u\n", f);
      }
      if( sn ) {
        struct json_object* freq_obj =  json_object_new_object();
        
        if(sn == dongle->con_state->active_freq){
          json_object_object_add ( freq_obj, "active", json_object_new_boolean(TRUE));
        }else{
          json_object_object_add ( freq_obj, "active", json_object_new_boolean(FALSE));
        }
        
        sprintf( buf, "%u", sn->freq );
        json_object_object_add ( freq_obj, "freq", json_object_new_string(buf) );

        sprintf( buf, "%d", sn->squelch_level );
        json_object_object_add ( freq_obj, "squalch", json_object_new_string(buf) );
        
        sprintf( buf, "%d", sn->gain / 10 );
        json_object_object_add ( freq_obj, "gain", json_object_new_string(buf) );
        
        json_object_object_add ( freq_obj, "mod", json_object_new_string(sn->mod) );

        //add freq to freqs array
        json_object_array_add( freqs_obj, freq_obj);
      }
    }
    
    json_object_object_add(dongle_obj, "freqs", freqs_obj);


    //add dongle_obj to dongles_obj array
    json_object_array_add(dongles_obj, dongle_obj);
  }

  

  return dongles_obj;

}

/* Set RPC Functions */

//Adds a frequancy to the scanning list
struct json_object * add_freq( struct json_object * req_obj ){
  
  //extract variables
  json_object * params_obj, * value_obj; 
  json_object_object_get_ex(req_obj , "params", &params_obj);
  
  fprintf(stderr,"freq_add: %s\n",json_object_to_json_string(params_obj));

  //freq, modulation, squlch level, gain - and dongles serial number
  json_object_object_get_ex(params_obj, "serial", &value_obj);
  const char* dongle_serial = json_object_get_string(value_obj);
  json_object_object_get_ex(params_obj, "freq", &value_obj);
  const char* freq = json_object_get_string(value_obj);
  json_object_object_get_ex(params_obj, "mod", &value_obj);
  const char* mod = json_object_get_string(value_obj);
  json_object_object_get_ex(params_obj, "squalch", &value_obj);
  const char* squalch = json_object_get_string(value_obj);
  json_object_object_get_ex(params_obj, "gain", &value_obj);
  const char* gain = json_object_get_string(value_obj);

  
  if( !freq || !mod || !squalch || !gain || !dongle_serial ) {
    //an option is null
    return NULL;
  }

  dongle_t* d = dongle_get_by_serial( ( char* )dongle_serial );
  if( !d ) {
    return NULL;  //TODO: Dongle Does not Exsits
  }

  scan_node* sn = ( scan_node* ) malloc( sizeof( scan_node ) );
  //parse to ints etc
  //TODO: check all of below outputs
  sn->squelch_level = ( int )atof( squalch );
  sn->gain = ( int )( atof( gain ) * 10 );
  sn->freq = ( uint32_t )atofs( ( char* )freq );
  sn->mod = malloc( strlen( mod ) );
  strcpy( sn->mod, mod );

  //TODO: thread saftey - controller
  CirLinkList_push( d->con_state->freqs, sn );

  //TODO: return ok
  return NULL;
}

struct json_object * del_freq( struct json_object * req_obj ){

  //extract variables
  json_object * params_obj, * value_obj; 
  json_object_object_get_ex(req_obj , "params", &params_obj);
  
  fprintf(stderr,"del_freq: %s\n",json_object_to_json_string(params_obj));

  json_object_object_get_ex(params_obj, "serial", &value_obj);
  const char* dongle_serial = json_object_get_string(value_obj);
  json_object_object_get_ex(params_obj, "idx", &value_obj);
  const char* idx = json_object_get_string(value_obj);
  
  if( !dongle_serial || !idx ) {
    //an option is null
    return NULL;
  }

  dongle_t* d = dongle_get_by_serial( ( char* )dongle_serial );
  if( !d ) {
    return NULL;  //TODO: Dongle Does not Exsits
  }

  //TODO: thread saftey - controller
  CirLinkList_del( d->con_state->freqs, atoi(idx) );
  //TODO: return ok;
  return NULL;

}

struct json_object * next_freq( struct json_object * req_obj ){
   //extract variables
  json_object * params_obj, * value_obj; 
  json_object_object_get_ex(req_obj , "params", &params_obj);
  fprintf(stderr,"next_freq: %s\n",json_object_to_json_string(params_obj));
  
  json_object_object_get_ex(params_obj, "serial", &value_obj);
  const char* dongle_serial = json_object_get_string(value_obj);
  
  if( !dongle_serial) {
    //an option is null
    return NULL;
  }
  dongle_t* d = dongle_get_by_serial( ( char* )dongle_serial );
  if( !d ) {
    return NULL;  //TODO: Dongle Does not Exsits
  }

  safe_cond_signal(&d->con_state->hop, &d->con_state->hop_m);
  return NULL;
}

struct json_object * open_dongle ( struct json_object * req_obj )
{
  //extract variables
  json_object * params_obj, * value_obj; 
  json_object_object_get_ex(req_obj , "params", &params_obj);
  
  json_object_object_get_ex(params_obj, "serial", &value_obj);
  const char* dongle_serial = json_object_get_string(value_obj);

  if( !dongle_serial ) {
    //an option is null
    return NULL;
  }

  dongle_t* d = dongle_get_by_serial( ( char* )dongle_serial );
  fprintf(stderr, "%s() get dongle by serial: %p, %s\n", __FUNCTION__, d, dongle_serial);
  if( !d ) {
    //no such dongle
    return NULL;
  }
  //TODO: Thraed Saftey - controller
  int r;
  int idx = rtlsdr_get_index_by_serial( dongle_serial );
  if( ( r = rtlsdr_open( &d->dong_state->dev, idx ) ) < 0 ) {
    return NULL;
  }
  d->tuner = rtlsdr_get_tuner_type( d->dong_state->dev );

  /* check good serial number */
  uint64_t serial = atol( dongle_serial );
  if( serial < 100 ) {
    while( serial < 100 ) {
      serial = ( ( ( uint64_t )rand() << 32 ) | rand() ) % ( ( uint64_t )pow( 10,
               8 ) );
    }
    fprintf(stderr, "Changing Serial to: %08u", serial );
    //TODO: Write new serial number
  }
  
  /* Reset endpoint before we start reading from it (mandatory) */
  verbose_reset_buffer(d->dong_state->dev);
  //Start Dongle Threads
  pthread_create(&d->con_state->thread, NULL, controller_thread_fn, (void *)(d->dong_state));
	sleep(.1);
  //TODO: make output work with libmicrohttpd
  //pthread_create( &d->out_state->thread, NULL, output_http_thread_fn, ( void* )( d->dong_state ) );
	pthread_create(&d->dm_state->thread, NULL, demod_thread_fn, (void *)(d->dong_state));
	pthread_create(&d->dong_state->thread, NULL, dongle_thread_fn, (void *)(d->dong_state));

  return 0;
}

static void *controller_thread_fn(void *arg)
{
  //This is where the channel hopping should happen.
  fprintf(stderr,"Dongle Thread Started\n");

	uint64_t i;
  dongle_state *dongle = arg;
	controller_state *s = dongle->controller;
  demod_state *demod =  dongle->demod_target;


    //TODO: Start Other Threads From Controller
    //TODO: Be smarter

    //TODO: Thread saftey on controller
    s->active_freq=NULL;
    fprintf(stderr,"Controller Thread Scanning\n");
    
    while (!do_exit) {
        /* Next Frequancy */
        s->active_freq = (scan_node*) CirLinkList_inc(s->freqs);
        dongle->mute = BUFFER_DUMP;

        //if no freq
        while(s->active_freq == NULL){
          sleep(.1);
          s->active_freq = (scan_node*) CirLinkList_peek(s->freqs); //first frequancy
        }
      
        fprintf(stderr, "Tuning To: freq: %u, mod: %s, squalch: %d, gain: %d\n",
              s->active_freq->freq, s->active_freq->mod,
              s->active_freq->squelch_level, s->active_freq->gain);
        setdemod(demod, s->active_freq->mod);

        if ( demod->deemph ) {
          demod->deemph_a = ( int )round( 1.0 / ( ( 1.0 - exp( -1.0 /
                                            ( demod->rate_out * 75e-6 ) ) ) ) );
        }

        //Set Squlch
        demod->squelch_level = s->active_freq->squelch_level;

        /* Set the tuner gain */

        /* Set Gain */
        if (s->active_freq->gain == AUTO_GAIN) {
            verbose_auto_gain(dongle->dev);
        } else {
            s->active_freq->gain = nearest_gain(dongle->dev, s->active_freq->gain);
            verbose_gain_set(dongle->dev, s->active_freq->gain);
        }
        
        /* set up channel */
        
        optimal_settings(s->active_freq->freq, demod->rate_in, dongle);
        if (dongle->direct_sampling) {
            verbose_direct_sampling(dongle->dev, 1);
        }
        if (dongle->offset_tuning) {
            verbose_offset_tuning(dongle->dev);
        }

        /* Set the frequency */
        if(strcasecmp("wbfm", s->active_freq->mod) == 0){
          verbose_set_frequency(dongle->dev, dongle->freq+16000);
        }else{
          verbose_set_frequency(dongle->dev, dongle->freq);
        }
        
        fprintf(stderr, "Oversampling input by: %ix.\n", demod->downsample);
        fprintf(stderr, "Oversampling output by: %ix.\n", demod->post_downsample);
        fprintf(stderr, "Buffer size: %0.2fms\n",
            1000 * 0.5 * (float)ACTUAL_BUF_LENGTH / (float)dongle->rate);

        /* Set the sample rate */
        verbose_set_sample_rate(dongle->dev, dongle->rate);
        fprintf(stderr, "Output at %u Hz.\n", demod->rate_in/demod->post_downsample);
       
        //Wait for Channel hop
        if(do_exit) return 0;//just check before wait
        safe_cond_wait(&s->hop, &s->hop_m);
        fprintf(stderr,"Controller Thread Channel Hop\n");
      
    }
    return 0;
}


static void *demod_thread_fn(void *arg)
{
  dongle_state *dongle = arg;
  
	demod_state *d = dongle->demod_target;
	output_state *o = dongle->output_target;
  controller_state *controller = dongle->controller;
    
	while (!do_exit) {
		safe_cond_wait(&d->ready, &d->ready_m);
		pthread_rwlock_wrlock(&d->rw);
		full_demod(d);
		pthread_rwlock_unlock(&d->rw);
		if (d->exit_flag) {
			do_exit = 1;
		}
        
		if (d->squelch_level && d->squelch_hits > d->conseq_squelch) {
			d->squelch_hits = d->conseq_squelch + 1;  /* hair trigger */
			safe_cond_signal(&controller->hop, &controller->hop_m);
                        //continue; instead of skiping write, we overwrite zeros
                        memset ( d->result, 0, 2*d->result_len);
		}
		pthread_rwlock_wrlock(&o->rw);
		memcpy(o->result, d->result, 2*d->result_len);
		o->result_len = d->result_len;
		pthread_rwlock_unlock(&o->rw);
		safe_cond_broadcast(&o->ready, &o->ready_m); //signal all threads waiting for output
	}
	return 0;
}

static void *dongle_thread_fn(void *arg)
{
	dongle_state *s = arg;
  fprintf(stderr,"Dongle Thread Started\n");
	rtlsdr_read_async(s->dev, rtlsdr_callback, s, 0, s->buf_len);
	return 0;
}

void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	int i;
	dongle_state *s = ctx;
	demod_state *d = s->demod_target;
  //fprintf(stderr,"Dongle Thread Has DATA\n");   
	if (do_exit) {
		return;
  }
	if (!ctx) {
		return;
  }
	if (s->mute) {
		for (i=0; i<s->mute; i++) {
			buf[i] = 127;}
		s->mute = 0;
	}
  
	if (!s->offset_tuning) {
		rotate_90(buf, len);
        }
	for (i=0; i<(int)len; i++) {
		s->buf16[i] = (int16_t)buf[i] - 127;
    }

  pthread_rwlock_wrlock(&d->rw);
    
	memcpy(d->lowpassed, s->buf16, 2*len);
  d->lp_len = len;

  pthread_rwlock_unlock(&d->rw);
	safe_cond_signal(&d->ready, &d->ready_m);
}

static void* output_http_thread_fn( void* arg )
{
  
  
  uint16_t i;
  dongle_state* dongle = arg;
  output_state* s = dongle->output_target;
  
  pthread_rwlock_rdlock( &s->rw );

  #define MAX_SAMPLES (size_t)2048
  #define SOX_BUF_LENGTH (MAXIMUM_BUF_LENGTH*2)
  uint8_t sox_buffer[SOX_BUF_LENGTH];
  sox_sample_t sox_samples[MAX_SAMPLES];
  sox_effects_chain_t * sox_chain;
  sox_effect_t * e;
  sox_signalinfo_t interm_signal; /* @ intermediate points in the chain. */
  char * sox_args[10];
  size_t number_read;
  
  /* set sox input settings */
  const sox_encodinginfo_t inEncode = {
    SOX_ENCODING_SIGN2,
    16,
    0,
    sox_option_default,
    sox_option_default,
    sox_option_default,
    sox_false
  };
  const sox_signalinfo_t inSignal = {
    32000,
    1,
    0,
    0,
    NULL
  };

  sox_format_t* sox_in =  sox_open_mem_read(s->result, 2*MAXIMUM_BUF_LENGTH, &inSignal, &inEncode, "raw");
  
  /* set sox output settings */
  const sox_encodinginfo_t outEncode = {
    SOX_ENCODING_VORBIS,
    0,
    0,
    sox_option_default,
    sox_option_default,
    sox_option_default,
    sox_false
  };
  const sox_signalinfo_t outSignal = {
    8000,
    1,
    0,
    0,
    NULL
  };
  
  sox_format_t* sox_out = sox_open_mem_write(sox_buffer, SOX_BUF_LENGTH, &outSignal, &outEncode, "ogg", NULL);

  fprintf(stderr,"Sox Buffers Opened %p, %p\n", s->result, sox_buffer);

  /* setup sox chain */
  sox_chain = sox_create_effects_chain(&sox_in->encoding, &sox_out->encoding);
  interm_signal = sox_in->signal; /* NB: deep copy */


  //Input Effect in chain
  e = sox_create_effect(sox_find_effect("input"));
  sox_args[0] = (char *)sox_in, assert(sox_effect_options(e, 1, sox_args) == SOX_SUCCESS);
  assert(sox_add_effect(sox_chain, e, &interm_signal, &sox_in->signal) == SOX_SUCCESS);
  free(e);

  //TODO  add filters in here
  
  e = sox_create_effect(sox_find_effect("output"));
  sox_args[0] = (char *)sox_out, assert(sox_effect_options(e, 1, sox_args) == SOX_SUCCESS);
  assert(sox_add_effect(sox_chain, e, &interm_signal, &sox_out->signal) == SOX_SUCCESS);
  free(e);

  fprintf(stderr,"Sox Chain Opened\n");
  pthread_rwlock_unlock( &s->rw );


  while ( !do_exit ) {

    /* Wait For Data */
    safe_cond_wait(&s->ready, &s->ready_m);

    /* Lock Output State for read */
    pthread_rwlock_rdlock( &s->rw );
    /* do SOX transcode - result data will be in sox_buffer*/
    sox_flow_effects(sox_chain, NULL, NULL);
    pthread_rwlock_unlock( &s->rw );

    //use cond to notify http callbacks that want ogg

  }
  return 0;
}
//static ssize_t dir_reader (void *cls, uint64_t pos, char *buf, size_t max)
ssize_t http_raw_output_stream_cb (void *cls, uint64_t pos, char *buf, size_t max)
{
  //cls should be ref to dongle_t
  dongle_t* dongle = cls;
  output_state* out = dongle->out_state;
  
  //TODO: if dongle off return -1
  safe_cond_wait( &out->ready, &out->ready_m );
  pthread_rwlock_rdlock( &out->rw );
  memcpy(buf, out->result, out->result_len*2);
  pthread_rwlock_unlock( &out->rw );

  //return bytes copied 
  return out->result_len*2;
}

ssize_t http_ogg_output_stream_cb (void *cls, uint64_t pos, char *buf, size_t max)
{
  //cls should be ref to dongle_t
  dongle_t* dongle = cls;
  
  //TODO: if dongle off return -1 - could use timeout
  safe_cond_wait( &dongle->sox_ready, &dongle->sox_ready_m );
  pthread_rwlock_rdlock( &dongle->sox_rw );
  memcpy(buf, dongle->sox_buf, dongle->sox_len);
  pthread_rwlock_unlock( &dongle->sox_rw );

  //return bytes copied 
  return dongle->sox_len;
}

const char * NOT_FOUND = "<h1> Not Found </h1>";
int response_handler (void *cls,
          struct MHD_Connection *connection,
          const char *url,
          const char *method,
          const char *version,
          const char *upload_data,
          size_t *upload_data_size,
          void **ptr)
{
  fprintf(stderr,"Got Req - method: %s, url: %s, size: %u\n", method, url, *upload_data_size);

  struct MHD_Response * res = NULL;
    
  char * tok_state;
  char * tok = strtok_r((char*)url, "/", &tok_state);

  if(tok==NULL){
    return MHD_NO; //could redirect to web
  }
  if( tok && strcmp(tok, "out") == 0 ){
    tok = strtok_r(NULL, "/", &tok_state);
    if(tok){
      //tok is serial
      dongle_t * d = dongle_get_by_serial( tok );
      fprintf(stderr, "out req - serial: %s, dongle: %p\n", tok, d);
      //create callback response that returns raw output buffer if dongle exsits
      if(d){
        tok = strtok_r(NULL, "/", &tok_state);
        if(tok && strcmp(tok, "ogg") == 0){
          //use ogg cb
          res = MHD_create_response_from_callback(-1, 4096, &http_ogg_output_stream_cb, d, NULL);
        }else{
          res = MHD_create_response_from_callback(-1, 4096, &http_raw_output_stream_cb, d, NULL);
        }
      }
    }
  }
  if( tok && strcmp(tok, "web") == 0 ){
    fprintf(stderr,"WEB\n");
    tok = strtok_r(NULL, "/", &tok_state);
    if(!tok){
      fprintf(stderr,"WEB:main\n");
      res = MHD_create_response_from_buffer (web_all_len, web_all, MHD_RESPMEM_PERSISTENT);
    }
    else if( strcmp(tok, "js") == 0 ){
      fprintf(stderr,"WEB:js\n");
      res = MHD_create_response_from_buffer (js_all_len, js_all, MHD_RESPMEM_PERSISTENT);
    }
    else if( strcmp(tok, "css") == 0 ){
      fprintf(stderr,"WEB:css\n");
      res = MHD_create_response_from_buffer (css_all_len, css_all, MHD_RESPMEM_PERSISTENT);
      MHD_add_response_header(res, "Content-Type", "text/css");
    }
    
  }
  if( tok && strcmp(tok, "json") == 0 ){
    tok = strtok_r(NULL, "/", &tok_state);

    /* is it a POST */
    if ( strcmp(method, MHD_HTTP_METHOD_POST) == 0 ){
      fprintf(stderr, "json post\n");

      //No Data
      if(*upload_data_size == 0){
        if(ptr != NULL){
          MHD_add_response_header( *ptr, "Content-Type", "application/json");
          MHD_queue_response(connection, MHD_HTTP_OK, *ptr);
          MHD_destroy_response(*ptr);
          *ptr = NULL;
        }
        return MHD_YES;
      }
      
      /* if there is post data */
      if(*upload_data_size > 0){
        
        struct json_tokener * tok = json_tokener_new();
        struct json_object * req_obj;
        struct json_object * value;
        struct json_object * id_obj;
        json_bool haskey = FALSE;
        
        /* parse JSON str to obj */
        req_obj = json_tokener_parse_ex(tok, upload_data, *upload_data_size);
        *upload_data_size = 0;//data read
        if(req_obj == NULL && json_tokener_get_error(tok) == json_tokener_continue){
          fprintf(stderr,"JSON in Pieaces should implement this code\n");
          return MHD_NO; //TODO: implement multipart json decoding using **ptr
        }
        json_tokener_free(tok);
        
        /* use obj for jsonrpc goodness */
        
      
        /* test jsonrpc version */
        haskey = json_object_object_get_ex(req_obj, "jsonrpc", &value);
        if(haskey == FALSE || strcmp(json_object_get_string(value), "2.0") != 0){
          fprintf(stderr,"JSONrpc parse error\n");
          res = MHD_create_response_from_buffer(strlen(jsonrpc_parse_err), (void*)jsonrpc_parse_err, MHD_RESPMEM_PERSISTENT);
          *ptr = res;
          json_object_put(req_obj);
          return MHD_YES;
        }

        json_object_object_get_ex (req_obj, "id", &id_obj);        
        haskey = json_object_object_get_ex(req_obj, "method", &value);
        fprintf(stderr, "METHOD: %s\n", json_object_get_string(value));

        struct json_object * res_obj = json_object_new_object();
        json_object_object_add(res_obj, "jsonrpc", json_object_new_string("2.0"));
        json_object_object_add(res_obj, "id", id_obj);

        if(strcmp(json_object_get_string(value), "add_freq") == 0){
          json_object_object_add(res_obj, "result", add_freq(req_obj));
        }
        if(strcmp(json_object_get_string(value), "del_freq") == 0){
          json_object_object_add(res_obj, "result", del_freq(req_obj));
        }
        if(strcmp(json_object_get_string(value), "next_freq") == 0){
          json_object_object_add(res_obj, "result", next_freq(req_obj));
        }
        
        if(strcmp(json_object_get_string(value), "sync") == 0){
          json_object_object_add(res_obj, "result", rpc_sync(req_obj));
        }
        
        if(strcmp(json_object_get_string(value), "open") == 0){
          json_object_object_add(res_obj, "result", open_dongle(req_obj));
        }

        
        //Method not found handler - no result == no method
        if(json_object_object_get_ex(res_obj, "result", NULL) == FALSE){
          struct json_object * error = json_object_new_object();
          json_object_object_add(error, "code", json_object_new_int(-32601) );
          json_object_object_add(error, "message", json_object_new_string("Method not found") );
          json_object_object_add(res_obj, "error", error);
        }
        
        //turn into string
        char * json = (char*) json_object_to_json_string_ext(res_obj,  JSON_C_TO_STRING_PLAIN );
        fprintf(stderr,"%u: %s\n",strlen(json), json);
        //create response
        res = MHD_create_response_from_buffer(strlen(json), json, MHD_RESPMEM_MUST_COPY);
        *ptr = res;
        json_object_put(res_obj);//free json obj
        json_object_put(req_obj);
        return MHD_YES;
      }
      
    }
   
  }

  //No one claimed url
  if(res == NULL){
    res = MHD_create_response_from_buffer (strlen(NOT_FOUND), (void*)NOT_FOUND, MHD_RESPMEM_PERSISTENT);
    MHD_queue_response(connection, MHD_HTTP_NOT_FOUND, res);
  }else{
    MHD_queue_response(connection, MHD_HTTP_OK, res);
  }
  MHD_destroy_response(res);
  return MHD_YES;
}


int main( int argc, char** argv )
{
  char* port = "8080";

  int i;
  for ( i = 1; i < argc; i++ ) {
    if ( strcmp( argv[i], "-p" ) == 0 ) {
      port = argv[++i];
    }
  }

  //scan dongles, for infomantion and so we have referance
  Dongles = CirLinkList_init();
  char buf[256];
  uint32_t x;
  uint32_t number_devs = rtlsdr_get_device_count();
  for( x = 0; x < number_devs; x++ ) {
    rtlsdr_get_device_usb_strings( x, NULL, NULL, buf ); //get serial

    //Check if dongle has being seen
    dongle_t* dongle = dongle_get_by_serial( buf );
    if( !dongle ) {
      dongle = dongle_add( buf ); //add to list if not in list
    }
  }

  assert(sox_init() == SOX_SUCCESS); //start sox

  //Setup microhttpd
  struct MHD_Daemon *MHD;
  void * ptr;
  MHD = MHD_start_daemon (MHD_USE_THREAD_PER_CONNECTION | MHD_USE_DEBUG,
                        8888,
                        NULL, NULL, &response_handler, NULL, MHD_OPTION_END);
                        
  while(!do_exit){
    sleep(1);
  }

  return 0;

}
