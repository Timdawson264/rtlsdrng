/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 * written because people could not do real time
 * FM demod on Atom hardware with GNU radio
 * based on rtl_sdr.c and rtl_tcp.c
 *
 * lots of locks, but that is okay
 * (no many-to-many locks)
 *
 * todo:
 *       sanity checks
 *       scale squelch to other input parameters
 *       test all the demodulations
 *       pad output on hop
 *       frequency ranges could be stored better
 *       scaled AM demod amplification
 *       auto-hop after time limit
 *       peak detector to tune onto stronger signals
 *       fifo for active hop frequency
 *       clips
 *       noise squelch
 *       merge stereo patch
 *       merge soft agc patch
 *       merge udp patch
 *       testmode to detect overruns
 *       watchdog to reset bad dongle
 *       fix oversampling
 */

#include "fm_httpd_common.h"

dongle_state*
dongle; //global because every dongle is independednt and only one in use

void usage( void )
{
  fprintf( stderr,
           "rtl_fm, a simple narrow band FM demodulator for RTL2832 based DVB-T receivers\n\n"
           "Use:\trtl_fm -f freq [-options] [filename]\n"
           "\t-f frequency_to_tune_to [Hz]\n"
           "\t    use multiple -f for scanning (requires squelch)\n"
           "\t    ranges supported, -f 118M:137M:25k\n"
           "\t[-M modulation (default: fm)]\n"
           "\t    fm, wbfm, raw, am, usb, lsb\n"
           "\t    wbfm == -M fm -s 170k -o 4 -A fast -r 32k -l 0 -E deemp\n"
           "\t    raw mode outputs 2x16 bit IQ pairs\n"
           "\t[-s sample_rate (default: 24k)]\n"
           "\t[-d device_index (default: 0)]\n"
           "\t[-g tuner_gain (default: automatic)]\n"
           "\t[-l squelch_level (default: 0/off)]\n"
           //"\t    for fm squelch is inverted\n"
           //"\t[-o oversampling (default: 1, 4 recommended)]\n"
           "\t[-p ppm_error (default: 0)]\n"
           "\t[-E enable_option (default: none)]\n"
           "\t    use multiple -E to enable multiple options\n"
           "\t    edge:   enable lower edge tuning\n"
           "\t    dc:     enable dc blocking filter\n"
           "\t    deemp:  enable de-emphasis filter\n"
           "\t    direct: enable direct sampling\n"
           "\t    offset: enable offset tuning\n"
           "\tfilename ('-' means stdout)\n"
           "\t    omitting the filename also uses stdout\n\n"
           "Experimental options:\n"
           "\t[-r resample_rate (default: none / same as -s)]\n"
           "\t[-t squelch_delay (default: 10)]\n"
           "\t    +values will mute/scan, -values will exit\n"
           "\t[-F fir_size (default: off)]\n"
           "\t    enables low-leakage downsample filter\n"
           "\t    size can be 0 or 9.  0 has bad roll off\n"
           "\t[-A std/fast/lut choose atan math (default: std)]\n"
           //"\t[-C clip_path (default: off)\n"
           //"\t (create time stamped raw clips, requires squelch)\n"
           //"\t (path must have '\%s' and will expand to date_time_freq)\n"
           //"\t[-H hop_fifo (default: off)\n"
           //"\t (fifo will contain the active frequency)\n"
           "\n"
           "Produces signed 16 bit ints, use Sox or aplay to hear them.\n"
           "\trtl_fm ... | play -t raw -r 24k -es -b 16 -c 1 -V1 -\n"
           "\t           | aplay -r 24k -f S16_LE -t raw -c 1\n"
           "\t  -M wbfm  | play -r 32k ... \n"
           "\t  -s 22050 | multimon -t raw /dev/stdin\n\n" );
  exit( 1 );
}

#ifdef _WIN32
BOOL WINAPI
sighandler( int signum )
{
  if ( CTRL_C_EVENT == signum ) {
    fprintf( stderr, "Signal caught, exiting!\n" );
    do_exit = 1;
    rtlsdr_cancel_async( dongle->dev );
    return TRUE;
  }
  return FALSE;
}
#else
static void sighandler( int signum )
{
  if( do_exit != 1 ) {
    fprintf( stderr, "Signal caught, exiting!\n" );
    do_exit = 1;
    rtlsdr_cancel_async( dongle->dev );
  }
}
#endif

void rtlsdr_callback( unsigned char* buf, uint32_t len, void* ctx )
{
  int i;
  dongle_state* s = ctx;
  demod_state* d = s->demod_target;

  if ( do_exit ) {
    return;
  }
  if ( !ctx ) {
    return;
  }
  if ( s->mute ) {
    for ( i = 0; i < s->mute; i++ ) {
      buf[i] = 127;
    }
    s->mute = 0;
  }
  if ( !s->offset_tuning ) {
    rotate_90( buf, len );
  }
  for ( i = 0; i < ( int )len; i++ ) {
    s->buf16[i] = ( int16_t )buf[i] - 127;
  }
  pthread_rwlock_wrlock( &d->rw );

  memcpy( d->lowpassed, s->buf16, 2 * len );
  d->lp_len = len;
  pthread_rwlock_unlock( &d->rw );
  safe_cond_signal( &d->ready, &d->ready_m );
}


static void* dongle_thread_fn( void* arg )
{
  dongle_state* s = arg;
  rtlsdr_read_async( s->dev, rtlsdr_callback, s, 0, s->buf_len );
  return 0;
}

static void* demod_thread_fn( void* arg )
{
  dongle_state* dongle = arg;
  demod_state* d = dongle->demod_target;
  output_state* o = dongle->output_target;
  controller_state* controller = dongle->controller;

  while ( !do_exit ) {
    safe_cond_wait( &d->ready, &d->ready_m );
    pthread_rwlock_wrlock( &d->rw );
    full_demod( d );
    pthread_rwlock_unlock( &d->rw );
    if ( d->exit_flag ) {
      do_exit = 1;
    }

    if ( d->squelch_level && d->squelch_hits > d->conseq_squelch ) {
      d->squelch_hits = d->conseq_squelch + 1;  /* hair trigger */
      safe_cond_signal( &controller->hop, &controller->hop_m );
      //continue; instead of skiping write, we overwrite zeros
      memset ( d->result, 0, 2 * d->result_len );
    }
    pthread_rwlock_wrlock( &o->rw );
    memcpy( o->result, d->result, 2 * d->result_len );
    o->result_len = d->result_len;
    pthread_rwlock_unlock( &o->rw );
    safe_cond_signal( &o->ready, &o->ready_m );
  }
  return 0;
}

static void* output_thread_fn( void* arg )
{
  dongle_state* dongle = arg;
  output_state* s = dongle->output_target;

  while ( !do_exit ) {
    // use timedwait and pad out under runs
    safe_cond_wait( &s->ready, &s->ready_m );
    pthread_rwlock_rdlock( &s->rw );
    fwrite( s->result, 2, s->result_len, s->file );
    pthread_rwlock_unlock( &s->rw );
  }
  return 0;
}

static void* controller_thread_fn( void* arg )
{
  //This is where the channel hopping and dongle selection should happen.


  uint64_t i;
  dongle_state* dongle = arg;
  controller_state* s = dongle->controller;
  demod_state* demod =  dongle->demod_target;

  for( i = 0; i < s->freqs->size; i++ ) {
    scan_node* n = ( scan_node* ) CirLinkList_get( s->freqs, i );
    if( strcmp( "wbfm", n->mod ) == 0 || strcmp( "WBFM", n->mod ) == 0 ) {
      n->freq += 16000;
    }
  }

  scan_node* n = ( scan_node* ) CirLinkList_peek( s->freqs ); //first frequancy

  while ( !do_exit ) {


    setdemod( demod, n->mod );
    //Set Squlch
    demod->squelch_level = n->squelch_level;
    /* Set the tuner gain */

    /* Set Gain */
    if ( n->gain == AUTO_GAIN ) {
      verbose_auto_gain( dongle->dev );
    } else {
      n->gain = nearest_gain( dongle->dev, n->gain );
      verbose_gain_set( dongle->dev, n->gain );
    }



    /* set up primary channel */
    optimal_settings( n->freq, demod->rate_in, dongle );
    if ( dongle->direct_sampling ) {
      verbose_direct_sampling( dongle->dev, 1 );
    }
    if ( dongle->offset_tuning ) {
      verbose_offset_tuning( dongle->dev );
    }

    /* Set the frequency */
    verbose_set_frequency( dongle->dev, dongle->freq );
    fprintf( stderr, "Oversampling input by: %ix.\n", demod->downsample );
    fprintf( stderr, "Oversampling output by: %ix.\n", demod->post_downsample );
    fprintf( stderr, "Buffer size: %0.2fms\n",
             1000 * 0.5 * ( float )ACTUAL_BUF_LENGTH / ( float )dongle->rate );

    /* Set the sample rate */
    verbose_set_sample_rate( dongle->dev, dongle->rate );
    fprintf( stderr, "Output at %u Hz.\n",
             demod->rate_in / demod->post_downsample );

    //Wait for Channel hop
    if( do_exit ) {
      return 0;  //just check before wait
    }
    safe_cond_wait( &s->hop, &s->hop_m );
    if ( s->freqs->size <= 1 ) {
      continue;
    }

    /* Next Frequancy */
    n = ( scan_node* ) CirLinkList_inc( s->freqs );
    dongle->mute = BUFFER_DUMP;
  }
  return 0;
}

void frequency_range( controller_state* s, char* arg )
{
  char* start, *stop, *step;
  int i;
  start = arg;
  stop = strchr( start, ':' ) + 1;
  stop[-1] = '\0';
  step = strchr( stop, ':' ) + 1;
  step[-1] = '\0';
  for( i = ( int )atofs( start ); i <= ( int )atofs( stop );
       i += ( int )atofs( step ) ) {
    scan_node* node = malloc( sizeof( scan_node ) );
    node->freq = ( uint32_t )i;
    CirLinkList_push( s->freqs, node ); //add to scanner list
  }
  stop[-1] = ':';
  step[-1] = ':';
}

int main( int argc, char** argv )
{
#ifndef _WIN32
  struct sigaction sigact;
#endif
  int r, opt;
  int dev_given = 0;
  int custom_ppm = 0;

  controller_state* controller = controller_init();
  demod_state* demod = demod_init();
  output_state* output = output_init();
  dongle = dongle_init( controller, demod, output ); //global var


  while ( ( opt = getopt( argc, argv,
                          "d:f:g:s:b:l:o:t:r:p:E:F:A:M:h" ) ) != -1 ) {
    switch ( opt ) {
      case 'd':
        dongle->dev_index = verbose_device_search( optarg );
        dev_given = 1;
        break;
      case 'f':
        if ( strchr( optarg, ':' ) ) {
          frequency_range( controller, optarg );
        } else {
          scan_node* node = malloc( sizeof( scan_node ) );
          if( controller->freqs->size > 0 ) { //default to prev nodes settings
            memcpy( node, CirLinkList_peek( controller->freqs ), sizeof( scan_node ) ) ;
          }
          node->freq = ( uint32_t )atofs( optarg );
          CirLinkList_push( controller->freqs, node ); //add to scanner list
        }
        break;
      case 'g':
        ( ( scan_node* )CirLinkList_peek( controller->freqs ) )->gain = ( int )( atof(
              optarg ) * 10 );
        break;
      case 'l':
        ( ( scan_node* )CirLinkList_peek( controller->freqs ) )->squelch_level =
          ( int )atof( optarg );
        break;
      case 's':
        demod->rate_in = ( uint32_t )atofs( optarg );
        demod->rate_out = ( uint32_t )atofs( optarg );
        break;
      case 'r':
        output->rate = ( int )atofs( optarg );
        demod->rate_out2 = ( int )atofs( optarg );
        break;
      case 'o':
        fprintf( stderr, "Warning: -o is very buggy\n" );
        demod->post_downsample = ( int )atof( optarg );
        if ( demod->post_downsample < 1 ||
             demod->post_downsample > MAXIMUM_OVERSAMPLE ) {
          fprintf( stderr, "Oversample must be between 1 and %i\n", MAXIMUM_OVERSAMPLE );
        }
        break;
      case 't':
        //TODO: set per freq
        demod->conseq_squelch = ( int )atof( optarg );
        if ( demod->conseq_squelch < 0 ) {
          demod->conseq_squelch = -demod->conseq_squelch;
          demod->terminate_on_squelch = 1;
        }
        break;
      case 'p':
        dongle->ppm_error = atoi( optarg );
        custom_ppm = 1;
        break;
      case 'E':
        //TODO: add some to perfreq
        if ( strcmp( "edge",  optarg ) == 0 ) {
          controller->edge = 1;
        }
        if ( strcmp( "dc", optarg ) == 0 ) {
          demod->dc_block = 1;
        }
        if ( strcmp( "deemp",  optarg ) == 0 ) {
          demod->deemph = 1;
        }
        if ( strcmp( "direct",  optarg ) == 0 ) {
          dongle->direct_sampling = 1;
        }
        if ( strcmp( "offset",  optarg ) == 0 ) {
          dongle->offset_tuning = 1;
        }
        break;
      case 'F':
        demod->downsample_passes = 1;  /* truthy placeholder */
        demod->comp_fir_size = atoi( optarg );
        break;
      case 'A':
        if ( strcmp( "std",  optarg ) == 0 ) {
          demod->custom_atan = 0;
        }
        if ( strcmp( "fast", optarg ) == 0 ) {
          demod->custom_atan = 1;
        }
        if ( strcmp( "lut",  optarg ) == 0 ) {
          atan_lut_init();
          demod->custom_atan = 2;
        }
        break;
      case 'M':
        ( ( scan_node* )CirLinkList_peek( controller->freqs ) )->mod = optarg;
        setdemod( demod, optarg );
        break;
      case 'h':
      default:
        usage();
        break;
    }
  }

  /* quadruple sample_rate to limit to Δθ to ±π/2 */
  demod->rate_in *= demod->post_downsample;

  if ( !output->rate ) {
    output->rate = demod->rate_out;
  }

  sanity_checks( controller );

  if ( controller->freqs->size > 1 ) {
    demod->terminate_on_squelch = 0;
  }

  if ( argc <= optind ) {
    output->filename = "-";
  } else {
    output->filename = argv[optind];
  }

  ACTUAL_BUF_LENGTH = lcm_post[demod->post_downsample] * DEFAULT_BUF_LENGTH;

  if ( !dev_given ) {
    dongle->dev_index = verbose_device_search( "0" );
  }

  if ( dongle->dev_index < 0 ) {
    exit( 1 );
  }

  r = rtlsdr_open( &dongle->dev, ( uint32_t )dongle->dev_index );
  if ( r < 0 ) {
    fprintf( stderr, "Failed to open rtlsdr device #%d.\n", dongle->dev_index );
    exit( 1 );
  }
#ifndef _WIN32
  sigact.sa_handler = sighandler;
  sigemptyset( &sigact.sa_mask );
  sigact.sa_flags = 0;
  sigaction( SIGINT, &sigact, NULL );
  sigaction( SIGTERM, &sigact, NULL );
  sigaction( SIGQUIT, &sigact, NULL );
  sigaction( SIGPIPE, &sigact, NULL );
#else
  SetConsoleCtrlHandler( ( PHANDLER_ROUTINE ) sighandler, TRUE );
#endif

  if ( demod->deemph ) {
    demod->deemph_a = ( int )round( 1.0 / ( ( 1.0 - exp( -1.0 /
                                            ( demod->rate_out * 75e-6 ) ) ) ) );
  }

  /* Set the tuner gain */
  if ( dongle->gain == AUTO_GAIN ) {
    verbose_auto_gain( dongle->dev );
  } else {
    dongle->gain = nearest_gain( dongle->dev, dongle->gain );
    verbose_gain_set( dongle->dev, dongle->gain );
  }

  verbose_ppm_set( dongle->dev, dongle->ppm_error );

  if ( strcmp( output->filename, "-" ) == 0 ) { /* Write samples to stdout */
    output->file = stdout;
#ifdef _WIN32
    _setmode( _fileno( output->file ), _O_BINARY );
#endif
  } else {
    output->file = fopen( output->filename, "wb" );
    if ( !output->file ) {
      fprintf( stderr, "Failed to open %s\n", output->filename );
      exit( 1 );
    }
  }

  //r = rtlsdr_set_testmode(dongle->dev, 1);

  /* Reset endpoint before we start reading from it (mandatory) */
  verbose_reset_buffer( dongle->dev );

  pthread_create( &controller->thread, NULL, controller_thread_fn,
                  ( void* )( dongle ) );
  usleep( 100000 );
  pthread_create( &output->thread, NULL, output_thread_fn, ( void* )( dongle ) );
  pthread_create( &demod->thread, NULL, demod_thread_fn, ( void* )( dongle ) );
  pthread_create( &dongle->thread, NULL, dongle_thread_fn, ( void* )( dongle ) );


  while ( !do_exit ) {
    usleep( 100000 );
  }

  if ( do_exit ) {
    fprintf( stderr, "\nUser cancel, exiting...\n" );
  } else {
    fprintf( stderr, "\nLibrary error %d, exiting...\n", r );
  }

  rtlsdr_cancel_async( dongle->dev );
  pthread_join( dongle->thread, NULL );
  fprintf( stderr, "Dongle Ended\n" );
  safe_cond_signal( &demod->ready, &demod->ready_m );
  pthread_join( demod->thread, NULL );
  fprintf( stderr, "Demod Ended\n" );
  safe_cond_signal( &output->ready, &output->ready_m );
  pthread_join( output->thread, NULL );
  fprintf( stderr, "Output Ended\n" );
  safe_cond_signal( &controller->hop, &controller->hop_m );
  pthread_join( controller->thread, NULL );
  fprintf( stderr, "Controller Ended\n" );



  demod_cleanup( demod );
  output_cleanup( output );
  controller_cleanup( controller );
  dongle_cleanup( dongle ); //dongle stopped in sighandle


  return r >= 0 ? r : -r;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
