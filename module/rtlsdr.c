/*
 * USB rtlsdr driver - 2.2
 *
 * Copyright (C) 2015 Timothy Dawson (tdawson@waikato.ac.nz)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * but has been rewritten to be easier to read and use.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/circ_buf.h>
#include <linux/mm.h>

#include <linux/time.h>
#include <linux/ktime.h>

#include "rtlsdr.h"

/* Define these values to match your devices */
#define USB_rtlsdr_VENDOR_ID	0x0bda
#define USB_rtlsdr_PRODUCT_ID	0x2838

/* table of devices that work with this driver */
static const struct usb_device_id rtlsdr_table[] = {
  { USB_DEVICE( USB_rtlsdr_VENDOR_ID, USB_rtlsdr_PRODUCT_ID ) },
  { }					/* Terminating entry */
};
MODULE_DEVICE_TABLE( usb, rtlsdr_table );


/* Get a minor range for your devices from the usb maintainer */
#define USB_rtlsdr_MINOR_BASE	200


/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */
#define WRITES_IN_FLIGHT	8
/* arbitrarily chosen */

#define CONCURRENT_READS        16

/* Structure to hold all of our device specific stuff */
struct usb_rtlsdr {
  struct usb_device*	udev;			/* the usb device for this device */
  struct usb_interface*	interface;		/* the interface for this device */
  struct usb_anchor	submitted;		/* in case we need to retract our submissions */
  struct urb*		ctrl_urb;		/* the urb todo ctrl with */


/* usb Buffers */
  struct urb*		        bulk_in_urb[CONCURRENT_READS];		/* the urbs to read data with */
  unsigned char*         bulk_in_buffer_urb[CONCURRENT_READS];	/* the data buffers*/
  unsigned long			bulk_in_size_urb;		/* the size of the buffers */
  int	        		bulk_in_number_urb;		/* the number of buffers */
  
/* ring buffer */
  unsigned char*                bulk_in_buffer;	        /* the data buffer*/
  unsigned long			bulk_in_size;		/* the size of the buffer */
  unsigned long			bulk_in_tail;		/* the point at which the consumer finds the next item in the buffer */
  unsigned long			bulk_in_head;		/* the point at which the producer inserts items into the buffer */
  spinlock_t                    producer_lock;
  spinlock_t                    consumer_lock;
  
  __u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
  __u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
  int			errors;			/* the last request tanked */
  bool			ongoing_read;		/* a read is going on */
  spinlock_t		err_lock;		/* lock for errors */
  struct kref		kref;
  struct mutex		io_mutex;		/* synchronize I/O with disconnect */
  struct mutex          dev_mutex;              /* device global mutex */
  wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */
};

#define to_rtlsdr_dev(d) container_of(d, struct usb_rtlsdr, kref)

static struct usb_driver rtlsdr_driver;
static void rtlsdr_draw_down( struct usb_rtlsdr* dev );

static void rtlsdr_delete( struct kref* kref )
{
  int i;
  struct usb_rtlsdr* dev = to_rtlsdr_dev( kref );

  for(i=0; i<dev->bulk_in_number_urb; i++){
        usb_free_urb( dev->bulk_in_urb[i] );
        kfree( dev->bulk_in_buffer_urb[i] );
  }
  
  usb_put_dev( dev->udev );
  free_pages(dev->bulk_in_buffer, get_order(dev->bulk_in_size) );
  kfree( dev );
}

static int rtlsdr_open( struct inode* inode, struct file* file )
{
  struct usb_rtlsdr* dev;
  struct usb_interface* interface;
  int subminor;
  int retval = 0;

  subminor = iminor( inode );

  interface = usb_find_interface( &rtlsdr_driver, subminor );
  if ( !interface ) {
    pr_err( "%s - error, can't find device for minor %d\n",
            __func__, subminor );
    retval = -ENODEV;
    goto exit;
  }

  dev = usb_get_intfdata( interface );
  if ( !dev ) {
    retval = -ENODEV;
    goto exit;
  }

  retval = usb_autopm_get_interface( interface );
  if ( retval ) {
    goto exit;
  }

  /* increment our usage count for the device */
  kref_get( &dev->kref );

  /* save our object in the file's private structure */
  file->private_data = dev;

  
  dev_info( &interface->dev, "rtlsdr-%d %s", interface->minor, __FUNCTION__);

  //zero ringbuf
  dev->bulk_in_head = 0;
  dev->bulk_in_tail = 0;
  smp_store_release(&dev->bulk_in_tail, 0 );
  smp_store_release(&dev->bulk_in_head, 0 );
  
exit:
  return retval;
}


static int rtlsdr_release( struct inode* inode, struct file* file )
{
  struct usb_rtlsdr* dev;

  dev = file->private_data;
  if ( dev == NULL ) {
    return -ENODEV;
  }

  /* allow the device to be autosuspended */
  mutex_lock( &dev->io_mutex );
  if ( dev->interface ) {
    usb_autopm_put_interface( dev->interface );
  }
  mutex_unlock( &dev->io_mutex );

  /* decrement the count on our device */
  kref_put( &dev->kref, rtlsdr_delete );
  return 0;
}

static int rtlsdr_flush( struct file* file, fl_owner_t id )
{
  struct usb_rtlsdr* dev;
  int res;

  dev = file->private_data;
  if ( dev == NULL ) {
    return -ENODEV;
  }

  /* wait for io to stop */
  mutex_lock( &dev->io_mutex );
  rtlsdr_draw_down( dev );

  /* read out errors, leave subsequent opens a clean slate */
  spin_lock_irq( &dev->err_lock );
  res = dev->errors ? ( dev->errors == -EPIPE ? -EPIPE : -EIO ) : 0;
  dev->errors = 0;
  spin_unlock_irq( &dev->err_lock );

  mutex_unlock( &dev->io_mutex );
  
  //spin_unlock(&dev->consumer_lock);
  
  return res;
}


static void rtlsdr_continuous_read_callback( struct urb* urb ){

  struct usb_rtlsdr* dev;
  struct timespec ts; //time of arrival (TOA)
  //ktime_t tv; 
  int rv;
  size_t i;
  unsigned long head,tail;

  dev = urb->context;

  //tv = current_kernel_time();
  //getnstimeofday(&tv);
  ktime_get_ts(&ts); //CLOCK_MONOTONIC - Raw time scaled by ntp (adjtime) to correct frequancy
  //tv = ktime_get();
  //dev_info( &dev->interface->dev, "rtlsdr-%d %s, TOA: %lu.%lu", dev->interface->minor, __FUNCTION__, tv.tv_sec, tv.tv_usec);

  spin_lock(&dev->producer_lock);
  
  head = dev->bulk_in_head;
  tail = ACCESS_ONCE(dev->bulk_in_tail);

  if(urb->actual_length < urb->transfer_buffer_length){
          dev_info( &dev->interface->dev, "rtlsdr-%d %s, urb req %u < rxed: %u", dev->interface->minor, __FUNCTION__, urb->transfer_buffer_length , urb->actual_length);
  }
  
  if (CIRC_SPACE(head, tail, dev->bulk_in_size) >= urb->actual_length+sizeof(struct timespec)){

    //dev_info( &dev->interface->dev, "rtlsdr-%d %s adding to circ free: %lu urb: %u", dev->interface->minor, __FUNCTION__, CIRC_SPACE(head, tail, dev->bulk_in_size) , urb->actual_length);

    /* Copy in timestamp */
    for(i=0; i<sizeof(struct timespec) ; i++){
      dev->bulk_in_buffer[ (head+i) & (dev->bulk_in_size-1) ] = ((char*)&ts)[i];
    }

    head = ( head + sizeof(ktime_t) ) & (dev->bulk_in_size-1); //Add timestamp to ringbuf
    
    /* Copy in USB payload */
    for(i=0;i<urb->actual_length;i++){
      dev->bulk_in_buffer[ (head+i) & (dev->bulk_in_size-1) ] = ((char*)urb->transfer_buffer)[i];
    }

    smp_store_release(&dev->bulk_in_head, (head + urb->actual_length) & (dev->bulk_in_size - 1) );
    /* wake_up() will make sure that the head is committed before
    * waking anyone up */
    wake_up_interruptible( &dev->bulk_in_wait ); //wake up threads sleeping for data
  
  }else{
    //we overrun our circ buffer
    dev_info( &dev->interface->dev, "rtlsdr-%d %s overrun", dev->interface->minor, __FUNCTION__);
    //dev_info( &dev->interface->dev, "rtlsdr-%d %s circ free: %lu urb: %u", dev->interface->minor, __FUNCTION__, CIRC_SPACE(head, tail, dev->bulk_in_size) , urb->actual_length);
  }
  spin_unlock(&dev->producer_lock);
  
  //submit urb again
  rv = usb_submit_urb( urb, GFP_ATOMIC );
  if ( rv < 0 ) {
          dev_info( &dev->interface->dev, "%s - failed submitting read urb, error %d\n", __func__, rv );
          rv = ( rv == -ENOMEM ) ? rv : -EIO;
          spin_lock_irq( &dev->err_lock );
          dev->ongoing_read = 0;
          spin_unlock_irq( &dev->err_lock );
  }else{
          usb_anchor_urb(urb, &dev->submitted);
  }

}

static int rtlsdr_do_read_io( struct usb_rtlsdr* dev )
{
  int rv;
  int i;

  /* tell everybody to leave the URB alone */
  spin_lock_irq( &dev->err_lock );
  dev->ongoing_read = 1;
  spin_unlock_irq( &dev->err_lock );

  //reset state
  usb_reset_endpoint(dev->udev,0x81);
  /* prepare a reads */
  for(i=0; i<dev->bulk_in_number_urb; i++){
        usb_fill_bulk_urb(   dev->bulk_in_urb[i],
                             dev->udev,
                             usb_rcvbulkpipe( dev->udev, 0x81),
                             dev->bulk_in_buffer_urb[i],
                             dev->bulk_in_size_urb,
                             rtlsdr_continuous_read_callback,
                             dev );

        rv = usb_submit_urb( dev->bulk_in_urb[i], GFP_ATOMIC );
        if ( rv < 0 ) {
                dev_err( &dev->interface->dev,  "%s - failed submitting read urb, error %d\n", __func__, rv);
                rv = ( rv == -ENOMEM ) ? rv : -EIO;
                spin_lock_irq( &dev->err_lock );
                dev->ongoing_read = 0;
                spin_unlock_irq( &dev->err_lock );
                return rv;
        }else{
                usb_anchor_urb(dev->bulk_in_urb[i], &dev->submitted);
        }
  }
  return rv;
}

static ssize_t rtlsdr_read_blocking ( struct file* file, char* buffer, size_t count, loff_t* ppos )
{
  int res;
  struct usb_rtlsdr* dev;
  struct usb_interface*	interface;
  int length;

  uint8_t * buf;  
  count = min(count,(size_t)512); //Limit packet size
  
  dev = file->private_data;
  if ( dev == NULL ) {
    return -ENODEV;
  }
  interface = dev->interface;

  buf = kmalloc (count , GFP_KERNEL);
  
  res = usb_bulk_msg (  dev->udev,
                        usb_rcvbulkpipe( dev->udev, 0x81 ),
                        buf,
                        count,
                        &length,
                        1000);
  if(res!=0){
    return res;
  }

  copy_to_user(buffer, buf, length); //copy into userspace
  dev_info( &dev->interface->dev, "rtlsdr-%d %s read, copied: %u", dev->interface->minor, __FUNCTION__, length);
  kfree(buf);

  return length;
}


static ssize_t rtlsdr_continuouse_read( struct file* file, char* buffer, size_t count,
                            loff_t* ppos )
{
        struct usb_rtlsdr* dev;
        ssize_t len,nocopy; //data read
        unsigned long head, tail;
        size_t i;
	
        dev = file->private_data;
        /* device disconected */
        if(!dev || !dev->interface || !&dev->interface->dev) return -EIO;
        
        //Start up the callback
        if(dev->ongoing_read == 0){
                rtlsdr_do_read_io( dev ); //full urb and submit for the first time
        }
        
retry:
	/* Read index before reading contents at that index. */
	head = smp_load_acquire(&dev->bulk_in_head);
	tail = dev->bulk_in_tail;

        
	if ( (len=CIRC_CNT(head, tail, dev->bulk_in_size)) > 0) {

                /* write continuoise block */                
                len = min((size_t)len,count); //make sure we dont copy more data then we have

                if( !access_ok (VERIFY_WRITE, buffer, len)){
                        dev_err( &dev->interface->dev, "rtlsdr-%d %s unsafe pointer", dev->interface->minor, __FUNCTION__);
                }
                
                nocopy = copy_to_user(buffer, dev->bulk_in_buffer+tail, len); //returns number bytes copied
                //dev_info( &dev->interface->dev, "rtlsdr-%d %s reading, copied: %lu of %lu", dev->interface->minor, __FUNCTION__, len-nocopy, (size_t)len);
                len-=nocopy;//data not copied
                
                /* Finish reading descriptor before incrementing tail. */
                smp_store_release(&dev->bulk_in_tail, (tail + len) & (dev->bulk_in_size - 1) );
	} else {

                /* nonblocking IO shall not wait */
                if ( file->f_flags & O_NONBLOCK ) {
                  dev_info( &dev->interface->dev, "rtlsdr-%d %s underrun", dev->interface->minor, __FUNCTION__);
                  len = -EAGAIN;
                  goto exit;
                }else{
                  /* wait 1000 jiffes and if there is data read it out */
                  if( wait_event_interruptible_timeout ( dev->bulk_in_wait , (head != smp_load_acquire(&dev->bulk_in_head) ), msecs_to_jiffies(1000) ) == 0 ){
                        goto retry;
                  }     
                  return 0;
                }
        }
        
exit:
        return len;
}

int rtlsdr_do_control(struct usb_rtlsdr * dev, struct rtlsdr_ctrltransfer * trf){

  int res;
  unsigned int pipe;
  struct rtlsdr_ctrltransfer * ctrltransfer;
  void * data;

  ctrltransfer = kmalloc (sizeof(struct rtlsdr_ctrltransfer) , GFP_KERNEL);
  copy_from_user(ctrltransfer, trf, sizeof(struct rtlsdr_ctrltransfer) );

  data = kmalloc (ctrltransfer->wLength , GFP_KERNEL);
  copy_from_user(data, ctrltransfer->data, ctrltransfer->wLength);


  if( (0x80&ctrltransfer->bRequestType) == 0x80){
    pipe = usb_rcvctrlpipe(dev->udev, 0);
  }else{
    pipe = usb_sndctrlpipe(dev->udev, 0);
  }

  res = usb_control_msg ( dev->udev,
                          pipe,
                          ctrltransfer->bRequest,
                          ctrltransfer->bRequestType,
                          ctrltransfer->wValue,
                          ctrltransfer->wIndex,
                          data,
                          ctrltransfer->wLength,
                          ctrltransfer->timeout );

  dev_info( &dev->interface->dev, "rtlsdr-%d %s: %d ", dev->interface->minor, __FUNCTION__,res);

  if(res>0){
    //copy data back to user
    copy_to_user(ctrltransfer->data, data, res);
  }
  
  //kfree
  kfree (	data);
  kfree (	ctrltransfer );

  return res;

}


static long rtlsdr_ioctl (struct file * file, unsigned int cmd, unsigned long arg)
{
  long err;
  struct usb_rtlsdr* dev;
  struct usb_interface*	interface;
  
  dev = file->private_data;
  if ( dev == NULL ) {

    return -ENODEV;
  }
  
  if(!dev || !dev->interface || !&dev->interface->dev) return -EIO;
  
  interface = dev->interface;
  err=-1;

  
  switch(cmd){
    case RTLSDR_CONTROL:{
      dev_info( &interface->dev, "rtlsdr-%d %s(control)", interface->minor, __FUNCTION__);
      err = rtlsdr_do_control(dev, (struct rtlsdr_ctrltransfer*) arg);
      break;
    }
    case RTLSDR_RESET:{
      dev_info( &interface->dev, "rtlsdr-%d %s(reset)", interface->minor, __FUNCTION__);
      break;
    }
    
    default:{
      dev_info( &interface->dev, "rtlsdr-%d %s(%#x)", interface->minor, __FUNCTION__, cmd);
    }
  }

  return err;
}

static const struct file_operations rtlsdr_fops = {
  .owner =	THIS_MODULE,
  .read =		rtlsdr_continuouse_read,
  //.read = rtlsdr_read_blocking,
  //.write =	rtlsdr_write,
  .open =	rtlsdr_open,
  .release =	rtlsdr_release,
  .flush =	rtlsdr_flush,
  .llseek =	noop_llseek,
  .unlocked_ioctl = rtlsdr_ioctl,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have th       e device registered with the driver core
 */
static struct usb_class_driver rtlsdr_class = {
  .name =		"rtlsdr%d",
  .fops =		&rtlsdr_fops,
  //.minor_base =	USB_rtlsdr_MINOR_BASE,
};

static int rtlsdr_probe( struct usb_interface* interface,
                         const struct usb_device_id* id )
{
  struct usb_rtlsdr* dev;
  struct usb_host_interface* iface_desc;
  struct usb_endpoint_descriptor* endpoint;
  size_t buffer_size;
  int i;
  int retval = -ENOMEM;

  /* allocate memory for our device state and initialize it */
  dev = kzalloc( sizeof( *dev ), GFP_KERNEL );
  if ( !dev ) {
    dev_err( &interface->dev, "Out of memory\n" );
    goto error;
  }
  kref_init( &dev->kref );
  mutex_init( &dev->io_mutex );
  spin_lock_init( &dev->err_lock );
  init_usb_anchor( &dev->submitted );
  init_waitqueue_head( &dev->bulk_in_wait );

  spin_lock_init(&dev->consumer_lock);
  spin_lock_init(&dev->producer_lock);

  dev->udev = usb_get_dev( interface_to_usbdev( interface ) );
  dev->interface = interface;

  /* set up the endpoint information */
  /* Find rtlsdr endpoints */
  iface_desc = interface->cur_altsetting;

    /* Wrong interface we only want interface with bulk ep */
  if( iface_desc->desc.bNumEndpoints == 0 ) {
    retval = -1; //not really an error
    usb_set_intfdata( interface, NULL );
    goto error;
  }

/* setup Endpoint */
        endpoint = &iface_desc->endpoint[0].desc;

        dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;

        buffer_size = 32768; //ring buffer size, must be a power of two 

        dev->bulk_in_buffer = (unsigned char*)__get_free_pages(GFP_KERNEL, get_order(buffer_size));
        dev->bulk_in_size = buffer_size;
        dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;

        if (!dev->bulk_in_buffer) {
                dev_err(&interface->dev,
                "Could not allocate bulk_in_buffer\n");
                goto error;
        }


        dev->bulk_in_size_urb = 512;
        dev->bulk_in_number_urb = CONCURRENT_READS;
        
        for(i=0; i<dev->bulk_in_number_urb; i++){
                dev->bulk_in_buffer_urb[i] = kmalloc(dev->bulk_in_size_urb, GFP_KERNEL);
                if (!dev->bulk_in_buffer_urb[i]) {
                        dev_err(&interface->dev,
                        "Could not allocate bulk_in_buffer_urb, %u\n", i);
                        goto error;
                }
                
                dev->bulk_in_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
                if (!dev->bulk_in_urb[i]) {
                        dev_err(&interface->dev,
                        "Could not allocate bulk_in_urb\n");
                        goto error;
                }
                //anchor the urbs so they can be canceled
                //usb_anchor_urb(dev->bulk_in_urb[i], &dev->submitted);
        }

        //zero ringbuf
        dev->bulk_in_head = 0;
        dev->bulk_in_tail = 0;

        dev_info( &interface->dev, "Endpoint addr: 0x%x\n", endpoint->bEndpointAddress );
        dev_info( &interface->dev, "Endpoint buffer size: %llu\n", (uint64_t)dev->bulk_in_size_urb );


  /* save our data pointer in this interface device */
  usb_set_intfdata( interface, dev );

  /* we can register the device now, as it is ready */
  retval = usb_register_dev( interface, &rtlsdr_class );
  dev_info( &interface->dev, "Dev minor: %u\n", interface->minor );
  if ( retval ) {
    /* something prevented us from registering this driver */
    dev_err( &interface->dev,
             "Not able to get a minor for this device.\n" );
    usb_set_intfdata( interface, NULL );
    goto error;
  }

  /* let the user know what node this device is now attached to */
  dev_info( &interface->dev,
            "USB rtlsdr device now attached to rtlsdr-%d",
            interface->minor );
  return 0;

error:
  if ( dev )
    /* this frees allocated memory */
  {
    kref_put( &dev->kref, rtlsdr_delete );
  }
  return retval;
}



static void rtlsdr_disconnect( struct usb_interface* interface )
{
  struct usb_rtlsdr* dev;
  int minor = interface->minor;

  dev = usb_get_intfdata( interface );
  usb_set_intfdata( interface, NULL );

  /* give back our minor */
  usb_deregister_dev( interface, &rtlsdr_class );

  /* stop current I/O */
  rtlsdr_draw_down(dev);

  /* prevent more I/O from starting */
  mutex_lock( &dev->io_mutex );
  dev->interface = NULL;
  mutex_unlock( &dev->io_mutex );

  /* decrement our usage count */
  kref_put( &dev->kref, rtlsdr_delete );

  dev_info( &interface->dev, "USB rtlsdr #%d now disconnected", minor );
}

//stop all transfers
static void rtlsdr_draw_down( struct usb_rtlsdr* dev )
{
  if(!dev || !dev->interface || !&dev->interface->dev) return;

  dev_info( &dev->interface->dev,  "%s - start", __func__);
  usb_kill_anchored_urbs(&dev->submitted);
  if( !usb_wait_anchor_empty_timeout(&dev->submitted, 1000) ){
    dev_info( &dev->interface->dev,  "%s - timeout", __func__);
  }else{
    dev_info( &dev->interface->dev,  "%s - done", __func__);
  }
}

static int rtlsdr_suspend( struct usb_interface* intf, pm_message_t message )
{
  struct usb_rtlsdr* dev = usb_get_intfdata( intf );

  if ( !dev ) {
    return 0;
  }
  rtlsdr_draw_down( dev );
  return 0;
}

static int rtlsdr_resume( struct usb_interface* intf )
{
  return 0;
}

static int rtlsdr_pre_reset( struct usb_interface* intf )
{
  struct usb_rtlsdr* dev = usb_get_intfdata( intf );

  mutex_lock( &dev->io_mutex );
  rtlsdr_draw_down( dev );

  return 0;
}

static int rtlsdr_post_reset( struct usb_interface* intf )
{
  struct usb_rtlsdr* dev = usb_get_intfdata( intf );

  /* we are sure no URBs are active - no locking needed */
  dev->errors = -EPIPE;
  mutex_unlock( &dev->io_mutex );

  return 0;
}



static struct usb_driver rtlsdr_driver = {
  .name =		"rtlsdr",
  .probe =	rtlsdr_probe,
  .disconnect =	rtlsdr_disconnect,
  .suspend =	rtlsdr_suspend,
  .resume =	rtlsdr_resume,
  .pre_reset =	rtlsdr_pre_reset,
  .post_reset =	rtlsdr_post_reset,
  .id_table =	rtlsdr_table,
  .supports_autosuspend = 1,
};

module_usb_driver( rtlsdr_driver );

MODULE_LICENSE( "GPL" );
