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

/* Structure to hold all of our device specific stuff */
struct usb_rtlsdr {
  struct usb_device*	udev;			/* the usb device for this device */
  struct usb_interface*	interface;		/* the interface for this device */
  struct semaphore	limit_sem;		/* limiting the number of writes in progress */
  struct usb_anchor	submitted;		/* in case we need to retract our submissions */
  struct urb*		ctrl_urb;		/* the urb todo ctrl with */
  struct urb*		bulk_in_urb;		/* the urb to read data with */

/* usb Buffers */
  unsigned char*                bulk_in_buffer_urb;	        /* the data buffer*/
  unsigned long			bulk_in_size_urb;		/* the size of the buffer */
  
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
  struct usb_rtlsdr* dev = to_rtlsdr_dev( kref );

  usb_free_urb( dev->bulk_in_urb );
  usb_put_dev( dev->udev );
  kfree( dev->bulk_in_buffer );
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

  return res;
}


static void rtlsdr_continuous_read_callback( struct urb* urb ){

        struct usb_rtlsdr* dev;
        dev = urb->context;
        size_t i;
        
	spin_lock(&dev->producer_lock);

	unsigned long head = dev->bulk_in_head;
	/* The spin_unlock() and next spin_lock() provide needed ordering. */
	unsigned long tail = ACCESS_ONCE(dev->bulk_in_tail);

	if (CIRC_SPACE(head, tail, dev->bulk_in_size) >= urb->actual_length) {

                /* insert one 512 usb packet worth of data into the buffer */
                for(i=0;i<urb->actual_length;i++){
                  dev->bulk_in_buffer[ (head+i) & (dev->bulk_in_size-1) ]=dev->bulk_in_buffer_urb[i];
                }

		smp_store_release(dev->bulk_in_head, (head + urb->actual_length) & (dev->bulk_in_size - 1) );

		/* wake_up() will make sure that the head is committed before
		 * waking anyone up */
		//wake_up(consumer);
	}else{
         //we overrun our circ buffer
         dev_info( &dev->interface->dev, "rtlsdr-%d %s overrun.", dev->interface->minor, __FUNCTION__);
        }
        
        //submit urb again
        rv = usb_submit_urb( dev->bulk_in_urb, GFP_KERNEL );
        if ( rv < 0 ) {
                dev_err( &dev->interface->dev,
                     "%s - failed submitting read urb, error %d\n",
                     __func__, rv );
                     
                rv = ( rv == -ENOMEM ) ? rv : -EIO;
                spin_lock_irq( &dev->err_lock );
                dev->ongoing_read = 0;
                spin_unlock_irq( &dev->err_lock );
        }

        wake_up_interruptible( &dev->bulk_in_wait ); //wake up threads sleeping for data
	spin_unlock(&dev->producer_lock);

}

static int rtlsdr_do_read_io( struct usb_rtlsdr* dev )
{
  int rv;

  /* prepare a read */
  usb_fill_bulk_urb( dev->bulk_in_urb,
                     dev->udev,
                     usb_rcvbulkpipe( dev->udev, 0x81),
                     dev->bulk_in_buffer_urb,
                     dev->bulk_in_size_urb,
                     rtlsdr_continuous_read_callback,
                     dev );
                     
  /* tell everybody to leave the URB alone */
  spin_lock_irq( &dev->err_lock );
  dev->ongoing_read = 1;
  spin_unlock_irq( &dev->err_lock );

  /* submit bulk in urb, which means no data to deliver */
  dev->bulk_in_head = 0;
  dev->bulk_in_tail = 0;

  /* do it */
  rv = usb_submit_urb( dev->bulk_in_urb, GFP_KERNEL );
  if ( rv < 0 ) {
    dev_err( &dev->interface->dev,
             "%s - failed submitting read urb, error %d\n",
             __func__, rv );
             
    rv = ( rv == -ENOMEM ) ? rv : -EIO;
    spin_lock_irq( &dev->err_lock );
    dev->ongoing_read = 0;
    spin_unlock_irq( &dev->err_lock );
  }

  return rv;
}

static ssize_t rtlsdr_read_blocking ( struct file* file, char* buffer, size_t count, loff_t* ppos )
{
  int res;
  struct usb_rtlsdr* dev;
  struct usb_interface*	interface;
  int length;
  int dataread;

  uint8_t * buf;  
  
  dev = file->private_data;
  if ( dev == NULL ) {
    return -ENODEV;
  }
  interface = dev->interface;

  buf = kmalloc (count , GFP_KERNEL);
  
  res = usb_bulk_msg ( dev->udev,
                    usb_rcvbulkpipe( dev->udev, 0x81 ),
                    buf,
                    count,
                    &length,
                    1000);
  if(res!=0){
    return res;
  }

  copy_to_user(buffer, buf, length); //copy into userspace
  kfree(buf);
  return length;
}


static ssize_t rtlsdr_continuouse_read( struct file* file, char* buffer, size_t count,
                            loff_t* ppos )
{
        struct usb_rtlsdr* dev;
        ssize_t len; //data read
        
	spin_lock(&dev->consumer_lock);
        dev = file->private_data;

        //Start up the callback
        if(dev->ongoing_read == 0){
                rtlsdr_do_read_io( dev ); //full urb and submit for the first time
        }
retry:
	/* Read index before reading contents at that index. */
	unsigned long head = smp_load_acquire(dev->bulk_in_head);
	unsigned long tail = dev->bulk_in_tail;

	if (CIRC_CNT(head, tail, dev->bulk_in_size) >= 1) {

		/* write continuoise block */
                len = CIRC_SPACE_TO_END(head, tail, dev->bulk_in_size);
                len = min(len,count); //make sure we dont copy to much data
                
                copy_to_user(buffer, &dev->bulk_in_buffer[tail], len); 

		/* Finish reading descriptor before incrementing tail. */
		smp_store_release(dev->bulk_in_tail, (tail + len) & (dev->bulk_in_size - 1));
	} else {

                /* nonblocking IO shall not wait */
                if ( file->f_flags & O_NONBLOCK ) {
                  len = -EAGAIN;
                }else{
                  wait_event_interruptible( dev->bulk_in_wait, ( dev->ongoing_read ) );
                  goto retry;
                }
        }
        
exit:
	spin_unlock(&consumer_lock);

        return len;
}

static void rtlsdr_write_bulk_callback( struct urb* urb )
{
  struct usb_rtlsdr* dev;

  dev = urb->context;

  /* sync/async unlink faults aren't errors */
  if ( urb->status ) {
    if ( !( urb->status == -ENOENT ||
            urb->status == -ECONNRESET ||
            urb->status == -ESHUTDOWN ) )
      dev_err( &dev->interface->dev,
               "%s - nonzero write bulk status received: %d\n",
               __func__, urb->status );

    spin_lock( &dev->err_lock );
    dev->errors = urb->status;
    spin_unlock( &dev->err_lock );
  }

  /* free up our allocated buffer */
  usb_free_coherent( urb->dev, urb->transfer_buffer_length,
                     urb->transfer_buffer, urb->transfer_dma );
  up( &dev->limit_sem );
}

static ssize_t rtlsdr_write( struct file* file, const char* user_buffer,
                             size_t count, loff_t* ppos )
{
  struct usb_rtlsdr* dev;
  int retval = 0;
  struct urb* urb = NULL;
  char* buf = NULL;
  size_t writesize = min( count, ( size_t )MAX_TRANSFER );

  dev = file->private_data;

  /* verify that we actually have some data to write */
  if ( count == 0 ) {
    goto exit;
  }

  /*
   * limit the number of URBs in flight to stop a user from using up all
   * RAM
   */
  if ( !( file->f_flags & O_NONBLOCK ) ) {
    if ( down_interruptible( &dev->limit_sem ) ) {
      retval = -ERESTARTSYS;
      goto exit;
    }
  } else {
    if ( down_trylock( &dev->limit_sem ) ) {
      retval = -EAGAIN;
      goto exit;
    }
  }

  spin_lock_irq( &dev->err_lock );
  retval = dev->errors;
  if ( retval < 0 ) {
    /* any error is reported once */
    dev->errors = 0;
    /* to preserve notifications about reset */
    retval = ( retval == -EPIPE ) ? retval : -EIO;
  }
  spin_unlock_irq( &dev->err_lock );
  if ( retval < 0 ) {
    goto error;
  }

  /* create a urb, and a buffer for it, and copy the data to the urb */
  urb = usb_alloc_urb( 0, GFP_KERNEL );
  if ( !urb ) {
    retval = -ENOMEM;
    goto error;
  }

  buf = usb_alloc_coherent( dev->udev, writesize, GFP_KERNEL,
                            &urb->transfer_dma );
  if ( !buf ) {
    retval = -ENOMEM;
    goto error;
  }

  if ( copy_from_user( buf, user_buffer, writesize ) ) {
    retval = -EFAULT;
    goto error;
  }

  /* this lock makes sure we don't submit URBs to gone devices */
  mutex_lock( &dev->io_mutex );
  if ( !dev->interface ) {		/* disconnect() was called */
    mutex_unlock( &dev->io_mutex );
    retval = -ENODEV;
    goto error;
  }

  /* initialize the urb properly */
  usb_fill_bulk_urb( urb, dev->udev,
                     usb_sndbulkpipe( dev->udev, dev->bulk_out_endpointAddr ),
                     buf, writesize, rtlsdr_write_bulk_callback, dev );
  urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
  usb_anchor_urb( urb, &dev->submitted );

  /* send the data out the bulk port */
  retval = usb_submit_urb( urb, GFP_KERNEL );
  mutex_unlock( &dev->io_mutex );
  if ( retval ) {
    dev_err( &dev->interface->dev,
             "%s - failed submitting write urb, error %d\n",
             __func__, retval );
    goto error_unanchor;
  }

  /*
   * release our reference to this urb, the USB core will eventually free
   * it entirely
   */
  usb_free_urb( urb );


  return writesize;

error_unanchor:
  usb_unanchor_urb( urb );
error:
  if ( urb ) {
    usb_free_coherent( dev->udev, writesize, buf, urb->transfer_dma );
    usb_free_urb( urb );
  }
  up( &dev->limit_sem );

exit:
  return retval;
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

  res = usb_control_msg (	dev->udev,
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
  interface = dev->interface;
  err=-1;

  
  switch(cmd){
    case RTLSDR_CONTROL:{
      //dev_info( &interface->dev, "rtlsdr-%d %s(control)", interface->minor, __FUNCTION__);
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
  .write =	rtlsdr_write,
  .open =		rtlsdr_open,
  .release =	rtlsdr_release,
  .flush =	rtlsdr_flush,
  .llseek =	noop_llseek,
  .unlocked_ioctl = rtlsdr_ioctl,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
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
  sema_init( &dev->limit_sem, WRITES_IN_FLIGHT );
  mutex_init( &dev->io_mutex );
  spin_lock_init( &dev->err_lock );
  init_usb_anchor( &dev->submitted );
  init_waitqueue_head( &dev->bulk_in_wait );

  dev->udev = usb_get_dev( interface_to_usbdev( interface ) );
  dev->interface = interface;

  /* set up the endpoint information */
  /* Find rtlsdr endpoints */
  iface_desc = interface->cur_altsetting;


  for ( i = 0; i < iface_desc->desc.bNumEndpoints; ++i ) {
    endpoint = &iface_desc->endpoint[i].desc;

    dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;

    buffer_size = 65536;//64Mb buffer
    dev->bulk_in_size = buffer_size;
    dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
    dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
    
        if (!dev->bulk_in_buffer) {
                dev_err(&interface->dev,
                "Could not allocate bulk_in_buffer\n");
                goto error;
        }
        dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
        if (!dev->bulk_in_urb) {
                dev_err(&interface->dev,
                "Could not allocate bulk_in_urb\n");
                goto error;
        }

    dev_info( &interface->dev, "Endpoint addr: 0x%x\n", endpoint->bEndpointAddress );
    dev_info( &interface->dev, "Endpoint buffer size: %u\n", (uint64_t)buffer_size );
  }

  /* save our data pointer in this interface device */
  usb_set_intfdata( interface, dev );

  /* Wrong interface we only want interface with bulk ep */
  if( iface_desc->desc.bNumEndpoints == 0 ) {
    retval = -1; //not really an error
    usb_set_intfdata( interface, NULL );
    goto error;
  }

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

  /* prevent more I/O from starting */
  mutex_lock( &dev->io_mutex );
  dev->interface = NULL;
  mutex_unlock( &dev->io_mutex );

  usb_kill_anchored_urbs( &dev->submitted );

  /* decrement our usage count */
  kref_put( &dev->kref, rtlsdr_delete );

  dev_info( &interface->dev, "USB rtlsdr #%d now disconnected", minor );
}

static void rtlsdr_draw_down( struct usb_rtlsdr* dev )
{
  int time;

  time = usb_wait_anchor_empty_timeout( &dev->submitted, 1000 );
  if ( !time ) {
    usb_kill_anchored_urbs( &dev->submitted );
  }
  usb_kill_urb( dev->bulk_in_urb );
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
