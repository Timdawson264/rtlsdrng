#ifndef __RTLSDR_H
#define __RTLSDR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>

struct rtlsdr_ctrltransfer {
	__u8 bRequestType;
	__u8 bRequest;
	__u16 wValue;
	__u16 wIndex;
	__u16 wLength;
	__u32 timeout;  /* in milliseconds */
 	void *data;
};

#define RTLSDR_CONTROL    _IOWR('R', 0, struct rtlsdr_ctrltransfer)
#define RTLSDR_RESET    _IO('R', 1)

#ifdef __cplusplus
}
#endif

#endif /* __RTLSDR_H */
