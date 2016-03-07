#ifndef _CYUSB_DRIVER_H
#define _CYUSB_DRIVER_H

#include "CyAPI.h"
#define USB_VID    0x04B4
#define USB_PID    0x1004
#include "ErrorNum.h"

#define false 0
#define true  1

#define CTRL_EP_BUF_SIZE 4096
#define USB_TIME_OUT 3000
#define BULK_EP_BUF_SIZE 0x100000

class CUsbDevice
{
 public:
 	CUsbDevice();
	~CUsbDevice();

	int Open();
	int GetVersion();
	int AttachDevCnt();
	long BulkEndPtIn(long &Len, BYTE * Buf);
	long BulkEndPtOut(long Len, BYTE * Buf);
	void Close();

	CCyUSBDevice *m_USBDev;
	int DevCount;
	
	int usb_status;   //0 colsed 1 opened
	
 private:
	
};








#endif