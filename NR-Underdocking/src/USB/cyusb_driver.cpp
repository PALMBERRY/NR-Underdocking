

#include "USB/cyusb_driver.h"


CUsbDevice::CUsbDevice()
{
	GUID guid = CYUSBDRV_GUID;
	
	m_USBDev = new CCyUSBDevice(NULL,guid);   // Create an instance of CCyUSBDevice
}

CUsbDevice::~CUsbDevice()
{
	m_USBDev->Close();
	delete m_USBDev;
}

int CUsbDevice::GetVersion()
{
	return 0;//VER_INFO;
}

int CUsbDevice::AttachDevCnt()
{
	return m_USBDev->DeviceCount();	
}

int CUsbDevice::Open()
{
	int nDevCount;

	m_USBDev->Close();

	nDevCount = m_USBDev->DeviceCount();
	DevCount=nDevCount;
	for (int i = 0; i < nDevCount; i++)
	{
		m_USBDev->Open(i);
		if((m_USBDev->VendorID != USB_VID) || (m_USBDev->ProductID != USB_PID))
		{
			m_USBDev->Close();
			usb_status = 0;
			continue;
		}
		else
		{
			break;
		}
	}

	if(m_USBDev->IsOpen()) 
	{
		//initial end point timeout
		m_USBDev->ControlEndPt->SetXferSize(CTRL_EP_BUF_SIZE);
		m_USBDev->ControlEndPt->TimeOut = USB_TIME_OUT;
		m_USBDev->BulkInEndPt->SetXferSize(BULK_EP_BUF_SIZE);
		m_USBDev->BulkInEndPt->TimeOut = USB_TIME_OUT;
		m_USBDev->BulkOutEndPt->SetXferSize(BULK_EP_BUF_SIZE);
		m_USBDev->BulkOutEndPt->TimeOut = USB_TIME_OUT;
		usb_status = 1;
		return E_NO_ERR;
	}
    usb_status = 0;
	return  E_NO_USB_DEV;
}


long CUsbDevice::BulkEndPtIn(long &Len, BYTE * Buf)
{
	CCyBulkEndPoint *ept;
	LONG RecLen = 0;
	LONG XferLen;
	LONG XferLenBak;

	ept = m_USBDev->BulkInEndPt;
	if (m_USBDev->IsOpen() == false) 
	{
		Len = 0;
		return E_USB_IS_CLOSED;
	}

	//the length want to receive is great than 0
	if (Len == 0) 
		return E_PARAMETER_ERROR;

	while(1)
	{
		if(RecLen >= Len) 
		{
			Len = RecLen;
			return E_RECEIVE_SUCCES;
		}
		else if ( (Len - RecLen) < BULK_EP_BUF_SIZE ) 
		{
			XferLen = Len - RecLen;
		}
		else
		{
			XferLen = BULK_EP_BUF_SIZE;
		}

		XferLenBak = XferLen;
		if (ept->XferData((Buf+RecLen),XferLen) == false)
		{
			Len = RecLen;
			return E_USB_IO_ERR;
		}
		else if(XferLen != XferLenBak)
		{
			/* the device end the packet */
			RecLen = RecLen + XferLen;
			Len = RecLen;
			return E_RECEIVE_SUCCES;
		}

		RecLen = RecLen + XferLen;
	}
}

long CUsbDevice::BulkEndPtOut(long Len, BYTE * Buf)
{
	CCyBulkEndPoint *ept;
	LONG RecLen = 0;
	LONG XferLen;

	ept = m_USBDev->BulkOutEndPt;
	if (m_USBDev->IsOpen() == false) 
		  return E_USB_IS_CLOSED;

	//the length want to receive is great than 0
	if (Len == 0) 
		return E_PARAMETER_ERROR;

	while(1)
	{
		if(RecLen >= Len) 
			return E_SEND_SUCCES;
		else if ( (Len - RecLen) < BULK_EP_BUF_SIZE ) 
			XferLen = Len - RecLen;
		else
			XferLen = BULK_EP_BUF_SIZE;

		if (ept->XferData((Buf+RecLen),XferLen) == false)
		{
			return E_USB_IO_ERR;
		}

		RecLen = RecLen + XferLen;
	}
}

void CUsbDevice::Close()
{
	m_USBDev->Close();
}
